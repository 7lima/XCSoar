#include "harness_flight.hpp"
#include "harness_airspace.hpp"
#include "harness_waypoints.hpp"
#include "TaskEventsPrint.hpp"
#ifdef DO_PRINT
#include <fstream>
#endif

extern Airspaces *airspaces;

extern double time_elapsed;
extern double time_planned;
extern double time_remaining;
extern double calc_cruise_efficiency;
extern double calc_effective_mc;


bool test_speed_factor(int test_num, int n_wind) 
{
  // flying at opt speed should be minimum time flight!

  double te0, te1, te2;

  test_flight(test_num, n_wind, 1.0);
  te0 = time_elapsed;

  test_flight(test_num, n_wind, 0.8);
  te1 = time_elapsed;
  // time of this should be higher than nominal
  ok(te0<te1, test_name("vopt slow or",test_num, n_wind), 0);

  test_flight(test_num, n_wind, 1.2);
  te2 = time_elapsed;
  // time of this should be higher than nominal
  ok(te0<te2, test_name("vopt fast or",test_num, n_wind), 0);

  bool retval = (te0<te1) && (te0<te2);
  if (verbose || !retval) {
    printf("# sf 0.8 time_elapsed_rat %g\n",te1/te0);
    printf("# sf 1.2 time_elapsed_rat %g\n",te2/te0);
  }
  return retval;
}


bool test_cruise_efficiency(int test_num, int n_wind) 
{
  // tests functionality of cruise efficiency calculations

  double ce0, ce1, ce2, ce3, ce4, ce5, ce6;

  bearing_noise = 0.0;
  target_noise = 0.1;

  test_flight(test_num, n_wind);
  ce0 = calc_cruise_efficiency;

  // wandering
  bearing_noise = 40.0;
  test_flight(test_num, n_wind);
  ce1 = calc_cruise_efficiency;
  // cruise efficiency of this should be lower than nominal
  ok (ce0>ce1, test_name("ce wandering",test_num, n_wind),0);
  if (ce0<=ce1 || verbose) {
    printf("# calc cruise efficiency %g\n", calc_cruise_efficiency);
  }

  // flying too slow
  bearing_noise = 0.0;
  test_flight(test_num, n_wind, 0.8);
  ce2 = calc_cruise_efficiency;
  // cruise efficiency of this should be lower than nominal
  ok (ce0>ce2, test_name("ce speed slow",test_num, n_wind),0);
  if (ce0<=ce2 || verbose) {
    printf("# calc cruise efficiency %g\n", calc_cruise_efficiency);
  }

  // flying too fast
  bearing_noise = 0.0;
  test_flight(test_num, n_wind, 1.2);
  ce3 = calc_cruise_efficiency;
  // cruise efficiency of this should be lower than nominal
  ok (ce0>ce3, test_name("ce speed fast",test_num, n_wind),0);
  if (ce0<=ce3 || verbose) {
    printf("# calc cruise efficiency %g\n", calc_cruise_efficiency);
  }

  // higher than expected cruise sink
  sink_factor = 1.2;
  test_flight(test_num, n_wind);
  ce4 = calc_cruise_efficiency;
  ok (ce0>ce4, test_name("ce high sink",test_num, n_wind),0);
  // cruise efficiency of this should be lower than nominal
  sink_factor = 1.0;
  if (ce0<=ce4 || verbose) {
    printf("# calc cruise efficiency %g\n", calc_cruise_efficiency);
  }

  // slower than expected climb
  climb_factor = 0.8;
  test_flight(test_num, n_wind);
  ce5 = calc_cruise_efficiency;
  ok (ce0>ce5, test_name("ce slow climb",test_num, n_wind),0);
  // cruise efficiency of this should be lower than nominal
  climb_factor = 1.0;
  if (ce0<=ce5 || verbose) {
    printf("# calc cruise efficiency %g\n", calc_cruise_efficiency);
  }

  // lower than expected cruise sink; 
  sink_factor = 0.8;
  test_flight(test_num, n_wind);
  ce6 = calc_cruise_efficiency;
  ok (ce0<ce6, test_name("ce low sink",test_num, n_wind),0);
  // cruise efficiency of this should be greater than nominal
  sink_factor = 1.0;
  if (ce0>=ce6 || verbose) {
    printf("# calc cruise efficiency %g\n", calc_cruise_efficiency);
  }

  bool retval = (ce0>ce1) && (ce0>ce2) && (ce0>ce3) && (ce0>ce4) && (ce0>ce5)
    && (ce0<ce6);
  if (verbose || !retval) {
    printf("# ce nominal %g\n",ce0);
    printf("# ce wandering %g\n",ce1);
    printf("# ce speed slow %g\n",ce2);
    printf("# ce speed fast %g\n",ce3);
    printf("# ce high sink %g\n",ce4);
    printf("# ce slow climb %g\n",ce5);
    printf("# ce low sink %g\n",ce6);
  }
  return retval;
}




bool test_aat(int test_num, int n_wind) 
{

  // test whether flying to targets in an AAT task produces
  // elapsed (finish) times equal to desired time with 1.5% tolerance

  bool fine = test_flight(test_num, n_wind);
  double min_time = aat_min_time(test_num);

  const double t_ratio = fabs(time_elapsed/min_time-1.0);
  fine &= (t_ratio<0.015);
  if (!fine || verbose) {
    printf("# time ratio error (elapsed/target) %g\n", t_ratio);
  }
  return fine;
}


bool test_automc(int test_num, int n_wind) 
{
  target_noise = 0.1;

  // test whether flying by automc (starting above final glide)
  // arrives home faster than without

  test_flight(test_num, n_wind, 1.0, false);
  double t0 = time_elapsed;

  test_flight(test_num, n_wind, 1.0, true);
  double t1 = time_elapsed;

  bool fine = (t1/t0<1.015);
  ok(fine,test_name("faster with auto mc on",test_num, n_wind),0);
  
  if (!fine || verbose) {
    printf("# time ratio %g\n", t1/t0);
  }
  return fine;
}

bool test_bestcruisetrack(int test_num, int n_wind)
{

  // tests whether following the cruise track which compensates for wind drift
  // produces routes that are more on track than if the route is allowed to drift
  // downwind during climbs

  // this test allows for a small error margin

  enable_bestcruisetrack = false;
  test_flight(test_num, n_wind);
  double t0 = time_elapsed;

  enable_bestcruisetrack = true;
  test_flight(test_num, n_wind);
  double t1 = time_elapsed;
  enable_bestcruisetrack = false;

  bool fine = (t1/t0<1.01);
  ok(fine,test_name("faster flying with bestcruisetrack",test_num, n_wind),0);
  
  if (!fine || verbose) {
    printf("# time ratio %g\n", t1/t0);
  }
  return fine;
}


bool test_abort(int n_wind)
{
  GlidePolar glide_polar(fixed_two);
  Waypoints waypoints;
  setup_waypoints(waypoints);

  if (verbose) {
    distance_counts();
  }

  TaskBehaviour task_behaviour;
//  task_behaviour.auto_mc = auto_mc;

  task_behaviour.all_off();

  TaskEventsPrint default_events(verbose);

  TaskManager task_manager(default_events,
                           task_behaviour,
                           waypoints);

  task_manager.set_glide_polar(glide_polar);

  test_task(task_manager, waypoints, 2);

  task_manager.abort();
  task_report(task_manager, "abort");

  return run_flight(task_manager, true, target_noise, n_wind);

}

bool test_goto(int n_wind, unsigned id, bool auto_mc)
{
  GlidePolar glide_polar(fixed_two);
  Waypoints waypoints;
  setup_waypoints(waypoints);

  if (verbose) {
    distance_counts();
  }

  TaskBehaviour task_behaviour;

  task_behaviour.all_off();
  task_behaviour.auto_mc = auto_mc;

  TaskEventsPrint default_events(verbose);

  TaskManager task_manager(default_events,
                           task_behaviour,
                           waypoints);

  task_manager.set_glide_polar(glide_polar);

  test_task(task_manager, waypoints, 2);

  task_manager.do_goto(*waypoints.lookup_id(id));
  task_report(task_manager, "goto");

  return run_flight(task_manager, true, target_noise, n_wind);
}


bool test_null()
{
  GlidePolar glide_polar(fixed_two);
  Waypoints waypoints;
  setup_waypoints(waypoints);

  if (verbose) {
    distance_counts();
  }

  TaskBehaviour task_behaviour;
//  task_behaviour.auto_mc = auto_mc;

  task_behaviour.all_off();

  TaskEventsPrint default_events(verbose);

  TaskManager task_manager(default_events,
                           task_behaviour,
                           waypoints);

  task_manager.set_glide_polar(glide_polar);

  task_report(task_manager, "null");

  return run_flight(task_manager, true, target_noise, 0);
}


bool test_airspace(const unsigned n_airspaces)
{
  airspaces = new Airspaces;
  setup_airspaces(*airspaces, n_airspaces);
  bool fine = test_flight(4,0);
  delete airspaces; airspaces = NULL;
  return fine;
}



bool test_effective_mc(int test_num, int n_wind) 
{
  // tests functionality of effective mc calculations

  double ce0, ce1, ce2, ce3, ce4, ce5, ce6;

  bearing_noise = 0.0;
  target_noise = 0.1;

  test_flight(test_num, n_wind);
  ce0 = calc_effective_mc;

  // wandering
  bearing_noise = 40.0;
  test_flight(test_num, n_wind);
  ce1 = calc_effective_mc;
  // effective mc of this should be lower than nominal
  ok (ce0>ce1, test_name("emc wandering",test_num, n_wind),0);
  if (ce0<=ce1 || verbose) {
    printf("# calc effective mc %g\n", calc_effective_mc);
  }

  // flying too slow
  bearing_noise = 0.0;
  test_flight(test_num, n_wind, 0.8);
  ce2 = calc_effective_mc;
  // effective mc of this should be lower than nominal
  ok (ce0>ce2, test_name("emc speed slow",test_num, n_wind),0);
  if (ce0<=ce2 || verbose) {
    printf("# calc effective mc %g\n", calc_effective_mc);
  }

  // flying too fast
  bearing_noise = 0.0;
  test_flight(test_num, n_wind, 1.2);
  ce3 = calc_effective_mc;
  // effective mc of this should be lower than nominal
  ok (ce0>ce3, test_name("emc speed fast",test_num, n_wind),0);
  if (ce0<=ce3 || verbose) {
    printf("# calc effective mc %g\n", calc_effective_mc);
  }

  // higher than expected cruise sink
  sink_factor = 1.2;
  test_flight(test_num, n_wind);
  ce4 = calc_effective_mc;
  ok (ce0>ce4, test_name("emc high sink",test_num, n_wind),0);
  // effective mc of this should be lower than nominal
  sink_factor = 1.0;
  if (ce0<=ce4 || verbose) {
    printf("# calc effective mc %g\n", calc_effective_mc);
  }

  // slower than expected climb
  climb_factor = 0.8;
  test_flight(test_num, n_wind);
  ce5 = calc_effective_mc;
  ok (ce0>ce5, test_name("emc slow climb",test_num, n_wind),0);
  // effective mc of this should be lower than nominal
  climb_factor = 1.0;
  if (ce0<=ce5 || verbose) {
    printf("# calc effective mc %g\n", calc_effective_mc);
  }

  // lower than expected cruise sink; 
  sink_factor = 0.8;
  test_flight(test_num, n_wind);
  ce6 = calc_effective_mc;
  ok (ce0<ce6, test_name("emc low sink",test_num, n_wind),0);
  // effective mc of this should be greater than nominal
  sink_factor = 1.0;
  if (ce0>=ce6 || verbose) {
    printf("# calc effective mc %g\n", calc_effective_mc);
  }

  bool retval = (ce0>ce1) && (ce0>ce2) && (ce0>ce3) && (ce0>ce4) && (ce0>ce5)
    && (ce0<ce6);
  if (verbose || !retval) {
    printf("# emc nominal %g\n",ce0);
    printf("# emc wandering %g\n",ce1);
    printf("# emc speed slow %g\n",ce2);
    printf("# emc speed fast %g\n",ce3);
    printf("# emc high sink %g\n",ce4);
    printf("# emc slow climb %g\n",ce5);
    printf("# emc low sink %g\n",ce6);
  }
  return retval;
}
