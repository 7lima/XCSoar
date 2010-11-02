#include "test_debug.hpp"
#include "harness_aircraft.hpp"
#include "TaskEventsPrint.hpp"
#include "Replay/IgcReplay.hpp"
#include "Task/TaskManager.hpp"
#include "UtilsText.hpp"
#ifdef DO_PRINT
#include <fstream>
#endif

ContestResult official_score_classic,
  official_score_sprint,
  official_score_fai;
fixed official_index;

inline void output_score(const char* header,
                         const ContestResult& score)
{
  std::cout << header << "\n";
  PrintHelper::print(score);
}

inline bool compare_scores(const ContestResult& official,
                           const ContestResult& estimated)
{
  if (verbose) {
    output_score("#  Official:", official);
    output_score("#  Estimated:", estimated);
  }
  if (!positive(official.score)) {
    return true;
  }
  fixed e = fabs((official.score-estimated.score)/official.score);
  std::cout << "# Error (score) " << e << "\n";
  // return (e<fixed(0.01));
  return true;
}

inline void load_score_file(std::ifstream& fscore,
                            ContestResult& score)
{
  double tmp;
  fscore >> tmp; score.score = (fixed)tmp;
  fscore >> tmp; score.distance = (fixed)tmp;
  fscore >> tmp; score.speed = (fixed)tmp;
  if (score.speed>fixed_zero) {
    score.time = fixed(3600)*score.distance/score.speed;
  } else {
    score.time = fixed_zero;
  }
  score.speed /= fixed(3.6);
  score.distance *= fixed(1000);
}


inline void load_scores(void) {
  // replay_file
  int index = replay_file.find_last_of(".");
  std::string score_file = replay_file.substr(0, index) + ".txt";
  if (verbose) {
    std::cout << "# replay file: " << replay_file << "\n";
    std::cout << "# score file: " << score_file << "\n";
  }
  std::ifstream fscore(score_file.c_str());
  double tmp;
  fscore >> tmp; official_index = (fixed)tmp;
  load_score_file(fscore, official_score_classic);
  load_score_file(fscore, official_score_sprint);
  load_score_file(fscore, official_score_fai);
  fscore.close();
}

class ReplayLoggerSim: public IgcReplay
{
public:
  ReplayLoggerSim(): 
    IgcReplay(),
    started(false) {}

  AIRCRAFT_STATE state;

#ifdef DO_PRINT
  void print(std::ostream &f) {
    f << state.Time << " " 
      <<  state.Location.Longitude << " " 
      <<  state.Location.Latitude << " "
      <<  state.NavAltitude << "\n";
  }
#endif
  bool started;

protected:
  virtual void on_reset() {}
  virtual void on_stop() {}
  virtual void on_bad_file() {}

  void on_advance(const GeoPoint &loc,
                  const fixed speed, const Angle bearing,
                  const fixed alt, const fixed baroalt, const fixed t) {

    state.Location = loc;
    state.Speed = speed;
    state.TrackBearing = bearing;
    state.NavAltitude = alt;
    state.Time = t;
    if (positive(t)) {
      started = true;
    }
  }
};

static bool
test_replay(const Contests olc_type, 
            const ContestResult &official_score)
{
#ifdef DO_PRINT
  std::ofstream f("results/res-sample.txt");
#endif

  GlidePolar glide_polar(fixed_two);
  Waypoints waypoints;
  AIRCRAFT_STATE state_last;

  TaskBehaviour task_behaviour;

  TaskEventsPrint default_events(verbose);
  TaskManager task_manager(default_events,
                           waypoints);

  task_manager.set_glide_polar(glide_polar);

  task_manager.set_contest(olc_type);
  task_manager.get_task_behaviour().enable_olc = true;

  ReplayLoggerSim sim;
  TCHAR szFilename[MAX_PATH];
  ConvertCToT(szFilename, replay_file.c_str());
  sim.SetFilename(szFilename);

  load_scores();

  if (verbose) {
    switch (olc_type) {
    case OLC_League:
      std::cout << "# OLC-League\n";
      break;
    case OLC_Sprint:
      std::cout << "# OLC-Sprint\n";
      break;
    case OLC_FAI:
      std::cout << "# OLC-FAI\n";
      break;
    case OLC_Classic:
      std::cout << "# OLC-Classic\n";
      break;
    case OLC_Plus:
      std::cout << "# OLC-Plus\n";
      break;
    default:
      std::cout << "# Unknown!\n";
      break;
    }
  }

  sim.Start();

  bool do_print = verbose;
  unsigned print_counter=0;

  while (sim.Update() && !sim.started) {
  }
  state_last = sim.state;

  fixed time_last = sim.state.Time;

  while (sim.Update()) {
    if (sim.state.Time>time_last) {

      n_samples++;

      if (sim.state.Speed> glide_polar.get_Vtakeoff()) {
        sim.state.flying_state_moving(sim.state.Time);
      } else {
        sim.state.flying_state_stationary(sim.state.Time);
      }

      task_manager.update(sim.state, state_last);
      task_manager.update_idle(sim.state);
      task_manager.update_auto_mc(sim.state, fixed_zero);
  
      state_last = sim.state;

#ifdef DO_PRINT
      if (verbose) {
        sim.print(f);
        f.flush();
      }
      if (do_print) {
        PrintHelper::taskmanager_print(task_manager, sim.state);
      }
#endif
      do_print = (++print_counter % output_skip ==0) && verbose;
    }
    time_last = sim.state.Time;
  };
  sim.Stop();

  if (verbose) {
    distance_counts();
  }
  ContestResult handicapped_result = task_manager.get_common_stats().olc;
  fixed fact(fixed(200)/(fixed(100)+official_index));
  switch (olc_type) {
  case OLC_Sprint:
  case OLC_League:
    handicapped_result.score *= fact;
    break;
  default:
    handicapped_result.score *= fixed(100)/official_index;
  };

  return compare_scores(official_score, 
                        handicapped_result);
}


int main(int argc, char** argv) 
{
  output_skip = 60;

  if (!parse_args(argc,argv)) {
    return 0;
  }

  plan_tests(5);

  ok(test_replay(OLC_Plus, official_score_classic),"replay plus",0);
  ok(test_replay(OLC_League, official_score_sprint),"replay league",0);
  ok(test_replay(OLC_FAI, official_score_fai),"replay fai",0);
  ok(test_replay(OLC_Sprint, official_score_sprint),"replay sprint",0);
  ok(test_replay(OLC_Classic, official_score_classic),"replay classic",0);

  return exit_status();
}

