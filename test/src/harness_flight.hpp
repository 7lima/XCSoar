#ifndef HARNESS_FLIGHT_HPP
#define HARNESS_FLIGHT_HPP

#include "test_debug.hpp"
#include "harness_aircraft.hpp"
#include "harness_airspace.hpp"
#include "harness_waypoints.hpp"
#include "harness_task.hpp"

bool run_flight(TaskManager &task_manager,
                bool goto_target,
                double random_mag,
                int n_wind,
                const double speed_factor=1.0);

bool test_flight(int test_num, int n_wind, const double speed_factor=1.0,
                 const bool auto_mc=false);

#define NUM_WIND 9

const char* wind_name(int n_wind);

bool test_flight_times(int test_num, int n_wind);
bool test_aat(int test_num, int n_wind);
bool test_speed_factor(int test_num, int n_wind);
bool test_cruise_efficiency(int test_num, int n_wind);
bool test_effective_mc(int test_num, int n_wind);
bool test_automc(int test_num, int n_wind);
bool test_bestcruisetrack(int test_num, int n_wind);
bool test_abort(int n_wind);
bool test_goto(int n_wind, unsigned id, bool auto_mc=false);
bool test_null();
bool test_airspace(const unsigned n_airspaces);
bool test_olc(int n_wind, OLCRules id);

fixed
aat_min_time(int test_num);

#endif
