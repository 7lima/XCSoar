#include "CommonStats.hpp"
#include "Waypoint/Waypoint.hpp"

CommonStats::CommonStats():
  vector_home(fixed_zero, fixed_zero)
{
  reset();
}

void
CommonStats::reset_task()
{
  landable_reachable = false;
  task_started = false;
  task_finished = false;
  aat_time_remaining = fixed_zero;
  aat_speed_remaining = -fixed_one;
  aat_speed_max = -fixed_one;
  aat_speed_min = -fixed_one;
  task_time_remaining = fixed_zero;
  task_time_elapsed = fixed_zero;
  mode_abort = false;
  mode_goto = false;
  mode_ordered = false;
  ordered_valid = false;
  ordered_has_targets = false;

  active_has_next = false;
  active_has_previous = false; 
  next_is_last = false;
  previous_is_first = false;
}

void
CommonStats::reset()
{
  V_block = fixed_zero;
  V_dolphin = fixed_zero;

  current_mc = fixed_zero;
  current_risk_mc = fixed_zero;
  current_bugs = fixed_one;
  current_ballast = fixed_zero;

  distance_olc = fixed_zero;
  time_olc = fixed_zero;
  speed_olc = fixed_zero;

  reset_task();
}
