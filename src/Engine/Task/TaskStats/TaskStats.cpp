/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2011 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
 */
#include "TaskStats.hpp"
#include <assert.h>

TaskStats::TaskStats()
{
  reset();
}

void
TaskStats::reset()
{
  Time = fixed_zero;
  glide_required = fixed_zero;
  cruise_efficiency = fixed_one;
  effective_mc = fixed_zero;
  mc_best = fixed_zero;
  distance_nominal = fixed_zero;
  distance_max = fixed_zero;
  distance_min = fixed_zero;
  distance_scored = fixed_zero;
  task_valid = false;
  task_started = false;
  task_finished = false;
  has_targets = false;
  flight_mode_final_glide = false;
  flight_mode_height_margin = 120;
}

bool 
TaskStats::calc_flight_mode()
{
  const int margin = (flight_mode_final_glide? 1:0)*flight_mode_height_margin;

  const bool this_is_final = 
    positive(total.solution_remaining.AltitudeDifference + fixed(margin));

  const bool changed = (flight_mode_final_glide != this_is_final);

  if (changed) {
    flight_mode_final_glide = this_is_final;
  }

  return changed;
}

void
TaskStatsComputer::reset()
{
  data.reset();
  total.reset();
  current_leg.reset();
}
