/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

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
#include "TaskManager.hpp"
#include "Visitors/TaskVisitor.hpp"
#include "Visitors/TaskPointVisitor.hpp"

// uses delegate pattern


TaskManager::TaskManager(TaskEvents &te,
                         const TaskBehaviour &tb,
                         const Waypoints &wps): 
  m_glide_polar(fixed_zero),
  trace_full(),
  trace_sprint(9000, 2, 300),
  task_ordered(te,tb,task_advance,m_glide_polar),
  task_goto(te,tb,m_glide_polar),
  task_abort(te,tb,m_glide_polar,wps),
  task_olc(te,tb,m_glide_polar,common_stats, trace_full, trace_sprint),
  task_behaviour(tb),
  mode(MODE_NULL),
  active_task(NULL)
{
}

TaskManager::~TaskManager()
{
}

TaskManager::TaskMode_t 
TaskManager::set_mode(const TaskMode_t the_mode)
{
  switch(the_mode) {
  case (MODE_ABORT):
    active_task = &task_abort;
    mode = MODE_ABORT;
    break;
  case (MODE_ORDERED):
    if (task_ordered.task_size()) {
      active_task = &task_ordered;
      mode = MODE_ORDERED;
      break;
    }
  case (MODE_GOTO):
    if (task_goto.getActiveTaskPoint()) {
      active_task = &task_goto;
      mode = MODE_GOTO;
      break;
    }
  case (MODE_NULL):
    active_task = NULL;
    mode = MODE_NULL;
    break;
  };
  return mode;
}

void 
TaskManager::setActiveTaskPoint(unsigned index)
{
  if (active_task) active_task->setActiveTaskPoint(index);
}

unsigned 
TaskManager::getActiveTaskPointIndex() const
{
  if (active_task) {
    return active_task->getActiveTaskPointIndex();
  } else {
    return 0;
  }
}

void 
TaskManager::incrementActiveTaskPoint(int offset)
{
  if (active_task) {
    unsigned i = getActiveTaskPointIndex();
    setActiveTaskPoint(i+offset);
  }
}


TaskPoint* 
TaskManager::getActiveTaskPoint() const
{
  if (active_task) 
    return active_task->getActiveTaskPoint();
  else 
    return NULL;
}


bool 
TaskManager::validTaskPoint(const int index_offset) const
{
  if (active_task) 
    return active_task->validTaskPoint(index_offset);
  else 
    return false;
}


/**
 * Convenience class to find waypoints in task unobtrusively
 *
 */
class WaypointLister: public TaskPointConstVisitor
{
public:
  /**
   * Constructor; clears waypoints in task in stats struct
   */
  WaypointLister(CommonStats& the_stats):stats(the_stats) 
    {
      stats.clear_waypoints_in_task();
    };

  void Visit(const UnorderedTaskPoint& tp) {
    stats.append_waypoint_in_task(tp.get_waypoint());
  }
  void Visit(const FinishPoint& tp) {
    stats.append_waypoint_in_task(tp.get_waypoint());
  }
  void Visit(const StartPoint& tp) {
    stats.append_waypoint_in_task(tp.get_waypoint());
  }
  void Visit(const AATPoint& tp) {
    stats.append_waypoint_in_task(tp.get_waypoint());
  }
  void Visit(const ASTPoint& tp) {
    stats.append_waypoint_in_task(tp.get_waypoint());
  }
private:
  CommonStats& stats;
};

void
TaskManager::update_common_stats_times(const AIRCRAFT_STATE &state)
{
  if (task_ordered.task_size()>1) {
    common_stats.task_started = task_ordered.get_stats().task_started;
    common_stats.task_finished = task_ordered.get_stats().task_finished;

    common_stats.aat_time_remaining = max(fixed_zero, 
                                          task_ordered.get_ordered_task_behaviour().aat_min_time
                                          -task_ordered.get_stats().total.TimeElapsed);

    if (positive(common_stats.aat_time_remaining)) {
      common_stats.aat_speed_remaining = task_ordered.get_stats().total.remaining.get_distance()/
        common_stats.aat_time_remaining;
    } else {
      common_stats.aat_speed_remaining = -fixed_one;
    }

    fixed aat_min_time = task_ordered.get_ordered_task_behaviour().aat_min_time;

    if (positive(aat_min_time)) {
      common_stats.aat_speed_max = task_ordered.get_stats().distance_max / aat_min_time;
      common_stats.aat_speed_min = task_ordered.get_stats().distance_min / aat_min_time;
    } else {
      common_stats.aat_speed_max = -fixed_one;
      common_stats.aat_speed_min = -fixed_one;
    }

    common_stats.task_time_remaining = task_ordered.get_stats().total.TimeRemaining;
    common_stats.task_time_elapsed = task_ordered.get_stats().total.TimeElapsed;

  } else {
    common_stats.reset_task();
  }
}

void
TaskManager::update_common_stats_waypoints(const AIRCRAFT_STATE &state)
{
  common_stats.vector_home = task_abort.get_vector_home(state);

  // if during this update, no landables found, try abort task

  if (active_task && (active_task != &task_abort)) {
    // update abort task offline
    task_abort.update_offline(state);
  }
  common_stats.landable_reachable |= task_abort.has_landable_reachable();

  WaypointLister lister(common_stats);
  if (common_stats.ordered_valid) {
    task_ordered.CAccept(lister);
  }
  if (active_task && (active_task != &task_ordered)) {
    active_task->CAccept(lister);
  }
}

void
TaskManager::update_common_stats_task(const AIRCRAFT_STATE &state)
{
  common_stats.mode_abort = (mode==MODE_ABORT);
  common_stats.mode_goto = (mode==MODE_GOTO);
  common_stats.mode_ordered = (mode==MODE_ORDERED);

  common_stats.ordered_valid = task_ordered.check_task();

  if (active_task && active_task->get_stats().task_valid) {
    common_stats.active_has_next = active_task->validTaskPoint(1);
    common_stats.active_has_previous = active_task->validTaskPoint(-1);
    common_stats.next_is_last = active_task->validTaskPoint(1) && !active_task->validTaskPoint(2);
    common_stats.previous_is_first = active_task->validTaskPoint(-1) && !active_task->validTaskPoint(-2);
  } else {
    common_stats.active_has_next = false;
    common_stats.active_has_previous = false;
    common_stats.next_is_last = false;
    common_stats.previous_is_first = false;
  }
}

void
TaskManager::update_common_stats_polar(const AIRCRAFT_STATE &state)
{

  common_stats.current_mc = m_glide_polar.get_mc();
  common_stats.current_bugs = m_glide_polar.get_bugs();
  common_stats.current_ballast = m_glide_polar.get_ballast();

  common_stats.current_risk_mc = 
    m_glide_polar.mc_risk(state.working_band_fraction, 
                          task_behaviour.risk_gamma);

  GlidePolar risk_polar = m_glide_polar;
  risk_polar.set_mc(common_stats.current_risk_mc);

  common_stats.V_block = 
    m_glide_polar.speed_to_fly(state,
                               get_stats().current_leg.solution_remaining,
                               true);

  // note right now we only use risk mc for dolphin speeds

  common_stats.V_dolphin = 
    risk_polar.speed_to_fly(state,
                            get_stats().current_leg.solution_remaining,
                            false);

}

void
TaskManager::update_common_stats(const AIRCRAFT_STATE &state)
{
  update_common_stats_times(state);
  update_common_stats_task(state);
  update_common_stats_waypoints(state);
  update_common_stats_polar(state);
}


bool 
TaskManager::update(const AIRCRAFT_STATE &state, 
                    const AIRCRAFT_STATE& state_last)
{
  // always update ordered task so even if we are temporarily
  // in abort/goto mode, the task stats are still updated

  bool retval = false;

  if (state_last.Time > state.Time) {
    reset();
  }

  if (state.Flying) {
    trace_full.append(state);
    trace_sprint.append(state);
  }

  if (task_ordered.task_size()>1) {
    // always update ordered task
    retval |= task_ordered.update(state, state_last);
  }

  if (active_task && (active_task != &task_ordered)) {
    // update mode task
    retval |= active_task->update(state, state_last);
  }

  if (state.Flying) {
    // always update OLC sampling task
    retval |= task_olc.update_sample(state);
  }

  update_common_stats(state);

  return retval;
}

bool 
TaskManager::update_idle(const AIRCRAFT_STATE& state)
{
  // always update OLC
  bool retval = false;

  if (state.Flying) {
    retval |= trace_full.optimise_if_old();
    retval |= trace_sprint.optimise_if_old();

    if (task_behaviour.enable_olc) {
      retval |= task_olc.update_idle(state);
    }
  }

  if (active_task) {
    retval |= active_task->update_idle(state);
  }

  return retval;
}


const TaskStats& 
TaskManager::get_stats() const
{
  if (active_task) {
    return active_task->get_stats();
  } else {
    return null_stats;
  }
}

const CommonStats& 
TaskManager::get_common_stats() const
{
  return common_stats;
}

void
TaskManager::abort()
{
  set_mode(MODE_ABORT);
}

void
TaskManager::resume()
{
  set_mode(MODE_ORDERED);
}

bool
TaskManager::do_goto(const Waypoint & wp)
{
  if (task_goto.do_goto(wp)) {
    set_mode(MODE_GOTO);
    return true;
  } else {
    return false;
  }
}

bool 
TaskManager::check_task() const
{
  if (active_task) 
    return active_task->check_task();
  else
    return false;  
}

bool 
TaskManager::check_ordered_task() const
{
  return task_ordered.check_task();
}

void
TaskManager::CAccept(BaseVisitor& visitor) const
{
  if (active_task) active_task->CAccept(visitor);
}

void
TaskManager::ordered_CAccept(BaseVisitor& visitor) const
{
  task_ordered.CAccept(visitor);
}

void
TaskManager::Accept(BaseVisitor& visitor) 
{
  if (active_task) active_task->Accept(visitor);
}

void
TaskManager::ordered_Accept(BaseVisitor& visitor) 
{
  task_ordered.Accept(visitor);
}

void
TaskManager::reset()
{
  task_ordered.reset();
  task_goto.reset();
  task_abort.reset();
  task_olc.reset();
  common_stats.reset();
  m_glide_polar.set_cruise_efficiency(fixed_one);
  trace_full.clear();
  trace_sprint.clear();
}


unsigned 
TaskManager::task_size() const
{
  if (active_task) {
    return active_task->task_size();
  } else {
    return 0;
  }
}

GEOPOINT 
TaskManager::random_point_in_task(const unsigned index, const fixed mag) const
{
  if (active_task == &task_ordered) {
    if (index< task_size()) {
      return task_ordered.getTaskPoint(index)->randomPointInSector(mag);
    }
  }
  if (index<= task_size()) {
    return active_task->getActiveTaskPoint()->get_location();
  } else {
    GEOPOINT null_location;
    return null_location;
  }
}




TaskManager::TaskMode_t 
TaskManager::get_mode() const 
{
  return mode;
}

bool 
TaskManager::is_mode(const TaskMode_t the_mode) const 
{
  return mode == the_mode;
}

GlidePolar 
TaskManager::get_glide_polar()  
{
  return m_glide_polar;
}

const GlidePolar& 
TaskManager::get_glide_polar() const 
{
  return m_glide_polar;
}

void 
TaskManager::set_glide_polar(const GlidePolar& glide_polar) 
{
  m_glide_polar = glide_polar;
}

GlidePolar 
TaskManager::get_safety_polar() const 
{
  return task_abort.get_safety_polar();
}


TaskAdvance& 
TaskManager::get_task_advance() 
{
  return task_advance;
}


bool 
TaskManager::stats_valid() const 
{
  return get_stats().task_valid;
}


void 
TaskManager::default_task(const GEOPOINT loc, const bool force)
{
  /// @todo implement default_task
}


const TracePointVector& 
TaskManager::get_olc_points() const
{
  return task_olc.get_olc_points();
}


const TracePointVector& 
TaskManager::get_trace_points() const
{
  return task_olc.get_trace_points();
}


TracePointVector 
TaskManager::find_trace_points(const GEOPOINT &loc, const fixed range,
                               const unsigned mintime, const fixed resolution) const
{
  return trace_full.find_within_range(loc, range, mintime, resolution);
}


AIRCRAFT_STATE 
TaskManager::get_start_state() const
{
  return task_ordered.get_start_state();
}


AIRCRAFT_STATE 
TaskManager::get_finish_state() const
{
  return task_ordered.get_finish_state();
}

fixed 
TaskManager::get_finish_height() const
{
  if (active_task) {
    return active_task->get_finish_height();
  } else {
    return fixed_zero;
  }
}

bool 
TaskManager::update_auto_mc(const AIRCRAFT_STATE& state_now,
                            const fixed fallback_mc)
{
  if (active_task) {
    if (active_task->update_auto_mc(state_now, fallback_mc)) {
      return true;
    }
  } 

  if (!task_behaviour.auto_mc) 
    return false;

  if (task_behaviour.auto_mc_mode==TaskBehaviour::AUTOMC_FINALGLIDE) 
    return false;

  if (positive(fallback_mc)) {
    m_glide_polar.set_mc(fallback_mc);
    return true;
  }
  return false;
}

GEOPOINT
TaskManager::get_task_center(const GEOPOINT& fallback_location) const
{
  if (active_task) {
    return active_task->get_task_center(fallback_location);
  } else {
    return fallback_location;
  }
}

fixed
TaskManager::get_task_radius(const GEOPOINT& fallback_location) const
{
  if (active_task) {
    return active_task->get_task_radius(fallback_location);
  } else {
    return fixed_zero;
  }
}


OrderedTask* 
TaskManager::clone(TaskEvents &te, 
                   const TaskBehaviour &tb,
                   TaskAdvance &ta,
                   GlidePolar &gp) const
{
  return task_ordered.clone(te, tb, ta, gp);
}

bool 
TaskManager::commit(const OrderedTask& other)
{
  bool retval;
  if ((mode== MODE_ORDERED) && !other.task_size()) {
    set_mode(MODE_NULL);
  } 
  retval = task_ordered.commit(other);

  if (mode== MODE_NULL) {
    setActiveTaskPoint(0);
    set_mode(MODE_ORDERED);
  }
  return retval;
}


//////////////////

AbstractTaskFactory& 
TaskManager::get_factory() const 
{
  return task_ordered.get_factory();
}


OrderedTask::Factory_t 
TaskManager::set_factory(const OrderedTask::Factory_t the_factory)
{
  return task_ordered.set_factory(the_factory);
}

const OrderedTaskBehaviour& 
TaskManager::get_ordered_task_behaviour() const
{
  return task_ordered.get_ordered_task_behaviour();
}
