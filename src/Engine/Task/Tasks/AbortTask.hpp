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

#ifndef ABORTTASK_H
#define ABORTTASK_H

#include "UnorderedTask.hpp"
#include "BaseTask/UnorderedTaskPoint.hpp"
#include "Waypoint/Waypoints.hpp"
#include "GlideSolvers/GlidePolar.hpp"
#include "Navigation/SpeedVector.hpp"

#include <vector>
#include <assert.h>

class AbortIntersectionTest {
public:
  virtual bool intersects(const AGeoPoint& destination) = 0;
};

/**
 * Abort task provides automatic management of a sorted list of task points
 * that are reachable or close to reachable, and landable (with airfields preferred).
 *
 * @todo
 *  - should prefer landable points that are non-intersecting with terrain
 *
 * Sorting order is as follows:
 * - airfields reachable from final glide (sorted by arrival time) at safety mc
 * - landpoints reachable from final glide (sorted by arrival time) at safety mc
 * - airfields reachable with climb (sorted by arrival time including climb time) 
 *   at current mc
 * - landpoints reachable with climb (sorted by arrival time including climb time) 
 *   at current mc
 * 
 */
class AbortTask: public UnorderedTask
{
public:
  friend class PrintHelper;

  /** max number of items in list */
  static const unsigned max_abort;
  /** min search range in km */
  static const fixed min_search_range;
  /** max search range in km */
  static const fixed max_search_range;

  struct Alternate {
    Waypoint waypoint;

    GlideResult solution;

    Alternate(const Waypoint &_waypoint)
      :waypoint(_waypoint)
    {
      solution.Reset();
    }

    Alternate(const Waypoint &_waypoint, const GlideResult &_solution)
      :waypoint(_waypoint), solution(_solution) {}
  };

  /** Vector of waypoints and solutions used to store candidates */
  typedef std::vector <Alternate> AlternateVector;

protected:
  struct AlternateTaskPoint: public UnorderedTaskPoint
  {
    GlideResult solution;

    AlternateTaskPoint(const Waypoint &waypoint, const TaskBehaviour &tb,
                       const GlideResult &_solution)
      :UnorderedTaskPoint(waypoint, tb), solution(_solution) {}
  };

  typedef std::vector<AlternateTaskPoint> AlternateTaskVector;
  AlternateTaskVector task_points;

  /** whether the AbortTask is the master or running in background */
  bool is_active;

  const Waypoints &waypoints;

  /** Hook for external intersection tests */
  AbortIntersectionTest* intersection_test;

private:
  unsigned active_waypoint;
  GlidePolar polar_safety;
  bool m_landable_reachable;

public:
  /** 
   * Base constructor.
   * 
   * @param te Task events callback class (shared among all tasks) 
   * @param tb Global task behaviour settings
   * @param gp Global glide polar used for navigation calculations
   * @param wps Waypoints container to be scanned during updates
   * 
   * @return Initialised object (with nothing in task)
   */
  AbortTask(TaskEvents &te, const TaskBehaviour &tb, const GlidePolar &gp,
            const Waypoints &wps);
  virtual ~AbortTask();

  virtual void SetTaskBehaviour(const TaskBehaviour &tb);

  /**
   * Size of task
   *
   * @return Number of taskpoints in task
   */
  unsigned TaskSize() const;

  const UnorderedTaskPoint &GetAlternate(unsigned i) const {
    assert(i < task_points.size());

    return task_points[i];
  }

  /**
   * Retrieves the active task point sequence.
   *
   * @return Index of active task point sequence
   */
  TaskWaypoint* GetActiveTaskPoint() const;

  /**
   * Retrieves the active task point index.
   *
   * @return Index of active task point sequence
   */
  unsigned getActiveIndex() const {
    return activeTaskPoint;
  }

  /**
   * Set active task point index
   *
   * @param index Desired active index of task sequence
   */
  void SetActiveTaskPoint(unsigned index);

  /**
   * Determine whether active task point optionally shifted points to
   * a valid task point.
   *
   * @param index_offset offset (default 0)
   */
  bool IsValidTaskPoint(int index_offset = 0) const;

  /**
   * Update internal states when aircraft state advances.
   * This performs a scan of reachable waypoints.
   *
   * \todo
   * - check tracking of active waypoint
   *
   * @param state_now Aircraft state at this time step
   * @param full_update Force update due to task state change
   *
   * @return True if internal state changes
   */
  bool update_sample(const AircraftState &state_now, 
                     bool full_update);

  /**
   * Determine if any landable reachable waypoints were found in the
   * last update.
   *
   * @return True if a landable waypoint was found
   */
  bool has_landable_reachable() const {
    return m_landable_reachable;
  }

  virtual void reset();

  /**
   * Calculate vector to home waypoint
   *
   * @param state State of aircraft
   * @return Vector to home waypoint
   */
  GeoVector get_vector_home(const AircraftState &state) const;

  /**
   * Retrieve copy of safety glide polar used by task system
   *
   * @return Copy of safety glide polar
   */
  GlidePolar get_safety_polar() const;

  GeoPoint get_task_center(const GeoPoint& fallback_location) const;
  fixed get_task_radius(const GeoPoint& fallback_location) const;

protected:
  /**
   * Test whether (and how) transitioning into/out of task points should occur, typically
   * according to task_advance mechanism.  This also may call the task_event callbacks.
   *
   * @param state_now Aircraft state at this time step
   * @param state_last Aircraft state at previous time step
   *
   * @return True if transition occurred
   */
  bool check_transitions(const AircraftState& state_now, 
                         const AircraftState& state_last);

  /**
   * Clears task points in list
   *
   */
  virtual void clear();

  /**
   * Check whether abort task list is full
   *
   * @return True if no more task points can be added
   */
  gcc_pure
  bool task_full() const;

  /**
   * Calculate distance to search for landable waypoints for aircraft.
   *
   * @param state_now Aircraft state
   *
   * @return Distance (m) of approximate glide range of aircraft
   */
  gcc_pure
  fixed abort_range(const AircraftState &state_now) const;

  /**
   * Propagate changes to safety glide polar from global glide polar.
   *
   * @param wind Wind at aircraft
   */
  void update_polar(const SpeedVector& wind);

  /**
   * Fill abort task list with candidate waypoints given a list of
   * waypoints satisfying approximate range queries.  Can be used
   * to add airfields only, or landpoints.
   *
   * @param state Aircraft state
   * @param approx_waypoints List of candidate waypoints
   * @param polar Polar used for tests
   * @param only_airfield If true, only add waypoints that are airfields.
   * @param final_glide Whether solution must be glide only or climb allowed
   * @param safety Whether solution uses safety polar
   *
   * @return True if a landpoint within final glide was found
   */
  bool fill_reachable(const AircraftState &state,
                      AlternateVector &approx_waypoints,
                      const GlidePolar &polar, bool only_airfield,
                      bool final_glide, bool safety);

protected:
  /**
   * This is called by update_sample after the turnpoint list has 
   * been filled with landpoints.
   * It's first called after the reachable scan, then may be called again after scanning
   * for unreachable.
   */
  virtual void client_update(const AircraftState &state_now, bool reachable);

public:
  /**
   * Specify whether the task is active or not.  If it's active, it will
   * notify the user about the abort task point being changed via the events.
   *
   */
  void set_active(bool _active) {
    is_active = _active;
  }

  /**
   * Set external test function to be used for additional intersection tests
   */
  void set_intersection_test(AbortIntersectionTest* _test) {
    intersection_test = _test;
  }

  /**
   * Accept a const task point visitor; makes the visitor visit
   * all TaskPoint in the task
   *
   * @param visitor Visitor to accept
   * @param reverse Visit task points in reverse order
   *
   */
  void tp_CAccept(TaskPointConstVisitor& visitor, bool reverse = false) const;
};

#endif
