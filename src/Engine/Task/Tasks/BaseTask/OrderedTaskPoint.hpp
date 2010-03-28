/*
  Copyright_License {

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

#ifndef ORDEREDTASKPOINT_HPP
#define ORDEREDTASKPOINT_HPP

#include "TaskLeg.hpp"
#include "ScoredTaskPoint.hpp"
#include "ObservationZoneClient.hpp"

/**
 *  Abstract compound specialisation of TaskLeg and ScoredTaskPoint,
 *  for task points which are organised in an ordered sequence.  This
 *  class manages the concept of an active task point, and therefore
 *  in a task, one OrderedTaskPoint will be marked as active, and the
 *  others marked either before or after active.
 *
 *  The OrderedTaskPoint tracks previous and next OrderedTaskPoints. 
 */
class OrderedTaskPoint : 
  public TaskLeg,
  public ScoredTaskPoint,
  public ObservationZoneClient
{
public:
/** 
 * Constructor.
 * Ownership of oz is transferred to this object
 * 
 * @param _oz Observation zone for this task point
 * @param tp Projection used for internal representations
 * @param wp Waypoint associated with this task point
 * @param tb Task Behaviour defining options (esp safety heights)
 * @param to OrderedTask Behaviour defining options 
 * @param b_scored Whether distance within OZ is scored 
 * 
 * @return Partially initialised object 
 */
  OrderedTaskPoint(ObservationZonePoint* _oz,
                   const TaskProjection& tp,
                   const Waypoint & wp, 
                   const TaskBehaviour &tb,
                   const OrderedTaskBehaviour& to,
                   const bool b_scored=false);

  virtual ~OrderedTaskPoint() {}

  /**
   * Create a clone of the task point. 
   * Caller is responsible for destruction.
   *
   * @param task_behaviour Task behaviour of clone
   * @param task_projection Task projection of clone
   * @param wp Waypoint to shift to (or NULL)
   */
  OrderedTaskPoint* clone(const TaskBehaviour &task_behaviour,
                          const OrderedTaskBehaviour &task_behaviour,
                          const TaskProjection &task_projection,
                          const Waypoint* waypoint=NULL) const;

  /** 
   * Call this when any geometry or OZ parameters are changed
   * 
   */
  void update_oz();

  /** 
   * Update observation zone geometry (or other internal data) when
   * previous/next turnpoint changes.
   */
  void update_geometry();

/** 
 * Set previous/next task points.
 * 
 * @param prev Previous (incoming leg's origin) task point
 * @param next Next (outgoing leg's destination) task point
 */
  virtual void set_neighbours(OrderedTaskPoint* prev,
                              OrderedTaskPoint* next);

/** 
 * Accessor for previous task point
 * 
 * @return Previous task point
 */
  OrderedTaskPoint* get_previous() const;

/** 
 * Accessor for next task point
 * 
 * @return Next task point
 */
  OrderedTaskPoint* get_next() const;

  /**
   * States each task point can be in (with respect to which OrderedTaskPoint is
   * active/selected).
   */
  enum ActiveState_t {
    NOTFOUND_ACTIVE = 0,        /**< Active task point was not found, ERROR! */
    BEFORE_ACTIVE,              /**< This taskpoint is before the active one */
    CURRENT_ACTIVE,             /**< This taskpoint is currently the active one */
    AFTER_ACTIVE                /**< This taskpoint is after the active one */
  };
  
/** 
 * Accessor for activation state of this task point.
 * This is valid only after scan_active() has been called. 
 * 
 * @return Activation state of this task point
 */
  ActiveState_t getActiveState() const {
    return m_active_state;
  }

/** 
 * Scan forward through successors to set the activity
 * state of all connected task points.  Should only be
 * called on the known first task point in the list.
 * 
 * @param atp The current active task point
 * 
 * @return True if the active task point is found
 */
  bool scan_active(OrderedTaskPoint* atp);

/** 
 * Calculate vector remaining from aircraft state.
 * If this task point is after active, uses the planned reference points
 * 
 * (This uses memento values)
 *
 * @param state Aircraft state
 * @return Vector remaining to this taskpoint (or next planned)
 */
  const GeoVector get_vector_remaining(const AIRCRAFT_STATE &state) const {
    return vector_remaining;
  }

/** 
 * Calculate vector from this task point to the aircraft.
 * If this task point is after active, returns null;
 * if this task point is before active, uses scored/best points.
 *
 * (This uses memento values)
 *
 * 
 * @return Vector from this taskpoint to aircraft (or next planned)
 */
  const GeoVector get_vector_travelled(const AIRCRAFT_STATE &) const {
    return vector_travelled;
  }

/** 
 * Calculate vector around planned task with this task point as the destination.
 * 
 * @return Vector planned to this taskpoint
 */
  const GeoVector get_vector_planned() const {
    return vector_planned;
  }

/** 
 * Test whether a taskpoint is equivalent to this one
 * 
 * For this abstract orderedtaskpoint, only compare OZ and WP
 *
 * @param other Taskpoint to compare to
 * 
 * @return True if same WP, type and OZ
 */
  virtual bool equals(const OrderedTaskPoint* other) const;

#ifdef DO_PRINT
  virtual void print(std::ostream& f, const AIRCRAFT_STATE& state,
                     const int item=0) const;
  virtual void print_boundary(std::ostream& f, const AIRCRAFT_STATE &state) const;

#endif

protected:

  const OrderedTaskBehaviour &m_ordered_task_behaviour; /**< Reference to ordered task behaviour (for task-specific options) */

/** 
 * Calculate distance from previous remaining/planned location to a point,
 * and from that point to the next remaining/planned location.
 * Use of this function facilitates speed-up over simply calculating
 * both distances and adding them.
 * 
 * @param ref Reference location
 * 
 * @return Distance (m)
 */
  fixed double_leg_distance(const GEOPOINT &ref) const;

private:

  bool search_nominal_if_unsampled() const;
  bool search_boundary_points() const;

  ActiveState_t m_active_state; /**< ActiveState determined from scan_active() */

  OrderedTaskPoint* tp_next;

  OrderedTaskPoint* tp_previous;

};


#endif
