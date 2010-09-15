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

#ifndef TASKINTERFACE_H
#define TASKINTERFACE_H

#include "Compiler.h"

struct AIRCRAFT_STATE;
struct GeoPoint;
struct GeoVector;

class TaskStats;
class TaskPoint;

/**
 *  Abstract interface for all tasks and task manager.  This defines
 *  the functionality all tasks should have, and also provides some convenience
 *  methods that will be common to all tasks.
 */
class TaskInterface {
public:
  enum type {
    ORDERED,
    ABORT,
    GOTO,
  };

  const enum type type;

  TaskInterface(const enum type _type):type(_type) {}

/** 
 * Size of task
 * 
 * @return Number of taskpoints in task
 */
  gcc_pure
  virtual unsigned task_size() const = 0;

/** 
 * Set index in sequence of active task point.  Concrete classes providing
 * this method should ensure the index is valid.
 * 
 * @param new_index Desired sequence index of active task point
 */
  virtual void setActiveTaskPoint(unsigned new_index) = 0;

/** 
 * Accessor for active task point.  Typically could be used
 * to access information about the task point for user feedback.
 * 
 * @return Active task point
 */  
  gcc_pure
  virtual TaskPoint* getActiveTaskPoint() const = 0;

/**
 * Determine whether active task point optionally shifted points to
 * a valid task point.
 *
 * @param index_offset offset (default 0)
 */
  gcc_pure
  virtual bool validTaskPoint(const int index_offset=0) const = 0;

/** 
 * Accessor for task statistics for this task
 * 
 * @return Task statistics reference
 */
  gcc_pure
  virtual const TaskStats& get_stats() const = 0;

/** 
 * Update internal states as flight progresses.  This may perform
 * callbacks to the task_events, and advance the active task point
 * based on task_behaviour.
 * 
 * @param state_now Aircraft state at this time step
 * @param state_last Aircraft state at previous time step
 * 
 * @return True if internal state changed
 */
  virtual bool update(const AIRCRAFT_STATE& state_now, 
                      const AIRCRAFT_STATE& state_last) = 0;

/** 
 * Update internal states (non-essential) for housework, or where functions are slow
 * and would cause loss to real-time performance.
 * 
 * @param state_now Aircraft state at this time step
 * 
 * @return True if internal state changed
 */
  virtual bool update_idle(const AIRCRAFT_STATE &state_now) = 0;

#ifdef DO_PRINT
  virtual void print(const AIRCRAFT_STATE &state) = 0;
#endif

};
#endif //TASKINTERFACE_H
