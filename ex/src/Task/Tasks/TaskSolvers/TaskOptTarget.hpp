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
#ifndef TASKOPTTARGET_HPP
#define TASKOPTTARGET_HPP

#include "TaskMacCreadyRemaining.hpp"
#include "Util/ZeroFinder.hpp"
#include "Task/TaskPoints/StartPoint.hpp"
#include "Task/TaskPoints/AATIsolineSegment.hpp"

/**
 * Adjust target lateral offset for active task point to minimise
 * elapsed time.
 *
 * \todo
 * - Merge with TaskMinTarget?
 * - Refactor, since passing in AATPoint is a hack
 */

class TaskOptTarget: 
  public ZeroFinder
{
public:
/** 
 * Constructor for ordered task points
 * 
 * @param tps Vector of ordered task points comprising the task
 * @param activeTaskPoint Current active task point in sequence
 * @param _aircraft Current aircraft state
 * @param _gp Glide polar to copy for calculations
 * @param _tp_current Active AATPoint
 * @param _ts StartPoint of task (to initiate scans)
 */
  TaskOptTarget(const std::vector<OrderedTaskPoint*>& tps,
                const unsigned activeTaskPoint,
                const AIRCRAFT_STATE &_aircraft,
                const GlidePolar &_gp,
                AATPoint& _tp_current,
                StartPoint *_ts);
  virtual ~TaskOptTarget() {};

  virtual fixed f(const fixed p);

/** 
 * Test validity of a solution given search parameter
 * 
 * @param p Search parameter (isoline parameter [0,1])
 * 
 * @return True if solution is valid
 */
  virtual bool valid(const fixed p);

/** 
 * Search for active task point's target isoline to minimise elapsed time
 * to finish.
 *
 * Running this adjusts the target values for the active task point. 
 * 
 * @param p Default isoline value (0-1)
 * 
 * @return Isoline value for solution
 */
  virtual fixed search(const fixed p);

private:
  void set_target(const fixed p); /**< Sets target location along isoline */
  TaskMacCreadyRemaining tm; /**< Object to calculate remaining task statistics */
  GlideResult res; /**< Glide solution used in search */
  const AIRCRAFT_STATE &aircraft; /**< Observer */
  StartPoint *tp_start; /**< Start of task */
  AATPoint &tp_current; /**< Active AATPoint */
  AATIsolineSegment iso; /**< Isoline for active AATPoint target */
};


#endif
