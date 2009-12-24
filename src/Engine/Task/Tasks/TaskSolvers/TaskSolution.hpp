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
#ifndef TASK_SOLUTION_HPP
#define TASK_SOLUTION_HPP

#include "GlideSolvers/GlideState.hpp"
#include "GlideSolvers/GlideResult.hpp"

class GlidePolar;

#include "Task/Tasks/BaseTask/TaskPoint.hpp"

/**
 * Utility class for calculating glide solutions for individual points and whole tasks
 * This is used to de-couple the task system from glide calculations
 */
class TaskSolution
{
public:
/** 
 * Compute optimal glide solution from aircraft to destination.
 * 
 * @param taskpoint The taskpoint representing the destination
 * @param state Aircraft state at origin
 * @param polar Glide polar used for computations
 * @param minH Minimum height at destination over-ride (max of this or the task points's elevation is used)
 * @return GlideResult of task leg
 */
  static GlideResult glide_solution_remaining(const TaskPoint& taskpoint,
                                              const AIRCRAFT_STATE &state, 
                                              const GlidePolar &polar,
                                              const fixed minH=0);

/** 
 * Compute optimal glide solution from aircraft to destination, with
 * externally supplied sink rate.  This is used to calculate the sink
 * rate required for glide-only solutions.
 * 
 * @param taskpoint The taskpoint representing the destination
 * @param state Aircraft state at origin
 * @param polar Glide polar used for computations
 * @param S Sink rate (m/s, positive down)
 * @return GlideResult of task leg
 */
  static GlideResult glide_solution_sink(const TaskPoint& taskpoint,
                                         const AIRCRAFT_STATE &state, 
                                         const GlidePolar &polar,
                                         const fixed S);

/** 
 * Compute optimal glide solution from previous point to aircraft towards destination.
 * (For pure TaskPoints, this is null)
 * 
 * @param taskpoint The taskpoint representing the destination
 * @param state Aircraft state
 * @param polar Glide polar used for computations
 * @param minH Minimum height at destination over-ride (max of this or the task points's elevation is used)
 * @return GlideResult of task leg
 */
  static GlideResult glide_solution_travelled(const TaskPoint& taskpoint,
                                              const AIRCRAFT_STATE &state, 
                                              const GlidePolar &polar,
                                              const fixed minH=0);

/** 
 * Compute optimal glide solution from aircraft to destination, or modified
 * destination (e.g. where specialised TaskPoint has a target)
 * 
 * @param taskpoint The taskpoint representing the destination
 * @param state Aircraft state at origin
 * @param polar Glide polar used for computations
 * @param minH Minimum height at destination over-ride (max of this or the task points's elevation is used)
 * @return GlideResult of task leg
 */
  static GlideResult glide_solution_planned(const TaskPoint& taskpoint,
                                            const AIRCRAFT_STATE &state, 
                                            const GlidePolar &polar,
                                            const fixed minH=0);
};

#endif
