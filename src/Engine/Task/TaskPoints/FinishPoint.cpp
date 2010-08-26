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

#include "FinishPoint.hpp"
#include <stdlib.h>
#include <assert.h>

FinishPoint::FinishPoint(ObservationZonePoint* _oz,
                         const TaskProjection& tp,
                         const Waypoint & wp,
                         const TaskBehaviour& tb,
                         const OrderedTaskBehaviour& to) : 
  OrderedTaskPoint(FINISH, _oz, tp, wp, tb, to),
  fai_finish_height(fixed_zero)
{ 
}

void 
FinishPoint::reset()
{
  OrderedTaskPoint::reset();
  fai_finish_height = fixed_zero;
}

bool 
FinishPoint::entry_precondition() const
{
  return get_previous()->has_entered();
}

fixed
FinishPoint::get_elevation() const
{
  const fixed nominal_elevation = m_elevation
    +m_task_behaviour.safety_height_arrival;

  if (m_ordered_task_behaviour.fai_finish) {
    return max(nominal_elevation, fai_finish_height);
  } else {
    return nominal_elevation;
  }
}


void 
FinishPoint::set_neighbours(OrderedTaskPoint* _prev,
                           OrderedTaskPoint* _next)
{
  assert(_next==NULL);
  // should not ever have an outbound leg
  OrderedTaskPoint::set_neighbours(_prev, _next);
}

void 
FinishPoint::set_fai_finish_height(const fixed height)
{
  fai_finish_height = max(fixed_zero, height);
}

bool 
FinishPoint::isInSector(const AIRCRAFT_STATE &state) const
{
  if (!ObservationZoneClient::isInSector(state)) 
    return false;

  return is_in_height_limit(state);
}

bool
FinishPoint::is_in_height_limit(const AIRCRAFT_STATE &state) const
{
  if (!m_ordered_task_behaviour.check_finish_height(state, m_elevation))
    return false;

  if (m_ordered_task_behaviour.fai_finish) {
    return state.NavAltitude > fai_finish_height;
  }
  return true;
}


bool 
FinishPoint::check_transition_enter(const AIRCRAFT_STATE & ref_now, 
                                    const AIRCRAFT_STATE & ref_last) const
{
  const bool now_in_height = is_in_height_limit(ref_now);
  const bool last_in_height = is_in_height_limit(ref_last);

  if (now_in_height && last_in_height) {
    // both within height limit, so use normal location checks
    return ObservationZone::check_transition_enter(ref_now, ref_last);
  }
  return false;
}
