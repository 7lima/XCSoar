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

#include "ObservationZoneClient.hpp"
#include "ObservationZonePoint.hpp"
#include "Task/Visitors/ObservationZoneVisitor.hpp"
#include "Task/Tasks/BaseTask/TaskPoint.hpp"

ObservationZoneClient::~ObservationZoneClient() {
  delete m_oz;
}

bool
ObservationZoneClient::isInSector(const AIRCRAFT_STATE &ref) const
{
  return m_oz->isInSector(ref);
}

GeoPoint
ObservationZoneClient::randomPointInSector(const fixed mag) const
{
  return m_oz->randomPointInSector(mag);
}

fixed
ObservationZoneClient::score_adjustment() const
{
  return m_oz->score_adjustment();
}

GeoPoint
ObservationZoneClient::get_boundary_parametric(fixed t) const
{
  return m_oz->get_boundary_parametric(t);
}

bool
ObservationZoneClient::transition_constraint(const AIRCRAFT_STATE & ref_now,
                                             const AIRCRAFT_STATE & ref_last) const
{
  return m_oz->transition_constraint(ref_now, ref_last);
}

void 
ObservationZoneClient::set_legs(const TaskPoint *previous,
                                const TaskPoint *current,
                                const TaskPoint *next)
{
  m_oz->set_legs(previous != NULL ? &previous->get_location() : NULL,
                 current != NULL ? &current->get_location() : NULL,
                 next != NULL ? &next->get_location() : NULL);
}
