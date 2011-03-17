/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
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

#include "RoutePlannerGlue.hpp"
#include "Thread/Guard.hpp"
#include "Terrain/RasterTerrain.hpp"
#include "Navigation/SpeedVector.hpp"
#include "NMEA/Derived.hpp"
#include <assert.h>

RoutePlannerGlue::RoutePlannerGlue(const GlidePolar& polar,
                                   const Airspaces& master):
  terrain(NULL),
  m_planner(polar, SpeedVector(Angle::degrees(fixed_zero), fixed_zero), master)
{
}

void
RoutePlannerGlue::set_terrain(RasterTerrain* _terrain)
{
  terrain = _terrain;
  if (terrain) {
    RasterTerrain::ExclusiveLease lease(*terrain);
    m_planner.set_terrain(&terrain->map);
  } else {
    m_planner.set_terrain(NULL);
  }
}

bool
RoutePlannerGlue::solve(const AGeoPoint& origin,
                        const AGeoPoint& destination,
                        const RoutePlannerConfig& config,
                        const short h_ceiling)
{
  RasterTerrain::ExclusiveLease lease(*terrain);
  return m_planner.solve(origin, destination, config, h_ceiling);
}

void
RoutePlannerGlue::solve_reach(const AGeoPoint& origin)
{
  if (!terrain)
    return;
  RasterTerrain::ExclusiveLease lease(*terrain);
  m_planner.solve_reach(origin);
}

bool
RoutePlannerGlue::find_positive_arrival(const AGeoPoint& dest,
                                        short& arrival_height) const
{
  return m_planner.find_positive_arrival(dest, arrival_height);
}

void
RoutePlannerGlue::accept_in_range(const GeoBounds& bounds,
                                  TriangleFanVisitor& visitor) const
{
  m_planner.accept_in_range(bounds, visitor);
}

bool
RoutePlannerGlue::intersection(const AGeoPoint& origin,
                               const AGeoPoint& destination,
                               GeoPoint& intx) const
{
  RasterTerrain::ExclusiveLease lease(*terrain);
  return m_planner.intersection(origin, destination, intx);
}
