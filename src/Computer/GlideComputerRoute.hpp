/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#ifndef XCSOAR_GLIDE_COMPUTER_ROUTE_HPP
#define XCSOAR_GLIDE_COMPUTER_ROUTE_HPP

#include "Task/ProtectedRoutePlanner.hpp"
#include "Engine/Route/RoutePlanner.hpp"
#include "GPSClock.hpp"

struct MoreData;
struct DerivedInfo;
struct GlideSettings;
struct RoutePlannerConfig;
class ProtectedRoutePlanner;
class RoutePlannerGlue;
class RasterTerrain;
class GlidePolar;

class GlideComputerRoute {
  RoutePlannerGlue route_planner;
  ProtectedRoutePlanner protected_route_planner;

  GPSClock route_clock;
  GPSClock reach_clock;

  const RasterTerrain *terrain;

public:
  GlideComputerRoute(const Airspaces &airspace_database);

  /**
   * Returns a reference to the unprotected route planner object,
   * which must not be used outside of the calculation thread.
   */
  const RoutePlannerGlue &GetRoutePlanner() const {
    return route_planner;
  }

  const ProtectedRoutePlanner &GetProtectedRoutePlanner() const {
    return protected_route_planner;
  }

  /**
   * Release all references to airspace objects from the "master"
   * container.  Call this before modifying the container.
   */
  void ClearAirspaces() {
    route_planner.reset();
  }

  void ResetFlight();
  void ProcessRoute(const MoreData &basic, DerivedInfo &calculated,
                    const DerivedInfo &last_calculated,
                    const GlideSettings &settings,
                    const RoutePlannerConfig &config,
                    const GlidePolar &glide_polar,
                    const GlidePolar &safety_polar);

  void set_terrain(const RasterTerrain* _terrain);

private:
  void TerrainWarning(const MoreData &basic,
                      DerivedInfo &calculated,
                      const DerivedInfo &last_calculated,
                      const RoutePlannerConfig &config);

  void Reach(const MoreData &basic, DerivedInfo &calculated,
             const RoutePlannerConfig &config);
};

#endif
