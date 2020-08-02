/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
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

#include "Device/Driver/Zen.hpp"

#include <algorithm> // std::min and std::max

#include "Device/Driver.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "NMEA/InputLine.hpp"
#include "Units/System.hpp"
#include "Engine/GlideSolvers/PolarCoefficients.hpp"
#include "Operation/Operation.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Components.hpp"
#include "Engine/Waypoint/Waypoint.hpp"

class ZenDevice : public AbstractDevice {
	Port &port;

public:

  ZenDevice(Port &_port):port(_port) {}
  
  /* virtual methods from class Device */
  bool ParseNMEA(const char *line, NMEAInfo &info) override;

  void OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated) override;
};

bool
ZenDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  return false;
}

void
ZenDevice::OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated)
{
  if(!basic.location_available || !basic.gps_altitude_available)
	  return;

  const auto polar = calculated.glide_polar_safety.GetCoefficients();
  const auto wp = protected_task_manager->GetActiveWaypoint();

  NullOperationEnvironment env;
  const auto lat = basic.location.latitude.Degrees();
  const auto lon = basic.location.longitude.Degrees();
  double alt = basic.gps_altitude;
  double nxt_lat = wp->location.latitude.Degrees();
  double nxt_lon = wp->location.longitude.Degrees();

  const auto wind_vector = calculated.GetWindOrZero();
  const auto wind_speed = wind_vector.bearing.Degrees();
  const auto wind_dir = wind_vector.norm; 

  const auto prev_thermals_active = calculated.thermal_encounter_collection.Valid();
  const auto prev_thermals_floor = prev_thermals_active ? calculated.thermal_encounter_collection.GetFloor() : 10000.0;
  const auto prev_thermals_ceiling = prev_thermals_active ? calculated.thermal_encounter_collection.GetCeiling() : 0.0;
  
  const auto cur_thermal_active = calculated.thermal_encounter_band.Valid();
  const auto cur_thermal_floor = cur_thermal_active ? calculated.thermal_encounter_band.GetFloor() : 10000.0;;
  const auto cur_thermal_ceiling = cur_thermal_active ? calculated.thermal_encounter_band.GetCeiling() : 0.0;

  const auto thermal_floor = std::min(cur_thermal_floor, prev_thermals_floor);
  const auto thermal_ceiling = std::max(cur_thermal_ceiling, prev_thermals_ceiling);

  char buffer[100];
  sprintf(buffer, "PZENF,%f,%f,%.2f,%f,%f,%f,%f,%f,%.1f,%.1f,%.0f,%.0f",lat, lon, alt, polar.a, polar.b, polar.c, nxt_lat, nxt_lon, wind_speed, wind_dir, thermal_floor, thermal_ceiling);
  PortWriteNMEA(port, buffer, env);
// Get Polar
// Get current lat/lon/alt
// Get next waypoint lat/lon
// Get current airspeed, vario, netto vario
// Write out $PZENF
}

static Device *
ZenCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new ZenDevice(com_port);
}

const struct DeviceRegister zen_driver = {
  _T("Zen Pathfinder"),
  _T("Zen Pathfinder"),
  DeviceRegister::SEND_SETTINGS,
  ZenCreateOnPort,
};
