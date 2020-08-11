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
#include "Geo/SpeedVector.hpp"
#include "Geo/GeoPoint.hpp"

template <class... Args>
bool PortPrintNMEA(Port & p, const char * format, Args&&... args);

class ZenDevice : public AbstractDevice {
private:
	Port &port;
	PolarCoefficients m_polar;
	GeoPoint m_next_waypoint;
	SpeedVector m_wind_vector;
	AGeoPoint m_last_position;
	double m_thermal_ceiling;
	double m_thermal_floor;	

public:

  ZenDevice(Port &_port):port(_port), m_polar(), m_next_waypoint(), m_wind_vector(), m_last_position(), m_thermal_ceiling(), m_thermal_floor() {}
  
  /* virtual methods from class Device */
  bool ParseNMEA(const char *line, NMEAInfo &info) override;

  void OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated) override;
};

bool
ZenDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  return false;
}

template <class... Args>
bool
PortPrintNMEA(Port & p, const char * format, Args&&... args)
{
	NullOperationEnvironment env;
	char buffer[74];
	sprintf(buffer, format, std::forward<Args>(args)...);
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePXCSG(Port & p, const AGeoPoint & geo)
{
	return PortPrintNMEA(p, "PXCSG,%f,%f,%.1f", geo.latitude.Degrees(), geo.longitude.Degrees(), geo.altitude);
}

inline bool
WriteGPMWV(Port & p, const SpeedVector & wind)
{
	return PortPrintNMEA(p, "GPMWV,%.1f,T,%.1f,A", wind.bearing.Degrees(), wind.norm);
}

inline bool
WritePXCSP(Port & p, const PolarCoefficients & polar)
{
	return PortPrintNMEA(p, "PXCSP,%f,%f,%f", polar.a, polar.b, polar.c);
}

inline bool
WritePXCSW(Port & p, const GeoPoint & wp)
{
	return PortPrintNMEA(p, "PXCSW,%f,%f", wp.latitude.Degrees(), wp.longitude.Degrees());
}

inline bool
WritePXCST(Port & p, double floor, double ceiling)
{
	return PortPrintNMEA(p, "PXCST,%.0f,%.0f", floor, ceiling);
}

void
ZenDevice::OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated)
{
  if(!basic.location_available || !basic.gps_altitude_available)
	  return;

  /* Update Polar */
  const auto polar = calculated.glide_polar_safety.GetCoefficients();
  if(polar.a != m_polar.a || polar.b != m_polar.b || polar.c != m_polar.c) {
	  m_polar = polar;
	  WritePXCSP(port, m_polar);
  }

  /* Update Next Waypoint */
  const auto wp = protected_task_manager->GetActiveWaypoint();
  if(wp != nullptr && m_next_waypoint != wp->location) {
	  m_next_waypoint = wp->location;
	  WritePXCSW(port, m_next_waypoint);
  }

  /* Update Wind Vector */
  const auto wind_vector = calculated.GetWindOrZero();
  if(wind_vector.bearing != m_wind_vector.bearing || wind_vector.norm != m_wind_vector.norm) {
	  m_wind_vector = wind_vector;
	  WriteGPMWV(port, m_wind_vector);
  }

  /* Update our own position */
  const AGeoPoint pos(basic.location, basic.gps_altitude );
  if(pos.latitude != m_last_position.latitude || pos.longitude != m_last_position.longitude || pos.altitude != m_last_position.altitude) {
	m_last_position = pos;
	WritePXCSG(port, pos);
  }

  const auto prev_thermals = calculated.thermal_encounter_collection;
  const auto cur_thermal = calculated.thermal_encounter_band;
  ThermalEncounterCollection all_thermals = prev_thermals;
  all_thermals.Merge(cur_thermal);
  if (all_thermals.Valid() && (m_thermal_ceiling != all_thermals.GetCeiling() || m_thermal_floor != all_thermals.GetFloor())) {
	  m_thermal_floor = all_thermals.GetFloor();
	  m_thermal_ceiling = all_thermals.GetCeiling();
	  WritePXCST(port, m_thermal_floor, m_thermal_ceiling);
  }
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
