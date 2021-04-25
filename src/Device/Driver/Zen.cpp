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
#include "NMEA/Validity.hpp"
#include "Units/System.hpp"
#include "Engine/GlideSolvers/PolarCoefficients.hpp"
#include "Operation/Operation.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Components.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Geo/SpeedVector.hpp"
#include "Geo/GeoPoint.hpp"
#include "FLARM/List.hpp"
#include "Plane/Plane.hpp"
#include "Interface.hpp"
  
constexpr static std::size_t max_sentence_length = 256;

class ZenDevice : public AbstractDevice {
private:
	Port &port;
	PolarCoefficients m_polar;
	GeoPoint m_next_waypoint;
	SpeedVector m_wind_vector;
	AGeoPoint m_last_position;
	double m_thermal_ceiling;
	double m_thermal_floor;	
	Plane m_plane;
	Angle m_last_track;
	double m_last_time;
  Validity m_last_update;

  bool PZENR(NMEAInputLine & line, NMEAInfo & info);

public:

  ZenDevice(Port &_port):port(_port), m_polar(0, 0, 0), m_next_waypoint(Angle::Zero(), Angle::Zero()), m_wind_vector(Angle::Zero(), 0), m_last_position(GeoPoint(Angle::Zero(), Angle::Zero()), 0), m_thermal_ceiling(0), m_thermal_floor(0), m_plane(), m_last_track(Angle::Zero()), m_last_time(0) {}
  
  /* virtual methods from class Device */
  bool ParseNMEA(const char *line, NMEAInfo &info) override;

  void OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated) override;
};

inline bool
ZenDevice::PZENR(NMEAInputLine & line, NMEAInfo & info)
{
  /*
   * $PZENR,entry,total_entries,dtime,lat,lon,alt,stdev,maccready,*hh
   * Field number:
   * 1) Index of the current entry in the route
   * 2) Time cost from current position in seconds
   * 3) Approximate latitude of position in degrees
   * 4) Approximate longitude of position in degrees
   * 5) Mean altitude at position in meters MSL
   * 6) Uncertainty of position
   * 7) Optimal maccready setting at position
   */
   unsigned int route_index = 0;
   if(!line.ReadChecked(route_index))
     return false;

   double cost = 0;
   if(!line.ReadChecked(cost))
     return false;

   double lat = 0;
   if(!line.ReadChecked(lat))
     return false;

   double lon = 0;
   if(!line.ReadChecked(lon))
     return false;

   double alt = 0;
   if(!line.ReadChecked(alt))
     return false;

   double stdev = 0;
   if(!line.ReadChecked(stdev))
     return false;

   double mc = 0;
   if(!line.ReadChecked(mc))
     return false;

   if(route_index >= info.route.max_size())
     return false;

   {
     info.route.resize(route_index+1);
     const GeoPoint gps(Angle::Degrees(lon), Angle::Degrees(lat));
     info.route[route_index] = AGeoPoint(gps, alt);
   }
   return true;
}

bool
ZenDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  NMEAInputLine line(_line);
  m_last_update.Update(info.clock);

  char type[16];
  line.Read(type, 16);

  if (StringIsEqual(type + 1, "PZENR"))
    return PZENR(line, info);

  return false;
}

inline bool
WriteGPMWV(Port & p, const SpeedVector & wind)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	if(snprintf(buffer, max_sentence_length, "GPMWV,%.1f,T,%.1f,A", wind.bearing.Degrees(), wind.norm) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePXCSP(Port & p, const PolarCoefficients & polar)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	if(snprintf(buffer, max_sentence_length, "PXCSP,%f,%f,%f", polar.a, polar.b, polar.c) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePXCSR(Port & p, const Plane & plane)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	if(snprintf(buffer, max_sentence_length, "PXCSR,%s", plane.registration.c_str()) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePXCSW(Port & p, const GeoPoint & wp)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	if(snprintf(buffer, max_sentence_length, "PXCSW,%f,%f", wp.latitude.Degrees(), wp.longitude.Degrees()) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePXCST(Port & p, double floor, double ceiling)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	if(snprintf(buffer, max_sentence_length, "PXCST,%.0f,%.0f", floor, ceiling) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePOGNB(Port & p, const FlarmTraffic & plane)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	char flarm_id[8];
	plane.id.Format(flarm_id);
	if(snprintf(buffer, max_sentence_length, "POGNB,%u,%u,%s,%f,%f,%.1f,%.1f,%.1f,%.1f,%.1f",
			(unsigned)plane.type,
			(unsigned)plane.id_type,
			flarm_id,
			plane.location.latitude.Degrees(),
			plane.location.longitude.Degrees(),
			(double)plane.altitude,
			(double)plane.speed,
			((Angle)plane.track).Degrees(),
			(double)plane.turn_rate,
			(double)plane.climb_rate) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}

inline bool
WritePXCSG(Port & p, const AGeoPoint & geo, double ground_speed, const Angle & track, double turn_rate, double noncomp_vario)
{
	NullOperationEnvironment env;
	char buffer[max_sentence_length];
	if(snprintf(buffer, max_sentence_length, "PXCSG,%f,%f,%.1f,%.1f,%.1f,%.1f,%.1f", geo.latitude.Degrees(), geo.longitude.Degrees(), geo.altitude, ground_speed, track.Degrees(), turn_rate, noncomp_vario) == max_sentence_length) return false;
	return PortWriteNMEA(p, buffer, env);
}


void
ZenDevice::OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated)
{
  if(!basic.location_available || !basic.gps_altitude_available)
	  return;

  if(m_last_update.IsValid() &&m_last_update.IsOlderThan(basic.clock, 1))
    return;

  /* Update Registration and Polar */
  const auto polar = calculated.glide_polar_safety.GetCoefficients();
  if(polar.IsValid() && (polar.a != m_polar.a || polar.b != m_polar.b || polar.c != m_polar.c)) {
	  m_polar = polar;
	  WritePXCSP(port, m_polar);
  }

  /* Update registration */
  const auto plane = CommonInterface::GetComputerSettings().plane;
  if(m_plane.registration != plane.registration) {
	  m_plane = plane;
	  WritePXCSR(port, m_plane);
  }


  /* Update Next Waypoint */
  if(protected_task_manager != nullptr) {
    const auto wp = protected_task_manager->GetActiveWaypoint();
    if(wp != nullptr && m_next_waypoint != wp->location) {
	    m_next_waypoint = wp->location;
	    WritePXCSW(port, m_next_waypoint);
    }
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
	const auto dtime = basic.time - m_last_time;
	const auto dtrack = basic.track - m_last_track;
	auto turn_rate = 0.0;
	if(basic.track_available && dtime > 0) {
		turn_rate = dtrack.Degrees()/dtime;
		m_last_track = basic.track;
		m_last_time = basic.time;
	}
	WritePXCSG(port, pos, basic.ground_speed, basic.track, turn_rate, basic.gps_vario);
  }

  /* Update thermal band, also during thermalling */
  const auto prev_thermals = calculated.thermal_encounter_collection;
  const auto cur_thermal = calculated.thermal_encounter_band;
  ThermalEncounterCollection all_thermals = prev_thermals;
  all_thermals.Merge(cur_thermal);
  if (all_thermals.Valid() && (m_thermal_ceiling != all_thermals.GetCeiling() || m_thermal_floor != all_thermals.GetFloor())) {
	  m_thermal_floor = all_thermals.GetFloor();
	  m_thermal_ceiling = all_thermals.GetCeiling();
	  WritePXCST(port, m_thermal_floor, m_thermal_ceiling);
  }

  /* Update FLARM traffic, if it was updated less than a second ago */
  for(const auto & plane : basic.flarm.traffic.list) {
	  if(!plane.stealth) {
		  WritePOGNB(port, plane);
	  }
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
