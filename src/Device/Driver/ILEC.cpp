/*
Copyright_License {

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

#include "Device/Driver/ILEC.hpp"
#include "Device/Parser.hpp"
#include "Device/Driver.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Units.hpp"
#include "Protection.hpp"

#include <stdlib.h>
#include <math.h>

class ILECDevice : public AbstractDevice {
private:
  Port *port;

public:
  ILECDevice(Port *_port):port(_port) {}

public:
  virtual bool ParseNMEA(const char *line, struct NMEA_INFO *info,
                         bool enable_baro);
};

static bool
ReadSpeedVector(NMEAInputLine &line, SpeedVector &value_r)
{
  fixed norm, bearing;

  bool bearing_valid = line.read_checked(bearing);
  bool norm_valid = line.read_checked(norm);

  if (bearing_valid && norm_valid) {
    value_r.norm = Units::ToSysUnit(norm, unKiloMeterPerHour);
    value_r.bearing = Angle::degrees(bearing);
    return true;
  } else
    return false;
}

/**
 * Parse a "$PILC,PDA1" sentence.
 *
 * Example: "$PILC,PDA1,1489,-3.21,274,15,58*7D"
 */
static bool
ParsePDA1(NMEAInputLine &line, NMEA_INFO &info, bool enable_baro)
{
  // altitude [m]
  int altitude;
  if (line.read_checked(altitude) && enable_baro)
    info.ProvideBaroAltitudeTrue(NMEA_INFO::BARO_ALTITUDE_ILEC,
                                 fixed(altitude));

  // total energy vario [m/s]
  info.TotalEnergyVarioAvailable = line.read_checked(info.TotalEnergyVario);

  // wind direction [degrees, kph]
  info.ExternalWindAvailable = ReadSpeedVector(line, info.ExternalWind);

  // confidence [0..100]
  // not used

  TriggerVarioUpdate();

  return true;
}

bool
ILECDevice::ParseNMEA(const char *_line, NMEA_INFO *info, bool enable_baro)
{
  NMEAInputLine line(_line);
  char type[16];
  line.read(type, sizeof(type));

  if (strcmp(type, "$PILC") == 0) {
    line.read(type, sizeof(type));
    if (strcmp(type, "PDA1") == 0)
      return ParsePDA1(line, *info, enable_baro);
    else
      return false;
  } else
    return false;
}

static Device *
ILECCreateOnPort(Port *com_port)
{
  return new ILECDevice(com_port);
}

const struct DeviceRegister ilec_device_driver = {
  _T("ILEC SN10"),
  drfGPS | drfBaroAlt | drfVario,
  ILECCreateOnPort,
};
