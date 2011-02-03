/*
Copyright_License {

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

#include "Device/Driver/Zander.hpp"
#include "Device/Parser.hpp"
#include "Device/Driver.hpp"
#include "Protection.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"

#include <stdlib.h>

class ZanderDevice : public AbstractDevice {
public:
  virtual bool ParseNMEA(const char *line, struct NMEA_INFO *info,
                         bool enable_baro);
};

static bool
PZAN1(NMEAInputLine &line, NMEA_INFO *GPS_INFO, bool enable_baro)
{
  if (!enable_baro)
    return true;

  fixed baro_altitude;
  if (line.read_checked(baro_altitude))
    /* the ZS1 documentation does not specify wheter the altitude is
       STD or QNH, but Franz Poeschl confirmed via email that it is
       the QNH altitude */
    GPS_INFO->ProvideBaroAltitudeTrue(NMEA_INFO::BARO_ALTITUDE_ZANDER,
                                      baro_altitude);

  return true;
}

static bool
PZAN2(NMEAInputLine &line, NMEA_INFO *GPS_INFO)
{
  fixed vtas, wnet;

  if (line.read_checked(vtas)) {
    vtas /= fixed(3.6); // km/h -> m/s

    GPS_INFO->TrueAirspeed = vtas;
    GPS_INFO->IndicatedAirspeed = GPS_INFO->BaroAltitudeAvailable
      ? vtas / AtmosphericPressure::AirDensityRatio(GPS_INFO->BaroAltitude)
      : fixed_zero;
    GPS_INFO->AirspeedAvailable = true;
  }

  if (line.read_checked(wnet)) {
    GPS_INFO->TotalEnergyVario = (wnet - fixed(10000)) / 100;
    GPS_INFO->TotalEnergyVarioAvailable = true;
  }

  TriggerVarioUpdate();

  return true;
}

bool
ZanderDevice::ParseNMEA(const char *String, NMEA_INFO *GPS_INFO,
                        bool enable_baro)
{
  NMEAInputLine line(String);
  char type[16];
  line.read(type, 16);

  if (strcmp(type, "$PZAN1") == 0)
    return PZAN1(line, GPS_INFO, enable_baro);

  if (strcmp(type, "$PZAN2") == 0)
    return PZAN2(line, GPS_INFO);

  return false;
}

static Device *
ZanderCreateOnPort(Port *com_port)
{
  return new ZanderDevice();
}

const struct DeviceRegister zanderDevice = {
  _T("Zander"),
  drfGPS | drfBaroAlt | drfSpeed | drfVario,
  ZanderCreateOnPort,
};
