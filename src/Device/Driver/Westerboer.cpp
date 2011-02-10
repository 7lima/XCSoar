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

#include "Device/Driver/Westerboer.hpp"
#include "Device/Parser.hpp"
#include "Device/Driver.hpp"
#include "Units.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Protection.hpp"

#include <tchar.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * Device driver for Westerboer VW1150.
 *
 * Unfortunately, Westerboer refuses to publish technical
 * documentation.  I was able to obtain a PDF document describing the
 * NMEA sentences implemented in this driver, but Peter Maciejewski
 * from Westerboer asked me to remove the PDF from my web site, for
 * copyright violation.  -- Max Kellermann
 *
 */
class WesterboerDevice : public AbstractDevice {
public:
  virtual bool ParseNMEA(const char *line, struct NMEA_INFO *info,
                         bool enable_baro);
};

/**
 * $PWES0,DD,VVVV,MMMM,NNNN,BBBB,SSSS,AAAAA,QQQQQ,IIII,TTTT,UUU,CCC*CS<CR><LF>
 */
static bool
PWES0(NMEAInputLine &line, NMEA_INFO &info, bool enable_baro)
{
  int i, k;

  line.skip(); /* device */

  if (line.read_checked(i))
    info.ProvideTotalEnergyVario(fixed(i) / 10);

  line.skip(); /* average vario */

  if (line.read_checked(i))
    info.ProvideNettoVario(fixed(i) / 10);

  line.skip(); /* average netto vario */
  line.skip(); /* speed to fly */

  line.skip(); /* baro altitude 1013 */

  if (line.read_checked(i) && enable_baro)
    info.ProvideBaroAltitudeTrue(NMEA_INFO::BARO_ALTITUDE_WESTERBOER,
                                 fixed(i));

  bool have_ias = line.read_checked(i);
  bool have_tas = line.read_checked(k);
  if (have_ias && have_tas)
    info.ProvideBothAirspeeds(Units::ToSysUnit(fixed(i) / 10,
                                               unKiloMeterPerHour),
                              Units::ToSysUnit(fixed(k) / 10,
                                               unKiloMeterPerHour));

  if (line.read_checked(i))
    info.SupplyBatteryVoltage = fixed(i) / 10;

  if (line.read_checked(i)) {
    info.OutsideAirTemperature = fixed(i) / 10;
    info.TemperatureAvailable = true;
  }

  TriggerVarioUpdate();

  return true;
}

bool
WesterboerDevice::ParseNMEA(const char *String, NMEA_INFO *GPS_INFO,
                            bool enable_baro)
{
  if (!NMEAParser::NMEAChecksum(String))
    return false;

  NMEAInputLine line(String);
  char type[16];
  line.read(type, 16);

  if (strcmp(type, "$PWES0") == 0)
    return PWES0(line, *GPS_INFO, enable_baro);

  return false;
}

static Device *
WesterboerCreateOnPort(Port *com_port)
{
  return new WesterboerDevice();
}

const struct DeviceRegister westerboer_device_driver = {
  _T("Westerboer VW1150"),
  drfBaroAlt,
  WesterboerCreateOnPort,
};
