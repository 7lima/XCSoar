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
#include "Device/Driver.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Units/System.hpp"

class ZenDevice : public AbstractDevice {
	Port &port;

public:

  ZenDevice(Port &_port):port(_port) {}
  
  /* virtual methods from class Device */
  bool ParseNMEA(const char *line, NMEAInfo &info) override;

  bool PutMacCready(double mc, OperationEnvironment &env) override;
  bool PutBallast(double fraction, double overload,
                  OperationEnvironment &env) override;
  bool PutBugs(double bugs, OperationEnvironment &env) override;
				  
  static bool POV(NMEAInputLine &line, NMEAInfo &info);
};

bool
ZenDevice::PutMacCready(double mc, OperationEnvironment &env)
{
  if (!EnableNMEA(env))
    return false;
  
  char buffer[30];
  sprintf(buffer,"POV,C,MC,%0.2f", (double)mc);
  return PortWriteNMEA(port, buffer, env);
}

bool
ZenDevice::PutBallast(double fraction, double overload, OperationEnvironment &env)
{
   
  if (!EnableNMEA(env))
    return false;
  
  char buffer[30];
  sprintf(buffer,"POV,C,WL,%3f", overload);
  return PortWriteNMEA(port, buffer, env);
}

bool
ZenDevice::PutBugs(double bugs, OperationEnvironment &env)
{
  //unsigned _bugs = uround(bugs);
  double _bugs = (double)(bugs);

  char buffer[32];
  sprintf(buffer, "$POV,C,BU,%0.2f\r", _bugs);
  return PortWriteNMEA(port, buffer, env);
}

bool
ZenDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  if (!VerifyNMEAChecksum(_line))
    return false;

  NMEAInputLine line(_line);
  if (line.ReadCompare("$POV"))
    return POV(line, info);

  return false;
}

bool
ZenDevice::POV(NMEAInputLine &line, NMEAInfo &info)
{
  /*
   * Type definitions:
   *
   * E: TE vario in m/s
   * P: static pressure in hPa
   * Q: dynamic pressure in Pa
   * R: total pressure in hPa
   * S: true airspeed in km/h
   * T: temperature in deg C
   */

  while (!line.IsEmpty()) {
    char type = line.ReadOneChar();
    if (type == '\0')
      break;

    double value;
    if (!line.ReadChecked(value))
      break;

    switch (type) {
      case 'E': {
        info.ProvideTotalEnergyVario(value);
        break;
      }
      case 'P': {
	AtmosphericPressure pressure;
	if (value > double(9998.0))
	{
		info.static_pressure_available.Clear();
	}
	else
	{
		pressure = AtmosphericPressure::HectoPascal(value);
		info.ProvideStaticPressure(pressure);
	}
        break;
      }
      case 'Q': {
	AtmosphericPressure pressure;
	if (value > double(9998.0))
	{
		info.dyn_pressure_available.Clear();
	}
	else
	{
		pressure = AtmosphericPressure::Pascal(value);
		info.ProvideDynamicPressure(pressure);
	}
        break;
      }
      case 'R': {
        AtmosphericPressure pressure = AtmosphericPressure::HectoPascal(value);
        info.ProvidePitotPressure(pressure);
        break;
      }
      case 'S': {
        value = Units::ToSysUnit(value, Unit::KILOMETER_PER_HOUR);
        info.ProvideTrueAirspeed(value);
        break;
      }
      case 'T': {
        info.temperature = Temperature::FromCelsius(value);
        info.temperature_available = true;
        break;
      }	  
      case 'V': {
        info.voltage = value;
	info.voltage_available.Update(info.clock);
        break;
      }
    }
  }

  return true;
}

static Device *
ZenCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new ZenDevice(com_port);
}

const struct DeviceRegister zen_driver = {
  _T("Zen Soaring"),
  _T("Zen Soaring"),
  0,
  ZenCreateOnPort,
};
