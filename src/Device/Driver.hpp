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

#ifndef XCSOAR_DEVICE_DRIVER_HPP
#define XCSOAR_DEVICE_DRIVER_HPP

#include "Device/Declaration.hpp"

#include <tchar.h>

struct NMEA_INFO;
class Port;
class AtmosphericPressure;

/**
 * This is the interface for a device driver.
 */
class Device {
public:
  virtual ~Device();

  virtual bool Open() = 0;

  virtual void LinkTimeout() = 0;

  virtual bool IsLogger() = 0;
  virtual bool IsBaroSource() = 0;

  virtual bool ParseNMEA(const char *line, struct NMEA_INFO *info,
                         bool enable_baro) = 0;

  virtual bool PutMacCready(double MacCready) = 0;
  virtual bool PutBugs(double bugs) = 0;
  virtual bool PutBallast(double ballast) = 0;
  virtual bool PutQNH(const AtmosphericPressure& pres) = 0;
  virtual bool PutVoice(const TCHAR *sentence) = 0;
  virtual bool PutVolume(int volume) = 0;
  virtual bool PutActiveFrequency(double frequency) = 0;
  virtual bool PutStandbyFrequency(double frequency) = 0;

  virtual bool Declare(const struct Declaration *declaration) = 0;

  virtual void OnSysTicker() = 0;
};

/**
 * This class implements all #Device methods.  You may use it as a
 * base class for specific device drivers.
 */
class AbstractDevice : public Device {
public:
  virtual bool Open();

  virtual void LinkTimeout();

  virtual bool IsLogger();
  virtual bool IsBaroSource();

  virtual bool ParseNMEA(const char *line, struct NMEA_INFO *info,
                         bool enable_baro);

  virtual bool PutMacCready(double MacCready);
  virtual bool PutBugs(double bugs);
  virtual bool PutBallast(double ballast);
  virtual bool PutQNH(const AtmosphericPressure& pres);
  virtual bool PutVoice(const TCHAR *sentence);
  virtual bool PutVolume(int volume);
  virtual bool PutActiveFrequency(double frequency);
  virtual bool PutStandbyFrequency(double frequency);

  virtual bool Declare(const struct Declaration *declaration);

  virtual void OnSysTicker();
};

typedef enum {
  dfLogger,
  dfBaroAlt,
  dfNmeaOut,
  dfRadio,
} DeviceFlags_t;

#define drfLogger	(1l << dfLogger)
#define drfBaroAlt	(1l << dfBaroAlt)
#define drfNmeaOut	(1l << dfNmeaOut)
#define drfRadio	(1l << dfRadio)

/**
 * This is the structure exported by a device driver.
 */
struct DeviceRegister {
  const TCHAR *Name;
  unsigned int Flags;
  Device *(*CreateOnPort)(Port *com_port);
};

#endif
