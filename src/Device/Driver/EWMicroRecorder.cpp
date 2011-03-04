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


// ToDo

// adding baro alt sentance parser to support baro source priority  if (d == pDevPrimaryBaroSource){...}

#include "Device/Driver/EWMicroRecorder.hpp"
#include "Device/Driver.hpp"
#include "Device/Parser.hpp"
#include "Device/Port.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Waypoint/Waypoint.hpp"
#include "Units.hpp"

#include <tchar.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

#ifdef _UNICODE
#include <windows.h>
#endif

// Additional sentance for EW support

class EWMicroRecorderDevice : public AbstractDevice {
protected:
  Port *port;

  char user_data[2500];

public:
  EWMicroRecorderDevice(Port *_port)
    :port(_port) {}

protected:
  bool TryConnect();

  bool DeclareInner(const Declaration *declaration);

public:
  virtual bool ParseNMEA(const char *line, struct NMEA_INFO *info,
                         bool enable_baro);
  virtual bool Declare(const Declaration *declaration,
                       OperationEnvironment &env);
};

static bool
ReadAltitude(NMEAInputLine &line, fixed &value_r)
{
  fixed value;
  bool available = line.read_checked(value);
  char unit = line.read_first_char();
  if (!available)
    return false;

  if (unit == _T('f') || unit == _T('F'))
    value = Units::ToSysUnit(value, unFeet);

  value_r = value;
  return true;
}

bool
EWMicroRecorderDevice::ParseNMEA(const char *String, NMEA_INFO *GPS_INFO,
                                 bool enable_baro)
{
  if (!NMEAParser::NMEAChecksum(String))
    return false;

  NMEAInputLine line(String);
  char type[16];
  line.read(type, 16);

  if (strcmp(type, "$PGRMZ") == 0) {
    fixed value;

    /* The normal Garmin $PGRMZ line contains the "true" barometric
       altitude above MSL (corrected with QNH), but EWMicroRecorder
       differs here slightly: it emits the uncorrected barometric
       altitude.  That is the only reason why we catch this sentence
       in the driver instead of letting the generic class NMEAParser
       do it. */
    if (ReadAltitude(line, value) && enable_baro)
      GPS_INFO->ProvideBaroAltitude1013(NMEA_INFO::BARO_ALTITUDE_EW,
                                        value);

    return true;
  } else
    return false;
}

bool
EWMicroRecorderDevice::TryConnect()
{
  int retries=10;

  while (--retries){

    port->Write('\x02');         // send IO Mode command

    unsigned user_size = 0;
    bool started = false;

    int i;
    while ((i = port->GetChar()) != EOF) {
      char ch = (char)i;

      if (!started && ch == _T('-'))
        started = true;

      if (started) {
        if (ch == 0x13) {
          port->Write('\x16');
          user_data[user_size] = 0;
          // found end of file
          return true;
        } else {
          if (user_size < sizeof(user_data) - 1) {
            user_data[user_size] = ch;
            user_size++;
          }
        }
      }
    }

  }

  return false;
}


static void
EWMicroRecorderPrintf(Port *port, const TCHAR *fmt, ...)
{
  TCHAR EWStr[128];
  va_list ap;

  va_start(ap, fmt);
  _vstprintf(EWStr, fmt, ap);
  va_end(ap);

#ifdef _UNICODE
  char buffer[256];
  if (::WideCharToMultiByte(CP_ACP, 0, EWStr, -1, buffer, sizeof(buffer),
                            NULL, NULL) <= 0)
    return;
#else
  const char *buffer = EWStr;
#endif

  port->Write(buffer);
}

static void
EWMicroRecorderWriteWayPoint(Port *port,
                             const Waypoint &way_point, const TCHAR *EWType)
{
  int DegLat, DegLon;
  double tmp, MinLat, MinLon;
  TCHAR NoS, EoW;

  // prepare latitude
  tmp = way_point.Location.Latitude.value_degrees();
  NoS = _T('N');
  if (tmp < 0)
    {
      NoS = _T('S');
      tmp = -tmp;
    }

  DegLat = (int)tmp;
  MinLat = (tmp - DegLat) * 60 * 1000;

  // prepare long
  tmp = way_point.Location.Longitude.value_degrees();
  EoW = _T('E');
  if (tmp < 0)
    {
      EoW = _T('W');
      tmp = -tmp;
    }

  DegLon = (int)tmp;
  MinLon = (tmp - DegLon) * 60 * 1000;

  EWMicroRecorderPrintf(port,
                        _T("%-17s %02d%05d%c%03d%05d%c %s\r\n"),
                        EWType,
                        DegLat, (int)MinLat, NoS,
                        DegLon, (int)MinLon, EoW,
                        way_point.Name.c_str());
}

bool
EWMicroRecorderDevice::DeclareInner(const Declaration *decl)
{
  assert(decl != NULL);
  assert(decl->size() >= 2);
  assert(decl->size() <= 12);

  if (!TryConnect())
    return false;

  port->Write('\x18');         // start to upload file
  port->Write(user_data);
  EWMicroRecorderPrintf(port, _T("%-15s %s\r\n"),
                        _T("Pilot Name:"), decl->PilotName.c_str());
  EWMicroRecorderPrintf(port, _T("%-15s %s\r\n"),
                        _T("Competition ID:"), decl->AircraftRego.c_str());
  EWMicroRecorderPrintf(port, _T("%-15s %s\r\n"),
                        _T("Aircraft Type:"), decl->AircraftType.c_str());
  port->Write("Description:      Declaration\r\n");

  for (unsigned i = 0; i < 11; i++) {
    if (i+1>= decl->size()) {
      EWMicroRecorderPrintf(port, _T("%-17s %s\r\n"),
               _T("TP LatLon:"), _T("0000000N00000000E TURN POINT"));
    } else {
      const Waypoint &wp = decl->get_waypoint(i);
      if (i == 0) {
        EWMicroRecorderWriteWayPoint(port, wp, _T("Take Off LatLong:"));
        EWMicroRecorderWriteWayPoint(port, wp, _T("Start LatLon:"));
      } else if (i + 1 < decl->size()) {
        EWMicroRecorderWriteWayPoint(port, wp, _T("TP LatLon:"));
      }
    }
  }

  const Waypoint &wp = decl->get_last_waypoint();
  EWMicroRecorderWriteWayPoint(port, wp, _T("Finish LatLon:"));
  EWMicroRecorderWriteWayPoint(port, wp, _T("Land LatLon:"));

  port->Write('\x03');         // finish sending user file

  return port->ExpectString("uploaded successfully");
}

bool
EWMicroRecorderDevice::Declare(const Declaration *decl,
                               OperationEnvironment &env)
{
  // Must have at least two, max 12 waypoints
  if (decl->size() < 2 || decl->size() > 12)
    return false;

  port->StopRxThread();

  port->SetRxTimeout(500);                     // set RX timeout to 500[ms]

  bool success = DeclareInner(decl);

  port->Write("!!\r\n");         // go back to NMEA mode

  port->SetRxTimeout(0);                       // clear timeout
  port->StartRxThread();                       // restart RX thread

  return success;
}


static Device *
EWMicroRecorderCreateOnPort(Port *com_port)
{
  return new EWMicroRecorderDevice(com_port);
}

const struct DeviceRegister ewMicroRecorderDevice = {
  _T("EW MicroRecorder"),
  drfLogger | drfBaroAlt,
  EWMicroRecorderCreateOnPort,
};
