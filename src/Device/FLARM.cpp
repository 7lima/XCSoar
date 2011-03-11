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

#include "Device/FLARM.hpp"
#include "Device/device.hpp"
#include "Device/Port.hpp"
#include "Device/Declaration.hpp"

#include <assert.h>
#include <tchar.h>

#ifdef _UNICODE
#include <windows.h>
#endif

static bool
FlarmDeclareSetGet(Port *port, char *Buffer)
{
  assert(port != NULL);
  assert(Buffer != NULL);

  port->Write('$');
  port->Write(Buffer);
  port->Write("\r\n");

  Buffer[6] = _T('A');
  return port->ExpectString(Buffer);
}

#ifdef _UNICODE
static bool
FlarmDeclareSetGet(Port *port, TCHAR *s)
{
  assert(port != NULL);
  assert(s != NULL);

  char buffer[_tcslen(s) * 4 + 1];
  return ::WideCharToMultiByte(CP_ACP, 0, s, -1, buffer, sizeof(buffer),
                               NULL, NULL) > 0 &&
    FlarmDeclareSetGet(port, buffer);
}
#endif

static bool
FlarmDeclareInternal(Port *port, const Declaration *decl)
{
  TCHAR Buffer[256];

  _stprintf(Buffer, _T("PFLAC,S,PILOT,%s"), decl->PilotName.c_str());
  if (!FlarmDeclareSetGet(port, Buffer))
    return false;

  _stprintf(Buffer, _T("PFLAC,S,GLIDERID,%s"), decl->AircraftReg.c_str());
  if (!FlarmDeclareSetGet(port, Buffer))
    return false;

  _stprintf(Buffer, _T("PFLAC,S,GLIDERTYPE,%s"), decl->AircraftType.c_str());
  if (!FlarmDeclareSetGet(port, Buffer))
    return false;

  _stprintf(Buffer, _T("PFLAC,S,NEWTASK,Task"));
  if (!FlarmDeclareSetGet(port, Buffer))
    return false;

  _stprintf(Buffer, _T("PFLAC,S,ADDWP,0000000N,00000000E,TAKEOFF"));
  if (!FlarmDeclareSetGet(port, Buffer))
    return false;

  for (unsigned i = 0; i < decl->size(); ++i) {
    int DegLat, DegLon;
    double tmp, MinLat, MinLon;
    char NoS, EoW;

    tmp = decl->get_location(i).Latitude.value_degrees();
    NoS = 'N';
    if(tmp < 0)
      {
	NoS = 'S';
	tmp = -tmp;
      }
    DegLat = (int)tmp;
    MinLat = (tmp - DegLat) * 60 * 1000;

    tmp = decl->get_location(i).Longitude.value_degrees();
    EoW = 'E';
    if(tmp < 0)
      {
	EoW = 'W';
	tmp = -tmp;
      }
    DegLon = (int)tmp;
    MinLon = (tmp - DegLon) * 60 * 1000;

    _stprintf(Buffer,
	      _T("PFLAC,S,ADDWP,%02d%05.0f%c,%03d%05.0f%c,%s"),
	      DegLat, MinLat, NoS, DegLon, MinLon, EoW,
	      decl->get_name(i));
    if (!FlarmDeclareSetGet(port, Buffer))
      return false;
  }

  _stprintf(Buffer, _T("PFLAC,S,ADDWP,0000000N,00000000E,LANDING"));
  if (!FlarmDeclareSetGet(port, Buffer))
    return false;

  // PFLAC,S,KEY,VALUE
  // Expect
  // PFLAC,A,blah
  // PFLAC,,COPIL:
  // PFLAC,,COMPID:
  // PFLAC,,COMPCLASS:

  // PFLAC,,NEWTASK:
  // PFLAC,,ADDWP:

  return true;
}

bool
FlarmDeclare(Port *port, const Declaration *decl)
{
  assert(port != NULL);

  port->StopRxThread();
  port->SetRxTimeout(500); // set RX timeout to 500[ms]

  bool result = FlarmDeclareInternal(port, decl);

  // TODO bug: JMW, FLARM Declaration checks
  // Note: FLARM must be power cycled to activate a declaration!
  // Only works on IGC approved devices
  // Total data size must not surpass 183 bytes
  // probably will issue PFLAC,ERROR if a problem?

  port->SetRxTimeout(0); // clear timeout
  port->StartRxThread(); // restart RX thread

  return result;
}
