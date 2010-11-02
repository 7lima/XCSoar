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

#include "AirfieldDetails.h"
#include "Language.hpp"
#include "Profile/ProfileKeys.hpp"
#include "LogFile.hpp"
#include "Interface.hpp"
#include "StringUtil.hpp"
#include "UtilsText.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Engine/Waypoint/Waypoints.hpp"
#include "IO/ConfiguredFile.hpp"
#include "ProgressGlue.hpp"

static const Waypoint *
find_waypoint(Waypoints &way_points, const TCHAR *name)
{
  const Waypoint *wp = way_points.lookup_name(name);
  if (wp != NULL)
    return wp;

  size_t name_length = _tcslen(name);
  TCHAR buffer[name_length + 4];
  _tcscpy(buffer, name);
  _tcscpy(buffer + name_length, _T(" AF"));
  wp = way_points.lookup_name(buffer);
  if (wp != NULL)
    return wp;

  _tcscpy(buffer + name_length, _T(" AD"));
  wp = way_points.lookup_name(buffer);
  if (wp != NULL)
    return wp;

  if (name_length > 5 && _tcscmp(name + name_length - 5, _T("=HOME")) == 0) {
    buffer[name_length - 5] = _T('\0');
    wp = way_points.lookup_name(buffer);
    if (wp != NULL) {
      XCSoarInterface::SetSettingsComputer().HomeWaypoint = wp->id;
      return wp;
    }
  }

  return NULL;
}

static void
SetAirfieldDetails(Waypoints &way_points, const TCHAR *name,
                   const tstring &Details)
{
  const Waypoint *wp = find_waypoint(way_points, name);
  if (wp != NULL)
    way_points.set_details(*wp, Details);
}

/**
 * Parses the data provided by the airfield details file handle
 */
static void
ParseAirfieldDetails(Waypoints &way_points, TLineReader &reader)
{
  tstring Details;
  TCHAR Name[201];

  Name[0] = 0;

  bool inDetails = false;
  int i;

  long filesize = std::max(reader.size(), 1l);
  ProgressGlue::SetRange(100);

  TCHAR *TempString;
  while ((TempString = reader.read()) != NULL) {
    if (TempString[0] == '[') { // Look for start
      if (inDetails)
        SetAirfieldDetails(way_points, Name, Details);

      Details.clear();

      // extract name
      for (i = 1; i <= 201; i++) {
        if (TempString[i] == ']')
          break;

        Name[i - 1] = TempString[i];
      }
      Name[i - 1] = 0;

      inDetails = true;

      ProgressGlue::SetValue(reader.tell() * 100 / filesize);
    } else {
      // append text to details string
      if (!string_is_empty(TempString)) {
        Details += TempString;
        Details += _T('\n');
      }
    }
  }

  if (inDetails) {
    SetAirfieldDetails(way_points, Name, Details);
    Details.clear();
  }
}

/**
 * Opens the airfield details file and parses it
 */
void
ReadAirfieldFile(Waypoints &way_points)
{
  LogStartUp(_T("ReadAirfieldFile"));

  TLineReader *reader =
    OpenConfiguredTextFile(szProfileAirfieldFile, _T("airfields.txt"));
  if (reader == NULL)
    return;

  ProgressGlue::Create(_("Loading Airfield Details File..."));

  ParseAirfieldDetails(way_points, *reader);
  delete reader;
}
