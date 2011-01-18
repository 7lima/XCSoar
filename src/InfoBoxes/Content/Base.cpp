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

#include "Base.hpp"
#include "InfoBoxes/InfoBoxWindow.hpp"
#include "Interface.hpp"
#include "Waypoint/Waypoint.hpp"
#include "Math/Angle.hpp"
#include "Units.hpp"

#include <stdio.h>

static
void FillInfoBoxWaypointName(InfoBoxWindow& infobox, const Waypoint* way_point,
                             const bool title=true)
{
  TCHAR tmp[32];
  if (!way_point) {
    tmp[0] = '\0';
  } else {
    switch(XCSoarInterface::SettingsMap().DisplayTextType) {
    case DISPLAYFIRSTTHREE:
      _tcsncpy(tmp, way_point->Name.c_str(), 3);
      tmp[3] = '\0';
      break;

    case DISPLAYFIRSTFIVE:
      _tcsncpy(tmp, way_point->Name.c_str(), 5);
      tmp[5] = '\0';
      break;

    case DISPLAYNUMBER:
      _stprintf(tmp, _T("%d"), way_point->id);
      break;

    default:
      _tcsncpy(tmp, way_point->Name.c_str(), (sizeof(tmp) / sizeof(TCHAR)) - 1);
      tmp[(sizeof(tmp) / sizeof(TCHAR)) - 1] = '\0';
      break;
    }
  }
  if (title) {
    infobox.SetTitle(tmp);
  } else {
    infobox.SetComment(tmp);
  }
}

void
InfoBoxContent::SetValueBearingDifference(InfoBoxWindow &infobox, Angle delta)
{
  double delta_degrees = delta.as_delta().value_degrees();
  TCHAR tmp[32];
  if (delta_degrees > 1)
    _stprintf(tmp, _T("%2.0f°»"), delta_degrees);
  else if (delta_degrees < -1)
    _stprintf(tmp, _T("«%2.0f°"), -delta_degrees);
  else
    _tcscpy(tmp, _T("«»"));

  infobox.SetValue(tmp);
}

void
InfoBoxContent::SetTitleFromWaypointName(InfoBoxWindow &infobox,
                                         const Waypoint* waypoint)
{
  FillInfoBoxWaypointName(infobox, waypoint, true);
}

void
InfoBoxContent::SetCommentFromWaypointName(InfoBoxWindow &infobox,
                                           const Waypoint* waypoint)
{
  FillInfoBoxWaypointName(infobox, waypoint, false);
}

void
InfoBoxContent::SetValueFromFixed(InfoBoxWindow &infobox,
                                  const TCHAR* format, fixed value)
{
  TCHAR tmp[32];
  _stprintf(tmp, format, (double)value);
  infobox.SetValue(tmp);
}

void
InfoBoxContent::SetValueFromDistance(InfoBoxWindow &infobox, fixed distance)
{
  TCHAR tmp[32];
  Units::FormatUserDistance(distance, tmp, 32, false);
  infobox.SetValue(tmp);

  infobox.SetValueUnit(Units::DistanceUnit);
}
