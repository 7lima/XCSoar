/***********************************************************************
**
**   WindStore.cpp
**
**   This file is part of Cumulus
**
************************************************************************
**
**   Copyright (c):  2002 by Andr� Somers
**
**   This file is distributed under the terms of the General Public
**   Licence. See the file COPYING for more information.
**
**   $Id$
**
***********************************************************************/

/*
NOTE: Some portions copyright as above

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

#include "Wind/WindStore.hpp"
#include "Math/Constants.h"
#include "NMEA/Info.hpp"
#include "NMEA/Derived.hpp"

#include <math.h>

WindStore::WindStore()
{
  //create the lists
  windlist = new WindMeasurementList();
  updated = true;
}

WindStore::~WindStore()
{
  delete windlist;
}

void
WindStore::reset()
{
  delete windlist;
  windlist = new WindMeasurementList();
  updated = true;
  _lastAltitude = fixed_zero;
  _lastWind = Vector();
}

/**
 * Called with new measurements. The quality is a measure for how
 * good the measurement is. Higher quality measurements are more
 * important in the end result and stay in the store longer.
 */
void
WindStore::SlotMeasurement(const NMEA_INFO &info, DERIVED_INFO &derived,
                           Vector windvector, int quality)
{
  updated = true;
  windlist->addMeasurement(info.Time, windvector,
                           info.GPSAltitude, quality);
  //we may have a new wind value, so make sure it's emitted if needed!
  recalculateWind(info, derived);
}

/**
 * Called if the altitude changes.
 * Determines where measurements are stored and may result in a
 * NewWind signal.
 */
void
WindStore::SlotAltitude(const NMEA_INFO &info, DERIVED_INFO &derived)
{
  if ((fabs(info.GPSAltitude - _lastAltitude) > fixed(100)) || updated) {
    //only recalculate if there is a significant change
    recalculateWind(info, derived);

    updated = false;
    _lastAltitude = info.GPSAltitude;
  }
}

const Vector
WindStore::GetWind(fixed Time, fixed h, bool *found) const
{
  return windlist->getWind(Time, h, found);
}

/** Recalculates the wind from the stored measurements.
  * May result in a NewWind signal. */

void
WindStore::recalculateWind(const NMEA_INFO &info, DERIVED_INFO &derived)
{
  bool found;
  Vector CurWind = windlist->getWind(info.Time, info.GPSAltitude, &found);

  if (found) {
    if ((fabs(CurWind.x - _lastWind.x) > fixed_one)
        || (fabs(CurWind.y - _lastWind.y) > fixed_one)
        || updated) {
      _lastWind = CurWind;

      updated = false;
      _lastAltitude = info.GPSAltitude;

      NewWind(info, derived, CurWind);
    }
  } // otherwise, don't change anything
}

void
WindStore::NewWind(const NMEA_INFO &info, DERIVED_INFO &derived,
    Vector &wind)
{
  fixed mag = hypot(wind.x, wind.y);
  fixed bearing;

  if (wind.y == fixed_zero && wind.x == fixed_zero)
    bearing = fixed_zero;
  else
    bearing = atan2(wind.y, wind.x) * fixed_rad_to_deg;

  if (mag < fixed(30)) { // limit to reasonable values
    if (negative(bearing))
      bearing += fixed_360;

    derived.estimated_wind = SpeedVector(Angle::degrees(bearing), mag);
  } else {
    // TODO code: give warning, wind estimate bogus or very strong!
  }

  #ifdef DEBUG_WIND
  LogDebug(_T("%f %f 0 # wind estimate\n"), wind.x, wind.y);
  #endif

}
