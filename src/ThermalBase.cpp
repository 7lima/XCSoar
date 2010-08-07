/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

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

#include "ThermalBase.hpp"
#include "Terrain/RasterTerrain.hpp"
#include "Terrain/RasterMap.hpp"
#include "Components.hpp"
#include "Math/Earth.hpp"

void
EstimateThermalBase(const GEOPOINT Thermal_Location,
                    const fixed altitude, const fixed wthermal,
                    const SpeedVector wind,
                    GEOPOINT *ground_location, fixed *ground_alt)
{
  if ((Thermal_Location.Longitude == Angle())
      || (Thermal_Location.Latitude == Angle())
      || (wthermal < fixed_one)) {
    ground_location->Longitude = Angle();
    ground_location->Latitude = Angle();
    *ground_alt = fixed_minus_one;
    return;
  }

  fixed Tmax;
  Tmax = (altitude / wthermal);
  fixed dt = Tmax / 10;

  if (terrain != NULL)
    terrain->Lock();

  GEOPOINT loc;
  FindLatitudeLongitude(Thermal_Location, wind.bearing, wind.norm * dt, &loc);

  for (fixed t = fixed_zero; t <= Tmax; t += dt) {
    FindLatitudeLongitude(Thermal_Location, wind.bearing, wind.norm * t,
                          &loc);

    fixed hthermal = altitude - wthermal * t;
    short hground = terrain != NULL
      ? terrain->GetTerrainHeight(loc)
      : RasterTerrain::TERRAIN_INVALID;
    if (hground == RasterTerrain::TERRAIN_INVALID)
      hground = 0;

    fixed dh = hthermal - fixed(hground);
    if (negative(dh)) {
      t = t + dh / wthermal;
      FindLatitudeLongitude(Thermal_Location, wind.bearing, wind.norm * t,
                            &loc);
      break;
    }
  }

  short hground = terrain != NULL
    ? terrain->GetTerrainHeight(loc)
    : RasterTerrain::TERRAIN_INVALID;
  if (hground == RasterTerrain::TERRAIN_INVALID)
    hground = 0;

  if (terrain != NULL)
    terrain->Unlock();

  *ground_location = loc;
  *ground_alt = fixed(hground);
}

