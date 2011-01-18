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

#ifndef XCSOAR_NMEA_THERMAL_BAND_H
#define XCSOAR_NMEA_THERMAL_BAND_H

#include "Math/fixed.hpp"

#define NUMTHERMALBUCKETS 10

/**
 * Derived thermal climb rate histogram by altitude (time averaged)
 * 
 */
struct ThermalBandInfo
{
  /** Maximum height achieved in circling */ 
  fixed MaxThermalHeight;
  /** Number of samples in each bucket */ 
  int    ThermalProfileN[NUMTHERMALBUCKETS];
  /** Average climb rate in each bucket */ 
  fixed ThermalProfileW[NUMTHERMALBUCKETS];

  void clear();

  /**
   * Calculates the bucket number for the specified height.
   */
  unsigned bucket_for_height(fixed height) const;

  /**
   * Calculates the base height of the specified bucket.
   */
  fixed bucket_height(unsigned bucket) const;

  void add(fixed height, fixed total_energy_vario);
};

#endif
