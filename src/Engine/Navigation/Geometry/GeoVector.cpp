/* Copyright_License {

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
#include "GeoVector.hpp"
#include "Math/Earth.hpp"

GeoVector::GeoVector(const GeoPoint &source, const GeoPoint &target,
                     const bool is_average)
{
  source.distance_bearing(target, Distance, Bearing);
}

GeoPoint 
GeoVector::end_point(const GeoPoint &source) const
{
  if (!positive(Distance)) {
    return source;
  } else {
    GeoPoint p;
    ::FindLatitudeLongitude(source, Bearing, Distance, &p);
    return p;
  }
}

GeoPoint 
GeoVector::mid_point(const GeoPoint &source) const
{
  if (!positive(Distance)) {
    return source;
  } else {
    GeoPoint p;
    ::FindLatitudeLongitude(source, Bearing, Distance*fixed_half, &p);
    return p;
  }
}

fixed
GeoVector::minimum_distance(const GeoPoint &source,
                            const GeoPoint &ref) const
{
  const GeoPoint end = end_point(source);
  return (::CrossTrackError(source, end, ref, NULL));
}

GeoPoint 
GeoVector::intermediate_point(const GeoPoint &source, 
                              const fixed distance) const
{
  return source.intermediate_point(end_point(source), distance);
}


bool operator != (const GeoPoint&g1, const GeoPoint &g2) {
  return (g1.Latitude != g2.Latitude) || (g1.Longitude != g2.Longitude);
}

