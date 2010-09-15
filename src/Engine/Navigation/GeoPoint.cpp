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
#include "Navigation/GeoPoint.hpp"
#include "Math/Earth.hpp"

GeoPoint 
GeoPoint::parametric(const GeoPoint &delta, const fixed t) const
{
  return (*this) + delta * t;
}

GeoPoint 
GeoPoint::interpolate(const GeoPoint &end, const fixed t) const
{
  return (*this) + (end - (*this)) * t;
}

fixed
GeoPoint::distance(const GeoPoint &other) const
{
  return ::Distance(*this, other);
}

Angle
GeoPoint::bearing(const GeoPoint &other) const
{
  return ::Bearing(*this, other);
}

fixed 
GeoPoint::projected_distance(const GeoPoint &from,
                             const GeoPoint &to) const
{
  return ::ProjectedDistance(from, to, *this);
}

bool 
GeoPoint::equals(const GeoPoint &other) const
{
  return (Longitude == other.Longitude) && (Latitude == other.Latitude);
}

bool 
GeoPoint::sort(const GeoPoint &sp) const
{
  if (Longitude < sp.Longitude)
    return false;
  else if (Longitude == sp.Longitude)
    return Latitude > sp.Latitude;
  else
    return true;
}


GeoPoint 
GeoPoint::intermediate_point(const GeoPoint &destination, 
                             const fixed distance) const
{
  /* slow way */
  return ::IntermediatePoint(*this, destination, distance);
  /* fast way (linear interpolation)
  GeoPoint end = end_point(source);
  if (Distance > fixed_zero) {
    fixed t = distance / Distance;
    return source + (end - source) * t;
  } else {
    return source;
  }
  */
}

bool
GeoPoint::is_null() const
{
  return (Longitude.value_degrees() == fixed_zero) &&
         (Latitude.value_degrees() == fixed_zero);
}
