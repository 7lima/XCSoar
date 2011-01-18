/* Copyright_License {

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

#include "Navigation/GeoPoint.hpp"
#include "Navigation/Geometry/GeoVector.hpp"
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

void
GeoPoint::distance_bearing(const GeoPoint &other, fixed &distance,
                           Angle &bearing) const
{
  ::DistanceBearing(*this, other, &distance, &bearing);
}

GeoVector
GeoPoint::distance_bearing(const GeoPoint &other) const
{
  GeoVector gv;
  ::DistanceBearing(*this, other, &gv.Distance, &gv.Bearing);
  return gv;
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
