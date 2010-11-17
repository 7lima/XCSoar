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

#ifndef XCSOAR_GeoPoint_HPP
#define XCSOAR_GeoPoint_HPP

#include "Math/Angle.hpp"
#include "Compiler.h"

struct GeoVector;

/**
 * Geodetic coordinate expressed as Longitude and Latitude angles.
 */
struct GeoPoint {
  friend class Serialiser;

  /**
   * Constructor (origin)
   *
   * @return Point initialised at origin
   */
  GeoPoint()
    :Longitude(Angle::native(fixed_zero)),
     Latitude(Angle::native(fixed_zero)) {}

  /**
   * Constructor (supplied location)
   *
   * @param _Longitude Longitude of point
   * @param _Latitude Latitude of point
   *
   * @return Initialised object
   */
  GeoPoint(const Angle &_Longitude, const Angle &_Latitude) :
    Longitude(_Longitude), Latitude(_Latitude) {}

  Angle Longitude; /**< Longitude (deg) */
  Angle Latitude; /**< Latitude (deg) */

  /**
   * Normalize the values, so this object can be used properly in
   * calculations, without unintended side effects (such as -1 degrees
   * vs 359 degrees).  This modification is in-place.
   */
  GeoPoint &normalize() {
    Longitude = Longitude.as_delta();
    return *this;
  }

  /**
   * Find location a parametric distance along a vector from this point
   *
   * @param delta Vector to feed in
   * @param t Parametric distance along delta to add [0,1]
   *
   * @return Location of point
   */
  gcc_pure
  GeoPoint parametric(const GeoPoint &delta, const fixed t) const;

  /**
   * Find location interpolated from this point towards end
   *
   * @param end Endpoint of interpolation
   * @param t Parametric distance along this to end [0,1]
   *
   * @return Location of point
   */
  gcc_pure
  GeoPoint interpolate(const GeoPoint &end, const fixed t) const;

  /**
   * Multiply a point by a factor (used for deltas)
   *
   * @param x Factor to magnify
   *
   * @return Modified point
   */
  gcc_pure
  GeoPoint operator* (const fixed x) const {
    GeoPoint res = *this;
    res.Longitude *= x;
    res.Latitude *= x;
    return res;
  };

  /**
   * Add a delta to a point
   *
   * @param delta Delta to add
   *
   * @return Modified point
   */
  gcc_pure
  GeoPoint operator+ (const GeoPoint &delta) const {
    GeoPoint res = *this;
    res.Longitude += delta.Longitude;
    res.Latitude += delta.Latitude;
    return res;
  };

  /**
   * Add a delta to a point
   *
   * @param delta Delta to add
   *
   * @return Modified point
   */
  const GeoPoint& operator+= (const GeoPoint &delta) {
    Longitude += delta.Longitude;
    Latitude += delta.Latitude;
    return *this;
  };

  /**
   * Subtracts a delta from a point
   *
   * @param delta Delta to subtract
   *
   * @return Modified point
   */
  gcc_pure
  GeoPoint operator- (const GeoPoint &delta) const {
    GeoPoint res = *this;
    res.Longitude -= delta.Longitude;
    res.Latitude -= delta.Latitude;
    return res.normalize();
  };

  /**
   * Calculate great circle distance from this to the other
   *
   * @param other Other location
   *
   * @return Distance (m)
   */
  gcc_pure
  fixed distance(const GeoPoint &other) const;

  /**
   * Calculate great circle initial bearing from this to the other
   *
   * @param other Other location
   *
   * @return Bearing (deg)
   */
  gcc_pure
  Angle bearing(const GeoPoint &other) const;

  /**
   * Calculate great circle distance and initial bearing from this to the other
   */
  void distance_bearing(const GeoPoint &other, fixed &distance, Angle &bearing) const;

  /**
   * Calculate great circle distance and initial bearing from this to the other
   */
  gcc_pure
  GeoVector distance_bearing(const GeoPoint &other) const;

  /**
   * Find distance along a great-circle path that this point
   * is projected to
   *
   * @param from Start location
   * @param to End location
   *
   * @return Distance (m) along from-to line
   */
  gcc_pure
  fixed projected_distance(const GeoPoint &from,
                           const GeoPoint &to) const;

  /**
   * Find point a set distance along a great-circle path towards
   * a destination
   *
   * @param destination End location
   * @param distance distance (m)
   *
   * @return Location of point
   */
  gcc_pure
  GeoPoint intermediate_point(const GeoPoint &destination, 
                              const fixed distance) const;

  /**
   * Test whether two points are co-located
   *
   * @param other Point to compare
   *
   * @return True if coincident
   */
  gcc_pure
  bool equals(const GeoPoint &other) const;

  /**
   * Test whether two points are co-located
   *
   * @param other Point to compare
   *
   * @return True if coincident
   */
  gcc_pure
  bool operator== (const GeoPoint &other) const {
    return equals(other);
  }

  /**
   * Rank two points according to longitude, then latitude
   *
   * @param other Point to compare to
   *
   * @return True if this point is further left (or if equal, lower) than the other
   */
  gcc_pure
  bool sort (const GeoPoint &other) const;

  /**
   * Test whether the point is exactly at (0, 0)
   *
   * @return True if Lat = Lon = 0
   */
  gcc_pure
  bool is_null() const;
};

#endif
