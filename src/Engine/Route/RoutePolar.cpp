/* Copyright_License {

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
#include "RoutePolar.hpp"
#include "GlideSolvers/GlidePolar.hpp"
#include "GlideSolvers/GlideState.hpp"
#include "GlideSolvers/GlideResult.hpp"
#include "Navigation/SpeedVector.hpp"
#include "Navigation/TaskProjection.hpp"
#include "Math/FastMath.h"
#include "Terrain/RasterMap.hpp"
#include <assert.h>
#include <limits.h>

#define MC_CEILING_PENALTY_FACTOR 5.0

GlideResult
RoutePolar::solve_task(const GlidePolar& glide_polar,
                       const SpeedVector& wind,
                       const Angle theta, const bool glide) const
{
  fixed altitude = glide? fixed(1.0e5): fixed_zero;
  GlideState task(GeoVector(fixed(1.0), theta), fixed_zero, altitude, wind);
  return glide_polar.solve(task);
}


void RoutePolar::initialise(const GlidePolar& polar,
                            const SpeedVector& wind,
                            const bool is_glide)
{
  for (unsigned i=0; i< ROUTEPOLAR_POINTS; ++i) {
    Angle ang(Angle::radians(fixed_half_pi-i*fixed_two_pi/ROUTEPOLAR_POINTS));
    GlideResult res = solve_task(polar, wind, ang, is_glide);
    RoutePolarPoint point(res.TimeElapsed, res.HeightGlide);
    if (res.Solution != GlideResult::RESULT_OK)
      point.valid = false;
    points[i] = point;
  }
}


int
RoutePolar::dxdy_to_index(const int dx, const int dy)
{
  const int adx = abs(dx);
  const int ady = abs(dy);
  const int v = i_normalise_sine(adx, ady);
  int index;
  if (adx<= ady) {
    index = v;
  } else {
    index = ROUTEPOLAR_Q1 - v;
  }
  if (dx<0)
    if (dy<0) {
      return index + ROUTEPOLAR_Q2;
    } else {
      return ROUTEPOLAR_Q2-index;
    }
  else if (dy<0)
    return ROUTEPOLAR_Q3-index;

  return index;
}


void
RoutePolar::index_to_dxdy(const int index, int& dx, int& dy)
{
  static const int sx[ROUTEPOLAR_POINTS]= {128, 126, 123, 118, 111, 102, 91, 79, 66, 51, 36, 20, 4, -12, -28, -44, -59, -73, -86, -97, -107, -115, -121, -125, -127, -127, -125, -121, -115, -107, -97, -86, -73, -59, -44, -28, -12, 4, 20, 36, 51, 66, 79, 91, 102, 111, 118, 123, 126, };
  static const int sy[ROUTEPOLAR_POINTS]= {0, 16, 32, 48, 62, 76, 89, 100, 109, 117, 122, 126, 127, 127, 124, 120, 113, 104, 94, 82, 69, 55, 40, 24, 8, -8, -24, -40, -55, -69, -82, -94, -104, -113, -120, -124, -127, -127, -126, -122, -117, -109, -100, -89, -76, -62, -48, -32, -16, };

  dx = sx[index];
  dy = sy[index];
}


GeoPoint
RoutePolars::msl_intercept(const int index, const AGeoPoint& p, const TaskProjection& proj) const
{
  const FlatGeoPoint fp = proj.project(p);
  const fixed d = p.altitude*polar_glide.get_point(index).inv_gradient;
  const fixed scale = proj.get_approx_scale();
  const int steps = int(d / scale) + 1;
  int dx;
  int dy;
  RoutePolar::index_to_dxdy(index, dx, dy);
  dx= (dx*steps)>>7;
  dy= (dy*steps)>>7;
  const FlatGeoPoint dp(fp.Longitude+dx, fp.Latitude+dy);
  return (proj.unproject(dp)-(GeoPoint)p)+(GeoPoint)p;
}

void
RoutePolars::calc_footprint(const AGeoPoint& origin,
                            GeoPoint p[ROUTEPOLAR_POINTS],
                            const RasterMap* map,
                            const TaskProjection& proj) const
{
  const bool valid = map && map->isMapLoaded();
  const short altitude = origin.altitude-(short)config.safety_height_terrain;
  AGeoPoint m_origin((GeoPoint)origin, altitude);
  for (int i=0; i< ROUTEPOLAR_POINTS; ++i) {
    const GeoPoint dest = msl_intercept(i, m_origin, proj);
    if (valid)
      p[i] = map->Intersection(m_origin, altitude, altitude, dest);
    else
      p[i] = dest;
  }
}


RoutePolars::RoutePolars(const GlidePolar& polar,
                         const SpeedVector& wind)
{
  initialise(polar, wind);
}


void
RoutePolars::initialise(const GlidePolar& polar,
                        const SpeedVector& wind)
{
  polar_glide.initialise(polar, wind, true);
  polar_cruise.initialise(polar, wind, false);
  const fixed &mc = polar.get_mc();
  if (positive(mc)) {
    inv_M = fixed(MC_CEILING_PENALTY_FACTOR)/mc;
  } else {
    inv_M = fixed_zero;
  }
}


RouteLink::RouteLink (const RouteLinkBase& _link,
                      const TaskProjection &proj):
  RouteLinkBase(_link)
{
  calc_speedups(proj);
}

RouteLink::RouteLink (const RoutePoint& _destination,
                      const RoutePoint& _origin,
                      const TaskProjection &proj):
  RouteLinkBase(_destination, _origin)
{
  calc_speedups(proj);
}

void
RouteLink::calc_speedups(const TaskProjection& proj)
{
  const fixed scale = proj.get_approx_scale();
  const fixed dx = fixed(first.Longitude-second.Longitude);
  const fixed dy = fixed(first.Latitude-second.Latitude);
  mag_rmag(dx, dy, d, inv_d);
  d*= scale;
  inv_d/= scale;
  assert(!negative(d));
  polar_index = RoutePolar::dxdy_to_index(dx, dy);
}


unsigned 
RoutePolars::round_time(const unsigned val)
{
  return val | 0x07;
}


unsigned
RoutePolars::calc_time(const RouteLink& link) const
{
  const int dh = link.second.altitude-link.first.altitude;
  if ((dh<0) && !positive(inv_M))
    return UINT_MAX; // impossible, can't climb

  // dh/d = gradient
  const fixed rho = (dh>0)?
    std::min(fixed_one, (dh*link.inv_d*polar_glide.get_point(link.polar_index).inv_gradient))
    : fixed_zero;

  if ((rho< fixed_one) && !polar_cruise.get_point(link.polar_index).valid)
    return UINT_MAX; // impossible, can't cruise
  if (positive(rho) && !polar_glide.get_point(link.polar_index).valid)
    return UINT_MAX; // impossible, can't glide

  const int t_cruise = (int)(link.d*(rho*polar_glide.get_point(link.polar_index).slowness+
                                     (fixed_one-rho)*polar_cruise.get_point(link.polar_index).slowness));

  if (link.second.altitude > cruise_altitude) {
    // penalise any climbs required above cruise altitude
    const int h_penalty = std::max(0, link.second.altitude-std::max(cruise_altitude, link.first.altitude));
    return t_cruise+(int)(h_penalty*inv_M);
  } else {
    return t_cruise;
  }
}

short
RoutePolars::calc_vheight(const RouteLink &link) const
{
  return iround(polar_glide.get_point(link.polar_index).gradient * link.d);
}

bool
RoutePolars::check_clearance(const RouteLink &e, const RasterMap* map,
                             const TaskProjection &proj,
                             RoutePoint& inp) const
{
  if (!config.terrain_enabled())
    return true;

  GeoPoint int_x;
  short int_h;
  GeoPoint start = proj.unproject(e.first);
  GeoPoint dest = proj.unproject(e.second);

  assert(map);

  if (!map->FirstIntersection(start, e.first.altitude,
                             dest, e.second.altitude,
                             calc_vheight(e), climb_ceiling,
                             (short)config.safety_height_terrain,
                             int_x, int_h))
    return true;

  inp = RoutePoint(proj.project(int_x), int_h);
  return false;
}




RouteLink
RoutePolars::generate_intermediate (const RoutePoint& _dest,
                                    const RoutePoint& _origin,
                                    const TaskProjection& proj) const
{
  RouteLink link(_dest, _origin, proj);
  const short vh = calc_vheight(link)+_dest.altitude;
  if (can_climb())
    link.second.altitude = std::max(_dest.altitude, std::min(vh, cruise_altitude));
  else
    link.second.altitude = vh;
  return link;
}

RouteLink
RoutePolars::neighbour_link(const RoutePoint &start,
                            const RoutePoint &end,
                            const TaskProjection &proj,
                            const int sign) const
{
  const FlatGeoPoint d = end-start;

  // table of rotations for different maximum lengths.  these are calculated so
  // there is sufficient rotation as lengths get small for deltas to not
  // disappear with rounding.

  // rotation matrix is [c -s
  //                     s c]

  // Table calculations:
  // sina = 256/maxv
  // a = asin(sina/256)
  // cosa = 256*cos(a)

  static const int sina[] =
    {256, 128, 85, 64, 51, 43, 37, 32, 28, 26, 23, 21, 20, 18 };
  static const int cosa[] =
    {256, 222, 241, 248, 251, 252, 253, 254, 254, 255, 255, 255, 255, 255 };

  const int index = std::min((int)8, std::max(abs(d.Longitude), abs(d.Latitude))-1);

  FlatGeoPoint dr((d.Longitude * cosa[index] - d.Latitude * sina[index] * sign)>>8,
                  (d.Longitude * sina[index] * sign + d.Latitude * cosa[index])>>8);
  RoutePoint pd(start+dr,
                start.altitude);
  pd.round_location();
  return generate_intermediate(start, pd, proj);
}


void
AFlatGeoPoint::round_location()
{
  // round point to correspond roughly with terrain step size
  Longitude = (Longitude>>2)<<2;
  Latitude = (Latitude>>2)<<2;
}

bool
RoutePolars::achievable(const RouteLink& link, const bool check_ceiling) const
{
  if (can_climb())
    return true;
  if (check_ceiling && config.use_ceiling &&
      (link.second.altitude > climb_ceiling))
    return false;
  return (link.second.altitude <= cruise_altitude)
    && (link.second.altitude-link.first.altitude >= calc_vheight(link));
}

RouteLink
RouteLink::flat() const
{
  RouteLink copy(*this);
  copy.second.altitude = copy.first.altitude;
  return copy;
}

#define ROUTE_MIN_STEP 3

bool
RouteLinkBase::is_short() const
{
  return (abs(first.Longitude-second.Longitude)<ROUTE_MIN_STEP)
    && (abs(first.Latitude-second.Latitude)<ROUTE_MIN_STEP);
}

void
RoutePolars::set_config(const RoutePlannerConfig& _config,
                        const short _cruise_alt,
                        const short _ceiling_alt)
{
  config = _config;

  cruise_altitude = _cruise_alt;
  if (config.use_ceiling) {
    climb_ceiling = std::max(_ceiling_alt, cruise_altitude);
  } else {
    climb_ceiling = SHRT_MAX;
  }
}

bool
RoutePolars::can_climb() const {
  return config.allow_climb && positive(inv_M);
}

bool
RoutePolars::intersection(const AGeoPoint& origin,
                          const AGeoPoint& destination,
                          const RasterMap* map,
                          const TaskProjection& proj,
                          GeoPoint& intx) const
{
  if (!map || !map->isMapLoaded())
    return false;

  RouteLink e(RoutePoint(proj.project(destination), destination.altitude),
              RoutePoint(proj.project(origin), origin.altitude), proj);
  if (!positive(e.d)) return false;

  const short h_diff = origin.altitude-destination.altitude;

  if (h_diff <= 0) {
    // assume gradual climb to destination
    intx = map->Intersection(origin, origin.altitude-(short)config.safety_height_terrain,
                            h_diff, destination);
    return !(intx == destination);
  }

  const short vh = calc_vheight(e);

  if (h_diff > vh) {
    // have excess height to glide, scan pure glide, will arrive at destination high

    intx = map->Intersection(origin,
                             origin.altitude-(short)config.safety_height_terrain,
                             vh, destination);
    return !(intx == destination);
  }

  // mixed cruise-climb then glide segments, do separate searches for each

  // proportion of flight as glide
  const fixed p = fixed(h_diff)/fixed(vh);

  // location of start of glide
  const GeoPoint p_glide = destination.interpolate(origin, p);

  // intersects during cruise-climb?
  intx = map->Intersection(origin, origin.altitude-(short)config.safety_height_terrain,
                           0, destination);
  if (!(intx == destination))
    return true;

  // intersects during glide?
  intx = map->Intersection(p_glide, origin.altitude-(short)config.safety_height_terrain,
                           h_diff, destination);
  return !(intx == destination);
}
