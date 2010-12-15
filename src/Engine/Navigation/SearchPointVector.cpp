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
#include "SearchPointVector.hpp"
#include "Navigation/ConvexHull/GrahamScan.hpp"
#include "Navigation/Flat/FlatRay.hpp"
#include "Navigation/Flat/FlatBoundingBox.hpp"
#include <algorithm>
#include <functional>

bool 
prune_interior(SearchPointVector& spv)
{
  GrahamScan gs(spv);
  return gs.prune_interior();
}

bool
thin_to_size(SearchPointVector& spv, const unsigned max_size)
{
  const fixed tolerance = fixed(1.0e-8);
  unsigned i=2;
  bool retval = false;
  while (spv.size()> max_size) {
    GrahamScan gs(spv, tolerance*i);
    retval|= gs.prune_interior();
    i*= i;
  }
  return retval;
}

bool 
is_convex(const SearchPointVector& spv)
{
  SearchPointVector copy = spv;
  GrahamScan gs(copy);
  return !gs.prune_interior();
}

void 
project(SearchPointVector& spv, const TaskProjection& tp)
{
  for (SearchPointVector::iterator i = spv.begin(); i!= spv.end(); ++i) {
    i->project(tp);
  }
}

static FlatGeoPoint
nearest_point(const FlatGeoPoint &p1, const FlatGeoPoint &p2,
              const FlatGeoPoint &p3)
{
  const FlatGeoPoint p12 = p2-p1;
  const fixed rsq(p12.dot(p12));
  if (!positive(rsq)) {
    return p1;
  }
  const FlatGeoPoint p13 = p3-p1;
  const fixed numerator(p13.dot(p12));
  
  if (!positive(numerator)) {
    return p1;
  } else if (numerator>= rsq) {
    return p2;
  } else {
    fixed t = numerator/rsq;
    return p1+(p2-p1)*t;
  }
}

static FlatGeoPoint
segment_nearest_point(const SearchPointVector& spv,
                      const SearchPointVector::const_iterator i1,
                      const FlatGeoPoint &p3)
{
  if (i1+1 == spv.end()) {
    return nearest_point(i1->get_flatLocation(),
                         spv.begin()->get_flatLocation(),
                         p3);
  } else {
    return nearest_point(i1->get_flatLocation(),
                         (i1+1)->get_flatLocation(),
                         p3);
  }
}

static FlatGeoPoint
nearest_point_nonconvex(const SearchPointVector& spv, const FlatGeoPoint &p3)
{
  unsigned distance_min = 0-1;
  SearchPointVector::const_iterator i_best = spv.end();
  for (SearchPointVector::const_iterator i = spv.begin(); 
       i!= spv.end(); ++i) {

    FlatGeoPoint pa = segment_nearest_point(spv,i,p3);
    unsigned d_this = p3.distance_sq_to(pa);
    if (d_this<distance_min) {
      distance_min = d_this;
      i_best = i;
    }
  }
  return i_best->get_flatLocation();
}

SearchPointVector::const_iterator
nearest_index_convex(const SearchPointVector& spv, const FlatGeoPoint &p3)
{
  unsigned distance_min = 0-1;

  SearchPointVector::const_iterator i_best = spv.end();

  // find nearest point in vector
  for (SearchPointVector::const_iterator i = spv.begin(); 
       i!= spv.end(); ++i) {

    unsigned d_this = p3.distance_sq_to(i->get_flatLocation());
    if (d_this<distance_min) {
      distance_min = d_this;
      i_best = i;
    }
  }
  return i_best;
}


static
FlatGeoPoint
nearest_point_convex(const SearchPointVector& spv, const FlatGeoPoint &p3)
{
  unsigned distance_min = 0-1;

  SearchPointVector::const_iterator i_best =
    nearest_index_convex(spv, p3);

  FlatGeoPoint pc = i_best->get_flatLocation();

  // find nearest point on this segment
  FlatGeoPoint pa = segment_nearest_point(spv,i_best,p3);
  if (!(pa == pc)) {
    unsigned d_seg = pa.distance_sq_to(p3);
    if (d_seg < distance_min) {
      distance_min = d_seg;
      pc = pa;
    }
  }

  // find nearest point on previous segment
  SearchPointVector::const_iterator i_prev;
  if (i_best == spv.begin()) {
    i_prev = spv.end()-1;
  } else {
    i_prev = i_best-1;
  }

  FlatGeoPoint pb = segment_nearest_point(spv,i_prev,p3);
  if (!(pb == pc)) {
    unsigned d_seg = pb.distance_sq_to(p3);
    if (d_seg < distance_min) {
      distance_min = d_seg;
      pc = pb;
    }
  }

  return pc;
}

FlatGeoPoint nearest_point(const SearchPointVector& spv, 
                            const FlatGeoPoint &p3,
                            const bool is_convex)
{
  // special case
  if (spv.empty()) {
    return p3; // really should be error
  } else if (spv.size()==1) {
    return spv[0].get_flatLocation();
  }

  if (is_convex) {
    /** \todo Strictly speaking it isn't correct to use this function
     */
    return nearest_point_convex(spv,p3);
  } else {
    return nearest_point_nonconvex(spv,p3);
  }
}

bool intersects(const SearchPointVector& spv,
                const FlatRay& ray)
{
  for (SearchPointVector::const_iterator it= spv.begin();
       it+1 != spv.end(); ++it) {

    const FlatRay r_seg(it->get_flatLocation(),
                        (it+1)->get_flatLocation());

    if (r_seg.intersects_distinct(ray))
      return true;
  }
  return false;
}

FlatBoundingBox
compute_boundingbox(const SearchPointVector& spv)
{
  FlatGeoPoint fmin;
  FlatGeoPoint fmax;
  bool empty=true;
  for (SearchPointVector::const_iterator v = spv.begin();
       v != spv.end(); ++v) {
    FlatGeoPoint f = v->get_flatLocation();
    if (empty) {
      empty = false;
      fmin = f;
      fmax = f;
    } else {
      fmin.Longitude = min(fmin.Longitude, f.Longitude);
      fmin.Latitude = min(fmin.Latitude, f.Latitude);
      fmax.Longitude = max(fmax.Longitude, f.Longitude);
      fmax.Latitude = max(fmax.Latitude, f.Latitude);
    }
  }
  if (!empty) {
    // note +/- 1 to ensure rounding keeps bb valid
    fmin.Longitude-= 1; fmin.Latitude-= 1;
    fmax.Longitude+= 1; fmax.Latitude+= 1;
    return FlatBoundingBox(fmin,fmax);
  } else {
    return FlatBoundingBox(FlatGeoPoint(0,0),FlatGeoPoint(0,0));
  }
}


void
circular_next(SearchPointVector::const_iterator &i, const SearchPointVector& spv)
{
  i++;
  if (i==spv.end())
    i= spv.begin();
}

void
circular_previous(SearchPointVector::const_iterator &i, const SearchPointVector& spv)
{
  if (i== spv.begin()) {
    i = spv.begin()+spv.size()-1;
  } else {
    i--;
  }
}
