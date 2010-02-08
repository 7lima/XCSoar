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

#include "TaskLeg.hpp"
#include "OrderedTaskPoint.hpp"
#include <assert.h>
#include <algorithm>


TaskLeg::TaskLeg(OrderedTaskPoint &_destination):
    vector_travelled(fixed_zero, fixed_zero),
    vector_remaining(fixed_zero, fixed_zero),
    vector_planned(fixed_zero, fixed_zero),
    destination(_destination)
{

}

const OrderedTaskPoint* 
TaskLeg::origin() const
{
  return destination.get_previous();
}

OrderedTaskPoint* 
TaskLeg::next() const
{
  return destination.get_next();
}

GeoVector 
TaskLeg::leg_vector_planned() const
{
  if (!origin()) {
    return GeoVector(fixed_zero);
  } else {
    return memo_planned.calc(origin()->get_location_remaining(), 
                             destination.get_location_remaining());
  }
}


GeoVector 
TaskLeg::leg_vector_remaining(const GEOPOINT &ref) const
{
  switch (destination.getActiveState()) {
  case OrderedTaskPoint::AFTER_ACTIVE:
    if (!origin()) {
      return GeoVector(fixed_zero);
    }
    // this leg totally included
    return memo_remaining.calc(origin()->get_location_remaining(), 
                               destination.get_location_remaining());
    break;
  case OrderedTaskPoint::CURRENT_ACTIVE:
    if (!origin()) {
      return GeoVector(fixed_zero, 
                       ref.bearing(destination.get_location_remaining()));
    }
    // this leg partially included
    return memo_remaining.calc(ref, 
                               destination.get_location_remaining());
    break;
  case OrderedTaskPoint::BEFORE_ACTIVE:
    // this leg not included
  default:
    assert(1); // error!
    return GeoVector(fixed_zero);
  };
}


GeoVector
TaskLeg::leg_vector_travelled(const GEOPOINT &ref) const
{
  switch (destination.getActiveState()) {
  case OrderedTaskPoint::BEFORE_ACTIVE:
    if (!origin()) {
      return GeoVector(fixed_zero);
    }
    // this leg totally included
    return memo_travelled.calc(origin()->get_location_travelled(), 
                               destination.get_location_travelled());
    break;
  case OrderedTaskPoint::CURRENT_ACTIVE:
    // this leg partially included
    if (!origin()) {
      return GeoVector(fixed_zero, 
                       ref.bearing(destination.get_location_remaining()));
    }
    if (destination.has_entered()) {
      return memo_travelled.calc(origin()->get_location_travelled(), 
                                 destination.get_location_travelled());
    } else {
      return memo_travelled.calc(origin()->get_location_travelled(), 
                                 ref);
    }
    break;
  case OrderedTaskPoint::AFTER_ACTIVE:
    if (!origin()) {
      return GeoVector(fixed_zero);
    }
    // this leg may be partially included
    if (origin()->has_entered()) {
      return memo_travelled.calc(origin()->get_location_travelled(), 
                                 ref);
    }
  default:
    return GeoVector(fixed_zero);
  };
}


fixed 
TaskLeg::leg_distance_scored(const GEOPOINT &ref) const
{
  if (!origin()) {
    return fixed_zero;
  }

  switch (destination.getActiveState()) {
  case OrderedTaskPoint::BEFORE_ACTIVE:
    // this leg totally included
    return 
      max(fixed_zero,
               origin()->get_location_scored().distance(
                 destination.get_location_scored())
               -origin()->score_adjustment()-destination.score_adjustment());
    break;
  case OrderedTaskPoint::CURRENT_ACTIVE:
    // this leg partially included
    if (destination.has_entered()) {
      max(fixed_zero,
               origin()->get_location_scored().distance( 
                 destination.get_location_scored())
               -origin()->score_adjustment()-destination.score_adjustment());
    } else {
      return 
        max(fixed_zero,
                 ref.projected_distance(origin()->get_location_scored(), 
                                        destination.get_location_scored())
                 -origin()->score_adjustment());
    }
    break;
  case OrderedTaskPoint::AFTER_ACTIVE:
    // this leg may be partially included
    if (origin()->has_entered()) {
      return max(fixed_zero,
                      memo_travelled.calc(origin()->get_location_scored(), 
                                          ref).Distance
                      -origin()->score_adjustment());
    }
  default:
    return fixed_zero;
    break;
  };
  return fixed_zero;
}


fixed 
TaskLeg::leg_distance_nominal() const
{
  if (origin()) {
    return memo_nominal.Distance(origin()->get_location(), 
                                 destination.get_location());
  } else {
    return fixed_zero; 
  }
}


fixed 
TaskLeg::leg_distance_max() const
{
  if (origin()) {
    return memo_max.Distance(origin()->get_location_max(), 
                             destination.get_location_max());
  } else {
    return fixed_zero; 
  }
}


fixed 
TaskLeg::leg_distance_min() const
{
  if (origin()) {
    return memo_min.Distance(origin()->get_location_min(), 
                             destination.get_location_min());
  } else {
    return fixed_zero; 
  }
}

fixed 
TaskLeg::scan_distance_travelled(const GEOPOINT &ref) 
{
  vector_travelled = leg_vector_travelled(ref);
  return vector_travelled.Distance 
    +(next()? next()->scan_distance_travelled(ref):fixed_zero);
}


fixed 
TaskLeg::scan_distance_remaining(const GEOPOINT &ref) 
{
  vector_remaining = leg_vector_remaining(ref);
  return vector_remaining.Distance 
    +(next()? next()->scan_distance_remaining(ref):fixed_zero);
}


fixed 
TaskLeg::scan_distance_planned() 
{
  vector_planned = leg_vector_planned();
  return vector_planned.Distance 
    +(next()? next()->scan_distance_planned():fixed_zero);
}


fixed 
TaskLeg::scan_distance_max() const
{
  return leg_distance_max()
    +(next()? next()->scan_distance_max():fixed_zero);
}


fixed 
TaskLeg::scan_distance_min() const
{
  return leg_distance_min()
    +(next()? next()->scan_distance_min():fixed_zero);
}


fixed 
TaskLeg::scan_distance_nominal() const
{
  return leg_distance_nominal()
    +(next()? next()->scan_distance_nominal():fixed_zero);
}

fixed 
TaskLeg::scan_distance_scored(const GEOPOINT &ref) const
{
  return leg_distance_scored(ref)
    +(next()? next()->scan_distance_scored(ref):fixed_zero);
}
