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
#ifndef WAYPOINTENVELOPE_HPP
#define WAYPOINTENVELOPE_HPP

#include "Waypoint.hpp"
#include "Navigation/Flat/FlatGeoPoint.hpp"

class TaskProjection;

/**
 * Waypoint envelope class for KDTree storage management (envelope-letter pattern)
 */
class WaypointEnvelope
{
public:

/** 
 * Constructor for real waypoints.  Makes a copy of the waypoint for storage.
 * 
 * @return Uninitialised object
 */
  WaypointEnvelope(const Waypoint& wp):waypoint(wp) {};

/** 
 * Constructor for virtual waypoint (used for lookups by kd-tree)
 * 
 * @param location Location of virtual waypoint
 * @param task_projection Projection to apply to flat location
 *
 * @return Initialised (virtual) object.  Not to be added to actual Waypoints class
 */
  WaypointEnvelope(const GeoPoint &location,
    const TaskProjection &task_projection);

  /** 
   * Project geolocation to flat location
   * 
   * @param task_projection Projection to apply
   */
  void project(const TaskProjection& task_projection);

/** 
 * Get distance in internal flat projected units (fast)
 * 
 * @param f Point to get distance to
 * 
 * @return Distance in flat units
 */
  unsigned flat_distance_to(const FlatGeoPoint &f) const {
    return FlatLocation.distance_to(f);
  }

/** 
 * Accessor for waypoint in the envelope (the letter!)
 * @return Reference to waypoint in the envelope
 */
  const Waypoint& get_waypoint() const {
    return waypoint;
  }

/**
 *  Set airfield details
 * @param Details text of airfield details
 */
  void set_details(const tstring& Details) const {
    waypoint.Details = Details;
  }

/** 
 * Set/clear home flag of waypoint
 * @param set True/false
 */
  void set_home(const bool set) const {
    waypoint.Flags.Home = set;
  }

/** 
 * Operator to give access to letter
 * 
 * @return Reference to waypoint
 */
  const Waypoint& operator()(const WaypointEnvelope& en) {
    return en.waypoint;
  }

public:
  /**
   * Function object used to provide access to coordinate values by kd-tree
   */
  struct kd_get_location {    
    typedef int result_type; /**< type of returned value */
    /**
     * Retrieve coordinate value from object given coordinate index
     * @param d WaypointEnvelope object
     * @param k index of coordinate
     *
     * @return Coordinate value
     */
    int operator() ( const WaypointEnvelope &d, const unsigned k) const {
      switch(k) {
      case 0:
        return d.FlatLocation.Longitude;
      case 1:
        return d.FlatLocation.Latitude;
      };
      return 0; 
    };
  };

  /**
   * Equality operator (by id)
   * 
   * @param wpe Waypoint object to match against
   *
   * @return true if ids match
   */
  bool operator==(const WaypointEnvelope &wpe) const {
    return waypoint == wpe.waypoint;
  }

private:
  FlatGeoPoint FlatLocation; /**< Flat projected location */
  mutable Waypoint waypoint; /**< Actual waypoint contained */
};


#endif
