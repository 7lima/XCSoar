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

#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include "Util/tstring.hpp"
#include "Navigation/GeoPoint.hpp"

#ifdef DO_PRINT
#include <iostream>
#endif

class TaskProjection;

/** 
 * Bitfield structure for Waypoint capabilities
 * Several of these capabilities are not used by XCSoar, but are
 * present for compatibility
 */
struct WaypointFlags {
  /** If waypoint is an airport/airfield */
  bool Airport:1;
  /** If waypoint can be used as a turnpoint */
  bool TurnPoint:1;
  /** If waypoint can be landed at although not an airport */
  bool LandPoint:1;
  /** If waypoint is to be used as home */
  bool Home:1;
  /** If waypoint is marked as a potential start point */
  bool StartPoint:1;
  /** If waypoint is marked as a potential finish point */
  bool FinishPoint:1;
  /** If waypoint is marked for restricted access (unused?) */
  bool Restricted:1;
  bool WaypointFlag:1;

  /**
   * Set default flags (all off except turnpoint)
   *
   * @param turnpoint Whether the waypoint is a turnpoint
   */
  void setDefaultFlags(bool turnpoint);
};

/**
 * Class for waypoints.  
 * This is small enough currently to be used with local copies (e.g. in a TaskWayPoint),
 * but this may change if we include airfield details inside.
 *
 * @todo
 * - consider having a static factory method provide the ID automatically
 *   so we know they will be unique.
 */
class Waypoint {
public:
  friend class Serialiser;

  /**
   * Constructor for real waypoints
   *
   * @param is_turnpoint Whether newly created waypoint is a turnpoint
   * @return Uninitialised object
   */
  Waypoint(const GeoPoint &_location, const bool is_turnpoint = false);

  /** Unique id */
  unsigned id;

  /**
   * The id number as specified in the input file.
   */
  unsigned original_id;

  /** Geodetic location */
  GeoPoint Location;
  /** Height AMSL (m) of waypoint terrain */
  fixed Altitude;
  /** Main runway direction in degrees (0-359, -1 unknown) */
  Angle RunwayDirection;
  /** Main runway length in m (0 for unknown) */
  int RunwayLength;
  /** Flag types of this waypoint */
  WaypointFlags Flags;
  /** File number to store waypoint in (0,1), -1 to delete/ignore */
  int FileNum;
  /** Name of waypoint */
  tstring Name;
  /** Additional comment text for waypoint */
  tstring Comment;
  /** Airfield or additional (long) details */
  tstring Details;

  /** 
   * Determine if waypoint is marked as able to be landed at
   * 
   * @return True if waypoint is landable
   */
  bool
  is_landable() const
  {
    return Flags.LandPoint || Flags.Airport;
  }

  /**
   * Determine if waypoint is marked as an airport
   *
   * @return True if waypoint is landable
   */
  bool
  is_airport() const
  {
    return Flags.Airport;
  }

  /**
   * Determine if waypoint is marked as a turnpoint
   *
   * @return True if waypoint is landable
   */
  bool
  is_turnpoint() const
  {
    return Flags.TurnPoint;
  }

  /**
   * Equality operator (by id)
   * 
   * @param wp Waypoint object to match against
   *
   * @return true if ids match
   */
  bool
  operator==(const Waypoint&wp) const
  {
    return id == wp.id;
  }

  /**
   * Determine if a waypoint is close to a given location within
   * a threshold
   *
   * @param location Location to compare to
   * @param range Distance threshold (m)
   *
   * @return True if close to reference location
   */
  bool
  is_close_to(const GeoPoint &location, const fixed range) const;

public:
#ifdef DO_PRINT
  friend std::ostream& operator<< (std::ostream& o, const Waypoint& wp);
#endif
};

#endif
