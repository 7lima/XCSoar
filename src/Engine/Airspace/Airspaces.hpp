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
#ifndef AIRSPACES_HPP
#define AIRSPACES_HPP

#include "AirspacesInterface.hpp"
#include "AirspacePredicate.hpp"
#include "Util/NonCopyable.hpp"
#include "Navigation/TaskProjection.hpp"
#include <deque>

class RasterTerrain;
class AtmosphericPressure;
class AirspaceVisitor;
class AirspaceIntersectionVisitor;

/**
 * Container for airspaces using kd-tree representation internally for fast 
 * geospatial lookups.
 */

class Airspaces:
  public AirspacesInterface,
  private NonCopyable
{
public:
  /** 
   * Constructor.
   * Note this class can't safely be copied (yet)
   * 
   * @return empty Airspaces class.
   */
  Airspaces():m_QNH(0)
    {
    };

/** 
 * Destructor.
 * This also destroys Airspace objects contained in the tree or temporary buffer
 * 
 */
  ~Airspaces();

  /** 
   * Add airspace to the internal airspace tree.  
   * The airspace is not copied; ownership is transferred to this class.
   * 
   * @param asp New airspace to be added.
   */
  void insert(AbstractAirspace* asp);

  /** 
   * Re-organise the internal airspace tree after inserting/deleting.
   * Should be called after inserting/deleting airspaces prior to performing
   * any searches, but can be done once after a batch insert/delete.
   */
  void optimise();

/** 
 * Clear the airspace store
 * 
 */
  void clear();

/** 
 * Size of airspace (in tree, not in temporary store) ---
 * must call optimise() before this for it to be accurate.
 * 
 * @return Number of airspaces in tree
 */
  unsigned size() const;

/** 
 * Whether airspace store is empty
 * 
 * @return True if no airspace stored
 */
  bool empty() const;

  /** 
   * Set terrain altitude for all AGL-referenced airspace altitudes 
   * 
   * @param terrain Terrain model for lookup
   */
  void set_ground_levels(const RasterTerrain &terrain);

  /** 
   * Set QNH pressure for all FL-referenced airspace altitudes.
   * Doesn't do anything if QNH is unchanged
   * 
   * @param press Atmospheric pressure model and QNH
   */
  void set_flight_levels(const AtmosphericPressure &press);

  /** 
   * Call visitor class on airspaces within range of location.
   * Note that the visitor is not instantiated separately for each match
   * 
   * @param loc location of origin of search
   * @param range distance in meters of search radius
   * @param visitor visitor class to call on airspaces within range
   */
  void visit_within_range(const GeoPoint &loc, 
                          const fixed range,
                          AirspaceVisitor& visitor,
                          const AirspacePredicate &predicate
                          =AirspacePredicate::always_true) const;

  /** 
   * Call visitor class on airspaces intersected by vector.
   * Note that the visitor is not instantiated separately for each match
   * 
   * @param loc location of origin of search
   * @param vec vector of line along with to search for intersections
   * @param visitor visitor class to call on airspaces intersected by line
   */
  void visit_intersecting(const GeoPoint &loc, 
                          const GeoVector &vec,
                          AirspaceIntersectionVisitor& visitor) const;

  /** 
   * Search for airspaces within range of the aircraft.
   * 
   * @param location location of aircraft, from which to search
   * @param range distance in meters of search radius
   * @param condition condition to be applied to matches
   * 
   * @return vector of airspaces intersecting search radius
   */
  const AirspaceVector scan_range(const GeoPoint &location,
                                  const fixed range,
                                  const AirspacePredicate &condition
                                  =AirspacePredicate::always_true) const;

  /** 
   * Search for airspaces nearest to the aircraft.
   * 
   * @param location location of aircraft, from which to search
   * @param condition condition to be applied to matches
   * 
   * @return single nearest airspace if external, or all airspaces enclosing the aircraft
   */
  const AirspaceVector scan_nearest(const GeoPoint &location,
                                    const AirspacePredicate &condition
                                    =AirspacePredicate::always_true) const;

  /** 
   * Find airspaces the aircraft is inside (taking altitude into account)
   * 
   * @param state state of aircraft for which to search
   * @param condition condition to be applied to matches
   * 
   * @return airspaces enclosing the aircraft
   */
  const AirspaceVector find_inside(const AIRCRAFT_STATE &state,
                                   const AirspacePredicate &condition
                                   =AirspacePredicate::always_true) const;

/** 
 * Access first airspace in store, for use in iterators.
 * 
 * @return First airspace in store
 */
  AirspaceTree::const_iterator begin() const;

/** 
 * Access end airspace in store, for use in iterators as end point.
 * 
 * @return End airspace in store
 */
  AirspaceTree::const_iterator end() const;

  void lock() const {};
  void unlock() const {};

private:

  fixed m_QNH;

  AirspaceTree airspace_tree;
  TaskProjection task_projection;

  std::deque< AbstractAirspace* > tmp_as;

};

#endif
