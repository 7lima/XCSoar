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
#include "Airspaces.hpp"
#include "AirspaceVisitor.hpp"
#include "AirspaceIntersectionVisitor.hpp"
#include "Atmosphere/Pressure.hpp"

#ifdef INSTRUMENT_TASK
extern unsigned n_queries;
extern long count_intersections;
#endif

class AirspacePredicateVisitorAdapter {
  const AirspacePredicate *predicate;
  AirspaceVisitor *visitor;

public:
  AirspacePredicateVisitorAdapter(const AirspacePredicate &_predicate,
                                  AirspaceVisitor &_visitor)
    :predicate(&_predicate), visitor(&_visitor) {}

  void operator()(Airspace as) {
    AbstractAirspace &aas = *as.get_airspace();
    if (predicate->condition(aas))
      visitor->Visit(as);
  }
};

void 
Airspaces::visit_within_range(const GEOPOINT &loc, 
                              const fixed range,
                              AirspaceVisitor& visitor,
                              const AirspacePredicate &predicate) const
{
  Airspace bb_target(loc, task_projection);
  int mrange = task_projection.project_range(loc, range);
  AirspacePredicateVisitorAdapter adapter(predicate, visitor);
  airspace_tree.visit_within_range(bb_target, -mrange, adapter);

#ifdef INSTRUMENT_TASK
  n_queries++;
#endif
}

class IntersectingAirspaceVisitorAdapter {
  const FlatRay *ray;
  AirspaceVisitor *visitor;

public:
  IntersectingAirspaceVisitorAdapter(const FlatRay &_ray,
                                     AirspaceVisitor &_visitor)
    :ray(&_ray), visitor(&_visitor) {}

  void operator()(Airspace as) {
    if (as.intersects(*ray))
      visitor->Visit(as);
  }
};

void 
Airspaces::visit_intersecting(const GEOPOINT &loc, 
                              const GeoVector &vec,
                              AirspaceIntersectionVisitor& visitor) const
{
  FlatRay ray(task_projection.project(loc), 
              task_projection.project(vec.end_point(loc)));

  GEOPOINT c = vec.mid_point(loc);
  Airspace bb_target(c, task_projection);
  int mrange = task_projection.project_range(c, vec.Distance / 2);
  IntersectingAirspaceVisitorAdapter adapter(ray, visitor);
  airspace_tree.visit_within_range(bb_target, -mrange, adapter);

#ifdef INSTRUMENT_TASK
  n_queries++;
#endif
}

// SCAN METHODS

const Airspaces::AirspaceVector
Airspaces::scan_nearest(const GEOPOINT location,
                        const AirspacePredicate &condition) const 
{
  Airspace bb_target(location, task_projection);

  std::pair<AirspaceTree::const_iterator, double> 
    found = airspace_tree.find_nearest(bb_target);

#ifdef INSTRUMENT_TASK
  n_queries++;
#endif

  AirspaceVector res;
  if (found.first != airspace_tree.end()) {
    // also should do scan_range with range = 0 since there
    // could be more than one with zero dist
    if (found.second == 0) {
      return scan_range(location, fixed_zero, condition);
    } else {
      if (condition(*found.first->get_airspace()))
        res.push_back(*found.first);
    }
  }

  return res;
}

const Airspaces::AirspaceVector
Airspaces::scan_range(const GEOPOINT location,
                      const fixed range,
                      const AirspacePredicate &condition) const
{
  Airspace bb_target(location, task_projection);
  int mrange = task_projection.project_range(location, range);
  
  std::deque< Airspace > vectors;
  airspace_tree.find_within_range(bb_target, -mrange, std::back_inserter(vectors));

#ifdef INSTRUMENT_TASK
  n_queries++;
#endif

  AirspaceVector res;

  for (std::deque<Airspace>::iterator v = vectors.begin(); v != vectors.end(); ++v) {
    if (!condition(*v->get_airspace()))
      continue;

    if (fixed((*v).distance(bb_target)) > range)
      continue;

    if ((*v).inside(location) || positive(range))
      res.push_back(*v);
  }

  return res;
}

const Airspaces::AirspaceVector
Airspaces::find_inside(const AIRCRAFT_STATE &state,
                       const AirspacePredicate &condition) const
{
  Airspace bb_target(state.Location, task_projection);

  AirspaceVector vectors;
  airspace_tree.find_within_range(bb_target, 0, std::back_inserter(vectors));

#ifdef INSTRUMENT_TASK
  n_queries++;
#endif

  for (AirspaceVector::iterator v = vectors.begin(); v != vectors.end();) {

#ifdef INSTRUMENT_TASK
    count_intersections++;
#endif
    
    if (!condition(*v->get_airspace()) || !(*v).inside(state))
      vectors.erase(v);
    else
      ++v;
  }

  return vectors;
}

void 
Airspaces::optimise()
{
  if (task_projection.update_fast()) {
    // task projection changed, so need to push items back onto stack
    // to re-build airspace envelopes

    for (AirspaceTree::iterator it = airspace_tree.begin();
         it != airspace_tree.end(); ++it)
      tmp_as.push_back(it->get_airspace());

    airspace_tree.clear();
  }

  if (!tmp_as.empty()) {
    while (!tmp_as.empty()) {
      Airspace as(*tmp_as.front(), task_projection);
      airspace_tree.insert(as);
      tmp_as.pop_front();
    }
    airspace_tree.optimise();
  }
}

void 
Airspaces::insert(AbstractAirspace* asp)
{
  if (!asp)
    // nothing to add
    return;

  if (empty())
    task_projection.reset(asp->get_center());

  task_projection.scan_location(asp->get_center());

  tmp_as.push_back(asp);
}

void
Airspaces::clear()
{
  // delete temporaries in case they were added without an optimise() call
  while (!tmp_as.empty()) {
    AbstractAirspace *aa = tmp_as.front();
    delete aa;
    tmp_as.pop_front();
  }

  // delete items in the tree
  for (AirspaceTree::iterator v = airspace_tree.begin();
       v != airspace_tree.end(); ++v) {
    Airspace a = *v;
    a.destroy();
  }

  // then delete the tree
  airspace_tree.clear();
}

unsigned
Airspaces::size() const
{
  return airspace_tree.size();
}

bool
Airspaces::empty() const
{
  return airspace_tree.empty() && tmp_as.empty();
}

Airspaces::~Airspaces()
{
  clear();
}

void 
Airspaces::set_flight_levels(const AtmosphericPressure &press)
{
  if (press.get_QNH() != m_QNH) {
    m_QNH = press.get_QNH();

    for (AirspaceTree::iterator v = airspace_tree.begin();
         v != airspace_tree.end(); ++v) {
      v->set_flight_level(press);
    }
  }
}

Airspaces::AirspaceTree::const_iterator
Airspaces::begin() const
{
  return airspace_tree.begin();
}

Airspaces::AirspaceTree::const_iterator
Airspaces::end() const
{
  return airspace_tree.end();
}
