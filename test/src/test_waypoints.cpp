/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#include "harness_waypoints.hpp"
#include "test_debug.hpp"

#include "Waypoint/WaypointVisitor.hpp"

#include <stdio.h>
#include <tchar.h>

class WaypointVisitorPrint: public WaypointVisitor {
public:
  WaypointVisitorPrint():count(0) {};

  virtual void Visit(const Waypoint& wp) {
    if (verbose) {
      _tprintf(_T("# visiting wp %d, '%s'\n"), wp.id, wp.name.c_str());
    }
    count++;
  }
  unsigned count;
  void reset() {
    count = 0;
  }
};

static unsigned
test_location(const Waypoints& waypoints, bool good)
{
  GeoPoint loc(Angle::Zero(), Angle::Zero());
  if (!good) {
    loc.longitude = Angle::Degrees(fixed(-23.4));
  }
  const Waypoint *r = waypoints.LookupLocation(loc);
  if (r) {
    WaypointVisitorPrint v;
    v.Visit(*r);
    return good;
  } else {
    return !good;
  }
}


static unsigned
test_range(const Waypoints& waypoints, const double range)
{
  const Waypoint *r = waypoints.LookupId(3);
  if (r) {
    WaypointVisitorPrint v;
    waypoints.VisitWithinRange(r->location, fixed(range), v);
    return v.count;
  } else {
    return 0;
  }
}

static bool
test_nearest(const Waypoints& waypoints)
{
  const Waypoint *r = waypoints.LookupId(3);
  if (!r)
    return false;

  r = waypoints.GetNearest(r->location, fixed_zero);
  if (!r)
    return false;

  return r->id == 3;
}

static bool
test_nearest_landable(const Waypoints& waypoints)
{
  const Waypoint *r = waypoints.GetNearestLandable(GeoPoint(Angle::Degrees(fixed(0.99)),
                                                              Angle::Degrees(fixed(1.1))),
                                                     fixed(50000));
  if (!r)
    return false;

  return r->id == 3;
}

static unsigned
test_copy(Waypoints& waypoints)
{
  const Waypoint *r = waypoints.LookupId(5);
  if (!r) {
    return false;
  }
  unsigned size_old = waypoints.size();
  Waypoint wp = *r;
  wp.id = waypoints.size()+1;
  waypoints.Append(wp);
  waypoints.Optimise();
  unsigned size_new = waypoints.size();
  return (size_new == size_old+1);
}

static bool
test_lookup(const Waypoints& waypoints, unsigned id)
{
  const Waypoint* wp;
  wp = waypoints.LookupId(id);
  if (wp== NULL) {
    return false;
  }
  return true;
}

static bool
test_erase(Waypoints& waypoints, unsigned id)
{
  waypoints.Optimise();
  const Waypoint* wp;
  wp = waypoints.LookupId(id);
  if (wp== NULL) {
    return false;
  }
  waypoints.Erase(*wp);
  waypoints.Optimise();

  wp = waypoints.LookupId(id);
  if (wp!= NULL) {
    return false;
  }
  return true;
}

static bool
test_replace(Waypoints& waypoints, unsigned id)
{
  const Waypoint* wp;
  wp = waypoints.LookupId(id);
  if (wp== NULL) {
    return false;
  }
  tstring oldName = wp->name;

  Waypoint copy = *wp;
  copy.name = _T("Fred");
  waypoints.Replace(*wp,copy);
  waypoints.Optimise();

  wp = waypoints.LookupId(id);
  if (wp== NULL) {
    return false;
  }
  return (wp->name != oldName) && (wp->name == _T("Fred"));
}

int main(int argc, char** argv)
{
  if (!parse_args(argc,argv)) {
    return 0;
  }

  plan_tests(14);

  Waypoints waypoints;

  ok(setup_waypoints(waypoints),"waypoint setup",0);

  unsigned size = waypoints.size();

  ok(test_lookup(waypoints,3),"waypoint lookup",0);
  ok(!test_lookup(waypoints,5000),"waypoint bad lookup",0);
  ok(test_nearest(waypoints),"waypoint nearest",0);
  ok(test_nearest_landable(waypoints),"waypoint nearest landable",0);
  ok(test_location(waypoints,true),"waypoint location good",0);
  ok(test_location(waypoints,false),"waypoint location bad",0);
  ok(test_range(waypoints,100)==1,"waypoint visit range 100m",0);
  ok(test_range(waypoints,500000)== waypoints.size(),"waypoint range 500000m",0);

  // test clear
  waypoints.Clear();
  ok(waypoints.size()==0,"waypoint clear",0);
  setup_waypoints(waypoints);
  ok(size == waypoints.size(),"waypoint setup after clear",0);

  ok(test_copy(waypoints),"waypoint copy",0);

  ok(test_erase(waypoints,3),"waypoint erase",0);
  ok(test_replace(waypoints,4),"waypoint replace",0);

  return exit_status();
}
