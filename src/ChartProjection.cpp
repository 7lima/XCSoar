/*
Copyright_License {

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

#include "ChartProjection.hpp"
#include "Engine/Task/TaskManager.hpp"

ChartProjection::ChartProjection(const RECT &rc,
                                 const TaskManager &task,
                                 const GEOPOINT &fallback_loc)
{
  const GEOPOINT center = task.get_task_center(fallback_loc);
  const fixed radius = max(fixed(10000), task.get_task_radius(fallback_loc));
  set_projection(rc, center, radius);
}

ChartProjection::ChartProjection(const RECT &rc,
                                 const OrderedTask& task,
                                 const GEOPOINT &fallback_loc) 
{
  const GEOPOINT center = task.get_task_center(fallback_loc);
  const fixed radius = max(fixed(10000), task.get_task_radius(fallback_loc));
  set_projection(rc, center, radius);
}

ChartProjection::ChartProjection(const RECT &rc,
                                 const TracePointVector& trace,
                                 const GEOPOINT &fallback_loc) 
{
  const TaskProjection proj = get_bounds(trace, fallback_loc);
  const GEOPOINT center = proj.get_center();
  const fixed radius = max(fixed(10000), proj.get_radius());
  set_projection(rc, center, radius);
}

void ChartProjection::set_projection(const RECT &rc, 
                                     const GEOPOINT &center,
                                     const fixed radius)
{
  SetScaleMetersToScreen(fixed(max_dimension(rc)) / (radius * 2));
  PanLocation = center;
  MapRect = rc;
  Orig_Screen.x = (rc.left + rc.right)/2;
  Orig_Screen.y = (rc.bottom + rc.top)/2;
  UpdateScreenBounds();
}

ChartProjection::ChartProjection(const RECT &rc,
                                 const OrderedTaskPoint& point,
                                 const GEOPOINT &fallback_loc)
{
  TaskProjection task_projection;
  task_projection.reset(fallback_loc);
  point.scan_projection(task_projection);

  const GEOPOINT center = task_projection.get_center();
  const fixed radius = max(fixed(10000), task_projection.get_radius());
  set_projection(rc, center, radius * fixed(1.3));
}
