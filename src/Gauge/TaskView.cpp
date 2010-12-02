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

#include "Gauge/TaskView.hpp"
#include "Screen/Canvas.hpp"
#include "ChartProjection.hpp"
#include "BackgroundDrawHelper.hpp"
#include "RenderObservationZone.hpp"
#include "RenderTaskPoint.hpp"
#include "RenderTask.hpp"
#include "Engine/Task/Tasks/BaseTask/OrderedTaskPoint.hpp"
#include "Engine/Task/Tasks/OrderedTask.hpp"

static void
PaintTask(Canvas &canvas, const WindowProjection &projection,
          const OrderedTask &task,
          const GeoPoint &location, const SETTINGS_MAP &settings_map,
          const RasterTerrain *terrain)
{
  BackgroundDrawHelper background;
  background.set_terrain(terrain);
  background.Draw(canvas, projection, settings_map);

  RenderObservationZone ozv;
  RenderTaskPoint tpv(canvas, projection, settings_map,
                      task.get_task_projection(),
                      ozv, false, location);
  RenderTask dv(tpv);
  dv.Visit(task);
}

void
PaintTask(Canvas &canvas, const RECT &rc, const OrderedTask &task,
          const GeoPoint &location, const SETTINGS_MAP &settings_map,
          const RasterTerrain *terrain)
{
  ChartProjection projection(rc, task, location);
  PaintTask(canvas, projection, task, location, settings_map, terrain);
}

void
PaintTaskPoint(Canvas &canvas, const RECT &rc,
               const OrderedTask &task, const OrderedTaskPoint &point,
               const GeoPoint &location, const SETTINGS_MAP &settings_map,
               const RasterTerrain *terrain)
{
  ChartProjection projection(rc, point, point.get_location());
  PaintTask(canvas, projection, task, location, settings_map, terrain);
}
