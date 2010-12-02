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

#include "MapWindow.hpp"
#include "Math/Earth.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Util.hpp"
#include "Screen/Graphics.hpp"
#include "Task/ProtectedTaskManager.hpp"

#include <algorithm>

using std::min;
using std::max;

/**
 * This function returns the corresponding SnailTrail
 * color array index to the input
 * @param cv Input value between -1.0 and 1.0
 * @return SnailTrail color array index
 */
gcc_const
static int
GetSnailColorIndex(fixed cv)
{
  return max((short)0, min((short)(NUMSNAILCOLORS - 1),
                           (short)((cv + fixed_one) / 2 * NUMSNAILCOLORS)));
}

void
MapWindow::RenderTrail(Canvas &canvas, const RasterPoint aircraft_pos) const
{
  unsigned min_time = max(0, (int)Basic().Time - 600);
  DrawTrail(canvas, aircraft_pos, min_time);
}

void
MapWindow::DrawTrail(Canvas &canvas, const RasterPoint aircraft_pos,
                     unsigned min_time, bool enable_traildrift) const
{
  if (!SettingsMap().TrailActive || task == NULL)
    return;

  const MapWindowProjection &projection = render_projection;

  TracePointVector trace =
    task->find_trace_points(projection.GetGeoLocation(),
                            projection.GetScreenDistanceMeters(),
                            min_time, projection.DistancePixelsToMeters(3));

  if (trace.empty())
    return;

  GeoPoint traildrift(Angle::native(fixed_zero), Angle::native(fixed_zero));
  if (enable_traildrift) {
    GeoPoint tp1 = FindLatitudeLongitude(Basic().Location, Basic().wind.bearing,
                                         Basic().wind.norm);
    traildrift = Basic().Location - tp1;
  }

  fixed value_max, value_min;

  if (settings_map.SnailType == stAltitude) {
    value_max = fixed(1000);
    value_min = fixed(500);
    for (TracePointVector::const_iterator it = trace.begin();
         it != trace.end(); ++it) {
      value_max = max(it->NavAltitude, value_max);
      value_min = min(it->NavAltitude, value_min);
    }
  } else {
    value_max = fixed(0.75);
    value_min = fixed(-2.0);
    for (TracePointVector::const_iterator it = trace.begin();
         it != trace.end(); ++it) {
      value_max = max(it->NettoVario, value_max);
      value_min = min(it->NettoVario, value_min);
    }
    value_max = min(fixed(7.5), value_max);
    value_min = max(fixed(-5.0), value_min);
  }

  unsigned last_time = 0;
  RasterPoint last_point;
  for (TracePointVector::const_iterator it = trace.begin();
       it != trace.end(); ++it) {
    const fixed dt = Basic().Time - fixed(it->time);
    RasterPoint pt = projection.GeoToScreen(it->get_location().
        parametric(traildrift, dt * it->drift_factor));

    if (it->last_time == last_time) {
      if (settings_map.SnailType == stAltitude) {
        int index = (it->NavAltitude - value_min) / (value_max - value_min) *
                    (NUMSNAILCOLORS - 1);
        index = max(0, min(NUMSNAILCOLORS - 1, index));
        canvas.select(Graphics::hpSnailVario[index]);
      } else {
        const fixed colour_vario = negative(it->NettoVario) ?
                                   - it->NettoVario / value_min :
                                   it->NettoVario / value_max ;

        canvas.select(Graphics::hpSnailVario[GetSnailColorIndex(colour_vario)]);
      }
      canvas.line(last_point, pt);
    }
    last_time = it->time;
    last_point = pt;
  }

  canvas.line(last_point, aircraft_pos);
}
