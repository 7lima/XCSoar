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

#include "GlueMapWindow.hpp"
#include "Message.hpp"
#include "InputEvents.h"
#include "Screen/Layout.hpp"
#include "Appearance.hpp"
#include "Defines.h"
#include "Simulator.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "DeviceBlackboard.hpp"
#include "Math/Earth.hpp"
#include "Protection.hpp"
#include "Dialogs.h"
#include "UtilsSystem.hpp"
#include "Math/FastMath.h"
#include "Compiler.h"

#include <algorithm>

using std::min;
using std::max;

bool
GlueMapWindow::on_setfocus()
{
  MapWindow::on_setfocus();

  if (InputEvents::getModeID() == InputEvents::MODE_INFOBOX)
    // the focus comes from the info box; restore the "default" mode
    InputEvents::setMode(InputEvents::MODE_DEFAULT);

  return true;
}

bool
GlueMapWindow::on_mouse_double(int x, int y)
{
  mouse_down_clock.update();
  InputEvents::ShowMenu();
  ignore_single_click = true;
  return true;
}

bool
GlueMapWindow::on_mouse_move(int x, int y, unsigned keys)
{
#ifdef OLD_TASK // target control
  if (task != NULL &&
      task->getSettings().AATEnabled &&
      SettingsMap().TargetPan &&
      (TargetDrag_State > 0)) {
    // target follows "finger" so easier to drop near edge of
    // sector
    if (TargetDrag_State == 1) {
      GeoPoint mouseMove = Screen2LonLat(x, y);
      unsigned index = SettingsMap().TargetPanIndex;
      if (task->InAATTurnSector(mouseMove, index)) {
        // update waypoints so if we drag out of the cylinder, it
        // will remain adjacent to the edge

        TASK_POINT tp = task->getTaskPoint(index);
        tp.AATTargetLocation = mouseMove;
        task->setTaskPoint(index, tp);
        TargetDrag_Location = mouseMove;

        MapGfx.hBmpTarget.draw(get_canvas(), get_bitmap_canvas(), x, y);
        return true;
      }
    }
  }
#endif

  switch (drag_mode) {
  case DRAG_NONE:
    break;

  case DRAG_PAN:
    drag_last.x = x;
    drag_last.y = y;
    invalidate();
    return true;

  case DRAG_GESTURE:
    gestures.Update(x, y);
    return true;

  case DRAG_SIMULATOR:
    return true;
  }

  return MapWindow::on_mouse_move(x, y, keys);
}

bool
GlueMapWindow::on_mouse_down(int x, int y)
{
  // Ignore single click event if double click detected
  if (ignore_single_click || drag_mode != DRAG_NONE)
    return true;

  mouse_down_clock.update();

  set_focus();

  drag_start.x = x;
  drag_start.y = y;
  drag_start_geopoint = projection.Screen2LonLat(x, y);
  drag_last = drag_start;

  if (SettingsMap().EnablePan)
    drag_mode = DRAG_PAN;
  else if (is_simulator() && !Basic().gps.Replay)
    drag_mode = DRAG_SIMULATOR;
  else if (XCSoarInterface::SettingsComputer().EnableGestures) {
    gestures.Start(x, y, Layout::Scale(20));
    drag_mode = DRAG_GESTURE;
  }

  if (drag_mode != DRAG_NONE)
    set_capture();

#ifdef OLD_TASK // target control
  if (task != NULL &&
      task->getSettings().AATEnabled &&
      SettingsMap().TargetPan) {
    if (task->ValidTaskPoint(SettingsMap().TargetPanIndex)) {
      POINT tscreen;
      LonLat2Screen(task->getTargetLocation(SettingsMap().TargetPanIndex),
                    tscreen);
      double distance = hypot(drag_start.x - tscreen.x,
                              drag_start.y - tscreen.y);
      distance /= Layout::scale;

      if (distance < 10)
        TargetDrag_State = 1;
    }
  }
#endif

  return true;
}

bool
GlueMapWindow::on_mouse_up(int x, int y)
{
  if (drag_mode != DRAG_NONE)
    release_capture();

  // Ignore single click event if double click detected
  if (ignore_single_click) {
    ignore_single_click = false;
    return true;
  }

  int click_time = mouse_down_clock.elapsed();
  mouse_down_clock.reset();

  if (SettingsMap().TargetPan) {
#ifdef OLD_TASK // target control
    if (task != NULL &&
        task->getSettings().AATEnabled &&
        TargetDrag_State > 0) {
      TargetDrag_State = 2;
      if (task->InAATTurnSector(G, SettingsMap().TargetPanIndex))
        // if release mouse out of sector, don't update w/ bad coords
        TargetDrag_Location = G;

      return true;
    }

    return false;
#endif
  }

  enum drag_mode old_drag_mode = drag_mode;
  drag_mode = DRAG_NONE;

  switch (old_drag_mode) {
  case DRAG_NONE:
    break;

  case DRAG_PAN:
    if (compare_squared(drag_start.x - x, drag_start.y - y,
                        Layout::Scale(10)) == 1) {
      const GeoPoint start = projection.Screen2LonLat(drag_start.x,
                                                      drag_start.y);
      const GeoPoint end = projection.Screen2LonLat(x, y);

      XCSoarInterface::SetSettingsMap().PanLocation.Longitude +=
        start.Longitude - end.Longitude;
      XCSoarInterface::SetSettingsMap().PanLocation.Latitude +=
        start.Latitude - end.Latitude;
      ++ui_generation;

      ActionInterface::SendSettingsMap(true);
      return true;
    }

    break;

  case DRAG_SIMULATOR:
    if (click_time > 50 &&
        compare_squared(drag_start.x - x, drag_start.y - y,
                        Layout::Scale(36)) == 1) {
      GeoPoint G = projection.Screen2LonLat(x, y);

      double distance = hypot(drag_start.x - x, drag_start.y - y);

      // This drag moves the aircraft (changes speed and direction)
      const Angle oldbearing = Basic().TrackBearing;
      const fixed minspeed = fixed(1.1) * (task != NULL ?
                                           task->get_glide_polar() :
                                           GlidePolar(fixed_zero)).get_Vmin();
      const Angle newbearing = Bearing(drag_start_geopoint, G);
      if (((newbearing - oldbearing).as_delta().magnitude_degrees() < fixed(30)) ||
          (Basic().GroundSpeed < minspeed))
        device_blackboard.SetSpeed(min(fixed(100.0),
                                       max(minspeed,
                                           fixed(distance / (3 * Layout::scale)))));

      device_blackboard.SetTrackBearing(newbearing);
      // change bearing without changing speed if direction change > 30
      // 20080815 JMW prevent dragging to stop glider

      // JMW trigger recalcs immediately
      TriggerGPSUpdate();
      return true;
    }

    break;

  case DRAG_GESTURE:
    const char* gesture = gestures.Finish();
    if (gesture && on_mouse_gesture(gesture))
      return true;

    break;
  }

  if(click_time < 1000) {
    // click less then one second -> open nearest waypoint details
    if (way_points != NULL &&
        PopupNearestWaypointDetails(*way_points, drag_start_geopoint,
                                    projection.DistancePixelsToMeters(Layout::Scale(10)),
                                    true))
      return true;
  } else {
    // click more then one second -> open nearest airspace details
    if (airspace_database != NULL &&
        AirspaceDetailsAtPoint(drag_start_geopoint))
      return true;
  }

  return false;
}

bool
GlueMapWindow::on_mouse_wheel(int delta)
{
  if (drag_mode != DRAG_NONE)
    return true;

  if (delta > 0)
    // zoom in
    InputEvents::sub_ScaleZoom(1);
  else if (delta < 0)
    // zoom out
    InputEvents::sub_ScaleZoom(-1);

  return true;
}

bool
GlueMapWindow::on_mouse_gesture(const char* gesture)
{
  if (!XCSoarInterface::SettingsComputer().EnableGestures)
    return false;

  if (strcmp(gesture, "U") == 0) {
    InputEvents::processKey(VK_UP);
    return true;
  }
  if (strcmp(gesture, "D") == 0) {
    InputEvents::processKey(VK_DOWN);
    return true;
  }
  if (strcmp(gesture, "L") == 0) {
    InputEvents::processKey(VK_LEFT);
    return true;
  }
  if (strcmp(gesture, "R") == 0) {
    InputEvents::processKey(VK_RIGHT);
    return true;
  }

  if (strcmp(gesture, "DU") == 0) {
    InputEvents::ShowMenu();
    return true;
  }

  if (strcmp(gesture, "DR") == 0) {
    InputEvents::eventGotoLookup(_T(""));
    return true;
  }

  return false;
}

#if defined(GNAV)

bool
GlueMapWindow::on_key_down(unsigned key_code)
{
  return on_key_press(key_code) || MapWindow::on_key_down(key_code);
}

#else

bool
GlueMapWindow::on_key_up(unsigned key_code)
{
  return on_key_press(key_code) || MapWindow::on_key_up(key_code);
}

#endif

bool
GlueMapWindow::on_key_press(unsigned key_code)
{
  key_code = TranscodeKey(key_code);
  if (is_altair() && key_code == 0xF5) {
    XCSoarInterface::SignalShutdown(false);
    return true;
  }

  mouse_down_clock.reset();
  if (InputEvents::processKey(key_code)) {
    return true; // don't go to default handler
  }

  return false;
}

bool
GlueMapWindow::on_cancel_mode()
{
  MapWindow::on_cancel_mode();

  if (drag_mode != DRAG_NONE) {
    release_capture();
    drag_mode = DRAG_NONE;
  }

  return false;
}

void
GlueMapWindow::on_paint(Canvas &canvas)
{
  if (drag_mode == DRAG_PAN) {
    /* quick redraw while scrolling the map with the mouse */

    canvas.null_pen();
    canvas.white_brush();

    /* clear the areas around the buffer */

    if (drag_last.x > drag_start.x)
      canvas.rectangle(0, 0, drag_last.x - drag_start.x, canvas.get_height());
    else if (drag_last.x < drag_start.x)
      canvas.rectangle(canvas.get_width() + drag_last.x - drag_start.x, 0,
                       canvas.get_width(), canvas.get_height());

    if (drag_last.y > drag_start.y)
      canvas.rectangle(0, 0, canvas.get_width(), drag_last.y - drag_start.y);
    else if (drag_last.y < drag_start.y)
      canvas.rectangle(0, canvas.get_height() + drag_last.y - drag_start.y,
                       canvas.get_width(), canvas.get_height());

    /* now copy the buffer */

    ScopeLock protect(DoubleBufferWindow::mutex);
    canvas.copy(drag_last.x - drag_start.x, drag_last.y - drag_start.y,
                canvas.get_width(), canvas.get_height(),
                get_visible_canvas(), 0, 0);
  } else
    return MapWindow::on_paint(canvas);
}
