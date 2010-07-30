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

#include "Gauge/GaugeFLARM.hpp"
#include "Math/FastMath.h"
#include "Math/FastRotation.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/Layout.hpp"
#include "Screen/BitmapCanvas.hpp"
#include "NMEA/Info.hpp"
#include "Units.hpp"
#include "resource.h"
#include "Dialogs.h"

#include <stdio.h>

#include <algorithm>

using std::min;
using std::max;

#define FLARMMAXRANGE 2000

/**
 * Constructor of the GaugeFLARM class
 * @param parent Parent window
 * @param left Left edge of window pixel location
 * @param top Top edge of window pixel location
 * @param width Width of window (pixels)
 * @param height Height of window (pixels)
 */
GaugeFLARM::GaugeFLARM(ContainerWindow &parent,
                       int left, int top, unsigned width, unsigned height)
  :FlarmTrafficWindow(1, true),
   ForceVisible(false), Suppress(false)
{
  // start of new code for displaying FLARM window

  WindowStyle style;
  style.hide();

  set(parent, left, top, width, height, style);
}

void
GaugeFLARM::Update(bool enable, const NMEA_INFO &gps_info,
                   const SETTINGS_TEAMCODE &settings)
{
  // If FLARM alarm level higher then 0
  if (gps_info.flarm.FLARM_AlarmLevel > 0)
    // Show FLARM gauge and do not care about suppression
    Suppress = false;

  bool visible = ForceVisible ||
    (gps_info.flarm.FLARMTraffic && enable && !Suppress);
  if (visible) {
    FlarmTrafficWindow::Update(gps_info.TrackBearing, gps_info.flarm,
                               settings);
    send_user(MSG_SHOW);
  } else
    send_user(MSG_HIDE);
}

/**
 * This function is called when the mouse is pressed on the FLARM gauge and
 * opens the FLARM Traffic dialog
 * @param x x-Coordinate of the click
 * @param y x-Coordinate of the click
 * @return
 */
bool
GaugeFLARM::on_mouse_down(int x, int y)
{
  dlgFlarmTrafficShowModal();
  return true;
}

bool
GaugeFLARM::on_user(unsigned id)
{
  switch ((msg)id) {
  case MSG_SHOW:
    show();
    return true;

  case MSG_HIDE:
    hide();
    return true;
  }

  return Window::on_user(id);
}
