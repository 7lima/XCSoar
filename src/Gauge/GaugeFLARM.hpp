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

#ifndef GAUGE_FLARM_H
#define GAUGE_FLARM_H

#include "Screen/BufferWindow.hpp"
#include "Screen/Bitmap.hpp"

struct NMEA_INFO;
class ContainerWindow;

class GaugeFLARM: public BufferWindow
{
private:
  Bitmap hRoseBitMap;
  SIZE hRoseBitMapSize;
  POINT center;
  int radius;

public:
  bool Visible, ForceVisible, Suppress, Traffic;

public:
  GaugeFLARM(ContainerWindow &parent,
             int left, int top, unsigned width, unsigned height);
  void Render(const NMEA_INFO &gps_info);
  void RenderTraffic(Canvas &canvas, const NMEA_INFO &gps_info);
  void RenderBg(Canvas &canvas);
  void Show(const bool enable_gauge);
  void TrafficPresent(bool traffic);

  bool on_mouse_down(int x, int y);

private:
  int RangeScale(double d);
};

#endif
