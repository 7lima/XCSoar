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
#include "Screen/Fonts.hpp"
#include "Screen/TextInBox.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Layout.hpp"
#include "Math/Screen.hpp"
#include "Appearance.hpp"
#include "Units.hpp"

#include <stdlib.h>
#include <stdio.h>


void
MapWindow::DrawWind(Canvas &canvas, const RasterPoint &Start,
                               const RECT &rc) const
{
  if (SettingsMap().EnablePan && !SettingsMap().TargetPan)
    return;

  TCHAR sTmp[12];
  static SIZE tsize = { 0, 0 };

  const SpeedVector wind = Basic().wind;

  if (wind.norm < fixed_one)
    // JMW don't bother drawing it if not significant
    return;

  if (tsize.cx == 0) {
    canvas.select(Fonts::MapBold);
    tsize = canvas.text_size(_T("99"));
    tsize.cx = tsize.cx / 2;
  }

  canvas.select(Graphics::hpWind);
  canvas.select(Graphics::hbWind);

  int wmag = iround(4 * wind.norm);

  int kx = tsize.cx / Layout::FastScale(1) / 2;

  RasterPoint Arrow[7] = {
      { 0, -20 },
      { -6, -26 },
      { 0, -20 },
      { 6, -26 },
      { 0, -20 },
      { 8 + kx, -24 },
      { -8 - kx, -24 }
  };

  for (int i = 1; i < 4; i++)
    Arrow[i].y -= wmag;

  PolygonRotateShift(Arrow, 7, Start.x, Start.y,
                     wind.bearing - render_projection.GetScreenAngle());

  canvas.polygon(Arrow, 5);

  if (SettingsMap().WindArrowStyle == 1) {
    RasterPoint Tail[2] = {
      { 0, Layout::FastScale(-20) },
      { 0, Layout::FastScale(-26 - min(20, wmag) * 3) },
    };

    Angle angle = (wind.bearing - render_projection.GetScreenAngle()).as_bearing();
    PolygonRotateShift(Tail, 2, Start.x, Start.y, angle);

    // optionally draw dashed line
    Pen dash_pen(Pen::DASH, 1, Color::BLACK);
    canvas.select(dash_pen);
    canvas.line(Tail[0], Tail[1]);
  }

  _stprintf(sTmp, _T("%i"),
            iround(Units::ToUserUnit(wind.norm, Units::WindSpeedUnit)));

  canvas.set_text_color(Color::BLACK);

  TextInBoxMode_t TextInBoxMode;
  TextInBoxMode.Align = Center;
  TextInBoxMode.Mode = Outlined;

  if (Arrow[5].y >= Arrow[6].y)
    TextInBox(canvas, sTmp, Arrow[5].x - kx, Arrow[5].y, TextInBoxMode, rc);
  else
    TextInBox(canvas, sTmp, Arrow[6].x - kx, Arrow[6].y, TextInBoxMode, rc);
}

void
MapWindow::DrawCompass(Canvas &canvas, const RECT &rc) const
{
  RasterPoint Start;

  if (Appearance.CompassAppearance == apCompassDefault) {
    Start.y = IBLSCALE(19) + rc.top;
    Start.x = rc.right - IBLSCALE(19);

    RasterPoint Arrow[5] = { { 0, -18 }, { -6, 10 }, { 0, 0 }, { 6, 10 }, { 0, -18 } };

    canvas.select(Graphics::hpCompass);
    canvas.select(Graphics::hbCompass);

    // North arrow
    PolygonRotateShift(Arrow, 5, Start.x, Start.y,
                       Angle::native(fixed_zero) - render_projection.GetScreenAngle());
    canvas.polygon(Arrow, 5);
  } else if (Appearance.CompassAppearance == apCompassAltA) {

    static Angle lastDisplayAngle = Angle::native(fixed_zero);
    static int lastRcRight = 0;
    static RasterPoint Arrow[5] = { { 0, -11 }, { -5, 9 }, { 0, 3 }, { 5, 9 }, { 0, -11 } };

    if (lastDisplayAngle != render_projection.GetScreenAngle() ||
        lastRcRight != rc.right) {
      Arrow[0].x = 0;
      Arrow[0].y = -11;
      Arrow[1].x = -5;
      Arrow[1].y = 9;
      Arrow[2].x = 0;
      Arrow[2].y = 3;
      Arrow[3].x = 5;
      Arrow[3].y = 9;
      Arrow[4].x = 0;
      Arrow[4].y = -11;

      Start.y = rc.top + IBLSCALE(10);
      Start.x = rc.right - IBLSCALE(11);

      // North arrow
      PolygonRotateShift(Arrow, 5, Start.x, Start.y,
                         Angle::native(fixed_zero) - render_projection.GetScreenAngle());

      lastDisplayAngle = render_projection.GetScreenAngle();
      lastRcRight = rc.right;
    }
    canvas.polygon(Arrow, 5);

    canvas.select(Graphics::hpCompass);
    canvas.polygon(Arrow, 5);
  }
}

void
MapWindow::DrawBestCruiseTrack(Canvas &canvas, const RasterPoint aircraft_pos) const
{
  if (!Basic().gps.Connected ||
      !Calculated().task_stats.task_valid ||
      Calculated().task_stats.current_leg.solution_remaining.Vector.Distance
      < fixed(0.010))
    return;

  canvas.select(Graphics::hpBestCruiseTrack);
  canvas.select(Graphics::hbBestCruiseTrack);

  const Angle angle = Calculated().task_stats.current_leg.solution_remaining.CruiseTrackBearing
                    - render_projection.GetScreenAngle();

  RasterPoint Arrow[] = { { -1, -40 }, { -1, -62 }, { -6, -62 }, {  0, -70 },
                    {  6, -62 }, {  1, -62 }, {  1, -40 }, { -1, -40 } };

  PolygonRotateShift(Arrow, sizeof(Arrow) / sizeof(Arrow[0]),
                     aircraft_pos.x, aircraft_pos.y, angle);

  canvas.polygon(Arrow, sizeof(Arrow) / sizeof(Arrow[0]));
}
