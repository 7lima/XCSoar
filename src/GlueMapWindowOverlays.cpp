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

#include "GlueMapWindow.hpp"

#include "Screen/Icon.hpp"
#include "Language.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Layout.hpp"
#include "Logger/Logger.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Appearance.hpp"
#include "Screen/TextInBox.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/UnitSymbol.hpp"
#include "Terrain/RasterWeather.hpp"

#include <stdio.h>

void
GlueMapWindow::DrawCrossHairs(Canvas &canvas) const
{
  Pen dash_pen(Pen::DASH, 1, Color(50, 50, 50));
  canvas.select(dash_pen);

  const RasterPoint Orig_Screen = render_projection.GetScreenOrigin();

  canvas.line(Orig_Screen.x + 20, Orig_Screen.y,
              Orig_Screen.x - 20, Orig_Screen.y);
  canvas.line(Orig_Screen.x, Orig_Screen.y + 20,
              Orig_Screen.x, Orig_Screen.y - 20);
}

void
GlueMapWindow::DrawGPSStatus(Canvas &canvas, const RECT &rc,
                             const GPS_STATE &gps) const
{
  const TCHAR *txt;
  MaskedIcon *icon = NULL;

  if (!gps.Connected) {
    icon = &Graphics::hGPSStatus2;
    txt = _("GPS not connected");
  } else if (gps.NAVWarning || (gps.SatellitesUsed == 0)) {
    icon = &Graphics::hGPSStatus1;
    txt = _("GPS waiting for fix");
  } else {
    return; // early exit
  }

  int x = rc.left + Layout::FastScale(2);
  int y = rc.bottom - Layout::FastScale(35);
  icon->draw(canvas, x, y);

  x += icon->get_size().cx + Layout::FastScale(2);
  y = rc.bottom - Layout::FastScale(34);

  TextInBoxMode_t mode;
  mode.Mode = RoundedBlack;

  TextInBox(canvas, txt, x, y, mode, rc, NULL);
}

void
GlueMapWindow::DrawFlightMode(Canvas &canvas, const RECT &rc) const
{
  int offset = -1;

  if (logger.isLoggerActive()) {
    offset -= 7;
    bool flip = (Basic().DateTime.second % 2) == 0;
    MaskedIcon &icon = flip ? Graphics::hLogger : Graphics::hLoggerOff;
    icon.draw(canvas, rc.right + IBLSCALE(offset), rc.bottom + IBLSCALE(-7));
  }

  MaskedIcon *bmp;

  if (task != NULL && (task->get_mode() == TaskManager::MODE_ABORT))
    bmp = &Graphics::hAbort;
  else if (GetDisplayMode() == dmCircling)
    bmp = &Graphics::hClimb;
  else if (GetDisplayMode() == dmFinalGlide)
    bmp = &Graphics::hFinalGlide;
  else
    bmp = &Graphics::hCruise;

  offset -= 25;

  bmp->draw(canvas, rc.right + IBLSCALE(offset),
            rc.bottom + IBLSCALE(-21));
}

void
GlueMapWindow::DrawFinalGlide(Canvas &canvas, const RECT &rc) const
{
  RasterPoint GlideBar[6] = {
      { 0, 0 }, { 9, -9 }, { 18, 0 }, { 18, 0 }, { 9, 0 }, { 0, 0 }
  };
  RasterPoint GlideBar0[6] = {
      { 0, 0 }, { 9, -9 }, { 18, 0 }, { 18, 0 }, { 9, 0 }, { 0, 0 }
  };

  TCHAR Value[10];

  int Offset;
  int Offset0;
  int i;

  if (Calculated().task_stats.task_valid) {
    const int y0 = ((rc.bottom - rc.top) / 2) + rc.top;

    // 60 units is size, div by 8 means 60*8 = 480 meters.
    Offset = ((int)Calculated().task_stats.total.solution_remaining.AltitudeDifference) / 8;

    // JMW OLD_TASK this is broken now
    Offset0 = ((int)Calculated().task_stats.total.solution_mc0.AltitudeDifference) / 8;
    // TODO feature: should be an angle if in final glide mode

    if (Offset > 60)
      Offset = 60;
    if (Offset < -60)
      Offset = -60;

    Offset = IBLSCALE(Offset);
    if (Offset < 0)
      GlideBar[1].y = IBLSCALE(9);

    if (Offset0 > 60)
      Offset0 = 60;
    if (Offset0 < -60)
      Offset0 = -60;

    Offset0 = IBLSCALE(Offset0);
    if (Offset0 < 0)
      GlideBar0[1].y = IBLSCALE(9);

    for (i = 0; i < 6; i++) {
      GlideBar[i].y += y0;
      GlideBar[i].x = IBLSCALE(GlideBar[i].x) + rc.left;
    }

    GlideBar[0].y -= Offset;
    GlideBar[1].y -= Offset;
    GlideBar[2].y -= Offset;

    for (i = 0; i < 6; i++) {
      GlideBar0[i].y += y0;
      GlideBar0[i].x = IBLSCALE(GlideBar0[i].x) + rc.left;
    }

    GlideBar0[0].y -= Offset0;
    GlideBar0[1].y -= Offset0;
    GlideBar0[2].y -= Offset0;

    if ((Offset < 0) && (Offset0 < 0)) {
      // both below
      if (Offset0 != Offset) {
        int dy = (GlideBar0[0].y - GlideBar[0].y) +
            (GlideBar0[0].y - GlideBar0[3].y);
        dy = max(IBLSCALE(3), dy);
        GlideBar[3].y = GlideBar0[0].y - dy;
        GlideBar[4].y = GlideBar0[1].y - dy;
        GlideBar[5].y = GlideBar0[2].y - dy;

        GlideBar0[0].y = GlideBar[3].y;
        GlideBar0[1].y = GlideBar[4].y;
        GlideBar0[2].y = GlideBar[5].y;
      } else {
        Offset0 = 0;
      }
    } else if ((Offset > 0) && (Offset0 > 0)) {
      // both above
      GlideBar0[3].y = GlideBar[0].y;
      GlideBar0[4].y = GlideBar[1].y;
      GlideBar0[5].y = GlideBar[2].y;

      if (abs(Offset0 - Offset) < IBLSCALE(4))
        Offset = Offset0;
    }

    // draw actual glide bar
    if (Offset <= 0) {
      if (Calculated().common_stats.landable_reachable) {
        canvas.select(Graphics::hpFinalGlideBelowLandable);
        canvas.select(Graphics::hbFinalGlideBelowLandable);
      } else {
        canvas.select(Graphics::hpFinalGlideBelow);
        canvas.select(Graphics::hbFinalGlideBelow);
      }
    } else {
      canvas.select(Graphics::hpFinalGlideAbove);
      canvas.select(Graphics::hbFinalGlideAbove);
    }
    canvas.polygon(GlideBar, 6);

    // draw glide bar at mc 0
    if (Offset0 <= 0) {
      if (Calculated().common_stats.landable_reachable) {
        canvas.select(Graphics::hpFinalGlideBelowLandable);
        canvas.hollow_brush();
      } else {
        canvas.select(Graphics::hpFinalGlideBelow);
        canvas.hollow_brush();
      }
    } else {
      canvas.select(Graphics::hpFinalGlideAbove);
      canvas.hollow_brush();
    }

    if (Offset != Offset0)
      canvas.polygon(GlideBar0, 6);

    // draw x on final glide bar if unreachable at current Mc
    if (!Calculated().task_stats.total.achievable()) {
      canvas.select(Graphics::hpAircraftBorder);
      canvas.line(Layout::Scale(9 - 5), y0 + Layout::Scale(9 - 5),
                  Layout::Scale(9 + 5), y0 + Layout::Scale(9 + 5));
      canvas.line(Layout::Scale(9 - 5), y0 + Layout::Scale(9 + 5),
                  Layout::Scale(9 + 5), y0 + Layout::Scale(9 - 5));
    }

    if (Appearance.IndFinalGlide == fgFinalGlideDefault) {
      Units::FormatUserAltitude(Calculated().task_stats.total.solution_remaining.AltitudeDifference,
                                Value, sizeof(Value) / sizeof(Value[0]),
                                false);

      if (Offset >= 0)
        Offset = GlideBar[2].y + Offset + IBLSCALE(5);
      else if (Offset0 > 0)
        Offset = GlideBar0[1].y - IBLSCALE(15);
      else
        Offset = GlideBar[2].y + Offset - IBLSCALE(15);

      canvas.set_text_color(Color::BLACK);

      TextInBoxMode_t TextInBoxMode;
      TextInBoxMode.Mode = RoundedBlack;
      TextInBox(canvas, Value, 0, (int)Offset, TextInBoxMode, rc);

    } else if (Appearance.IndFinalGlide == fgFinalGlideAltA) {

      SIZE TextSize;
      int y = GlideBar[3].y;
      // was ((rc.bottom - rc.top )/2)-rc.top-
      //            Appearance.MapWindowBoldFont.CapitalHeight/2-1;
      int x = GlideBar[2].x + IBLSCALE(1);

      Units::FormatUserAltitude(Calculated().task_stats.total.solution_remaining.AltitudeDifference,
                                Value, sizeof(Value) / sizeof(Value[0]), false);

      canvas.select(Fonts::MapBold);
      TextSize = canvas.text_size(Value);

      canvas.fill_rectangle(x, y, x + IBLSCALE(1) + TextSize.cx,
                            y + Fonts::MapBold.get_capital_height() + IBLSCALE(2),
                            Color::WHITE);

      canvas.set_text_color(Color::BLACK);
      canvas.text(x + IBLSCALE(1),
                  y + Fonts::MapBold.get_capital_height() -
                  Fonts::MapBold.get_ascent_height() + IBLSCALE(1), Value);

      const UnitSymbol *unit_symbol = GetUnitSymbol(
        Units::GetUserAltitudeUnit());

      if (unit_symbol != NULL)
        unit_symbol->draw(canvas, x + TextSize.cx + IBLSCALE(1), y);
    }
  }
}

void
GlueMapWindow::DrawThermalBand(Canvas &canvas, const RECT &rc) const
{
  RasterPoint GliderBand[5] = { { 0, 0 }, { 23, 0 }, { 22, 0 }, { 24, 0 }, { 0, 0 } };

  if ((Calculated().task_stats.total.solution_remaining.AltitudeDifference > fixed(50))
      && GetDisplayMode() == dmFinalGlide)
    return;

  const ThermalBandInfo &thermal_band = Calculated().thermal_band;

  // JMW TODO accuracy: gather proper statistics
  // note these should/may also be relative to ground
  int i;
  fixed mth = thermal_band.MaxThermalHeight;
  fixed maxh, minh;
  fixed h;
  fixed Wt[NUMTHERMALBUCKETS];
  fixed ht[NUMTHERMALBUCKETS];
  fixed Wmax = fixed_zero;
  int TBSCALEY = ((rc.bottom - rc.top) / 2) - IBLSCALE(30);
#define TBSCALEX 20

  // calculate height above safety altitude
  fixed hoffset = SettingsComputer().safety_height_terrain +
                   Calculated().TerrainBase;
  h = Basic().NavAltitude - hoffset;

  bool draw_start_height = false;
  fixed hstart = fixed_zero;

  OrderedTaskBehaviour task_props;
  if (task != NULL)
    task_props = task->get_ordered_task_behaviour();

  draw_start_height = Calculated().common_stats.ordered_valid
                      && (task_props.start_max_height != 0)
                      && Calculated().TerrainValid;
  if (draw_start_height) {
    if (task_props.start_max_height_ref == 0) {
      hstart = fixed(task_props.start_max_height) + Calculated().TerrainAlt;
    } else {
      hstart = fixed(task_props.start_max_height);
    }
    hstart -= hoffset;
  }

  // calculate top/bottom height
  maxh = max(h, mth);
  minh = min(h, fixed_zero);

  if (draw_start_height) {
    maxh = max(maxh, hstart);
    minh = min(minh, hstart);
  }

  // no thermalling has been done above safety altitude
  if (mth <= fixed_one)
    return;
  if (maxh <= minh)
    return;

  // normalised heights
  fixed hglider = (h - minh) / (maxh - minh);
  hstart = (hstart - minh) / (maxh - minh);

  // calculate averages
  int numtherm = 0;

  const fixed mc = get_glide_polar().get_mc();
  Wmax = max(fixed_half, mc);

  for (i = 0; i < NUMTHERMALBUCKETS; i++) {
    fixed wthis = fixed_zero;
    // height of this thermal point [0,mth]
    fixed hi = i * mth / NUMTHERMALBUCKETS;
    fixed hp = ((hi - minh) / (maxh - minh));

    if (thermal_band.ThermalProfileN[i] > 5) {
      // now requires 10 items in bucket before displaying,
      // to eliminate kinks
      wthis = thermal_band.ThermalProfileW[i] / thermal_band.ThermalProfileN[i];
    }
    if (positive(wthis)) {
      ht[numtherm] = hp;
      Wt[numtherm] = wthis;
      Wmax = max(Wmax, wthis * 2 / 3);
      numtherm++;
    }
  }

  if ((!draw_start_height) && (numtherm<=1))
    // don't display if insufficient statistics
    // but do draw if start height needs to be drawn
    return;

  // position of thermal band
  if (numtherm > 1) {
    canvas.select(Graphics::hpThermalBand);
    canvas.select(Graphics::hbThermalBand);

    RasterPoint ThermalProfile[NUMTHERMALBUCKETS + 2];
    for (i = 0; i < numtherm; i++) {
      ThermalProfile[1 + i].x =
          (iround((Wt[i] / Wmax) * IBLSCALE(TBSCALEX))) + rc.left;

      ThermalProfile[1 + i].y =
          IBLSCALE(4) + iround(TBSCALEY * (fixed_one - ht[i])) + rc.top;
    }
    ThermalProfile[0].x = rc.left;
    ThermalProfile[0].y = ThermalProfile[1].y;
    ThermalProfile[numtherm + 1].x = rc.left;
    ThermalProfile[numtherm + 1].y = ThermalProfile[numtherm].y;

    canvas.polygon(ThermalProfile, numtherm + 2);
  }

  // position of thermal band

  GliderBand[0].y = IBLSCALE(4) + iround(TBSCALEY * (fixed_one - hglider)) + rc.top;
  GliderBand[1].y = GliderBand[0].y;
  GliderBand[1].x =
      max(iround((mc / Wmax) * IBLSCALE(TBSCALEX)), IBLSCALE(4)) + rc.left;

  GliderBand[2].x = GliderBand[1].x - IBLSCALE(4);
  GliderBand[2].y = GliderBand[0].y - IBLSCALE(4);
  GliderBand[3].x = GliderBand[1].x;
  GliderBand[3].y = GliderBand[1].y;
  GliderBand[4].x = GliderBand[1].x - IBLSCALE(4);
  GliderBand[4].y = GliderBand[0].y + IBLSCALE(4);

  canvas.select(Graphics::hpThermalBandGlider);

  canvas.polyline(GliderBand, 2);
  canvas.polyline(GliderBand + 2, 3); // arrow head

  if (draw_start_height) {
    canvas.select(Graphics::hpFinalGlideBelow);
    GliderBand[0].y = IBLSCALE(4) + iround(TBSCALEY * (fixed_one - hstart)) + rc.top;
    GliderBand[1].y = GliderBand[0].y;
    canvas.polyline(GliderBand, 2);
  }
}

static fixed
findMapScaleBarSize(const RECT &rc, const MapWindowProjection &projection)
{
  // units/pixel
  fixed pixelsize = Units::ToUserDistance(projection.DistancePixelsToMeters(1));
  fixed half_displaysize = Units::ToUserDistance(
      projection.DistancePixelsToMeters((rc.bottom - rc.top) / 2));

  // find largest bar size that will fit two of (black and white) in display
  if (half_displaysize > fixed(100))
    return fixed(100) / pixelsize;

  if (half_displaysize > fixed_ten)
    return fixed_ten / pixelsize;

  if (half_displaysize > fixed_one)
    return fixed_one / pixelsize;

  return fixed_one / 10 / pixelsize;
}

void
GlueMapWindow::DrawMapScaleBar(Canvas &canvas, const RECT &rc,
                               const MapWindowProjection &projection) const
{
  canvas.select(Graphics::hpMapScale);

  bool color = false;
  RasterPoint Start, End = { 0, 0 };

  int barsize = iround(findMapScaleBarSize(rc, projection));

  End.x = rc.right - 1;
  End.y = projection.GetScreenOrigin().y;
  Start = End;
  for (Start.y += barsize; Start.y < rc.bottom + barsize; Start.y += barsize) {
    if (color)
      canvas.white_pen();
    else
      canvas.black_pen();

    canvas.line(Start, End);

    End = Start;
    color = !color;
  }

  color = true;
  End.y = projection.GetScreenOrigin().y;
  Start = End;
  for (Start.y -= barsize; Start.y > rc.top - barsize; Start.y -= barsize) {
    if (color)
      canvas.white_pen();
    else
      canvas.black_pen();

    canvas.line(Start, End);

    End = Start;
    color = !color;
  }
}

void
GlueMapWindow::DrawMapScale(Canvas &canvas, const RECT &rc,
                            const MapWindowProjection &projection) const
{
  fixed MapWidth;
  TCHAR ScaleInfo[80];

  int Height;
  Units_t Unit;

  MapWidth = projection.GetScreenWidthMeters();

  canvas.select(Fonts::MapBold);
  Units::FormatUserMapScale(&Unit, MapWidth, ScaleInfo,
                            sizeof(ScaleInfo) / sizeof(TCHAR), false);
  SIZE TextSize = canvas.text_size(ScaleInfo);

  Height = Fonts::MapBold.get_capital_height() + IBLSCALE(2);
  // 2: add 1pix border

  canvas.fill_rectangle(IBLSCALE(4), rc.bottom - Height,
                        TextSize.cx + IBLSCALE(16), rc.bottom,
                        Color::WHITE);

  canvas.background_transparent();
  canvas.set_text_color(Color::BLACK);

  canvas.text(IBLSCALE(7),
              rc.bottom - Fonts::MapBold.get_ascent_height() - IBLSCALE(1),
              ScaleInfo);

  Graphics::hBmpMapScaleLeft.draw(canvas, 0, rc.bottom - Height);
  Graphics::hBmpMapScaleRight.draw(canvas, IBLSCALE(14) + TextSize.cx,
                                   rc.bottom - Height);

  const UnitSymbol *symbol = GetUnitSymbol(Unit);
  if (symbol != NULL)
    symbol->draw(canvas, IBLSCALE(8) + TextSize.cx, rc.bottom - Height);

  int y = rc.bottom - Height - Fonts::Title.get_ascent_height();

  ScaleInfo[0] = '\0';
  if (SettingsMap().AutoZoom)
    _tcscat(ScaleInfo, _T("AUTO "));

  if (SettingsMap().TargetPan)
    _tcscat(ScaleInfo, _T("TARGET "));
  else if (SettingsMap().EnablePan)
    _tcscat(ScaleInfo, _T("PAN "));

  if (SettingsMap().EnableAuxiliaryInfo)
    _tcscat(ScaleInfo, _T("AUX "));

  if (Basic().gps.Replay)
    _tcscat(ScaleInfo, _T("REPLAY "));

  if (task != NULL && SettingsComputer().BallastTimerActive) {
    TCHAR TEMP[20];
    _stprintf(TEMP, _T("BALLAST %d LITERS "),
              (int)task->get_glide_polar().get_ballast_litres());
    _tcscat(ScaleInfo, TEMP);
  }

  if (weather != NULL && weather->GetParameter() > 0) {
    const TCHAR *label = weather->ItemLabel(weather->GetParameter());
    if (label != NULL)
      _tcscat(ScaleInfo, label);
  }

  if (ScaleInfo[0]) {
    canvas.select(Fonts::Title);
    canvas.background_opaque();
    canvas.set_background_color(Color::WHITE);
    canvas.text(0, y, ScaleInfo);
  }
}

void
GlueMapWindow::DrawHorizon(Canvas &canvas, const RECT &rc) const
{
  /*
  FEATURE TEMPORARILY DISABLED DUE TO USE OF XCSOAR IN FAI COMPETITIONS

  This feature of having a backup artificial horizon based on inferred
  orientation from GPS and vario data is useful, and reasonably well
  tested, but has the issue of potentially invalidating use of XCSoar in
  FAI contests due to rule ref Annex A to Section 3 (2010 Edition) 4.1.2
  "No instruments permitting pilots to fly without visual reference to
  the ground may be carried on board, even if made unserviceable."  The
  quality of XCSoar's pseudo-AH is arguably good enough that this
  violates the rule.  We need to seek clarification as to whether this
  is the case or not.
  */

  RasterPoint Start;
  Start.y = IBLSCALE(55) + rc.top;
  Start.x = rc.right - IBLSCALE(19);

  Pen hpHorizonSky(IBLSCALE(1), Color(0x40, 0x40, 0xff));
  Brush hbHorizonSky(Color(0xA0, 0xA0, 0xff));
  Pen hpHorizonGround(IBLSCALE(1), Color(106, 55, 12));
  Brush hbHorizonGround(Color(157, 101, 60));

#define fixed_div fixed(1.0 / 50.0)
#define fixed_89 fixed_int_constant(89)

  int radius = IBLSCALE(17);
  fixed phi = max(-fixed_89,
                  min(fixed_89, Basic().acceleration.BankAngle.value_degrees()));
  fixed alpha = fixed_rad_to_deg *
                acos(max(-fixed_one,
                     min(fixed_one,
                         Basic().acceleration.PitchAngle.value_degrees() * fixed_div)));
  fixed sphi = fixed_180 - phi;
  Angle alpha1 = Angle::degrees(sphi - alpha);
  Angle alpha2 = Angle::degrees(sphi + alpha);

  canvas.select(hpHorizonSky);
  canvas.select(hbHorizonSky);

  canvas.segment(Start.x, Start.y, radius, alpha2, alpha1, true);

  canvas.select(hpHorizonGround);
  canvas.select(hbHorizonGround);

  canvas.segment(Start.x, Start.y, radius, alpha1, alpha2, true);

  Pen dash_pen(Pen::DASH, 2, Color::BLACK);
  canvas.select(dash_pen);

  canvas.line(Start.x + radius / 2, Start.y, Start.x - radius / 2, Start.y);
  canvas.line(Start.x, Start.y - radius / 4, Start.x - radius / 2, Start.y);

  unsigned rr2p = uround(radius * fixed_sqrt_half) + IBLSCALE(1);
  unsigned rr2n = uround(radius * fixed_sqrt_half);

  canvas.black_pen();
  canvas.line(Start.x + rr2p, Start.y - rr2p, Start.x + rr2n, Start.y - rr2n);
  canvas.line(Start.x - rr2p, Start.y - rr2p, Start.x - rr2n, Start.y - rr2n);

  // JMW experimental, display stall sensor
  fixed s = max(fixed_zero, min(fixed_one, Basic().StallRatio));
  long m = (long)((rc.bottom - rc.top) * s * s);

  canvas.black_pen();
  canvas.line(rc.right - 1, rc.bottom - m, rc.right - 11, rc.bottom - m);
}
