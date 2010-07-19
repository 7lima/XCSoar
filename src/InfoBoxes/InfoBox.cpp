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

#include "InfoBoxes/InfoBox.hpp"
#include "InputEvents.h"
#include "Compatibility/string.h"
#include "PeriodClock.hpp"
#include "Screen/UnitSymbol.hpp"
#include "Screen/BitmapCanvas.hpp"
#include "Screen/Layout.hpp"
#include "Screen/BufferCanvas.hpp"
#include "Screen/ContainerWindow.hpp"
#include "Appearance.hpp"
#include "Defines.h"
#include "Asset.hpp"

#include <algorithm>

using std::max;

#define SELECTORWIDTH IBLSCALE(5)

InfoBox::InfoBox(ContainerWindow &_parent, int X, int Y, int Width, int Height,
                 int border_flags,
                 const InfoBoxLook &_look)
  :parent(_parent),
   look(_look),
   mBorderKind(border_flags),
   focus_timer(0)
{
  mSmallerFont = false;

  colorValue = 0;
  colorTitle = 0;
  colorComment = 0;

  WindowStyle style;
  style.enable_double_clicks();
  set(parent, X, Y, Width, Height, style);

  mValueUnit = unUndef;

  _tcscpy(mTitle, TEXT(""));
  _tcscpy(mValue, TEXT(""));
  _tcscpy(mComment, TEXT(""));
}

void
InfoBox::SetValueUnit(Units_t Value)
{
  mValueUnit = Value;
  invalidate(recValue);
}

void
InfoBox::SetTitle(const TCHAR *Value)
{
  TCHAR sTmp[TITLESIZE + 1];

  _tcsncpy(sTmp, Value, TITLESIZE);
  sTmp[TITLESIZE] = '\0';

  if (Appearance.InfoTitelCapital)
    _tcsupr(sTmp);

  if (_tcscmp(mTitle, sTmp) != 0) {
    _tcscpy(mTitle, sTmp);
    invalidate(recTitle);
  }
}

void
InfoBox::SetValue(const TCHAR *Value)
{
  if (_tcscmp(mValue, Value) != 0) {
    _tcsncpy(mValue, Value, VALUESIZE);
    mValue[VALUESIZE] = '\0';
    invalidate(recValue);
  }
}

void
InfoBox::SetColor(int value)
{
  if (Appearance.InfoBoxColors)
    colorValue = value;
  else
    colorValue = 0;

  invalidate(recValue);
}

void
InfoBox::SetColorBottom(int value)
{
  if (Appearance.InfoBoxColors)
    colorComment = value;
  else
    colorComment = 0;

  invalidate(recComment);
}

void
InfoBox::SetColorTop(int value)
{
  if (Appearance.InfoBoxColors)
    colorTitle = value;
  else
    colorTitle = 0;

  invalidate(recTitle);
}

void
InfoBox::SetComment(const TCHAR *Value)
{
  if (_tcscmp(mComment, Value) != 0) {
    _tcsncpy(mComment, Value, COMMENTSIZE);
    mComment[COMMENTSIZE] = '\0';
    invalidate(recComment);
  }
}

void
InfoBox::SetSmallerFont(bool smallerFont)
{
  this->mSmallerFont = smallerFont;
  invalidate(recValue);
}

void
InfoBox::PaintTitle(Canvas &canvas)
{
  SIZE tsize;
  int x, y;
  int halftextwidth;

  canvas.set_background_color(look.title.bg_color);
  canvas.set_text_color(look.get_title_color(colorTitle));

  const Font &font = *look.title.font;
  canvas.select(font);

  tsize = canvas.text_size(mTitle);

  halftextwidth = (recTitle.left + recTitle.right - tsize.cx) / 2;

  x = max(1, (int)recTitle.left + halftextwidth);

  y = recTitle.top + 1 + font.get_capital_height() - font.get_ascent_height();

  canvas.text_opaque(x, y, &recTitle, mTitle);

  if (Appearance.InfoBoxBorder == apIbTab && halftextwidth > IBLSCALE(3)) {
    int ytop = recTitle.top + font.get_capital_height() / 2;
    int ytopedge = ytop + IBLSCALE(2);
    int ybottom = recTitle.top + IBLSCALE(6) + font.get_capital_height();

    canvas.select(look.border_pen);

    POINT tab[8];
    tab[0].x = tab[1].x = recTitle.left + IBLSCALE(1);
    tab[0].y = tab[7].y = ybottom;
    tab[2].x = recTitle.left + IBLSCALE(3);
    tab[2].y = tab[5].y = tab[3].y = tab[4].y = ytop;
    tab[1].y = tab[6].y = ytopedge;
    tab[5].x = recTitle.right - IBLSCALE(4);
    tab[6].x = tab[7].x = recTitle.right - IBLSCALE(2);
    tab[3].x = recTitle.left + halftextwidth - IBLSCALE(1);
    tab[4].x = recTitle.right - halftextwidth + IBLSCALE(1);

    canvas.polyline(tab, 4);
    canvas.polyline(tab + 4, 4);
  }
}

void
InfoBox::PaintValue(Canvas &canvas)
{
  SIZE tsize;
  int x, y;

  canvas.set_background_color(look.value.bg_color);
  canvas.set_text_color(look.get_value_color(colorValue));

  const Font &font = mSmallerFont ? *look.small_font : *look.value.font;
  canvas.select(font);

  tsize = canvas.text_size(mValue);

  SIZE unit_size;
  const UnitSymbol *unit_symbol = GetUnitSymbol(mValueUnit);
  if (unit_symbol != NULL) {
    unit_size = unit_symbol->get_size();
  } else {
    unit_size.cx = 0;
    unit_size.cy = 0;
  }

  x = max(1, (int)(recValue.left + recValue.right - tsize.cx
                   - Layout::FastScale(unit_size.cx)) / 2);

  y = recValue.top + 1 - font.get_ascent_height() +
    (recValue.bottom - recValue.top + font.get_capital_height()) / 2;

  canvas.text_opaque(x, y, &recValue, mValue);

  if (unit_symbol != NULL && colorValue >= 0) {
    POINT origin = unit_symbol->get_origin(Appearance.InverseInfoBox
                                           ? UnitSymbol::INVERSE
                                           : UnitSymbol::NORMAL);

    BitmapCanvas temp(canvas, *unit_symbol);

    canvas.scale_copy(x + tsize.cx,
                      y + font.get_ascent_height()
                      - Layout::FastScale(unit_size.cy),
                      temp,
                      origin.x, origin.y,
                      unit_size.cx, unit_size.cy);
  }
}

void
InfoBox::PaintComment(Canvas &canvas)
{
  SIZE tsize;
  int x, y;

  canvas.set_background_color(look.comment.bg_color);
  canvas.set_text_color(look.get_comment_color(colorComment));

  const Font &font = *look.comment.font;
  canvas.select(font);

  tsize = canvas.text_size(mComment);

  x = max(1, (int)(recComment.left + recComment.right - tsize.cx) / 2);
  y = recComment.top + 1 + font.get_capital_height()
    - font.get_ascent_height();

  canvas.text_opaque(x, y, &recComment, mComment);
}

void
InfoBox::PaintSelector(Canvas &canvas)
{
  canvas.select(look.selector_pen);

  const unsigned width = canvas.get_width(), height = canvas.get_height();

  canvas.two_lines(width - SELECTORWIDTH - 1, 0,
                   width - 1, 0,
                   width - 1, SELECTORWIDTH + 1);

  canvas.two_lines(width - 1, height - SELECTORWIDTH - 2,
                   width - 1, height - 1,
                   width - SELECTORWIDTH - 1, height - 1);

  canvas.two_lines(SELECTORWIDTH + 1, height - 1,
                   0, height - 1,
                   0, height - SELECTORWIDTH - 2);

  canvas.two_lines(0, SELECTORWIDTH + 1,
                   0, 0,
                   SELECTORWIDTH + 1, 0);
}

void
InfoBox::Paint(Canvas &canvas)
{
  canvas.background_opaque();

  PaintTitle(canvas);
  PaintComment(canvas);
  PaintValue(canvas);

  if (mBorderKind != 0) {
    canvas.select(look.border_pen);

    const unsigned width = canvas.get_width(), height = canvas.get_height();

    if (mBorderKind & BORDERTOP) {
      canvas.line(0, 0, width - 1, 0);
    }

    if (mBorderKind & BORDERRIGHT) {
      canvas.line(width - 1, 0, width - 1, height);
    }

    if (mBorderKind & BORDERBOTTOM) {
      canvas.line(0, height - 1, width - 1, height - 1);
    }

    if (mBorderKind & BORDERLEFT) {
      canvas.line(0, 0, 0, height - 1);
    }
  }
}

void
InfoBox::PaintInto(Canvas &dest, int xoff, int yoff, int width, int height)
{
  SIZE size = get_size();
  BufferCanvas buffer(dest, size.cx, size.cy);

  Paint(buffer);
  dest.stretch(xoff, yoff, width, height, buffer, 0, 0, size.cx, size.cy);
}

bool
InfoBox::on_resize(unsigned width, unsigned height)
{
  PaintWindow::on_resize(width, height);

  RECT rc = get_client_rect();

  if (mBorderKind & BORDERLEFT)
    rc.left += BORDER_WIDTH;

  if (mBorderKind & BORDERRIGHT)
    rc.right -= BORDER_WIDTH;

  if (mBorderKind & BORDERTOP)
    rc.top += BORDER_WIDTH;

  if (mBorderKind & BORDERBOTTOM)
    rc.bottom -= BORDER_WIDTH;

  recTitle = rc;
  recTitle.bottom = rc.top + look.title.font->get_capital_height() + 2;

  recComment = rc;
  recComment.top = recComment.bottom
    - (look.comment.font->get_capital_height() + 2);

  recValue = rc;
  recValue.top = recTitle.bottom;
  recValue.bottom = recComment.top;

  return true;
}

bool
InfoBox::on_key_down(unsigned key_code)
{
  // Get the input event_id of the event
  unsigned event_id = InputEvents::key_to_event(InputEvents::MODE_INFOBOX,
                                                key_code);
  if (event_id > 0) {
    // If input event exists -> process it
    InputEvents::processGo(event_id);

    // restart focus timer if not idle
    if (focus_timer != 0)
      kill_timer(focus_timer);

    focus_timer = set_timer(100, FOCUSTIMEOUTMAX * 500);
    return true;
  }

  return PaintWindow::on_key_down(key_code);
}

bool
InfoBox::on_mouse_down(int x, int y)
{
  // synthetic double click detection with no proximity , good for infoboxes
  static PeriodClock double_click;

  // if double clicked -> show menu
  if (!double_click.check_always_update(DOUBLECLICKINTERVAL)) {
    InputEvents::ShowMenu();
    return true;
  }

  // if single clicked -> focus the InfoBox
  set_focus();
  return true;
}

bool
InfoBox::on_mouse_double(int x, int y)
{
  if (!is_altair()) {
    // JMW capture double click, so infoboxes double clicked also bring up menu
    // VENTA3: apparently this is working only on PC ! Disable it to let PC work
    // with same timeout of PDA and PNA versions with synthetic DBLCLK
    InputEvents::ShowMenu();
  }

  return true;
}

void
InfoBox::on_paint(Canvas &canvas)
{
  // Call the parent function
  Paint(canvas);

  // Paint the selector
  if (has_focus())
    PaintSelector(canvas);
}

bool
InfoBox::on_setfocus()
{
  // Call the parent function
  PaintWindow::on_setfocus();

  // Start the focus-auto-return timer
  // to automatically return focus back to MapWindow if idle
  focus_timer = set_timer(100, FOCUSTIMEOUTMAX * 500);

  // Redraw fast to paint the selector
  invalidate();

  return true;
}

bool
InfoBox::on_killfocus()
{
  // Call the parent function
  PaintWindow::on_killfocus();

  // Destroy the time if it exists
  if (focus_timer != 0) {
    kill_timer(focus_timer);
    focus_timer = 0;
  }

  // Redraw fast to remove the selector
  invalidate();

  return true;
}

bool
InfoBox::on_timer(timer_t id)
{
  if (id != focus_timer)
    return PaintWindow::on_timer(id);

  kill_timer(focus_timer);
  focus_timer = 0;

  parent.set_focus();

  return true;
}
