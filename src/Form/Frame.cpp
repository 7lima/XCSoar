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

#include "Form/Frame.hpp"

WndFrame::WndFrame(ContainerControl *Owner, const TCHAR *Name,
                   int X, int Y, int Width, int Height)
  :ContainerControl(Owner, NULL, Name, X, Y, Width, Height),
   mCaptionStyle(DT_EXPANDTABS | DT_LEFT | DT_NOCLIP | DT_WORDBREAK)
{
  SetForeColor(GetOwner()->GetForeColor());
  SetBackColor(GetOwner()->GetBackColor());
}

void WndFrame::SetCaption(const TCHAR *Value){
  if (Value == NULL)
    Value = TEXT("");

  if (_tcscmp(mCaption, Value) != 0){
    _tcscpy(mCaption, Value);  // todo size check
    invalidate();
  }
}

UINT WndFrame::SetCaptionStyle(UINT Value){
  UINT res = mCaptionStyle;
  if (res != Value){
    mCaptionStyle = Value;
    invalidate();
  }
  return res;
}

unsigned
WndFrame::GetTextHeight()
{
  RECT rc = get_client_rect();
  ::InflateRect(&rc, -2, -2); // todo border width

  Canvas &canvas = get_canvas();
  canvas.select(*GetFont());
  canvas.formatted_text(&rc, mCaption, mCaptionStyle | DT_CALCRECT);

  return rc.bottom - rc.top;
}

void
WndFrame::on_paint(Canvas &canvas)
{
  ContainerControl::on_paint(canvas);

  if (mCaption != 0){
    canvas.set_text_color(GetForeColor());
    canvas.set_background_color(GetBackColor());
    canvas.background_transparent();

    canvas.select(*GetFont());

    RECT rc = get_client_rect();
    InflateRect(&rc, -2, -2); // todo border width

//    h = rc.bottom - rc.top;

    canvas.formatted_text(&rc, mCaption,
      mCaptionStyle // | DT_CALCRECT
    );
  }
}
