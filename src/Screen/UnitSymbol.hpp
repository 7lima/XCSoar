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

#ifndef XCSOAR_SCREEN_UNIT_SYMBOL_HPP
#define XCSOAR_SCREEN_UNIT_SYMBOL_HPP

#include "Units.hpp"
#include "Screen/Bitmap.hpp"

class Canvas;
class BitmapCanvas;

class UnitSymbol {
public:
  enum kind {
    NORMAL = 0,
    INVERSE = 1,
    GRAY = 2,
    INVERSE_GRAY = INVERSE | GRAY,
  };

protected:
  Bitmap bitmap;
  SIZE size;

public:
  void load(unsigned id, unsigned width, unsigned height) {
    bitmap.load(id);
    size.cx = width;
    size.cy = height;
  }

  void reset() {
    bitmap.reset();
  }

  bool defined() const {
    return bitmap.defined();
  }

  const Bitmap &get_bitmap() const {
    return bitmap;
  }

  operator const Bitmap &() const {
    return bitmap;
  }

  const POINT get_origin(enum kind kind) const;

  const SIZE get_size() const {
    return size;
  }

  void draw(Canvas& canvas, BitmapCanvas& bitmap_canvas,
            const int x, const int y) const;
};

void
LoadUnitSymbols();

const UnitSymbol *
GetUnitSymbol(Units_t unit);

#endif

