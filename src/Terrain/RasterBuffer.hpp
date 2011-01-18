/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2011 The XCSoar Project
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

#ifndef XCSOAR_RASTER_BUFFER_HPP
#define XCSOAR_RASTER_BUFFER_HPP

#include "Util/NonCopyable.hpp"
#include "Compiler.h"

#include <cstddef>

class RasterBuffer : private NonCopyable {
public:
  /** invalid value for terrain */
  static const short TERRAIN_INVALID = -32768;

  gcc_const
  static bool is_invalid(short h) {
    return h == TERRAIN_INVALID;
  }

  gcc_const
  static bool is_water(short h) {
    return h <= 0 && !is_invalid(h);
  }

  gcc_const
  static bool is_special(short h) {
    return h <= 0;
  }

private:
  unsigned width, height;
  short *data;

public:
  RasterBuffer():width(0), height(0), data(NULL) {}
  RasterBuffer(unsigned _width, unsigned _height)
    :width(0), height(0), data(NULL) {
    resize(_width, _height);
  }

  ~RasterBuffer() {
    delete[] data;
  }

  bool defined() const {
    return data != NULL;
  }

  unsigned get_width() const {
    return width;
  }

  unsigned get_height() const {
    return height;
  }

  short *get_data() {
    return data;
  }

  const short *get_data() const {
    return data;
  }

  const short *get_data_at(unsigned x, unsigned y) const {
    return data + y * width + x;
  }

  void reset() {
    delete[] data;
    data = NULL;
    width = height = 0;
  }

  void resize(unsigned _width, unsigned _height);

  gcc_pure
  short get_interpolated(unsigned lx, unsigned ly,
                         unsigned ix, unsigned iy) const;

  gcc_pure
  short get_interpolated(unsigned lx, unsigned ly) const;

  gcc_pure
  short get(unsigned x, unsigned y) const {
    return *get_data_at(x, y);
  }

  gcc_pure
  short get_max() const;
};

#endif
