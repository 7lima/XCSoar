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

#ifndef SHAPE_RENDERER_HPP
#define SHAPE_RENDERER_HPP

#include "Screen/Pen.hpp"
#include "Screen/Point.hpp"
#include "Util/NonCopyable.hpp"
#include "Util/AllocatedArray.hpp"

#ifdef ENABLE_OPENGL
class Canvas;
class Brush;
#else
#include "Screen/Canvas.hpp"
#include "Screen/Brush.hpp"
#endif

#include <assert.h>

/**
 * A helper class optimized for doing bulk draws on OpenGL.
 */
class ShapeRenderer : private NonCopyable {
  AllocatedArray<RasterPoint> points;
  unsigned num_points;

#ifndef ENABLE_OPENGL
  const Pen *pen;
  const Brush *brush;

  enum { NONE, OUTLINE, SOLID } mode;
#endif

public:
  void configure(const Pen *_pen, const Brush *_brush) {
#ifdef ENABLE_OPENGL
    _pen->get_color().set();
#else
    pen = _pen;
    brush = _brush;
    mode = NONE;
#endif

    num_points = 0;
  }

  void begin_shape(unsigned n) {
    assert(num_points == 0);

    points.grow_discard(((n - 1) | 0x3ff) + 1);
  }

  void add_point(RasterPoint pt) {
    assert(num_points < points.size());

    points[num_points++] = pt;
  }

  /**
   * Adds the point only if it a few pixels distant from the previous
   * one.  Useful to reduce the complexity of small figures.
   */
   void add_point_if_distant(RasterPoint pt) {
    assert(num_points < points.size());

    if (num_points == 0 || manhattan_distance(points[num_points - 1], pt) >= 8)
      add_point(pt);
  }

  void finish_polyline(Canvas &canvas) {
#ifdef ENABLE_OPENGL
    glVertexPointer(2, GL_VALUE, 0, &points[0].x);
    glDrawArrays(GL_LINE_STRIP, 0, num_points);
#else
    if (mode != OUTLINE) {
      canvas.select(*pen);
      mode = OUTLINE;
    }

    canvas.polyline(points.begin(), num_points);
#endif

    num_points = 0;
  }

  void finish_polygon(Canvas &canvas) {
#ifdef ENABLE_OPENGL
    glVertexPointer(2, GL_VALUE, 0, &points[0].x);
#ifdef ANDROID
    // XXX
    glDrawArrays(GL_TRIANGLES, 0, num_points / 3);
#else
    glDrawArrays(GL_POLYGON, 0, num_points);
#endif
#else
    if (mode != SOLID) {
      canvas.null_pen();
      canvas.select(*brush);
      mode = SOLID;
    }

    canvas.polygon(points.begin(), num_points);
#endif

    num_points = 0;
  }

  void commit() {
    assert(num_points == 0);
  }
};

#endif
