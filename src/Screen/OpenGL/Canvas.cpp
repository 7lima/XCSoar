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

#include "Screen/Canvas.hpp"
#include "Screen/Bitmap.hpp"
#include "Screen/OpenGL/Globals.hpp"
#include "Screen/OpenGL/Texture.hpp"
#include "Screen/OpenGL/Scope.hpp"
#include "Screen/OpenGL/Cache.hpp"
#include "Screen/OpenGL/VertexArray.hpp"
#include "Screen/OpenGL/Draw.hpp"
#include "Screen/Util.hpp"

#include <assert.h>

AllocatedArray<RasterPoint> Canvas::vertex_buffer;

void
Canvas::rectangle(int left, int top, int right, int bottom)
{
  GLRectangleVertices vertices(left, top, right, bottom);
  vertices.bind();

  if (!brush.is_hollow()) {
    brush.set();
    GLubyte i[] = { 0, 1, 2, 0, 2, 3 };
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, i);
  }

  if (pen_over_brush()) {
    pen.set();
    glDrawArrays(GL_LINE_LOOP, 0, 4);
  }
}

void
Canvas::fill_rectangle(int left, int top, int right, int bottom,
                       const Color color)
{
  color.set();
  GLFillRectangle(left, top, right, bottom);
}

void
Canvas::outline_rectangle(int left, int top, int right, int bottom)
{
  pen.set();

  const RasterPoint v[] = {
    { left, top },
    { right, top },
    { right, bottom },
    { left, bottom },
    { left, top },
  };

  glVertexPointer(2, GL_VALUE, 0, v);
  glDrawArrays(GL_LINE_STRIP, 0, 5);
}

void
Canvas::raised_edge(RECT &rc)
{
  Pen bright(1, Color(240, 240, 240));
  select(bright);
  two_lines(rc.left, rc.bottom - 2, rc.left, rc.top,
            rc.right - 2, rc.top);

  Pen dark(1, Color(128, 128, 128));
  select(dark);
  two_lines(rc.left + 1, rc.bottom - 1, rc.right - 1, rc.bottom - 1,
            rc.right - 1, rc.top + 1);

  ++rc.left;
  ++rc.top;
  --rc.right;
  --rc.bottom;
}

void
Canvas::polyline(const RasterPoint *points, unsigned num_points)
{
  glVertexPointer(2, GL_VALUE, 0, points);

  pen.set();
  glDrawArrays(GL_LINE_STRIP, 0, num_points);
}

void
Canvas::polygon(const RasterPoint *points, unsigned num_points)
{
  if (brush.is_hollow() && !pen.defined())
    return;

  glVertexPointer(2, GL_VALUE, 0, points);

  if (!brush.is_hollow() && num_points >= 3) {
    brush.set();

    static AllocatedArray<GLushort> triangle_buffer;
    triangle_buffer.grow_discard(3 * (num_points - 2));
    int idx_count = polygon_to_triangle(points, num_points,
                                        triangle_buffer.begin());
    if (idx_count > 0)
      glDrawElements(GL_TRIANGLES, idx_count, GL_UNSIGNED_SHORT,
                     triangle_buffer.begin());
  }

  if (pen_over_brush()) {
    pen.set();
    if (pen.get_width() <= 2) {
      glDrawArrays(GL_LINE_LOOP, 0, num_points);
    } else {
      vertex_buffer.grow_discard(2 * (num_points + 1));
      unsigned vertices = line_to_triangle(points, num_points,
                                           vertex_buffer.begin(),
                                           pen.get_width(), true);
      if (vertices > 0) {
        glVertexPointer(2, GL_VALUE, 0, vertex_buffer.begin());
        glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices);
      }
    }
  }
}

void
Canvas::line(int ax, int ay, int bx, int by)
{
  pen.set();

  const GLvalue v[] = { ax, ay, bx, by };
  glVertexPointer(2, GL_VALUE, 0, v);
  glDrawArrays(GL_LINE_STRIP, 0, 2);
}

/**
 * Draw a line from a to b, using triangle caps if pen-size > 2 to hide
 * gaps between consecutive lines.
 */
void
Canvas::line_piece(const RasterPoint a, const RasterPoint b)
{
  pen.set();

  const RasterPoint v[] = { {a.x, a.y}, {b.x, b.y} };
  if (pen.get_width() > 2) {
    RasterPoint strip[6];
    unsigned strip_len = line_to_triangle(v, 2, strip, pen.get_width(),
                                          false, true);
    if (strip_len > 0) {
      glVertexPointer(2, GL_VALUE, 0, &strip[0].x);
      glDrawArrays(GL_TRIANGLE_STRIP, 0, strip_len);
    }
  } else {
    glVertexPointer(2, GL_VALUE, 0, &v[0].x);
    glDrawArrays(GL_LINE_STRIP, 0, 2);
  }
}

void
Canvas::two_lines(int ax, int ay, int bx, int by, int cx, int cy)
{
  pen.set();

  const GLvalue v[] = { ax, ay, bx, by, cx, cy };
  glVertexPointer(2, GL_VALUE, 0, v);
  glDrawArrays(GL_LINE_STRIP, 0, 3);
}

void
Canvas::two_lines(const RasterPoint a, const RasterPoint b,
                  const RasterPoint c)
{
  pen.set();

  const RasterPoint v[] = { a, b, c };
  glVertexPointer(2, GL_VALUE, 0, v);
  glDrawArrays(GL_LINE_STRIP, 0, 3);
}

void
Canvas::circle(int x, int y, unsigned radius)
{
  if (pen_over_brush() && pen.get_width() > 2) {
    GLDonutVertices vertices(x, y,
                             radius - pen.get_width()/2,
                             radius + pen.get_width()/2);
    if (!brush.is_hollow()) {
      vertices.bind_inner_circle();
      brush.set();
      glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.CIRCLE_SIZE);
    }
    vertices.bind();
    pen.set();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices.SIZE);
  } else {
    GLCircleVertices vertices(x, y, radius);
    vertices.bind();

    if (!brush.is_hollow()) {
      brush.set();
      glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.SIZE);
    }

    if (pen_over_brush()) {
      pen.set();
      glDrawArrays(GL_LINE_LOOP, 0, vertices.SIZE);
    }
  }
}

void
Canvas::segment(int x, int y, unsigned radius,
                Angle start, Angle end, bool horizon)
{
  ::Segment(*this, x, y, radius, start, end, horizon);
}

void
Canvas::draw_focus(RECT rc)
{
  Pen pen(1, Color::DARK_GRAY);
  select(pen);
  outline_rectangle(rc.left, rc.top, rc.right, rc.bottom);
}

void
Canvas::text(int x, int y, const TCHAR *text)
{
#ifdef ANDROID
  assert(x_offset == OpenGL::translate_x);
  assert(y_offset == OpenGL::translate_y);
#endif

  if (font == NULL)
    return;

  GLTexture *texture = TextCache::get(font, Color::BLACK, Color::WHITE, text);
  if (texture == NULL)
    return;

  if (background_mode == OPAQUE)
    /* draw the opaque background */
    fill_rectangle(x, y, x + texture->get_width(), y + texture->get_height(),
                   background_color);

  GLEnable scope(GL_TEXTURE_2D);
  texture->bind();
  GLLogicOp logic_op(GL_AND_INVERTED);

  if (background_mode != OPAQUE || background_color != Color::BLACK) {
    /* cut out the shape in black */
    glColor4f(1.0, 1.0, 1.0, 1.0);
    texture->draw(x, y);
  }

  if (text_color != Color::BLACK) {
    /* draw the text color on top */
    logic_op.set(GL_OR);
    text_color.set();
    texture->draw(x, y);
  }
}

void
Canvas::stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const GLTexture &texture,
                int src_x, int src_y,
                unsigned src_width, unsigned src_height)
{
#ifdef ANDROID
  assert(x_offset == OpenGL::translate_x);
  assert(y_offset == OpenGL::translate_y);
#endif

  texture.draw(dest_x, dest_y, dest_width, dest_height,
               src_x, src_y, src_width, src_height);
}

void
Canvas::stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const GLTexture &texture)
{
  stretch(dest_x, dest_y, dest_width, dest_height,
          texture, 0, 0, texture.get_width(), texture.get_height());
}

void
Canvas::copy(int dest_x, int dest_y,
             unsigned dest_width, unsigned dest_height,
             const Bitmap &src, int src_x, int src_y)
{
  stretch(dest_x, dest_y, dest_width, dest_height,
          src, src_x, src_y, dest_width, dest_height);
}

void
Canvas::copy(const Bitmap &src)
{
  copy(0, 0, src.get_width(), src.get_height(), src, 0, 0);
}

void
Canvas::stretch_transparent(const Bitmap &src, Color key)
{
  assert(src.defined());

  // XXX
  stretch(src);
}

void
Canvas::invert_stretch_transparent(const Bitmap &src, Color key)
{
  assert(src.defined());

  // XXX
  GLLogicOp invert(GL_COPY_INVERTED);
  stretch(src);
}

void
Canvas::stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const Bitmap &src, int src_x, int src_y,
                unsigned src_width, unsigned src_height)
{
#ifdef ANDROID
  assert(x_offset == OpenGL::translate_x);
  assert(y_offset == OpenGL::translate_y);
#endif
  assert(src.defined());

  glColor4f(1.0, 1.0, 1.0, 1.0);

  GLTexture &texture = *src.native();
  GLEnable scope(GL_TEXTURE_2D);
  texture.bind();
  texture.draw(dest_x, dest_y, dest_width, dest_height,
               src_x, src_y, src_width, src_height);
}

void
Canvas::stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const Bitmap &src)
{
#ifdef ANDROID
  assert(x_offset == OpenGL::translate_x);
  assert(y_offset == OpenGL::translate_y);
#endif
  assert(src.defined());

  glColor4f(1.0, 1.0, 1.0, 1.0);

  GLTexture &texture = *src.native();
  GLEnable scope(GL_TEXTURE_2D);
  texture.bind();
  texture.draw(dest_x, dest_y, dest_width, dest_height,
               0, 0, src.get_width(), src.get_height());
}

void
Canvas::copy_or(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const Bitmap &src, int src_x, int src_y)
{
  assert(src.defined());

  GLLogicOp logic_op(GL_OR);
  copy(dest_x, dest_y, dest_width, dest_height,
       src, src_x, src_y);
}

void
Canvas::copy_and(int dest_x, int dest_y,
                 unsigned dest_width, unsigned dest_height,
                 const Bitmap &src, int src_x, int src_y)
{
  assert(src.defined());

  GLLogicOp logic_op(GL_AND);
  copy(dest_x, dest_y, dest_width, dest_height,
       src, src_x, src_y);
}
