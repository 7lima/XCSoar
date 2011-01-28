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

#include "Topology/TopologyRenderer.hpp"
#include "Topology/TopologyFile.hpp"
#include "Topology/XShape.hpp"
#include "WindowProjection.hpp"
#include "SettingsMap.hpp"
#include "resource.h"
#include "Screen/Canvas.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/LabelBlock.hpp"
#include "Math/Matrix2D.hpp"
#include "shapelib/map.h"
#include "Util/AllocatedArray.hpp"
#include "Geo/GeoClip.hpp"

#include <algorithm>

TopologyFileRenderer::TopologyFileRenderer(const TopologyFile &_file)
  :file(_file), pen(file.get_pen_width(), file.get_color()),
   brush(file.get_color())
{
  if (file.get_icon() == IDB_TOWN)
    icon.load_big(IDB_TOWN, IDB_TOWN_HD);
}

gcc_const
static rectObj
ConvertRect(const GeoBounds br)
{
  rectObj dest;
  dest.minx = br.west.value_degrees();
  dest.maxx = br.east.value_degrees();
  dest.miny = br.south.value_degrees();
  dest.maxy = br.north.value_degrees();
  return dest;
}

void
TopologyFileRenderer::Paint(Canvas &canvas,
                            const WindowProjection &projection) const
{
  fixed map_scale = projection.GetMapScale();
  if (!file.is_visible(map_scale))
    return;

  // TODO code: only draw inside screen!
  // this will save time with rendering pixmaps especially
  // we already do an outer visibility test, but may need a test
  // in screen coords

  shape_renderer.configure(&pen, &brush);

  // get drawing info

  const rectObj screenRect =
    ConvertRect(projection.GetScreenBounds());

#ifdef ENABLE_OPENGL
  const unsigned level = file.thinning_level(map_scale);
  const unsigned min_distance = file.min_point_distance(level);

#ifndef ANDROID
  float opengl_matrix[16];
  glGetFloatv(GL_MODELVIEW_MATRIX, opengl_matrix);
#endif

  glPushMatrix();
#ifdef ANDROID
#ifdef FIXED_MATH
  GLfixed angle = projection.GetScreenAngle().value_degrees().as_glfixed();
  GLfixed scale = projection.GetScale().as_glfixed_scale();
#else
  GLfixed angle = projection.GetScreenAngle().value_degrees() * (1<<16);
  GLfixed scale = projection.GetScale() * (1LL<<32);
#endif
  glTranslatex((int)projection.GetScreenOrigin().x << 16,
               (int)projection.GetScreenOrigin().y << 16, 0);
  glRotatex(angle, 0, 0, -(1<<16));
  glScalex(scale, scale, 1<<16);
#else
  glTranslatef(projection.GetScreenOrigin().x, projection.GetScreenOrigin().y, 0.);
  glRotatef(projection.GetScreenAngle().value_degrees(), 0., 0., -1.);
  glScalef(projection.GetScale(), projection.GetScale(), 1.);
#endif
#else // !ENABLE_OPENGL
  const GeoClip clip(projection.GetScreenBounds().scale(fixed(1.1)));
  AllocatedArray<GeoPoint> geo_points;

  int iskip = file.GetSkipSteps(map_scale);
#endif

  for (unsigned i = 0; i < file.size(); ++i) {
    XShape *cshape = file[i];
    if (!cshape || !cshape->is_visible(file.get_label_field()))
      continue;

    if (!projection.GetScreenBounds().overlaps(cshape->get_bounds()))
      continue;

#ifdef ENABLE_OPENGL
    const ShapePoint *points = cshape->get_points();

    const ShapePoint translation =
      cshape->shape_translation(projection.GetGeoLocation());
    glPushMatrix();
#ifdef ANDROID
    glTranslatex(translation.x, translation.y, 0);
#else
    glTranslatef(translation.x, translation.y, 0.);
#endif
#else // !ENABLE_OPENGL
    const unsigned short *lines = cshape->get_lines();
    const unsigned short *end_lines = lines + cshape->get_number_of_lines();
    const GeoPoint *points = cshape->get_points();
#endif

    switch (cshape->get_type()) {
    case MS_SHAPE_POINT:
      if (!icon.defined())
        break;

#ifdef ENABLE_OPENGL
      // TODO: for now i assume there is only one point for point-XShapes
      {
        RasterPoint sc;
        if (projection.GeoToScreenIfVisible(cshape->get_center(), sc)) {
#ifndef ANDROID
          glPushMatrix();
          glLoadMatrixf(opengl_matrix);
#endif
          icon.draw(canvas, sc.x, sc.y);
#ifndef ANDROID
          glPopMatrix();
#endif
        }
      }
#else // !ENABLE_OPENGL
      for (; lines < end_lines; ++lines) {
        const GeoPoint *end = points + *lines;
        for (; points < end; ++points) {
          RasterPoint sc;
          if (projection.GeoToScreenIfVisible(*points, sc))
            icon.draw(canvas, sc.x, sc.y);
        }
      }
#endif
      break;

    case MS_SHAPE_LINE:
#ifdef ENABLE_OPENGL
#ifdef ANDROID
      glVertexPointer(2, GL_FIXED, 0, &points[0].x);
#else
      glVertexPointer(2, GL_INT, 0, &points[0].x);
#endif

      if (level == 0) {
        const GLushort *count = cshape->get_lines();
        const GLushort *end_count = count + cshape->get_number_of_lines();
        for (int offset = 0; count < end_count; offset += *count++)
          glDrawArrays(GL_LINE_STRIP, offset, *count);
      } else {
        const GLushort *index_count;
        const GLushort *indices;
        indices = cshape->get_indices(level, min_distance, index_count);
        const GLushort *end_count = index_count + cshape->get_number_of_lines();
        for (; index_count < end_count; indices += *index_count++)
          glDrawElements(GL_LINE_STRIP, *index_count, GL_UNSIGNED_SHORT,
                         indices);
      }
#else // !ENABLE_OPENGL
      for (; lines < end_lines; ++lines) {
        unsigned msize = *lines;
        shape_renderer.begin_shape(msize);

        const GeoPoint *end = points + msize - 1;
        for (; points < end; ++points)
          shape_renderer.add_point_if_distant(projection.GeoToScreen(*points));

        // make sure we always draw the last point
        shape_renderer.add_point(projection.GeoToScreen(*points));

        shape_renderer.finish_polyline(canvas);
      }
#endif
      break;

    case MS_SHAPE_POLYGON:
#ifdef ENABLE_OPENGL
      {
        const GLushort *index_count;
        const GLushort *triangles = cshape->get_indices(level, min_distance,
                                                        index_count);

#ifdef ANDROID
        glVertexPointer(2, GL_FIXED, 0, &points[0].x);
#else
        glVertexPointer(2, GL_INT, 0, &points[0].x);
#endif
        glDrawElements(GL_TRIANGLE_STRIP, *index_count, GL_UNSIGNED_SHORT,
                       triangles);
      }
#else // !ENABLE_OPENGL
      for (; lines < end_lines; ++lines) {
        unsigned msize = *lines / iskip;

        /* copy all polygon points into the geo_points array and clip
           them, to avoid integer overflows (as RasterPoint may store
           only 16 bit integers on some platforms) */

        geo_points.grow_discard(msize * 3);

        for (unsigned i = 0; i < msize; ++i)
          geo_points[i] = points[i * iskip];

        msize = clip.clip_polygon(geo_points.begin(),
                                  geo_points.begin(), msize);
        if (msize < 3)
          continue;

        shape_renderer.begin_shape(msize);

        for (unsigned i = 0; i < msize; ++i) {
          GeoPoint g = geo_points[i];
          shape_renderer.add_point_if_distant(projection.GeoToScreen(g));
        }

        shape_renderer.finish_polygon(canvas);
      }
#endif
      break;
    }
#ifdef ENABLE_OPENGL
    glPopMatrix();
#endif
  }
#ifdef ENABLE_OPENGL
  glPopMatrix();
#endif

  shape_renderer.commit();
}

void
TopologyFileRenderer::PaintLabels(Canvas &canvas,
                                  const WindowProjection &projection,
                                  LabelBlock &label_block,
                                  const SETTINGS_MAP &settings_map) const
{
  fixed map_scale = projection.GetMapScale();
  if (!file.is_visible(map_scale))
    return;
  if (!file.is_label_visible(map_scale))
    return;

  // TODO code: only draw inside screen!
  // this will save time with rendering pixmaps especially
  // we already do an outer visibility test, but may need a test
  // in screen coords

  canvas.select(file.is_label_important(map_scale) ?
                Fonts::MapLabelImportant : Fonts::MapLabel);
  canvas.set_text_color(Color(0x20, 0x20, 0x20));
  canvas.background_transparent();

  // get drawing info

  int iskip = file.GetSkipSteps(map_scale);

  rectObj screenRect =
    ConvertRect(projection.GetScreenBounds());

#ifdef ENABLE_OPENGL
  Matrix2D m1;
  m1.Translate(projection.GetScreenOrigin());
  m1.Rotate(projection.GetScreenAngle());
  m1.Scale(projection.GetScale());
#endif

  for (unsigned i = 0; i < file.size(); ++i) {
    const XShape *cshape = file[i];
    if (!cshape || !cshape->is_visible(file.get_label_field()))
      continue;

    const TCHAR *label = cshape->get_label();
    if (label == NULL)
      continue;

    if (!projection.GetScreenBounds().overlaps(cshape->get_bounds()))
      continue;

    const unsigned short *lines = cshape->get_lines();
    const unsigned short *end_lines = lines + cshape->get_number_of_lines();
#ifdef ENABLE_OPENGL
    const ShapePoint *points = cshape->get_points();

    Matrix2D m2(m1);
    m2.Translatex(cshape->shape_translation(projection.GetGeoLocation()));
#else
    const GeoPoint *points = cshape->get_points();
#endif

    for (; lines < end_lines; ++lines) {
      int minx = canvas.get_width();
      int miny = canvas.get_height();

#ifdef ENABLE_OPENGL
      const ShapePoint *end = points + *lines;
#else
      const GeoPoint *end = points + *lines;
#endif
      for (; points < end; points += iskip) {
#ifdef ENABLE_OPENGL
        RasterPoint pt = m2.Apply(*points);
#else
        RasterPoint pt = projection.GeoToScreen(*points);
#endif

        if (pt.x <= minx) {
          minx = pt.x;
          miny = pt.y;
        }
      }

      points = end;

      minx += 2;
      miny += 2;

      SIZE tsize = canvas.text_size(label);
      RECT brect;
      brect.left = minx;
      brect.right = brect.left + tsize.cx;
      brect.top = miny;
      brect.bottom = brect.top + tsize.cy;

      if (!label_block.check(brect))
        continue;

      canvas.text(minx, miny, label);
    }
  }
}

TopologyRenderer::TopologyRenderer(const TopologyStore &_store)
  :store(_store)
{
  for (unsigned i = 0; i < store.size(); ++i)
    files[i] = new TopologyFileRenderer(store[i]);
}

TopologyRenderer::~TopologyRenderer()
{
  for (unsigned i = 0; i < store.size(); ++i)
    delete files[i];
}

void
TopologyRenderer::Draw(Canvas &canvas,
                       const WindowProjection &projection) const
{
  for (unsigned i = 0; i < store.size(); ++i)
    files[i]->Paint(canvas, projection);
}

void
TopologyRenderer::DrawLabels(Canvas &canvas,
                             const WindowProjection &projection,
                             LabelBlock &label_block,
                             const SETTINGS_MAP &settings_map) const
{
  for (unsigned i = 0; i < store.size(); ++i)
    files[i]->PaintLabels(canvas, projection, label_block, settings_map);
}
