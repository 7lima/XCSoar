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

#include "Topology.h"
#include <ctype.h> // needed for Wine
#include "Interface.hpp"
#include "wcecompat/ts_string.h"
#include "Screen/Util.hpp"
#include "MapWindow.hpp"
#include "MapWindowProjection.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/LabelBlock.hpp"
#include "SettingsUser.hpp"

#include <stdlib.h>
#include <tchar.h>

#include "LogFile.hpp"

XShape::XShape()
{
  hide = false;
}

XShape::~XShape()
{
  clear();
}

void
XShape::clear()
{
  msFreeShape(&shape);
}

void
XShape::load(shapefileObj* shpfile, int i)
{
  msInitShape(&shape);
  msSHPReadShape(shpfile->hSHP, i, &shape);
}

void
Topology::loadBitmap(const int xx)
{
  hBitmap.load(xx);
}

Topology::Topology(const char* shpname, const Color thecolor, bool doappend)
{
  append = doappend;
  memset((void*)&shpfile, 0, sizeof(shpfile));
  shapefileopen = false;
  triggerUpdateCache = false;
  scaleThreshold = 0;
  shpCache = NULL;

  in_scale = false;

  strcpy(filename, shpname);
  hPen.set(1, thecolor);
  hbBrush.set(thecolor);
  Open();
}

void
Topology::Open()
{
  shapefileopen = false;

  if (msSHPOpenFile(&shpfile, (append ? "rb+" : "rb"), filename) == -1)
    return;

  scaleThreshold = 1000.0;
  shpCache = (XShape**)malloc(sizeof(XShape*) * shpfile.numshapes);
  if (!shpCache)
    return;

  shapefileopen = true;
  for (int i = 0; i < shpfile.numshapes; i++) {
    shpCache[i] = NULL;
  }
}

void
Topology::Close()
{
  if (!shapefileopen)
    return;

  if (shpCache) {
    flushCache();
    free(shpCache);
    shpCache = NULL;
  }

  msSHPCloseFile(&shpfile);
  shapefileopen = false;
}

Topology::~Topology()
{
  Close();
}

bool
Topology::CheckScale(double map_scale)
{
  return (map_scale <= scaleThreshold);
}

void
Topology::TriggerIfScaleNowVisible(MapWindowProjection &map_projection)
{
  triggerUpdateCache |= (CheckScale(map_projection.GetMapScaleUser()) != in_scale);
}

void
Topology::flushCache()
{
  for (int i = 0; i < shpfile.numshapes; i++) {
    removeShape(i);
  }
  shapes_visible_count = 0;
}

void
Topology::updateCache(MapWindowProjection &map_projection, rectObj thebounds,
    bool purgeonly)
{
  if (!triggerUpdateCache)
    return;

  if (!shapefileopen)
    return;

  in_scale = CheckScale(map_projection.GetMapScaleUser());

  if (!in_scale) {
    // not visible, so flush the cache
    // otherwise we waste time on looking up which shapes are in bounds
    flushCache();
    triggerUpdateCache = false;
    return;
  }

  if (purgeonly)
    return;

  triggerUpdateCache = false;

  msSHPWhichShapes(&shpfile, thebounds, 0);
  if (!shpfile.status) {
    // this happens if entire shape is out of range
    // so clear buffer.
    flushCache();
    return;
  }

  shapes_visible_count = 0;

  for (int i = 0; i < shpfile.numshapes; i++) {
    if (msGetBit(shpfile.status, i)) {
      if (shpCache[i] == NULL) {
        // shape is now in range, and wasn't before
        shpCache[i] = addShape(i);
      }
      shapes_visible_count++;
    } else {
      removeShape(i);
    }
  }
}

XShape*
Topology::addShape(const int i)
{
  XShape* theshape = new XShape();
  theshape->load(&shpfile, i);
  return theshape;
}

void
Topology::removeShape(const int i)
{
  if (!shpCache[i])
    return;

  delete shpCache[i];
  shpCache[i] = NULL;
}

bool
Topology::checkVisible(shapeObj& shape, rectObj &screenRect)
{
  return (msRectOverlap(&shape.bounds, &screenRect) == MS_TRUE);
}

void
Topology::Paint(Canvas &canvas, MapWindow &m_window, const RECT rc)
{
  if (!shapefileopen)
    return;

  MapWindowProjection &map_projection = m_window;
  LabelBlock *label_block = m_window.getLabelBlock();

  double map_scale = map_projection.GetMapScaleUser();

  if (map_scale > scaleThreshold)
    return;

  // TODO code: only draw inside screen!
  // this will save time with rendering pixmaps especially
  // we already do an outer visibility test, but may need a test
  // in screen coords

  canvas.select(hPen);
  canvas.select(hbBrush);
  canvas.select(MapLabelFont);

  // get drawing info

  int iskip = 1;

  if (map_scale > 0.25 * scaleThreshold)
    iskip = 2;
  if (map_scale > 0.5 * scaleThreshold)
    iskip = 3;
  if (map_scale > 0.75 * scaleThreshold)
    iskip = 4;

  rectObj screenRect = map_projection.CalculateScreenBounds(fixed_zero);

  static POINT pt[MAXCLIPPOLYGON];
  const bool render_labels = (m_window.SettingsMap().DeclutterLabels < 2);

  for (int ixshp = 0; ixshp < shpfile.numshapes; ixshp++) {
    XShape *cshape = shpCache[ixshp];

    if (!cshape || cshape->hide)
      continue;

    shapeObj *shape = &(cshape->shape);

    switch (shape->type) {
    case (MS_SHAPE_POINT):
      if (!checkVisible(*shape, screenRect))
        break;

      for (int tt = 0; tt < shape->numlines; tt++) {
        for (int jj = 0; jj < shape->line[tt].numpoints; jj++) {
          POINT sc;
          GEOPOINT l;
          l.Longitude = shape->line[tt].point[jj].x;
          l.Latitude = shape->line[tt].point[jj].y;

          if (m_window.draw_masked_bitmap_if_visible(canvas, hBitmap, l, 10, 10, &sc)) {
            if (render_labels)
              cshape->renderSpecial(canvas, *label_block, sc.x, sc.y);
          }
        }
      }
      break;

    case (MS_SHAPE_LINE):
      if (!checkVisible(*shape, screenRect))
        break;

      for (int tt = 0; tt < shape->numlines; tt++) {
        int minx = rc.right;
        int miny = rc.bottom;
        int msize = std::min(shape->line[tt].numpoints, (int)MAXCLIPPOLYGON);

        map_projection.LonLat2Screen(shape->line[tt].point, pt, msize, 1);

        for (int jj = 0; jj < msize; jj++) {
          if (pt[jj].x <= minx) {
            minx = pt[jj].x;
            miny = pt[jj].y;
          }
        }

        canvas.polyline(pt, msize);
        if (render_labels)
          cshape->renderSpecial(canvas, *label_block, minx, miny);
      }
      break;

    case (MS_SHAPE_POLYGON):
      if (!checkVisible(*shape, screenRect))
        break;

      for (int tt = 0; tt < shape->numlines; tt++) {
        int minx = rc.right;
        int miny = rc.bottom;
        int msize = std::min(shape->line[tt].numpoints / iskip, (int)MAXCLIPPOLYGON);

        map_projection.LonLat2Screen(shape->line[tt].point, pt,
            msize * iskip, iskip);

        for (int jj = 0; jj < msize; jj++) {
          if (pt[jj].x <= minx) {
            minx = pt[jj].x;
            miny = pt[jj].y;
          }
        }

        canvas.polygon(pt, msize);
        if (render_labels)
          cshape->renderSpecial(canvas, *label_block, minx, miny);
      }
      break;

    default:
      break;
    }
  }
}

TopologyLabel::TopologyLabel(const char* shpname, const Color thecolor,
    int field1) :
  Topology(shpname, thecolor)
{
  //sjt 02nov05 - enabled label fields
  setField(std::max(0, field1));
  // JMW this is causing XCSoar to crash on my system!
}

TopologyLabel::~TopologyLabel()
{
}

void
TopologyLabel::setField(int i)
{
  field = i;
}

XShape*
TopologyLabel::addShape(const int i)
{
  XShapeLabel* theshape = new XShapeLabel();
  theshape->load(&shpfile, i);
  theshape->setlabel(msDBFReadStringAttribute(shpfile.hDBF, i, field));
  return theshape;
}

void
XShapeLabel::renderSpecial(Canvas &canvas, LabelBlock &label_block, int x, int y)
{
  if (!label)
    return;

  TCHAR Temp[100];
  _stprintf(Temp, TEXT("%S"), label);
  canvas.background_transparent();

  // TODO code: JMW asks, what does this do?
  if (ispunct(Temp[0])) {
    double dTemp;

    Temp[0] = '0';
    dTemp = _tcstod(Temp, NULL);
    dTemp = Units::ToUserUnit(dTemp, Units::AltitudeUnit);
    if (dTemp > 999)
      _stprintf(Temp, TEXT("%.1f"), (dTemp / 1000));
    else
      _stprintf(Temp, TEXT("%d"), int(dTemp));
  }

  SIZE tsize = canvas.text_size(Temp);

  x += 2;
  y += 2;

  RECT brect;
  brect.left = x;
  brect.right = brect.left + tsize.cx;
  brect.top = y;
  brect.bottom = brect.top + tsize.cy;

  if (!label_block.check(brect))
    return;

  canvas.set_text_color(Color(0x20, 0x20, 0x20));
  canvas.text(x, y, Temp);
}

void XShapeLabel::setlabel(const char* src) {
  if (label)
    free(label);

  if (src &&
      (strcmp(src,"UNK") != 0) &&
      (strcmp(src,"RAILWAY STATION") != 0) &&
      (strcmp(src,"RAILROAD STATION") != 0)) {
    label = (char*)malloc(strlen(src) + 1);
    if (label)
      strcpy(label, src);

    hide = false;
  } else {
    label = NULL;
    hide = true;
  }
}

XShapeLabel::~XShapeLabel()
{
  if (!label)
    return;

  free(label);
  label = NULL;
}

void
XShapeLabel::clear()
{
  XShape::clear();

  if (!label)
    return;

  free(label);
  label = NULL;
}

//       wsprintf(Scale,TEXT("%1.2f%c"),MapScale, autozoomstring);

TopologyWriter::~TopologyWriter()
{
  if (!shapefileopen)
    return;

  Close();
  DeleteFiles();
}

TopologyWriter::TopologyWriter(const char* shpname, const Color thecolor) :
  Topology(shpname, thecolor, true)
{
  Reset();
}

void
TopologyWriter::DeleteFiles(void)
{
  // Delete all files, since zziplib interface doesn't handle file modes
  // properly
  if (strlen(filename) <= 0)
    return;

  TCHAR fname[MAX_PATH];

  ascii2unicode(filename, fname);
  _tcscat(fname, TEXT(".shp"));
  DeleteFile(fname);

  ascii2unicode(filename, fname);
  _tcscat(fname, TEXT(".shx"));
  DeleteFile(fname);

  ascii2unicode(filename, fname);
  _tcscat(fname, TEXT(".dbf"));
  DeleteFile(fname);
}

void
TopologyWriter::CreateFiles(void)
{
  // by default, now, this overwrites previous contents
  if (msSHPCreateFile(&shpfile, filename, SHP_POINT) == -1)
    return;

  char dbfname[100];
  strcpy(dbfname, filename);
  strcat(dbfname, ".dbf");
  shpfile.hDBF = msDBFCreate(dbfname);

  shapefileopen = true;
  Close();
}

void
TopologyWriter::Reset(void)
{
  if (shapefileopen)
    Close();

  DeleteFiles();
  CreateFiles();

  Open();
}

void
TopologyWriter::addPoint(double x, double y)
{
  pointObj p = { x, y, 0.0 };

  if (shapefileopen) {
    msSHPWritePoint(shpfile.hSHP, &p);
    Close();
  }

  Open();
}
