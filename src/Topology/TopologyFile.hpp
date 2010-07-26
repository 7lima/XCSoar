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

#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "shapelib/mapshape.h"
#include "Screen/Pen.hpp"
#include "Screen/Brush.hpp"
#include "Screen/Icon.hpp"

struct GEOPOINT;
class Canvas;
class Projection;
class LabelBlock;
struct SETTINGS_MAP;
class XShape;

class TopologyFile
{
  int label_field;

public:
  /**
   * The constructor opens the given shapefile and clears the cache
   * @param shpname The shapefile to open (*.shp)
   * @param thecolor The color to use for drawing
   * @param label_field The field in which the labels should be searched
   * @return
   */
  TopologyFile(const char* shpname, const Color thecolor, int label_field);

  /**
   * The destructor clears the cache and closes the shapefile
   */
  ~TopologyFile();

  void TriggerIfScaleNowVisible(const Projection &map_projection);

  void updateCache(Projection &map_projection,
		   const rectObj &thebounds, bool purgeonly = false);

  /**
   * Paints the polygons, lines and points/icons in the TopologyFile
   * @param canvas The canvas to paint on
   * @param bitmap_canvas Temporary canvas for the icon
   * @param projection
   */
  void Paint(Canvas &canvas, BitmapCanvas &bitmap_canvas,
             const Projection &projection) const;

  /**
   * Paints a topology label if the space is available in the LabelBlock
   * @param canvas The canvas to paint on
   * @param projection
   * @param label_block The LabelBlock class to use for decluttering
   * @param settings_map
   */
  void PaintLabels(Canvas &canvas,
                   const Projection &projection, LabelBlock &label_block,
                   const SETTINGS_MAP &settings_map) const;

  /**
   * The threshold value for the visibility check. If the current scale
   * is below this value the contents of this TopologyFile will be drawn.
   */
  double scaleThreshold;

  /**
   * This function loads an icon from the resource file that
   * will be drawn at each MS_SHAPE_POINT of the TopologyFile
   * @param
   */
  void loadIcon(const int);
  bool triggerUpdateCache;

private:
  /**
   * This returns whether the content of this TopologyFile is
   * visible at the given map_scale
   * @param map_scale The scale to check
   * @return True if visible, False if not
   */
  bool CheckScale(const double map_scale) const;

  XShape** shpCache;

  unsigned GetSkipSteps(double map_scale) const;

protected:
  void ClearCache();

  bool in_scale;
  Pen hPen;
  Brush hbBrush;
  MaskedIcon icon;
  shapefileObj shpfile;
  bool shapefileopen;
};

#endif
