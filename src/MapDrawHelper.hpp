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
#ifndef MAP_DRAW_HELPER_HPP
#define MAP_DRAW_HELPER_HPP

#include "Navigation/SearchPointVector.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Layout.hpp"

class Canvas;
class Projection;
struct SETTINGS_MAP;

/**
 * Utility class to draw multilayer items on a canvas with stencil masking
 */
class MapDrawHelper 
{
public:
  MapDrawHelper(Canvas &_canvas, 
                Canvas &_buffer, 
                Canvas &_stencil, 
                const Projection &_proj,
                const SETTINGS_MAP& settings_map);

  MapDrawHelper(MapDrawHelper &_that);

  Canvas &m_canvas;
  Canvas &m_buffer;
  Canvas &m_stencil;
  const Projection& m_proj;
  bool m_buffer_drawn;
  bool m_use_stencil;

  const SETTINGS_MAP& m_settings_map;

protected:

  void draw_great_circle(Canvas& the_canvas, const GEOPOINT &from,
                         const GEOPOINT &to);

  void draw_search_point_vector(Canvas& the_canvas, const SearchPointVector& points);

  void draw_circle(Canvas& the_canvas, const POINT& center, unsigned radius);

  void buffer_render_finish();

  void buffer_render_start();

  void clear_buffer();

  bool add_if_visible(std::vector<POINT>& screen, const GEOPOINT& pt) const;
  void add(std::vector<POINT>& screen, const GEOPOINT& pt) const;

};


#endif
