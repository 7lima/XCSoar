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

#include "Screen/VirtualCanvas.hpp"

#include <assert.h>

#ifdef ENABLE_SDL

VirtualCanvas::VirtualCanvas(unsigned _width, unsigned _height)
{
  set(_width, _height);
}

VirtualCanvas::VirtualCanvas(const Canvas &canvas,
                             unsigned _width, unsigned _height)
{
  set(_width, _height);
}

#else /* !ENABLE_SDL */

VirtualCanvas::VirtualCanvas(unsigned _width, unsigned _height)
  :Canvas(::CreateCompatibleDC(NULL), _width, _height)
{
}

VirtualCanvas::VirtualCanvas(const Canvas &canvas,
                             unsigned _width, unsigned _height)
  :Canvas(::CreateCompatibleDC(canvas), _width, _height)
{
  assert(canvas.defined());
}

#endif /* !ENABLE_SDL */

VirtualCanvas::~VirtualCanvas()
{
  reset();
}

void
VirtualCanvas::set(unsigned _width, unsigned _height)
{
  assert(_width < 0x1000000);
  assert(_height < 0x1000000);

  reset();

#ifdef ENABLE_SDL
  const SDL_Surface *video = ::SDL_GetVideoSurface();
  assert(video != NULL);
  const SDL_PixelFormat *format = video->format;

  SDL_Surface *surface;
  surface = ::SDL_CreateRGBSurface(SDL_SWSURFACE, _width, _height,
                                   format->BitsPerPixel,
                                   format->Rmask, format->Gmask,
                                   format->Bmask, format->Amask);
  if (surface != NULL)
    Canvas::set(surface);
#else /* !ENABLE_SDL */
  Canvas::set(CreateCompatibleDC(NULL), _width, _height);
#endif /* !ENABLE_SDL */
}

void
VirtualCanvas::set(const Canvas &canvas, unsigned _width, unsigned _height)
{
  assert(canvas.defined());

#ifdef ENABLE_SDL
  set(_width, _height);
#else /* !ENABLE_SDL */
  reset();
  Canvas::set(CreateCompatibleDC(canvas), _width, _height);
#endif /* !ENABLE_SDL */
}

void
VirtualCanvas::set(const Canvas &canvas)
{
  set(canvas, canvas.get_width(), canvas.get_height());
}

#ifndef ENABLE_SDL
void VirtualCanvas::reset()
{
  if (dc != NULL)
    ::DeleteDC(dc);
}
#endif /* !ENABLE_SDL */
