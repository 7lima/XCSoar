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

#ifndef XCSOAR_SCREEN_BUFFER_WINDOW_HXX
#define XCSOAR_SCREEN_BUFFER_WINDOW_HXX

#include "Screen/PaintWindow.hpp"
#include "Screen/BufferCanvas.hpp"

/**
 * A #PaintWindow with buffered painting, to avoid flickering.
 */
class BufferWindow : public PaintWindow {
private:
#ifndef ENABLE_SDL
  BufferCanvas buffer;
#endif /* !ENABLE_SDL */

public:
#ifndef ENABLE_SDL
  void set(ContainerWindow &parent, const TCHAR *cls,
           int left, int top, unsigned width, unsigned height,
           const WindowStyle style=WindowStyle()) {
    PaintWindow::set(parent, cls, left, top, width, height, style);
  }

  void set(ContainerWindow &parent,
           int left, int top, unsigned width, unsigned height,
           const WindowStyle style=WindowStyle()) {
    PaintWindow::set(parent, left, top, width, height, style);
  }
#endif /* !ENABLE_SDL */

  Canvas &get_canvas() {
#ifdef ENABLE_SDL
    return canvas;
#else
    return buffer;
#endif
  }

  const Canvas &get_canvas() const {
#ifdef ENABLE_SDL
    return canvas;
#else
    return buffer;
#endif
  }

  /**
   * Copies the buffer to the specified canvas.
   */
  void commit_buffer(Canvas &dest) {
    dest.copy(get_canvas());
  }

protected:
#ifndef ENABLE_SDL
  virtual bool on_create();
  virtual bool on_destroy();

  virtual bool on_resize(unsigned width, unsigned height);
#endif /* !ENABLE_SDL */

  virtual void on_paint(Canvas &canvas);
};

#endif
