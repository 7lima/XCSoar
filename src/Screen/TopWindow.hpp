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

#ifndef XCSOAR_SCREEN_TOP_WINDOW_HXX
#define XCSOAR_SCREEN_TOP_WINDOW_HXX

#include "Screen/ContainerWindow.hpp"

#ifdef HAVE_AYGSHELL_DLL
#include "OS/AYGShellDLL.hpp"
#endif

#ifdef ENABLE_SDL
#include "Thread/Mutex.hpp"

class TopCanvas : public Canvas {
public:
  void set();

  void full_screen();

  void flip() {
    ::SDL_Flip(surface);
  }
};
#endif

/**
 * A top-level full-screen window.
 */
class TopWindow : public ContainerWindow {
#ifdef ENABLE_SDL
  TopCanvas screen;

  Mutex invalidated_lock;
  bool invalidated;
#else /* !ENABLE_SDL */

  /**
   * On WM_ACTIVATE, the focus is returned to this window.
   */
  HWND hSavedFocus;

#ifdef HAVE_AYGSHELL_DLL
  SHACTIVATEINFO s_sai;
#endif
#endif /* !ENABLE_SDL */

public:
#ifdef HAVE_AYGSHELL_DLL
  const AYGShellDLL ayg_shell_dll;
#endif

public:
  TopWindow();

  static bool find(const TCHAR *cls, const TCHAR *text);

  void set(const TCHAR *cls, const TCHAR *text,
           int left, int top, unsigned width, unsigned height);

  void set_active() {
    assert_none_locked();

#ifdef ENABLE_SDL
    // XXX
#else
    ::SetActiveWindow(hWnd);
#endif
  }

  void full_screen();

#ifdef ENABLE_SDL
  virtual void invalidate();

  virtual void expose();

  void refresh();
#endif /* ENABLE_SDL */

  void close() {
    assert_none_locked();

#ifdef ENABLE_SDL
    on_close();
#else /* ENABLE_SDL */
    ::SendMessage(hWnd, WM_CLOSE, 0, 0);
#endif
  }

protected:
  virtual bool on_activate();
  virtual bool on_deactivate();

#ifdef ENABLE_SDL
  virtual bool on_event(const SDL_Event &event);
#else /* !ENABLE_SDL */
  virtual LRESULT on_message(HWND _hWnd, UINT message,
                             WPARAM wParam, LPARAM lParam);
#endif /* !ENABLE_SDL */

public:
  void post_quit();

  /**
   * Runs the event loop until the application quits.
   */
  int event_loop();
};

#endif
