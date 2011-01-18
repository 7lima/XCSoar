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

#ifndef XCSOAR_SCREEN_TOP_WINDOW_HXX
#define XCSOAR_SCREEN_TOP_WINDOW_HXX

#include "Screen/ContainerWindow.hpp"

#ifdef HAVE_AYGSHELL_DLL
#include "OS/AYGShellDLL.hpp"
#endif

#ifdef ENABLE_SDL
#include "Thread/Mutex.hpp"
#include "Screen/SDL/TopCanvas.hpp"
#endif

#ifdef ANDROID
struct Event;
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

#if !defined(ENABLE_SDL) && !defined(_WIN32_WCE)
  gcc_pure
  const RECT get_client_rect() const {
    if (::IsIconic(hWnd)) {
      /* for a minimized window, GetClientRect() returns the
         dimensions of the icon, which is not what we want */
      WINDOWPLACEMENT placement;
      if (::GetWindowPlacement(hWnd, &placement) &&
          (placement.showCmd == SW_MINIMIZE ||
           placement.showCmd == SW_SHOWMINIMIZED)) {
        placement.rcNormalPosition.right -= placement.rcNormalPosition.left;
        placement.rcNormalPosition.bottom -= placement.rcNormalPosition.top;
        placement.rcNormalPosition.left = 0;
        placement.rcNormalPosition.top = 0;
        return placement.rcNormalPosition;
      }
    }

    return ContainerWindow::get_client_rect();
  }

  gcc_pure
  const SIZE get_size() const {
    /* this is implemented again because Window::get_size() would call
       Window::get_client_rect() (method is not virtual) */
    RECT rc = get_client_rect();
    SIZE s;
    s.cx = rc.right;
    s.cy = rc.bottom;
    return s;
  }
#endif

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

#ifdef ANDROID
  bool on_event(const Event &event);
#elif defined(ENABLE_SDL)
  bool on_event(const SDL_Event &event);
#endif

protected:
  virtual bool on_activate();
  virtual bool on_deactivate();

#ifndef ENABLE_SDL
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
