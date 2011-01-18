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

#ifndef XCSOAR_SCREEN_CONTAINER_WINDOW_HXX
#define XCSOAR_SCREEN_CONTAINER_WINDOW_HXX

#include "Screen/PaintWindow.hpp"

#ifdef ENABLE_SDL
#include <list>
#endif /* !ENABLE_SDL */

/**
 * A container for more #Window objects.  It is also derived from
 * #PaintWindow, because you might want to paint a border between the
 * child windows.
 */
class ContainerWindow : public PaintWindow {
protected:
#ifdef ENABLE_SDL
  std::list<Window*> children;

  /**
   * The active child window is used to find the focused window.  If
   * this attribute is NULL, then the focused window is not an
   * (indirect) child window of this one.
   */
  Window *active_child;

  /**
   * The child window which captures the mouse.
   */
  Window *capture_child;

public:
  ContainerWindow();
  virtual ~ContainerWindow();
#endif /* ENABLE_SDL */

protected:
  virtual Brush *on_color(Window &window, Canvas &canvas);

#ifdef ENABLE_SDL
  virtual bool on_destroy();
  virtual bool on_mouse_move(int x, int y, unsigned keys);
  virtual bool on_mouse_down(int x, int y);
  virtual bool on_mouse_up(int x, int y);
  virtual bool on_mouse_double(int x, int y);
  virtual void on_paint(Canvas &canvas);
#else /* !ENABLE_SDL */
  virtual LRESULT on_message(HWND hWnd, UINT message,
                             WPARAM wParam, LPARAM lParam);
#endif

public:
#ifdef ENABLE_SDL
  void add_child(Window &child) {
    children.push_back(&child);
  }

  void remove_child(Window &child) {
    children.remove(&child);

    if (active_child == &child)
      active_child = NULL;
  }

  void bring_child_to_top(Window &child) {
    children.remove(&child);
    children.insert(children.begin(), &child);
    invalidate();
  }

  /**
   * Locate a child window by its relative coordinates.
   */
  gcc_pure
  Window *child_at(int x, int y);

  /**
   * Locates the child which should get a mouse event.  Prefers the
   * captured child.
   */
  gcc_pure
  Window *event_child_at(int x, int y);

  void set_active_child(Window &child);

  /**
   * Override the Window::get_focused_window() method, and search in
   * the active child window.
   */
  gcc_pure
  virtual Window *get_focused_window();

  void set_child_capture(Window *window);
  void release_child_capture(Window *window);
  virtual void clear_capture();
#endif /* ENABLE_SDL */

  /**
   * Sets the keyboard focus on the first descendant window which has
   * the WindowStyle::tab_stop() attribute.
   */
  void focus_first_control();

  /**
   * Sets the keyboard focus on the next descendant window which has
   * the WindowStyle::tab_stop() attribute.
   */
  void focus_next_control();

  /**
   * Sets the keyboard focus on the previous descendant window which
   * has the WindowStyle::tab_stop() attribute.
   */
  void focus_previous_control();
};

#endif
