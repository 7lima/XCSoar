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

#include "Screen/Window.hpp"
#include "Screen/ContainerWindow.hpp"

#ifdef ANDROID
#include "Screen/Android/Event.hpp"
#include "Android/Main.hpp"
#elif defined(ENABLE_SDL)
#include "Screen/SDL/Event.hpp"
#endif /* ENABLE_SDL */

#ifdef ENABLE_OPENGL
#include "Screen/OpenGL/Debug.hpp"
#endif

#include <assert.h>

Window::~Window()
{
  reset();
}

#ifndef NDEBUG

void
Window::assert_thread() const
{
#ifdef ENABLE_OPENGL
  assert(pthread_equal(pthread_self(), OpenGL::thread));
#elif !defined(ENABLE_SDL)
  assert(hWnd != NULL);
  assert(!::IsWindow(hWnd) ||
         ::GetWindowThreadProcessId(hWnd, NULL) == ::GetCurrentThreadId());
#endif
}

#endif /* !NDEBUG */

void
Window::set(ContainerWindow *parent, const TCHAR *cls, const TCHAR *text,
            int left, int top, unsigned width, unsigned height,
            const WindowStyle window_style)
{
  assert(width > 0);
  assert(width < 0x1000000);
  assert(height > 0);
  assert(height < 0x1000000);

  double_clicks = window_style.double_clicks;

#ifdef ENABLE_SDL
  this->parent = parent;
  this->left = left;
  this->top = top;
  this->width = width;
  this->height = height;

  visible = window_style.visible;
  text_style = window_style.text_style;

  if (parent != NULL)
    parent->add_child(*this);

  on_create();
  on_resize(width, height);
#else /* !ENABLE_SDL */
  DWORD style = window_style.style, ex_style = window_style.ex_style;

  if (window_style.custom_painting)
    enable_custom_painting();

  hWnd = ::CreateWindowEx(ex_style, cls, text, style,
                          left, top, width, height,
                          parent != NULL ? parent->hWnd : NULL,
                          NULL, NULL, this);

  /* this isn't good error handling, but this only happens if
     out-of-memory (we can't do anything useful) or if we passed wrong
     arguments - which is a bug */
  assert(hWnd != NULL);
#endif /* !ENABLE_SDL */
}

void
Window::reset()
{
  if (!defined())
    return;

  assert_thread();

#ifdef ENABLE_SDL
  on_destroy();

  width = 0;
  height = 0;
#else /* !ENABLE_SDL */
  ::DestroyWindow(hWnd);

  /* the on_destroy() method must have cleared the variable by
     now */
  assert(prev_wndproc == NULL || hWnd == NULL);

  hWnd = NULL;
  prev_wndproc = NULL;
#endif /* !ENABLE_SDL */
}

ContainerWindow *
Window::get_root_owner()
{
#ifdef ENABLE_SDL
  if (parent == NULL)
    /* no parent?  We must be a ContainerWindow instance */
    return (ContainerWindow *)this;

  ContainerWindow *root = parent;
  while (root->parent != NULL)
    root = root->parent;

  return root;
#else /* !ENABLE_SDL */
#ifndef _WIN32_WCE
  HWND hRoot = ::GetAncestor(hWnd, GA_ROOTOWNER);
  if (hRoot == NULL)
    return NULL;
#else
  HWND hRoot = hWnd;
  while (true) {
    HWND hParent = ::GetParent(hRoot);
    if (hParent == NULL)
      break;
    hRoot = hParent;
  }
#endif

  /* can't use the "checked" method get() because hRoot may be a
     dialog, and uses Dialog::DlgProc() */
  return (ContainerWindow *)get_unchecked(hRoot);
#endif /* !ENABLE_SDL */
}

bool
Window::on_create()
{
  return true;
}

bool
Window::on_destroy()
{
#ifdef ENABLE_SDL
  if (capture)
    release_capture();

  if (parent != NULL) {
    parent->remove_child(*this);
    parent = NULL;
  }

#ifdef ANDROID
  event_queue->purge(*this);
#else
  EventQueue::purge(*this);
#endif
#else /* !ENABLE_SDL */
  assert(hWnd != NULL);

  hWnd = NULL;
#endif /* !ENABLE_SDL */

  return true;
}

bool
Window::on_close()
{
  return false;
}

bool
Window::on_resize(unsigned width, unsigned height)
{
  return false;
}

bool
Window::on_mouse_move(int x, int y, unsigned keys)
{
  /* not handled here */
  return false;
}

bool
Window::on_mouse_down(int x, int y)
{
  return false;
}

bool
Window::on_mouse_up(int x, int y)
{
  return false;
}

bool
Window::on_mouse_double(int x, int y)
{
#ifdef ENABLE_SDL
  if (!double_clicks)
    return on_mouse_down(x, y);
#endif

  return false;
}

bool
Window::on_mouse_wheel(int delta)
{
  return false;
}

bool
Window::on_key_check(unsigned key_code) const
{
  return false;
}

bool
Window::on_key_down(unsigned key_code)
{
  return false;
}

bool
Window::on_key_up(unsigned key_code)
{
  return false;
}

bool
Window::on_command(unsigned id, unsigned code)
{
  return false;
}

bool
Window::on_cancel_mode()
{
#ifdef ENABLE_SDL
  release_capture();
#endif

  return false;
}

bool
Window::on_setfocus()
{
#ifdef ENABLE_SDL
  assert(!focused);

  focused = true;
  return true;
#else /* !ENABLE_SDL */
  return false;
#endif /* !ENABLE_SDL */
}

bool
Window::on_killfocus()
{
#ifdef ENABLE_SDL
  assert(focused);

#ifdef ENABLE_SDL
  release_capture();
#endif

  focused = false;
  return true;
#else /* !ENABLE_SDL */
  return false;
#endif /* !ENABLE_SDL */
}

bool
Window::on_timer(timer_t id)
{
  return false;
}

bool
Window::on_user(unsigned id)
{
  return false;
}

bool
Window::on_erase(Canvas &canvas)
{
  /* if on_paint() is implemented, then don't erase the background;
     on_paint() will paint on top */
#ifdef ENABLE_SDL
  return false;
#else
  return custom_painting;
#endif
}

void
Window::on_paint(Canvas &canvas)
{
}

void
Window::on_paint(Canvas &canvas, const RECT &dirty)
{
  on_paint(canvas);
}
