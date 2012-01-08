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

#include "Screen/TopWindow.hpp"
#include "PeriodClock.hpp"

#ifdef ANDROID
#include "Screen/Android/Event.hpp"
#include "Screen/OpenGL/Surface.hpp"
#include "Android/Main.hpp"
#include "Android/NativeView.hpp"
#else
#include "Screen/SDL/Event.hpp"
#endif

TopWindow::TopWindow()
  :invalidated(true)
#ifdef ANDROID
  , paused(false), resumed(false), resized(false)
#endif
{
}

bool
TopWindow::find(const TCHAR *cls, const TCHAR *text)
{
  return false; // XXX
}

void
TopWindow::set(const TCHAR *cls, const TCHAR *text,
               PixelScalar left, PixelScalar top,
               UPixelScalar width, UPixelScalar height)
{
  WindowStyle style;
  style.Popup();

  screen.Set(width, height);

  /* apply the mode which was chosen by TopCanvas */
  width = screen.get_width();
  height = screen.get_height();

  ContainerWindow::set(NULL, 0, 0, width, height, style);

#ifndef ANDROID
#ifdef _UNICODE
  char text2[_tcslen(text) * 4];
  ::WideCharToMultiByte(CP_UTF8, 0, text, -1, text2, sizeof(text2),
                        NULL, NULL);
#else
  const char *text2 = text;
#endif

  ::SDL_WM_SetCaption(text2, NULL);
#endif
}

void
TopWindow::full_screen()
{
  screen.Fullscreen();
}

void
TopWindow::invalidate()
{
  invalidated_lock.Lock();
  if (invalidated) {
    /* already invalidated, don't send the event twice */
    invalidated_lock.Unlock();
    return;
  }

  invalidated = true;
  invalidated_lock.Unlock();

  /* wake up the event loop */
#ifdef ANDROID
  event_queue->Push(Event::NOP);
#else
  /* note that SDL_NOEVENT is not documented, but since we just want
     to wake up without actually sending an event, I hope this works
     on all future SDL versions; if SDL_NOEVENT ever gets remove, I'll
     have to come up with something else */
  SDL_Event event;
  event.type = SDL_NOEVENT;
  ::SDL_PushEvent(&event);
#endif
}

void
TopWindow::expose() {
  OnPaint(screen);
  screen.Flip();
}

void
TopWindow::refresh()
{
#ifdef ANDROID
  if (resumed) {
    /* Try to reinitialize OpenGL.  This often fails on the first
       attempt (IllegalArgumentException "Make sure the SurfaceView or
       associated SurfaceHolder has a valid Surface"), therefore we're
       trying again until we're successful. */

    assert(paused);

    if (!native_view->initSurface())
      /* failed - retry later */
      return;

    paused = false;
    resumed = false;

    screen.Set();

    SurfaceCreated();

    if (resized) {
      resized = false;
      resize(new_width, new_height);
    }
  }

  if (paused || !surface_valid)
    /* the application is paused/suspended, and we don't have an
       OpenGL surface - ignore all drawing requests */
    return;
#endif

  invalidated_lock.Lock();
  if (!invalidated) {
    invalidated_lock.Unlock();
    return;
  }

  invalidated = false;
  invalidated_lock.Unlock();

  expose();
}

bool
TopWindow::on_activate()
{
  return false;
}

bool
TopWindow::on_deactivate()
{
  return false;
}

bool
TopWindow::OnClose()
{
  reset();
  return true;
}

#ifndef ANDROID

bool
TopWindow::on_event(const SDL_Event &event)
{
  switch (event.type) {
    Window *w;

  case SDL_VIDEOEXPOSE:
    invalidated_lock.Lock();
    invalidated = false;
    invalidated_lock.Unlock();

    expose();
    return true;

  case SDL_KEYDOWN:
    w = get_focused_window();
    if (w == NULL)
      w = this;

    return w->OnKeyDown(event.key.keysym.sym);

  case SDL_KEYUP:
    w = get_focused_window();
    if (w == NULL)
      w = this;

    return w->OnKeyUp(event.key.keysym.sym);

  case SDL_MOUSEMOTION:
    // XXX keys
    return OnMouseMove(event.motion.x, event.motion.y, 0);

  case SDL_MOUSEBUTTONDOWN:
    if (event.button.button == SDL_BUTTON_WHEELUP)
      return OnMouseWheel(event.button.x, event.button.y, 1);
    else if (event.button.button == SDL_BUTTON_WHEELDOWN)
      return OnMouseWheel(event.button.x, event.button.y, -1);

    static PeriodClock double_click;
    return double_click.check_always_update(300)
      ? OnMouseDown(event.button.x, event.button.y)
      : OnMouseDouble(event.button.x, event.button.y);

  case SDL_MOUSEBUTTONUP:
    if (event.button.button == SDL_BUTTON_WHEELUP ||
        event.button.button == SDL_BUTTON_WHEELDOWN)
      /* the wheel has already been handled in SDL_MOUSEBUTTONDOWN */
      return false;

    return OnMouseUp(event.button.x, event.button.y);

  case SDL_QUIT:
    return OnClose();
  }

  return false;
}

int
TopWindow::event_loop()
{
  refresh();

  EventLoop loop(*this);
  SDL_Event event;
  while (defined() && loop.Get(event))
    loop.Dispatch(event);

  return 0;
}

void
TopWindow::post_quit()
{
  SDL_Event event;
  event.type = SDL_QUIT;
  ::SDL_PushEvent(&event);
}

#endif /* !ANDROID */
