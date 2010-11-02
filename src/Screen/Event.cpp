/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
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

#include "Screen/Event.hpp"
#include "Screen/TopWindow.hpp"

bool
EventLoop::get(SDL_Event &event)
{
  if (event.type == SDL_QUIT)
    return false;

  if (bulk) {
    if (::SDL_PollEvent(&event))
      return true;

    /* that was the last event for now, refresh the screen now */
    top_window.refresh();
    bulk = false;
  }

  if (::SDL_WaitEvent(&event)) {
    bulk = true;
    return true;
  }

  return false;
}

void
EventLoop::dispatch(SDL_Event &event)
{
  if (event.type == Window::EVENT_USER && event.user.data1 != NULL) {
    Window *window = (Window *)event.user.data1;
    window->on_user(event.user.code);
  } else if (event.type == Window::EVENT_TIMER && event.user.data1 != NULL) {
    Window *window = (Window *)event.user.data1;
    SDLTimer *timer = (SDLTimer *)event.user.data2;
    window->on_timer(timer);
  } else
    ((Window &)top_window).on_event(event);
}
