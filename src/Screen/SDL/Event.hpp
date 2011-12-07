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

#ifndef XCSOAR_SCREEN_SDL_EVENT_HPP
#define XCSOAR_SCREEN_SDL_EVENT_HPP

#include "Util/NonCopyable.hpp"

#include <SDL_version.h>
#include <SDL_events.h>

class TopWindow;
class WindowTimer;
class Window;
class Notify;

enum {
  /**
   * A "user" event for a #Window.
   */
  EVENT_USER = SDL_USEREVENT,

  /**
   * A "timer" event for a #Window.
   */
  EVENT_TIMER,

  /**
   * An event for class #Notify.
   */
  EVENT_NOTIFY,
};

static inline bool
IsUserInput(const SDL_Event &event)
{
  return event.type == SDL_KEYDOWN || event.type == SDL_KEYUP ||
    event.type == SDL_MOUSEMOTION ||
    event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP;
}

class EventLoop : private NonCopyable {
  TopWindow &top_window;

  /**
   * True if working on a bulk of events.  At the end of that bulk,
   * TopWindow::validate() gets called.
   */
  bool bulk;

public:
  EventLoop(TopWindow &_top_window)
    :top_window(_top_window), bulk(true) {}

  bool Get(SDL_Event &event);
  void Dispatch(SDL_Event &event);
};

namespace EventQueue {
  /**
   * Purge all matching events from the event queue.
   */
  void Purge(Uint32 mask,
             bool (*match)(const SDL_Event &event, void *ctx), void *ctx);

  /**
   * Purge all events for this #Notify from the event queue.
   */
  void Purge(Notify &notify);

  /**
   * Purge all events for this Window from the event queue.
   */
  void Purge(Window &window);

  /**
   * Purge all events for this timer from the event queue.
   */
  void Purge(WindowTimer &timer);
};

#endif
