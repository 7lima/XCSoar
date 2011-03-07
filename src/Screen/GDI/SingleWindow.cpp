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

#include "Screen/SingleWindow.hpp"
#include "Screen/GDI/Event.hpp"
#include "Asset.hpp"

#include <cassert>

static bool
is_allowed_map_message(UINT message)
{
  return message == WM_LBUTTONDOWN || message == WM_LBUTTONUP ||
    message == WM_MOUSEMOVE;
}

static bool
is_allowed_map(HWND hWnd, UINT message, const Window *window)
{
  return !is_altair() && window != NULL && window->identify(hWnd) &&
    is_allowed_map_message(message);
}

bool
SingleWindow::FilterEvent(const MSG &msg, Window *allowed,
                          Window *second_allowed) const
{
  assert(allowed != NULL);

  if (is_user_input(msg.message)) {
    if (allowed->identify_descendant(msg.hwnd))
      /* events to the current modal dialog are allowed */
      return true;

    if (is_allowed_map(msg.hwnd, msg.message, second_allowed))
      /* certain events to the specified second Window are allowed */
      return true;

    return false;
  } else
    return true;
}
