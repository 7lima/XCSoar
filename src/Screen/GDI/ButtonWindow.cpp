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

#include "Screen/ButtonWindow.hpp"

#include <commctrl.h>

void
BaseButtonWindow::set(ContainerWindow &parent, const TCHAR *text, unsigned id,
                      int left, int top, unsigned width, unsigned height,
                      const WindowStyle style)
{
  Window::set(&parent, WC_BUTTON, text,
              left, top, width, height,
              style);

  ::SetWindowLong(hWnd, GWL_ID, id);

  install_wndproc();
}

bool
BaseButtonWindow::on_clicked()
{
  return false;
}

void
ButtonWindow::set_text(const TCHAR *_text) {
  assert_none_locked();
  assert_thread();

  TCHAR buffer[256]; /* should be large enough for buttons */
  static unsigned const int buffer_size=sizeof(buffer)/sizeof(buffer[0]);

  TCHAR const *s=_text;
  TCHAR *d=buffer;

  // Workaround WIN32 special use of '&' (replace every '&' with "&&")
  // Note: Terminates loop two chars before the buffer_size. This might prevent
  // potential char copies but assures that there is always room for
  // two '&'s and the 0-terminator.
  while (*s && d < buffer + buffer_size - 2) {
    if (*s == _T('&'))
      *d++ = *s;
    *d++ = *s++;
  }
  *d=0;

  ::SetWindowText(hWnd, buffer);
}

const tstring
ButtonWindow::get_text() const
{
  assert_none_locked();
  assert_thread();

  TCHAR buffer[256]; /* should be large enough for buttons */

  int length = GetWindowText(hWnd, buffer,
                             sizeof(buffer) / sizeof(buffer[0]));
  return tstring(buffer, length);
}
