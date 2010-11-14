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

#include "Screen/ProgressWindow.hpp"
#include "Screen/VirtualCanvas.hpp"
#include "Gauge/LogoView.hpp"
#include "resource.h"

#include <algorithm>

using std::min;

ProgressWindow::ProgressWindow(ContainerWindow &parent)
  :background_color(Color::WHITE),
   background_brush(background_color),
   position(0)
{
  RECT rc = parent.get_client_rect();
  set(parent, rc.left, rc.top, rc.right, rc.bottom);

  unsigned width = rc.right - rc.left, height = rc.bottom - rc.top;

  // Load progress bar background
  bitmap_progress_border.load(IDB_PROGRESSBORDER);

  // Determine text height
#ifdef ENABLE_SDL
  text_height = 20; // XXX font bootstrapping on SDL needed
#else
  VirtualCanvas canvas(1, 1);
  text_height = canvas.text_height(_T("W"));
#endif

  // Make progress bar height proportional to window height
  unsigned progress_height = height / 20;
  unsigned progress_horizontal_border = progress_height / 2;
  progress_border_height = progress_height * 2;

  // Initialize message text field
  TextWindowStyle message_style;
  message_style.center();
  message.set(*this, NULL, 0,
              height - progress_border_height - text_height - (height/48),
              width, text_height, message_style);

  // Initialize progress bar
  ProgressBarStyle pb_style;
  progress_bar.set(*this, progress_horizontal_border,
                   height - progress_border_height + progress_horizontal_border,
                   width - progress_height,
                   progress_height, pb_style);

  message.install_wndproc(); // needed for on_color()

  // Set progress bar step size and range
  set_range(0, 1000);
  set_step(50);

  // Show dialog
  bring_to_top();
  update();
}

void
ProgressWindow::set_message(const TCHAR *text)
{
  assert_none_locked();
  assert_thread();

  message.set_text(text);
}

void
ProgressWindow::set_range(unsigned min_value, unsigned max_value)
{
  progress_bar.set_range(min_value, max_value);
}

void
ProgressWindow::set_step(unsigned size)
{
  progress_bar.set_step(size);
}

void
ProgressWindow::set_pos(unsigned value)
{
  assert_none_locked();
  assert_thread();

  if (value == position)
    return;

  position = value;
  progress_bar.set_position(value);
}

void
ProgressWindow::step()
{
  progress_bar.step();
}

void
ProgressWindow::on_paint(Canvas &canvas)
{
  canvas.clear(background_color);

  // Determine window size
  int window_width = canvas.get_width();
  int window_height = canvas.get_height();

  RECT logo_rect;
  logo_rect.left = 0;
  logo_rect.top = 0;
  logo_rect.right = window_width;
  logo_rect.bottom = window_height - progress_border_height;
  DrawLogo(canvas, logo_rect);

  // Draw progress bar background
  canvas.stretch(0, (window_height - progress_border_height),
                 window_width, progress_border_height,
                 bitmap_progress_border);

  ContainerWindow::on_paint(canvas);
}

Brush *
ProgressWindow::on_color(Window &window, Canvas &canvas)
{
  canvas.set_text_color(Color::BLACK);
  canvas.set_background_color(background_color);
  return &background_brush;
}
