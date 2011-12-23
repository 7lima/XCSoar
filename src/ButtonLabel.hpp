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

#ifndef BUTTON_LABEL_HPP
#define BUTTON_LABEL_HPP

#include "Screen/Point.hpp"

#include <tchar.h>

class Font;
class ContainerWindow;
class MenuBar;

class ButtonLabel
{
protected:
  static MenuBar *bar;

public:
  static void CreateButtonLabels(ContainerWindow &parent);
  static void SetFont(const Font &Font);
  static void Destroy();
  static void SetLabelText(unsigned i, const TCHAR *text);
  static bool IsEnabled(unsigned i);
  static void OnResize(const PixelRect &rc);

  static bool ExpandMacros(const TCHAR *In, TCHAR *OutBuffer, size_t Size);
};

#endif
