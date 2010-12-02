/* Copyright_License {

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

#include "Screen/Ramp.hpp"
#include "TestUtil.hpp"

int main(int argc, char **argv)
{
  const COLORRAMP ramp[] = {
    {0,    0xff, 0x80, 0x00},
    {1000, 0x00, 0x40, 0xcc},
  };
  Color color;

  plan_tests(9);

  // Test lower limit
  color = ColorRampLookup(0, ramp, 2);
  ok1(color.red() == 0xff);
  ok1(color.green() == 0x80);
  ok1(color.blue() == 0x00);

  // Test upper limit
  color = ColorRampLookup(1000, ramp, 2);
  ok1(color.red() == 0x00);
  ok1(color.green() == 0x40);
  ok1(color.blue() == 0xcc);

  // Test middle
  color = ColorRampLookup(500, ramp, 2);
  ok1(color.red() == 0x7f);
  ok1(color.green() == 0x60);
  ok1(color.blue() == 0x66);

  return exit_status();
}
