/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

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

#include "Filter.hpp"
#include <math.h>
#include <assert.h>
#include <stdio.h>

// ref http://unicorn.us.com/alex/2polefilters.html

Filter::Filter(const fixed cutoff_wavelength, const bool bessel) :
  m_bessel(bessel)
{
  design(cutoff_wavelength);
}

bool
Filter::design(const fixed cutoff_wavelength)
{
  fixed sample_freq = fixed_one;
  fixed n = fixed_one;
  fixed c;
  fixed g;
  fixed p;

  if (m_bessel) {
    // Bessel
    c = pow((sqrt(pow(fixed_two, fixed_one / n) - fixed(0.75)) - fixed_half),
            -fixed_half) / sqrt(fixed(3));
    g = 3;
    p = 3;
  } else {
    // Critically damped
    c = pow((pow(fixed_two, fixed_one / (2 * n)) - fixed_one), -fixed_half);
    g = fixed_one;
    p = fixed_two;
  }

  fixed f_star = c / (sample_freq * cutoff_wavelength);

  assert(f_star < fixed_one / 8);

  if (f_star >= fixed_one / 8) {
    ok = false;
    return false;
  }

  fixed omega0 = tan(fixed_pi * f_star);
  fixed K1 = p * omega0;
  fixed K2 = g * omega0 * omega0;

  a[0] = K2 / (fixed_one + K1 + K2);
  a[1] = 2 * a[0];
  a[2] = a[0];
  b[0] = 2 * a[0] * (fixed_one / K2 - fixed_one);
  b[1] = fixed_one - (a[0] + a[1] + a[2] + b[0]);

  reset(fixed_zero);
  ok = true;

  return true;
}

fixed
Filter::reset(const fixed _x)
{
  x[0] = _x;
  y[0] = _x;
  x[1] = _x;
  y[1] = _x;
  x[2] = _x;

  return _x;
}

fixed
Filter::update(const fixed _x)
{
  if (!ok)
    return _x;

  x[2] = x[1];
  x[1] = x[0];
  x[0] = _x;

  fixed _y = a[0] * x[0]
            + a[1] * x[1]
            + a[2] * x[2]
            + b[0] * y[0]
            + b[1] * y[1];

  y[1] = y[0];
  y[0] = _y;

  return _y;
}
