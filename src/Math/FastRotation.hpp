/*
Copyright_License {

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

#ifndef XCSOAR_MATH_FASTROTATION_HPP
#define XCSOAR_MATH_FASTROTATION_HPP

#include "Compiler.h"
#include "Math/Angle.hpp"

#include <utility>

/**
 * Rotate coordinates around the zero origin.
 */
class FastRotation {
  Angle angle;
  fixed cost, sint;

public:
  typedef std::pair<fixed,fixed> Pair;

  FastRotation()
    :angle(Angle::native(fixed_zero)), cost(1), sint(0) {}
  FastRotation(Angle _angle):angle(Angle::radians(-fixed(9999))) { SetAngle(_angle); }

  Angle GetAngle() const {
    return angle;
  }

  /**
   * Sets the new angle, and precalculates the sine/cosine values.
   *
   * @param _angle an angle between 0 and 360
   */
  void SetAngle(Angle _angle);

  const FastRotation &operator =(Angle _angle) {
    SetAngle(_angle);
    return *this;
  }

  /**
   * Rotates the point (xin, yin).
   *
   * @param x X value
   * @param y Y value
   * @return the rotated coordinates
   */
  gcc_pure
  Pair Rotate(fixed x, fixed y) const;

  gcc_pure
  Pair Rotate(const Pair p) const {
    return Rotate(p.first, p.second);
  }
};

/**
 * Same as #FastRotation, but works with integer coordinates.
 */
class FastIntegerRotation {
  Angle angle;
  int cost, sint;

public:
  typedef std::pair<int,int> Pair;

  FastIntegerRotation()
 :angle(Angle::native(fixed_zero)), cost(1024), sint(0) {}
  FastIntegerRotation(Angle _angle):angle(Angle::radians(-fixed(9999))) { SetAngle(_angle); }

  Angle GetAngle() const {
    return angle;
  }

  void SetAngle(Angle _angle);

  const FastIntegerRotation &operator =(Angle _angle) {
    SetAngle(_angle);
    return *this;
  }

  /**
   * Rotates the point (xin, yin).
   *
   * @param x X value
   * @param y Y value
   * @return the rotated coordinates
   */
  gcc_pure
  Pair Rotate(int x, int y) const;

  gcc_pure
  Pair Rotate(const Pair p) const {
    return Rotate(p.first, p.second);
  }
};

#endif
