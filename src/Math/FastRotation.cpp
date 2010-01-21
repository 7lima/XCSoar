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

#include "Math/FastRotation.hpp"
#include "Math/Geometry.hpp"
#include "Math/FastMath.h"

#include <math.h>

void
FastRotation::SetAngle(fixed _angle)
{
  _angle = AngleLimit360(_angle);
  if (_angle == angle)
    return;

  angle = _angle;
  cost = fastcosine(angle);
  sint = fastsine(angle);
}

FastRotation::Pair
FastRotation::Rotate(double x, double y) const
{
  return Pair(x * cost - y * sint, y * cost + x * sint);
}

void
FastIntegerRotation::SetAngle(fixed _angle)
{
  _angle = AngleLimit360(_angle);
  if (_angle == angle)
    return;

  angle = _angle;
  cost = ifastcosine(angle);
  sint = ifastsine(angle);
}

FastIntegerRotation::Pair
FastIntegerRotation::Rotate(int x, int y) const
{
  return Pair((x * cost - y * sint + 512) / 1024,
              (y * cost + x * sint + 512) / 1024);
}


/**
 * Detects if angle (x) is between two other angles (Angle0 and Angle1)
 * @param Angle0 Limit angle 1
 * @param Angle1 Limit angle 2
 * @param x Input angle
 * @param is_signed Is the input angle signed?
 * @return True if between, False if not
 */
bool AngleInRange(double Angle0, double Angle1, double x, bool is_signed) {
  Angle0 = AngleLimit360(Angle0);
  Angle1 = AngleLimit360(Angle1);
  x = AngleLimit360(x);

  if (Angle1>= Angle0) {
    if ((x>=Angle0) && (x<= Angle1)) {
      return true;
    }
  } else {
    if (is_signed) {
      if ((x>=Angle0) || (x<= Angle1)) {
        return true;
      }
    } else {
      if ((x<=Angle0) || (x>= Angle1)) {
        return true;
      }
    }
  }
  return false;
}
