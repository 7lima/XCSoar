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
#ifndef FLATELLIPSE_HPP
#define FLATELLIPSE_HPP

#include "FlatPoint.hpp"
#include "FlatLine.hpp"
#include "Compiler.h"

/**
 * 2-d ellipse in real-valued projected coordinates, with methods for
 * intersection tests etc.  The ellipse itself need not be axis-aligned.
 */
struct FlatEllipse 
{
  /** 
   * Constructor.
   * 
   * @param _f1 Focus A
   * @param _f2 Focus B
   * @param _ap Any point on the ellipse
   * 
   * @return Initialised object
   */
  FlatEllipse(const FlatPoint &_f1,
              const FlatPoint &_f2,
              const FlatPoint &_ap);

/** 
 * Dummy constructor for zero-sized ellipse
 * 
 * @return Initialised object
 */
  FlatEllipse():f1(fixed_zero, fixed_zero),
                f2(fixed_zero, fixed_zero),
                ap(fixed_zero, fixed_zero), 
                p(fixed_zero, fixed_zero),
                a(fixed_one), b(fixed_one), 
                theta(),
                theta_initial()
    {
    };

/** 
 * Parametric representation of ellipse
 * 
 * @param t Parameter [0,1] 
 * 
 * @return Location on ellipse
 */
  gcc_pure
  FlatPoint parametric(const fixed t) const;

/** 
 * Find intersection of line from focus 1 to p, through the ellipse
 * 
 * @param p Reference point
 * @param i1 Intersection point 1 if found
 * @param i2 Intersection point 2 if found
 * 
 * @return True if line intersects
 */
  bool intersect_extended(const FlatPoint &p,
                          FlatPoint &i1,
                          FlatPoint &i2) const;

private:
  FlatPoint f1, f2, ap;
  FlatPoint p;
  fixed a;
  fixed b;
  Angle theta;

  Angle theta_initial;

  gcc_pure
  fixed ab() const;

  gcc_pure
  fixed ba() const;

  bool intersect(const FlatLine &line, 
                 FlatPoint &i1, 
                 FlatPoint &i2) const;

};

#endif
