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

#ifndef XCSOAR_MATH_CONSTANTS_HPP
#define XCSOAR_MATH_CONSTANTS_HPP

#include <math.h>

#define DEG_TO_RAD .0174532925199432958
#define RAD_TO_DEG 57.2957795131

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//#define   BAM_270_DEG       0xc0000000L    // BAM representation of 270 deg
//#define   BAM_180_DEG       0x80000000L    // BAM representation of 180 deg
//#define   NOT_BAM_180_DEG   0x7fffffffL    // BAM representation of 180 deg-
//#define   BAM_90_DEG        0x40000000L    // BAM representation of  90 deg
//#define   MINUS_BAM_90_DEG  0xc0000000L    // BAM representation of -90 deg
//#define   R                 3.43775e+05    // Radius of earth in 1/100 NM
//#define   TWO_R             6.87550e+05    // Diameter of earth in 1/100 NM
//#define   TWO_R_SQUARED     4.72725e+11    // TWO_R * TWO_R
//#define   TWO_R_HP          6.87550e+07    // Diam. of earth in 1/10000 NM
//#define   SQUARED_TWO_R_HP  4.72725e+15    // TWO_R_HP * TWO_R_HP
//#define BAM_TO_RAD 1.462918079e-9          // BAM to radians
//#define RAD_TO_BAM 6.835652757e+8          // Radians to BAM
//#define RAD_TO_BAM_DIV_2  0.5*RAD_TO_BAM

/** Diameter of earth in meters */
#define EARTH_DIAMETER 12733426.0
/** Diameter of earth in meters (squared) (EARTH_DIAMETER * EARTH_DIAMETER) */
#define SQUARED_EARTH_DIAMETER 162140137697476.0

#endif
