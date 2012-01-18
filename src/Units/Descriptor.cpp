/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#include "Units/Descriptor.hpp"
#include "Units/Units.hpp"
#include "Atmosphere/Temperature.hpp"
#include "Util/Macros.hpp"

//SI to Local Units

const UnitDescriptor Units::unit_descriptors[] = {
  { NULL, fixed_one, fixed_zero },
  { _T("km"), fixed_constant(0.001, 0x41893LL), fixed_zero },
  { _T("NM"), fixed_constant(0.000539956803, 0x2362fLL), fixed_zero },
  { _T("mi"), fixed_constant(0.000621371192, 0x28b8eLL), fixed_zero },
  { _T("km/h"), fixed_constant(3.6, 0x39999999LL), fixed_zero },
  { _T("kt"), fixed_constant(1.94384449, 0x1f19fcaeLL), fixed_zero },
  { _T("mph"), fixed_constant(2.23693629, 0x23ca7db5LL), fixed_zero },
  { _T("m/s"), fixed_one, fixed_zero },
  { _T("fpm"), fixed_constant(196.850394, 0xc4d9b36bdLL), fixed_zero },
  { _T("m"), fixed_one, fixed_zero },
  { _T("ft"), fixed_constant(3.2808399, 0x347e51faLL), fixed_zero },
  { _T("FL"), fixed_constant(0.032808399, 0x866219LL), fixed_zero },
  { _T("K"), fixed_one, fixed_zero },
  { _T(DEG)_T("C"), fixed_one, -CELSIUS_OFFSET },
  { _T(DEG)_T("F"), fixed_constant(1.8, 0x1cccccccLL),
    fixed_constant(-459.67, -123391726059LL) },
  { _T("hPa"), fixed_one, fixed_zero },
  { _T("mb"), fixed_one, fixed_zero },
  { _T("mmHg"), fixed(0.7500616827041698), fixed_zero },
  { _T("inHg"), fixed(0.0295287441401431), fixed_zero },
};

static_assert(ARRAY_SIZE(Units::unit_descriptors) == (size_t)Unit::COUNT,
              "number of unit descriptions does not match number of units");

const TCHAR *
Units::GetUnitName(Unit unit)
{
  const unsigned i = (unsigned)unit;
  assert(i < ARRAY_SIZE(unit_descriptors));
  assert(unit_descriptors[i].name != NULL);

  return unit_descriptors[i].name;
}
