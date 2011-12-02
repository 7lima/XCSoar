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

#include "Units/UnitsStore.hpp"
#include "Units/Units.hpp"
#include "Language/Language.hpp"
#include "Util/Macros.hpp"

struct UnitStoreItem
{
  const TCHAR* Name;
  UnitSetting Units;
};

static gcc_constexpr_data UnitStoreItem Presets[] =
{
  { N_("European"), {
    unKiloMeter,
    unMeter,
    unGradCelcius,
    unKiloMeterPerHour,
    unMeterPerSecond,
    unKiloMeterPerHour,
    unKiloMeterPerHour,
    unHectoPascal,
  } },
  { N_("British"), {
    unKiloMeter,
    unFeet,
    unGradCelcius,
    unKnots,
    unKnots,
    unKnots,
    unKiloMeterPerHour,
    unMilliBar,
  } },
  { N_("American"), {
    unStatuteMiles,
    unFeet,
    unGradFahrenheit,
    unKnots,
    unKnots,
    unKnots,
    unStatuteMilesPerHour,
    unInchMercury,
  } },
  { N_("Australian"), {
    unKiloMeter,
    unFeet,
    unGradCelcius,
    unKnots,
    unKnots,
    unKnots,
    unKiloMeterPerHour,
    unHectoPascal,
  } }
};

const TCHAR*
Units::Store::GetName(unsigned i)
{
  assert(i < Count());
  return gettext(Presets[i].Name);
}

const UnitSetting&
Units::Store::Read(unsigned i)
{
  assert(i < Count());
  return Presets[i].Units;
}

unsigned
Units::Store::Count()
{
  return ARRAY_SIZE(Presets);
}
