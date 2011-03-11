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

#ifndef XCSOAR_FORM_UTIL_HPP
#define XCSOAR_FORM_UTIL_HPP

#include "Units.hpp"
#include "Compiler.h"

#include <tchar.h>

class WndForm;

/**
 * Loads the specified value into the form.
 *
 * @param form the form
 * @param control_name the name of the control in the form
 * @param value the new value
 */
void
LoadFormProperty(WndForm &form, const TCHAR *control_name, bool value);
void
LoadFormProperty(WndForm &form, const TCHAR *control_name, int value);
void
LoadFormProperty(WndForm &form, const TCHAR *control_name, unsigned int value);

void
LoadFormProperty(WndForm &form, const TCHAR *control_name, fixed value);

void
LoadFormProperty(WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, int value);

static inline void
LoadFormProperty(WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, unsigned value)
{
  LoadFormProperty(form, control_name, unit_group, (int)value);
}

void
LoadFormProperty(WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, fixed value);

gcc_pure
int
GetFormValueInteger(const WndForm &form, const TCHAR *control_name);

gcc_pure
bool
GetFormValueBoolean(const WndForm &form, const TCHAR *control_name);

gcc_pure
fixed
GetFormValueFixed(const WndForm &form, const TCHAR *control_name);

bool
SaveFormProperty(const WndForm &form, const TCHAR* field, bool &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR* field, unsigned int &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR* field, int &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR* field, short &value);

bool
SaveFormProperty(WndForm &form, const TCHAR *control_name, fixed &value);

#ifdef FIXED_MATH
bool
SaveFormProperty(WndForm &form, const TCHAR *control_name, double &value);
#endif

bool
SaveFormProperty(const WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, fixed &value);

/**
 * Saves a form value into a variable and into the registry.
 *
 * @param form the form
 * @param control_name the name of the control in the form
 * @param value the new value
 * @param registry_name the name of the registry key
 * @return true if the value has been modified
 */
bool
SaveFormProperty(const WndForm &form, const TCHAR *control_name,
                 bool &value, const TCHAR *registry_name);

bool
SaveFormProperty(const WndForm &form, const TCHAR *field, const TCHAR *reg,
                 bool &value);

/**
 * Same as SaveFormProperty(), but negates the input value.
 */
bool
SaveFormPropertyNegated(const WndForm &form, const TCHAR *field,
                        const TCHAR *profile_key, bool &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR *field, const TCHAR *reg,
                 unsigned int &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR *field, const TCHAR *reg,
                 int &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR *field, const TCHAR *reg,
                 short &value);

bool
SaveFormProperty(const WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, int &value,
                 const TCHAR *registry_name);

bool
SaveFormProperty(const WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, unsigned &value,
                 const TCHAR *registry_name);

bool
SaveFormProperty(const WndForm &form, const TCHAR *control_name,
                 UnitGroup_t unit_group, fixed &value,
                 const TCHAR *registry_name);

template<typename T>
static inline bool
SaveFormPropertyEnum(const WndForm &form, const TCHAR *field,
                     T &value)
{
  int value2 = (int)value;
  if (!SaveFormProperty(form, field, value2))
    return false;

  value = (T)value2;
  return true;
}

template<typename T>
static inline bool
SaveFormPropertyEnum(const WndForm &form, const TCHAR *field, const TCHAR *reg,
                     T &value)
{
  int value2 = (int)value;
  if (!SaveFormProperty(form, field, reg, value2))
    return false;

  value = (T)value2;
  return true;
}

#endif
