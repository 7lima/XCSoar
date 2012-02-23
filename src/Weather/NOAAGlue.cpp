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

#include "NOAAGlue.hpp"
#include "Net/Features.hpp"

#ifdef HAVE_NET

#include "NOAAStore.hpp"
#include "Profile/Profile.hpp"

#include <tchar.h>

namespace NOAAStore
{
  bool LoadFromString(const TCHAR *string);
};

static bool
IsValidCode(const TCHAR* code)
{
  for (unsigned i = 0; i < 4; ++i)
    if (code[i] < _T('A') || code[i] > _T('Z'))
      return false;

  return true;
}

bool
NOAAStore::LoadFromString(const TCHAR *string)
{
  const TCHAR *s = string;
  while (s != NULL && *s) {
    const TCHAR *next = _tcschr(s, _T(','));
    if ((next != NULL && next - s == 4) || (next == NULL && _tcslen(s) == 4)) {
      TCHAR code[5];
      std::copy(s, s+4, code);
      code[4] = '\0';
      if (IsValidCode(code))
        NOAAStore::AddStation(code);
    }
    s = (next == NULL) ? NULL : next + 1;
  }
  return true;
}

bool
NOAAStore::LoadFromProfile()
{
  TCHAR buffer[120];
  if (!Profile::Get(szProfileWeatherStations, buffer, 120))
    return false;

  return LoadFromString(buffer);
}

void
NOAAStore::SaveToProfile()
{
  TCHAR buffer[120];
  for (unsigned i = 0; i < NOAAStore::Count(); i++) {
    const TCHAR *code = NOAAStore::GetCodeT(i);
    _tcscpy(buffer + (i * 5), code);
    _tcscpy(buffer + (i * 5) + 4, _T(","));
  }

  Profile::Set(szProfileWeatherStations, buffer);
}

#else

bool
NOAAStore::LoadFromProfile()
{
  return false;
}

void
NOAAStore::SaveToProfile()
{
}

#endif
