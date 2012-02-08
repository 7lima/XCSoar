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

#ifndef XCSOAR_LOG_FILE_HPP
#define XCSOAR_LOG_FILE_HPP

#include "Compiler.h"

#include <tchar.h>

#if !defined(NDEBUG) && !defined(GNAV)

#define LogDebug(...) LogStartUp(__VA_ARGS__)

#else /* NDEBUG */

/* not using an empty inline function here because we don't want to
   evaluate the parameters */
#define LogDebug(...)

#endif /* NDEBUG */

/**
 * Saves the given string (Str) to the logfile
 * @param Str String to be logged
 */
#ifndef _UNICODE
gcc_printf(1, 2)
#endif
void LogStartUp(const TCHAR *Str, ...);

#endif
