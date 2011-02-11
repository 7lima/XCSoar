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

#include "OS/Clock.hpp"

#if defined(HAVE_POSIX) && !defined(__CYGWIN__)
#include <time.h>
#else /* !HAVE_POSIX */
#include <windows.h>
#endif /* !HAVE_POSIX */

unsigned
MonotonicClockMS()
{
#if defined(HAVE_POSIX) && !defined(__CYGWIN__)
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
#else /* !HAVE_POSIX */
  return ::GetTickCount();
#endif /* !HAVE_POSIX */
}
