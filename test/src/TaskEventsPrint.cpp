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
#include "TaskEventsPrint.hpp"
#include "Task/Tasks/BaseTask/TaskPoint.hpp"

#ifdef DO_PRINT
#include <stdio.h>
#endif

void 
TaskEventsPrint::transition_enter(const TaskPoint& tp) 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- entered sector\n");
#endif
  }
}

void 
TaskEventsPrint::transition_exit(const TaskPoint &tp) 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- exited sector\n");
#endif
  }
}


void 
TaskEventsPrint::active_changed(const TaskPoint &tp) 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- active changed to wp %d\n", tp.get_waypoint().id);
#endif
  }
}

void 
TaskEventsPrint::construction_error(const char* error) 
{
  if (verbose>2) {
#ifdef DO_PRINT
    printf("#Task construction error: ");
    printf("#%s",error);
    printf("\n");
#endif
  }
}

void 
TaskEventsPrint::warning_start_speed() 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- warning speed excessive\n");
#endif
  }
}

void 
TaskEventsPrint::request_arm(const TaskPoint &tp)  
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- ready to advance\n");
#endif
  }
}

void 
TaskEventsPrint::task_start() 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- task started\n");
#endif
  }
}

void 
TaskEventsPrint::task_finish() 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- task finished\n");
#endif
  }
}


void 
TaskEventsPrint::active_advanced(const TaskPoint &tp, const int i) 
{
  if (verbose) {
#ifdef DO_PRINT
    printf("#- advance to sector %d\n", i);
#endif
  }
}

void 
TaskEventsPrint::transition_flight_mode(const bool is_final)
{
  if (verbose) {
#ifdef DO_PRINT
    if (is_final) {
      printf("#- flight mode final glide \n");
    } else {
      printf("#- flight mode cruise \n");
    }
#endif
  }
}
