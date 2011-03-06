/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
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

#include "Replay/IGCParser.hpp"
#include "IO/FileLineReader.hpp"
#include "Engine/Math/Earth.hpp"
#include "Engine/Trace/Trace.hpp"
#include "Engine/Task/Tasks/ContestManager.hpp"
#include "Engine/Navigation/Aircraft.hpp"
#include "Printing.hpp"

#include <assert.h>
#include <stdio.h>

Trace full_trace;
Trace sprint_trace(9000, 300);
unsigned handicap = 100;

ContestResult stats_classic;
ContestManager olc_classic(OLC_Classic, handicap, 
                           stats_classic,
                           full_trace, sprint_trace);

ContestResult stats_fai;
ContestManager olc_fai(OLC_FAI, handicap, stats_fai,
                       full_trace, sprint_trace);

ContestResult stats_sprint;
ContestManager olc_sprint(OLC_Sprint, handicap, 
                          stats_sprint,
                          full_trace, sprint_trace);

ContestResult stats_league;
ContestManager olc_league(OLC_League, handicap, 
                          stats_league,
                          full_trace, sprint_trace);

ContestResult stats_plus;
ContestManager olc_plus(OLC_Plus, handicap, 
                        stats_plus,
                        full_trace, sprint_trace);

static void
on_advance(const GeoPoint &loc, const fixed speed,
           const Angle bearing, const fixed alt,
           const fixed baroalt, const fixed t)
{
  AIRCRAFT_STATE new_state;
  new_state.Location = loc;
  new_state.Speed = speed;
  new_state.NavAltitude = alt;
  new_state.TrackBearing = bearing;
  new_state.Time = t;
  new_state.AltitudeAGL = alt;

  full_trace.append(new_state);
  sprint_trace.append(new_state);

  full_trace.optimise_if_old();
  sprint_trace.optimise_if_old();
}

static int
TestOLC(const char *filename)
{
  FileLineReader reader(filename);
  if (reader.error()) {
    fprintf(stderr, "Failed to open %s\n", filename);
    return EXIT_FAILURE;
  }

  TCHAR *line;
  for (int i = 1; (line = reader.read()) != NULL; i++) {
    if (i % 500 == 0) {
      putchar('.');
      fflush(stdout);
    }

    IGCFix fix;
    if (!IGCParseFix(line, fix))
      continue;

    on_advance(fix.location, fixed(30), Angle::native(fixed_zero),
               fix.gps_altitude, fix.pressure_altitude,
               fix.time);

    olc_classic.update_idle();
    olc_fai.update_idle();
    olc_sprint.update_idle();
    olc_league.update_idle();
    olc_plus.update_idle();
  }

  olc_classic.solve_exhaustive();

  putchar('\n');

  std::cout << "classic\n";
  PrintHelper::print(stats_classic);
  std::cout << "league\n";
  PrintHelper::print(stats_league);
  std::cout << "fai\n";
  PrintHelper::print(stats_fai);
  std::cout << "sprint\n";
  PrintHelper::print(stats_sprint);
  std::cout << "plus\n";
  PrintHelper::print(stats_plus);

  return 0;
}


int main(int argc, char **argv)
{
  assert(argc >= 2);

  return TestOLC(argv[1]);
}
