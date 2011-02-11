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

#include "Device/Parser.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"

#include <stdio.h>

/**
 * This function creates some simulated traffic for FLARM debugging
 * @param GPS_INFO Pointer to the NMEA_INFO struct
 */
void NMEAParser::TestRoutine(NMEA_INFO *GPS_INFO) {
  static int i = 90;

  i++;
  if (i > 255)
    i = 0;

  if (i > 80)
    return;

  const Angle angle = Angle::degrees(fixed((i * 360) / 255)).as_bearing();

  // PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>,
  //   <RelativeVertical>,<RelativeDistance>(,<ID>)
  int h1;
  int n1;
  int e1;
  int t1;
  unsigned l;
  h1 = (angle.ifastsine()) / 7;
  n1 = (angle.ifastsine()) / 2 - 200;
  e1 = (angle.ifastcosine()) / 1.5;
  t1 = -angle.as_bearing().value_degrees();

  l = (i % 30 > 13 ? 0 : (i % 30 > 5 ? 2 : 1));
  int h2;
  int n2;
  int e2;
  int t2;
  Angle dangle = (angle + Angle::degrees(fixed(120))).as_bearing();
  Angle hangle = dangle.flipped().as_bearing();

  h2 = (angle.ifastcosine()) / 10;
  n2 = (dangle.ifastsine()) / 1.20 + 300;
  e2 = (dangle.ifastcosine()) + 500;
  t2 = hangle.value_degrees();

  // PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,
  //   <IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<AcftType>
  char t_laa1[50];
  sprintf(t_laa1, "%d,%d,%d,%d,2,DDA85C,%d,0,35,0,1", l, n1, e1, h1, t1);
  char t_laa2[50];
  sprintf(t_laa2, "0,%d,%d,%d,2,AA9146,%d,0,27,0,1", n2, e2, h2, t2);

  char t_lau[50];
  sprintf(t_lau, "2,1,2,1,%d", l);

  NMEAInputLine line(t_lau);
  PFLAU(line, GPS_INFO->flarm, GPS_INFO->Time);

  line = NMEAInputLine(t_laa1);
  PFLAA(line, GPS_INFO);

  line = NMEAInputLine(t_laa2);
  PFLAA(line, GPS_INFO);
}
