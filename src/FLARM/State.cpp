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

#include "FLARM/State.hpp"

void
FLARM_STATE::clear()
{
  FLARM_Available.clear();
  FLARMTraffic = false;
  NewTraffic = false;

  for (unsigned i = 0; i < FLARM_MAX_TRAFFIC; ++i)
    FLARM_Traffic[i].Clear();
}

const FLARM_TRAFFIC *
FLARM_STATE::FindMaximumAlert() const
{
  const FLARM_TRAFFIC *alert = NULL;

  for (unsigned i = 0; i < FLARM_MAX_TRAFFIC; ++i) {
    const FLARM_TRAFFIC &traffic = FLARM_Traffic[i];

    if (traffic.defined() && traffic.HasAlarm() &&
        (alert == NULL ||
         (traffic.AlarmLevel > alert->AlarmLevel ||
          (traffic.AlarmLevel == alert->AlarmLevel &&
           /* if the levels match -> let the distance decide (smaller
              distance wins) */
           traffic.SquareDistance() < alert->SquareDistance()))))
      alert = &traffic;
  }

  return alert;
}
