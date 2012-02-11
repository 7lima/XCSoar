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

#ifndef XCSOAR_TRACK_THREAD_HPP
#define XCSOAR_TRACK_THREAD_HPP

#include "Tracking/Features.hpp"

#ifdef HAVE_TRACKING

#include "Tracking/TrackingSettings.hpp"
#include "Thread/StandbyThread.hpp"
#include "Tracking/LiveTrack24.hpp"
#include "PeriodClock.hpp"
#include "Engine/Navigation/GeoPoint.hpp"
#include "DateTime.hpp"

struct NMEAInfo;
struct DerivedInfo;

class TrackingGlue : protected StandbyThread {
  struct LiveTrack24State
  {
    LiveTrack24::SessionID session_id;
    unsigned packet_id;

    void ResetSession() {
      session_id = 0;
    }

    bool HasSession() {
      return session_id != 0;
    }
  };

  PeriodClock clock;

  TrackingSettings settings;
  LiveTrack24State state;

  BrokenDateTime date_time;
  GeoPoint location;
  unsigned altitude;
  unsigned ground_speed;
  Angle track;
  bool flying;

public:
  TrackingGlue();
  void StopAsync();
  void WaitStopped();

  void SetSettings(const TrackingSettings &_settings);
  void OnTimer(const NMEAInfo &basic, const DerivedInfo &calculated);

protected:
  virtual void Tick();
};

#endif /* HAVE_TRACKING */
#endif
