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

#ifndef XCSOAR_GLIDECOMPUTER_STATS_HPP
#define XCSOAR_GLIDECOMPUTER_STATS_HPP

#include "Geo/GeoPoint.hpp"
#include "FlightStatistics.hpp"
#include "GPSClock.hpp"

#include <assert.h>

struct NMEAInfo;
struct MoreData;
struct DerivedInfo;
struct LoggerSettings;
class Logger;

class GlideComputerStats {
  GeoPoint last_location;

  fixed last_climb_start_time, last_cruise_start_time;
  fixed last_thermal_end_time;

  FlightStatistics flightstats;
  GPSClock log_clock;
  GPSClock stats_clock;
  /** number of points to log at high rate */
  unsigned fast_log_num;

protected:
  Logger *logger;

public:
  GlideComputerStats();

  void SetLogger(Logger *_logger) {
    assert(logger == NULL);
    assert(_logger != NULL);

    logger = _logger;
  }

  /** Returns the FlightStatistics object */
  FlightStatistics &GetFlightStats() { return flightstats; }
  const FlightStatistics &GetFlightStats() const { return flightstats; }

  void ResetFlight(const bool full = true);
  void StartTask(const NMEAInfo &basic);
  bool DoLogging(const MoreData &basic, const DerivedInfo &calculated,
                 const LoggerSettings &settings_logger);
  void SetFastLogging();

private:
  void OnClimbBase(const DerivedInfo &calculated, fixed StartAlt);
  void OnClimbCeiling(const DerivedInfo &calculated);
  void OnDepartedThermal(const DerivedInfo &calculated);

public:
  /**
   * Check of climbing has started or ended, and collect statistics
   * about that.
   */
  void ProcessClimbEvents(const DerivedInfo &calculated);
};

#endif
