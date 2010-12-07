/*
Copyright_License {

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

#if !defined(XCSOAR_GLIDECOMPUTER_HPP)
#define XCSOAR_GLIDECOMPUTER_HPP

#include "Audio/VegaVoice.h"
#include "GPSClock.hpp"
#include "GlideComputerAirData.hpp"
#include "GlideComputerStats.hpp"
#include "GlideComputerTask.hpp"

class ProtectedTaskManager;
class GlideComputerTaskEvents;

// TODO: replace copy constructors so copies of these structures
// do not replicate the large items or items that should be singletons
// OR: just make them static?

class GlideComputer:
    public GlideComputerAirData, GlideComputerTask, GlideComputerStats
{
public:
  GlideComputer(ProtectedTaskManager& task,
                ProtectedAirspaceWarningManager &_awm,
                GlideComputerTaskEvents& events);

  void ResetFlight(const bool full=true);
  void Initialise();
  bool ProcessGPS(); // returns true if idle needs processing
  virtual void ProcessIdle();

  // TODO: make these const
  /** Returns the FlightStatistics object */
  FlightStatistics &GetFlightStats() { return flightstats; }
  const FlightStatistics &GetFlightStats() const { return flightstats; }

  void OnStartTask();
  void OnFinishTask();
  void OnTransitionEnter();

protected:
  VegaVoice    vegavoice;
  virtual void OnTakeoff();
  virtual void OnLanding();
  virtual void OnSwitchClimbMode(bool isclimb, bool left);
  virtual void OnDepartedThermal();

private:
  void CalculateTeammateBearingRange();
  void CalculateOwnTeamCode();
  void FLARM_ScanTraffic();
};

#endif
