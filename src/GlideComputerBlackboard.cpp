/*
Copyright_License {

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

#include "GlideComputerBlackboard.hpp"
#include "Protection.hpp"
#include "SettingsComputer.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/Derived.hpp"
#include "GlideRatio.hpp"

//#include "Persist.hpp"

GlideComputerBlackboard::GlideComputerBlackboard(TaskClientCalc& task):
  m_task(task)
{

}


/**
 * Initializes the GlideComputerBlackboard
 */
void GlideComputerBlackboard::Initialise()
{
}

/**
 * Resets the GlideComputerBlackboard
 * @param full Reset all data?
 */
void
GlideComputerBlackboard::ResetFlight(const bool full)
{
  unsigned i;

  /*
    \todo also need to call flight_state_reset() on Basic() ?

    calculated_info.Flying = false;
    if (full) {
      calculated_info.FlightTime = 0;
      calculated_info.TakeOffTime = 0;
    }
  */

  if (full) {
    calculated_info.timeCruising = 0;
    calculated_info.timeCircling = 0;
    calculated_info.TotalHeightClimb = 0;

    calculated_info.CruiseStartTime = -1;
    calculated_info.ClimbStartTime = -1;

    calculated_info.CruiseLD = INVALID_GR;
    calculated_info.AverageLD = INVALID_GR;
    calculated_info.LD = INVALID_GR;
    calculated_info.LDvario = INVALID_GR;
    calculated_info.AverageThermal = 0;

    for (i = 0; i < 200; i++) {
      calculated_info.AverageClimbRate[i]= 0;
      calculated_info.AverageClimbRateN[i]= 0;
    }

    calculated_info.MinAltitude = 0;
    calculated_info.MaxHeightGain = 0;
  }

  calculated_info.MaxThermalHeight = 0;
  for (i = 0; i < NUMTHERMALBUCKETS; i++) {
    calculated_info.ThermalProfileN[i]=0;
    calculated_info.ThermalProfileW[i]=0;
  }
  // clear thermal sources for first time.
  for (i = 0; i < MAX_THERMAL_SOURCES; i++) {
    calculated_info.ThermalSources[i].LiftRate= -1.0;
  }

  calculated_info.Circling = false;
  for (int i = 0; i <= TERRAIN_ALT_INFO::NUMTERRAINSWEEPS; i++) {
    calculated_info.GlideFootPrint[i].Longitude = Angle();
    calculated_info.GlideFootPrint[i].Latitude = Angle();
  }
  calculated_info.TerrainWarningLocation.Latitude = Angle();
  calculated_info.TerrainWarningLocation.Longitude = Angle();

  // If you load persistent values, you need at least these reset:
  calculated_info.LastThermalAverage=0.0;
  calculated_info.ThermalGain=0.0;
}

/**
 * Starts the task on the GlideComputerBlackboard
 */
void
GlideComputerBlackboard::StartTask()
{
  calculated_info.CruiseStartLocation = gps_info.Location;
  calculated_info.CruiseStartAlt = gps_info.NavAltitude;
  calculated_info.CruiseStartTime = gps_info.Time;

  // JMW reset time cruising/time circling stats on task start
  calculated_info.timeCircling = 0;
  calculated_info.timeCruising = 0;
  calculated_info.TotalHeightClimb = 0;

  // reset max height gain stuff on task start
  calculated_info.MaxHeightGain = 0;
  calculated_info.MinAltitude = 0;
}


void
GlideComputerBlackboard::SaveFinish()
{
  // JMW save calculated data at finish
  Finish_Derived_Info = calculated_info;
}

void
GlideComputerBlackboard::RestoreFinish()
{
  FLYING_STATE flying_state = Basic().flight;

  calculated_info = Finish_Derived_Info;

  // \todo restore flying state
  //  SetBasic().flying_state = flying_state;
}

/**
 * Returns the average vertical speed in the current thermal
 * @return Average vertical speed in the current thermal
 */
double
GlideComputerBlackboard::GetAverageThermal() const
{
  return max(fixed_zero, calculated_info.AverageThermal);
}

/**
 * Retrieves GPS data from the DeviceBlackboard
 * @param nmea_info New GPS data
 */
void
GlideComputerBlackboard::ReadBlackboard(const NMEA_INFO &nmea_info)
{
  _time_retreated = false;

  if (nmea_info.Time < gps_info.Time) {
    // backwards in time, so reset last
    last_gps_info = nmea_info;
    last_calculated_info = calculated_info;
    _time_retreated = true;
  } else if (nmea_info.Time > gps_info.Time) {
    // forwards in time, so save state
    last_gps_info = gps_info;
    last_calculated_info = calculated_info;
  }

  gps_info = nmea_info;

  // if time hasn't advanced, don't copy last calculated
}

/**
 * Retrieves settings from the DeviceBlackboard
 * @param settings New settings
 */
void
GlideComputerBlackboard::ReadSettingsComputer(const SETTINGS_COMPUTER &settings)
{
  settings_computer = settings;
}
