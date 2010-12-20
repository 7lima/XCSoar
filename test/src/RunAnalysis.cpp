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

#include "Screen/SingleWindow.hpp"
#include "Screen/Blank.hpp"
#include "Screen/BufferCanvas.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Init.hpp"
#include "ResourceLoader.hpp"
#include "Interface.hpp"
#include "InfoBoxes/InfoBoxLayout.hpp"
#include "Logger/Logger.hpp"
#include "Terrain/RasterWeather.hpp"
#include "Terrain/RasterTerrain.hpp"
#include "WayPointGlue.hpp"
#include "Dialogs/Dialogs.h"
#include "Airspace/AirspaceParser.hpp"
#include "Airspace/ProtectedAirspaceWarningManager.hpp"
#include "Airspace/AirspaceGlue.hpp"
#include "Profile/Profile.hpp"
#include "Engine/Waypoint/Waypoints.hpp"
#include "Engine/Airspace/Airspaces.hpp"
#include "Engine/Airspace/AirspaceWarningManager.hpp"
#include "Engine/Task/TaskManager.hpp"
#include "Engine/Task/TaskEvents.hpp"
#include "GlideComputer.hpp"
#include "GlideComputerInterface.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Thread/Trigger.hpp"
#include "GlideComputerInterface.hpp"

/* fake symbols: */

#include "ConditionMonitor.hpp"
#include "InputEvents.hpp"
#include "Logger/Logger.hpp"
#include "ThermalBase.hpp"
#include "ThermalLocator.hpp"
#include "LocalTime.hpp"

unsigned InfoBoxLayout::ControlWidth = 100;

Trigger airspaceWarningEvent(_T("airspaceWarning"));
Trigger triggerClimbEvent(_T("triggerClimb"));

void dlgBasicSettingsShowModal() {}
void dlgTaskCalculatorShowModal(SingleWindow &parent) {}
void dlgWindSettingsShowModal() {}
void dlgAirspaceWarningShowDlg(SingleWindow &parent, bool auto_close) {}

void ConditionMonitorsUpdate(const GlideComputer &cmp) {}

bool InputEvents::processGlideComputer(unsigned) { return false; }

Logger logger;

void Logger::LogStartEvent(const NMEA_INFO &gps_info) {}
void Logger::LogFinishEvent(const NMEA_INFO &gps_info) {}
void Logger::LogPoint(const NMEA_INFO &gps_info) {}
LoggerImpl::LoggerImpl() {}
LoggerImpl::~LoggerImpl() {}

ThermalLocator::ThermalLocator() {}
void ThermalLocator::Reset() {}

void
ThermalLocator::Process(const bool circling,
                        const fixed time, 
                        const GeoPoint &location, 
                        const fixed w,
                        const SpeedVector wind,
                        THERMAL_LOCATOR_INFO& therm) {}

long GetUTCOffset() { return 0; }

void
EstimateThermalBase(const GeoPoint Thermal_Location,
                    const fixed altitude, const fixed wthermal,
                    const SpeedVector wind,
                    GeoPoint &ground_location, fixed &ground_alt) {}

#ifndef ENABLE_SDL
#include "MapWindow.hpp"
bool MapWindow::identify(HWND hWnd) { return false; }
#endif /* !ENABLE_SDL */

/* done with fake symbols. */

InterfaceBlackboard CommonInterface::blackboard;

Waypoints way_points;

static GlideComputerTaskEvents task_events;

static TaskManager task_manager(task_events, way_points);

Airspaces airspace_database;

static AirspaceWarningManager airspace_warning(airspace_database,
                                               task_manager);

static ProtectedAirspaceWarningManager airspace_warnings(airspace_warning);

ProtectedTaskManager protected_task_manager(task_manager,
                                            XCSoarInterface::SettingsComputer(),
                                            task_events);

GlideComputer glide_computer(way_points, protected_task_manager,
                             airspace_warnings,
                             task_events);

RasterTerrain *terrain;

static void
LoadFiles()
{
  terrain = RasterTerrain::OpenTerrain(NULL);

  AtmosphericPressure pressure;
  ReadAirspace(airspace_database, terrain, pressure);
}

#ifndef WIN32
int main(int argc, char **argv)
#else
int WINAPI
WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
#ifdef _WIN32_WCE
        LPWSTR lpCmdLine,
#else
        LPSTR lpCmdLine2,
#endif
        int nCmdShow)
#endif
{
  Profile::SetFiles(_T(""));
  Profile::Load();

  LoadFiles();

  ScreenGlobalInit screen_init;

#ifdef WIN32
  ResourceLoader::Init(hInstance);

  PaintWindow::register_class(hInstance);
#endif

  SingleWindow main_window;
  main_window.set(_T("STATIC"), _T("RunDialog"),
                  0, 0, 640, 480);

  SETTINGS_MAP settings_map;
  Graphics::Initialise();
  Graphics::InitialiseConfigured(CommonInterface::SetSettingsMap());

  Fonts::Initialize();
  main_window.show();

  dlgAnalysisShowModal(main_window, -1);

  Fonts::Deinitialize();

  delete terrain;

  return 0;
}
