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

#include "ProcessTimer.hpp"
#include "Protection.hpp"
#include "Interface.hpp"
#include "InputEvents.h"
#include "Device/device.hpp"
#include "Device/All.hpp"
#include "Dialogs.h"
#include "Screen/Blank.hpp"
#include "SettingsUser.hpp"
#include "UtilsSystem.hpp"
#include "InfoBoxes/InfoBoxManager.hpp"
#include "MapWindow.hpp"
#include "Math/Earth.hpp"
#include "DeviceBlackboard.hpp"
#include "Components.hpp"
#include "PeriodClock.hpp"
#include "MainWindow.hpp"
#include "Asset.hpp"
#include "Simulator.hpp"
#include "Replay/Replay.hpp"

void
ProcessTimer::HeapCompact()
{
  static int iheapcompact = 0;
  // called 2 times per second, compact heap every minute.
  iheapcompact++;
  if (iheapcompact == 120) {
    MyCompactHeaps();
    iheapcompact = 0;
  }
}

void
ProcessTimer::SystemProcessTimer()
{
  HeapCompact();
}

void
ProcessTimer::MessageProcessTimer()
{
  // don't display messages if airspace warning dialog is active
  if (!dlgAirspaceWarningVisible()) {
    if (main_window.popup.Render()) {
      // turn screen on if blanked and receive a new message
      ResetDisplayTimeOut();
    }
  }
}

void
ProcessTimer::AirspaceProcessTimer()
{
  if (globalRunningEvent.test()) {
    if (airspaceWarningEvent.test()) {
      airspaceWarningEvent.reset();
      ResetDisplayTimeOut();
      dlgAirspaceWarningShowDlg();
    }
  }
}

void
ProcessTimer::CommonProcessTimer()
{
  CheckDisplayTimeOut(false);

  DisplayModes();
  ExchangeBlackboard();

  // send ignore command
  SetSettingsMap().MapScale = 0;

  InfoBoxManager::ProcessTimer();
  InputEvents::ProcessTimer();

  AirspaceProcessTimer();
  MessageProcessTimer();
}

int
ProcessTimer::ConnectionProcessTimer(int itimeout)
{
  devConnectionMonitor();

  static bool connected_last = false;
  static bool wait_connect = false;
  static bool wait_lock = false;

  if (!Basic().gps.Connected) {
    // if gps is not connected, set navwarning to true so
    // calculations flight timers don't get updated
    device_blackboard.SetNAVWarning(true);
  }

  bool connected_now = device_blackboard.LowerConnection();
  if (connected_now && Basic().gps.NAVWarning) {
    if (!wait_lock) {
      // waiting for lock first time
      wait_lock = true;
      itimeout = 0;
      InputEvents::processGlideComputer(GCE_GPS_FIX_WAIT);
#ifndef DISABLEAUDIO
      MessageBeep(MB_ICONEXCLAMATION);
#endif
    }

    // If GPS connected but no lock, must be in hangar
    if (InterfaceTimeoutCheck()) {
      if (is_altair()) {
        // TODO feature: ask question about shutdown or give warning
        // then shutdown if no activity.
        // Shutdown();
      }
    }
  } else if (connected_now) {
    // !navwarning
    wait_connect = false;
    wait_lock = false;
    itimeout = 0;
  } else {
    // not connected
    wait_lock = false;
  }

  if (!connected_now && !connected_last) {
    AllDevicesLinkTimeout();

    if (!wait_connect) {
      // gps is waiting for connection first time
      wait_connect = true;
      InputEvents::processGlideComputer(GCE_GPS_CONNECTION_WAIT);
#ifndef DISABLEAUDIO
      MessageBeep(MB_ICONEXCLAMATION);
#endif
    } else if (itimeout % 60 == 0) {
      itimeout = 0;
      // we've been waiting for connection a long time
      // no activity for 30 seconds, so assume PDA has been
      // switched off and on again
      //
#ifndef WINDOWSPC
      if (!is_altair()) {
        InputEvents::processGlideComputer(GCE_COMMPORT_RESTART);
        devRestart();
      }
#endif
    }
  }

  connected_last = connected_now;
  return itimeout;
}

void
ProcessTimer::Process(void)
{
  CommonProcessTimer();

  if (!is_simulator()) {
    // now check GPS status
    devTick();

    static int itimeout = -1;
    itimeout++;

    // also service replay logger
    if (replay.Update()) {
      if (Basic().gps.MovementDetected && !replay.NmeaReplayEnabled())
        replay.Stop();

      device_blackboard.RaiseConnection();
      device_blackboard.SetNAVWarning(false);
      return;
    }

    if (itimeout % 10 == 0)
      // check connection status every 5 seconds
      itimeout = ConnectionProcessTimer(itimeout);
  } else {
    static PeriodClock m_clock;
    if (m_clock.elapsed() < 0)
      m_clock.update();

    if (replay.Update()) {
      m_clock.update();
    } else if (m_clock.elapsed() >= 1000) {
      m_clock.update();
      device_blackboard.ProcessSimulation();
      TriggerGPSUpdate();
      device_blackboard.RaiseConnection();
    }
  }
}
