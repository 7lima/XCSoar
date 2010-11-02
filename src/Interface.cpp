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

#include "Interface.hpp"
#include "Thread/Mutex.hpp"
#include "MainWindow.hpp"
#include "Language.hpp"
#include "Dialogs/Message.hpp"
#include "StatusMessage.hpp"
#include "InfoBoxes/InfoBoxManager.hpp"
#include "InfoBoxes/InfoBoxLayout.hpp"
#include "Screen/Layout.hpp"
#include "Asset.hpp"
#include "Components.hpp"
#include "DrawThread.hpp"
#include "Gauge/GaugeVario.hpp"
#include "Gauge/GaugeFLARM.hpp"
#include "PeriodClock.hpp"
#include "Screen/Blank.hpp"
#include "LogFile.hpp"
#include "Protection.hpp"
#include "DeviceBlackboard.hpp"

bool ActionInterface::doForceShutdown = false;

InterfaceBlackboard CommonInterface::blackboard;
HINSTANCE CommonInterface::hInst; // The current instance
StatusMessageList CommonInterface::status_messages;
MainWindow CommonInterface::main_window(status_messages);

// settings used only by interface thread scope
bool ActionInterface::LockSettingsInFlight = true;
unsigned XCSoarInterface::debounceTimeout = 250;
unsigned ActionInterface::MenuTimeoutMax = MENUTIMEOUTMAX;

void
XCSoarInterface::ExchangeBlackboard()
{
  ScopeLock protect(mutexBlackboard);
  ReceiveBlackboard();
  ReceiveMapProjection();
  SendSettingsComputer();
  SendSettingsMap();
}

void
XCSoarInterface::ReceiveBlackboard()
{
  ScopeLock protect(mutexBlackboard);
  ReadBlackboardBasic(device_blackboard.Basic());
  ReadBlackboardCalculated(device_blackboard.Calculated());
}

void
ActionInterface::SendSettingsComputer()
{
  ScopeLock protect(mutexBlackboard);
  // send computer settings to the device because we know
  // that it won't be reading from them if we lock it, and
  // then others can retrieve from it at their convenience.
  device_blackboard.ReadSettingsComputer(SettingsComputer());
  // TODO: trigger refresh if the settings are changed
}

void
XCSoarInterface::ReceiveMapProjection()
{
  ScopeLock protect(mutexBlackboard);
  ReadMapProjection(device_blackboard.MapProjection());
}

/**
 * Send the own SettingsMap to the DeviceBlackboard
 * @param trigger_draw Triggers the draw event after sending if true
 */
void
ActionInterface::SendSettingsMap(const bool trigger_draw)
{
  // trigger_draw: asks for an immediate exchange of blackboard data
  // (via ProcessTimer()) rather than waiting for the idle timer every 500ms

  if (trigger_draw) {
    DisplayModes();
    InfoBoxManager::ProcessTimer();
  }

  // Copy InterfaceBlackboard.SettingsMap to the DeviceBlackboard
  ScopeLock protect(mutexBlackboard);
  device_blackboard.ReadSettingsMap(SettingsMap());

  if (trigger_draw)
    draw_thread->trigger_redraw();

  // TODO: trigger refresh if the settings are changed
}

void
ActionInterface::SignalShutdown(bool force)
{
  doForceShutdown = force;
  main_window.close(); // signals close
}

bool
XCSoarInterface::CheckShutdown()
{
  if (doForceShutdown)
    return true;

  return MessageBoxX(_("Quit program?"), _T("XCSoar"),
                     MB_YESNO | MB_ICONQUESTION) == IDYES;
}

// Debounce input buttons (does not matter which button is pressed)
bool
XCSoarInterface::Debounce(void)
{
#if defined(GNAV) || defined(PCGNAV)
  return true;
#else
  static PeriodClock fps_last;

  ResetDisplayTimeOut();

  if (SettingsMap().ScreenBlanked)
    // prevent key presses working if screen is blanked,
    // so a key press just triggers turning the display on again
    return false;

  return fps_last.check_update(debounceTimeout);
#endif
}

/**
 * Determine whether the vario gauge should be drawn depending on the
 * display orientation and the infobox layout
 * @return True if vario gauge should be drawn, False otherwise
 */
static bool
vario_visible()
{
  return Layout::landscape &&
    InfoBoxLayout::InfoBoxGeometry == InfoBoxLayout::ibGNav;
}

/**
 * Determine whether vario gauge, FLARM radar and infoboxes should be drawn
 */
void
ActionInterface::DisplayModes()
{
  bool full_screen = main_window.GetFullScreen();

  // Determine whether the vario gauge should be drawn
  SetSettingsMap().EnableVarioGauge = vario_visible() && !full_screen;

  if (main_window.vario) {
    if (!SettingsMap().ScreenBlanked && SettingsMap().EnableVarioGauge)
      main_window.vario->show();
    else
      main_window.vario->hide();
  }

  if (Basic().flarm.NewTraffic) {
    GaugeFLARM *gauge_flarm = main_window.flarm;
    if (gauge_flarm != NULL)
      gauge_flarm->Suppress = false;
  }
}
