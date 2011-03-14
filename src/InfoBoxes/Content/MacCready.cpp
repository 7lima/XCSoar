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

#include "InfoBoxes/Content/Thermal.hpp"
#include "InfoBoxes/Content/MacCready.hpp"

#include "InfoBoxes/InfoBoxWindow.hpp"
#include "InfoBoxes/InfoBoxManager.hpp"
#include "UnitsFormatter.hpp"
#include "Interface.hpp"

#include "Components.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "DeviceBlackboard.hpp"

#include "Dialogs/dlgInfoBoxAccess.hpp"
#include "Screen/Layout.hpp"

#include <tchar.h>
#include <stdio.h>

static void
SetVSpeed(InfoBoxWindow &infobox, fixed value)
{
  TCHAR buffer[32];
  Units::FormatUserVSpeed(value, buffer, 32, false);
  infobox.SetValue(buffer[0] == _T('+') ? buffer + 1 : buffer);
  infobox.SetValueUnit(Units::Current.VerticalSpeedUnit);
}

/*
 * InfoBoxContentMacCready
 *
 * Subpart Panel Edit
 */

static int InfoBoxID;

Window*
InfoBoxContentMacCready::PnlEditLoad(SingleWindow &parent, TabBarControl* wTabBar,
                                 WndForm* wf, const int id)
{
  assert(wTabBar);
  assert(wf);
//  wf = _wf;

  InfoBoxID = id;

  Window *wInfoBoxAccessEdit =
      LoadWindow(dlgContent.CallBackTable, wf, *wTabBar, _T("IDR_XML_INFOBOXMACCREADYEDIT"));
  assert(wInfoBoxAccessEdit);

  return wInfoBoxAccessEdit;
}

void
InfoBoxContentMacCready::PnlEditOnCloseClicked(WndButton &Sender)
{
  (void)Sender;
  dlgInfoBoxAccess::OnClose();
}

void
InfoBoxContentMacCready::PnlEditOnPlusSmall(WndButton &Sender)
{
  (void)Sender;
  InfoBoxManager::ProcessQuickAccess(InfoBoxID, _T("+0.1"));
}

void
InfoBoxContentMacCready::PnlEditOnPlusBig(WndButton &Sender)
{
  (void)Sender;
  InfoBoxManager::ProcessQuickAccess(InfoBoxID, _T("+0.5"));
}

void
InfoBoxContentMacCready::PnlEditOnMinusSmall(WndButton &Sender)
{
  (void)Sender;
  InfoBoxManager::ProcessQuickAccess(InfoBoxID, _T("-0.1"));
}

void
InfoBoxContentMacCready::PnlEditOnMinusBig(WndButton &Sender)
{
  (void)Sender;
  InfoBoxManager::ProcessQuickAccess(InfoBoxID, _T("-0.5"));
}

/*
 * Subpart Panel Setup
 */

Window*
InfoBoxContentMacCready::PnlSetupLoad(SingleWindow &parent, TabBarControl* wTabBar,
                                 WndForm* wf, const int id)
{
  assert(wTabBar);
  assert(wf);
//  wf = _wf;

  InfoBoxID = id;

  Window *wInfoBoxAccessSetup =
      LoadWindow(dlgContent.CallBackTable, wf, *wTabBar, _T("IDR_XML_INFOBOXMACCREADYSETUP"));
  assert(wInfoBoxAccessSetup);

  return wInfoBoxAccessSetup;
}

bool
InfoBoxContentMacCready::PnlSetupPreShow(TabBarControl::EventType EventType)
{

  if (XCSoarInterface::SettingsComputer().auto_mc)
    ((WndButton *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("cmdMode")))->SetCaption(_("MANUAL"));
  else
    ((WndButton *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("cmdMode")))->SetCaption(_("AUTO"));

  return true;
}

void
InfoBoxContentMacCready::PnlSetupOnSetup(WndButton &Sender) {
  (void)Sender;
  InfoBoxManager::SetupFocused(InfoBoxID);
  dlgInfoBoxAccess::OnClose();
}

void
InfoBoxContentMacCready::PnlSetupOnMode(WndButton &Sender)
{
  (void)Sender;

  if (XCSoarInterface::SettingsComputer().auto_mc)
    Sender.SetCaption(_("AUTO"));
  else
    Sender.SetCaption(_("MANUAL"));

  InfoBoxManager::ProcessQuickAccess(InfoBoxID, _T("mode"));
}


/*
 * Subpart callback function pointers
 */


InfoBoxContentMacCready::PanelContent InfoBoxContentMacCready::Panels[] = {
  InfoBoxContentMacCready::PanelContent (
    _T("Edit"),
    (*InfoBoxContentMacCready::PnlEditLoad)),

  InfoBoxContentMacCready::PanelContent (
    _T("Setup"),
    (*InfoBoxContentMacCready::PnlSetupLoad),
    NULL,
    (*InfoBoxContentMacCready::PnlSetupPreShow))
};

CallBackTableEntry InfoBoxContentMacCready::CallBackTable[] = {
  DeclareCallBackEntry(InfoBoxContentMacCready::PnlEditOnPlusSmall),
  DeclareCallBackEntry(InfoBoxContentMacCready::PnlEditOnPlusBig),
  DeclareCallBackEntry(InfoBoxContentMacCready::PnlEditOnMinusSmall),
  DeclareCallBackEntry(InfoBoxContentMacCready::PnlEditOnMinusBig),

  DeclareCallBackEntry(InfoBoxContentMacCready::PnlSetupOnSetup),
  DeclareCallBackEntry(InfoBoxContentMacCready::PnlSetupOnMode),

  DeclareCallBackEntry(NULL)
};

InfoBoxContentMacCready::InfoBoxDlgContent InfoBoxContentMacCready::dlgContent = {
    InfoBoxContentMacCready::PANELSIZE,
    InfoBoxContentMacCready::Panels,
    InfoBoxContentMacCready::CallBackTable
};

InfoBoxContentMacCready::InfoBoxDlgContent*
InfoBoxContentMacCready::GetInfoBoxDlgContent() {
  return &dlgContent;
}

/*
 * Subpart normal operations
 */

void
InfoBoxContentMacCready::Update(InfoBoxWindow &infobox)
{
  if (protected_task_manager == NULL) {
    infobox.SetInvalid();
    return;
  }

  GlidePolar polar = protected_task_manager->get_glide_polar();
  SetVSpeed(infobox, polar.get_mc());

  // Set Comment
  if (XCSoarInterface::SettingsComputer().auto_mc)
    infobox.SetComment(_("AUTO"));
  else
    infobox.SetComment(_("MANUAL"));
}

bool
InfoBoxContentMacCready::HandleKey(const InfoBoxKeyCodes keycode)
{
  if (protected_task_manager == NULL)
    return false;

  GlidePolar polar = protected_task_manager->get_glide_polar();
  fixed mc = polar.get_mc();

  switch (keycode) {
  case ibkUp:
    mc = std::min(mc + fixed_one / 10, fixed(5));
    polar.set_mc(mc);
    protected_task_manager->set_glide_polar(polar);
    device_blackboard.SetMC(mc);
    return true;

  case ibkDown:
    mc = std::max(mc - fixed_one / 10, fixed_zero);
    polar.set_mc(mc);
    protected_task_manager->set_glide_polar(polar);
    device_blackboard.SetMC(mc);
    return true;

  case ibkLeft:
    XCSoarInterface::SetSettingsComputer().auto_mc = false;
    return true;

  case ibkRight:
    XCSoarInterface::SetSettingsComputer().auto_mc = true;
    return true;

  case ibkEnter:
    XCSoarInterface::SetSettingsComputer().auto_mc =
        !XCSoarInterface::SettingsComputer().auto_mc;
    return true;
  }
  return false;
}

bool
InfoBoxContentMacCready::HandleQuickAccess(const TCHAR *misc)
{
  if (protected_task_manager == NULL)
    return false;

  GlidePolar polar = protected_task_manager->get_glide_polar();
  fixed mc = polar.get_mc();

  if (_tcscmp(misc, _T("+0.1")) == 0) {
    return HandleKey(ibkUp);

  } else if (_tcscmp(misc, _T("+0.5")) == 0) {
    mc = std::min(mc + fixed_one / 2, fixed(5));
    polar.set_mc(mc);
    protected_task_manager->set_glide_polar(polar);
    device_blackboard.SetMC(mc);
    return true;

  } else if (_tcscmp(misc, _T("-0.1")) == 0) {
    return HandleKey(ibkDown);

  } else if (_tcscmp(misc, _T("-0.5")) == 0) {
    mc = std::max(mc - fixed_one / 2, fixed_zero);
    polar.set_mc(mc);
    protected_task_manager->set_glide_polar(polar);
    device_blackboard.SetMC(mc);
    return true;

  } else if (_tcscmp(misc, _T("mode")) == 0) {
    return HandleKey(ibkEnter);
  }

  return false;
}
