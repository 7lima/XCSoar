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

#include "InfoBoxes/Content/Altitude.hpp"

#include "InfoBoxes/InfoBoxWindow.hpp"
#include "InfoBoxes/InfoBoxManager.hpp"
#include "UnitsFormatter.hpp"
#include "Units.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Engine/Waypoint/Waypoints.hpp"

#include "DeviceBlackboard.hpp"
#include "Simulator.hpp"
#include "FlightStatisticsRenderer.hpp"
#include "GlideComputer.hpp"

#include "Dialogs/dlgInfoBoxAccess.hpp"
#include "Screen/Layout.hpp"
#include "DataField/Float.hpp"
#include "DataField/Base.hpp"

#include <tchar.h>
#include <stdio.h>

/*
 * InfoBoxContentAltitude
 *
 * Subpart Panel Info
 */

static int InfoBoxID;

Window*
InfoBoxContentAltitude::PnlInfoLoad(SingleWindow &parent, TabBarControl* wTabBar,
                                    WndForm* wf, const int id)
{
  assert(wTabBar);
  assert(wf);
//  wf = _wf;

  InfoBoxID = id;

  Window *wInfoBoxAccessInfo =
      LoadWindow(CallBackTable, wf, *wTabBar, _T("IDR_XML_INFOBOXALTITUDEINFO"));
  assert(wInfoBoxAccessInfo);

  wf->SetTimerNotify(OnTimerNotify);

  return wInfoBoxAccessInfo;
}

void
InfoBoxContentAltitude::OnTimerNotify(WndForm &Sender)
{
  (void)Sender;

  PnlInfoUpdate();
}

bool
InfoBoxContentAltitude::PnlInfoOnTabPreShow(TabBarControl::EventType EventType)
{
  return PnlInfoUpdate();
}

bool
InfoBoxContentAltitude::PnlInfoUpdate()
{
  const DERIVED_INFO &calculated = CommonInterface::Calculated();
  const NMEA_INFO &basic = CommonInterface::Basic();
  TCHAR sTmp[32];

  if (!calculated.AltitudeAGLValid) {
    ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpAltAGL")))->SetText(_("N/A"));
  } else {
    // Set Value
    _stprintf(sTmp, _T("%.0f %s"), (double)Units::ToUserAltitude(calculated.AltitudeAGL),
                                   Units::GetAltitudeName());

    ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpAltAGL")))->SetText(sTmp);
  }

  if (!basic.BaroAltitudeAvailable) {
    ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpAltBaro")))->SetText(_("N/A"));
  } else {
    // Set Value
    _stprintf(sTmp, _T("%.0f %s"), (double)Units::ToUserAltitude(basic.BaroAltitude),
                                       Units::GetAltitudeName());

    ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpAltBaro")))->SetText(sTmp);
  }

  if (!basic.GPSAltitudeAvailable) {
    ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpAltGPS")))->SetText(_("N/A"));
  } else {
    // Set Value
     _stprintf(sTmp, _T("%.0f %s"), (double)Units::ToUserAltitude(basic.GPSAltitude),
                                         Units::GetAltitudeName());

      ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpAltGPS")))->SetText(sTmp);
  }

  if (basic.gps.NAVWarning || !calculated.TerrainValid){
    ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpTerrain")))->SetText(_("N/A"));
  } else {
    // Set Value
     _stprintf(sTmp, _T("%.0f %s"), (double)Units::ToUserAltitude(calculated.TerrainAlt),
                                         Units::GetAltitudeName());

      ((WndProperty *)dlgInfoBoxAccess::GetWindowForm()->FindByName(_T("prpTerrain")))->SetText(sTmp);
  }

  return true;
}

/*
 * Subpart Panel Setup
 */

Window*
InfoBoxContentAltitude::PnlSetupLoad(SingleWindow &parent, TabBarControl* wTabBar,
                                     WndForm* wf, const int id)
{
  assert(wTabBar);
  assert(wf);
//  wf = _wf;

  InfoBoxID = id;

  Window *wInfoBoxAccessSetup =
      LoadWindow(CallBackTable, wf, *wTabBar, _T("IDR_XML_INFOBOXALTITUDESETUP"));
  assert(wInfoBoxAccessSetup);

  LoadFormProperty(*wf, _T("prpQNH"),
                   XCSoarInterface::Basic().pressure.get_QNH());

  return wInfoBoxAccessSetup;
}

void
InfoBoxContentAltitude::PnlSetupOnQNH(DataField *_Sender, DataField::DataAccessKind_t Mode)
{
  DataFieldFloat *Sender = (DataFieldFloat *)_Sender;

  switch (Mode) {
  case DataField::daChange:
    device_blackboard.SetQNH(Sender->GetAsFixed());
    break;

  case DataField::daInc:
  case DataField::daDec:
  case DataField::daSpecial:
    return;
  }
}

void
InfoBoxContentAltitude::PnlSetupOnSetup(WndButton &Sender) {
  (void)Sender;
  InfoBoxManager::SetupFocused(InfoBoxID);
  dlgInfoBoxAccess::OnClose();
}

/*
 * Subpart callback function pointers
 */

InfoBoxContentAltitude::PanelContent InfoBoxContentAltitude::Panels[] = {
InfoBoxContentAltitude::PanelContent (
  _T("Info"),
  (*InfoBoxContentAltitude::PnlInfoLoad),
  NULL,
  (*InfoBoxContentAltitude::PnlInfoOnTabPreShow)),

InfoBoxContentAltitude::PanelContent (
  _T("Setup"),
  (*InfoBoxContentAltitude::PnlSetupLoad))
};

CallBackTableEntry InfoBoxContentAltitude::CallBackTable[] = {
  DeclareCallBackEntry(InfoBoxContentAltitude::PnlSetupOnQNH),
  DeclareCallBackEntry(InfoBoxContentAltitude::PnlSetupOnSetup),

  DeclareCallBackEntry(NULL)
};

InfoBoxContentAltitude::DialogContent InfoBoxContentAltitude::dlgContent = {
    InfoBoxContentAltitude::PANELSIZE,
    InfoBoxContentAltitude::Panels,
    InfoBoxContentAltitude::CallBackTable
};

InfoBoxContentAltitude::DialogContent*
InfoBoxContentAltitude::GetDialogContent() {
  return &dlgContent;
}

void
InfoBoxContentAltitudeGPS::Update(InfoBoxWindow &infobox)
{
  const NMEA_INFO &basic = CommonInterface::Basic();
  TCHAR sTmp[32];

  if (!basic.GPSAltitudeAvailable) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  Units::FormatUserAltitude(basic.GPSAltitude, sTmp,
                            sizeof(sTmp) / sizeof(sTmp[0]), false);
  infobox.SetValue(sTmp);

  // Set Comment
  Units::FormatAlternateUserAltitude(basic.GPSAltitude, sTmp,
                                     sizeof(sTmp) / sizeof(sTmp[0]));
  infobox.SetComment(sTmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

bool
InfoBoxContentAltitudeGPS::HandleKey(const InfoBoxKeyCodes keycode)
{
  const NMEA_INFO &basic = CommonInterface::Basic();

  if (!is_simulator())
    return false;
  if (basic.gps.Replay)
    return false;

  fixed fixed_step = (fixed)Units::ToSysAltitude(fixed(100));
  const Angle a5 = Angle::degrees(fixed(5));

  switch (keycode) {
  case ibkUp:
    device_blackboard.SetAltitude(
        basic.GPSAltitude + fixed_step);
    return true;

  case ibkDown:
    device_blackboard.SetAltitude(
        max(fixed_zero, basic.GPSAltitude - fixed_step));
    return true;

  case ibkLeft:
    device_blackboard.SetTrackBearing(
        basic.TrackBearing - a5);
    return true;

  case ibkRight:
    device_blackboard.SetTrackBearing(
        basic.TrackBearing + a5);
    return true;

  case ibkEnter:
    break;
  }

  return false;
}

void
InfoBoxContentAltitudeAGL::Update(InfoBoxWindow &infobox)
{
  const DERIVED_INFO &calculated = CommonInterface::Calculated();
  TCHAR sTmp[32];

  if (!calculated.AltitudeAGLValid) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  Units::FormatUserAltitude(calculated.AltitudeAGL, sTmp,
                            sizeof(sTmp) / sizeof(sTmp[0]), false);
  infobox.SetValue(sTmp);

  // Set Comment
  Units::FormatAlternateUserAltitude(calculated.AltitudeAGL, sTmp,
                                     sizeof(sTmp) / sizeof(sTmp[0]));
  infobox.SetComment(sTmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);

  // Set Color (red/black)
  infobox.SetColor(calculated.AltitudeAGL <
      XCSoarInterface::SettingsComputer().route_planner.safety_height_terrain ? 1 : 0);
}

void
InfoBoxContentAltitudeBaro::Update(InfoBoxWindow &infobox)
{
  const NMEA_INFO &basic = CommonInterface::Basic();
  TCHAR sTmp[32];

  if (!basic.BaroAltitudeAvailable) {
    infobox.SetInvalid();

    if (basic.PressureAltitudeAvailable && !basic.QNHAvailable)
      infobox.SetComment(_("no QNH"));

    return;
  }

  // Set Value
  Units::FormatUserAltitude(basic.BaroAltitude, sTmp,
                            sizeof(sTmp) / sizeof(sTmp[0]), false);
  infobox.SetValue(sTmp);

  // Set Comment
  Units::FormatAlternateUserAltitude(basic.BaroAltitude, sTmp,
                                     sizeof(sTmp) / sizeof(sTmp[0]));
  infobox.SetComment(sTmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentAltitudeQFE::Update(InfoBoxWindow &infobox)
{
  const NMEA_INFO &basic = CommonInterface::Basic();
  TCHAR sTmp[32];

  if (!basic.GPSAltitudeAvailable) {
    infobox.SetInvalid();
    return;
  }

  fixed Value = basic.GPSAltitude;

  const Waypoint* home_waypoint = way_points.find_home();
  if (home_waypoint)
    Value -= home_waypoint->Altitude;

  // Set Value
  Units::FormatUserAltitude(Value, sTmp,
                            sizeof(sTmp) / sizeof(sTmp[0]), false);
  infobox.SetValue(sTmp);

  // Set Comment
  Units::FormatAlternateUserAltitude(Value, sTmp,
                                     sizeof(sTmp) / sizeof(sTmp[0]));
  infobox.SetComment(sTmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentFlightLevel::Update(InfoBoxWindow &infobox)
{
  const NMEA_INFO &basic = CommonInterface::Basic();
  TCHAR sTmp[32];

  if (basic.PressureAltitudeAvailable) {
    fixed Altitude = Units::ToUserUnit(basic.PressureAltitude, unFeet);

    // Title color black
    infobox.SetColorTop(0);

    // Set Value
    _stprintf(sTmp, _T("%03d"), iround(Altitude/100));
    infobox.SetValue(sTmp);

    // Set Comment
    _stprintf(sTmp, _T("%dft"), iround(Altitude));
    infobox.SetComment(sTmp);

  } else if (basic.GPSAltitudeAvailable && basic.QNHAvailable) {
    // Take gps altitude as baro altitude. This is inaccurate but still fits our needs.
    fixed Altitude = Units::ToUserUnit(basic.pressure.QNHAltitudeToPressureAltitude(basic.GPSAltitude), unFeet);

    // Title color red
    infobox.SetColorTop(1);

    // Set Value
    _stprintf(sTmp, _T("%03d"), iround(Altitude/100));
    infobox.SetValue(sTmp);

    // Set Comment
    _stprintf(sTmp, _T("%dft"), iround(Altitude));
    infobox.SetComment(sTmp);

  } else if ((basic.BaroAltitudeAvailable || basic.GPSAltitudeAvailable) && !basic.QNHAvailable) {
    infobox.SetInvalid();
    infobox.SetComment(_("no QNH"));
  } else {
    infobox.SetInvalid();
  }
}

void
InfoBoxContentTerrainHeight::Update(InfoBoxWindow &infobox)
{
  const NMEA_INFO &basic = CommonInterface::Basic();
  const DERIVED_INFO &calculated = CommonInterface::Calculated();
  TCHAR sTmp[32];

  if (basic.gps.NAVWarning || !calculated.TerrainValid){
    infobox.SetInvalid();
    return;
  }

  // Set Value
  Units::FormatUserAltitude(calculated.TerrainAlt, sTmp,
                            sizeof(sTmp) / sizeof(sTmp[0]), false);
  infobox.SetValue(sTmp);

  // Set Comment
  Units::FormatAlternateUserAltitude(calculated.TerrainAlt, sTmp,
                                     sizeof(sTmp) / sizeof(sTmp[0]));
  infobox.SetComment(sTmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentBarogram::Update(InfoBoxWindow &infobox)
{
  const NMEA_INFO &basic = CommonInterface::Basic();
  TCHAR sTmp[32];

  Units::FormatUserAltitude(basic.NavAltitude, sTmp,
                            sizeof(sTmp) / sizeof(sTmp[0]));
  infobox.SetComment(sTmp);

  infobox.SetValue(_T(""));
  infobox.invalidate();
}

void
InfoBoxContentBarogram::on_custom_paint(InfoBoxWindow &infobox, Canvas &canvas)
{
  RECT rc = infobox.get_value_rect();
  rc.top += Layout::FastScale(2);

  FlightStatisticsRenderer fs(glide_computer->GetFlightStats());
  fs.RenderBarographSpark(canvas, rc, XCSoarInterface::Basic(),
                          XCSoarInterface::Calculated(), protected_task_manager);
}
