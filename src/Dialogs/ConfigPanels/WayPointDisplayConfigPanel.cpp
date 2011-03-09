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

#include "Profile/ProfileKeys.hpp"
#include "Profile/Profile.hpp"
#include "Form/Edit.hpp"
#include "Form/Util.hpp"
#include "DataField/Enum.hpp"
#include "Interface.hpp"
#include "Appearance.hpp"
#include "Screen/Graphics.hpp"
#include "WayPointDisplayConfigPanel.hpp"

static WndForm* wf = NULL;


void
WayPointDisplayConfigPanel::Init(WndForm *_wf)
{
  assert(_wf != NULL);
  wf = _wf;
  WndProperty *wp;

  wp = (WndProperty*)wf->FindByName(_T("prpWaypointLabels"));
  if (wp) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->EnableItemHelp(true);
    dfe->addEnumText(_("Full Name"), DISPLAYNAME, _("The full name of each waypoint is displayed."));
    dfe->addEnumText(_("First Word of Name"), DISPLAYUNTILSPACE, _("The first word of the waypoint name is displayed."));
    dfe->addEnumText(_("First 3 Letters"), DISPLAYFIRSTTHREE, _("The first 3 letters of the waypoint name are displayed."));
    dfe->addEnumText(_("First 5 Letters"), DISPLAYFIRSTFIVE, _("The first 5 letters of the waypoint name are displayed."));
    dfe->addEnumText(_("None"), DISPLAYNONE, _("No waypoint name is displayed."));
    dfe->Set(XCSoarInterface::SettingsMap().DisplayTextType);
    wp->RefreshDisplay();
  }

  wp = (WndProperty*)wf->FindByName(_T("prpWaypointLabelStyle"));
  if (wp) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->addEnumText(_("Rounded Rectangle"), RoundedBlack);
    dfe->addEnumText(_("Outlined"), OutlinedInverted);
    dfe->Set(XCSoarInterface::SettingsMap().LandableRenderMode);
    wp->RefreshDisplay();
  }

  wp = (WndProperty*)wf->FindByName(_T("prpWayPointLabelSelection"));
  if (wp) {
    //Determines what waypoint labels are displayed for each waypoint (space permitting):&#10;
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->EnableItemHelp(true);
    dfe->addEnumText(_("All"), wlsAllWayPoints,
                     _("All waypoints labels will displayed."));
    dfe->addEnumText(_("Task Waypoints & Landables"),
                     wlsTaskAndLandableWayPoints,
                     _("All waypoints labels part of a task and all landables will be displayed."));
    dfe->addEnumText(_("Task Waypoints"), wlsTaskWayPoints,
                     _("All waypoints labels part of a task will be displayed."));
    dfe->addEnumText(_("None"), wlsNoWayPoints,
                     _("No waypoint labels will be displayed."));
    dfe->Set(XCSoarInterface::SettingsMap().WayPointLabelSelection);
    wp->RefreshDisplay();
  }

  wp = (WndProperty*)wf->FindByName(_T("prpAppIndLandable"));
  if (wp) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->addEnumText(_("Purple Circle"));
    dfe->addEnumText(_("B/W"));
    dfe->addEnumText(_("Orange"));
    dfe->Set(Appearance.IndLandable);
    wp->RefreshDisplay();
  }

  LoadFormProperty(*wf, _T("prpAppUseSWLandablesRendering"),
                   Appearance.UseSWLandablesRendering);

  LoadFormProperty(*wf, _T("prpAppLandableRenderingScale"),
                   Appearance.LandableRenderingScale);

  LoadFormProperty(*wf, _T("prpAppScaleRunwayLength"),
                   Appearance.ScaleRunwayLength);
}


bool
WayPointDisplayConfigPanel::Save()
{
  bool changed = false;
  WndProperty *wp;

  changed |= SaveFormPropertyEnum(*wf, _T("prpWaypointLabels"),
                                  szProfileDisplayText,
                                  XCSoarInterface::SetSettingsMap().DisplayTextType);

  changed |= SaveFormPropertyEnum(*wf, _T("prpWaypointLabelStyle"),
                                  szProfileWaypointLabelStyle,
                                  XCSoarInterface::SetSettingsMap().LandableRenderMode);

  changed |= SaveFormPropertyEnum(*wf, _T("prpWayPointLabelSelection"),
                                  szProfileWayPointLabelSelection,
                                  XCSoarInterface::SetSettingsMap().WayPointLabelSelection);

  wp = (WndProperty*)wf->FindByName(_T("prpAppIndLandable"));
  if (wp) {
    if (Appearance.IndLandable != (IndLandable_t)(wp->GetDataField()->GetAsInteger())) {
      Appearance.IndLandable = (IndLandable_t)(wp->GetDataField()->GetAsInteger());
      Profile::Set(szProfileAppIndLandable, Appearance.IndLandable);
      changed = true;
      Graphics::InitLandableIcons();
    }
  }

  changed |= SaveFormProperty(*wf, _T("prpAppUseSWLandablesRendering"),
                              szProfileAppUseSWLandablesRendering,
                              Appearance.UseSWLandablesRendering);

  changed |= SaveFormProperty(*wf, _T("prpAppLandableRenderingScale"),
                              szProfileAppLandableRenderingScale,
                              Appearance.LandableRenderingScale);

  changed |= SaveFormProperty(*wf, _T("prpAppScaleRunwayLength"),
                              szProfileAppScaleRunwayLength,
                              Appearance.ScaleRunwayLength);


  return changed;
}
