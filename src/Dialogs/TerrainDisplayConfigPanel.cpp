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
#include "TerrainDisplayConfigPanel.hpp"

static WndForm* wf = NULL;


void
TerrainDisplayConfigPanel::Init(WndForm *_wf)
{
  assert(_wf != NULL);
  wf = _wf;
  WndProperty *wp;

  LoadFormProperty(*wf, _T("prpEnableTerrain"),
                   XCSoarInterface::SettingsMap().EnableTerrain);

  LoadFormProperty(*wf, _T("prpEnableTopography"),
                   XCSoarInterface::SettingsMap().EnableTopography);

  wp = (WndProperty*)wf->FindByName(_T("prpSlopeShadingType"));
  if (wp) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->addEnumText(_("Off"));
    dfe->addEnumText(_("Fixed"));
    dfe->addEnumText(_("Sun"));
    dfe->addEnumText(_("Wind"));
    dfe->Set(XCSoarInterface::SettingsMap().SlopeShadingType);
    wp->RefreshDisplay();
  }

  LoadFormProperty(*wf, _T("prpTerrainContrast"),
                   XCSoarInterface::SettingsMap().TerrainContrast * 100 / 255);

  LoadFormProperty(*wf, _T("prpTerrainBrightness"),
                   XCSoarInterface::SettingsMap().TerrainBrightness * 100 / 255);

  wp = (WndProperty*)wf->FindByName(_T("prpTerrainRamp"));
  if (wp) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->addEnumText(_("Low lands"));
    dfe->addEnumText(_("Mountainous"));
    dfe->addEnumText(_("Imhof 7"));
    dfe->addEnumText(_("Imhof 4"));
    dfe->addEnumText(_("Imhof 12"));
    dfe->addEnumText(_("Imhof Atlas"));
    dfe->addEnumText(_("ICAO"));
    dfe->addEnumText(_("Grey"));
    dfe->Set(XCSoarInterface::SettingsMap().TerrainRamp);
    wp->RefreshDisplay();
  }
}


bool
TerrainDisplayConfigPanel::Save()
{
  bool changed = false;
  WndProperty *wp;

  changed |= SaveFormProperty(*wf, _T("prpEnableTerrain"),
                              szProfileDrawTerrain,
                              XCSoarInterface::SetSettingsMap().EnableTerrain);

  changed |= SaveFormProperty(*wf, _T("prpEnableTopography"),
                              szProfileDrawTopography,
                              XCSoarInterface::SetSettingsMap().EnableTopography);

  wp = (WndProperty*)wf->FindByName(_T("prpSlopeShadingType"));
  if (wp) {
    if (XCSoarInterface::SettingsMap().SlopeShadingType !=
        wp->GetDataField()->GetAsInteger()) {
      XCSoarInterface::SetSettingsMap().SlopeShadingType =
          (SlopeShadingType_t)wp->GetDataField()->GetAsInteger();
      Profile::Set(szProfileSlopeShadingType,
                   XCSoarInterface::SettingsMap().SlopeShadingType);
      changed = true;
    }
  }

  wp = (WndProperty*)wf->FindByName(_T("prpTerrainContrast"));
  if (wp) {
    if (XCSoarInterface::SettingsMap().TerrainContrast * 100 / 255 !=
        wp->GetDataField()->GetAsInteger()) {
      XCSoarInterface::SetSettingsMap().TerrainContrast = (short)(wp->GetDataField()->GetAsInteger() * 255 / 100);
      Profile::Set(szProfileTerrainContrast,XCSoarInterface::SettingsMap().TerrainContrast);
      changed = true;
    }
  }

  wp = (WndProperty*)wf->FindByName(_T("prpTerrainBrightness"));
  if (wp) {
    if (XCSoarInterface::SettingsMap().TerrainBrightness * 100 / 255 !=
        wp->GetDataField()->GetAsInteger()) {
      XCSoarInterface::SetSettingsMap().TerrainBrightness = (short)(wp->GetDataField()->GetAsInteger() * 255 / 100);
      Profile::Set(szProfileTerrainBrightness,
                    XCSoarInterface::SettingsMap().TerrainBrightness);
      changed = true;
    }
  }

  changed |= SaveFormProperty(*wf, _T("prpTerrainRamp"), szProfileTerrainRamp,
                              XCSoarInterface::SetSettingsMap().TerrainRamp);

  return changed;
}
