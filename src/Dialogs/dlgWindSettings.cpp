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

#include "Dialogs/Internal.hpp"
#include "Blackboard.hpp"
#include "SettingsUser.hpp"
#include "SettingsComputer.hpp"
#include "Units.hpp"
#include "Registry.hpp"
#include "Profile.hpp"
#include "DataField/Enum.hpp"
#include "MainWindow.hpp"

static WndForm *wf=NULL;

static void OnCloseClicked(WindowControl * Sender){
	(void)Sender;
  wf->SetModalResult(mrOK);
}

static void UpdateWind(bool set) {
  WndProperty *wp;
  double ws = 0.0, wb = 0.0;
  wp = (WndProperty*)wf->FindByName(_T("prpSpeed"));
  if (wp) {
    ws = Units::ToSysSpeed(wp->GetDataField()->GetAsFloat());
  }
  wp = (WndProperty*)wf->FindByName(_T("prpDirection"));
  if (wp) {
    wb = wp->GetDataField()->GetAsFloat();
  }
  if ((ws != XCSoarInterface::Basic().wind.norm)
      ||(wb != XCSoarInterface::Basic().wind.bearing)) {
    /* JMW illegal
    if (set) {
      SetWindEstimate(ws, wb);
    }
    XCSoarInterface::Calculated().WindSpeed = ws;
    XCSoarInterface::Calculated().WindBearing = wb;
    */
  }
}


static void OnSaveClicked(WindowControl * Sender){
  (void)Sender;
  UpdateWind(true);
  Profile::SaveWindToRegistry();
  wf->SetModalResult(mrOK);
}


static void OnWindSpeedData(DataField *Sender, DataField::DataAccessKind_t Mode){

  switch(Mode){
    case DataField::daGet:
      Sender->SetMax(Units::ToUserWindSpeed(Units::ToSysUnit(200.0, unKiloMeterPerHour)));
      Sender->Set(Units::ToUserWindSpeed(XCSoarInterface::Basic().wind.norm));
    break;
    case DataField::daPut:
      UpdateWind(false);
    break;
    case DataField::daChange:
      // calc alt...
    break;
  }
}

static void OnWindDirectionData(DataField *Sender, DataField::DataAccessKind_t Mode){

  double lastWind;

  switch(Mode){
    case DataField::daGet:
      lastWind = XCSoarInterface::Basic().wind.bearing;
      if (lastWind < 0.5)
        lastWind = 360.0;
      Sender->Set(lastWind);
    break;
    case DataField::daPut:
      UpdateWind(false);
    break;
    case DataField::daChange:
      lastWind = Sender->GetAsFloat();
      if (lastWind < 0.5)
        Sender->Set(360.0);
      if (lastWind > 360.5)
        Sender->Set(1.0);
    break;
  }
}

static CallBackTableEntry_t CallBackTable[]={
  DeclareCallBackEntry(OnWindSpeedData),
  DeclareCallBackEntry(OnWindDirectionData),
  DeclareCallBackEntry(OnSaveClicked),
  DeclareCallBackEntry(OnCloseClicked),
  DeclareCallBackEntry(NULL)
};

void dlgWindSettingsShowModal(void){
  wf = dlgLoadFromXML(CallBackTable,
                      _T("dlgWindSettings.xml"),
		      XCSoarInterface::main_window,
		      _T("IDR_XML_WINDSETTINGS"));
  if (wf == NULL)
    return;

  WndProperty* wp;

  wp = (WndProperty*)wf->FindByName(_T("prpSpeed"));
  if (wp) {
    wp->GetDataField()->SetUnits(Units::GetSpeedName());
    wp->RefreshDisplay();
  }

  wp = (WndProperty*)wf->FindByName(_T("prpAutoWind"));
  if (wp) {
    DataFieldEnum* dfe;
    dfe = (DataFieldEnum*)wp->GetDataField();
    dfe->addEnumText(gettext(_T("Manual")));
    dfe->addEnumText(gettext(_T("Circling")));
    dfe->addEnumText(gettext(_T("ZigZag")));
    dfe->addEnumText(gettext(_T("Both")));
    wp->GetDataField()->Set(XCSoarInterface::SettingsComputer().AutoWindMode);
    wp->RefreshDisplay();

    wp = (WndProperty*)wf->FindByName(_T("prpTrailDrift"));
    if (wp) {
      wp->GetDataField()->Set(XCSoarInterface::SettingsMap().EnableTrailDrift);
      wp->RefreshDisplay();
    }
  }

  wf->ShowModal();

  SetValueRegistryOnChange(wf, _T("prpAutoWind"), szRegistryAutoWind,
                           XCSoarInterface::SetSettingsComputer().AutoWindMode);
  SetValueOnChange(wf, _T("prpTrailDrift"),
                   XCSoarInterface::SetSettingsMap().EnableTrailDrift);

  delete wf;
}
