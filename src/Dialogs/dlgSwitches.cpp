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
#include "Units.hpp"
#include "InputEvents.h"
#include "DataField/Base.hpp"
#include "MainWindow.hpp"

static WndForm *wf=NULL;

static void UpdateValues() {
  const SWITCH_INFO &switches = XCSoarInterface::Basic().SwitchState;

  WndProperty* wp;
  wp = (WndProperty*)wf->FindByName(_T("prpFlapLanding"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.FlapLanding) {
      wp->GetDataField()->Set(switches.FlapLanding);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpAirbrakeExtended"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.AirbrakeLocked) {
      wp->GetDataField()->Set(switches.AirbrakeLocked);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpFlapPositive"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.FlapPositive) {
      wp->GetDataField()->Set(switches.FlapPositive);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpFlapNeutral"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.FlapNeutral) {
      wp->GetDataField()->Set(switches.FlapNeutral);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpFlapNegative"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.FlapNegative) {
      wp->GetDataField()->Set(switches.FlapNegative);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpGearExtended"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.GearExtended) {
      wp->GetDataField()->Set(switches.GearExtended);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpAcknowledge"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.Acknowledge) {
      wp->GetDataField()->Set(switches.Acknowledge);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpRepeat"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.Repeat) {
      wp->GetDataField()->Set(switches.Repeat);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpSpeedCommand"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.SpeedCommand) {
      wp->GetDataField()->Set(switches.SpeedCommand);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpUserSwitchUp"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.UserSwitchUp) {
      wp->GetDataField()->Set(switches.UserSwitchUp);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpUserSwitchMiddle"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.UserSwitchMiddle) {
      wp->GetDataField()->Set(switches.UserSwitchMiddle);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpUserSwitchDown"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.UserSwitchDown) {
      wp->GetDataField()->Set(switches.UserSwitchDown);
      wp->RefreshDisplay();
    }
  }
  wp = (WndProperty*)wf->FindByName(_T("prpVarioCircling"));
  if (wp) {
    if (wp->GetDataField()->GetAsBoolean() != switches.VarioCircling) {
      wp->GetDataField()->Set(switches.VarioCircling);
      wp->RefreshDisplay();
    }
  }
}

static int OnTimerNotify(WindowControl * Sender) {
	(void)Sender;
  UpdateValues();
  return 0;
}

static void OnCloseClicked(WindowControl * Sender){
	(void)Sender;
  wf->SetModalResult(mrOK);
}


static CallBackTableEntry_t CallBackTable[]={
  DeclareCallBackEntry(OnCloseClicked),
  DeclareCallBackEntry(NULL)
};



void dlgSwitchesShowModal(void){
  wf = dlgLoadFromXML(CallBackTable,
                      _T("dlgSwitches.xml"),
		      XCSoarInterface::main_window,
		      _T("IDR_XML_SWITCHES"));
  if (wf == NULL)
    return;

  wf->SetTimerNotify(OnTimerNotify);

  UpdateValues();
  wf->ShowModal();

  delete wf;
}
