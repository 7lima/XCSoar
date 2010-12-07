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

#include "Dialogs/Internal.hpp"
#include "Protection.hpp"
#include "Blackboard.hpp"
#include "DeviceBlackboard.hpp"
#include "SettingsComputer.hpp"
#include "Units.hpp"
#include "Atmosphere.h"
#include "DataField/Base.hpp"
#include "DataField/Float.hpp"
#include "MainWindow.hpp"
#include "Components.hpp"
#include "GlideSolvers/GlidePolar.hpp"
#include "Task/ProtectedTaskManager.hpp"

#include <math.h>

static WndForm *wf = NULL;

static bool changed = false;

static GlidePolar *glide_polar = NULL;

static void
SetButtons()
{
  WndButton* wb;

  if ((wb = (WndButton *)wf->FindByName(_T("cmdDump"))) != NULL) {
    wb->set_visible(glide_polar->is_ballastable());
    wb->SetCaption(XCSoarInterface::SettingsComputer().BallastTimerActive ?
        _("Stop") : _("Dump"));
  }
}

static void
OnCloseClicked(WndButton &Sender)
{
  (void)Sender;
  wf->SetModalResult(mrOK);
}

static void
SetBallastTimer(bool active)
{
  if (active == XCSoarInterface::SettingsComputer().BallastTimerActive)
    return;

  XCSoarInterface::SetSettingsComputer().BallastTimerActive = active;

  SetButtons();
}

static void
OnBallastDump(WndButton &Sender)
{
  (void)Sender;

  SetBallastTimer(!XCSoarInterface::SettingsComputer().BallastTimerActive);
}

static void
OnQnhData(DataField *_Sender, DataField::DataAccessKind_t Mode)
{
  DataFieldFloat *Sender = (DataFieldFloat *)_Sender;
  WndProperty* wp;

  switch (Mode) {
  case DataField::daChange:
    device_blackboard.SetQNH(Sender->GetAsFixed());
    wp = (WndProperty*)wf->FindByName(_T("prpAltitude"));
    if (wp) {
      DataFieldFloat &df = *(DataFieldFloat *)wp->GetDataField();
      df.SetAsFloat(Units::ToUserUnit(
          XCSoarInterface::Basic().BaroAltitude, Units::AltitudeUnit));
      wp->RefreshDisplay();
    }
    break;

  case DataField::daInc:
  case DataField::daDec:
  case DataField::daSpecial:
    return;
  }
}

static void
SetAltitude()
{
  static fixed altlast(-2);
  if (fabs(XCSoarInterface::Basic().BaroAltitude - altlast) > fixed_one) {
    WndProperty* wp;
    wp = (WndProperty*)wf->FindByName(_T("prpAltitude"));
    if (wp) {
      if (!XCSoarInterface::Basic().BaroAltitudeAvailable) {
        wp->hide();
      } else {
        DataFieldFloat &df = *(DataFieldFloat *)wp->GetDataField();
        df.SetAsFloat(Units::ToUserUnit(
            XCSoarInterface::Basic().BaroAltitude, Units::AltitudeUnit));
        wp->RefreshDisplay();
      }
    }
  }
  altlast = XCSoarInterface::Basic().BaroAltitude;
}

static void
SetBallast(void)
{
  WndProperty* wp;

  wp = (WndProperty*)wf->FindByName(_T("prpBallastPercent"));
  if (wp) {
    if (glide_polar->is_ballastable()) {
      DataFieldFloat &df = *(DataFieldFloat *)wp->GetDataField();
      df.SetAsFloat(glide_polar->get_ballast() * 100);
    } else
      wp->hide();

    wp->RefreshDisplay();
  }
  wp = (WndProperty*)wf->FindByName(_T("prpBallastLitres"));
  if (wp) {
    if (glide_polar->is_ballastable()) {
      DataFieldFloat &df = *(DataFieldFloat *)wp->GetDataField();
      df.SetAsFloat(glide_polar->get_ballast_litres());
    } else
      wp->hide();

    wp->RefreshDisplay();
  }
  wp = (WndProperty*)wf->FindByName(_T("prpWingLoading"));
  if (wp) {
    const fixed wl = glide_polar->get_wing_loading();
    if (wl > fixed_zero) {
      DataFieldFloat &df = *(DataFieldFloat *)wp->GetDataField();
      df.SetAsFloat(wl);
    } else
      wp->hide();

    wp->RefreshDisplay();
  }
}

/**
 * This function is called repeatedly by the timer and updates the
 * current altitude and ballast. The ballast can change without user
 * input due to the dump function.
 */
static void
OnTimerNotify(WndForm &Sender)
{
  (void)Sender;

  SetBallast();
  SetAltitude();
}

static void
OnBallastData(DataField *Sender, DataField::DataAccessKind_t Mode)
{
  DataFieldFloat &df = *(DataFieldFloat *)Sender;

  switch (Mode) {
  case DataField::daSpecial:
    SetBallastTimer(glide_polar->get_ballast() > fixed(0.01) &&
                    !XCSoarInterface::SettingsComputer().BallastTimerActive);
    break;
  case DataField::daChange:
    glide_polar->set_ballast(df.GetAsFixed() / 100);
    changed = true;
    SetBallast();
    break;

  case DataField::daInc:
  case DataField::daDec:
    return;
  }
}

static void
OnBugsData(DataField *_Sender, DataField::DataAccessKind_t Mode)
{
  DataFieldFloat *Sender = (DataFieldFloat *)_Sender;

  switch (Mode) {
  case DataField::daChange:
    glide_polar->set_bugs(Sender->GetAsFixed() / 100);
    changed = true;
    break;

  case DataField::daInc:
  case DataField::daDec:
  case DataField::daSpecial:
    return;
  }
}

static void
OnTempData(DataField *_Sender, DataField::DataAccessKind_t Mode)
{
  DataFieldFloat *Sender = (DataFieldFloat *)_Sender;
  switch (Mode) {
  case DataField::daChange:
    CuSonde::setForecastTemperature(Units::ToUserUnit(Units::ToSysTemperature(Sender->GetAsFixed()),
                                                      unGradCelcius));
    break;

  case DataField::daInc:
  case DataField::daDec:
  case DataField::daSpecial:
    return;
  }
}

static CallBackTableEntry CallBackTable[] = {
  DeclareCallBackEntry(OnBugsData),
  DeclareCallBackEntry(OnTempData),
  DeclareCallBackEntry(OnBallastData),
  DeclareCallBackEntry(OnQnhData),
  DeclareCallBackEntry(OnCloseClicked),
  DeclareCallBackEntry(OnBallastDump),
  DeclareCallBackEntry(NULL)
};

void
dlgBasicSettingsShowModal()
{
  GlidePolar gp_copy = protected_task_manager.get_glide_polar();
  glide_polar = &gp_copy;

  wf = LoadDialog(CallBackTable, XCSoarInterface::main_window,
                      _T("IDR_XML_BASICSETTINGS"));
  if (wf == NULL)
    return;

  changed = false;

  wf->SetTimerNotify(OnTimerNotify);
  OnTimerNotify(*wf);

  SetButtons();

  LoadFormProperty(*wf, _T("prpBugs"), glide_polar->get_bugs() * 100);
  LoadFormProperty(*wf, _T("prpQNH"),
                   XCSoarInterface::Basic().pressure.get_QNH());

  WndProperty* wp;
  wp = (WndProperty*)wf->FindByName(_T("prpTemperature"));
  if (wp) {
    DataFieldFloat &df = *(DataFieldFloat *)wp->GetDataField();
    df.SetMin(Units::ToUserTemperature(Units::ToSysUnit(fixed(-50), unGradCelcius)));
    df.SetMax(Units::ToUserTemperature(Units::ToSysUnit(fixed(60), unGradCelcius)));
    df.SetUnits(Units::GetTemperatureName());
    df.Set(Units::ToUserTemperature(Units::ToSysUnit(fixed(CuSonde::maxGroundTemperature),
                                                     unGradCelcius)));
    wp->RefreshDisplay();
  }
  wp = (WndProperty*)wf->FindByName(_T("prpAltitude"));
  if (wp) {
    wp->GetDataField()->SetUnits(Units::GetAltitudeName());
    wp->RefreshDisplay();
  }

  if ((wf->ShowModal() == mrOK) && changed)
    protected_task_manager.set_glide_polar(gp_copy);

  delete wf;
}
