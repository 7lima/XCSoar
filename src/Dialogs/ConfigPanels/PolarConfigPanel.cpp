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

#include "PolarConfigPanel.hpp"
#include "DataField/Enum.hpp"
#include "DataField/ComboList.hpp"
#include "DataField/FileReader.hpp"
#include "Form/Form.hpp"
#include "Form/Button.hpp"
#include "Form/Edit.hpp"
#include "Form/Util.hpp"
#include "Form/Frame.hpp"
#include "Dialogs/ComboPicker.hpp"
#include "Dialogs/TextEntry.hpp"
#include "Screen/SingleWindow.hpp"
#include "Polar/PolarStore.hpp"
#include "Polar/PolarGlue.hpp"
#include "Polar/Polar.hpp"
#include "GlideSolvers/GlidePolar.hpp"
#include "Profile/ProfileKeys.hpp"
#include "Profile/Profile.hpp"
#include "Interface.hpp"
#include "LocalPath.hpp"
#include "OS/PathName.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Components.hpp"
#include "MainWindow.hpp"
#include "OS/FileUtil.hpp"
#include "Language.hpp"

#include <cstdio>

static WndForm* wf = NULL;
static WndButton* buttonList = NULL;
static WndButton* buttonImport = NULL;
static WndButton* buttonExport = NULL;
static bool loading = false;

static void
UpdatePolarFields(const PolarInfo &polar)
{
  LoadFormProperty(*wf, _T("prpPolarV1"), polar.v1);
  LoadFormProperty(*wf, _T("prpPolarV2"), polar.v2);
  LoadFormProperty(*wf, _T("prpPolarV3"), polar.v3);

  LoadFormProperty(*wf, _T("prpPolarW1"), polar.w1);
  LoadFormProperty(*wf, _T("prpPolarW2"), polar.w2);
  LoadFormProperty(*wf, _T("prpPolarW3"), polar.w3);

  LoadFormProperty(*wf, _T("prpPolarMassDry"), polar.dry_mass);
  LoadFormProperty(*wf, _T("prpPolarMaxBallast"), polar.max_ballast);

  LoadFormProperty(*wf, _T("prpPolarWingArea"), polar.wing_area);

  if (positive(polar.v_no))
    LoadFormProperty(*wf, _T("prpMaxManoeuveringSpeed"),
                     ugHorizontalSpeed, polar.v_no);
}

static void
SetPolarTitle(const TCHAR* title)
{
  TCHAR caption[255];
  _tcscpy(caption,  _("Polar"));
  _tcscat(caption, _T(": "));
  _tcscat(caption, title);
  ((WndFrame *)wf->FindByName(_T("lblPolar")))->SetCaption(caption);
}

static void
UpdatePolarTitle()
{
  TCHAR title[255];
  if (Profile::Get(szProfilePolarName, title, 255))
    SetPolarTitle(title);
}

static bool
SaveFormToPolar(PolarInfo &polar)
{
  bool changed = SaveFormProperty(*wf, _T("prpPolarV1"), polar.v1);
  changed |= SaveFormProperty(*wf, _T("prpPolarV2"), polar.v2);
  changed |= SaveFormProperty(*wf, _T("prpPolarV3"), polar.v3);

  changed |= SaveFormProperty(*wf, _T("prpPolarW1"), polar.w1);
  changed |= SaveFormProperty(*wf, _T("prpPolarW2"), polar.w2);
  changed |= SaveFormProperty(*wf, _T("prpPolarW3"), polar.w3);

  changed |= SaveFormProperty(*wf, _T("prpPolarMassDry"), polar.dry_mass);
  changed |= SaveFormProperty(*wf, _T("prpPolarMaxBallast"), polar.max_ballast);

  changed |= SaveFormProperty(*wf, _T("prpPolarWingArea"), polar.wing_area);
  changed |= SaveFormProperty(*wf, _T("prpMaxManoeuveringSpeed"),
                              ugHorizontalSpeed, polar.v_no);

  return changed;
}

static void
UpdatePolarInvalidLabel()
{
  PolarInfo polar;
  polar.Init();
  SaveFormToPolar(polar);
  if (polar.IsValid())
    ((WndFrame *)wf->FindByName(_T("lblPolarInvalid")))->hide();
  else
    ((WndFrame *)wf->FindByName(_T("lblPolarInvalid")))->show();
}

void
PolarConfigPanel::OnLoadInternal(WndButton &button)
{
  ComboList list;
  unsigned len = PolarStore::Count();
  for (unsigned i = 0; i < len; i++)
    list.Append(i, PolarStore::GetName(i));

  list.Sort();

  /* let the user select */

  int result = ComboPicker(XCSoarInterface::main_window, _("Load Polar"), list, NULL);
  if (result >= 0) {
    PolarInfo polar;
    PolarStore::Read(list[result].DataFieldIndex, polar);
    UpdatePolarFields(polar);

    Profile::Set(szProfilePolarName, list[result].StringValue);
    UpdatePolarTitle();
    UpdatePolarInvalidLabel();

    unsigned contest_handicap =
      PolarStore::GetContestHandicap(list[result].DataFieldIndex);
    if (contest_handicap > 0)
      LoadFormProperty(*wf, _T("prpHandicap"), contest_handicap);
  }
}

class PolarFileVisitor: public File::Visitor
{
private:
  ComboList &list;

public:
  PolarFileVisitor(ComboList &_list): list(_list) {}

  void Visit(const TCHAR* path, const TCHAR* filename) {
    list.Append(0, path, filename);
  }
};

void
PolarConfigPanel::OnLoadFromFile(WndButton &button)
{
  ComboList list;
  PolarFileVisitor fv(list);

  // Fill list
  VisitDataFiles(_T("*.plr"), fv);

  // Sort list
  list.Sort();

  // Show selection dialog
  int result = ComboPicker(XCSoarInterface::main_window,
                           _("Load Polar From File"), list, NULL);
  if (result >= 0) {
    const TCHAR* path = list[result].StringValue;
    PolarInfo polar;
    PolarGlue::LoadFromFile(polar, path);
    UpdatePolarFields(polar);

    Profile::Set(szProfilePolarName, list[result].StringValueFormatted);
    UpdatePolarTitle();
    UpdatePolarInvalidLabel();
  }
}

void
PolarConfigPanel::OnExport(WndButton &button)
{
  TCHAR filename[69] = _T("");
  if (!dlgTextEntryShowModal(filename, 64))
    return;

  TCHAR path[MAX_PATH];
  _tcscat(filename, _T(".plr"));
  LocalPath(path, filename);

  PolarInfo polar;
  SaveFormToPolar(polar);
  if (PolarGlue::SaveToFile(polar, path)) {
    Profile::Set(szProfilePolarName, filename);
    UpdatePolarTitle();
  }
}

void
PolarConfigPanel::OnFieldData(DataField *Sender, DataField::DataAccessKind_t Mode)
{
  switch (Mode) {
  case DataField::daChange:
    if (!loading)
      Profile::Set(szProfilePolarName, _T("Custom"));

    UpdatePolarTitle();
    UpdatePolarInvalidLabel();
    break;

  case DataField::daInc:
  case DataField::daDec:
  case DataField::daSpecial:
    return;
  }
}

void
PolarConfigPanel::SetVisible(bool active)
{
  if (buttonList != NULL)
    buttonList->set_visible(active);

  if (buttonImport != NULL)
    buttonImport->set_visible(active);

  if (buttonExport != NULL)
    buttonExport->set_visible(active);
}

void
PolarConfigPanel::Init(WndForm *_wf)
{
  loading = true;

  assert(_wf != NULL);
  wf = _wf;
  buttonList = ((WndButton *)wf->FindByName(_T("cmdLoadInternalPolar")));
  buttonImport = ((WndButton *)wf->FindByName(_T("cmdLoadPolarFile")));
  buttonExport = ((WndButton *)wf->FindByName(_T("cmdSavePolarFile")));

  PolarInfo polar;
  if (!PolarGlue::LoadFromProfile(polar)) {
    PolarGlue::LoadDefault(polar);
    SetPolarTitle(polar.name);
  }
  UpdatePolarFields(polar);
  UpdatePolarTitle();
  UpdatePolarInvalidLabel();


  const SETTINGS_COMPUTER &settings_computer = XCSoarInterface::SettingsComputer();

  LoadFormProperty(*wf, _T("prpHandicap"),
                   settings_computer.contest_handicap);

  LoadFormProperty(*wf, _T("prpBallastSecsToEmpty"),
                   settings_computer.BallastSecsToEmpty);

  loading = false;
}

bool
PolarConfigPanel::Save()
{
  bool changed = false;
  SETTINGS_COMPUTER &settings_computer = XCSoarInterface::SetSettingsComputer();

  changed |= SaveFormProperty(*wf, _T("prpHandicap"), szProfileHandicap,
                              settings_computer.contest_handicap);

  changed |= SaveFormProperty(*wf, _T("prpBallastSecsToEmpty"),
                              szProfileBallastSecsToEmpty,
                              settings_computer.BallastSecsToEmpty);

  PolarInfo polar;
  PolarGlue::LoadFromProfile(polar);
  if (SaveFormToPolar(polar)) {
    PolarGlue::SaveToProfile(polar);
    changed = true;

    if (protected_task_manager != NULL) {
      GlidePolar gp = protected_task_manager->get_glide_polar();
      PolarGlue::LoadFromProfile(gp, settings_computer);
      protected_task_manager->set_glide_polar(gp);
    }
  }

  return changed;
}
