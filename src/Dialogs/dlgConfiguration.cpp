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

#include "Dialogs/Internal.hpp"
#include "Screen/Busy.hpp"
#include "Screen/Key.h"
#include "Form/CheckBox.hpp"
#include "Screen/Layout.hpp"
#include "MainWindow.hpp"
#include "Profile/Profile.hpp"
#include "DataField/FileReader.hpp"
#include "LogFile.hpp"
#include "ConfigPanels/PagesConfigPanel.hpp"
#include "ConfigPanels/PolarConfigPanel.hpp"
#include "ConfigPanels/UnitsConfigPanel.hpp"
#include "ConfigPanels/LoggerConfigPanel.hpp"
#include "ConfigPanels/DevicesConfigPanel.hpp"
#include "ConfigPanels/AirspaceConfigPanel.hpp"
#include "ConfigPanels/SiteConfigPanel.hpp"
#include "ConfigPanels/MapDisplayConfigPanel.hpp"
#include "ConfigPanels/WayPointDisplayConfigPanel.hpp"
#include "ConfigPanels/SymbolsConfigPanel.hpp"
#include "ConfigPanels/TerrainDisplayConfigPanel.hpp"
#include "ConfigPanels/GlideComputerConfigPanel.hpp"
#include "ConfigPanels/SafetyFactorsConfigPanel.hpp"
#include "ConfigPanels/RouteConfigPanel.hpp"
#include "ConfigPanels/InterfaceConfigPanel.hpp"
#include "ConfigPanels/LayoutConfigPanel.hpp"
#include "ConfigPanels/GaugesConfigPanel.hpp"
#include "ConfigPanels/TaskRulesConfigPanel.hpp"
#include "ConfigPanels/TaskDefaultsConfigPanel.hpp"
#include "ConfigPanels/InfoBoxesConfigPanel.hpp"
#include "ConfigPanels/ExperimentalConfigPanel.hpp"

#include <assert.h>

enum config_page {
  PAGE_SITE,
  PAGE_AIRSPACE,
  PAGE_MAP,
  PAGE_WAYPOINTS,
  PAGE_SYMBOLS,
  PAGE_TERRAIN,
  PAGE_GLIDE_COMPUTER,
  PAGE_SAFETY_FACTORS,
  PAGE_ROUTE,
  PAGE_POLAR,
  PAGE_DEVICES,
  PAGE_UNITS,
  PAGE_INTERFACE,
  PAGE_LAYOUT,
  PAGE_VARIO,
  PAGE_TASK_RULES,
  PAGE_TASK_DEFAULTS,
  PAGE_INFOBOXES,
  PAGE_LOGGER,
  PAGE_PAGES,
  PAGE_EXPERIMENTAL,
};

static const TCHAR *const captions[] = {
  N_("Site"),
  N_("Airspace"),
  N_("Map Display"),
  N_("Waypoint Display"),
  N_("Symbols"),
  N_("Terrain Display"),
  N_("Glide Computer"),
  N_("Safety Factors"),
  N_("Route"),
  N_("Polar"),
  N_("Devices"),
  N_("Units"),
  N_("Interface"),
  N_("Layout"),
  N_("FLARM And Other Gauges"),
  N_("Default Task Rules"),
  N_("Default Task Turnpoint Types"),
  N_("InfoBoxes"),
  N_("Logger"),
  N_("Pages"),
  N_("Experimental Features"),
};


static config_page current_page;
static WndForm *wf = NULL;
TabbedControl *configuration_tabbed;

static void
PageSwitched()
{
  current_page = (config_page)configuration_tabbed->GetCurrentPage();

  TCHAR caption[64];
  _sntprintf(caption, 64, _T("%u %s"),
             (unsigned)current_page + 1,
             gettext(captions[(unsigned)current_page]));
  wf->SetCaption(caption);

  InterfaceConfigPanel::SetVisible(current_page == PAGE_INTERFACE);
  SiteConfigPanel::SetVisible(current_page == PAGE_SITE);
  PolarConfigPanel::SetVisible(current_page == PAGE_POLAR);
}


static void
OnUserLevel(CheckBoxControl &control)
{
  Profile::Set(szProfileUserLevel, control.get_checked());
  wf->FilterAdvanced(control.get_checked());
}

static void
OnNextClicked(gcc_unused WndButton &button)
{
  configuration_tabbed->NextPage();
  PageSwitched();
}

static void
OnPrevClicked(gcc_unused WndButton &button)
{
  configuration_tabbed->PreviousPage();
  PageSwitched();
}

static void
OnCloseClicked(gcc_unused WndButton &button)
{
  wf->SetModalResult(mrOK);
}

static bool
FormKeyDown(WndForm &Sender, unsigned key_code)
{
  (void)Sender;

  switch (key_code) {
  case VK_LEFT:
#ifdef GNAV
  case '6':
#endif
    ((WndButton *)wf->FindByName(_T("cmdPrev")))->set_focus();
    configuration_tabbed->PreviousPage();
    PageSwitched();
    return true;

  case VK_RIGHT:
#ifdef GNAV
  case '7':
#endif
    ((WndButton *)wf->FindByName(_T("cmdNext")))->set_focus();
    configuration_tabbed->NextPage();
    PageSwitched();
    return true;

  default:
    return false;
  }
}

static CallBackTableEntry CallBackTable[] = {
  DeclareCallBackEntry(AirspaceConfigPanel::OnAirspaceColoursClicked),
  DeclareCallBackEntry(AirspaceConfigPanel::OnAirspaceModeClicked),
  DeclareCallBackEntry(OnNextClicked),
  DeclareCallBackEntry(OnPrevClicked),
  DeclareCallBackEntry(DevicesConfigPanel::OnSetupDeviceAClicked),
  DeclareCallBackEntry(DevicesConfigPanel::OnSetupDeviceBClicked),
  DeclareCallBackEntry(DevicesConfigPanel::OnDeviceAData),
  DeclareCallBackEntry(DevicesConfigPanel::OnDeviceBData),
  DeclareCallBackEntry(PolarConfigPanel::OnLoadInternal),
  DeclareCallBackEntry(PolarConfigPanel::OnLoadFromFile),
  DeclareCallBackEntry(PolarConfigPanel::OnExport),
  DeclareCallBackEntry(PolarConfigPanel::OnFieldData),
  DeclareCallBackEntry(UnitsConfigPanel::OnLoadPreset),
  DeclareCallBackEntry(UnitsConfigPanel::OnFieldData),
  DeclareCallBackEntry(UnitsConfigPanel::OnUTCData),
  DeclareCallBackEntry(OnUserLevel),
  DeclareCallBackEntry(NULL)
};

static void
setVariables()
{
  PolarConfigPanel::Init(wf);
  UnitsConfigPanel::Init(wf);
  LoggerConfigPanel::Init(wf);
  DevicesConfigPanel::Init(wf);
  AirspaceConfigPanel::Init(wf);
  SiteConfigPanel::Init(wf);
  MapDisplayConfigPanel::Init(wf);
  WayPointDisplayConfigPanel::Init(wf);
  SymbolsConfigPanel::Init(wf);
  TerrainDisplayConfigPanel::Init(wf);
  GlideComputerConfigPanel::Init(wf);
  SafetyFactorsConfigPanel::Init(wf);
  RouteConfigPanel::Init(wf);
  InterfaceConfigPanel::Init(wf);
  LayoutConfigPanel::Init(wf);
  GaugesConfigPanel::Init(wf);
  TaskRulesConfigPanel::Init(wf);
  TaskDefaultsConfigPanel::Init(wf);
  InfoBoxesConfigPanel::Init(wf);
  ExperimentalConfigPanel::Init(wf);
  PagesConfigPanel::Init(wf);
}

static void
PrepareConfigurationDialog()
{
  gcc_unused ScopeBusyIndicator busy;

  wf = LoadDialog(CallBackTable, XCSoarInterface::main_window,
                  Layout::landscape ? _T("IDR_XML_CONFIGURATION_L") :
                                      _T("IDR_XML_CONFIGURATION"));
  if (wf == NULL)
    return;

  wf->SetKeyDownNotify(FormKeyDown);

  bool expert_mode = false;
  Profile::Get(szProfileUserLevel, expert_mode);

  CheckBox *cb = (CheckBox *)wf->FindByName(_T("Expert"));
  cb->set_checked(expert_mode);
  wf->FilterAdvanced(expert_mode);

  ((WndButton *)wf->FindByName(_T("cmdClose")))->SetOnClickNotify(OnCloseClicked);

  configuration_tabbed = ((TabbedControl *)wf->FindByName(_T("tabbed")));
  assert(configuration_tabbed != NULL);

  setVariables();

  /* restore previous page */
  configuration_tabbed->SetCurrentPage((unsigned)current_page);
  PageSwitched();
}

void dlgConfigurationShowModal(void)
{
  PrepareConfigurationDialog();

  if (wf->ShowModal() != mrOK) {
    delete wf;
    return;
  }

  /* save page number for next time this dialog is opened */
  current_page = (config_page)configuration_tabbed->GetCurrentPage();

  // TODO enhancement: implement a cancel button that skips all this
  // below after exit.
  bool changed = false;
  bool requirerestart = false;
  changed |= PagesConfigPanel::Save();
  changed |= PolarConfigPanel::Save();
  changed |= LoggerConfigPanel::Save();
  changed |= DevicesConfigPanel::Save(requirerestart);
  changed |= AirspaceConfigPanel::Save(requirerestart);
  changed |= SiteConfigPanel::Save();
  changed |= MapDisplayConfigPanel::Save();
  changed |= WayPointDisplayConfigPanel::Save();
  changed |= SymbolsConfigPanel::Save();
  changed |= TerrainDisplayConfigPanel::Save();
  changed |= GlideComputerConfigPanel::Save(requirerestart);
  changed |= SafetyFactorsConfigPanel::Save();
  changed |= RouteConfigPanel::Save();
  changed |= InterfaceConfigPanel::Save(requirerestart);
  changed |= LayoutConfigPanel::Save();
  changed |= GaugesConfigPanel::Save();
  changed |= TaskRulesConfigPanel::Save();
  changed |= TaskDefaultsConfigPanel::Save();
  changed |= InfoBoxesConfigPanel::Save(requirerestart);
  changed |= ExperimentalConfigPanel::Save(requirerestart);
  // Units need to be saved last to prevent
  // conversion problems with other values
  changed |= UnitsConfigPanel::Save();

  if (changed) {
    Profile::Save();
    LogDebug(_T("Configuration: Changes saved"));
    if (requirerestart)
      MessageBoxX(_("Changes to configuration saved.  Restart XCSoar to apply changes."),
                  _T(""), MB_OK);
  }

  delete wf;
  wf = NULL;
}
