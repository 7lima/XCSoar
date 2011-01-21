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
#include "../Message.hpp"
#include "Profile/Profile.hpp"
#include "Profile/AirspaceConfig.hpp"
#include "Profile/ProfileKeys.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Layout.hpp"
#include "MainWindow.hpp"
#include "SettingsAirspace.hpp"
#include "Airspace/ProtectedAirspaceWarningManager.hpp"
#include "Airspace/AirspaceClass.hpp"
#include "Engine/Airspace/AirspaceWarningManager.hpp"
#include "Components.hpp"

#include <assert.h>

static WndForm *wf = NULL;
static WndListFrame *wAirspaceList = NULL;

static bool colormode = false;

static void
OnAirspacePaintListItem(Canvas &canvas, const RECT rc, unsigned i)
{
  assert(i < AIRSPACECLASSCOUNT);

  int w1, w2, x0;
  int w0 = rc.right - rc.left - Layout::FastScale(4);

  w1 = canvas.text_width(_("Warn")) + Layout::FastScale(10);
  w2 = canvas.text_width(_("Display")) + Layout::FastScale(10);
  x0 = w0 - w1 - w2;

  if (colormode) {
    canvas.white_pen();
#ifdef ENABLE_SDL
    canvas.select(Brush(Graphics::Colours[XCSoarInterface::SettingsMap().iAirspaceColour[i]]));
#else
    canvas.set_text_color(Graphics::GetAirspaceColourByClass(i,
        XCSoarInterface::SettingsMap()));
    canvas.set_background_color(Color(0xFF, 0xFF, 0xFF));
    canvas.select(Graphics::GetAirspaceBrushByClass(i,
        XCSoarInterface::SettingsMap()));
#endif
    canvas.rectangle(rc.left + x0, rc.top + Layout::FastScale(2),
        rc.right - Layout::FastScale(2), rc.bottom - Layout::FastScale(2));
  } else {
    const SETTINGS_AIRSPACE &settings_airspace =
      XCSoarInterface::SettingsComputer();

    if (settings_airspace.airspace_warnings.class_warnings[i])
      canvas.text(rc.left + w0 - w1 - w2, rc.top + Layout::FastScale(2),
                  _("Warn"));

    if (settings_airspace.DisplayAirspaces[i])
      canvas.text(rc.left + w0 - w2, rc.top + Layout::FastScale(2),
                  _("Display"));
  }

  canvas.text_clipped(rc.left + Layout::FastScale(2),
      rc.top + Layout::FastScale(2), x0 - Layout::FastScale(10),
                      airspace_class_as_text((AirspaceClass_t)i, false));
}

static bool changed = false;

static void
OnAirspaceListEnter(unsigned ItemIndex)
{
  assert(ItemIndex < AIRSPACECLASSCOUNT);

  if (colormode) {
    int c = dlgAirspaceColoursShowModal();
    if (c >= 0) {
      XCSoarInterface::SetSettingsMap().iAirspaceColour[ItemIndex] = c;
      Profile::SetAirspaceColor(ItemIndex,
          XCSoarInterface::SettingsMap().iAirspaceColour[ItemIndex]);
      changed = true;
      Graphics::InitAirspacePens(XCSoarInterface::SettingsMap());
    }

#ifndef ENABLE_SDL
    int p = dlgAirspacePatternsShowModal();
    if (p >= 0) {
      XCSoarInterface::SetSettingsMap().iAirspaceBrush[ItemIndex] = p;
      Profile::SetAirspaceBrush(ItemIndex,
          XCSoarInterface::SettingsMap().iAirspaceBrush[ItemIndex]);
      changed = true;
    }
#endif
  } else {
    SETTINGS_AIRSPACE &settings_airspace =
      XCSoarInterface::SetSettingsComputer();

    settings_airspace.DisplayAirspaces[ItemIndex] =
        !settings_airspace.DisplayAirspaces[ItemIndex];
    if (!settings_airspace.DisplayAirspaces[ItemIndex])
      settings_airspace.airspace_warnings.class_warnings[ItemIndex] =
        !settings_airspace.airspace_warnings.class_warnings[ItemIndex];

    Profile::SetAirspaceMode(ItemIndex);
    changed = true;
  }

  wAirspaceList->invalidate();
}

static void
OnCloseClicked(WndButton &Sender)
{
  (void)Sender;
  wf->SetModalResult(mrOK);
}

static void
OnLookupClicked(WndButton &Sender)
{
  (void)Sender;
  dlgAirspaceSelect();
}

static CallBackTableEntry CallBackTable[] = {
  DeclareCallBackEntry(OnCloseClicked),
  DeclareCallBackEntry(OnLookupClicked),
  DeclareCallBackEntry(NULL)
};

void
dlgAirspaceShowModal(bool coloredit)
{
  colormode = coloredit;

  wf = LoadDialog(CallBackTable, XCSoarInterface::main_window,
                  !Layout::landscape ? _T("IDR_XML_AIRSPACE_L") :
                                       _T("IDR_XML_AIRSPACE"));
  assert(wf != NULL);

  wAirspaceList = (WndListFrame*)wf->FindByName(_T("frmAirspaceList"));
  assert(wAirspaceList != NULL);
  wAirspaceList->SetActivateCallback(OnAirspaceListEnter);
  wAirspaceList->SetPaintItemCallback(OnAirspacePaintListItem);
  wAirspaceList->SetLength(AIRSPACECLASSCOUNT);

  changed = false;

  wf->ShowModal();

  // now retrieve back the properties...
  if (changed) {
    if (!colormode && airspace_warnings != NULL) {
      ProtectedAirspaceWarningManager::ExclusiveLease awm(*airspace_warnings);
      awm->set_config(XCSoarInterface::SetSettingsComputer().airspace_warnings);
    }

    Profile::Save();
    Message::AddMessage(_("Configuration saved"));
  }

  delete wf;
}
