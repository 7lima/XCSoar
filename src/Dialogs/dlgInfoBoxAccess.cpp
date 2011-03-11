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

#include "Dialogs/dlgInfoBoxAccess.hpp"
#include "Dialogs/Internal.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Key.h"
#include "Components.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/Graphics.hpp"
#include "LocalPath.hpp"
#include "OS/FileUtil.hpp"

#include "InfoBoxes/InfoBoxManager.hpp"
#include "InfoBoxes/InfoBoxLayout.hpp"

#include "Form/TabBar.hpp"
#include "Form/Panel.hpp"
#include <assert.h>
#include <stdio.h>

static WndForm *wf = NULL;

static TabBarControl* wTabBar = NULL;

void
dlgInfoBoxAccessShowModal(SingleWindow &parent, const int id)
{
  dlgInfoBoxAccess::dlgInfoBoxAccessShowModal(parent, id);
}

WndForm* dlgInfoBoxAccess::GetWindowForm()
{
  return wf;
}

void
dlgInfoBoxAccess::dlgInfoBoxAccessShowModal(SingleWindow &parent, const int id)
{
  // check for another instance of this window
  if (dlgInfoBoxAccess::GetWindowForm() != NULL) return;

  InfoBoxContent::InfoBoxDlgContent *dlgContent;
  dlgContent = InfoBoxManager::GetInfoBoxDlgContent(id);

  if (!dlgContent)
    return;

  const RECT targetRect = InfoBoxManager::layout.remaining;

  wf = LoadDialog(dlgContent->CallBackTable, parent,
                  _T("IDR_XML_INFOBOXACCESS"), &targetRect);

  assert(wf != NULL);

  // Load tabs
  wTabBar = (TabBarControl*)wf->FindByName(_T("TabBar"));
  assert(wTabBar != NULL);

  Window* wPanel[dlgContent->PANELSIZE];

  for (int i = 0; i < dlgContent->PANELSIZE; i++) {
    assert(dlgContent->Panels[i].load);

    wPanel[i] =
      dlgContent->Panels[i].load(parent, wTabBar, wf, id);

    assert(wPanel[i]);
    wTabBar->AddClient(wPanel[i], dlgContent->Panels[i].name, false, NULL, (*dlgContent->Panels[i].preHide), (*dlgContent->Panels[i].preShow),
                                                                           (*dlgContent->Panels[i].postShow), (*dlgContent->Panels[i].reClick));
  }

  Window* wClose =
    dlgInfoBoxAccess::pnlCloseLoad(parent, wTabBar, wf, dlgContent->CallBackTable);
  assert(wClose);
  wTabBar->AddClient(wClose, _T("Close"), false, NULL, NULL,  dlgInfoBoxAccess::pnlCloseOnTabPreShow,
                                                 NULL, dlgInfoBoxAccess::pnlCloseOnTabReClick);

  wTabBar->SetCurrentPage(0);

  wf->ShowModal();

  delete wf;
  // unset wf because wf is still static and public
  wf = NULL;
}

bool
dlgInfoBoxAccess::OnClose()
{
  wf->SetModalResult(mrOK);
  return true;
}


// panel close

void
dlgInfoBoxAccess::pnlCloseOnCloseClicked(WndButton &Sender)
{
  (void)Sender;
  dlgInfoBoxAccess::OnClose();
}

void
dlgInfoBoxAccess::pnlCloseOnTabReClick()
{
  dlgInfoBoxAccess::OnClose();
}

bool
dlgInfoBoxAccess::pnlCloseOnTabPreShow(TabBarControl::EventType EventType)
{
  if (EventType == TabBarControl::MouseOrButton) {
    dlgInfoBoxAccess::OnClose();
    return true;
  }
  return true;
}

Window*
dlgInfoBoxAccess::pnlCloseLoad(SingleWindow &parent, TabBarControl* wTabBar,
                            WndForm* _wf, CallBackTableEntry* CallBackTable)
{
  assert(wTabBar);

  assert(_wf);
  wf = _wf;

  Window *wInfoBoxAccessClose =
      LoadWindow(CallBackTable, wf, *wTabBar, _T("IDR_XML_INFOBOXACCESSCLOSE"));
  assert(wInfoBoxAccessClose);

  return wInfoBoxAccessClose;
}
