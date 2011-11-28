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

#include "Dialogs/Dialogs.h"
#include "Dialogs/dlgInfoBoxAccess.hpp"
#include "Dialogs/Internal.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Key.h"
#include "Components.hpp"
#include "LocalPath.hpp"
#include "OS/FileUtil.hpp"

#include "InfoBoxes/InfoBoxManager.hpp"
#include "InfoBoxes/InfoBoxLayout.hpp"
#include "Form/TabBar.hpp"
#include "Form/Panel.hpp"
#include "Form/XMLWidget.hpp"

#include <assert.h>
#include <stdio.h>

class CloseInfoBoxAccess : public XMLWidget {
public:
  virtual void Prepare(ContainerWindow &parent, const PixelRect &rc);
  virtual bool Click();
  virtual void ReClick();
};

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

  const InfoBoxContent::DialogContent *dlgContent;
  dlgContent = InfoBoxManager::GetDialogContent(id);

  if (!dlgContent)
    return;

  const PixelRect targetRect = InfoBoxManager::layout.remaining;

  wf = LoadDialog(NULL, parent,
                  _T("IDR_XML_INFOBOXACCESS"), &targetRect);

  assert(wf != NULL);

  // Load tabs
  wTabBar = (TabBarControl*)wf->FindByName(_T("TabBar"));
  assert(wTabBar != NULL);

  Widget *wPanel[dlgContent->PANELSIZE];

  for (int i = 0; i < dlgContent->PANELSIZE; i++) {
    assert(dlgContent->Panels[i].load);

    wPanel[i] = dlgContent->Panels[i].load(id);

    if (wPanel[i] == NULL)
      continue;

    wTabBar->AddTab(wPanel[i], gettext(dlgContent->Panels[i].name));
  }

  Widget *wClose = new CloseInfoBoxAccess();
  wTabBar->AddTab(wClose, _("Close"));

  wTabBar->SetCurrentPage(0);

  StaticString<32> buffer;
  buffer.Format(_T("InfoBox: %s"), InfoBoxManager::GetTitle(id));

  wf->SetCaption(buffer);
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
CloseInfoBoxAccess::Prepare(ContainerWindow &parent, const PixelRect &rc)
{
  LoadWindow(NULL, parent, _T("IDR_XML_INFOBOXACCESSCLOSE"));
}

bool
CloseInfoBoxAccess::Click()
{
  dlgInfoBoxAccess::OnClose();
  return false;
}

void
CloseInfoBoxAccess::ReClick()
{
  dlgInfoBoxAccess::OnClose();
}
