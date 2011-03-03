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

#include <stdio.h>
#include "PagesConfigPanel.hpp"
#include "DataField/Enum.hpp"
#include "Form/Edit.hpp"
#include "Pages.hpp"

using namespace Pages;

static void
UpdateComboBox(DataFieldEnum* dfe, unsigned page)
{
  TCHAR buffer[128];
  const PageLayout* pl;
  const PageLayout* currentPL = GetLayout(page);
  assert(currentPL != NULL);
  currentPL->MakeTitle(buffer);
  // dont offer "None" for first page
  unsigned i = (page == 0) ? 1 : 0;
  while (NULL != (pl = PossiblePageLayout(i))) {
    pl->MakeTitle(buffer);
    dfe->addEnumText(buffer, i);
    if (*pl == *currentPL)
      dfe->Set(i);
    i++;
  }
}

void
PagesConfigPanel::Init(WndForm *wf)
{
  TCHAR prpName[64];
  for (unsigned i = 0; i < MAX_PAGES; i++) {
    _stprintf(prpName, _T("prpPageLayout%u"), i);
    WndProperty* wp = (WndProperty*)wf->FindByName(prpName);
    if (wp) {
      DataFieldEnum* dfe = (DataFieldEnum*)wp->GetDataField();
      UpdateComboBox(dfe, i);
      wp->RefreshDisplay();
    }
  }
}

bool
PagesConfigPanel::Save(WndForm *wf)
{
  TCHAR prpName[64];
  bool changed = false;
  for (unsigned int i = 0; i < MAX_PAGES; i++) {
     _stprintf(prpName, _T("prpPageLayout%u"), i);
    WndProperty* wp = (WndProperty*)wf->FindByName(prpName);
    if (wp) {
      const PageLayout* currentPL = GetLayout(i);
      const PageLayout* setPL = PossiblePageLayout(wp->GetDataField()->GetAsInteger());
      if (! (*currentPL == *setPL)) {
        SetLayout(i, *setPL);
        changed = true;
      }
    }
  }
  return changed;
}
