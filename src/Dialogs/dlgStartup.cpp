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
#include "Screen/Layout.hpp"
#include "Gauge/LogoView.hpp"
#include "DataField/FileReader.hpp"
#include "LogFile.hpp"
#include "MainWindow.hpp"
#include "Asset.hpp"
#include "StringUtil.hpp"
#include "LocalPath.hpp"
#include "OS/PathName.hpp"
#include "Compiler.h"

static WndForm *wf = NULL;

extern TCHAR startProfileFile[];

static void
OnLogoPaint(WndOwnerDrawFrame *Sender, Canvas &canvas)
{
  canvas.clear_white();
  DrawLogo(canvas, Sender->get_client_rect());
}

static void
OnCloseClicked(gcc_unused WndButton &button)
{
  wf->SetModalResult(mrOK);
}

static void
OnQuit(gcc_unused WndButton &button)
{
  wf->SetModalResult(mrCancel);
}

static CallBackTableEntry CallBackTable[] = {
  DeclareCallBackEntry(OnLogoPaint),
  DeclareCallBackEntry(NULL)
};

bool
dlgStartupShowModal()
{
  LogStartUp(_T("Startup dialog"));

  wf = LoadDialog(CallBackTable, XCSoarInterface::main_window,
                  Layout::landscape ? _T("IDR_XML_STARTUP_L") :
                                      _T("IDR_XML_STARTUP"));
  assert(wf != NULL);

  WndProperty* wp = ((WndProperty *)wf->FindByName(_T("prpProfile")));
  assert(wp != NULL);

  DataFieldFileReader* dfe = (DataFieldFileReader*)wp->GetDataField();
  assert(dfe != NULL);

  ((WndButton *)wf->FindByName(_T("cmdClose")))
    ->SetOnClickNotify(OnCloseClicked);

  ((WndButton *)wf->FindByName(_T("cmdQuit")))->SetOnClickNotify(OnQuit);

  dfe->ScanDirectoryTop(is_altair() ? _T("config/*.prf") : _T("*.prf"));
  dfe->Lookup(startProfileFile);
  wp->RefreshDisplay();

  if (dfe->GetNumFiles() <= 2) {
    delete wf;
    return true;
  }

  if (wf->ShowModal() != mrOK) {
    delete wf;
    return false;
  }

  const TCHAR *path = dfe->GetPathFile();
  if (!string_is_empty(path)) {
    _tcscpy(startProfileFile, path);

    /* When a profile from a secondary data path is used, this path
       becomes the primary data path */
    TCHAR temp[MAX_PATH];
    SetPrimaryDataPath(DirName(path, temp));
  }

  delete wf;
  return true;
}
