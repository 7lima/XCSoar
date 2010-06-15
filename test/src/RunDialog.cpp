/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000 - 2009

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
#include "Screen/Blank.hpp"
#include "Screen/SingleWindow.hpp"
#include "wcecompat/ts_string.h"
#include "Screen/Layout.hpp"
#include "InputEvents.h"
#include "MapWindow.hpp"
#include "StatusMessage.hpp"
#include "Asset.hpp"

#include <tchar.h>
#include <stdio.h>

#ifdef HAVE_BLANK
int DisplayTimeOut = 0;
#endif

#ifdef WIN32
#include <shellapi.h>
#endif

void
LocalPath(TCHAR *buf, const TCHAR* file)
{
  _tcscpy(buf, file);
}

#ifdef _UNICODE
void
LocalPath(char *buf, const TCHAR* file)
{
//  strcpy(buf, (const char *)file);
  unicode2ascii(file, buf);
  printf("File %s\n",buf);
}
#endif

void dlgHelpShowModal(const TCHAR* Caption, const TCHAR* HelpText)
{
}

pt2Event
InputEvents::findEvent(const TCHAR *)
{
  return NULL;
}

#ifndef ENABLE_SDL
bool
MapWindow::identify(HWND hWnd)
{
  return false;
}
#endif /* !ENABLE_SDL */

StatusMessageList::StatusMessageList() {}

static const StatusMessageList messages;

Font MapWindowFont;
Font MapWindowBoldFont;
Font TitleWindowFont;
Font CDIWindowFont;
Font InfoWindowFont;

#ifndef WIN32
int main(int argc, char **argv)
#else
int WINAPI
WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
#ifdef _WIN32_WCE
        LPWSTR lpCmdLine,
#else
        LPSTR lpCmdLine2,
#endif
        int nCmdShow)
#endif
{
#ifdef WIN32
#ifndef _WIN32_WCE
  /* on Windows (non-CE), the lpCmdLine argument is narrow, and we
     have to use GetCommandLine() to get the UNICODE string */
  LPCTSTR lpCmdLine = GetCommandLine();
#endif

#ifdef _WIN32_WCE
  int argc = 2;

  WCHAR arg0[] = _T("");
  LPWSTR argv[] = { arg0, lpCmdLine, NULL };
#else
  int argc;
  LPWSTR* argv = CommandLineToArgvW(lpCmdLine, &argc);
#endif

  CommonInterface::hInst = hInstance;

  PaintWindow::register_class(hInstance);
#endif

  if (argc < 2) {
    fprintf(stderr, "Usage: %s XMLFILE\n", argv[0]);
    return 1;
  }

  Layout::Initialize(320,240);
  SingleWindow main_window;
  main_window.set(_T("STATIC"), _T("RunDialog"),
                  0, 0, 320, 240);
  main_window.show();

  WndForm *form = dlgLoadFromXML(NULL, argv[1], main_window, argv[2]);
  if (form == NULL) {
    fprintf(stderr, "Failed to load XML file '%S' / ID '%S'\n", argv[1], argv[2]);
    return 1;
  }

  form->ShowModal();
  delete form;

  return 0;
}
