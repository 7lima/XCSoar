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

#include "Dialogs/Message.hpp"
#include "Language.hpp"
#include "Form/Button.hpp"
#include "Form/Form.hpp"
#include "Form/Frame.hpp"
#include "Form/Edit.hpp"
#include "MainWindow.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/Layout.hpp"

#include <assert.h>
#include <limits.h>

static WndForm *
FindForm(WindowControl *w)
{
  assert(w != NULL);

  while (true) {
    WindowControl *parent = w->GetOwner();
    if (parent == NULL)
      return (WndForm *)w;
    w = parent;
  }
}

/**
 * This event is triggered when a button of the MessageBox is pressed
 */
static void
OnButtonClick(WindowControl * Sender)
{
  FindForm(Sender)->SetModalResult(Sender->GetTag());
}

// Message Box Replacement
/**
 * Displays a MessageBox and returns the pressed button
 * @param lpText Text displayed inside the MessageBox
 * @param lpCaption Text displayed in the Caption of the MessageBox
 * @param uType Type of MessageBox to display (OK+Cancel, Yes+No, etc.)
 * @return
 */
int WINAPI
MessageBoxX(LPCTSTR lpText, LPCTSTR lpCaption, UINT uType)
{
  WndForm *wf = NULL;
  WndFrame *wText = NULL;
  int X, Y, Width, Height;
  WndButton *wButtons[10];
  int ButtonCount = 0;
  int i, x, y, d, w, h, res, dY;
  RECT rc;

  assert(lpText != NULL);
  assert(lpCaption != NULL);

  // JMW this makes the first key if pressed quickly, ignored
  // TODO bug: doesn't work sometimes. buttons have to be pressed multiple times (TB)
  XCSoarInterface::Debounce();

  rc = XCSoarInterface::main_window.get_screen_position();

#ifdef ALTAIRSYNC
  Width = Layout::Scale(220);
  Height = Layout::Scale(160);
#else
  Width = Layout::Scale(200);
  Height = Layout::Scale(160);
#endif

  X = ((rc.right - rc.left) - Width) / 2;
  Y = ((rc.bottom - rc.top) - Height) / 2;

  y = Layout::Scale(100);
  w = Layout::Scale(60);
  h = Layout::Scale(32);

  // Create dialog
  wf = new WndForm(XCSoarInterface::main_window, _T("frmXcSoarMessageDlg"),
                   lpCaption, X, Y, Width, Height);
  wf->SetFont(MapWindowBoldFont);
  wf->SetTitleFont(MapWindowBoldFont);
  wf->SetBackColor(Color(0xDA, 0xDB, 0xAB));

  // Create text element
  wText = new WndFrame(wf, _T("frmMessageDlgText"),
                       0, Layout::Scale(5), Width, Height);

  wText->SetCaption(lpText);
  wText->SetFont(MapWindowBoldFont);
  wText->SetCaptionStyle(DT_EXPANDTABS | DT_CENTER | DT_NOCLIP | DT_WORDBREAK);
  // | DT_VCENTER

  /* TODO code: this doesnt work to set font height
  dY = wText->GetLastDrawTextHeight() - Height;
  */
  dY = Layout::Scale(-40);
  wText->resize(Width, wText->GetTextHeight() + 5);
  wf->resize(Width, wf->get_size().cy + dY);

  y += dY;

  // Create buttons
  uType = uType & 0x000f;
  if (uType == MB_OK || uType == MB_OKCANCEL) {
    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("OK")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDOK);
    ButtonCount++;
  }

  if (uType == MB_YESNO || uType == MB_YESNOCANCEL) {
    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("Yes")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDYES);
    ButtonCount++;

    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("No")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDNO);
    ButtonCount++;
  }

  if (uType == MB_ABORTRETRYIGNORE || uType == MB_RETRYCANCEL) {
    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("Retry")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDRETRY);
    ButtonCount++;
  }

  if (uType == MB_OKCANCEL || uType == MB_RETRYCANCEL || uType == MB_YESNOCANCEL) {
    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("Cancel")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDCANCEL);
    ButtonCount++;
  }

  if (uType == MB_ABORTRETRYIGNORE) {
    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("Abort")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDABORT);
    ButtonCount++;

    wButtons[ButtonCount] =
        new WndButton(wf, _T(""), gettext(_T("Ignore")), 0, y, w, h, OnButtonClick);

    wButtons[ButtonCount]->SetTag(IDIGNORE);
    ButtonCount++;
  }

  d = Width / (ButtonCount);
  x = d / 2 - w / 2;

  // Move buttons to the right positions
  for (i = 0; i < ButtonCount; i++) {
    wButtons[i]->move(x, y);
    x += d;
  }

  // Show MessageBox and save result
  res = wf->ShowModal();

  delete wf;

#ifdef ALTAIRSYNC
  // force a refresh of the window behind
  InvalidateRect(hWnd,NULL,true);
  UpdateWindow(hWnd);
#endif

  return(res);
}
