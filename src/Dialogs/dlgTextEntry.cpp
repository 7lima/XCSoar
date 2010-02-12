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

#include "Dialogs/Internal.hpp"
#include "MainWindow.hpp"
#include "Compatibility/string.h"
#include "SettingsUser.hpp"
#include "Appearance.hpp"
#include "Asset.hpp"
#include "StringUtil.hpp"

#include <algorithm>

using std::min;

static WndForm *wf=NULL;
static WndOwnerDrawFrame *wGrid=NULL;

#define MAX_TEXTENTRY 40
static unsigned int cursor = 0;
static int lettercursor=0;
static int max_width = MAX_TEXTENTRY;

static TCHAR edittext[MAX_TEXTENTRY];

static TCHAR EntryLetters[] = _T(" ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890.-");

#define MAXENTRYLETTERS (sizeof(EntryLetters)/sizeof(EntryLetters[0])-1)

static void
OnCloseClicked(gcc_unused WndButton &button)
{
  wf->SetModalResult(mrOK);
}


static void
OnTextPaint(WindowControl *Sender, Canvas &canvas)
{
  // background is painted in the base-class

  // Do the actual painting of the text

  SIZE tsize = canvas.text_size(edittext);
  SIZE tsizec = canvas.text_size(edittext, cursor);
  SIZE tsizea = canvas.text_size(edittext, cursor + 1);

  POINT p[5];
  p[0].x = 10;
  p[0].y = 20;

  p[2].x = p[0].x + tsizec.cx;
  p[2].y = p[0].y + tsize.cy+5;

  p[3].x = p[0].x + tsizea.cx;
  p[3].y = p[0].y + tsize.cy+5;

  p[1].x = p[2].x;
  p[1].y = p[2].y-2;

  p[4].x = p[3].x;
  p[4].y = p[3].y-2;

  canvas.white_pen();
  canvas.polyline(p + 1, 4);

  /*
  int x = (int)((xv-x_min)*xscale)+rc.left-tsize.cx/2;
  int y = (int)((y_max-yv)*yscale)+rc.top-tsize.cy/2;
  */
  canvas.background_opaque();
  canvas.text_opaque(p[0].x, p[0].y, NULL, edittext);
  canvas.background_transparent();
}



static void UpdateCursor(void) {
  if (lettercursor >= (int)MAXENTRYLETTERS)
    lettercursor = 0;
  if (lettercursor<0)
    lettercursor = MAXENTRYLETTERS-1;
  edittext[cursor] = EntryLetters[lettercursor];

  if (wGrid != NULL)
    wGrid->invalidate();

}


static void MoveCursor(void) {
  if (cursor>=_tcslen(edittext)) {
    edittext[cursor+1] = 0;
  }
  for (lettercursor = 0; lettercursor < (int)MAXENTRYLETTERS; lettercursor++) {
    if (edittext[cursor]== EntryLetters[lettercursor])
      break;
  }
  if (lettercursor== MAXENTRYLETTERS) {
    lettercursor = 0;
    edittext[cursor] = EntryLetters[lettercursor];
  }
  if (edittext[cursor]== 0) {
    lettercursor= 0;
  }
  UpdateCursor();
}

static bool
FormKeyDown(WindowControl *Sender, unsigned key_code)
{
  (void)Sender;

  switch (key_code) {
    case VK_LEFT:
      if (cursor<1)
        return true; // min width
      cursor--;
      MoveCursor();
    return true;

    case VK_RIGHT:
      if ((int)cursor>=(max_width-2))
        return true; // max width
      cursor++;
      MoveCursor();
    return true;

    case VK_UP:
      lettercursor--;
      UpdateCursor();
    return true;

    case VK_DOWN:
      lettercursor++;
      UpdateCursor();
    return true;

    case VK_RETURN:
      wf->SetModalResult(mrOK);
    return true;

  default:
    return false;
  }
}



static CallBackTableEntry_t CallBackTable[]={
  DeclareCallBackEntry(OnTextPaint),
  DeclareCallBackEntry(NULL)
};


static void
OnLeftClicked(WndButton &button)
{
  FormKeyDown(&button, VK_LEFT);
}

static void
OnRightClicked(WndButton &button)
{
  FormKeyDown(&button, VK_RIGHT);
}

static void
OnUpClicked(WndButton &button)
{
  FormKeyDown(&button, VK_UP);
}

static void
OnDownClicked(WndButton &button)
{
  FormKeyDown(&button, VK_DOWN);
}



static void dlgTextEntryHighscoreType(TCHAR *text, int width)
{
  wf = NULL;
  wGrid = NULL;

  if (width==0) {
    width = MAX_TEXTENTRY;
  }
  max_width = min(MAX_TEXTENTRY, width);

  wf = dlgLoadFromXML(CallBackTable,
                      is_altair() ? _T("dlgTextEntry_T.xml")
                      : _T("dlgTextEntry.xml"),
		      XCSoarInterface::main_window,
                      is_altair() ? _T("IDR_XML_TEXTENTRY_T")
                      : _T("IDR_XML_TEXTENTRY"));
  if (!wf) return;

  wGrid = (WndOwnerDrawFrame*)wf->FindByName(_T("frmGrid"));

  WndButton* wb;
  wb = (WndButton *)(wf->FindByName(_T("cmdClose")));
  if (wb) {
    wb->SetOnClickNotify(OnCloseClicked);
  }

  wb = (WndButton *)(wf->FindByName(_T("cmdLeft")));
  if (wb) {
    wb->SetOnClickNotify(OnLeftClicked);
  }

  wb = (WndButton *)(wf->FindByName(_T("cmdRight")));
  if (wb) {
    wb->SetOnClickNotify(OnRightClicked);
  }

  wb = (WndButton *)(wf->FindByName(_T("cmdUp")));
  if (wb) {
    wb->SetOnClickNotify(OnUpClicked);
  }

  wb = (WndButton *)(wf->FindByName(_T("cmdDown")));
  if (wb) {
    wb->SetOnClickNotify(OnDownClicked);
  }


  cursor = 0;
  edittext[0]= 0;
  edittext[1]= 0;
  if (!string_is_empty(text)) {
    _tcsupr(text);
    _tcsncpy(edittext, text, max_width-1);
    edittext[max_width-1]= 0;
  }
  MoveCursor();

  wf->SetKeyDownNotify(FormKeyDown);

  wf->ShowModal();

  _tcsncpy(text, edittext, max_width);
  text[max_width-1]=0;

  // strip trailing spaces
  int len = _tcslen(text)-1;
  while ((len>0) && (text[len] == _T(' '))) {
    text[len] = 0;
    len--;
  }

  delete wf;
}


bool dlgTextEntryShowModal(TCHAR *text, int width)
{
  bool bRetVal=false;
  switch (Appearance.TextInputStyle)
    {
    case tiKeyboard:
      bRetVal=dlgTextEntryKeyboardShowModal(text, width);
      break;
    case tiHighScore:
    default:
      dlgTextEntryHighscoreType(text, width);
      bRetVal=true;
      break;
    }
  return bRetVal;
}
