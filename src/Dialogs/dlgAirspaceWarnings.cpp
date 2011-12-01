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

#include "Dialogs/AirspaceWarningDialog.hpp"
#include "Dialogs/Airspace.hpp"
#include "Dialogs/Internal.hpp"
#include "Units/Units.hpp"
#include "Units/UnitsFormatter.hpp"
#include "Components.hpp"
#include "Screen/Layout.hpp"
#include "Screen/Key.h"
#include "Airspace/AirspaceWarning.hpp"
#include "Airspace/ProtectedAirspaceWarningManager.hpp"
#include "Airspace/AirspaceWarningManager.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "Util/TrivialArray.hpp"

#include "Compiler.h"

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

struct WarningItem {
  const AbstractAirspace *airspace;
  AirspaceWarning::State state;
  AirspaceInterceptSolution solution;
  bool ack_expired, ack_day;

  WarningItem() = default;

  WarningItem(const AirspaceWarning &w)
    :airspace(&w.GetAirspace()),
     state(w.GetWarningState()),
     solution(w.GetSolution()),
     ack_expired(w.IsAckExpired()), ack_day(w.GetAckDay()) {}

  bool operator==(const AbstractAirspace &other) const {
    return &other == airspace;
  }
};

typedef TrivialArray<WarningItem, 64u> WarningList;

static WndForm *wf = NULL;
static WndButton *wbAck1 = NULL; // frmAck1 = Ack warn
static WndButton *wbAck2 = NULL; // frmAck2 = Ack day
static WndButton *wbAck = NULL; // frmAck = Ack space
static WndButton *wbEnable = NULL; // Enable

static WndListFrame *wAirspaceList = NULL;
static gcc_constexpr_data Color inside_color(254,50,50);
static gcc_constexpr_data Color near_color(254,254,50);
static gcc_constexpr_data Color inside_ack_color(254,100,100);
static gcc_constexpr_data Color near_ack_color(254,254,100);
static bool AutoClose = true;

static WarningList warning_list;

static const AbstractAirspace* CursorAirspace = NULL; // Current list cursor airspace
static const AbstractAirspace* FocusAirspace = NULL;  // Current action airspace

static const AbstractAirspace *
GetSelectedAirspace()
{
  return HasPointer() || FocusAirspace == NULL
    ? CursorAirspace
    : FocusAirspace;
}

static void
UpdateButtons()
{
  const AbstractAirspace *airspace = GetSelectedAirspace();
  if (airspace == NULL) {
    wbAck1->set_visible(false);
    wbAck2->set_visible(false);
    wbAck->set_visible(false);
    wbEnable->set_visible(false);
    return;
  }

  bool ack_expired, ack_day, inside;

  {
    ProtectedAirspaceWarningManager::ExclusiveLease lease(*airspace_warnings);
    const AirspaceWarning &warning = lease->GetWarning(*airspace);
    ack_expired = warning.IsAckExpired();
    ack_day = warning.GetAckDay();
    inside = warning.GetWarningState() == AirspaceWarning::WARNING_INSIDE;
  }

  wbAck1->set_visible(ack_expired && !inside);
  wbAck2->set_visible(!ack_day);
  wbAck->set_visible(ack_expired && inside);
  wbEnable->set_visible(!ack_expired);
}

static void
update_list();

static void
AirspaceWarningCursorCallback(unsigned i)
{
  CursorAirspace = i < warning_list.size()
    ? warning_list[i].airspace
    : NULL;

  UpdateButtons();
}

static void
OnAirspaceListEnter(gcc_unused unsigned i)
{
  if (!HasPointer())
    /* on platforms without a pointing device (e.g. ALTAIR), allow
       "focusing" an airspace by pressing enter */
    FocusAirspace = CursorAirspace;
  else if (CursorAirspace != NULL)
    dlgAirspaceDetails(*CursorAirspace);
}

static bool
HasWarning()
{
  ProtectedAirspaceWarningManager::Lease lease(*airspace_warnings);
  for (auto i = lease->begin(), end = lease->end(); i != end; ++i)
    if (i->IsAckExpired())
      return true;

  return false;
}

static void
Hide()
{
  wf->hide();
  wf->SetModalResult(mrOK);
}

static void
AutoHide()
{
  // Close the dialog if no warning exists and AutoClose is set
  if (!HasWarning() && AutoClose)
    Hide();
}

/** ack inside */
static void
Ack()
{
  const AbstractAirspace *airspace = GetSelectedAirspace();
  if (airspace != NULL) {
    airspace_warnings->acknowledge_inside(*airspace, true);
    update_list();
    AutoHide();
  }
}

static void
OnAckClicked(gcc_unused WndButton &Sender)
{
  Ack();
}

/** ack warn */
static void
Ack1()
{
  const AbstractAirspace *airspace = GetSelectedAirspace();
  if (airspace != NULL) {
    airspace_warnings->acknowledge_warning(*airspace, true);
    update_list();
    AutoHide();
  }
}

static void
OnAck1Clicked(gcc_unused WndButton &Sender)
{
  Ack1();
}

/** ack day */
static void
Ack2()
{
  const AbstractAirspace *airspace = GetSelectedAirspace();
  if (airspace != NULL) {
    airspace_warnings->acknowledge_day(*airspace, true);
    update_list();
    AutoHide();
  }
}

static void
OnAck2Clicked(gcc_unused WndButton &Sender)
{
  Ack2();
}

/** unack */
static void
Enable()
{
  const AbstractAirspace *airspace = GetSelectedAirspace();
  if (airspace == NULL)
    return;

  {
    ProtectedAirspaceWarningManager::ExclusiveLease lease(*airspace_warnings);
    AirspaceWarning *w = lease->GetWarningPtr(*airspace);
    if (w == NULL)
      return;

    w->AcknowledgeInside(false);
    w->AcknowledgeWarning(false);
    w->AcknowledgeDay(false);
  }

 update_list();
}

static void
OnEnableClicked(gcc_unused WndButton &Sender)
{
  Enable();
}

static void
OnCloseClicked(gcc_unused WndButton &Sender)
{
  Hide();
}

static bool
OnKeyDown(gcc_unused WndForm &Sender, unsigned key_code)
{
  switch (key_code){
    case VK_ESCAPE:
      Hide();
    return true;

#ifdef GNAV
    case VK_APP1:
    case '6':
      Ack1();
    return true;

    case VK_APP2:
    case '7':
      Ack();
    return true;

    case VK_APP3:
    case '8':
      Ack2();
    return true;

    case VK_APP4:
    case '9':
      Enable();
    return true;
#endif

  default:
    return false;
  }
}

static void
OnAirspaceListItemPaint(Canvas &canvas, const PixelRect paint_rc, unsigned i)
{
  TCHAR sTmp[128];
  const int paint_rc_margin = 2;   ///< This constant defines the margin that should be respected for renderring within the paint_rc area.

  if (i == 0 && warning_list.empty()) {
    /* the warnings were emptied between the opening of the dialog and
       this refresh, so only need to display "No Warnings" for top
       item, otherwise exit immediately */
    canvas.text(paint_rc.left + Layout::Scale(paint_rc_margin),
                paint_rc.top + Layout::Scale(paint_rc_margin), _("No Warnings"));
    return;
  }

  assert(i < warning_list.size());

  const WarningItem &warning = warning_list[i];
  const AbstractAirspace &as = *warning.airspace;
  const AirspaceInterceptSolution &solution = warning.solution;

  tstring sName = as.GetNameText();
  tstring sTop = as.GetTopText(true);
  tstring sBase = as.GetBaseText(true);
  tstring sType = as.GetTypeText(true);

  const UPixelScalar TextHeight = 12, TextTop = 1;

  const int statusColWidth = canvas.CalcTextWidth(_T("inside"));     //<-- word "inside" is used as the etalon, because it is longer than "near" and currently (9.4.2011) there is no other possibility for the status text.
  const int heightColWidth = canvas.CalcTextWidth(_T("1888 m AGL")); // <-- "1888" is used in order to have enough space for 4-digit heights with "AGL"


  /// Dynamic columns scaling - "name" column is flexible, altitude and state columns are fixed-width.
  const PixelScalar Col0LeftScreenCoords = Layout::FastScale(paint_rc_margin),
    Col2LeftScreenCoords = paint_rc.right - Layout::FastScale(paint_rc_margin) - (statusColWidth + 2 * Layout::FastScale(paint_rc_margin)),
    Col1LeftScreenCoords = Col2LeftScreenCoords - Layout::FastScale(paint_rc_margin) - heightColWidth;

  PixelRect rcTextClip;

  rcTextClip = paint_rc;
  rcTextClip.right = Col1LeftScreenCoords - Layout::FastScale(paint_rc_margin);

  Color old_text_color = canvas.GetTextColor();
  if (!warning.ack_expired)
    canvas.SetTextColor(COLOR_GRAY);

  { // name, altitude info
    _stprintf(sTmp, _T("%-20s"), sName.c_str());

    canvas.text_clipped(paint_rc.left + Col0LeftScreenCoords,
                        paint_rc.top + Layout::Scale(TextTop),
                        rcTextClip, sTmp);

    _stprintf(sTmp, _T("%-20s"), sTop.c_str());
    canvas.text(paint_rc.left + Col1LeftScreenCoords,
                paint_rc.top + Layout::Scale(TextTop), sTmp);

    _stprintf(sTmp, _T("%-20s"), sBase.c_str());
    canvas.text(paint_rc.left + Col1LeftScreenCoords,
                paint_rc.top + Layout::Scale(TextTop + TextHeight),
                sTmp);
  }

  if (warning.state != AirspaceWarning::WARNING_INSIDE &&
      warning.state > AirspaceWarning::WARNING_CLEAR &&
      solution.IsValid()) {

    _stprintf(sTmp, _T("%d secs"),
              (int)solution.elapsed_time);

    if (positive(solution.distance))
      _stprintf(sTmp + _tcslen(sTmp), _T(" dist %d m"),
                (int)solution.distance);
    else {
      /* the airspace is right above or below us - show the vertical
         distance */
      _tcscat(sTmp, _T(" vertical "));

      fixed delta = solution.altitude - CommonInterface::Basic().nav_altitude;
      Units::FormatUserArrival(delta,
                               sTmp + _tcslen(sTmp), 128 - _tcslen(sTmp),
                               true);
    }

    canvas.text_clipped(paint_rc.left + Col0LeftScreenCoords,
                        paint_rc.top + Layout::Scale(TextTop + TextHeight),
                        rcTextClip, sTmp);
  }

  /* draw the warning state indicator */

  Color state_color;
  const TCHAR *state_text;

  if (warning.state == AirspaceWarning::WARNING_INSIDE) {
    state_color = warning.ack_expired ? inside_color : inside_ack_color;
    state_text = _T("inside");
  } else if (warning.state > AirspaceWarning::WARNING_CLEAR) {
    state_color = warning.ack_expired ? near_color : near_ack_color;
    state_text = _T("near");
  } else {
    state_color = COLOR_WHITE;
    state_text = NULL;
  }

  const PixelSize state_text_size =
    canvas.CalcTextSize(state_text != NULL ? state_text : _T("W"));

  if (state_color != COLOR_WHITE) {
    /* colored background */
    PixelRect rc;

    rc.left = paint_rc.left + Col2LeftScreenCoords;
    rc.top = paint_rc.top + Layout::FastScale(paint_rc_margin);
    rc.right = paint_rc.right - Layout::FastScale(paint_rc_margin);
    rc.bottom = paint_rc.bottom - Layout::FastScale(paint_rc_margin);

    canvas.DrawFilledRectangle(rc, state_color);
  }

  if (state_text != NULL) {
    // -- status text will be centered inside its table cell:
    canvas.text(paint_rc.left + Col2LeftScreenCoords + Layout::FastScale(paint_rc_margin) + (statusColWidth / 2)  - (canvas.CalcTextWidth(state_text) / 2),
                (paint_rc.bottom + paint_rc.top - state_text_size.cy) / 2,
                state_text);
  }

  if (!warning.ack_expired)
    canvas.SetTextColor(old_text_color);
}

static void
CopyList()
{
  const ProtectedAirspaceWarningManager::Lease lease(*airspace_warnings);

  warning_list.clear();
  for (auto i = lease->begin(), end = lease->end();
       i != end && !warning_list.full(); ++i)
    warning_list.push_back(*i);
}

static void
update_list()
{
  CopyList();

  if (!warning_list.empty()) {
    wAirspaceList->SetLength(warning_list.size());

    int i = -1;
    if (CursorAirspace != NULL) {
      auto it = std::find(warning_list.begin(), warning_list.end(),
                          *CursorAirspace);
      if (it != warning_list.end()) {
        i = it - warning_list.begin();
        wAirspaceList->SetCursorIndex(i);
      }
    }

    if (i < 0)
      /* the selection may have changed, update CursorAirspace */
      AirspaceWarningCursorCallback(wAirspaceList->GetCursorIndex());
  } else {
    wAirspaceList->SetLength(1);
    CursorAirspace = NULL;
  }
  wAirspaceList->invalidate();
  UpdateButtons();
  AutoHide();
}

static void
OnTimer(gcc_unused WndForm &Sender)
{
  update_list();
}

bool
dlgAirspaceWarningVisible()
{
  return (wf != NULL);
}

static gcc_constexpr_data CallBackTableEntry CallBackTable[] = {
  DeclareCallBackEntry(OnAckClicked),
  DeclareCallBackEntry(OnAck1Clicked),
  DeclareCallBackEntry(OnAck2Clicked),
  DeclareCallBackEntry(OnEnableClicked),
  DeclareCallBackEntry(OnCloseClicked),
  DeclareCallBackEntry(NULL)
};

void
dlgAirspaceWarningsShowModal(SingleWindow &parent, bool auto_close)
{
  if (dlgAirspaceWarningVisible())
    return;

  assert(airspace_warnings != NULL);
  assert(warning_list.empty());

  wf = LoadDialog(CallBackTable, parent, _T("IDR_XML_AIRSPACEWARNINGS"));
  assert(wf != NULL);

  wbAck1 = (WndButton *)wf->FindByName(_T("frmAck1"));
  wbAck2 = (WndButton *)wf->FindByName(_T("frmAck2"));
  wbAck = (WndButton *)wf->FindByName(_T("frmAck"));
  wbEnable = (WndButton *)wf->FindByName(_T("frmEnable"));
  assert(wbAck1 != NULL);
  assert(wbAck2 != NULL);
  assert(wbAck != NULL);
  assert(wbEnable != NULL);

  wf->SetKeyDownNotify(OnKeyDown);

  wAirspaceList = (WndListFrame*)wf->FindByName(_T("frmAirspaceWarningList"));
  assert(wAirspaceList != NULL);
  wAirspaceList->SetPaintItemCallback(OnAirspaceListItemPaint);
  wAirspaceList->SetCursorCallback(AirspaceWarningCursorCallback);
  wAirspaceList->SetActivateCallback(OnAirspaceListEnter);

  AutoClose = auto_close;
  update_list();

  // JMW need to deselect everything on new reopening of dialog
  CursorAirspace = NULL;
  FocusAirspace = NULL;

  wf->SetTimerNotify(OnTimer);
  wAirspaceList->SetCursorIndex(0);
  wf->ShowModal();
  wf->SetTimerNotify(NULL);

  delete wf;

  // Needed for dlgAirspaceWarningVisible()
  wf = NULL;

  warning_list.clear();
}
