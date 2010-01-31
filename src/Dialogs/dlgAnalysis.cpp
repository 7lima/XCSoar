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

#include "Task/TaskManager.hpp"

#include "Dialogs/Internal.hpp"
#include "SettingsComputer.hpp"
#include "SettingsTask.hpp"
#include "Math/FastMath.h"
#include "Math/Geometry.hpp"
#include "Math/Earth.hpp"
#include "Screen/Fonts.hpp"
#include "Screen/Graphics.hpp"
#include "Screen/Layout.hpp"
#include "MainWindow.hpp"
#include "GlideComputer.hpp"
#include "Atmosphere.h"
#include "Blackboard.hpp"
#include "Components.hpp"
#include "Protection.hpp"

#include "GlideSolvers/GlidePolar.hpp"

#include "RasterTerrain.h" // OLD_TASK for temporary locking of task_manager

#define MAXPAGE 8

static int page = 0;
static WndForm *wf = NULL;
static WndOwnerDrawFrame *wGrid = NULL;
static WndFrame *wInfo;
static WndButton *wCalc = NULL;

static void
SetCalcCaption(const TCHAR* caption)
{
  if (wCalc)
    wCalc->SetCaption(gettext(caption));
}

#define ANALYSIS_PAGE_BAROGRAPH    0
#define ANALYSIS_PAGE_CLIMB        1
#define ANALYSIS_PAGE_TASK_SPEED   2
#define ANALYSIS_PAGE_WIND         3
#define ANALYSIS_PAGE_POLAR        4
#define ANALYSIS_PAGE_TEMPTRACE    5
#define ANALYSIS_PAGE_TASK         6
#define ANALYSIS_PAGE_OLC          7
#define ANALYSIS_PAGE_AIRSPACE     8

static void
OnAnalysisPaint(WindowControl *Sender, Canvas &canvas)
{
  RECT rcgfx = Sender->get_client_rect();

  // background is painted in the base-class

  const FlightStatistics &fs = glide_computer.GetFlightStats();

  terrain.Lock(); // OLD_TASK temporary locking
  const TracePointVector trace = task_manager.get_trace_points();
  const TracePointVector olc = task_manager.get_olc_points();
  const GlidePolar glide_polar = task_manager.get_glide_polar();
  terrain.Unlock();

  switch (page) {
  case ANALYSIS_PAGE_BAROGRAPH:
    SetCalcCaption(_T("Settings"));
    terrain.Lock();
    fs.RenderBarograph(canvas, rcgfx, XCSoarInterface::Basic(), task_manager);
    terrain.Unlock();
    break;
  case ANALYSIS_PAGE_CLIMB:
    SetCalcCaption(_T("Task calc"));
    fs.RenderClimb(canvas, rcgfx, glide_polar);
    break;
  case ANALYSIS_PAGE_WIND:
    SetCalcCaption(_T("Set wind"));
    fs.RenderWind(canvas, rcgfx, XCSoarInterface::Basic(),
                  glide_computer.windanalyser.windstore);
    break;
  case ANALYSIS_PAGE_POLAR:
    SetCalcCaption(_T("Settings"));
    fs.RenderGlidePolar(canvas, rcgfx, XCSoarInterface::Calculated(),
                        XCSoarInterface::SettingsComputer(),
                        glide_polar);
    break;
  case ANALYSIS_PAGE_TEMPTRACE:
    SetCalcCaption(_T("Settings"));
    fs.RenderTemperature(canvas, rcgfx);
    break;
  case ANALYSIS_PAGE_TASK:
    SetCalcCaption(_T("Task calc"));
    terrain.Lock(); // OLD_TASK temporary locking
    fs.RenderTask(canvas, rcgfx, XCSoarInterface::Basic(),
                  XCSoarInterface::SettingsComputer(),
                  XCSoarInterface::SettingsMap(),
                  task_manager,
                  trace);
    terrain.Unlock();
    break;
  case ANALYSIS_PAGE_OLC:
    SetCalcCaption(_T("Optimise"));
    fs.RenderOLC(canvas, rcgfx, XCSoarInterface::Basic(),
                 XCSoarInterface::SettingsComputer(),
                 XCSoarInterface::SettingsMap(),
                 olc, trace);
    break;
  case ANALYSIS_PAGE_AIRSPACE:
    SetCalcCaption(_T("Warnings"));
    fs.RenderAirspace(canvas, rcgfx, XCSoarInterface::Basic(),
                      XCSoarInterface::Calculated(),
                      XCSoarInterface::SettingsMap(),
                      airspace_database, terrain);
    break;
  case ANALYSIS_PAGE_TASK_SPEED:
    SetCalcCaption(_T("Task calc"));
    terrain.Lock(); // OLD_TASK temporary locking
    fs.RenderSpeed(canvas, rcgfx, XCSoarInterface::Basic(), task_manager);
    terrain.Unlock();
    break;
  default:
    // should never get here!
    break;
  }
}

static void
Update(void)
{
  TCHAR sTmp[1000];
  //  WndProperty *wp;

  FlightStatistics &fs = glide_computer.GetFlightStats();

  switch (page) {
  case ANALYSIS_PAGE_BAROGRAPH:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Barograph")));
    wf->SetCaption(sTmp);
    fs.CaptionBarograph(sTmp);
    wInfo->SetCaption(sTmp);

    break;
  case ANALYSIS_PAGE_CLIMB:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Climb")));
    wf->SetCaption(sTmp);
    fs.CaptionClimb(sTmp);
    wInfo->SetCaption(sTmp);

    break;
  case ANALYSIS_PAGE_WIND:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Wind at Altitude")));
    wf->SetCaption(sTmp);
    _stprintf(sTmp, _T(" "));
    wInfo->SetCaption(sTmp);
    break;
  case ANALYSIS_PAGE_POLAR:
    _stprintf(sTmp, _T("%s: %s (Mass %d kg)"), gettext(_T("Analysis")),
              gettext(_T("Glide Polar")),
              (int)task_manager.get_glide_polar().get_all_up_weight());
    wf->SetCaption(sTmp);
    fs.CaptionPolar(sTmp, task_manager.get_glide_polar());
    wInfo->SetCaption(sTmp);
    break;
  case ANALYSIS_PAGE_TEMPTRACE:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Temp trace")));
    wf->SetCaption(sTmp);
    fs.CaptionTempTrace(sTmp);
    wInfo->SetCaption(sTmp);
    break;
  case ANALYSIS_PAGE_TASK_SPEED:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Task speed")));
    wf->SetCaption(sTmp);
    wInfo->SetCaption(_T(""));
    break;
  case ANALYSIS_PAGE_TASK:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Task")));
    wf->SetCaption(sTmp);
    fs.CaptionTask(sTmp, XCSoarInterface::Calculated());
    wInfo->SetCaption(sTmp);
    break;
  case ANALYSIS_PAGE_OLC:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("OnLine Contest")));
    wf->SetCaption(sTmp);

    TCHAR timetext1[100];
    Units::TimeToText(timetext1, (int)XCSoarInterface::Calculated().common_stats.time_olc);
    if (Layout::landscape) {
      _stprintf(sTmp,
                _T("%s:\r\n  %d %s\r\n%s: %s\r\n%s: %d %s\r\n"),
                gettext(_T("Distance")),
                (int)(DISTANCEMODIFY * XCSoarInterface::Calculated().common_stats.distance_olc),
                Units::GetDistanceName(),
                gettext(_T("Time")),
                timetext1,
                gettext(_T("Speed")),
                (int)(TASKSPEEDMODIFY * XCSoarInterface::Calculated().common_stats.speed_olc),
                Units::GetTaskSpeedName());
    } else {
      _stprintf(sTmp,
                _T("%s: %d %s\r\n%s: %s\r\n%s: %d %s\r\n"),
                gettext(_T("Distance")),
                (int)(DISTANCEMODIFY * XCSoarInterface::Calculated().common_stats.distance_olc),
                Units::GetDistanceName(),
                gettext(_T("Time")),
                timetext1,
                gettext(_T("Speed")),
                (int)(TASKSPEEDMODIFY * XCSoarInterface::Calculated().common_stats.speed_olc),
                Units::GetTaskSpeedName());
    }
    wInfo->SetCaption(sTmp);
    break;
  case ANALYSIS_PAGE_AIRSPACE:
    _stprintf(sTmp, _T("%s: %s"), gettext(_T("Analysis")),
              gettext(_T("Airspace")));
    wf->SetCaption(sTmp);
    wInfo->SetCaption(_T(" "));
    break;
  }

  wGrid->set_visible(page < MAXPAGE + 1);

  if (wGrid != NULL)
    wGrid->invalidate();
}

static void
NextPage(int Step)
{
  page += Step;

  if (page > MAXPAGE)
    page = 0;
  if (page < 0)
    page = MAXPAGE;

  Update();
}

static void
OnNextClicked(WindowControl * Sender)
{
  (void)Sender;
  NextPage(+1);
}

static void
OnPrevClicked(WindowControl * Sender)
{
  (void)Sender;
  NextPage(-1);
}

static void
OnCloseClicked(WindowControl * Sender)
{
  (void)Sender;
  wf->SetModalResult(mrOK);
}

static bool
FormKeyDown(WindowControl *Sender, unsigned key_code)
{
  (void)Sender;

  if (wGrid->GetFocused())
    return false;

  switch (key_code) {
  case VK_LEFT:
  case '6':
    ((WndButton *)wf->FindByName(_T("cmdPrev")))->set_focus();
    NextPage(-1);
    return true;

  case VK_RIGHT:
  case '7':
    ((WndButton *)wf->FindByName(_T("cmdNext")))->set_focus();
    NextPage(+1);
    return true;

  default:
    return false;
  }
}

static void
OnCalcClicked(WindowControl *Sender)
{
  (void)Sender;
  if (page == ANALYSIS_PAGE_BAROGRAPH) {
    dlgBasicSettingsShowModal();
  }
  if (page == ANALYSIS_PAGE_CLIMB) {
    wf->hide();
    dlgTaskCalculatorShowModal();
    wf->show();
  }
  if (page == ANALYSIS_PAGE_WIND) {
    dlgWindSettingsShowModal();
  }
  if (page == ANALYSIS_PAGE_POLAR) {
    dlgBasicSettingsShowModal();
  }
  if (page == ANALYSIS_PAGE_TEMPTRACE) {
    dlgBasicSettingsShowModal();
  }
  if ((page == ANALYSIS_PAGE_TASK) || (page == ANALYSIS_PAGE_TASK_SPEED)) {
    wf->hide();
    dlgTaskCalculatorShowModal();
    wf->show();
  }
  if (page == ANALYSIS_PAGE_OLC) {
    // nothing to do!
  }
  if (page == ANALYSIS_PAGE_AIRSPACE) {
    airspaceWarningEvent.trigger();
  }
  Update();
}

static CallBackTableEntry_t CallBackTable[]={
  DeclareCallBackEntry(OnAnalysisPaint),
  DeclareCallBackEntry(OnNextClicked),
  DeclareCallBackEntry(OnPrevClicked),
  DeclareCallBackEntry(OnCalcClicked),
  DeclareCallBackEntry(NULL)
};

void
dlgAnalysisShowModal(void)
{

  wf = NULL;
  wGrid = NULL;
  wInfo = NULL;
  wCalc = NULL;

  if (!Layout::landscape)
    wf = dlgLoadFromXML(CallBackTable, _T("dlgAnalysis_L.xml"),
                        XCSoarInterface::main_window, _T("IDR_XML_ANALYSIS_L"));
  else
    wf = dlgLoadFromXML(CallBackTable, _T("dlgAnalysis.xml"),
                        XCSoarInterface::main_window, _T("IDR_XML_ANALYSIS"));

  if (!wf)
    return;

  wf->SetKeyDownNotify(FormKeyDown);

  wGrid = (WndOwnerDrawFrame*)wf->FindByName(_T("frmGrid"));
  wInfo = (WndFrame *)wf->FindByName(_T("frmInfo"));
  wCalc = ((WndButton *)wf->FindByName(_T("cmdCalc")));

  ((WndButton *)wf->FindByName(_T("cmdClose")))->
      SetOnClickNotify(OnCloseClicked);

  Update();

  wf->ShowModal();

  delete wf;
  wf = NULL;
}
