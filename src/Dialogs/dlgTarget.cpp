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
#include "Screen/Layout.hpp"
#include "Protection.hpp"
#include "Blackboard.hpp"
#include "SettingsTask.hpp"
#include "Units.hpp"
#include "Math/Earth.hpp"
#include "LogFile.hpp"
#include "Calculations.h"
#include "MapWindow.h"
#include "Math/Geometry.hpp"
#include "DataField/Enum.hpp"
#include "MainWindow.hpp"
#include "WayPoint.hpp"
#include "Protection.hpp"
#include "WayPointList.hpp"
#include "Components.hpp"
#include "Task.h"

#include <math.h>

#include <algorithm>

using std::min;
using std::max;

static WndForm *wf=NULL;
static WindowControl *btnMove = NULL;
static unsigned ActiveTaskPointOnEntry = 0;


static double Range = 0;
static double Radial = 0;
static unsigned target_point = 0;
static bool TargetMoveMode = false;

static void OnOKClicked(WindowControl * Sender){
  (void)Sender;
  wf->SetModalResult(mrOK);
}




static void MoveTarget(double adjust_angle) {
  if (!task.getSettings().AATEnabled) return;
  if (target_point==0) return;
  if (!task.ValidTaskPoint(target_point)) return;
  if (!task.ValidTaskPoint(target_point+1)) return;
  if (target_point < task.getActiveIndex()) return;

  GEOPOINT target_location;
  double bearing, distance;

  TASK_POINT tp = task.getTaskPoint(target_point);

  distance = 500;
  if (tp.AATType == AAT_SECTOR) {
    distance = max(tp.AATSectorRadius/20.0,distance);
  } else {
    distance = max(tp.AATCircleRadius/20.0,distance);
  }

  // JMW illegal
  bearing = AngleLimit360(XCSoarInterface::main_window.map.GetDisplayAngle()
                          + adjust_angle);

  FindLatitudeLongitude (tp.AATTargetLocation,
                         bearing, distance,
                         &target_location);

  if (task.InAATTurnSector(target_location,
                           target_point)) {
    if (XCSoarInterface::Calculated().IsInSector
        && (target_point == task.getActiveIndex())) {
      // set range/radial for inside sector
      double course_bearing, target_bearing;
      DistanceBearing(task.getTargetLocation(target_point-1),
                      XCSoarInterface::Basic().Location,
                      NULL, &course_bearing);

      DistanceBearing(XCSoarInterface::Basic().Location,
                      target_location,
                      &distance, &target_bearing);
      bearing = AngleLimit180(target_bearing-course_bearing);

      if (fabs(bearing)<90.0) {
        tp.AATTargetLocation = target_location;
        Radial = bearing;
        tp.AATTargetOffsetRadial = Radial;
        Range =
          task.FindInsideAATSectorRange(XCSoarInterface::Basic().Location,
                                        target_point,
                                        target_bearing,
                                        distance);
        tp.AATTargetOffsetRadius = Range;
        task.setTaskPoint(target_point, tp);
        task.SetTargetModified();
      }
    } else {
      // OK to change it..
      tp.AATTargetLocation = target_location;

      // set range/radial for outside sector
      DistanceBearing(task.getTaskPointLocation(target_point),
                      tp.AATTargetLocation,
                      &distance, &bearing);
      bearing = AngleLimit180(bearing-tp.Bisector);
      if (tp.AATType == AAT_SECTOR) {
        Range = (fabs(distance)/tp.AATSectorRadius)*2-1;
      } else {
        if (fabs(bearing)>90.0) {
          distance = -distance;
          bearing = AngleLimit180(bearing+180);
        }
        Range = distance/tp.AATCircleRadius;
      }
      tp.AATTargetOffsetRadius = Range;
      tp.AATTargetOffsetRadial = bearing;
      Radial = bearing;
      task.setTaskPoint(target_point, tp);
      task.SetTargetModified();
    }
  }
}


static void DragTarget(const GEOPOINT target_location) {
  if (!task.getSettings().AATEnabled) return;
  if (target_point==0) return;
  if (!task.ValidTaskPoint(target_point)) return;
  if (!task.ValidTaskPoint(target_point+1)) return;
  if (target_point < task.getActiveIndex()) return;

  double distance, bearing;
  TASK_POINT tp = task.getTaskPoint(target_point);

  if (task.InAATTurnSector(target_location,
                           target_point)) {
    if (XCSoarInterface::Calculated().IsInSector
        && (target_point == task.getActiveIndex())) {
      // set range/radial for inside sector
      double course_bearing, target_bearing;
      DistanceBearing(task.getTargetLocation(target_point-1),
                      XCSoarInterface::Basic().Location,
                      NULL, &course_bearing);

      DistanceBearing(XCSoarInterface::Basic().Location,
                      target_location,
                      &distance, &target_bearing);
      bearing = AngleLimit180(target_bearing-course_bearing);

      if (fabs(bearing)<90.0) {
        tp.AATTargetLocation = target_location;
        Radial = bearing;
        tp.AATTargetOffsetRadial = Radial;
        Range =
          task.FindInsideAATSectorRange(XCSoarInterface::Basic().Location,
                                        target_point,
                                        target_bearing,
                                        distance);
        tp.AATTargetOffsetRadius = Range;
        task.setTaskPoint(target_point, tp);
	task.SetTargetModified();
      }
    } else {
      // OK to change it..
      tp.AATTargetLocation = target_location;

      // set range/radial for outside sector
      DistanceBearing(task.getTaskPointLocation(target_point),
                      tp.AATTargetLocation,
                      &distance, &bearing);
      bearing = AngleLimit180(bearing-tp.Bisector);
      if (tp.AATType == AAT_SECTOR) {
        Range = (fabs(distance)/task.getTaskPoint(target_point).AATSectorRadius)*2-1;
      } else {
        if (fabs(bearing)>90.0) {
          distance = -distance;
          bearing = AngleLimit180(bearing+180);
        }
        Range = distance/tp.AATCircleRadius;
      }
      tp.AATTargetOffsetRadius = Range;
      tp.AATTargetOffsetRadial = bearing;
      Radial = bearing;
      task.setTaskPoint(target_point, tp);
      task.SetTargetModified();
    }
  }
}

static bool
FormKeyDown(WindowControl *Sender, unsigned key_code)
{
	(void)Sender;
  switch(key_code){
    case '2':
#ifdef GNAV
    case VK_F2:
#endif
      MoveTarget(0);
    return true;

    case '3':
#ifdef GNAV
    case VK_F3:
#endif
      MoveTarget(180);
    return true;

    case '6':
      MoveTarget(270);
    return true;

    case '7':
      MoveTarget(90);
    return true;
  }

  if (TargetMoveMode) {
    StartupStore(_T("moving\n"));
    switch (key_code) {
    case VK_UP:
      MoveTarget(0);
      return true;

    case VK_DOWN:
      MoveTarget(180);
      return true;

    case VK_LEFT:
      MoveTarget(270);
      return true;

    case VK_RIGHT:
      MoveTarget(90);
      return true;
    }
  }

  return false;
}



static void RefreshCalculator(void) {
  WndProperty* wp;

  task.RefreshTask(XCSoarInterface::SettingsComputer(),
                   XCSoarInterface::Basic());
  RefreshTaskStatistics();
  target_point = max(target_point,task.getActiveIndex());

  bool nodisplay = !task.getSettings().AATEnabled
    || (target_point==0)
    || !task.ValidTaskPoint(target_point+1);

  if (btnMove) {
    btnMove->set_visible(!nodisplay);
    if (nodisplay)
      TargetMoveMode = false;
  }

  nodisplay = nodisplay || TargetMoveMode;

  wp = (WndProperty*)wf->FindByName(_T("prpTaskPoint"));
  if (wp) {
    wp->set_visible(!TargetMoveMode);
  }

  WindowControl* wc = (WindowControl*)wf->FindByName(_T("btnOK"));
  if (wc) {
    wc->set_visible(!TargetMoveMode);
  }

  wp = (WndProperty*)wf->FindByName(_T("prpAATTargetLocked"));
  if (wp) {
    wp->GetDataField()->Set(task.getTaskPoint(target_point).AATTargetLocked);
    wp->RefreshDisplay();
    wp->set_visible(!nodisplay);
  }

  wp = (WndProperty*)wf->FindByName(_T("prpRange"));
  if (wp) {
    wp->GetDataField()->SetAsFloat(Range*100.0);
    wp->RefreshDisplay();
    wp->set_visible(!nodisplay);
  }

  wp = (WndProperty*)wf->FindByName(_T("prpRadial"));
  if (wp) {
    wp->GetDataField()->SetAsFloat(Radial);
    wp->RefreshDisplay();
    wp->set_visible(!nodisplay);
  }

  // update outputs
  double dd = XCSoarInterface::Calculated().TaskTimeToGo;
  if ((XCSoarInterface::Calculated().TaskStartTime>0.0)&&(XCSoarInterface::Calculated().Flying)) {
    dd += XCSoarInterface::Basic().Time-XCSoarInterface::Calculated().TaskStartTime;
  }
  dd= min(24.0*60.0,dd/60.0);
  wp = (WndProperty*)wf->FindByName(_T("prpAATEst"));
  if (wp) {
    wp->GetDataField()->SetAsFloat(dd);
    wp->RefreshDisplay();
  }
  wp = (WndProperty*)wf->FindByName(_T("prpAATDelta"));
  if (wp) {
    wp->GetDataField()->SetAsFloat(dd-task.getSettings().AATTaskLength);
    wp->set_visible(task.getSettings().AATEnabled);
    wp->RefreshDisplay();
  }

  double v1;
  if (XCSoarInterface::Calculated().TaskTimeToGo>0) {
    v1 = XCSoarInterface::Calculated().TaskDistanceToGo/
      XCSoarInterface::Calculated().TaskTimeToGo;
  } else {
    v1 = 0;
  }

  wp = (WndProperty*)wf->FindByName(_T("prpSpeedRemaining"));
  if (wp) {
    wp->GetDataField()->SetAsFloat(v1*TASKSPEEDMODIFY);
    wp->GetDataField()->SetUnits(Units::GetTaskSpeedName());
    wp->RefreshDisplay();
  }

  wp = (WndProperty*)wf->FindByName(_T("prpSpeedAchieved"));
  if (wp) {
    wp->GetDataField()->SetAsFloat(XCSoarInterface::Calculated().TaskSpeed*TASKSPEEDMODIFY);
    wp->GetDataField()->SetUnits(Units::GetTaskSpeedName());
    wp->RefreshDisplay();
  }

}


static int OnTimerNotify(WindowControl * Sender) {
  (void)Sender;
  GEOPOINT loc;
  // JMW illegal
  if (XCSoarInterface::main_window.map.TargetDragged(&loc.Longitude, &loc.Latitude)) {
    DragTarget(loc);
  }
  if (task.isTargetModified()) {
    RefreshCalculator();
  }
  return 0;
}


static void OnMoveClicked(WindowControl * Sender){
  (void)Sender;
  TargetMoveMode = !TargetMoveMode;
  if (TargetMoveMode) {
    btnMove->SetCaption(_T("Cursor"));
  } else {
    btnMove->SetCaption(_T("Move"));
  }
  RefreshCalculator();
}


static void OnRangeData(DataField *Sender, DataField::DataAccessKind_t Mode) {
  double RangeNew;
  switch(Mode){
    case DataField::daGet:
      //      Sender->Set(Range*100.0);
    break;
    case DataField::daPut:
    case DataField::daChange:
      if (target_point>=task.getActiveIndex()) {
        RangeNew = Sender->GetAsFloat()/100.0;
        if (RangeNew != Range) {
          TASK_POINT tp = task.getTaskPoint(target_point);
          tp.AATTargetOffsetRadius = RangeNew;
          task.setTaskPoint(target_point, tp);
          Range = RangeNew;
          task.SetTargetModified();
        }
      }
    break;
  }
}


static void OnRadialData(DataField *Sender, DataField::DataAccessKind_t Mode) {
  double RadialNew;
  bool updated = false;
  bool dowrap = false;
  switch(Mode){
    case DataField::daGet:
      //      Sender->Set(Range*100.0);
    break;
    case DataField::daPut:
    case DataField::daChange:
      TASK_POINT tp = task.getTaskPoint(target_point);

      if (target_point>=task.getActiveIndex()) {
        if (!XCSoarInterface::Calculated().IsInSector
            || (target_point != task.getActiveIndex())) {
          dowrap = true;
        }
        RadialNew = Sender->GetAsFloat();
        if (fabs(RadialNew)>90) {
          if (dowrap) {
            RadialNew = AngleLimit180(RadialNew+180);
            // flip!
            Range = -Range;
            tp.AATTargetOffsetRadius= -tp.AATTargetOffsetRadius;
            updated = true;
          } else {
            RadialNew = max(-90.0, min(90.0, RadialNew));
            updated = true;
          }
        }
        if (RadialNew != Radial) {
          tp.AATTargetOffsetRadial = RadialNew;
          Radial = RadialNew;
          updated = true;
        }
      }
      if (updated) {
        task.setTaskPoint(target_point, tp);
	task.SetTargetModified();
      }
    break;
  }
}


static void RefreshTargetPoint(void) {
  target_point = max(target_point, task.getActiveIndex());
  if (task.ValidTaskPoint(target_point)) {
    XCSoarInterface::SetSettingsMap().TargetPanIndex = target_point;
    XCSoarInterface::SetSettingsMap().TargetPan = true;
    Range = task.getTaskPoint(target_point).AATTargetOffsetRadius;
    Radial = task.getTaskPoint(target_point).AATTargetOffsetRadial;
  } else {
    Range = 0;
    Radial = 0;
  }
  RefreshCalculator();
}


static void OnLockedData(DataField *Sender, DataField::DataAccessKind_t Mode) {
  switch(Mode){
    case DataField::daGet:
    break;
    case DataField::daPut:
    case DataField::daChange:
      bool lockedthis = Sender->GetAsBoolean();
      if (task.ValidTaskPoint(target_point)) {
        TASK_POINT tp = task.getTaskPoint(target_point);
        if (tp.AATTargetLocked != lockedthis) {
          tp.AATTargetLocked = lockedthis;
          task.setTaskPoint(target_point, tp);
	  task.SetTargetModified();
        }
      }
    break;
  }
}


static void OnTaskPointData(DataField *Sender, DataField::DataAccessKind_t Mode) {
  unsigned old_target_point = target_point;
  switch(Mode){
    case DataField::daGet:
    break;
    case DataField::daPut:
    case DataField::daChange:
      target_point = Sender->GetAsInteger() + ActiveTaskPointOnEntry;
      target_point = max(target_point,task.getActiveIndex());
      if (target_point != old_target_point) {
        RefreshTargetPoint();
      }
    break;
  }
}


static CallBackTableEntry_t CallBackTable[]={
  DeclareCallBackEntry(OnTaskPointData),
  DeclareCallBackEntry(OnRangeData),
  DeclareCallBackEntry(OnRadialData),
  DeclareCallBackEntry(OnLockedData),
  DeclareCallBackEntry(OnOKClicked),
  DeclareCallBackEntry(OnMoveClicked),
  DeclareCallBackEntry(NULL)
};


void dlgTarget(void) {

  if (!task.ValidTaskPoint(task.getActiveIndex())) {
    return;
  }
  ActiveTaskPointOnEntry = task.getActiveIndex();

  if (!Layout::landscape) {
    wf = dlgLoadFromXML(CallBackTable,
                        _T("dlgTarget_L.xml"),
                        XCSoarInterface::main_window,
                        _T("IDR_XML_TARGET_L"));
  } else {
    wf = dlgLoadFromXML(CallBackTable,
                        _T("dlgTarget.xml"),
                        XCSoarInterface::main_window,
                        _T("IDR_XML_TARGET"));
  }

  if (!wf) return;

  targetManipEvent.trigger();
  TargetMoveMode = false;

  if (Layout::landscape)
  {// make flush right in landscape mode (at top in portrait mode)
    WndFrame *wf2 = (WndFrame*)wf->FindByName(_T("frmTarget"));
    if (wf2)
    {
      RECT rc = XCSoarInterface::main_window.get_client_rect();
      wf->move(rc.top, rc.right - wf2->get_size().cx);
    }
  }

  btnMove = (WindowControl*)wf->FindByName(_T("btnMove"));

  wf->SetKeyDownNotify(FormKeyDown);

  WndProperty *wp;
  wp = (WndProperty*)wf->FindByName(_T("prpTaskPoint"));
  DataFieldEnum* dfe;
  dfe = (DataFieldEnum*)wp->GetDataField();
  TCHAR tp_label[80];
  TCHAR tp_short[21];
  if (!task.ValidTaskPoint(target_point)) {
    target_point = ActiveTaskPointOnEntry;
  } else {
    target_point = max(target_point, ActiveTaskPointOnEntry);
  }
  target_point = max(0,min((int)target_point, task.getFinalWaypoint()));

  for (unsigned i=ActiveTaskPointOnEntry; task.ValidTaskPoint(i); i++) {
    _tcsncpy(tp_short, task.getWaypoint(i).Name, 20);
    tp_short[20] = 0;
    _stprintf(tp_label, _T("%d %s"), i, tp_short);
    dfe->addEnumText(tp_label);
  }
  dfe->Set(max(0,(int)target_point-(int)ActiveTaskPointOnEntry));
  wp->RefreshDisplay();

  RefreshTargetPoint();

  wf->SetTimerNotify(OnTimerNotify);

  wf->ShowModal(true); // enable map

  XCSoarInterface::SetSettingsMap().TargetPan = false;

  targetManipEvent.reset();

  delete wf;
  wf = NULL;
}
