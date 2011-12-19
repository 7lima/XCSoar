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

#include "InfoBoxes/Content/Task.hpp"

#include "InfoBoxes/InfoBoxWindow.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Dialogs/Waypoint.hpp"
#include "Dialogs/dlgAnalysis.hpp"
#include "MainWindow.hpp"
#include "LocalTime.hpp"
#include "Engine/Util/Gradient.hpp"
#include "Units/UnitsFormatter.hpp"

#include <tchar.h>
#include <stdio.h>

void
InfoBoxContentBearing::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!task_stats.task_valid || !vector_remaining.IsValid() ||
      vector_remaining.Distance <= fixed(10)) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  infobox.SetValue(vector_remaining.Bearing, _T("T"));
}

void
InfoBoxContentBearingDiff::Update(InfoBoxWindow &infobox)
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!basic.track_available || !task_stats.task_valid ||
      !vector_remaining.IsValid() || vector_remaining.Distance <= fixed(10)) {
    infobox.SetInvalid();
    return;
  }

  Angle Value = vector_remaining.Bearing - basic.track;
  SetValueBearingDifference(infobox, Value);
}

void
InfoBoxContentNextWaypoint::Update(InfoBoxWindow &infobox)
{
  // use proper non-terminal next task stats

  const Waypoint* way_point = protected_task_manager != NULL
    ? protected_task_manager->getActiveWaypoint()
    : NULL;

  if (!way_point) {
    infobox.SetTitle(_("Next"));
    infobox.SetInvalid();
    return;
  }
  SetTitleFromWaypointName(infobox, way_point);

  // Set Comment
  if (way_point->radio_frequency.IsDefined()) {
    StaticString<128> comment;
    const unsigned freq = way_point->radio_frequency.GetKiloHertz();
    _sntprintf(comment.buffer(), comment.MAX_SIZE, _T("%u.%03u %s"),
               freq / 1000, freq % 1000, way_point->comment.c_str());
    infobox.SetComment(comment);
  }
  else
    infobox.SetComment(way_point->comment.c_str());

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GlideResult &solution_remaining =
    task_stats.current_leg.solution_remaining;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!basic.track_available || !task_stats.task_valid ||
      !vector_remaining.IsValid()) {
    infobox.SetValueInvalid();
    return;
  }

  // Set Value
  Angle Value = vector_remaining.Bearing - basic.track;
  SetValueBearingDifference(infobox, Value);

  // Set Color (blue/black)
  infobox.SetColor(solution_remaining.IsFinalGlide() ? 2 : 0);
}

bool
InfoBoxContentNextWaypoint::HandleKey(const InfoBoxKeyCodes keycode)
{
  if (protected_task_manager == NULL)
    return false;

  switch (keycode) {
  case ibkRight:
  case ibkUp:
    protected_task_manager->incrementActiveTaskPoint(1);
    return true;

  case ibkLeft:
  case ibkDown:
    protected_task_manager->incrementActiveTaskPoint(-1);
    return true;

  case ibkEnter:
    const Waypoint *wp = protected_task_manager->getActiveWaypoint();
    if (wp) {
      dlgWaypointDetailsShowModal(XCSoarInterface::main_window, *wp);
      return true;
    }
  }

  return false;
}

void
InfoBoxContentNextDistance::Update(InfoBoxWindow &infobox)
{
  // use proper non-terminal next task stats

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!task_stats.task_valid || !vector_remaining.IsValid()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromDistance(infobox, vector_remaining.Distance);

  if (basic.track_available) {
    Angle bd = vector_remaining.Bearing - basic.track;
    SetCommentBearingDifference(infobox, bd);
  } else
    infobox.SetCommentInvalid();
}

void
InfoBoxContentNextETE::Update(InfoBoxWindow &infobox)
{
  // use proper non-terminal next task stats

  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.current_leg.IsAchievable()) {
    infobox.SetInvalid();
    return;
  }

  assert(!negative(task_stats.current_leg.time_remaining));

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  const int dd = (int)task_stats.current_leg.time_remaining;
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  infobox.SetValue(HHMMSSsmart);
  infobox.SetComment(SSsmart);
}

void
InfoBoxContentNextETA::Update(InfoBoxWindow &infobox)
{
  // use proper non-terminal next task stats

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.current_leg.IsAchievable()) {
    infobox.SetInvalid();
    return;
  }

  TCHAR tmp[32];
  int dd = (int)(task_stats.current_leg.solution_remaining.time_elapsed) +
    DetectCurrentTime(basic);
  const BrokenTime t = BrokenTime::FromSecondOfDayChecked(abs(dd));

  // Set Value
  _stprintf(tmp, _T("%02u:%02u"), t.hour, t.minute);
  infobox.SetValue(tmp);

  // Set Comment
  _stprintf(tmp, _T("%02u"), t.second);
  infobox.SetComment(tmp);
}

void
InfoBoxContentNextAltitudeDiff::Update(InfoBoxWindow &infobox)
{
  // pilots want this to be assuming terminal flight to this wp

  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const GlideResult &next_solution = XCSoarInterface::Calculated().common_stats.next_solution;
  if (!task_stats.task_valid || !next_solution.IsAchievable()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  TCHAR tmp[32];
  Units::FormatUserAltitude(next_solution.altitude_difference, tmp, 32, false);
  infobox.SetValue(tmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentNextAltitudeRequire::Update(InfoBoxWindow &infobox)
{
  // pilots want this to be assuming terminal flight to this wp

  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const GlideResult &next_solution = XCSoarInterface::Calculated().common_stats.next_solution;
  if (!task_stats.task_valid || !next_solution.IsAchievable()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  TCHAR tmp[32];
  Units::FormatUserAltitude(next_solution.altitude_required, tmp, 32, false);
  infobox.SetValue(tmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentNextAltitudeArrival::Update(InfoBoxWindow &infobox)
{
  // pilots want this to be assuming terminal flight to this wp

  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const GlideResult next_solution = XCSoarInterface::Calculated().common_stats.next_solution;
  if (!task_stats.task_valid || !next_solution.IsFinalGlide()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  TCHAR tmp[32];
  fixed alt = next_solution.GetArrivalAltitude(XCSoarInterface::Basic().NavAltitude);
  Units::FormatUserAltitude(alt, tmp, 32, false);
  infobox.SetValue(tmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}


void
InfoBoxContentNextLD::Update(InfoBoxWindow &infobox)
{
  // pilots want this to be assuming terminal flight to this wp, and this
  // is what current_leg gradient does.

  if (!XCSoarInterface::Calculated().task_stats.task_valid) {
    infobox.SetInvalid();
    return;
  }

  fixed gradient = XCSoarInterface::Calculated().task_stats.current_leg.gradient;

  if (!positive(gradient)) {
    infobox.SetValue(_T("+++"));
    return;
  }
  if (::GradientValid(gradient)) {
    TCHAR tmp[32];
    _stprintf(tmp, _T("%d"), (int)gradient);
    infobox.SetValue(tmp);
  } else {
    infobox.SetInvalid();
  }
}

void
InfoBoxContentFinalDistance::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;

  if (!task_stats.task_valid ||
      !task_stats.current_leg.vector_remaining.IsValid() ||
      !task_stats.total.remaining.IsDefined()) {
    infobox.SetInvalid();
    return;
  }

  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  // Set Value
  SetValueFromDistance(infobox, common_stats.task_finished
                       ? task_stats.current_leg.vector_remaining.Distance
                       : task_stats.total.remaining.get_distance());
}

void
InfoBoxContentFinalETE::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;

  if (!task_stats.task_valid || !task_stats.total.IsAchievable()) {
    infobox.SetInvalid();
    return;
  }

  assert(!negative(task_stats.total.time_remaining));

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  const int dd = abs((int)task_stats.total.time_remaining);
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  infobox.SetValue(HHMMSSsmart);
  infobox.SetComment(SSsmart);
}

void
InfoBoxContentFinalETA::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.total.IsAchievable()) {
    infobox.SetInvalid();
    return;
  }

  TCHAR tmp[32];
  int dd = (int)task_stats.total.solution_remaining.time_elapsed +
    DetectCurrentTime(XCSoarInterface::Basic());
  const BrokenTime t = BrokenTime::FromSecondOfDayChecked(abs(dd));

  // Set Value
  _stprintf(tmp, _T("%02u:%02u"), t.hour, t.minute);
  infobox.SetValue(tmp);

  // Set Comment
  _stprintf(tmp, _T("%02u"), t.second);
  infobox.SetComment(tmp);
}

void
InfoBoxContentFinalAltitudeDiff::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid ||
      !task_stats.total.solution_remaining.IsOk()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  TCHAR tmp[32];
  Units::FormatUserAltitude(task_stats.total.solution_remaining.altitude_difference,
                            tmp, 32, false);
  infobox.SetValue(tmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentFinalAltitudeRequire::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid ||
      !task_stats.total.solution_remaining.IsOk()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  TCHAR tmp[32];
  Units::FormatUserAltitude(task_stats.total.solution_remaining.altitude_required,
                            tmp, 32, false);
  infobox.SetValue(tmp);

  // Set Unit
  infobox.SetValueUnit(Units::Current.AltitudeUnit);
}

void
InfoBoxContentTaskSpeed::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.total.travelled.IsDefined()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromFixed(infobox, _T("%2.0f"),
                    Units::ToUserTaskSpeed(task_stats.total.travelled.get_speed()));

  // Set Unit
  infobox.SetValueUnit(Units::Current.TaskSpeedUnit);
}

void
InfoBoxContentTaskSpeedAchieved::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid ||
      !task_stats.total.remaining_effective.IsDefined()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromFixed(infobox, _T("%2.0f"),
                    Units::ToUserTaskSpeed(task_stats.total.remaining_effective.get_speed()));

  // Set Unit
  infobox.SetValueUnit(Units::Current.TaskSpeedUnit);
}

void
InfoBoxContentTaskSpeedInstant::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.IsPirkerSpeedAvailable()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromFixed(infobox, _T("%2.0f"),
                    Units::ToUserTaskSpeed(task_stats.get_pirker_speed()));

  // Set Unit
  infobox.SetValueUnit(Units::Current.TaskSpeedUnit);
}

void
InfoBoxContentFinalLD::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid) {
    infobox.SetInvalid();
    return;
  }

  fixed gradient = task_stats.total.gradient;

  if (!positive(gradient)) {
    infobox.SetValue(_T("+++"));
    return;
  }
  if (::GradientValid(gradient)) {
    TCHAR tmp[32];
    _stprintf(tmp, _T("%d"), (int)gradient);
    infobox.SetValue(tmp);
  } else {
    infobox.SetInvalid();
  }
}

void
InfoBoxContentFinalGR::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  if (!task_stats.task_valid) {
    infobox.SetInvalid();
    return;
  }

  fixed gradient = task_stats.total.gradient;

  if (!positive(gradient)) {
    infobox.SetValue(_T("+++"));
    return;
  }
  if (::GradientValid(gradient)) {
    TCHAR tmp[32];
    _stprintf(tmp, _T("%d"), (int)gradient);
    infobox.SetValue(tmp);
  } else {
    infobox.SetInvalid();
  }
}

void
InfoBoxContentHomeDistance::Update(InfoBoxWindow &infobox)
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  // Set Value
  SetValueFromDistance(infobox, common_stats.vector_home.Distance);

  if (basic.track_available) {
    Angle bd = common_stats.vector_home.Bearing - basic.track;
    SetCommentBearingDifference(infobox, bd);
  } else
    infobox.SetCommentInvalid();
}

void
InfoBoxContentOLC::Update(InfoBoxWindow &infobox)
{
  if (!XCSoarInterface::SettingsComputer().task.enable_olc ||
      !protected_task_manager) {
    infobox.SetInvalid();
    return;
  }

  const ContestResult& result_olc = 
    XCSoarInterface::Calculated().contest_stats.get_contest_result();

  if (result_olc.score < fixed_one) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromDistance(infobox, result_olc.distance);

  TCHAR tmp[32];
  _stprintf(tmp, _T("%.1f pts"), (double)result_olc.score);
  infobox.SetComment(tmp);
}

bool
InfoBoxContentOLC::HandleKey(const InfoBoxKeyCodes keycode)
{
  switch (keycode) {
  case ibkEnter:
    dlgAnalysisShowModal(XCSoarInterface::main_window,
                         *CommonInterface::main_window.look,
                         CommonInterface::Full(), *glide_computer,
                         protected_task_manager, &airspace_database, terrain,
                         8);
    return true;

  default:
    break;
  }

  return false;
}

void
InfoBoxContentTaskAATime::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid || !task_stats.total.IsAchievable() ||
      !positive(common_stats.aat_time_remaining)) {
    infobox.SetInvalid();
    return;
  }

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  const int dd = abs((int)common_stats.aat_time_remaining);
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  infobox.SetValue(HHMMSSsmart);
  infobox.SetComment(SSsmart);
}

void
InfoBoxContentTaskAATimeDelta::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid || !task_stats.total.IsAchievable() ||
      !positive(common_stats.aat_time_remaining)) {
    infobox.SetInvalid();
    return;
  }

  assert(!negative(task_stats.total.time_remaining));

  fixed diff = task_stats.total.time_remaining -
    common_stats.aat_time_remaining;

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  const int dd = abs((int)diff);
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  TCHAR tmp[32];
  _stprintf(tmp, negative(diff) ? _T("-%s") : _T("%s"), HHMMSSsmart);
  infobox.SetValue(tmp);

  infobox.SetComment(SSsmart);

  // Set Color (red/blue/black)
  infobox.SetColor(negative(diff) ? 1 :
                   task_stats.total.time_remaining >
                       common_stats.aat_time_remaining + fixed(5*60) ? 2 : 0);
}

void
InfoBoxContentTaskAADistance::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid ||
      !task_stats.total.planned.IsDefined()) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromDistance(infobox, task_stats.total.planned.get_distance());
}

void
InfoBoxContentTaskAADistanceMax::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromDistance(infobox, task_stats.distance_max);
}

void
InfoBoxContentTaskAADistanceMin::Update(InfoBoxWindow &infobox)
{
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromDistance(infobox, task_stats.distance_min);
}

void
InfoBoxContentTaskAASpeed::Update(InfoBoxWindow &infobox)
{
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid || !positive(common_stats.aat_speed_remaining)) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromFixed(infobox, _T("%2.0f"),
                    Units::ToUserTaskSpeed(common_stats.aat_speed_remaining));

  // Set Unit
  infobox.SetValueUnit(Units::Current.TaskSpeedUnit);
}

void
InfoBoxContentTaskAASpeedMax::Update(InfoBoxWindow &infobox)
{
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid || !positive(common_stats.aat_speed_max)) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromFixed(infobox, _T("%2.0f"),
                    Units::ToUserTaskSpeed(common_stats.aat_speed_max));

  // Set Unit
  infobox.SetValueUnit(Units::Current.TaskSpeedUnit);
}

void
InfoBoxContentTaskAASpeedMin::Update(InfoBoxWindow &infobox)
{
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;

  if (!common_stats.ordered_has_targets ||
      !task_stats.task_valid || !positive(common_stats.aat_speed_min)) {
    infobox.SetInvalid();
    return;
  }

  // Set Value
  SetValueFromFixed(infobox, _T("%2.0f"),
                    Units::ToUserTaskSpeed(common_stats.aat_speed_min));

  // Set Unit
  infobox.SetValueUnit(Units::Current.TaskSpeedUnit);
}

void
InfoBoxContentTaskTimeUnderMaxHeight::Update(InfoBoxWindow &infobox)
{
  const CommonStats &common_stats = XCSoarInterface::Calculated().common_stats;
  const TaskStats &task_stats = XCSoarInterface::Calculated().task_stats;
  const fixed maxheight = fixed(protected_task_manager->
                                get_ordered_task_behaviour().start_max_height);

  if (!task_stats.task_valid || !positive(maxheight)
      || !protected_task_manager
      || !positive(common_stats.TimeUnderStartMaxHeight)) {
    infobox.SetInvalid();
    return;
  }

  const int dd = (int)(XCSoarInterface::Basic().time -
      common_stats.TimeUnderStartMaxHeight);

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  infobox.SetValue(HHMMSSsmart);
  infobox.SetComment(_("Time Below"));
}

void
InfoBoxContentNextETEVMG::Update(InfoBoxWindow &infobox)
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!basic.ground_speed_available || task_stats.task_valid ||
      !task_stats.current_leg.remaining.IsDefined()) {
    infobox.SetInvalid();
    return;
  }

  const fixed d = task_stats.current_leg.remaining.get_distance();
  const fixed v = basic.ground_speed;

  if (!task_stats.task_valid ||
      !positive(d) ||
      !positive(v)) {
    infobox.SetInvalid();
    return;
  }

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  const int dd = (int)(d/v);
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  infobox.SetValue(HHMMSSsmart);
  infobox.SetComment(SSsmart);
}

void
InfoBoxContentFinalETEVMG::Update(InfoBoxWindow &infobox)
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!basic.ground_speed_available || task_stats.task_valid ||
      task_stats.total.remaining.IsDefined()) {
    infobox.SetInvalid();
    return;
  }

  const fixed d = task_stats.total.remaining.get_distance();
  const fixed &v = basic.ground_speed;

  if (!task_stats.task_valid ||
      !positive(d) ||
      !positive(v)) {
    infobox.SetInvalid();
    return;
  }

  TCHAR HHMMSSsmart[32];
  TCHAR SSsmart[32];
  const int dd = (int)(d/v);
  Units::TimeToTextSmart(HHMMSSsmart, SSsmart, dd);

  infobox.SetValue(HHMMSSsmart);
  infobox.SetComment(SSsmart);
}
