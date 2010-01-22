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

#include "ButtonLabel.hpp"
#include "Language.hpp"
#include "Gauge/GaugeFLARM.hpp"
#include "Logger.h"
#include "MainWindow.hpp"
#include "Interface.hpp"
#include "SettingsComputer.hpp"
#include "SettingsTask.hpp"
#include "Components.hpp"
#include "Compatibility/string.h"
#include "InfoBoxManager.h"
#include "SettingsUser.hpp"
#include "Asset.hpp"
#include "Waypoint/Waypoints.hpp"
#include "Airspace/Airspaces.hpp"
#include "Task/TaskManager.hpp"

#include <stdlib.h>

/**
 * Replaces ToReplace with ReplaceWith in String
 * @param String Buffer string
 * @param ToReplace The string that will be replaced
 * @param ReplaceWith The replacement
 * @param Size (?)
 */
static void ReplaceInString(TCHAR *String, const TCHAR *ToReplace,
                            const TCHAR *ReplaceWith, size_t Size){
  TCHAR TmpBuf[MAX_PATH];
  size_t iR = _tcslen(ToReplace);
  TCHAR *pC;

  while((pC = _tcsstr(String, ToReplace)) != NULL){
    _tcscpy(TmpBuf, pC + iR);
    _tcscpy(pC, ReplaceWith);
    _tcscat(pC, TmpBuf);
  }

}

/**
 * If Condition is true, Macro in Buffer will be replaced by TrueText,
 * otherwise by FalseText.
 * @param Condition Condition to be checked
 * @param Buffer Buffer string
 * @param Macro The string that will be replaced
 * @param TrueText The replacement if Condition is true
 * @param FalseText The replacement if Condition is false
 * @param Size (?)
 */
static void CondReplaceInString(bool Condition, TCHAR *Buffer,
                                const TCHAR *Macro, const TCHAR *TrueText,
                                const TCHAR *FalseText, size_t Size){
  if (Condition)
    ReplaceInString(Buffer, Macro, TrueText, Size);
  else
    ReplaceInString(Buffer, Macro, FalseText, Size);
}

bool ButtonLabel::ExpandMacros(const TCHAR *In,
			       TCHAR *OutBuffer, size_t Size) {
  // ToDo, check Buffer Size
  bool invalid = false;
  _tcsncpy(OutBuffer, In, Size);
  OutBuffer[Size-1] = '\0';

  if (_tcsstr(OutBuffer, TEXT("$(")) == NULL) return false;

  if (_tcsstr(OutBuffer, TEXT("$(CheckAirspace)"))) {
    if (airspace_database.empty())
      invalid = true;

    ReplaceInString(OutBuffer, TEXT("$(CheckAirspace)"), TEXT(""), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(CheckTaskResumed)"))) {
    // TODO code: check, does this need to be set with temporary task?
    invalid |= Calculated().common_stats.mode_abort;
    invalid |= Calculated().common_stats.mode_goto;
    ReplaceInString(OutBuffer, TEXT("$(CheckTaskResumed)"), TEXT(""), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(CheckTask)"))) {
    if (!Calculated().task_stats.task_valid) {
      invalid = true;
    }
    ReplaceInString(OutBuffer, TEXT("$(CheckTask)"), TEXT(""), Size);
  }

  if (!Calculated().task_stats.task_valid || Calculated().common_stats.mode_goto) {

    if (_tcsstr(OutBuffer, TEXT("$(WaypointNext)"))) {
      invalid = true;
      ReplaceInString(OutBuffer, TEXT("$(WaypointNext)"), TEXT("Waypoint\nNext"), Size);

    } else if (_tcsstr(OutBuffer, TEXT("$(WaypointPrevious)"))) {
      invalid = true;
      ReplaceInString(OutBuffer, TEXT("$(WaypointPrevious)"), TEXT("Waypoint\nPrevious"), Size);
    }

  } else if (Calculated().common_stats.mode_abort) {

    if (_tcsstr(OutBuffer, TEXT("$(WaypointNext)"))) {
      // Waypoint\nNext
      invalid |= !Calculated().common_stats.active_has_next;
      CondReplaceInString(Calculated().common_stats.next_is_last,
                          OutBuffer,
                          TEXT("$(WaypointNext)"),
                          TEXT("Landpoint\nFurthest"),
                          TEXT("Landpoint\nNext"), Size);

    } else if (_tcsstr(OutBuffer, TEXT("$(WaypointPrevious)"))) {
      // Waypoint\nNext
      invalid |= !Calculated().common_stats.active_has_previous;
      CondReplaceInString(Calculated().common_stats.previous_is_first,
                          OutBuffer,
                          TEXT("$(WaypointPrevious)"),
                          TEXT("Landpoint\nClosest"),
                          TEXT("Landpoint\nPrevious"), Size);
    }

  } else {
    if (_tcsstr(OutBuffer, TEXT("$(WaypointNext)"))) {
      // Waypoint\nNext
      invalid |= !Calculated().common_stats.active_has_next;
      CondReplaceInString(Calculated().common_stats.next_is_last,
                          OutBuffer,
                          TEXT("$(WaypointNext)"),
                          TEXT("Waypoint\nFinish"),
                          TEXT("Waypoint\nNext"), Size);

    } else if (_tcsstr(OutBuffer, TEXT("$(WaypointPrevious)"))) {
      invalid |= !Calculated().common_stats.active_has_previous;
      CondReplaceInString(Calculated().common_stats.previous_is_first,
                          OutBuffer,
                          TEXT("$(WaypointPrevious)"),
                          TEXT("Waypoint\nStart"),
                          TEXT("Waypoint\nPrevious"), Size);
    } 
#ifdef OLD_TASK
    else if (task.getSettings().EnableMultipleStartPoints) {
      invalid |= !task.ValidTaskPoint(0);
      CondReplaceInString((task.getActiveIndex()==0),
                          OutBuffer,
                          TEXT("$(WaypointPrevious)"),
                          TEXT("StartPoint\nCycle"), TEXT("Waypoint\nPrevious"), Size);
    } 
    else {
      invalid |= !Calculated().common_stats.active_has_previous;
      ReplaceInString(OutBuffer, TEXT("$(WaypointPrevious)"), TEXT("Waypoint\nPrevious"), Size);
    }
#endif
  }

#ifdef OLD_TASK
  if (_tcsstr(OutBuffer, TEXT("$(AdvanceArmed)"))) {
    switch (task.getSettings().AutoAdvance) {
    case 0:
      ReplaceInString(OutBuffer, TEXT("$(AdvanceArmed)"), TEXT("(manual)"), Size);
      invalid = true;
      break;
    case 1:
      ReplaceInString(OutBuffer, TEXT("$(AdvanceArmed)"), TEXT("(auto)"), Size);
      invalid = true;
      break;
    case 2:
      if (task.getActiveIndex()>0) {
        if (task.ValidTaskPoint(task.getActiveIndex()+1)) {
          CondReplaceInString(task.isAdvanceArmed(), OutBuffer, TEXT("$(AdvanceArmed)"),
                              TEXT("Cancel"), TEXT("TURN"), Size);
        } else {
          ReplaceInString(OutBuffer, TEXT("$(AdvanceArmed)"),
                          TEXT("(finish)"), Size);
          invalid = true;
        }
      } else {
        CondReplaceInString(task.isAdvanceArmed(), OutBuffer, TEXT("$(AdvanceArmed)"),
                            TEXT("Cancel"), TEXT("START"), Size);
      }
      break;
    case 3:
      if (task.getActiveIndex()==0) {
        CondReplaceInString(task.isAdvanceArmed(), OutBuffer, TEXT("$(AdvanceArmed)"),
                            TEXT("Cancel"), TEXT("START"), Size);
      } else if (task.getActiveIndex()==1) {
        CondReplaceInString(task.isAdvanceArmed(), OutBuffer, TEXT("$(AdvanceArmed)"),
                            TEXT("Cancel"), TEXT("RESTART"), Size);
      } else {
        ReplaceInString(OutBuffer, TEXT("$(AdvanceArmed)"), TEXT("(auto)"), Size);
        invalid = true;
      }
      // TODO bug: no need to arm finish
    default:
      break;
    }
  }
#endif

  if (_tcsstr(OutBuffer, TEXT("$(CheckAutoMc)"))) {
    if (!Calculated().task_stats.task_valid
        && ((SettingsComputer().AutoMacCreadyMode==0)
	    || (SettingsComputer().AutoMacCreadyMode==2))) {
      invalid = true;
    }
    ReplaceInString(OutBuffer, TEXT("$(CheckAutoMc)"), TEXT(""), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(TaskAbortToggleActionName)"))) {
    if (Calculated().common_stats.mode_goto) {
      CondReplaceInString(Calculated().common_stats.ordered_valid,
                          OutBuffer, TEXT("$(TaskAbortToggleActionName)"),
                          TEXT("Resume"), TEXT("Abort"), Size);
    } else 
      CondReplaceInString(Calculated().common_stats.mode_abort,
                          OutBuffer, TEXT("$(TaskAbortToggleActionName)"),
                          TEXT("Resume"), TEXT("Abort"), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(CheckReplay)"))) {
    if (!Basic().Replay && Basic().MovementDetected) {
      invalid = true;
    }
    ReplaceInString(OutBuffer, TEXT("$(CheckReplay)"), TEXT(""), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(CheckWaypointFile)"))) {
    invalid |= way_points.empty();
    ReplaceInString(OutBuffer, TEXT("$(CheckWaypointFile)"), TEXT(""), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(CheckSettingsLockout)"))) {
    if (!is_simulator() && XCSoarInterface::LockSettingsInFlight &&
        Basic().Flying) {
      invalid = true;
    }
    ReplaceInString(OutBuffer, TEXT("$(CheckSettingsLockout)"), TEXT(""), Size);
  }

  if (_tcsstr(OutBuffer, TEXT("$(CheckFLARM)"))) {
    if (!Basic().flarm.FLARM_Available) {
      invalid = true;
    }
    ReplaceInString(OutBuffer, TEXT("$(CheckFLARM)"), TEXT(""), Size);
  }
  if (_tcsstr(OutBuffer, TEXT("$(CheckTerrain)"))) {
    if (!Calculated().TerrainValid) {
      invalid = true;
    }
    ReplaceInString(OutBuffer, TEXT("$(CheckTerrain)"), TEXT(""), Size);
  }

  CondReplaceInString(logger.isLoggerActive(), OutBuffer,
                      TEXT("$(LoggerActive)"), TEXT("Stop"), TEXT("Start"), Size);

  if (_tcsstr(OutBuffer, TEXT("$(SnailTrailToggleName)"))) {
    switch(SettingsMap().TrailActive) {
    case 0:
      ReplaceInString(OutBuffer, TEXT("$(SnailTrailToggleName)"), TEXT("Long"), Size);
      break;
    case 1:
      ReplaceInString(OutBuffer, TEXT("$(SnailTrailToggleName)"), TEXT("Short"), Size);
      break;
    case 2:
      ReplaceInString(OutBuffer, TEXT("$(SnailTrailToggleName)"), TEXT("Full"), Size);
      break;
    case 3:
      ReplaceInString(OutBuffer, TEXT("$(SnailTrailToggleName)"), TEXT("Off"), Size);
      break;
    }
  }
// VENTA3 VisualGlide
  if (_tcsstr(OutBuffer, TEXT("$(VisualGlideToggleName)"))) {
    switch(SettingsMap().VisualGlide) {
    case 0:
      ReplaceInString(OutBuffer, TEXT("$(VisualGlideToggleName)"), TEXT("Steady"), Size);
      break;
    case 1:
	if (SettingsMap().ExtendedVisualGlide)
		ReplaceInString(OutBuffer, TEXT("$(VisualGlideToggleName)"), TEXT("Moving"), Size);
	else
		ReplaceInString(OutBuffer, TEXT("$(VisualGlideToggleName)"), TEXT("Off"), Size);
      break;
    case 2:
      ReplaceInString(OutBuffer, TEXT("$(VisualGlideToggleName)"), TEXT("Off"), Size);
      break;
    }
  }

// VENTA3 AirSpace event
  if (_tcsstr(OutBuffer, TEXT("$(AirSpaceToggleName)"))) {
    switch(SettingsMap().OnAirSpace) {
    case 0:
      ReplaceInString(OutBuffer, TEXT("$(AirSpaceToggleName)"), TEXT("ON"), Size);
      break;
    case 1:
      ReplaceInString(OutBuffer, TEXT("$(AirSpaceToggleName)"), TEXT("OFF"), Size);
      break;
    }
  }

  if (_tcsstr(OutBuffer, TEXT("$(TerrainTopologyToggleName)"))) {
    char val;
    val = 0;
    if (SettingsMap().EnableTopology) val++;
    if (SettingsMap().EnableTerrain) val += (char)2;
    switch(val) {
    case 0:
      ReplaceInString(OutBuffer, TEXT("$(TerrainTopologyToggleName)"),
                      TEXT("Topology\nOn"), Size);
      break;
    case 1:
      ReplaceInString(OutBuffer, TEXT("$(TerrainTopologyToggleName)"),
                      TEXT("Terrain\nOn"), Size);
      break;
    case 2:
      ReplaceInString(OutBuffer, TEXT("$(TerrainTopologyToggleName)"),
                      TEXT("Terrain\nTopology"), Size);
      break;
    case 3:
      ReplaceInString(OutBuffer, TEXT("$(TerrainTopologyToggleName)"),
                      gettext(TEXT("Terrain\nOff")), Size);
      break;
    }
  }

  CondReplaceInString(SettingsMap().FullScreen, OutBuffer,
                      TEXT("$(FullScreenToggleActionName)"),
                      TEXT("Off"), TEXT("On"), Size);
  CondReplaceInString(SettingsMap().AutoZoom, OutBuffer,
		      TEXT("$(ZoomAutoToggleActionName)"),
		      TEXT("Manual"), TEXT("Auto"), Size);
  CondReplaceInString(SettingsMap().EnableTopology, OutBuffer, TEXT("$(TopologyToggleActionName)"), TEXT("Off"), TEXT("On"), Size);
  CondReplaceInString(SettingsMap().EnableTerrain, OutBuffer, TEXT("$(TerrainToggleActionName)"), TEXT("Off"), TEXT("On"), Size);

  if (_tcsstr(OutBuffer, TEXT("$(MapLabelsToggleActionName)"))) {
    switch(SettingsMap().DeclutterLabels) {
    case 0:
      ReplaceInString(OutBuffer, TEXT("$(MapLabelsToggleActionName)"),
                      TEXT("MID"), Size);
      break;
    case 1:
      ReplaceInString(OutBuffer, TEXT("$(MapLabelsToggleActionName)"),
                      TEXT("OFF"), Size);
      break;
    case 2:
      ReplaceInString(OutBuffer, TEXT("$(MapLabelsToggleActionName)"),
                      TEXT("ON"), Size);
      break;
    }
  }

  CondReplaceInString(SettingsComputer().auto_mc, OutBuffer, TEXT("$(MacCreadyToggleActionName)"), TEXT("Manual"), TEXT("Auto"), Size);
  CondReplaceInString(SettingsMap().EnableAuxiliaryInfo, OutBuffer, TEXT("$(AuxInfoToggleActionName)"), TEXT("Off"), TEXT("On"), Size);

  CondReplaceInString(SettingsMap().UserForceDisplayMode == dmCircling, OutBuffer, TEXT("$(DispModeClimbShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().UserForceDisplayMode == dmCruise, OutBuffer, TEXT("$(DispModeCruiseShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().UserForceDisplayMode == dmNone, OutBuffer, TEXT("$(DispModeAutoShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().UserForceDisplayMode == dmFinalGlide, OutBuffer, TEXT("$(DispModeFinalShortIndicator)"), TEXT("(*)"), TEXT(""), Size);

  CondReplaceInString(SettingsComputer().AltitudeMode == ALLON, OutBuffer, TEXT("$(AirspaceModeAllShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsComputer().AltitudeMode == CLIP,  OutBuffer, TEXT("$(AirspaceModeClipShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsComputer().AltitudeMode == AUTO,  OutBuffer, TEXT("$(AirspaceModeAutoShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsComputer().AltitudeMode == ALLBELOW, OutBuffer, TEXT("$(AirspaceModeBelowShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsComputer().AltitudeMode == ALLOFF, OutBuffer, TEXT("$(AirspaceModeAllOffIndicator)"), TEXT("(*)"), TEXT(""), Size);

  CondReplaceInString(SettingsMap().TrailActive == 0, OutBuffer, TEXT("$(SnailTrailOffShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().TrailActive == 2, OutBuffer, TEXT("$(SnailTrailShortShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().TrailActive == 1, OutBuffer, TEXT("$(SnailTrailLongShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().TrailActive == 3, OutBuffer, TEXT("$(SnailTrailFullShortIndicator)"), TEXT("(*)"), TEXT(""), Size);

// VENTA3 VisualGlide
  CondReplaceInString(SettingsMap().VisualGlide == 0, OutBuffer, TEXT("$(VisualGlideOffShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().VisualGlide == 1, OutBuffer, TEXT("$(VisualGlideLightShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().VisualGlide == 2, OutBuffer, TEXT("$(VisualGlideHeavyShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
// VENTA3 AirSpace
  CondReplaceInString(SettingsMap().OnAirSpace  == 0, OutBuffer, TEXT("$(AirSpaceOffShortIndicator)"), TEXT("(*)"), TEXT(""), Size);
  CondReplaceInString(SettingsMap().OnAirSpace  == 1, OutBuffer, TEXT("$(AirSpaceOnShortIndicator)"), TEXT("(*)"), TEXT(""), Size);

  CondReplaceInString(SettingsMap().EnableFLARMGauge != 0, OutBuffer, TEXT("$(FlarmDispToggleActionName)"), TEXT("Off"), TEXT("On"), Size);

  return invalid;
}
