/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#include "Profile/UIProfile.hpp"
#include "Profile/MapProfile.hpp"
#include "Profile/InfoBoxConfig.hpp"
#include "Profile/PageProfile.hpp"
#include "Profile/Profile.hpp"
#include "Profile/UnitsConfig.hpp"
#include "UISettings.hpp"

namespace Profile {
  static void Load(VarioSettings &settings);
  static void Load(DialogSettings &settings);
};

void
Profile::Load(VarioSettings &settings)
{
  Get(szProfileAppGaugeVarioSpeedToFly, settings.ShowSpeedToFly);
  Get(szProfileAppGaugeVarioAvgText, settings.ShowAvgText);
  Get(szProfileAppGaugeVarioMc, settings.ShowMc);
  Get(szProfileAppGaugeVarioBugs, settings.ShowBugs);
  Get(szProfileAppGaugeVarioBallast, settings.ShowBallast);
  Get(szProfileAppGaugeVarioGross, settings.ShowGross);
  Get(szProfileAppAveNeedle, settings.ShowAveNeedle);
}

void
Profile::Load(DialogSettings &settings)
{
  GetEnum(szProfileAppTextInputStyle, settings.text_input_style);
  GetEnum(szProfileAppDialogTabStyle, settings.tab_style);
}

void
Profile::Load(UISettings &settings)
{
  Get(szProfileMenuTimeout, settings.menu_timeout);

  Get(szProfileUseCustomFonts, settings.custom_fonts);
  Get(szProfileAutoBlank, settings.enable_auto_blank);

  Get(szProfileEnableFLARMGauge, settings.enable_flarm_gauge);
  Get(szProfileAutoCloseFlarmDialog, settings.auto_close_flarm_dialog);
  Get(szProfileEnableTAGauge, settings.enable_thermal_assistant_gauge);

  GetEnum(szProfileAppStatusMessageAlignment, settings.popup_message_position);
  GetEnum(szProfileFlarmLocation, settings.flarm_location);

  GetEnum(szProfileHapticFeedback, settings.haptic_feedback);

  GetEnum(szProfileLatLonUnits, settings.coordinate_format);

  LoadUnits(settings.units);
  Load(settings.map);
  Load(settings.info_boxes);
  Load(settings.vario);
  Load(settings.pages);
  Load(settings.dialog);
}
