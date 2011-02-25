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

#ifndef XCSOAR_INFOBOX_CONTENT_ALTITUDE_HPP
#define XCSOAR_INFOBOX_CONTENT_ALTITUDE_HPP

#include "InfoBoxes/Content/Base.hpp"
#include "Form/TabBar.hpp"
#include "Form/Button.hpp"

class InfoBoxContentAltitude : public InfoBoxContent
{
private:
  static const int PANELSIZE = 2;

public:
  virtual InfoBoxDlgContent* GetInfoBoxDlgContent();

  static Window* PnlInfoLoad(SingleWindow &parent, TabBarControl* wTabBar, WndForm* wf, const int id);
  static bool PnlInfoOnTabPreShow(TabBarControl::EventType EventType);

  static CallBackTableEntry CallBackTable[];
  static InfoBoxPanelContent Panels[];

  static InfoBoxDlgContent dlgContent;

  static Window* PnlSetupLoad(SingleWindow &parent, TabBarControl* wTabBar, WndForm* wf, const int id);
  static void PnlSetupOnSetup(WndButton &Sender);
};

class InfoBoxContentAltitudeGPS : public InfoBoxContentAltitude
{
public:
  virtual void Update(InfoBoxWindow &infobox);
  virtual bool HandleKey(const InfoBoxKeyCodes keycode);
};

class InfoBoxContentAltitudeAGL : public InfoBoxContentAltitude
{
public:
  virtual void Update(InfoBoxWindow &infobox);
};

class InfoBoxContentAltitudeBaro : public InfoBoxContentAltitude
{
public:
  virtual void Update(InfoBoxWindow &infobox);
};

class InfoBoxContentAltitudeQFE : public InfoBoxContentAltitude
{
public:
  virtual void Update(InfoBoxWindow &infobox);
};

class InfoBoxContentTerrainHeight : public InfoBoxContentAltitude
{
public:
  virtual void Update(InfoBoxWindow &infobox);
};

#endif
