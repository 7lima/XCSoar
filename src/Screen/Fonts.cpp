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

#include "Screen/Fonts.hpp"
#include "LogFile.hpp"
#include "UtilsFont.hpp"
#include "InfoBoxes/InfoBoxLayout.hpp"
#include "ButtonLabel.hpp"
#include "Profile.hpp"
#include "Screen/Layout.hpp"
#include "Screen/VirtualCanvas.hpp"
#include "Appearance.hpp"
#include "InfoBoxes/InfoBoxLayout.hpp"

#include <stdio.h>

/// values inside infoboxes  like numbers, etc.
Font Fonts::InfoBox;
Font Fonts::InfoBoxSmall;
/// Titles of infoboxes like Next, WP L/D etc.
Font Fonts::Title;
/// text names on the map
Font Fonts::Map;
/// menu buttons, waypoint selection, messages, etc.
Font Fonts::MapBold;
/// vario display, runway informations
Font Fonts::CDI;
/// Flarm Traffic draweing and stats, map labels in italic
Font Fonts::MapLabel;

// these are the non-custom parameters
LOGFONT LogInfoBox;
LOGFONT LogTitle;
LOGFONT LogMap;
LOGFONT LogInfoBoxSmall;
LOGFONT LogMapBold;
LOGFONT LogCDI;
LOGFONT LogMapLabel;

#ifndef ENABLE_SDL

static bool
IsNullLogFont(LOGFONT logfont)
{
  LOGFONT LogFontBlank;
  memset((char *)&LogFontBlank, 0, sizeof(LOGFONT));
  return (memcmp(&logfont, &LogFontBlank, sizeof(LOGFONT)) == 0);
}

#endif /* !ENABLE_SDL */

static void
InitialiseLogfont(LOGFONT* font, const TCHAR* facename, int height,
                  bool bold = false, bool italic = false,
                  bool variable_pitch = true)
{
#ifndef ENABLE_SDL
  memset((char *)font, 0, sizeof(LOGFONT));

  _tcscpy(font->lfFaceName, facename);
  font->lfPitchAndFamily = (variable_pitch ? VARIABLE_PITCH : FIXED_PITCH)
                          | FF_DONTCARE;
  font->lfHeight = (long)height;
  font->lfWeight = (long)(bold ? FW_BOLD : FW_MEDIUM);
  font->lfItalic = italic;
  font->lfQuality = ANTIALIASED_QUALITY;
#endif /* !ENABLE_SDL */
}

void
Fonts::SetFont(Font *theFont, LOGFONT autoLogFont,
                  LOGFONT * LogFontUsed)
{
#ifdef ENABLE_SDL
  if (theFont->defined())
    return;

  // XXX hard coded path
  if (theFont->set("Data/Fonts/DejaVuSansCondensed2.ttf",
                   autoLogFont.lfHeight > 0 ? autoLogFont.lfHeight : 10,
                   autoLogFont.lfWeight >= 700,
                   autoLogFont.lfItalic) &&
      LogFontUsed != NULL)
    *LogFontUsed = autoLogFont;
#else /* !ENABLE_SDL */
  if (theFont->defined() || IsNullLogFont(autoLogFont))
    return;

  if (theFont->set(&autoLogFont) && LogFontUsed != NULL)
    *LogFontUsed = autoLogFont; // RLD save for custom font GUI
#endif /* !ENABLE_SDL */
}

void
Fonts::LoadCustomFont(Font *theFont, const TCHAR FontRegKey[], LOGFONT * LogFontUsed)
{
#ifdef ENABLE_SDL
  // XXX
#else /* !ENABLE_SDL */
  LOGFONT logfont;
  memset((char *)&logfont, 0, sizeof(LOGFONT));
  if (!Profile::GetFont(FontRegKey, &logfont))
    return;

  if (theFont->set(&logfont) && LogFontUsed != NULL)
    *LogFontUsed = logfont; // RLD save for custom font GUI
#endif /* !ENABLE_SDL */
}

static void
InitialiseFontsAltair()
{
  if (!is_altair())
    return;

  InitialiseLogfont(&LogInfoBox, _T("RasterGothicTwentyFourCond"), 24, true);
  InitialiseLogfont(&LogTitle, _T("RasterGothicNineCond"), 10);
  InitialiseLogfont(&LogCDI, _T("RasterGothicEighteenCond"), 19, true);
  InitialiseLogfont(&LogMapLabel, _T("RasterGothicTwelveCond"), 13);
  InitialiseLogfont(&LogMap, _T("RasterGothicFourteenCond"), 15);
  InitialiseLogfont(&LogMapBold, _T("RasterGothicFourteenCond"), 15, true);
  InitialiseLogfont(&LogInfoBoxSmall, _T("RasterGothicEighteenCond"), 19, true);
}

static void
InitialiseLogFonts()
{
  int FontHeight = Layout::SmallScale(35);

#ifndef ENABLE_SDL
  // oversize first so can then scale down
  InitialiseLogfont(&LogInfoBox, Fonts::GetStandardFontFace(),
                    (int)(FontHeight * 1.4), true, false, true);
  LogInfoBox.lfCharSet = ANSI_CHARSET;

  // JMW algorithm to auto-size info window font.
  // this is still required in case title font property doesn't exist.
  VirtualCanvas canvas(1, 1);
  SIZE tsize;
  do {
    HFONT TempWindowFont;
    HFONT hfOld;

    --LogInfoBox.lfHeight;

    TempWindowFont = CreateFontIndirect(&LogInfoBox);
    hfOld = (HFONT)SelectObject(canvas, TempWindowFont);

    tsize = canvas.text_size(_T("1234m"));
    // unselect it before deleting it
    SelectObject(canvas, hfOld);
    DeleteObject(TempWindowFont);
  } while ((unsigned)tsize.cx > InfoBoxLayout::ControlWidth);

  ++LogInfoBox.lfHeight;

#else /* !ENABLE_SDL */
  // XXX implement
#endif /* !ENABLE_SDL */

  InitialiseLogfont(&LogTitle, Fonts::GetStandardFontFace(),
                    FontHeight / 3, true);

  // new font for CDI Scale
  InitialiseLogfont(&LogCDI, Fonts::GetStandardFontFace(),
                    (int)(FontHeight * 0.6), false, false, false);

  // new font for map labels
  InitialiseLogfont(&LogMapLabel, Fonts::GetStandardFontFace(),
                    (int)(FontHeight * 0.39), false, true);

  // new font for map labels
  InitialiseLogfont(&LogMap, Fonts::GetStandardFontFace(),
                    (int)(FontHeight * 0.507));

  // Font for map bold text
  InitialiseLogfont(&LogMapBold, Fonts::GetStandardFontFace(),
                    (int)(FontHeight * 0.507), true);

  InitialiseLogfont(&LogInfoBoxSmall, Fonts::GetStandardFontFace(),
                    Layout::Scale(20));
}

void
Fonts::Initialize(bool use_custom)
{
  InitialiseLogFonts();

  InitialiseFontsAltair();

  SetFont(&InfoBox, LogInfoBox);
  SetFont(&InfoBoxSmall, LogInfoBoxSmall);
  SetFont(&Title, LogTitle);
  SetFont(&CDI, LogCDI);
  SetFont(&MapLabel, LogMapLabel);
  SetFont(&Map, LogMap);
  SetFont(&MapBold, LogMapBold);

  if (use_custom) {
    LoadCustomFont(&InfoBox, szProfileFontInfoWindowFont);
    LoadCustomFont(&InfoBoxSmall, szProfileFontTitleSmallWindowFont);
    LoadCustomFont(&Title, szProfileFontTitleWindowFont);
    LoadCustomFont(&CDI, szProfileFontCDIWindowFont);
    LoadCustomFont(&MapLabel, szProfileFontMapLabelFont);
    LoadCustomFont(&Map, szProfileFontMapWindowFont);
    LoadCustomFont(&MapBold, szProfileFontMapWindowBoldFont);
  }
}

const TCHAR*
Fonts::GetStandardFontFace()
{
  if (is_altair())
    return _T("RasterGothicFourteenCond");

  return _T("Tahoma");
}
