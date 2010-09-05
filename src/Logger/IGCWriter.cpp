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

#include "Logger/IGCWriter.hpp"
#include "IO/TextWriter.hpp"
#include "NMEA/Info.hpp"
#include "Version.hpp"
#include "Compatibility/string.h"

#include <assert.h>

const IGCWriter::LogPoint_GPSPosition &
IGCWriter::LogPoint_GPSPosition::operator=(const NMEA_INFO &gps_info)
{
  Location = gps_info.Location;
  GPSAltitude = (int)gps_info.GPSAltitude;
  Initialized = true;

  return *this;
}

static char *
igc_format_location(char *buffer, const GEOPOINT &location)
{
  const fixed Latitude = location.Latitude.value_degrees();
  const fixed Longitude = location.Longitude.value_degrees();

  int DegLat, DegLon;
  fixed MinLat, MinLon;
  char NoS, EoW;

  DegLat = (int)Latitude;
  MinLat = Latitude - fixed(DegLat);
  NoS = 'N';
  if (negative(MinLat) || (((int)MinLat - DegLat == 0) && DegLat < 0)) {
    NoS = 'S';
    DegLat *= -1;
    MinLat *= -1;
  }
  MinLat *= 60;
  MinLat *= 1000;

  DegLon = (int)Longitude;
  MinLon = Longitude - fixed(DegLon);
  EoW = 'E';
  if (negative(MinLon) || ((int)MinLon == DegLon && DegLon < 0)) {
    EoW = 'W';
    DegLon *= -1;
    MinLon *= -1;
  }
  MinLon *= 60;
  MinLon *= 1000;

  sprintf(buffer, "%02d%05.0f%c%03d%05.0f%c",
          DegLat, (double)MinLat, NoS, DegLon, (double)MinLon, EoW);

  return buffer + strlen(buffer);
}

IGCWriter::IGCWriter(const TCHAR *_path, const NMEA_INFO &gps_info)
  :Simulator(gps_info.gps.Simulator)
{
  _tcscpy(path, _path);

  frecord.reset();
  LastValidPoint.Initialized = false;

  if (!Simulator)
    grecord.Init();
}

bool
IGCWriter::flush()
{
  if (buffer.empty())
    return true;

  TextWriter writer(path, true);
  if (writer.error())
    return false;

  for (unsigned i = 0; i < buffer.length(); ++i) {
    if (!writer.writeln(buffer[i]))
      return false;

    grecord.AppendRecordToBuffer(buffer[i]);
  }

  if (!writer.flush())
    return false;

  buffer.clear();
  return true;
}

void
IGCWriter::finish(const NMEA_INFO &gps_info)
{
  if (gps_info.gps.Simulator)
    Simulator = true;

  flush();
}

static void
clean(char *p)
{
  for (; *p != 0; ++p)
    if (!GRecord::IsValidIGCChar(*p))
      *p = ' ';
}

bool
IGCWriter::writeln(const char *line)
{
  if (buffer.full() && !flush())
    return false;

  assert(!buffer.full());

  char *dest = buffer.append();
  strncpy(dest, line, MAX_IGC_BUFF);
  dest[MAX_IGC_BUFF - 1] = '\0';

  clean(dest);

  return true;
}

bool
IGCWriter::write_tstring(const char *a, const TCHAR *b)
{
  size_t a_length = strlen(a);
  size_t b_length = _tcslen(b);
  char buffer[a_length + b_length * 2 + 1];
  memcpy(buffer, a, a_length);

#ifdef _UNICODE
  sprintf(buffer, "%s%S", a, b);
#else
  memcpy(buffer + a_length, b, b_length + 1);
#endif

  return writeln(buffer);
}

/*
 HFDTE141203  <- should be UTC, same as time in filename
 HFFXA100
 HFPLTPILOT:JOHN WHARINGTON
 HFGTYGLIDERTYPE:LS 3
 HFGIDGLIDERID:VH-WUE
 HFDTM100GPSDATUM:WGS84
 HFRFWFIRMWAREVERSION:3.6
 HFRHWHARDWAREVERSION:3.4
 HFFTYFR TYPE:GARRECHT INGENIEURGESELLSCHAFT,VOLKSLOGGER 1.0
 HFCIDCOMPETITIONID:WUE
 HFCCLCOMPETITIONCLASS:FAI
 HFCIDCOMPETITIONID:WUE
 HFCCLCOMPETITIONCLASS:15M
*/

void
IGCWriter::header(const BrokenDateTime &DateTime,
                  const TCHAR *pilot_name, const TCHAR *aircraft_model,
                  const TCHAR *aircraft_registration,
                  const TCHAR *strAssetNumber, const TCHAR *driver_name)
{
  char datum[] = "HFDTM100Datum: WGS-84";
  char temp[100];

  // Flight recorder ID number MUST go first..
  sprintf(temp, "AXCS%c%c%c",
          (char)strAssetNumber[0],
          (char)strAssetNumber[1],
          (char)strAssetNumber[2]);
  writeln(temp);

  sprintf(temp, "HFDTE%02u%02u%02u",
          DateTime.day, DateTime.month, DateTime.year % 100);
  writeln(temp);

  if (!Simulator)
    writeln(GetHFFXARecord());

  write_tstring("HFPLTPILOT:", pilot_name);
  write_tstring("HFGTYGLIDERTYPE:", aircraft_model);
  write_tstring("HFGIDGLIDERID:", aircraft_registration);
  write_tstring("HFFTYFR TYPE:XCSOAR,XCSOAR ", XCSoar_VersionStringOld);
  write_tstring("HFGPS: ", driver_name);

  writeln(datum);

  if (!Simulator)
    writeln(GetIRecord());
}

void
IGCWriter::StartDeclaration(const BrokenDateTime &FirstDateTime, const int ntp)
{
  // TODO bug: this is causing problems with some analysis software
  // maybe it's because the date and location fields are bogus
  char start[] = "C0000000N00000000ETAKEOFF";
  char temp[100];

  // JMW added task start declaration line

  // LGCSTKF013945TAKEOFF DETECTED

  // IGC GNSS specification 3.6.1
  sprintf(temp, "C%02u%02u%02u%02u%02u%02u0000000000%02d",
          // DD  MM  YY  HH  MM  SS  DD  MM  YY IIII TT
          FirstDateTime.day,
          FirstDateTime.month,
          FirstDateTime.year % 100,
          FirstDateTime.hour,
          FirstDateTime.minute,
          FirstDateTime.second,
          ntp - 2);

  writeln(temp);
  // takeoff line
  // IGC GNSS specification 3.6.3
  writeln(start);
}

void
IGCWriter::EndDeclaration(void)
{
  // TODO bug: this is causing problems with some analysis software
  // maybe it's because the date and location fields are bogus
  const char start[] = "C0000000N00000000ELANDING";
  writeln(start);
}

void
IGCWriter::AddDeclaration(const GEOPOINT &location, const TCHAR *ID)
{
  char szCRecord[500];
  char IDString[MAX_PATH];
  int i;

  TCHAR tmpstring[MAX_PATH];
  _tcscpy(tmpstring, ID);
  _tcsupr(tmpstring);
  for (i = 0; i < (int)_tcslen(tmpstring); i++)
    IDString[i] = (char)tmpstring[i];

  IDString[i] = '\0';

  char *p = szCRecord;
  *p++ = 'C';
  p = igc_format_location(p, location);
  strcpy(p, IDString);

  writeln(szCRecord);
}

void
IGCWriter::LoggerNote(const TCHAR *text)
{
  char fulltext[500];
  sprintf(fulltext, "LPLT%S", text);
  writeln(fulltext);
}

void
IGCWriter::LogPoint(const NMEA_INFO& gps_info)
{
  char szBRecord[500];
  int iSIU = GetSIU(gps_info);
  fixed dEPE = GetEPE(gps_info);
  LogPoint_GPSPosition p;

  char IsValidFix;

  // if at least one GPS fix comes from the simulator, disable signing
  if (gps_info.gps.Simulator)
    Simulator = true;

  if (!Simulator) {
    const char *p = frecord.update(gps_info.gps.SatelliteIDs,
                                   gps_info.DateTime, gps_info.Time,
                                   gps_info.gps.NAVWarning);
    if (p != NULL)
      writeln(p);
  }

  if (!LastValidPoint.Initialized &&
      ((gps_info.GPSAltitude < fixed(-100))
       || (gps_info.BaroAltitude < fixed(-100))
          || gps_info.gps.NAVWarning))
    return;


  if (gps_info.gps.NAVWarning) {
    IsValidFix = 'V'; // invalid
    p = LastValidPoint;
  } else {
    IsValidFix = 'A'; // Active
    // save last active fix location
    p = LastValidPoint = gps_info;
  }

  char *q = szBRecord;
  sprintf(q, "B%02d%02d%02d",
          gps_info.DateTime.hour, gps_info.DateTime.minute,
          gps_info.DateTime.second);
  q += strlen(q);

  q = igc_format_location(q, p.Location);

  sprintf(q, "%c%05d%05d%03d%02d",
          IsValidFix, (int)gps_info.BaroAltitude, p.GPSAltitude,
          (int)dEPE, iSIU);

  writeln(szBRecord);
}

void
IGCWriter::LogEvent(const NMEA_INFO &gps_info, const char *event)
{
  char szBRecord[30];
  sprintf(szBRecord,"E%02d%02d%02d%s",
          gps_info.DateTime.hour, gps_info.DateTime.minute,
          gps_info.DateTime.second, event);

  writeln(szBRecord);
  // tech_spec_gnss.pdf says we need a B record immediately after an E record
  LogPoint(gps_info);
}

void
IGCWriter::sign()
{
  if (Simulator)
    return;

  // buffer is appended w/ each igc file write
  grecord.FinalizeBuffer();
  // read record built by individual file writes
  char OldGRecordBuff[MAX_IGC_BUFF];
  grecord.GetDigest(OldGRecordBuff);

  // now calc from whats in the igc file on disk
  grecord.Init();
  grecord.SetFileName(path);
  grecord.LoadFileToBuffer();
  grecord.FinalizeBuffer();
  char NewGRecordBuff[MAX_IGC_BUFF];
  grecord.GetDigest(NewGRecordBuff);

  bool bFileValid = strcmp(OldGRecordBuff, NewGRecordBuff) == 0;
  grecord.AppendGRecordToFile(bFileValid);
}
