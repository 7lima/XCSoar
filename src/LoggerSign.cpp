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

#include "LoggerImpl.hpp"
#include "Protection.hpp"
#include "Dialogs.h"
#include "Device/Port.hpp"
#include "SettingsTask.hpp"
#include "Math/Earth.hpp"
#include "LogFile.hpp"
#include "Asset.hpp"
#include "LocalPath.hpp"
#include "Device/device.hpp"
#include "InputEvents.h"
#include "Compatibility/string.h"
#include "UtilsFile.hpp"
#include "UtilsText.hpp" // for ConvertToC()

#include <assert.h>
#include <tchar.h>

/* the GetProcAddress() prototype differs between Windows CE and
   desktop Windows */
#ifdef _WIN32_WCE
#define PROC_NAME(x) _T(x)
#else
#define PROC_NAME(x) (x)
#endif

#ifdef WIN32

static HINSTANCE GRecordDLLHandle = NULL;

// Procedures for explicitly loaded (optional) GRecord DLL
typedef int (*GRRECORDGETVERSION)(TCHAR * szOut);
static GRRECORDGETVERSION GRecordGetVersion;

#endif /* !WIN32 */

typedef int (*GRECORDINIT)(void);
static GRECORDINIT GRecordInit;

#ifdef WIN32

typedef int (*GRECORDGETDIGESTMAXLEN)(void);
static GRECORDGETDIGESTMAXLEN GRecordGetDigestMaxLen;

#endif /* !WIN32 */

typedef int (*GRECORDAPPENDRECORDTOBUFFER)(TCHAR * szIn);
static GRECORDAPPENDRECORDTOBUFFER GRecordAppendRecordToBuffer;

typedef int (*GRECORDFINALIZEBUFFER)(void);
static GRECORDFINALIZEBUFFER GRecordFinalizeBuffer;

typedef int (*GRECORDGETDIGEST)(TCHAR * szOut);
static GRECORDGETDIGEST GRecordGetDigest;

typedef int (*GRECORDSETFILENAME)(TCHAR * szIn);
static GRECORDSETFILENAME GRecordSetFileName;

typedef int (*GRECORDLOADFILETOBUFFER)(void);
static GRECORDLOADFILETOBUFFER GRecordLoadFileToBuffer;

typedef int (*GRECORDAPPENDGRECORDTOFILE)(BOOL bValid);
static GRECORDAPPENDGRECORDTOFILE GRecordAppendGRecordToFile;

#ifdef WIN32

typedef int (*GRECORDREADGRECORDFROMFILE)(TCHAR szOutput[]);
static GRECORDREADGRECORDFROMFILE GRecordReadGRecordFromFile;

typedef int (*GRECORDVERIFYGRECORDINFILE)(void);
static GRECORDVERIFYGRECORDINFILE GRecordVerifyGRecordInFile;

#endif /* !WIN32 */

/**
 * Checks whether the character c is a valid IGC character
 * @param c Character to check
 * @return True if valid character, False otherwise
 */
bool
IsValidIGCChar(char c) //returns 1 if valid char for IGC files
{
  if (c >= 0x20 &&
      c <= 0x7E &&
      c != 0x0D &&
      c != 0x0A &&
      c != 0x24 &&
      c != 0x2A &&
      c != 0x2C &&
      c != 0x21 &&
      c != 0x5C &&
      c != 0x5E &&
      c != 0x7E)
    return true;
  else
    return false;
}

/**
 * Checks a string for invalid characters and replaces them with 0x20 (space)
 * @param szIn Input/Output string (pointer)
 */
void
CleanIGCRecord(char * szIn)
{  
  // don't clean terminating \r\n!
  int iLen = strlen(szIn) - 2;
  for (int i = 0; i < iLen; i++) {
    if (!IsValidIGCChar(szIn[i]))
      szIn[i] = ' ';
  }
}

bool
LoggerImpl::IGCWriteRecord(const char *szIn, const TCHAR* szLoggerFileName)
{
  Poco::ScopedRWLock protect(lock, true);

  char charbuffer[MAX_IGC_BUFF];

  strncpy(charbuffer, szIn, MAX_IGC_BUFF);
  // just to be safe
  charbuffer[MAX_IGC_BUFF - 1] = '\0';
  CleanIGCRecord(charbuffer);
  return DiskBufferAdd(charbuffer);
}

/**
 * Flushes the data in the DiskBuffer to the disk
 */
void
LoggerImpl::DiskBufferFlush()
{
  FILE * LoggerFILE;

  ConvertTToC(szLoggerFileName_c, szLoggerFileName);
  szLoggerFileName_c[_tcslen(szLoggerFileName)] = 0;
  // stays open for buffered io
  LoggerFILE = fopen (szLoggerFileName_c,"ab");

  if (!LoggerFILE)
    return;

  TCHAR buffer_G[MAX_IGC_BUFF];
  TCHAR * pbuffer_G;
  pbuffer_G = buffer_G;

  for (int i = 0; i < LoggerDiskBufferCount; i++) {
    unsigned int iLen = strlen(LoggerDiskBuffer[i]);

    // if (file write successful)
    if (fwrite(LoggerDiskBuffer[i], (size_t)iLen, (size_t)1, LoggerFILE) == (size_t)iLen) {
      int iBufLen = strlen(LoggerDiskBuffer[i]);

      for (int j = 0; (j <= iBufLen) && (j < MAX_IGC_BUFF); j++) {
        buffer_G[j] = (TCHAR)LoggerDiskBuffer[i][j];
      }

      if (!Simulator && LoggerGActive())
        GRecordAppendRecordToBuffer(pbuffer_G);
    }
  }

  fclose(LoggerFILE);
  DiskBufferReset();
}

/**
 * Adds the given string to the DiskBuffer
 * @param sIn Input string
 * @return True if adding was successful, False otherwise (Buffer full)
 */
bool
LoggerImpl::DiskBufferAdd(char *sIn)
{
  if (LoggerDiskBufferCount >= LOGGER_DISK_BUFFER_NUM_RECS) {
    DiskBufferFlush();
  }

  if (LoggerDiskBufferCount >= LOGGER_DISK_BUFFER_NUM_RECS)
    return false;

  strncpy(LoggerDiskBuffer[LoggerDiskBufferCount], sIn, MAX_IGC_BUFF);
  LoggerDiskBuffer[LoggerDiskBufferCount][MAX_IGC_BUFF - 1] = '\0';
  LoggerDiskBufferCount++;

  return true;
}

/**
 * Resets the DiskBuffer
 */
void
LoggerImpl::DiskBufferReset()
{
  for (int i = 0; i < LOGGER_DISK_BUFFER_NUM_RECS; i++) {
    LoggerDiskBuffer[i][0] = '\0';
  }
  LoggerDiskBufferCount = 0;
}

// VENTA3 TODO: if ifdef PPC2002 load correct dll. Put the dll inside
// XCSoarData, so users can place their executable XCS wherever they
// want.
//
// JMW: not sure that would work, I think dll has to be in OS
// directory or same directory as exe

void
LoggerImpl::LinkGRecordDLL()
{
#ifdef WIN32
  static bool bFirstTime = true;
  TCHAR szLoadResults [100];
  TCHAR szGRecordVersion[100];

  // only try to load DLL once per session
  if ((GRecordDLLHandle == NULL) && bFirstTime) {
    bFirstTime = false;

    LogStartUp(TEXT("Searching for GRecordDLL"));
    if (is_altair()) {
      if (FileExists(TEXT("\\NOR Flash\\GRecordDLL.dat"))) {
        LogStartUp(TEXT("Updating GRecordDLL.DLL"));
        DeleteFile(TEXT("\\NOR Flash\\GRecordDLL.DLL"));
        MoveFile(TEXT("\\NOR Flash\\GRecordDLL.dat"),
            TEXT("\\NOR Flash\\GRecordDLL.DLL"));
      }

      GRecordDLLHandle = LoadLibrary(TEXT("\\NOR Flash\\GRecordDLL.DLL"));
    } else {
      GRecordDLLHandle = LoadLibrary(TEXT("GRecordDLL.DLL"));
    }

    if (GRecordDLLHandle != NULL) {
      // if any pointers don't link, disable entire library
      BOOL bLoadOK = true;

      GRecordGetVersion = (GRRECORDGETVERSION)GetProcAddress(GRecordDLLHandle,
                                                             PROC_NAME("GRecordGetVersion"));

      // read version for log
      if (!GRecordGetVersion) {
        bLoadOK = false;
        _tcscpy(szGRecordVersion, TEXT("version unknown"));
      } else {
        GRecordGetVersion(szGRecordVersion);
      }

      GRecordInit = (GRECORDINIT)GetProcAddress(GRecordDLLHandle,
                                                PROC_NAME("GRecordInit"));

      if (!GRecordInit)
        bLoadOK = false;

      GRecordGetDigestMaxLen = (GRECORDGETDIGESTMAXLEN)GetProcAddress(
          GRecordDLLHandle, PROC_NAME("GRecordGetDigestMaxLen"));

      if (!GRecordGetDigestMaxLen)
        bLoadOK = false;

      GRecordAppendRecordToBuffer
          = (GRECORDAPPENDRECORDTOBUFFER)GetProcAddress(GRecordDLLHandle,
              PROC_NAME("GRecordAppendRecordToBuffer"));

      if (!GRecordAppendRecordToBuffer)
        bLoadOK = false;

      GRecordFinalizeBuffer = (GRECORDFINALIZEBUFFER)GetProcAddress(
          GRecordDLLHandle, PROC_NAME("GRecordFinalizeBuffer"));

      if (!GRecordFinalizeBuffer)
        bLoadOK = false;

      GRecordGetDigest = (GRECORDGETDIGEST)GetProcAddress(GRecordDLLHandle,
          PROC_NAME("GRecordGetDigest"));

      if (!GRecordGetDigest)
        bLoadOK = false;

      GRecordSetFileName = (GRECORDSETFILENAME)GetProcAddress(GRecordDLLHandle,
          PROC_NAME("GRecordSetFileName"));

      if (!GRecordSetFileName)
        bLoadOK = false;

      GRecordLoadFileToBuffer = (GRECORDLOADFILETOBUFFER)GetProcAddress(
          GRecordDLLHandle, PROC_NAME("GRecordLoadFileToBuffer"));

      if (!GRecordLoadFileToBuffer)
        bLoadOK = false;

      GRecordAppendGRecordToFile = (GRECORDAPPENDGRECORDTOFILE)GetProcAddress(
          GRecordDLLHandle, PROC_NAME("GRecordAppendGRecordToFile"));

      if (!GRecordAppendGRecordToFile)
        bLoadOK = false;

      GRecordReadGRecordFromFile = (GRECORDREADGRECORDFROMFILE)GetProcAddress(
          GRecordDLLHandle, PROC_NAME("GRecordReadGRecordFromFile"));

      if (!GRecordReadGRecordFromFile)
        bLoadOK = false;

      GRecordVerifyGRecordInFile = (GRECORDVERIFYGRECORDINFILE)GetProcAddress(
          GRecordDLLHandle, PROC_NAME("GRecordVerifyGRecordInFile"));

      if (!GRecordVerifyGRecordInFile)
        bLoadOK=false;

      // all need to link, or disable entire library.
      if (!bLoadOK) {
        _stprintf(szLoadResults, TEXT("Found GRecordDLL %s but incomplete"),
            szGRecordVersion);
        FreeLibrary(GRecordDLLHandle);
        GRecordDLLHandle = NULL;
      } else {
        _stprintf(szLoadResults, TEXT("Loaded GRecordDLL %s"),
            szGRecordVersion);
      }
    } else {
      _tcscpy(szLoadResults, TEXT("Can't load GRecordDLL"));
    }

    LogStartUp(szLoadResults);
  }
#endif
}

/**
 * Returns whether the GRecord DLL has been found
 * @return True if the GRecord DLL has been found, False otherwise
 */
bool
LoggerImpl::LoggerGActive() const
{
#ifdef WIN32
  if (GRecordDLLHandle)
    return true;
  else
#endif
    return false;
}

void
LoggerImpl::LoggerGStop(TCHAR* szLoggerFileName)
{
  BOOL bFileValid = true;
  TCHAR OldGRecordBuff[MAX_IGC_BUFF];
  TCHAR NewGRecordBuff[MAX_IGC_BUFF];

  // buffer is appended w/ each igc file write
  GRecordFinalizeBuffer();
  // read record built by individual file writes
  GRecordGetDigest(OldGRecordBuff);

  // now calc from whats in the igc file on disk
  GRecordInit();
  GRecordSetFileName(szLoggerFileName);
  GRecordLoadFileToBuffer();
  GRecordFinalizeBuffer();
  GRecordGetDigest(NewGRecordBuff);

  for (unsigned int i = 0; i < 128; i++)
    if (OldGRecordBuff[i] != NewGRecordBuff[i])
      bFileValid = false;

  GRecordAppendGRecordToFile(bFileValid);
}

/**
 * Initialize the GRecord part of the Logger
 * -> Link the DLL
 * -> Call external GRecordInit()
 */
void
LoggerImpl::LoggerGInit()
{
  // try to link DLL if it exists
  LinkGRecordDLL();
  if (LoggerGActive())
    GRecordInit();
}
