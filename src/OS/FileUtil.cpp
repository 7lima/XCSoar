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

#include "OS/FileUtil.hpp"

#include "OS/PathName.hpp"
#include "Compatibility/path.h"

#include <windef.h> /* for MAX_PATH */

#include <sys/types.h>
#include <sys/stat.h>

#ifdef HAVE_POSIX
#include <dirent.h>
#include <unistd.h>
#include <fnmatch.h>
#endif

void
Directory::Create(const TCHAR* path)
{
#ifdef HAVE_POSIX
  mkdir(path, 0777);
#else /* !HAVE_POSIX */
  CreateDirectory(path, NULL);
#endif /* !HAVE_POSIX */
}

bool
Directory::Exists(const TCHAR* path)
{
#ifdef HAVE_POSIX
  struct stat st;
  if (stat(NarrowPathName(path), &st) != 0)
    return false;

  return (st.st_mode & S_IFDIR);
#else
  DWORD attributes = GetFileAttributes(path);
  return attributes != INVALID_FILE_ATTRIBUTES &&
    (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
#endif
}

/**
 * Checks whether the given string str equals "." or ".."
 * @param str The string to check
 * @return True if string equals "." or ".."
 */
static bool
IsDots(const TCHAR* str)
{
  return !(_tcscmp(str, _T(".")) && _tcscmp(str, _T("..")));
}

#ifndef HAVE_POSIX /* we use fnmatch() on POSIX */
static bool
checkFilter(const TCHAR *filename, const TCHAR *filter)
{
  // filter = e.g. "*.igc" or "config/*.prf"
  // todo: make filters like "config/*.prf" work

  // if invalid or short filter "*" -> return true
  // todo: check for asterisk
  if (!filter || string_is_empty(filter + 1))
    return true;

  // Copy filter without first char into upfilter
  // *.igc         ->  .igc
  // config/*.prf  ->  onfig/*.prf
  TCHAR upfilter[MAX_PATH];
  _tcscpy(upfilter, filter + 1);

  // Search for upfilter in filename (e.g. ".igc" in "934CFAE1.igc") and
  //   save the position of the first occurence in ptr
  const TCHAR *ptr = _tcsstr(filename, upfilter);
  if (ptr != NULL && _tcslen(ptr) == _tcslen(upfilter))
    // If upfilter was found at the very end of filename
    // -> filename matches filter
    return true;

  // Convert upfilter to uppercase
  _tcsupr(upfilter);

  // And do it all again
  ptr = _tcsstr(filename, upfilter);

  // If still no match found -> filename does not match the filter
  return (ptr != NULL && _tcslen(ptr) == _tcslen(upfilter));
}

static bool
ScanFiles(File::Visitor &visitor, const TCHAR* sPath,
          const TCHAR* filter = _T("*"))
{
  HANDLE hFind; // file handle
  WIN32_FIND_DATA FindFileData;

  TCHAR DirPath[MAX_PATH];
  TCHAR FileName[MAX_PATH];

  if (sPath)
    _tcscpy(DirPath, sPath);
  else
    DirPath[0] = 0;

  _tcscat(DirPath, _T(DIR_SEPARATOR_S));
  _tcscpy(FileName, DirPath);
  _tcscat(DirPath, filter);

  hFind = FindFirstFile(DirPath, &FindFileData); // find the first file
  if (hFind == INVALID_HANDLE_VALUE)
    return false;

  _tcscpy(DirPath, FileName);

  bool done = false;
  do { // until we finds an entry
    if (!IsDots(FindFileData.cFileName) &&
        !(FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) &&
        checkFilter(FindFileData.cFileName, filter)) {
      _tcscat(FileName, FindFileData.cFileName);
      visitor.Visit(FileName, FindFileData.cFileName);
    }
    _tcscpy(FileName, DirPath);

    if (!FindNextFile(hFind, &FindFileData)) {
      if (GetLastError() == ERROR_NO_MORE_FILES) // no more files there
        done = true;
      else {
        // some error occured, close the handle and return false
        FindClose(hFind);
        return false;
      }
    }
  } while (!done);
  FindClose(hFind); // closing file handle

  return true;
}
#endif /* !HAVE_POSIX */

static bool
ScanDirectories(File::Visitor &visitor, bool recursive,
                const TCHAR* sPath, const TCHAR* filter = _T("*"))
{
#ifdef HAVE_POSIX
  DIR *dir = opendir(sPath);
  if (dir == NULL)
    return false;

  TCHAR FileName[MAX_PATH];
  _tcscpy(FileName, sPath);
  size_t FileNameLength = _tcslen(FileName);
  FileName[FileNameLength++] = '/';

  struct dirent *ent;
  while ((ent = readdir(dir)) != NULL) {
    if (IsDots(ent->d_name))
      continue;

    _tcscpy(FileName + FileNameLength, ent->d_name);

    struct stat st;
    if (stat(FileName, &st) < 0)
      continue;

    if (S_ISDIR(st.st_mode) && recursive)
      ScanDirectories(visitor, true, FileName, filter);
    else if (S_ISREG(st.st_mode) && fnmatch(filter, ent->d_name, 0) == 0)
      visitor.Visit(FileName, ent->d_name);
  }

  closedir(dir);
#else /* !HAVE_POSIX */
  TCHAR DirPath[MAX_PATH];
  TCHAR FileName[MAX_PATH];

  if (sPath) {
    _tcscpy(DirPath, sPath);
    _tcscpy(FileName, sPath);
  } else {
    DirPath[0] = 0;
    FileName[0] = 0;
  }

  ScanFiles(visitor, FileName, filter);

  if (!recursive)
    return true;

  _tcscat(DirPath, _T(DIR_SEPARATOR_S));
  _tcscat(FileName, _T(DIR_SEPARATOR_S "*"));

  WIN32_FIND_DATA FindFileData;
  HANDLE hFind = FindFirstFile(FileName, &FindFileData); // find the first file
  if (hFind == INVALID_HANDLE_VALUE)
    return false;

  bool done = false;
  do { // until we finds an entry
    _tcscpy(FileName, DirPath);
    if (!IsDots(FindFileData.cFileName) &&
        (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
      // we have found a directory, recurse
      _tcscat(FileName, FindFileData.cFileName);
      ScanDirectories(visitor, true, FileName, filter);
    }

    if (!FindNextFile(hFind, &FindFileData)) {
      if (GetLastError() == ERROR_NO_MORE_FILES) // no more files there
        done = true;
      else {
        // some error occured, close the handle and return false
        FindClose(hFind);
        return false;
      }
    }
  } while (!done);
  FindClose(hFind); // closing file handle

#endif /* !HAVE_POSIX */

  return true;
}

void
Directory::VisitFiles(const TCHAR* path, File::Visitor &visitor, bool recursive)
{
  ScanDirectories(visitor, recursive, path);
}

void
Directory::VisitSpecificFiles(const TCHAR* path, const TCHAR* filter,
                              File::Visitor &visitor, bool recursive)
{
  ScanDirectories(visitor, recursive, path, filter);
}

bool
File::Exists(const TCHAR* path)
{
#ifdef HAVE_POSIX
  struct stat st;
  if (stat(NarrowPathName(path), &st) != 0)
    return false;

  return (st.st_mode & S_IFREG);
#else
  DWORD attributes = GetFileAttributes(path);
  return attributes != INVALID_FILE_ATTRIBUTES &&
    (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
#endif
}
