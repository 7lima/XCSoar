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

#include "Topography/TopographyGlue.hpp"
#include "Topography/TopographyStore.hpp"
#include "Language.hpp"
#include "Profile/Profile.hpp"
#include "LogFile.hpp"
#include "ProgressGlue.hpp"
#include "IO/FileLineReader.hpp"
#include "IO/ZipLineReader.hpp"
#include "IO/FileLineReader.hpp"
#include "OS/FileUtil.hpp"
#include "OS/PathName.hpp"

#include <zzip/zzip.h>

#include <windef.h> /* for MAX_PATH */

/**
 * Load topology from a plain file, load the other files from the same
 * directory.
 */
static bool
LoadConfiguredTopologyFile(TopographyStore &store)
{
  TCHAR file[MAX_PATH];
  if (!Profile::GetPath(szProfileTopologyFile, file))
    return false;

  FileLineReaderA reader(file);
  if (reader.error()) {
    LogStartUp(_T("No topology file: %s"), file);
    return false;
  }

  TCHAR buffer[MAX_PATH];
  const TCHAR *directory = DirName(file, buffer);
  if (directory == NULL)
    return false;

  store.Load(reader, directory);
  return true;
}

/**
 * Load topology from the map file (ZIP), load the other files from
 * the same ZIP file.
 */
static bool
LoadConfiguredTopologyZip(TopographyStore &store)
{
  TCHAR path[MAX_PATH];
  if (!Profile::GetPath(szProfileMapFile, path))
    return false;

  ZZIP_DIR *dir = zzip_dir_open(NarrowPathName(path), NULL);
  if (dir == NULL)
    return false;

  ZipLineReaderA reader(dir, "topology.tpl");
  if (reader.error()) {
    LogStartUp(_T("No topology in map file: %s"), path);
    return false;
  }

  store.Load(reader, NULL, dir);
  zzip_dir_close(dir);
  return true;
}

bool
LoadConfiguredTopology(TopographyStore &store)
{
  LogStartUp(_T("Loading Topology File..."));
  ProgressGlue::Create(_("Loading Topology File..."));

  return LoadConfiguredTopologyFile(store) ||
    LoadConfiguredTopologyZip(store);
}
