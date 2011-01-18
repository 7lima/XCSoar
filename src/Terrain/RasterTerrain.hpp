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

#ifndef XCSOAR_TERRAIN_RASTER_TERRAIN_HPP
#define XCSOAR_TERRAIN_RASTER_TERRAIN_HPP

#include "RasterMap.hpp"
#include "Navigation/GeoPoint.hpp"
#include "Thread/Guard.hpp"
#include "Compiler.h"

#include <tchar.h>

class FileCache;

/**
 * Class to manage raster terrain database, potentially with 
 * caching or demand-loading.
 * 
 */
class RasterTerrain : public Guard<RasterMap> {
public:
  /** invalid value for terrain */
  static const short TERRAIN_INVALID = RasterBuffer::TERRAIN_INVALID;

protected:
  RasterMap map;

public:

/** 
 * Constructor.  Returns uninitialised object. 
 * 
 */
  RasterTerrain(const TCHAR *path, const TCHAR *world_file, FileCache *cache)
    :Guard<RasterMap>(map), map(path, world_file, cache) {}

/** 
 * Load the terrain.  Determines the file to load from profile settings.
 * 
 */
  static RasterTerrain *OpenTerrain(FileCache *cache);

  gcc_pure
  short GetTerrainHeight(const GeoPoint location) const {
    Lease lease(*this);
    return lease->GetField(location);
  }

  GeoPoint GetTerrainCenter() const {
    return map.GetMapCenter();
  }
};

#endif
