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

#ifndef RASTERMAP_H
#define RASTERMAP_H

#include "Sizes.h"
#include <zzip/lib.h>
#include "Poco/RWLock.h"

#include <windef.h> /* for MAX_PATH */

#include "Navigation/GeoPoint.hpp"

typedef struct _TERRAIN_INFO
{
  GEOPOINT TopLeft;
  GEOPOINT BottomRight;
  Angle StepSize;
  long Rows;
  long Columns;
} TERRAIN_INFO;

class RasterRounding;


class RasterMap {
 public:
  RasterMap():
    terrain_valid(false),
    max_field_value(0),
    DirectAccess(false),
    Paged(false)
    {}
  virtual ~RasterMap() {};

  inline bool isMapLoaded() const {
    return terrain_valid;
  }

  short max_field_value;
  TERRAIN_INFO TerrainInfo;

  virtual void SetViewCenter(const GEOPOINT &location) {};

  bool GetMapCenter(GEOPOINT *loc) const;

  // accurate method
  int GetEffectivePixelSize(fixed &pixel_D,
                            const GEOPOINT &location) const;

  virtual void SetFieldRounding(const GEOPOINT& delta,
                                RasterRounding &rounding) const;

  short GetField(const GEOPOINT &location,
    const RasterRounding &rounding);

  virtual void ServiceCache() {};
  virtual void ServiceFullReload(const GEOPOINT &location) {};

  bool IsDirectAccess(void) const {
    return DirectAccess;
  };

  bool IsPaged(void) const {
    return Paged;
  };

  // export methods to global, take care!
  virtual void LockRead();
  virtual void Unlock();

 protected:
  Poco::RWLock lock;

  bool terrain_valid;
  bool Paged;

  bool DirectAccess;

  virtual short _GetFieldAtXY(unsigned int lx,
                              unsigned int ly) = 0;
};

class RasterRounding {
public:
  RasterRounding() {};

  RasterRounding(const RasterMap &map,
                 const GEOPOINT& delta):
    DirectFine(false)
  {
    Set(map, delta);
  };

  RasterRounding(const RasterMap &map):
    DirectFine(false)
  {
    Set(map, GEOPOINT());
  };

  void Set(const RasterMap &map,
           const GEOPOINT& delta)
  {
    map.SetFieldRounding(delta,*this);
  }

  bool DirectFine;
  int xlleft;
  int xlltop;
  fixed fXrounding, fYrounding;
  fixed fXroundingFine, fYroundingFine;
  int Xrounding, Yrounding;
};


#endif
