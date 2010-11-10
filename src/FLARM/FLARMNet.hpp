/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
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

#ifndef XCSOAR_FLARM_NET_HPP
#define XCSOAR_FLARM_NET_HPP

#include "FLARM/Traffic.hpp"

#include <map>
#include <tchar.h>

class NLineReader;

/**
 * FLARMnet.org file entry
 */
class FLARMNetRecord
{
public:
  TCHAR id[7];          /**< FLARM id 6 bytes */
  TCHAR name[22];       /**< Name 15 bytes */
  TCHAR airfield[22];   /**< Airfield 4 bytes */
  TCHAR type[22];       /**< Aircraft type 1 byte */
  TCHAR reg[8];         /**< Registration 7 bytes */
  TCHAR cn[4];          /**< Callsign 3 bytes */
  TCHAR freq[8];        /**< Radio frequency 6 bytes */

  FlarmId GetId() const;
};

/**
 * Handles the FLARMnet.org file
 */
class FLARMNetDatabase
  : protected std::map<FlarmId, FLARMNetRecord*>
{
public:
  ~FLARMNetDatabase();

  unsigned LoadFile(NLineReader &reader);
  unsigned LoadFile(const TCHAR *path);

  const FLARMNetRecord *Find(FlarmId id) const;
  const FLARMNetRecord *Find(const TCHAR *cn) const;
};

#endif
