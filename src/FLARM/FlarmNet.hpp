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

#ifndef XCSOAR_FLARM_NET_HPP
#define XCSOAR_FLARM_NET_HPP

#include "Record.hpp"

#include <tchar.h>

struct FlarmRecord;
class NLineReader;
class FlarmId;

/**
 * Handles the FlarmNet.org file
 */
namespace FlarmNet
{
  void Destroy();

  unsigned LoadFile(NLineReader &reader);

  /**
   * Reads the FlarmNet.org file and fills the map
   *
   * @param path the path of the file
   * @return the number of records read from the file
   */
  unsigned LoadFile(const TCHAR *path);

  /**
   * Finds a FLARMNetRecord object based on the given FLARM id
   * @param id FLARM id
   * @return FLARMNetRecord object
   */
  const FlarmRecord *FindRecordById(FlarmId id);

  /**
   * Finds a FLARMNetRecord object based on the given Callsign
   * @param cn Callsign
   * @return FLARMNetRecord object
   */
  const FlarmRecord *FindFirstRecordByCallSign(const TCHAR *cn);
  unsigned FindRecordsByCallSign(const TCHAR *cn, const FlarmRecord *array[],
                                 unsigned size);
  unsigned FindIdsByCallSign(const TCHAR *cn, FlarmId array[], unsigned size);
};

#endif
