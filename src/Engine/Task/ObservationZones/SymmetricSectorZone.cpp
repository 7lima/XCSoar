/* Copyright_License {

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
#include "SymmetricSectorZone.hpp"

void SymmetricSectorZone::set_legs(const TaskPoint *previous,
                                   const TaskPoint *current,
                                   const TaskPoint *next) 
{
  Angle biSector;
  if (!next && previous) { 
    // final
    biSector = previous->bearing(current->get_location());
  } else if (next && previous) {
    // intermediate
    biSector = previous->bearing(current->get_location()).
      BiSector(current->bearing(next->get_location()));
  } else if (next && !previous) {
    // start
    biSector = next->bearing(current->get_location());
  } else {
    // single point
    biSector = Angle();
  }

  setStartRadial((biSector-SectorAngle*fixed_half).as_bearing());
  setEndRadial((biSector+SectorAngle*fixed_half).as_bearing());
}



bool
SymmetricSectorZone::equals(const ObservationZonePoint* other) const
{
  if (CylinderZone::equals(other)) {
    if (const SymmetricSectorZone* z = 
        dynamic_cast<const SymmetricSectorZone*>(other)) {
      return (SectorAngle == z->getSectorAngle());
    }
  }
  return false;
}
