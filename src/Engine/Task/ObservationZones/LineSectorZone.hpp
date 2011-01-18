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

#ifndef LINESECTORZONE_HPP
#define LINESECTORZONE_HPP
#include "SymmetricSectorZone.hpp"

/**
 * Observation zone represented as a line.
 * Tests for inSector return true if the subject is behind the line
 * (within a semi-circle of diameter equal to the line length).
 * The constraint test ensures transitioning to exit only occurs if the
 * line is crossed (rather than exiting from the back semi-circle).
 */
class LineSectorZone: 
  public SymmetricSectorZone 
{
public:
/** 
 * Constructor
 * 
 * @param loc Location of center point of line
 * @param length Length of line (m)
 * 
 * @return Initialised object
 */
  LineSectorZone(const GeoPoint loc, const fixed length=fixed(1000.0)):
    SymmetricSectorZone(LINE, loc, length * fixed_half,
                        Angle::radians(fixed_pi))
  {
    updateSector();
  }

  ObservationZonePoint* clone(const GeoPoint * _location=0) const {
    if (_location) {
      return new LineSectorZone(*_location, getLength());
    } else {
      return new LineSectorZone(get_location(), getLength());
    }
  }

/** 
 * Check transition constraints -- for lines, both points have to
 * be within radius of OZ (otherwise it is a circumference border crossing)
 * 
 * @param ref_now Current aircraft state
 * @param ref_last Previous aircraft state
 * 
 * @return True if constraints are satisfied
 */
  bool transition_constraint(const AIRCRAFT_STATE & ref_now, 
                             const AIRCRAFT_STATE & ref_last) const {
    return CylinderZone::isInSector(ref_now) && CylinderZone::isInSector(ref_last);
  }

  GeoPoint get_boundary_parametric(fixed t) const;  

/** 
 * Distance reduction for scoring when outside this OZ
 * 
 * @return Distance (m) to subtract from score
 */
  fixed score_adjustment() const;

/** 
 * Set length property
 * 
 * @param new_length Length (m) of line
 */
  void setLength(const fixed new_length) {
    setRadius(new_length*fixed_half);
  }
  
/** 
 * Get length property value
 * 
 * @return Length (m) of line
 */
  fixed getLength() const {
    return getRadius()*fixed_two;
  }
};

#endif
