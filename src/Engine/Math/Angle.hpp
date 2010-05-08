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
#ifndef ANGLE_HPP
#define ANGLE_HPP

#include "Math/fixed.hpp"
#ifdef DO_PRINT
#include <iostream>
#endif

class Angle {
private:
  Angle(const fixed& init_value): m_value(init_value) {};

public:
  Angle():m_value(fixed_zero) {};
  Angle(const Angle& angle): m_value(angle.m_value) {};

  static Angle native(const fixed& value) {
    return Angle(value);
  };
  fixed value_native() const {
    return m_value;
  }

#ifdef RADIANS
  static Angle degrees(const fixed& value) {
    return Angle(value*fixed_deg_to_rad);
  };
  static Angle radians(const fixed& value) {
    return Angle(value);
  };
  fixed value_degrees() const {
    return m_value*fixed_rad_to_deg;
  }
  fixed value_radians() const {
    return m_value;
  }
#else
  static Angle degrees(const fixed& value) {
    return Angle(value);
  };
  static Angle radians(const fixed& value) {
    return Angle(value*fixed_rad_to_deg);
  };
  fixed value_degrees() const {
    return m_value;
  }
  fixed value_radians() const {
    return m_value*fixed_deg_to_rad;
  }
#endif

  fixed sin() const;
  fixed cos() const;
  fixed fastsine() const;
  fixed fastcosine() const;
  fixed invfastcosine() const;
  int ifastsine() const;
  int ifastcosine() const;
  int sign() const;

  void sin_cos(fixed& s, fixed& c) const;
  
  fixed magnitude_degrees() const;
  fixed magnitude_radians() const;

  void flip();

  /**
   * Limits the angle (theta) to -180 - +180 degrees
   * @return Output angle (-180 - +180 degrees)
   */
  Angle as_delta() const;

  /**
   * Limits the angle (theta) to 0 - 360 degrees
   * @return Output angle (0-360 degrees)
   */
  Angle as_bearing() const;

  /**
   * Rotate angle by 180 degrees and limit to 0 - 360 degrees
   * @return Output angle (0 - 360 degrees)
   */
  Angle Reciprocal() const;

  Angle BiSector(const Angle &OutBound) const;

  Angle HalfAngle(const Angle &End) const;

  Angle operator* (const Angle x) const
    {
      return Angle(m_value*x.m_value);
    }

  Angle operator* (const fixed x) const
    {
      return Angle(m_value*x);
    }

  Angle operator/ (const fixed x) const
    {
      return Angle(m_value/x);
    }

  Angle operator+ (const Angle &x) const
    {
      return Angle(m_value+x.m_value);
    }

  Angle operator- (const Angle &x) const
    {
      return Angle(m_value-x.m_value);
    }

  const Angle& operator*= (const fixed x)
    {
      m_value*= x;
      return *this;
    }

  const Angle& 
  operator+= (const Angle& x)
    {
      m_value+= x.m_value;
      return *this;
    }

  const Angle& 
  operator-= (const Angle& x)
    {
      m_value-= x.m_value;
      return *this;
    }

  bool operator== (const Angle&x) const
    {
      return m_value== x.m_value;
    }

  bool operator!= (const Angle&x) const
    {
      return m_value!= x.m_value;
    }

  bool operator< (const Angle&x) const
    {
      return m_value < x.m_value;
    }

  bool operator> (const Angle&x) const
    {
      return m_value > x.m_value;
    }

  bool operator<= (const Angle&x) const
    {
      return m_value <= x.m_value;
    }

  bool operator>= (const Angle&x) const
    {
      return m_value >= x.m_value;
    }

#ifdef DO_PRINT
  friend std::ostream& operator<< (std::ostream& o, const Angle& a);
#endif

private:
  fixed m_value;
};

#endif
