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
#ifndef AIRSPACE_WARNING_HPP
#define AIRSPACE_WARNING_HPP

#include "AbstractAirspace.hpp"

/**
 * Class to hold information about active airspace warnings
 * \todo
 * - use settings for acknowledgement time
 */
class AirspaceWarning {
public:

  /**
   * Enumeration of airspace warning types
   */
  enum AirspaceWarningState {
    WARNING_CLEAR=0, /**< No warning active */
    WARNING_TASK, /**< Warning that task intersects airspace */
    WARNING_FILTER, /**< Warning that filtered state intersects airspace */
    WARNING_GLIDE, /**< Warning that short-term glide intersects airspace */
    WARNING_INSIDE /**< Warning that aircraft is currently inside airspace */
  };

/** 
 * Constructor.  All warnings uniquely refer to an airspace. 
 * 
 * @param the_airspace Airspace that this object will manage warnings for
 */
  AirspaceWarning(const AbstractAirspace& the_airspace);

/** 
 * Save warning state prior to performing update
 * 
 */
  void save_state();

/** 
 * Update warning state and solution vector
 * 
 * @param state New warning state
 * @param solution Intercept vector (to outside if currently inside, otherwise to inside)
 */
  void update_solution(const AirspaceWarningState state,
                       AirspaceInterceptSolution& solution);

/** 
 * Determine whether accepting a warning of the supplied state
 * will upgraded or equal the severity of the warning.
 * 
 * @param state New warning state
 */
  bool state_accepted(const AirspaceWarningState state) const;

/** 
 * Determine whether during last update, the state of this warning
 * changed.
 * 
 * @return True if state upgraded/downgraded
 */
  bool changed_state() const;

/** 
 * Access airspace managed by this object
 * 
 * @return Airspace 
 */
  const AbstractAirspace& get_airspace() const {
    return m_airspace;
  }

/** 
 * Access warning state
 * 
 * @return Warning state
 */
  const AirspaceWarningState& get_warning_state() const {
    return m_state;
  }

/** 
 * Update warnings, calculate whether this airspace still needs to
 * have warnings managed.
 * 
 * @return True if warning is still active
 */
  bool warning_live();

/** 
 * Determine if airspace warning was a dummy one created for testing but otherwise
 * can be removed without reporting.  This also applies to warnings that have
 * expired.
 * 
 * @return True if airspace warning has effectively always been clear.
 */
  bool trivial() const;

/** 
 * Access solution (nearest to enter, if outside, or to exit, if inside)
 * 
 * @return Reference to solution
 */
  const AirspaceInterceptSolution& get_solution() const {
    return m_solution;
  }

/** 
 * Determine if acknowledgement is expired (warning is current)
 * 
 * @return True if acknowledgement is expired
 */
  bool get_ack_expired() const;

/** 
 * Determine if acknowledgement is acknowledged for whole day
 * 
 * @return True if acknowledged
 */
  bool get_ack_day() const;

/** 
 * Acknowledge an airspace warning
 * 
 * @param set Whether to set or cancel acknowledgement
 */
  void acknowledge_warning(const bool set=true);

/** 
 * Acknowledge an airspace inside
 * 
 * @param set Whether to set or cancel acknowledgement
 */
  void acknowledge_inside(const bool set=true);

/** 
 * Acknowledge all warnings for airspace for whole day
 * 
 * @param set Whether to set or cancel acknowledgement
 */
  void acknowledge_day(const bool set=true);

/**
 * Ranking operator for warnings
 * 
 * @return True if this is more severe than that
 */
  bool operator < (const AirspaceWarning& that);

private:
  const AbstractAirspace& m_airspace;
  AirspaceWarningState m_state;
  AirspaceWarningState m_state_last;
  AirspaceInterceptSolution m_solution;

  unsigned m_acktime_warning;
  unsigned m_acktime_inside;
  unsigned m_debouncetime;
  bool m_ack_day;
  bool m_expired;
  bool m_expired_last;

public:

#ifdef DO_PRINT
  friend std::ostream& operator<< (std::ostream& f, 
                                   const AirspaceWarning& aw);
#endif
};

#endif
