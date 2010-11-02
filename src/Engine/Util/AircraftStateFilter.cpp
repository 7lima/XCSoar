/* Copyright_License {

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
#include "AircraftStateFilter.hpp"
#include "Navigation/Geometry/GeoVector.hpp"

AircraftStateFilter::AircraftStateFilter(const AIRCRAFT_STATE& state,
                                         const fixed cutoff_wavelength):
  m_df_x(fixed_zero),
  m_df_y(fixed_zero),
  m_df_alt(fixed_zero),
  m_lpf_x(cutoff_wavelength),
  m_lpf_y(cutoff_wavelength),
  m_lpf_alt(cutoff_wavelength),
  m_x(fixed_zero),
  m_y(fixed_zero),
  m_alt(state.NavAltitude)
{
  reset(state);
}

void 
AircraftStateFilter::reset(const AIRCRAFT_STATE& state)
{
  m_state_last = state;

  m_x = fixed_zero;
  m_y = fixed_zero;
  m_alt = state.NavAltitude;

  m_vx = fixed_zero;
  m_vy = fixed_zero;
  m_vz = fixed_zero;

  m_lpf_x.reset(fixed_zero);
  m_lpf_y.reset(fixed_zero);
  m_lpf_alt.reset(fixed_zero);
  m_df_x.reset(m_x, fixed_zero);
  m_df_y.reset(m_y, fixed_zero);
  m_df_alt.reset(m_alt, fixed_zero);
}

void 
AircraftStateFilter::update(const AIRCRAFT_STATE& state)
{
  // \todo
  // Should be able to use TrackBearing and Speed,
  // but for now, use low-level functions

  fixed dt = state.Time- m_state_last.Time;

  if (negative(dt)) {
    reset(state);
    return;
  }

  GeoVector vec(m_state_last.Location, state.Location);
  m_x+= vec.Bearing.sin()*vec.Distance;
  m_y+= vec.Bearing.cos()*vec.Distance;
  m_alt = state.NavAltitude;

  m_vx = m_lpf_x.update(m_df_x.update(m_x));
  m_vy = m_lpf_y.update(m_df_y.update(m_y));
  m_vz = m_lpf_alt.update(m_df_alt.update(m_alt));

  m_state_last = state;
}

fixed 
AircraftStateFilter::get_speed() const
{
  return hypot(m_vx, m_vy);
}

Angle 
AircraftStateFilter::get_bearing() const
{
  return Angle::radians(atan2(m_vx,m_vy)).as_bearing();
}

fixed 
AircraftStateFilter::get_climb_rate() const
{
  return m_vz;
}


bool 
AircraftStateFilter::design(const fixed cutoff_wavelength)
{
  bool ok = true;
  ok &= m_lpf_x.design(cutoff_wavelength);
  ok &= m_lpf_y.design(cutoff_wavelength);
  ok &= m_lpf_alt.design(cutoff_wavelength);
  return ok;
}

AIRCRAFT_STATE 
AircraftStateFilter::get_predicted_state(const fixed &in_time) const
{
  AIRCRAFT_STATE state_next = m_state_last;
  GeoVector vec(get_speed()*in_time, get_bearing());
  state_next.Location = vec.end_point(m_state_last.Location);
  state_next.NavAltitude = m_state_last.NavAltitude+get_climb_rate()*in_time;
  return state_next;
}
