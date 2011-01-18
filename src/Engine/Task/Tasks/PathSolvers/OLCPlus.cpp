/* Copyright_License {

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

#include "OLCPlus.hpp"

#include "Trace/Trace.hpp"
#include <assert.h>

OLCPlus::OLCPlus(const Trace &_trace, const unsigned &_handicap):
  AbstractContest(_trace, _handicap, 0)
{
  reset();
}

void
OLCPlus::reset()
{
  AbstractContest::reset();
  solution_classic.clear();
  solution_fai.clear();
  result_classic.reset();
  result_fai.reset();
}

bool
OLCPlus::solve()
{
  return save_solution();
}

void 
OLCPlus::copy_solution(TracePointVector &vec) const
{
  vec.clear();
  vec.reserve(5);
  for (unsigned i = 0; i < solution_classic.size(); ++i)
    vec.push_back(solution_classic[i]);
}

fixed 
OLCPlus::calc_distance() const 
{
  return result_classic.distance;
}

fixed 
OLCPlus::calc_score() const
{
  return apply_handicap((result_classic.distance +
                         fixed(0.3) * result_fai.distance) * fixed(0.001));
}

fixed 
OLCPlus::calc_time() const
{
  return result_classic.time;
}
