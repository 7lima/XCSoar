/* Copyright_License {

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

#ifndef OLC_TRIANGLE_HPP
#define OLC_TRIANGLE_HPP

#include "ContestDijkstra.hpp"

/**
 * Specialisation of OLC Dijkstra for OLC Triangle (triangle) rules
 */
class OLCTriangle: 
  public ContestDijkstra
{
protected:
  bool is_closed;
  bool is_complete;
  unsigned first_tp;
  unsigned best_d;
  bool is_fai;

public:
  OLCTriangle(const Trace &_trace,
              const bool _is_fai=true);

  void Reset();

protected:
  virtual bool SaveSolution();

  virtual fixed CalcScore() const;
  virtual fixed CalcDistance() const;
  virtual fixed CalcTime() const;

  fixed leg_distance(unsigned i) const;

  bool path_closed() const;

  void add_edges(ScanTaskPoint curNode);

  void start_search();

  bool UpdateScore();

private:
  void add_start_edges();
};

#endif
