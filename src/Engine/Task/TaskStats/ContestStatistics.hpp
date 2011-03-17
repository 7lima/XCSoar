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
#ifndef CONTEST_STATISTICS_HPP
#define CONTEST_STATISTICS_HPP

#include "Navigation/TracePoint.hpp"
#include "ContestResult.hpp"

struct ContestStatistics {

  void reset() {
    for (unsigned i = 0; i < 3; ++i) {
      solution[i].clear();
      result[i].reset();
    }
  }

  /**
   * Retrieve contest solution vector
   *
   * @param solution_index -1 for best, otherwise index of solution
   *
   * @return Vector of trace points selected for Contest
   */
  gcc_pure
  const TracePointVector& get_contest_solution(const int solution_index=-1) const {
    return solution[get_index_best(solution_index)];
  }

  gcc_pure
  const ContestResult& get_contest_result(const int solution_index=-1) const {
    return result[get_index_best(solution_index)];
  }

  int
  get_index_best(const int solution_index) const
  {
    if (solution_index >= 0)
      return solution_index;

    // Search for best solution by score
    fixed best = fixed_zero;
    int i_best = 0;
    for (int i = 0; i < 3; ++i) {
      if (result[i].defined() && (result[i].score > best)) {
        // Better scored solution found
        i_best = i;
        best = result[i].score;
      }
    }

    // Return index to the best solution
    return i_best;
  }

  ContestResult result[3];
  TracePointVector solution[3];
};

#endif
