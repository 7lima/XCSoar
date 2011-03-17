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
#ifndef ONLINE_CONTEST_HPP
#define ONLINE_CONTEST_HPP

#include "Navigation/Aircraft.hpp"
#include "PathSolvers/OLCSprint.hpp"
#include "PathSolvers/OLCFAI.hpp"
#include "PathSolvers/OLCClassic.hpp"
#include "PathSolvers/OLCLeague.hpp"
#include "PathSolvers/OLCPlus.hpp"
#include "PathSolvers/XContestFree.hpp"
#include "PathSolvers/XContestTriangle.hpp"
#include "PathSolvers/OLCSISAT.hpp"
#include "PathSolvers/Contests.hpp"
#include "Task/TaskStats/ContestStatistics.hpp"

class Trace;

/**
 * Special task holder for Online Contest calculations
 */
class ContestManager {
public:
  friend class PrintHelper;

  /** 
   * Base constructor.
   * 
   * @param _contest Contests that shall be used
   * @param _handicap Contest handicap factor
   * @param trace_full Trace object reference
   * containing full flight history for scanning
   * @param trace_sprint Trace object reference
   * containing 2.5 hour flight history for scanning
   */
  ContestManager(const Contests _contest,
                 const unsigned &_handicap,
                 const Trace &trace_full,
                 const Trace &trace_sprint);

  void set_contest(Contests _contest) {
    contest = _contest;
  }

  /**
   * Update internal states (non-essential) for housework,
   * or where functions are slow and would cause loss to real-time performance.
   *
   * @param exhaustive true to find the final solution, false stops
   * after a number of iterations (incremental search)
   * @return True if internal state changed
   */
  bool update_idle(bool exhaustive=false);

  bool solve_exhaustive() {
    return update_idle(true);
  }

  /** 
   * Reset the task (as if never flown)
   */
  void reset();

  const ContestStatistics& get_stats() const {
    return stats;
  }

private:
  Contests contest;

  ContestStatistics stats;

  const Trace &trace_full;
  const Trace &trace_sprint;

  bool run_contest(AbstractContest& the_contest, 
                   ContestResult &contest_result,
                   TracePointVector &contest_solution,
                   bool exhaustive);

  OLCSprint olc_sprint;
  OLCFAI olc_fai;
  OLCClassic olc_classic;
  OLCLeague olc_league;
  OLCPlus olc_plus;
  XContestFree olc_xcontest_free;
  XContestTriangle olc_xcontest_triangle;
  XContestFree olc_dhvxc_free;
  XContestTriangle olc_dhvxc_triangle;
  OLCSISAT olc_sisat;
};

#endif
