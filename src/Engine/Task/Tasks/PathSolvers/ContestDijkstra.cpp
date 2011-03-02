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

#include "ContestDijkstra.hpp"
#include "Task/TaskStats/ContestResult.hpp"
#include "Trace/Trace.hpp"

#include <algorithm>
#include <assert.h>
#include <limits.h>

unsigned long ContestDijkstra::count_olc_solve = 0;
unsigned long ContestDijkstra::count_olc_trace = 0;
unsigned ContestDijkstra::count_olc_size = 0;

const unsigned ContestDijkstra::max_contest_trace = 300;

// set size of reserved queue elements (may differ from Dijkstra default)
#define CONTEST_QUEUE_SIZE DIJKSTRA_QUEUE_SIZE

ContestDijkstra::ContestDijkstra(const Trace &_trace,
                                 const unsigned &_handicap,
                                 const unsigned n_legs,
                                 const unsigned finish_alt_diff):
  AbstractContest(_trace, _handicap, finish_alt_diff),
  NavDijkstra<TracePoint>(n_legs + 1),
  m_dijkstra(false, 0),
  solution_found(false)
{
  reset();
}

void
ContestDijkstra::set_weightings()
{
  assert(num_stages <= MAX_STAGES);

  std::fill(m_weightings, m_weightings + num_stages - 1, 5);
}


bool
ContestDijkstra::score(ContestResult &result)
{
  assert(num_stages <= MAX_STAGES);

  if (n_points < num_stages)
    return false;
  if (AbstractContest::score(result)) {
    solution_found = true;
    return true;
  }
  return false;
}

bool
ContestDijkstra::master_is_updated()
{
  assert(num_stages <= MAX_STAGES);

  const bool insufficient = (n_points < num_stages);
  const TracePoint& last_master = trace_master.get_last_point();

  // update trace if time and distance are greater than previous deltas,
  // or if time is greater than at least 30 seconds greater than previous

  const bool updated = 
    ((last_master.time >= last_point.time + min_delta_t_trace)
     && (last_master.approx_sq_dist(last_point) >= min_distance_trace))
    || (last_master.time >= last_point.time + max((unsigned)30, min_delta_t_trace));

  // need an update if there's insufficient data in the buffer, or if
  // the update was significant

  if (insufficient || updated) {
    last_point = last_master;
  }
  return insufficient || updated;
}


void
ContestDijkstra::clear_trace()
{
  trace_dirty = true;
  trace.clear();
  n_points = 0;
}


void
ContestDijkstra::update_trace()
{
  if (!master_is_updated()) 
    return;

  trace = trace_master.get_trace_points(max_contest_trace);
  n_points = trace.size();
  trace_dirty = true;

  count_olc_trace++;

  if (n_points<2) return;

  // find min distance and time step within this trace
  min_delta_t_trace = UINT_MAX;
  min_distance_trace = UINT_MAX;
  for (TracePointVector::const_iterator it = trace.begin();
       it+1 != trace.end(); ++it) {
    const TracePoint &p0 = *it;
    const TracePoint &p1 = *(it+1);
    min_distance_trace = min(p0.approx_sq_dist(p1), min_distance_trace);
    min_delta_t_trace = min(p1.time-p0.time, min_delta_t_trace);
  }
}


bool
ContestDijkstra::solve()
{
  if (m_dijkstra.empty()) {
    set_weightings();
    m_dijkstra.reserve(CONTEST_QUEUE_SIZE);
  }

  assert(num_stages <= MAX_STAGES);
  if (n_points < num_stages) {
    update_trace();
    return true;
  }

  if (m_dijkstra.empty()) {

    update_trace();
    if (n_points < num_stages)
      return true;

    // don't re-start search unless we have had new data appear
    if (!trace_dirty) {
      return true;
    }
    trace_dirty = false;

    m_dijkstra.restart(ScanTaskPoint(0, 0));
    start_search();
    add_start_edges();
    if (m_dijkstra.empty()) {
      return true;
    }
  }

  count_olc_solve++;
  count_olc_size = max(count_olc_size, m_dijkstra.queue_size());

  if (distance_general(m_dijkstra, 25)) {
    save_solution();
    update_trace();
    return true;
  }

  return !m_dijkstra.empty();
}

void
ContestDijkstra::reset()
{
  solution_found = false;
  m_dijkstra.clear();
  clear_trace();
  last_point.time = Trace::null_time;
  AbstractContest::reset();
  min_distance_trace = UINT_MAX;
  min_delta_t_trace = UINT_MAX;

  count_olc_solve = 0;
  count_olc_trace = 0;
  count_olc_size = 0;
}


fixed
ContestDijkstra::calc_time() const
{
  assert(num_stages <= MAX_STAGES);

  if (Trace::is_null(solution[num_stages-1]))
    return fixed_zero;
  else 
    return fixed(solution[num_stages - 1].time - solution[0].time);
}

fixed
ContestDijkstra::calc_distance() const
{
  assert(num_stages <= MAX_STAGES);

  fixed dist = fixed_zero;
  for (unsigned i = 0; i + 1 < num_stages; ++i)
    dist += solution[i].distance(solution[i + 1].get_location());

  return dist;
}

fixed
ContestDijkstra::calc_score() const
{
  assert(num_stages <= MAX_STAGES);

  fixed score = fixed_zero;
  for (unsigned i = 0; i + 1 < num_stages; ++i)
    score += get_weighting(i) *
             solution[i].distance(solution[i + 1].get_location());

  #define fixed_fifth fixed(0.0002)
  score *= fixed_fifth;

  return apply_handicap(score);
}

void
ContestDijkstra::add_start_edges()
{
  m_dijkstra.pop();

  assert(num_stages <= MAX_STAGES);
  assert(n_points > 0);

  ScanTaskPoint destination(0, 0);
  const ScanTaskPoint end(num_stages-1, n_points-1);

  for (; destination.second != n_points; ++destination.second) {
    // only add points that are valid for the finish
    solution[0] = get_point(destination);
    if (admit_candidate(end)) {
      m_dijkstra.link(destination, destination, 0);
    }
  }
}

void
ContestDijkstra::add_edges(DijkstraTaskPoint &dijkstra, const ScanTaskPoint& origin)
{
  ScanTaskPoint destination(origin.first + 1, origin.second);

  find_solution(dijkstra, origin);

  // only add last point!
  if (is_final(destination)) {
    assert(n_points > 0);
    destination.second = n_points-1;
  }

  for (; destination.second != n_points; ++destination.second) {
    if (admit_candidate(destination)) {
      const unsigned d = get_weighting(origin.first) *
                         distance(origin, destination);
      dijkstra.link(destination, origin, d);
    }
  }
}

const TracePoint &
ContestDijkstra::get_point(const ScanTaskPoint &sp) const
{
  assert(sp.second < n_points);
  return trace[sp.second];
}

unsigned
ContestDijkstra::get_weighting(const unsigned i) const
{
  assert(num_stages <= MAX_STAGES);
  assert(i+1 < num_stages);
  return m_weightings[i];
}

bool
ContestDijkstra::admit_candidate(const ScanTaskPoint &candidate) const
{
  if (!is_final(candidate))
    return true;
  else
    return finish_altitude_valid(solution[0], get_point(candidate));
}

bool
ContestDijkstra::save_solution()
{
  assert(num_stages <= MAX_STAGES);

  if (AbstractContest::save_solution()) {
    std::copy(solution, solution + num_stages, best_solution);
    return true;
  }
  return false;
}


void
ContestDijkstra::copy_solution(TracePointVector &vec) const
{
  assert(num_stages <= MAX_STAGES);

  vec.clear();
  if (solution_found) {
    assert(num_stages <= MAX_STAGES);

    vec.reserve(num_stages);
    for (unsigned i = 0; i < num_stages; ++i)
      vec.push_back(best_solution[i]);
  }
}

void 
ContestDijkstra::start_search()
{
  // nothing required by default
}




/*

OLC classic:
- start, 5 turnpoints, finish
- weightings: 1,1,1,1,0.8,0.6

FAI OLC:
- start, 2 turnpoints, finish
- if d<500km, shortest leg >= 28% d; else shortest leg >= 25%d
- considered closed if a fix is within 1km of starting point
- weightings: 1 all

OLC classic + FAI-OLC
- min finish alt is 1000m below start altitude 
- start altitude is lowest altitude before reaching start
- start time is time at which start altitude is reached
- The finish altitude is the highest altitude after reaching the finish point and before end of free flight.
- The finish time is the time at which the finish altitude is reached after the finish point is reached.

OLC league:
- start, 3 turnpoints, finish
- weightings: 1 all
- The sprint start point can not be higher than the sprint end point.
- The sprint start altitude is the altitude at the sprint start point.
- Sprint arrival height is the altitude at the sprint end point.
- The average speed (points) of each individual flight is the sum of
  the distances from sprint start, around up to three turnpoints, to the
  sprint end divided DAeC index increased by 100, multiplied by 200 and
  divided by 2.5h: [formula: Points = km / 2,5 * 200 / (Index+100)

*/
