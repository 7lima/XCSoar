
#include "OLCFAI.hpp"

/*
 @todo potential to use 3d convex hull to speed search
*/

OLCFAI::OLCFAI(const TracePointVector &_trace):
  ContestDijkstra(_trace, 3, 3000)
{

}


bool 
OLCFAI::finish_satisfied(const ScanTaskPoint &sp) const
{
  /// \todo implement checks for distance constraints
  // if d<500km, shortest leg >= 28% d; else shortest leg >= 25%d

  return ContestDijkstra::finish_satisfied(sp);
}


bool 
OLCFAI::admit_candidate(const ScanTaskPoint &candidate) const
{
  /// \todo implement check for closure
  /// (end point is within 1km of start)
  return ContestDijkstra::admit_candidate(candidate);
}


void 
OLCFAI::add_edges(DijkstraTaskPoint &dijkstra,
                  const ScanTaskPoint& origin) 
{
  ScanTaskPoint destination(origin.first+1, origin.second);

  if (!is_final(destination)) {
    ContestDijkstra::add_edges(dijkstra, origin);
    return;
  }

  find_solution(dijkstra, origin);

  const FlatGeoPoint prev = get_point(origin).get_flatLocation();
  const FlatGeoPoint v_close = solution[0].get_flatLocation() 
    - prev;
  
  for (; destination.second!= n_points; ++destination.second) {
    if (admit_candidate(destination)) {

      const FlatGeoPoint v_this = 
        get_point(destination).get_flatLocation() - prev;

      const unsigned d = get_weighting(origin.first)
        *v_this.projected_distance(v_close);

      dijkstra.link(destination, origin, d);
    }
  }
}
