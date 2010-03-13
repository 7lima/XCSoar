#ifndef AIRSPACE_SOONEST_SORT_HPP
#define AIRSPACE_SOONEST_SORT_HPP

#include "AirspaceNearestSort.hpp"
#include "AirspaceAircraftPerformance.hpp"

/**
 *  Class to sort nearest airspaces according to soonest intercept
 *  possible according to an AircraftPerformanceModel.
 */
class AirspaceSoonestSort:
  public AirspaceNearestSort
{
public:
/** 
 * Constructor
 * 
 * @param state State of aircraft
 * @param perf Aircraft performance model
 * @param max_time Maximum time allowed for results
 * @param condition Additional condition to be placed on queries (default always true)
 * 
 * @return Initialised object
 */
  AirspaceSoonestSort(const AIRCRAFT_STATE &state,
                      const AirspaceAircraftPerformance &perf,
                      const fixed max_time = fixed(1.0e6),
                      const AirspacePredicate &condition=AirspacePredicate::always_true):
    AirspaceNearestSort(state.Location, condition),
    m_state(state),
    m_perf(perf),
    m_max_time(max_time) {};

/** 
 * Compute intercept solution
 * 
 * @param a Airspace to solve intercept for
 * 
 * @return Intercept solution (whether valid or otherwise)
 */
  virtual AirspaceInterceptSolution solve_intercept(const AbstractAirspace &a) const;

/** 
 * Calculate metric for intercept solution.  In this case, returns the
 * time to intercept if valid.
 * 
 * @param ais solution to compute metric from
 * 
 * @return Value of metric (smaller is better)
 */
  virtual fixed metric(const AirspaceInterceptSolution& ais) const;

/** 
 * Convenience method, calls find_nearest(airspaces, range) with range
 * set according to limit of performance model.
 * 
 * @param airspaces Airspaces to search
 * 
 * @return Soonest arrival time airspace
 */
  const AbstractAirspace* find_nearest(const AirspacesInterface &airspaces);

protected:

private:
  const AIRCRAFT_STATE &m_state;
  const AirspaceAircraftPerformance &m_perf;
  const fixed& m_max_time;
};


#endif
