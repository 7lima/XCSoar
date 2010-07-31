#ifndef TRACE_POINT_HPP
#define TRACE_POINT_HPP

#include "SearchPoint.hpp"
#include "Aircraft.hpp"
#include "Compiler.h"

#include <vector>

/**
 * Class for points used in traces (snail trail, OLC scans)
 * Internally, keeps track of predecessors as a kind of a linked-list
 */
class TracePoint:
  public SearchPoint,
  public ALTITUDE_STATE,
  public VARIO_STATE
{
public:
/** 
 * Dummy constructor for null object
 * 
 * @return Null object
 */
  TracePoint():time(0-1),last_time(0-1) {};

/** 
 * Constructor for actual trace points
 * 
 * @param state State of aircraft
 * @param tp Projection used internally
 * 
 * @return Initialised object
 */  
  TracePoint(const AIRCRAFT_STATE &state, const TaskProjection& tp);

  unsigned time; /**< Time of sample */
  unsigned last_time; /**< Time of sample prior to this */
  fixed drift_factor; /**< Thermal drift factor: 1 indicates drift
                       * rate equal to wind speed, 0 indicates no
                       * drift.*/

  /** 
   * Calculate approximate squared (flat projected) distance between this point
   * and another
   * 
   * @param tp Point to calculate distance to
   * 
   * @return Approximate squared distance
   */
  gcc_pure
  unsigned approx_sq_dist(const TracePoint& tp) const {
    return dsqr(get_flatLocation().Longitude-tp.get_flatLocation().Longitude)+
      dsqr(get_flatLocation().Latitude-tp.get_flatLocation().Latitude);
  }

  /** 
   * Calculate approximate (flat projected) distance between this point
   * and another
   * 
   * @param tp Point to calculate distance to
   * 
   * @return Approximate distance
   */
  gcc_pure
  unsigned approx_dist(const TracePoint& tp) const {
    return (unsigned)sqrt(approx_sq_dist(tp));
  }

  /** 
   * Calculate time step of this point from previous
   * 
   * @return Time step (s)
   */
  gcc_pure
  unsigned dt() const {
    return time-last_time;
  }

  /**
   * Function object used to provide access to coordinate values by kd-tree
   */
  struct kd_get_location {    
    typedef int result_type; /**< type of returned value */
    /**
     * Retrieve coordinate value from object given coordinate index
     * @param d WaypointEnvelope object
     * @param k index of coordinate
     *
     * @return Coordinate value
     */
    int operator() ( const TracePoint &d, const unsigned k) const {
      switch(k) {
      case 0:
        return d.get_flatLocation().Longitude;
      case 1:
        return d.get_flatLocation().Latitude;
      };
      return 0; 
    };
  };

  /**
   * Structure for STL sorting by time
   */
  struct time_sort {
    /** 
     * Comparison operator
     * 
     * @return True if s1 is earlier than s2
     */
    bool operator()(const TracePoint& s1, const TracePoint& s2) {
      return s1.time < s2.time;
    }
  };

  /** 
   * Test match based on time (since time of a sample must be unique)
   * 
   * @param a Point to compare to
   * 
   * @return True if time matches
   */
  bool operator==(TracePoint const& a) { 
    return time == a.time; 
  }

private:
  gcc_const
  static inline unsigned dsqr(const int d) {
    return d*d;
  }
};

typedef std::vector<TracePoint> TracePointVector;

class TaskProjection;

gcc_pure
TaskProjection get_bounds(const TracePointVector& vec,
                          const GEOPOINT &fallback_location);

#endif

