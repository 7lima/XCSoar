#include "TracePoint.hpp"
#include "TaskProjection.hpp"

TracePoint::TracePoint(const AIRCRAFT_STATE &state, const TaskProjection& tp):
    SearchPoint(state.Location, tp, true),
    ALTITUDE_STATE(state),
    VARIO_STATE(state),
    time((int)state.Time),
    drift_factor(state.thermal_drift_factor())
{

}

TaskProjection get_bounds(const TracePointVector& trace,
                          const GEOPOINT &fallback_location) 
{
  TaskProjection task_projection;

  task_projection.reset(fallback_location);
  for (TracePointVector::const_iterator it = trace.begin(); 
       it != trace.end(); ++it) {
    task_projection.scan_location(it->get_location());
  }
  task_projection.update_fast();
  return task_projection;
}
