/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

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

#include "WindZigZag.h"
#include "LogFile.hpp"
#include "Math/FastMath.h"
#include "NMEA/Info.hpp"
#include "NMEA/Derived.hpp"

#include <math.h>

#ifndef _MSC_VER
  #include <algorithm>
  using std::min;
  using std::max;
#endif

/*
 * number of points required, should be
 * equivalent to a circle to prevent
 * zigzag from dominating when used in
 * conjunction with circling drift
 * calculation.
 */
#define NUM_SAMPLES 20

/*
 * equivalent to a circle to prevent
 * zigzag from dominating when used in
 * conjunction with circling drift
 * calculation.
 */
#define SAMPLE_RATE 4.0

/*
 * minimum number of seconds between
 * recalculating estimate
 */
#define UPDATE_RATE 20

// 15 m/s = 30 knots max approx
#define V_SCALE 20.0
// resolution on V search
#define NUM_V_POINTS 41
// resolution on theta search
#define NUM_THETA_POINTS (36)

#define DEGTORAD (M_PI/180.0)

#ifndef NDEBUG
  #define DEBUG_ZIGZAG
#endif

static double
anglelimit(double ang)
{
  while (ang < -M_PI)
    ang += 2.0 * M_PI;

  while (ang > M_PI)
    ang -= 2.0 * M_PI;

  return ang;
}

static int
VtoI(double V)
{
  return min(NUM_V_POINTS - 1, iround(V * (NUM_V_POINTS - 1) / V_SCALE));
}

static double
ItoV(int i)
{
  return max(0, min(i, NUM_V_POINTS - 1)) * V_SCALE / (NUM_V_POINTS - 1);
}

/**
 * Class for points used in ZigZag wind algorithm
 */
class ZigZagPoint
{
public:
  ZigZagPoint()
  {
    time = -1;

    for (int i = 0; i < NUM_V_POINTS; i++) {
      theta_west_ok[i] = false;
    }
  }

  double V_tas;
  double V_gps;
  double theta_gps;
  double time;

  void
  Set(double t, double aV_tas, double aV_gps, double atheta_gps)
  {
    V_tas = aV_tas;
    V_gps = aV_gps;
    time = t;
    theta_gps = atheta_gps;
    CalculateThetaWEst();
  }

  double theta_west_1[NUM_V_POINTS];
  double theta_west_2[NUM_V_POINTS];
  bool theta_west_ok[NUM_V_POINTS];

  double cos_theta_gps;
  double sin_theta_gps;

private:
  int V_gps_x;
  int V_gps_y;
  int V_tas_l;

  void
  CalculateThetaWEst()
  {
    int i;
    for (i = 0; i < NUM_V_POINTS; i++) {
      double Vwest = ItoV(i);
      theta_west_ok[i] = EstimateW0(Vwest, i);
    }

    cos_theta_gps = cos(theta_gps);
    sin_theta_gps = sin(theta_gps);

    V_gps_x = iround(100 * V_gps * cos_theta_gps);
    V_gps_y = iround(100 * V_gps * sin_theta_gps);
    V_tas_l = iround(100 * V_tas);
  }

  bool
  EstimateW0(double Vwest, int i)
  {
    double west_rat = Vwest / V_tas;
    double gps_rat = V_gps / V_tas;

    if (gps_rat < 0.001)
      // speed too small
      return false;

    if (gps_rat + west_rat < 1.0)
      // wind too weak
      return false;

    if ((Vwest > V_gps) && (Vwest > V_tas))
      // wind too strong
      return false;

    if (west_rat < 0.001) {
      // wind speed too small
      theta_west_1[i] = 0;
      theta_west_2[i] = 0;
      return true;
    }

    double cosgamma = (west_rat * west_rat + gps_rat * gps_rat - 1.0)
                      / (2.0 * west_rat * gps_rat);

    if (fabs(cosgamma) > 1.0)
      return false;

    double gamma = acos(cosgamma);
    theta_west_1[i] = -anglelimit(M_PI - theta_gps - gamma);
    theta_west_2[i] = -anglelimit(M_PI - theta_gps + gamma);

    return true;
  }

public:
  int
  EstimateW1(int V_west_x, int V_west_y)
  {
    int v_tas_x = V_gps_x + V_west_x;
    int v_tas_y = V_gps_y + V_west_y;
    long vv = isqrt4(v_tas_x * v_tas_x + v_tas_y * v_tas_y);
    long vdiff = (long)V_tas_l - vv;
    int err = (1000 * max(vdiff, -vdiff)) / V_tas_l;
    // returns error in tenths of percent
    return err;
  }
};

class ZigZag
{
public:

  ZigZagPoint points[NUM_SAMPLES];

  ZigZag()
  {
    for (int i = 0; i < NUM_SAMPLES; i++) {
      points[i].time = -1;
    }

    for (int k = 0; k < NUM_THETA_POINTS; k++) {
      double theta = anglelimit(k * 2.0 * M_PI / NUM_THETA_POINTS);
      thetalist[k] = theta;
    }
  }

  void
  AddPoint(double t, double V_tas, double V_gps, double theta_gps)
  {
    // find oldest point
    double toldest = 0;
    int ioldest = 0;
    int i;

    for (i = 0; i < NUM_SAMPLES; i++) {
      if (t < points[i].time) {
        points[i].time = -1;
      }
    }

    for (i = 0; i < NUM_SAMPLES; i++) {
      if ((points[i].time < toldest) || (i == 0) || (t < points[i].time)) {
        toldest = points[i].time;
        ioldest = i;
      }
    }

    i = ioldest;
    points[i].Set(t, V_tas, V_gps, theta_gps);
  }

  double
  CheckValidity(double t)
  {
    // requires:
    // -- all points to be initialised
    // -- all points to be within last 10 minutes
    int nf = 0;
    double ctg = 0.0;
    double stg = 0.0;
    int i;

    for (i = 0; i < NUM_SAMPLES; i++) {
      if (points[i].time > 0) {
        nf++;
        if (t - points[i].time > 10 * 60) {
          // clear point so it gets filled next time
          points[i].time = -1;
          return -1;
        }
        ctg += points[i].cos_theta_gps;
        stg += points[i].sin_theta_gps;
      }
    }

    if (nf < NUM_SAMPLES)
      return -1;

    double theta_av = atan2(stg, ctg);
    double dtheta_max = 0;
    double dtheta_min = 0;

    for (i = 0; i < NUM_SAMPLES; i++) {
      double da = anglelimit(points[i].theta_gps - theta_av);

      if (da > dtheta_max)
        dtheta_max = da;

      if (da < dtheta_min)
        dtheta_min = da;
    }

    return dtheta_max - dtheta_min;
  }

  bool
  CheckSpread(double time, double error)
  {
    double spread = CheckValidity(time);
    if (spread < 0) {
      #ifdef DEBUG_ZIGZAG_A
      LogDebug(_T("zigzag time invalid\n"));
      #endif

      return false;
    }
    spread /= DEGTORAD;
    double minspread;

    minspread = 10.0 + 30.0 / (1.0 + error * error / 30.0);
    // nominal spread required is 40 degrees, smaller if large
    // error is detected

    if ((spread > 360.0) || (spread < minspread)) {
      // invalid if really circling or if not enough zig-zag

      #ifdef DEBUG_ZIGZAG_A
      LogDebug(_T("zigzag spread invalid %03.1f\n"), spread);
      #endif

      return false;
    }
    return true;
  }

private:
  double thetalist[NUM_THETA_POINTS];

  // search all points for error against theta at wind speed index j
  double
  AngleError(double theta, int j)
  {
    double de = 0;
    int nf = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
      if (points[i].theta_west_ok[j]) {
        double e1 = fabs(anglelimit(theta - points[i].theta_west_1[j]));
        double e2 = fabs(anglelimit(theta - points[i].theta_west_2[j]));
        if (e1 <= e2)
          de += e1 * e1;
        else
          de += e2 * e2;

        nf++;
      }
    }

    if (nf > 0)
      return de / nf;
    else
      return -1;
  }

  // search for theta to give best match at wind speed index i
  double
  FindBestAngle(int i, double *theta_best)
  {
    double debest = 1000000;
    bool ok = false;

    for (int k = 0; k < NUM_THETA_POINTS; k++) {
      double ae = AngleError(thetalist[k], i);
      if (ae < 0)
        continue;

      ok = true;
      if (ae < debest) {
        debest = ae;
        *theta_best = thetalist[k];
      }
    }

    return ok;
  }

  // find average error in true airspeed given wind estimate
  int
  VError(int V_west_x, int V_west_y)
  {
    int verr = 0;
    int nf = 0;

    for (int j = 0; j < NUM_SAMPLES; j++) {
      if (points[j].time >= 0) {
        verr += points[j].EstimateW1(V_west_x, V_west_y);
        nf++;
      }
    }

    if (nf > 0)
      return verr / nf;
    else
      return 1000;
  }

  bool
  UpdateSearch_Inner(double V_west, double theta_best)
  {
    // given wind speed and direction, find TAS error
    int V_west_x = iround(100 * V_west * cos(theta_best));
    int V_west_y = iround(100 * V_west * sin(theta_best));
    int verr = VError(V_west_x, V_west_y);

    // search for minimum error
    // (this is not monotonous)
    if (verr >= error_best)
      return false;

    error_best = verr;
    V_west_best = V_west;
    theta_west_best = theta_best;

    return true;
  }

  bool
  UpdateSearch(double V_west)
  {
    int i = VtoI(V_west);

    // find best angle estimate for this assumed wind speed i
    double theta = 0.0;
    if (!FindBestAngle(i, &theta)) {
      theta = theta_west_best;
    }

    return (UpdateSearch_Inner(V_west, theta)
            || UpdateSearch_Inner(V_west, theta_west_best));
  }

public:
  int error_best;
  double V_west_best;
  double theta_west_best;

  double
  StartSearch(double V_start, double theta_start)
  {
    V_west_best = V_start;
    theta_west_best = theta_start;
    error_best = 10000;
    UpdateSearch(V_start);
    V_west_best = V_start;
    theta_west_best = theta_start;
    return error_best / 10.0;
  }

  bool
  Estimate(double *V_westb, double *theta_westb, double *error)
  {
    int i;

    bool scanned[NUM_V_POINTS];
    for (i = 0; i < NUM_V_POINTS; i++) {
      scanned[i] = false;
    }

    // scan for 6 points around current best estimate.
    // if a better estimate is found, keep scanning around
    // that point, and don't repeat scans

    bool improved = false;
    bool continue_search = true;
    bool full_search = false;

    while (continue_search) {
      continue_search = false;

      int ib = VtoI(V_west_best);
      int il, ih;
      if (full_search) {
        il = 0;
        ih = NUM_V_POINTS - 1;
      } else {
        il = min(NUM_V_POINTS - 1, max(0, ib - 3));
        ih = min(NUM_V_POINTS - 1, max(0, ib + 3));
      }
      for (i = il; i <= ih; i++) {
        if (scanned[i]) {
          continue;
        } else {
          scanned[i] = true;
          // see if we can find a better estimate
          double V_west = ItoV(i);
          if (UpdateSearch(V_west)) {
            improved = true;
            continue_search = true; // earnt more search
          }
        }
      }

      if (!continue_search && !full_search && (error_best > 100)) {
        full_search = true;
        continue_search = true;
        // if no improvement and still large error,
        // try searching all speeds that haven't been checked yet.
      }
    }

    // return true if estimate was improved
    *V_westb = V_west_best;
    *theta_westb = theta_west_best;
    while (*theta_westb < 0) {
      *theta_westb += 2.0 * M_PI;
    }
    *error = error_best / 10.0;
    return improved;
  }
};

ZigZag myzigzag;

#ifdef DEBUG_ZIGZAG
#ifdef WINDOWSPC

void TestZigZag(double V_wind, double theta_wind) {
  double t, V_tas, V_gps, theta_gps, theta_glider;

  int i;
  for (i=0; i<=NUM_SAMPLES; i++) {
    t = i;
    V_tas = 20.0;
    theta_glider = sin(t*M_PI*2.0/NUM_SAMPLES)*30*DEGTORAD;
    double V_gps_x = V_tas * sin(theta_glider) - V_wind*sin(theta_wind);
    double V_gps_y = V_tas * cos(theta_glider) - V_wind*cos(theta_wind);

    V_gps = hypot(V_gps_x, V_gps_y);
    theta_gps = atan2(V_gps_x,V_gps_y);

    myzigzag.AddPoint(t, V_tas, V_gps, theta_gps);
  }

  // ok, ready to calculate
  if (myzigzag.CheckSpread(t+1, 0.0)) {
    // data is ok to make an estimate
    double V_wind_estimate=1;
    double theta_wind_estimate=0;
    double percent_error;
    percent_error = myzigzag.StartSearch(V_wind_estimate, theta_wind_estimate);
    myzigzag.Estimate(&V_wind_estimate, &theta_wind_estimate, &percent_error);

    LogDebug(_T("%2.1f %2.1f %03.0f %03.0f %2.1f # test zigzag\n"),
            V_wind,
            V_wind_estimate,
            theta_wind/DEGTORAD,
            theta_wind_estimate/DEGTORAD,
            percent_error
            );
  }
}

void TestZigZagLoop() {
  static bool first = true;
  if (!first) return;
  first = false;
  for (double V_wind=2.0; V_wind<=10.0; V_wind+= 1.0) {
    for (double theta_wind=0; theta_wind<360.0; theta_wind+= 20.0) {
      TestZigZag(V_wind, theta_wind*DEGTORAD);
    }
  }
}

#endif
#endif

static bool
WindZigZagCheckAirData(const NMEA_INFO &basic)
{
  static double tLast = -1;
  static Angle bearingLast;

  bool airdata_invalid = false;
  if (!basic.flight.Flying) {
    airdata_invalid = true;
  } else if (fabs(basic.TurnRate) > 20.0) {
    airdata_invalid = true;
#ifdef DEBUG_ZIGZAG_A
    LogDebug(_T("zigzag airdata invalid - turn rate\n"));
#endif
  } else if (fabs(basic.GroundSpeed) < 2.5) {
    airdata_invalid = true;
#ifdef DEBUG_ZIGZAG_A
    LogDebug(_T("zigzag airdata invalid - ground speed\n"));
#endif
  } else if (fabs(basic.acceleration.Gload - 1.0) > 0.3) {
    airdata_invalid = true;
#ifdef DEBUG_ZIGZAG_A
    LogDebug(_T("zigzag airdata invalid - acceleration\n"));
#endif
  }

  if (airdata_invalid) {
    tLast = basic.Time; // blackout for SAMPLE_RATE seconds
    return false;
  }

  if (basic.Time < tLast + SAMPLE_RATE &&
      (bearingLast - basic.TrackBearing).as_delta().magnitude_degrees() < 10.0) {
    return false;
  } else {
    tLast = basic.Time;
    bearingLast = basic.TrackBearing;
  }

  return true;
}

int
WindZigZagUpdate(const NMEA_INFO &basic, const DERIVED_INFO &derived,
    double *zzwindspeed, double *zzwindbearing)
{
  static double tLastEstimate = -1;

  if (!basic.AirspeedAvailable)
    return 0;

  #ifdef WINDOWSPC
  #ifdef DEBUG_ZIGZAG
  TestZigZagLoop();
  #endif
  #endif

  // TODO accuracy: correct TAS for vertical speed if dynamic pullup

  if ((basic.Time <= tLastEstimate) || (tLastEstimate == -1))
    tLastEstimate = basic.Time - UPDATE_RATE;

  if (!WindZigZagCheckAirData(basic))
    return 0;

  // ok to add a point

  myzigzag.AddPoint(basic.Time,
                    basic.TrueAirspeed, basic.GroundSpeed,
                    basic.TrackBearing.value_radians());

  #ifdef DEBUG_ZIGZAG_A
  LogDebug("%f %03.0f %03.0f %03.0f # zigpoint\n",
             basic.Time,
             basic.TrueAirspeed,
             basic.GroundSpeed,
             basic.TrackBearing);
  #endif

  // don't update wind from zigzag more often than
  // every UPDATE_RATE seconds, so it is balanced with respect
  // to circling
  if (basic.Time < tLastEstimate + UPDATE_RATE)
    return 0;

  double V_wind_estimate = basic.wind.norm;
  double theta_wind_estimate = basic.wind.bearing.value_radians();
  double percent_error = myzigzag.StartSearch(V_wind_estimate, theta_wind_estimate);

  // Check spread of zig-zag manoeuver
  if (!myzigzag.CheckSpread(basic.Time, percent_error))
    return 0;

  double v_error = percent_error * basic.TrueAirspeed / 100.0;

  if (v_error < 0.5) {
    // don't refine search if error is small

    #ifdef DEBUG_ZIGZAG
    LogDebug(_T("zigzag error small %02.0f %03.1f"), percent_error, v_error);
    #endif

    return 0;
  }

  if (myzigzag.Estimate(&V_wind_estimate, &theta_wind_estimate, &percent_error)) {
    // ok, we have made an update
    tLastEstimate = basic.Time;

    theta_wind_estimate /= DEGTORAD;

    *zzwindspeed = V_wind_estimate;
    *zzwindbearing = theta_wind_estimate;

    // calculate error quality
    int quality;

    //double pes = v_error/(V_SCALE/NUM_V_POINTS);
    //quality = iround(0.5+4.5/(1.0+percent_error*percent_error/30.0));

    quality = max(1, 5 - iround(percent_error / 2));
    if (derived.Circling) {
      quality = max(1, quality / 2); // de-value updates in circling mode
    }

    #ifdef DEBUG_ZIGZAG
    LogDebug(_T("%f %3.1f %03.0f %3.1f %03.0f %f %d # zigzag"),
             (double)basic.Time,
             V_wind_estimate,
             theta_wind_estimate,
             (double)basic.wind.norm,
             (double)basic.wind.bearing.value_degrees(),
             percent_error,
             quality);
    #endif

    return quality;
  } else {
    #ifdef DEBUG_ZIGZAG
    LogDebug(_T("zigzag estimate failed to improve"));
    #endif
  }

  return 0;
}

