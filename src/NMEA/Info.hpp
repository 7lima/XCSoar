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

#ifndef XCSOAR_NMEA_INFO_H
#define XCSOAR_NMEA_INFO_H

#include "Navigation/GeoPoint.hpp"
#include "Navigation/Aircraft.hpp"
#include "Atmosphere/Pressure.hpp"
#include "FLARM/State.hpp"
#include "Sizes.h"

struct SWITCH_INFO
{
  bool AirbrakeLocked;
  bool FlapPositive;
  bool FlapNeutral;
  bool FlapNegative;
  bool GearExtended;
  bool Acknowledge;
  bool Repeat;
  bool SpeedCommand;
  bool UserSwitchUp;
  bool UserSwitchMiddle;
  bool UserSwitchDown;
  bool VarioCircling;
  bool FlapLanding;
  // bool Stall;
};

struct GPS_STATE
{
  //############
  //   Status
  //############

  /**
   * Is a GPS unit connected?
   *
   * 0 = not connected
   * 1 = connected, waiting for fix
   * 2 = connected, fix found
   */
  unsigned Connected;

  /** GPS fix not valid */
  int NAVWarning;

  /** Number of satellites used for gps fix */
  int SatellitesUsed;

  /** GPS Satellite ids */
  int SatelliteIDs[MAXSATELLITES];

  /** Is the GPS unit moving? (Speed > 2.0) */
  bool MovementDetected;

  /** Is XCSoar in replay mode? */
  bool Replay;
};

struct ACCELERATION_STATE
{
  //##################
  //   Acceleration
  //##################

  /** Estimated bank angle */
  fixed BankAngle;
  /** Estimated pitch angle */
  fixed PitchAngle;

  /**
   * Is G-load information available?
   * @see Gload
   * @see AccelX
   * @see AccelY
   */
  bool AccelerationAvailable;

  /**
   * G-Load information of external device in X-direction (if available)
   * @see AccelerationAvailable
   */
  double AccelX;
  /**
   * G-Load information of external device in Y-direction (if available)
   * @see AccelerationAvailable
   */
  double AccelZ;
};


/**
 * A struct that holds all the parsed data read from the connected devices
 */
struct NMEA_INFO: 
  public GPS_STATE,
  public AIRCRAFT_STATE,
  public FLARM_STATE,
  public ACCELERATION_STATE
{

  /** Bearing including wind factor */
  fixed Heading;

  /** Turn rate based on heading (including wind) */
  fixed TurnRateWind;

  /** Turn rate based on track */
  fixed TurnRate;

  /** Estimated track bearing at next time step @author JMW */
  fixed NextTrackBearing;

  //############
  //   Speeds
  //############

  /**
   * Is air speed information available?
   * If not, will be estimated from ground speed and wind estimate
   * @see TrueAirspeed in Aircraft
   */
  bool AirspeedAvailable;

  fixed TrueAirspeedEstimated;

  //##############
  //   Altitude
  //##############

  fixed GPSAltitude; /**< GPS altitude AMSL (m) */

  /**
   * Is a barometric altitude available?
   * @see BaroAltitude
   */
  bool BaroAltitudeAvailable;
  /**
   * Barometric altitude (if available)
   * @see BaroAltitudeAvailable
   * @see Altitude
   */
  fixed BaroAltitude;

  /** Energy height excess to slow to best glide speed @author JMW */
  fixed EnergyHeight;

  /** Nav Altitude + Energy height (m) */
  fixed TEAltitude;

  /** Height above working band/safety (m) */
  fixed working_band_height;

  /**
   * Troposhere atmosphere model for QNH correction
   */
  AtmosphericPressure pressure;

  //##########
  //   Time
  //##########

  /** GPS time (hours) */
  int Hour;
  /** GPS time (minutes) */
  int Minute;
  /**< GPS time (seconds) */
  int Second;
  /**< GPS date (month) */
  int Month;
  /**< GPS date (day) */
  int Day;
  /**< GPS date (year) */
  int Year;

  //###########
  //   Vario
  //###########

  fixed GliderSinkRate;

  /** GPS-based vario */
  fixed GPSVario;
  /** GPS-based vario including energy height */
  fixed GPSVarioTE;

  /**
   * Is an external vario signal available?
   * @see Vario
   */
  bool VarioAvailable;

  /**
   * Is an external netto vario signal available?
   * @see NettoVario
   */
  bool NettoVarioAvailable;

  //##############
  //   Settings
  //##############

  /** MacCready value of external device (if available) */
  double MacCready;

  /** Ballast information of external device (if available) */
  double Ballast;

  /** Bugs information of external device (if available) */
  double Bugs;

  //################
  //   Atmosphere
  //################

  /**
   * Is external wind information available?
   * @see ExternalWindSpeed
   * @see ExternalWindDirection
   */
  bool ExternalWindAvailable;

  /**
   * Is temperature information available?
   * @see OutsideAirTemperature
   */
  bool TemperatureAvailable;
  /**
   * Temperature of outside air (if available)
   * @see TemperatureAvailable
   */
  double OutsideAirTemperature;

  /**
   * Is humidity information available?
   * @see RelativeHumidity
   */
  bool HumidityAvailable;
  /**
   * Humidity of outside air (if available)
   * @see HumidityAvailable
   */
  double RelativeHumidity;

  //###########
  //   Other
  //###########

  /** Battery supply voltage information (if available) */
  double SupplyBatteryVoltage;

  /** Switch state of the user inputs */
  SWITCH_INFO SwitchState;

  double StallRatio;

  /**
   * Returns the barometric altitude, and falls back to the GPS
   * altitude.
   */
  fixed GetAnyAltitude() const {
    return BaroAltitudeAvailable
      ? BaroAltitude
      : GPSAltitude;
  }
};

#endif
