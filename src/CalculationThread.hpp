/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
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

#ifndef XCSOAR_CALCULATION_THREAD_HPP
#define XCSOAR_CALCULATION_THREAD_HPP

#include "Thread/WorkerThread.hpp"
#include "Thread/Trigger.hpp"

class GlideComputer;

/**
 * The CalculationThread handles all expensive calculations
 * that should not be done directly in the device thread.
 * Data transfer is handled by a blackboard system.
 */
class CalculationThread : public WorkerThread {
  /** The gps_trigger is used only when new GPS data is available */
  Trigger gps_trigger;

  /** Pointer to the GlideComputer that should be used */
  GlideComputer &glide_computer;

public:
  CalculationThread(GlideComputer &_glide_computer);

  /** Triggers the data_trigger */
  void trigger_data() {
    trigger();
  }

  /** Triggers the gps_trigger */
  void trigger_gps() {
    gps_trigger.trigger();
  }

protected:
  virtual void tick();
};

#endif
