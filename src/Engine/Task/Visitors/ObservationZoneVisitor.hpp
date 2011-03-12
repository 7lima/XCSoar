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
#ifndef OBSERVATION_POINT_VISITOR_HPP
#define OBSERVATION_POINT_VISITOR_HPP

class ObservationZonePoint;
class FAISectorZone;
class SectorZone;
class LineSectorZone;
class CylinderZone;
class KeyholeZone;
class BGAFixedCourseZone;
class BGAEnhancedOptionZone;
class BGAStartSectorZone;
class AnnularSectorZone;

/**
 * Generic const visitor for observation zones (for double-dispatch)
 */
class ObservationZoneConstVisitor {
protected:
  virtual void Visit(const FAISectorZone &z) = 0;
  virtual void Visit(const SectorZone &z) = 0;
  virtual void Visit(const LineSectorZone &z) = 0;
  virtual void Visit(const CylinderZone &z) = 0;
  virtual void Visit(const KeyholeZone &z) = 0;
  virtual void Visit(const BGAFixedCourseZone &z) = 0;
  virtual void Visit(const BGAEnhancedOptionZone &z) = 0;
  virtual void Visit(const BGAStartSectorZone &z) = 0;
  virtual void Visit(const AnnularSectorZone &z) = 0;

public:
  void Visit(const ObservationZonePoint &ozp);
};

/**
 * Generic visitor for observation zones (for double-dispatch)
 */
class ObservationZoneVisitor {
protected:
  virtual void Visit(FAISectorZone &z) = 0;
  virtual void Visit(SectorZone &z) = 0;
  virtual void Visit(LineSectorZone &z) = 0;
  virtual void Visit(CylinderZone &z) = 0;
  virtual void Visit(KeyholeZone &z) = 0;
  virtual void Visit(BGAFixedCourseZone &z) = 0;
  virtual void Visit(BGAEnhancedOptionZone &z) = 0;
  virtual void Visit(BGAStartSectorZone &z) = 0;
  virtual void Visit(AnnularSectorZone &z) = 0;

public:
  void Visit(ObservationZonePoint &ozp);
};

#endif
