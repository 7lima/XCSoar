/* Copyright_License {

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

#include "DataNode.hpp"

#include <stdio.h>

DataNode::~DataNode()
{
}

DataNode::DataNode(const tstring &node_name)
{
}


void 
DataNode::set_attribute(const tstring &name, Angle value)
{
  set_attribute(name, value.value_degrees());
}

void 
DataNode::set_attribute(const tstring &name, fixed value)
{
  TCHAR buf[100];
  _stprintf(buf, _T("%g"), (double)value);
  set_attribute(name, buf);
}

void 
DataNode::set_attribute(const tstring &name, int value)
{
  TCHAR buf[100];
  _stprintf(buf, _T("%d"), value);
  set_attribute(name, buf);
}

void 
DataNode::set_attribute(const tstring &name, unsigned value)
{
  TCHAR buf[100];
  _stprintf(buf, _T("%d"), value);
  set_attribute(name, buf);
}

void 
DataNode::set_attribute(const tstring &name, bool &value)
{
  TCHAR buf[100];
  _stprintf(buf, _T("%d"), (int)value);
  set_attribute(name, buf);
}

bool 
DataNode::get_attribute(const tstring &name, Angle &value) const 
{
  bool retval;
  fixed v = value.value_degrees();
  retval = get_attribute(name, v);
  value = Angle::degrees(v);
  return retval;
}


bool 
DataNode::get_attribute(const tstring &name, fixed &value) const 
{
  tstring val;
  if (get_attribute(name, val)) {
    value = (fixed)_tcstod(val.c_str(), NULL);
    return true;
  } else {
    return false;
  }
}

bool 
DataNode::get_attribute(const tstring &name, int &value) const
{
  tstring val;
  if (get_attribute(name, val)) {
    value = _tcstol(val.c_str(), NULL, 0);
    return true;
  } else {
    return false;
  }
}

bool 
DataNode::get_attribute(const tstring &name, unsigned &value) const
{
  tstring val;
  if (get_attribute(name, val)) {
    value = _tcstol(val.c_str(), NULL, 0);
    return true;
  } else {
    return false;
  }
}

bool 
DataNode::get_attribute(const tstring &name, bool &value) const
{
  tstring val;
  if (get_attribute(name, val)) {
    value = (_tcstol(val.c_str(), NULL, 0)>0);
    return true;
  } else {
    return false;
  }
}

