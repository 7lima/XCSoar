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

#ifndef XCSOAR_IO_BUFFERED_SOURCE_HPP
#define XCSOAR_IO_BUFFERED_SOURCE_HPP

#include "FifoBuffer.hpp"
#include "Source.hpp"

template<class T>
class BufferedSource : public Source<T> {
public:
  typedef std::pair<T*, unsigned> Range;

private:
  FifoBuffer<T> buffer;
  long position;

public:
  BufferedSource(unsigned size):buffer(size), position(0) {}

protected:
  virtual unsigned read(T *p, unsigned n) = 0;

public:
  virtual Range read() {
    Range r = buffer.write();
    if (r.second > 0) {
      unsigned n = read(r.first, r.second);
      buffer.append(n);
    }

    return buffer.read();
  }

  virtual void consume(unsigned n) {
    buffer.consume(n);
    position += n;
  }

  virtual long tell() const {
    return position;
  }
};

#endif
