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

#ifndef XCSOAR_DEVICE_PORT_HPP
#define XCSOAR_DEVICE_PORT_HPP

#include "FifoBuffer.hpp"

#include <Thread/Thread.hpp>

#include <windows.h>

#define NMEA_BUF_SIZE 100

// Forward declaration
struct DeviceDescriptor;

/**
 * Generic ComPort thread handler class
 */
class ComPort : protected Thread
{
public:
  /**
   * Interface with callbacks for the #ComPort class.
   */
  class Handler {
  public:
    virtual void LineReceived(const TCHAR *line) = 0;
  };

protected:
  unsigned baud_rate;

  Handler &handler;

public:
  /**
   * Creates a new serial port (RS-232) object, but does not open it
   * yet.
   *
   * @param path the path of the virtual file to open, e.g. "COM1:"
   * @param _baud_rate the speed of the port
   * @param _handler the callback object for input received on the
   * port
   */
  ComPort(const TCHAR *path, unsigned _baud_rate, Handler &_handler);

  ~ComPort()
  {
    Close();
  }

  void PutChar(BYTE);
  void WriteString(const TCHAR *);
  void Flush();

  bool Open();
  bool Close();

  int SetRxTimeout(int);
  unsigned long SetBaudrate(unsigned long);

  bool StopRxThread();
  bool StartRxThread();
  void ProcessChar(char);

  int GetChar();
  int Read(void *Buffer, size_t Size);

protected:
  virtual void run();

private:
#ifdef HAVE_POSIX
  int fd;
#else /* !HAVE_POSIX */
  HANDLE hPort;
  DWORD dwMask;
#endif /* !HAVE_POSIX */

  TCHAR sPortName[64];
  bool CloseThread;

  FifoBuffer<TCHAR> buffer;
};

#endif
