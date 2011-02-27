/*
Copyright_License {

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

#include "Device/AndroidBluetoothPort.hpp"
#include "Android/BluetoothHelper.hpp"
#include "OS/Sleep.h"

#include <assert.h>

AndroidBluetoothPort::AndroidBluetoothPort(const TCHAR *_address,
                                           Handler &_handler)
  :Port(_handler), address(_address), helper(NULL),
   buffer(256)
{
}

AndroidBluetoothPort::~AndroidBluetoothPort()
{
  Close();
}

bool
AndroidBluetoothPort::Open()
{
  helper = BluetoothHelper::connect(Java::GetEnv(), address);
  if (helper == NULL)
    return false;

  if (!StartRxThread()) {
    Close();
    return false;
  }

  return true;
}

void
AndroidBluetoothPort::Flush(void)
{
  helper->flush(Java::GetEnv());
}

void
AndroidBluetoothPort::run()
{
  assert(helper != NULL);

  SetRxTimeout(500);

  JNIEnv *const env = Java::GetEnv();

  while (!is_stopped()) {
    int ch = helper->read(env);
    if (ch >= 0)
      ProcessChar(ch);
  }
}

bool
AndroidBluetoothPort::Close()
{
  if (helper == NULL)
    return true;

  StopRxThread();

  delete helper;
  helper = NULL;
  return true;
}

void
AndroidBluetoothPort::Write(const void *data, unsigned length)
{
  if (helper == NULL)
    return;

  JNIEnv *env = Java::GetEnv();

  const uint8_t *bytes = (const uint8_t *)data;
  for (unsigned i = 0; i < length; ++i)
    if (!helper->write(env, bytes[i]))
      return;
}

bool
AndroidBluetoothPort::StopRxThread()
{
  // Make sure the thread isn't terminating itself
  assert(!Thread::inside());

  if (helper == NULL)
    return false;

  // If the thread is not running, cancel the rest of the function
  if (!Thread::defined())
    return true;

  stop();

  Thread::join();

  return true;
}

bool
AndroidBluetoothPort::StartRxThread(void)
{
  // Make sure the thread isn't starting itself
  assert(!Thread::inside());

  // Make sure the port was opened correctly
  if (helper == NULL)
    return false;

  // Start the receive thread
  StoppableThread::start();
  return true;
}

bool
AndroidBluetoothPort::SetRxTimeout(int Timeout)
{
  if (helper == NULL)
    return false;

  helper->setReadTimeout(Java::GetEnv(), Timeout);
  return true;
}

unsigned long
AndroidBluetoothPort::SetBaudrate(unsigned long BaudRate)
{
  // XXX implement K6-Bt commands?
  return BaudRate;
}

int
AndroidBluetoothPort::Read(void *Buffer, size_t Size)
{
  JNIEnv *env = Java::GetEnv();
  int ch = helper->read(env);
  if (ch < 0)
    return -1;

  *(uint8_t *)Buffer = ch;
  return 1;
}

void
AndroidBluetoothPort::ProcessChar(char c)
{
  FifoBuffer<char>::Range range = buffer.write();
  if (range.second == 0) {
    // overflow, so reset buffer
    buffer.clear();
    return;
  }

  if (c == '\n') {
    range.first[0] = _T('\0');
    buffer.append(1);

    range = buffer.read();
    handler.LineReceived(range.first);
    buffer.clear();
  } else if (c != '\r') {
    range.first[0] = c;
    buffer.append(1);
  }
}
