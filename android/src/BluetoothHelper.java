/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

package org.xcsoar;

import java.util.UUID;
import java.util.Set;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import android.util.Log;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;

/**
 * A utility class which wraps the Java API into an easier API for the
 * C++ code.
 */
final class BluetoothHelper extends AbstractAndroidPort {
  private static final String TAG = "XCSoar";
  private static final UUID THE_UUID =
        UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

  private static final BluetoothAdapter adapter;

  static {
    BluetoothAdapter _adapter;
    try {
      _adapter = BluetoothAdapter.getDefaultAdapter();
    } catch (Exception e) {
      Log.e(TAG, "BluetoothAdapter.getDefaultAdapter() failed: " +
            e.getMessage());
      _adapter = null;
    }

    adapter = _adapter;
  }

  public static void Initialize() {
  }

  BluetoothSocket socket;

  BluetoothHelper(BluetoothSocket _socket)
    throws IOException {
    super(_socket.getRemoteDevice().getAddress());
    socket = _socket;

    super.set(socket.getInputStream(), socket.getOutputStream());
  }

  public void close() {
    super.close();

    try {
      socket.close();
    } catch (IOException e) {
      /* ignore... what else should we do if closing the socket
         fails? */
    }
  }

  public int getBaudRate() {
    return 19200;
  }

  public boolean setBaudRate(int baudRate) {
    return true;
  }

  public static String[] list() {
    if (adapter == null)
      return null;

    Set<BluetoothDevice> devices = adapter.getBondedDevices();
    if (devices == null)
      return null;

    String[] addresses = new String[devices.size() * 2];
    int n = 0;
    for (BluetoothDevice device: devices) {
      addresses[n++] = device.getAddress();
      addresses[n++] = device.getName();
    }

    return addresses;
  }

  public static AndroidPort connect(String address) {
    if (adapter == null)
      return null;

    try {
      BluetoothDevice device = adapter.getRemoteDevice(address);
      if (device == null)
        return null;

      BluetoothSocket socket =
        device.createRfcommSocketToServiceRecord(THE_UUID);
      socket.connect();
      return new BluetoothHelper(socket);
    } catch (Exception e) {
      Log.e(TAG, "Failed to connect to Bluetooth: " + e.getMessage());
      return null;
    }
  }
}
