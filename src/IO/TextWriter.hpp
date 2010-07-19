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

#ifndef XCSOAR_IO_TEXT_WRITER_HPP
#define XCSOAR_IO_TEXT_WRITER_HPP

#include "Compiler.h"
#include "Util/ReusableArray.hpp"

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>

#ifdef _UNICODE
#include <tchar.h>
#endif

/**
 * Writer for an UTF-8 text file with native line endings.  All "const
 * char *" arguments must be either valid UTF-8 or 7 bit ASCII.
 *
 * The end-of-line marker is platform specific, and must not be passed
 * to this class.  Use the methods newline(), writeln() or printfln()
 * to finish a line.
 */
class TextWriter {
private:
  FILE *file;

#ifdef _UNICODE
  ReusableArray<TCHAR> format_buffer;
  ReusableArray<char> convert_buffer;
#endif

public:
  /**
   * Creates a new text file.  Truncates the old file if it exists,
   * unless the parameter "append" is true.
   */
  TextWriter(const char *path, bool append=false);

#ifdef _UNICODE
  TextWriter(const TCHAR *path, bool append=false);
#endif

  ~TextWriter();

  /**
   * Returns true if opening the file has failed.  This must be
   * checked before calling any other method.
   */
  bool error() const {
    return file == NULL;
  }

  /**
   * Ensure that all pending writes have been passed to the operating
   * system.  This does not guarantee that these have been written to
   * the physical device; they might still reside in the filesystem
   * cache.
   */
  bool flush() {
    return fflush(file) == 0;
  }

  /**
   * Write one character.
   */
  void write(char ch) {
    putc(ch, file);
  }

  /**
   * Finish the current line.
   */
  bool newline() {
#ifndef HAVE_POSIX
    write('\r');
#endif
    write('\n');
    return true;
  }

  /**
   * Write a chunk of text to the file.
   */
  bool write(const char *s, size_t length) {
    return fwrite(s, sizeof(*s), length, file) == length;
  }

  /**
   * Write a string to the file.
   */
  bool write(const char *s) {
    return fputs(s, file) >= 0;
  }

  /**
   * Write a string to the file, and finish the current line.
   */
  bool writeln(const char *s) {
    return write(s) && newline();
  }

#ifdef _UNICODE
  bool write(const TCHAR *s, size_t length);

  bool write(const TCHAR *s);

  bool writeln(const TCHAR *s) {
    return write(s) && newline();
  }
#endif

  void vprintf(const char *fmt, va_list ap) {
    vfprintf(file, fmt, ap);
  }

  void vprintfln(const char *fmt, va_list ap) {
    vprintf(fmt, ap);
    newline();
  }

#ifdef _UNICODE
  bool vprintf(const TCHAR *fmt, va_list ap);

  bool vprintfln(const TCHAR *fmt, va_list ap) {
    return vprintf(fmt, ap) && newline();
  }
#endif

  gcc_printf(2, 3)
  void printf(const char *s, ...);

  gcc_printf(2, 3)
  void printfln(const char *s, ...);

#ifdef _UNICODE
  void printf(const TCHAR *s, ...);
  void printfln(const TCHAR *s, ...);
#endif
};

#endif
