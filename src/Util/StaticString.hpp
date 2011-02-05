/*
 * Copyright (C) 2010 Max Kellermann <max@duempel.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef XCSOAR_STATIC_STRING_HPP
#define XCSOAR_STATIC_STRING_HPP

#include <assert.h>
#include <string.h>
#include <tchar.h>
#include <algorithm>

/**
 * A string with a maximum size known at compile time.
 */
template<unsigned max>
class StaticString {
public:
  static const unsigned MAX_SIZE = max;
  static const TCHAR SENTINEL = _T('\0');

protected:
  TCHAR data[max];

public:
  StaticString() {}
  explicit StaticString(const TCHAR *value) {
    set(value);
  }

  unsigned length() const {
    return _tcslen(data);
  }

  bool empty() const {
    return data[0] == SENTINEL;
  }

  bool full() const {
    return length() >= MAX_SIZE - 1;
  }

  void clear() {
    data[0] = SENTINEL;
  }

  bool equals(const TCHAR *other) const {
    assert(other != NULL);

    return _tcscmp(data, other) == 0;
  }

  /**
   * Returns one character.  No bounds checking.
   */
  TCHAR operator[](unsigned i) const {
    assert(i <= length());

    return data[i];
  }

  const TCHAR *begin() const {
    return data;
  }

  const TCHAR *end() const {
    return data + length();
  }

  TCHAR last() const {
    assert(length() > 0);

    return data[length() - 1];
  }

  const TCHAR *get() const {
    return data;
  }

  void set(const TCHAR *new_value) {
    assert(new_value != NULL);

    _tcsncpy(data, new_value, MAX_SIZE - 1);
    data[MAX_SIZE - 1] = SENTINEL;
  }

  const TCHAR *c_str() const {
    return get();
  }

  operator const TCHAR *() const {
    return get();
  }

  void operator =(const TCHAR *new_value) {
    return set(new_value);
  }

  /**
   * Don't use - not thread safe.
   */
  TCHAR *first_token(const TCHAR *delim) {
    return _tcstok(data, delim);
  }

  /**
   * Don't use - not thread safe.
   */
  TCHAR *next_token(const TCHAR *delim) {
    return _tcstok(NULL, delim);
  }
};

#endif
