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

#ifndef XCSOAR_FORM_LIST_HPP
#define XCSOAR_FORM_LIST_HPP

#include "Form/Control.hpp"
#include "Screen/Bitmap.hpp"

/**
 * A WndListFrame implements a scrollable list control based on the
 * WindowControl class.
 */
class WndListFrame : public WindowControl {
  class ScrollBar {
    Bitmap hScrollBarBitmapTop;
    Bitmap hScrollBarBitmapMid;
    Bitmap hScrollBarBitmapBot;
    Bitmap hScrollBarBitmapFill;

  protected:
    bool dragging;
    int drag_offset;
    /** Coordinates of the ScrollBar */
    RECT rc;
    /** Coordinates of the Slider */
    RECT button;

  public:
    enum {
      SCROLLBARWIDTH_INITIAL = 32,
    };

    /** Constructor of the ScrollBar class */
    ScrollBar();

    /** Returns the width of the ScrollBar */
    int get_width() const {
      return rc.right - rc.left;
    }

    /** Returns the height of the ScrollBar */
    int get_height() const {
      return rc.bottom - rc.top;
    }

    /** Returns the height of a button of the ScrollBar */
    int get_button_height() const {
      return button.bottom - button.top;
    }

    int get_netto_height() const {
      return get_height() - 2 * get_width();
    }

    /** Returns the height of the scroll area of the ScrollBar */
    int get_scroll_height() const {
      return get_netto_height() - get_button_height();
    }

    bool defined() const {
      return get_width() > 0;
    }

    unsigned get_left(const SIZE size) const {
      return defined() ? rc.left : size.cx;
    }

    /**
     * Returns whether the given POINT is in the ScrollBar area
     * @param pt POINT to check
     * @return True if the given POINT is in the ScrollBar area, False otherwise
     */
    bool in(const POINT pt) const {
      return ::PtInRect(&rc, pt);
    }

    /**
     * Returns whether the given POINT is in the ScrollBar button area
     * @param pt POINT to check
     * @return True if the given POINT is in the ScrollBar button area,
     * False otherwise
     */
    bool in_button(const POINT pt) const {
      return ::PtInRect(&button, pt);
    }

    bool in_up_arrow(int y) const {
      return y < rc.top + get_width();
    }

    bool in_down_arrow(int y) const {
      return y >= rc.bottom - get_width();
    }

    bool above_button(int y) const {
      return y < button.top;
    }

    bool below_button(int y) const {
      return y >= button.bottom;
    }

    /**
     * Sets the size of the ScrollBar
     * (actually just the height, width is automatically set)
     * @param size Size of the Control the ScrollBar is used with
     */
    void set(const SIZE size);
    void reset();
    void set_button(unsigned size, unsigned view_size, unsigned origin);
    unsigned to_origin(unsigned size, unsigned view_size, int y) const;

    void paint(Canvas &canvas, Color fore_color) const;

    bool is_dragging() const { return dragging; }
    void drag_begin(Window *w, unsigned y);
    void drag_end(Window *w);
    unsigned drag_move(unsigned size, unsigned view_size, int y) const;
  };

public:
  typedef void (*ActivateCallback_t)(unsigned idx);
  typedef void (*CursorCallback_t)(unsigned idx);
  typedef void (*PaintItemCallback_t)(Canvas &canvas, const RECT rc,
                                      unsigned idx);

protected:
  ScrollBar scroll_bar;

  /**
   * The height of one item on the screen, in pixels.
   */
  unsigned item_height;

  /**
   * The number of items in the list.
   */
  unsigned length;

  /**
   * The index of the topmost item currently being displayed.
   */
  unsigned origin;

  /**
   * The number of items visible at a time.
   */
  unsigned items_visible;

  /**
   * The index of the selected item on the screen.
   */
  unsigned relative_cursor;

  ActivateCallback_t ActivateCallback;
  CursorCallback_t CursorCallback;
  PaintItemCallback_t PaintItemCallback;

public:
  WndListFrame(ContainerControl *Owner, const TCHAR *Name,
               int X, int Y, int Width, int Height,
               unsigned _item_height);

  void SetActivateCallback(ActivateCallback_t cb) {
    ActivateCallback = cb;
  }

  void SetCursorCallback(CursorCallback_t cb) {
    CursorCallback = cb;
  }

  void SetPaintItemCallback(PaintItemCallback_t cb) {
    PaintItemCallback = cb;
  }

  void SetItemHeight(unsigned _item_height);

  /**
   * @return the number of items in the list
   */
  unsigned GetLength() const {
    return length;
  }

  /**
   * Changes the number of items in the list.
   */
  void SetLength(unsigned n);

  unsigned GetCursorIndex() const {
    return origin + relative_cursor;
  }

  /**
   * Moves the cursor to the specified position.
   *
   * @return true if the cursor was moved to the specified position,
   * false if the position was invalid
   */
  bool SetCursorIndex(unsigned i);

  /**
   * Scrolls to the specified index.
   */
  void SetOrigin(unsigned i);

protected:
  void show_or_hide_scroll_bar();

  void EnsureVisible(unsigned i);
  void SelectItemFromScreen(int xPos, int yPos);
  void DrawScrollBar(Canvas &canvas);

  virtual bool on_resize(unsigned width, unsigned height);
  virtual bool on_mouse_down(int x, int y);
  virtual bool on_mouse_up(int x, int y);
  virtual bool on_mouse_move(int x, int y, unsigned keys);
  virtual bool on_mouse_wheel(int delta);
  virtual bool on_key_down(unsigned key_code);

  /** from class PaintWindow */
  virtual void on_paint(Canvas &canvas);
};

#endif
