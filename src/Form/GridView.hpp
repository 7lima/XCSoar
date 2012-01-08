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

#ifndef XCSOAR_FORM_GRIDVIEW_HPP
#define XCSOAR_FORM_GRIDVIEW_HPP

#include "Screen/ContainerWindow.hpp"
#include "Util/StaticArray.hpp"
#include "Look/DialogLook.hpp"

class GridView : public ContainerWindow {

public:
  enum
  {
    MAX_ITEMS = 32,
  };

  enum class Direction
  {
    LEFT,
    RIGHT,
    UP,
    DOWN
  };

protected:
  StaticArray<Window *, MAX_ITEMS> items;
  UPixelScalar columnWidth;
  UPixelScalar rowHeight;
  UPixelScalar horizontalSpacing;
  UPixelScalar verticalSpacing;
  unsigned numColumns;
  unsigned numRows;
  unsigned currentPage;
  const DialogLook &look;

public:
  GridView(ContainerWindow &parent,
           PixelScalar x, PixelScalar y,
           UPixelScalar width, UPixelScalar height,
           const DialogLook &look,
           const WindowStyle style=WindowStyle());

  void SetItems(const TrivialArray<Window *, MAX_ITEMS> &items);
  UPixelScalar GetColumnWidth() const;
  UPixelScalar GetRowHeight() const;
  unsigned GetCurrentPage() const;
  unsigned GetNumColumns() const;
  unsigned GetNumRows() const;
  void SetNumRows(unsigned numRows);
  signed GetIndexOfItemInFocus();
  void MoveFocus(Direction direction);
  void ShowNextPage(Direction direction = Direction::RIGHT);
  void RefreshLayout();

protected:
  signed GetNextItemIndex(unsigned currIndex, Direction direction);
  signed GetNextEnabledItemIndex(signed currIndex, Direction direction);

#ifdef USE_GDI
  virtual void OnPaint(Canvas &canvas);
#endif
};

#endif
