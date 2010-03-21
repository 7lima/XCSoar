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
#ifndef GENERIC_VISITOR_HPP
#define GENERIC_VISITOR_HPP

#include <vector>

// Adapted from
// "Modern C++ design: generic programming and design patterns applied"
// By Andrei Alexandrescu
//
// Adapted to handle access via libkdtree methods,
//  and for const-ness of visitable items.
// Also added utility functions

/**
 * Base class for all visitors
 */
class BaseVisitor
{
public:
  virtual ~BaseVisitor() {}
};

/**
 * Generic visitor
 */
template <class T, typename R = void>
class ConstVisitor
{
public:
  /** Return type, available for clients */
  typedef R ReturnType;

  /**
   * Abstract visit method; this prototype method is called
   * on accepting instances.
   *
   * @return Return value of visitor
   */
  virtual ReturnType Visit(const T&) = 0;
};

template <class T, typename R = void>
class Visitor
{
public:
  /** Return type, available for clients */
  typedef R ReturnType;

  /**
   * Abstract visit method; this prototype method is called
   * on accepting instances.
   *
   * @return Return value of visitor
   */
  virtual ReturnType Visit(T&) = 0;
};

/**
 * Special visitor that can visit items within a kd-tree
 */
template <class T>
class TreeVisitor: public BaseVisitor
{
public:
  /** Vector of tree items */
  typedef std::vector< T > TVector;
  /** Iterator for tree items */
  typedef typename TVector::const_iterator TVectorIterator;

  /**
   * Utility function to call visitor on all items in a vector
   *
   * @param v Vector of items to be visited
   */
  void
  for_each(const TVector &v)
  {
    for (TVectorIterator i = v.begin(); i != v.end(); i++) {
      i->CAccept(*this);
    }
  }

  /**
   * Utility accessor to visit an item by calling the visitor with () operator
   * as used by libkdtree++
   */
  void
  operator()(const T& as)
  {
    as.CAccept(*this);
  }
};


/**
 * Class from which to inherit for a class to be visitable
 */
template <typename R = void>
class BaseConstVisitable
{
public:
  /** Accessible to clients */
  typedef R ReturnType;

  /**
   * Destructor
   */
  virtual ~BaseConstVisitable() {}

  /**
   * Double-dispatch abstract accept method for items that
   * can be visited.
   *
   * @return Return value of Visitor
   */
  virtual R CAccept(BaseVisitor&) const = 0;

protected:
  /**
   * Dispatcher for visitor-visitable double dispatch system
   *
   * @param visited Item to be visited
   * @param guest Guest visitor to be called on visited item
   *
   * @return Return value of guest
   */
  template<class T>
  static ReturnType
  CAcceptImpl(const T& visited, BaseVisitor& guest)
  {
    // Apply the acyclic visitor
    if (ConstVisitor<T>* p = dynamic_cast<ConstVisitor<T>*>(&guest))
      return p->Visit(visited);

    return ReturnType();
  }

};

template <typename R = void>
class BaseVisitable: public BaseConstVisitable<R>
{
public:
  /** Accessible to clients */
  typedef R ReturnType;

  /**
   * Double-dispatch abstract accept method for items that
   * can be visited.
   *
   * @return Return value of Visitor
   */
  virtual R Accept(BaseVisitor&) = 0;

protected:
  /**
   * Dispatcher for visitor-visitable double dispatch system
   *
   * @param visited Item to be visited
   * @param guest Guest visitor to be called on visited item
   *
   * @return Return value of guest
   */
  template<class T>
  static ReturnType
  AcceptImpl(T& visited, BaseVisitor& guest)
  {
    // Apply the acyclic visitor
    if (Visitor<T>* p = dynamic_cast<Visitor<T>*>(&guest))
      return p->Visit(visited);

    return ReturnType();
  }
};


#define DEFINE_CONSTVISITABLE() \
  virtual ReturnType CAccept(BaseVisitor& guest) const \
  { return CAcceptImpl(*this, guest); }

#define DEFINE_VISITABLE() \
  virtual ReturnType Accept(BaseVisitor& guest) \
  { return AcceptImpl(*this, guest); } \
  virtual ReturnType CAccept(BaseVisitor& guest) const \
  { return CAcceptImpl(*this, guest); }

#endif
