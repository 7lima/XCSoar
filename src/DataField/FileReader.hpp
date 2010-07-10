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

#ifndef XCSOAR_DATA_FIELD_FILE_READER_HPP
#define XCSOAR_DATA_FIELD_FILE_READER_HPP

#include "DataField/Base.hpp"

/** Maximum of files in the list */
#define DFE_MAX_FILES 100

/** FileList item */
typedef struct
{
  /** Filename */
  TCHAR *mTextFile;
  /** Path including Filename */
  TCHAR *mTextPathFile;
} DataFieldFileReaderEntry;

/**
 * #DataField specialisation that supplies options as a list of
 * files matching a suffix.  First entry is always blank for null entry.
 * 
 */
class DataFieldFileReader: public DataField
{
private:
  /** Number of files to choose from */
  unsigned int nFiles;
  /** Index of the active file */
  unsigned int mValue;
  /** FileList item array */
  DataFieldFileReaderEntry fields[DFE_MAX_FILES];

public:
  /**
   * Constructor of the DataFieldFileReader class
   * @param EditFormat
   * @param DisplayFormat
   * @param OnDataAccess
   */
  DataFieldFileReader(const TCHAR *EditFormat, const TCHAR *DisplayFormat,
                      DataAccessCallback_t OnDataAccess);

  /** Deconstructor */
  virtual ~DataFieldFileReader();

  /** Move the selection up (+1) */
  void Inc(void);
  /** Move the selection down (-1) */
  void Dec(void);
  /**
   * Prepares the ComboList items
   * @return The number of items in the ComboList
   */
  int CreateComboList(void);

  /**
   * Adds a filename/filepath couple to the filelist
   * @param fname The filename
   * @param fpname The filepath
   */
  void addFile(const TCHAR *fname, const TCHAR *fpname);

  /**
   * Checks whether the given filename (fname) matches the given filter
   * @param fname The filename to check
   * @param filter The filter used for checking
   * @return True if file matches the filter, False otherwise
   */
  bool checkFilter(const TCHAR *fname, const TCHAR* filter);

  /**
   * Returns the number of files in the list
   * @return The number of files in the list
   */
  int GetNumFiles(void) const;

  /**
   * Returns true if a file was selected.
   */
  virtual bool GetAsBoolean() const;

  /**
   * Returns the selection index in integer format
   * @return The selection index in integer format
   */
  virtual int GetAsInteger(void) const;

  /**
   * Returns the selection title (filename)
   * @return The selection title (filename)
   */
  virtual const TCHAR *GetAsString(void) const;

  /**
   * Iterates through the file list and tries to find an item where the path
   * is equal to the given text, if found the selection is changed to
   * that item
   * @param text PathFile to search for
   */
  void Lookup(const TCHAR* text);

  /**
   * Returns the PathFile of the currently selected item
   * @return The PathFile of the currently selected item
   */
  const TCHAR *GetPathFile(void) const;

  #if defined(__BORLANDC__)
  #pragma warn -hid
  #endif

  /**
   * Sets the selection to the given index
   * @param Value The array index to select
   */
  void Set(int Value);

  #if defined(__BORLANDC__)
  #pragma warn +hid
  #endif

  /**
   * @see Set()
   * @return The index that was set (min: 0 / max: nFiles)
   */
  int SetAsInteger(int Value);

  /** Sorts the filelist by filenames */
  void Sort();
  void ScanDirectoryTop(const TCHAR *filter);

  /** For use by other classes */
  unsigned size() const;
  const DataFieldFileReaderEntry& getItem(unsigned index) const;

 protected:
  bool ScanFiles(const TCHAR *pattern, const TCHAR *filter);
  bool ScanDirectories(const TCHAR *pattern, const TCHAR *filter);
};

#endif
