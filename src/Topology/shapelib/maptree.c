/*
** This code is based in part on the previous work of Frank Warmerdam. See
** the README for licence details.
*/
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

#include "maptree.h"
#include "maperror.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <stdbool.h>

static const bool bBigEndian = BYTE_ORDER == BIG_ENDIAN;

/* -------------------------------------------------------------------- */
/*      If the following is 0.5, nodes will be split in half.  If it    */
/*      is 0.6 then each subnode will contain 60% of the parent         */
/*      node, with 20% representing overlap.  This can be help to       */
/*      prevent small objects on a boundary from shifting too high      */
/*      up the tree.                                                    */
/* -------------------------------------------------------------------- */
#define SPLITRATIO  0.55

static void SwapWord( int length, void * wordP )
{
  int i;
  uchar	temp;

  for( i=0; i < length/2; i++ )
    {
      temp = ((uchar *) wordP)[i];
      ((uchar *)wordP)[i] = ((uchar *) wordP)[length-i-1];
      ((uchar *) wordP)[length-i-1] = temp;
    }
}

SHPTreeHandle msSHPDiskTreeOpen(const char * pszTree, int debug)
{
    char		*pszFullname, *pszBasename;
    SHPTreeHandle	psTree;

    char		pabyBuf[16];
    int			i;

  /* -------------------------------------------------------------------- */
  /*	Initialize the info structure.					    */
  /* -------------------------------------------------------------------- */
    psTree = (SHPTreeHandle) malloc(sizeof(SHPTreeInfo));

  /* -------------------------------------------------------------------- */
  /*	Compute the base (layer) name.  If there is any extension	    */
  /*	on the passed in filename we will strip it off.			    */
  /* -------------------------------------------------------------------- */
    pszBasename = (char *) malloc(strlen(pszTree)+5);
    strcpy( pszBasename, pszTree );
    for( i = strlen(pszBasename)-1;
       i > 0 && pszBasename[i] != '.' && pszBasename[i] != '/'
	 && pszBasename[i] != '\\';
       i-- ) {}

    if( pszBasename[i] == '.' )
      pszBasename[i] = '\0';

  /* -------------------------------------------------------------------- */
  /*	Open the .shp and .shx files.  Note that files pulled from	    */
  /*	a PC to Unix with upper case filenames won't work!		    */
  /* -------------------------------------------------------------------- */
    pszFullname = (char *) malloc(strlen(pszBasename) + 5);
    sprintf( pszFullname, "%s%s", pszBasename, MS_INDEX_EXTENSION);

    psTree->zfp = zzip_fopen(pszFullname, "rb" );

    msFree(pszBasename); // don't need these any more
    msFree(pszFullname);

    if( psTree->zfp == NULL ) {
      msFree(psTree);
      return( NULL );
    }

    zzip_fread( pabyBuf, 8, 1, psTree->zfp );

    memcpy( &psTree->signature, pabyBuf, 3 );
    if( strncmp(psTree->signature,"SQT",3) )
    {
  /* ---------------------------------------------------------------------- */
  /*     must check if the 2 first bytes equal 0 of max depth that cannot   */
  /*     be more than 65535. If yes, we must swap all value. The problem    */
  /*     here is if there's no Depth (bytea 5,6,7,8 in the file) all bytes  */
  /*     will be set to 0. So,we will test with the number of shapes (bytes */
  /*     1,2,3,4) that cannot be more than 65535 too.                       */
  /* ---------------------------------------------------------------------- */
      if (debug)
      {
          msDebug("WARNING in msSHPDiskTreeOpen(): %s is in old index format "
                  "which has been deprecated.  It is strongly recommended to "
                  "regenerate it in new format.\n", pszTree);
      }
      if((pabyBuf[4] == 0 && pabyBuf[5] == 0 &&
          pabyBuf[6] == 0 && pabyBuf[7] == 0))
      {
        psTree->LSB_order = !(pabyBuf[0] == 0 && pabyBuf[1] == 0);
      }
      else
      {
        psTree->LSB_order = !(pabyBuf[4] == 0 && pabyBuf[5] == 0);
      }
      psTree->needswap = ((psTree->LSB_order) != (!bBigEndian));

  /* ---------------------------------------------------------------------- */
  /*     poor hack to see if this quadtree was created by a computer with a */
  /*     different Endian                                                   */
  /* ---------------------------------------------------------------------- */
      psTree->version = 0;
    }
    else
    {
      psTree->needswap = (( pabyBuf[3] == MS_NEW_MSB_ORDER ) ^ ( bBigEndian ));

      psTree->LSB_order = ( pabyBuf[3] == MS_NEW_LSB_ORDER );
      memcpy( &psTree->version, pabyBuf+4, 1 );
      memcpy( &psTree->flags, pabyBuf+5, 3 );

      zzip_fread( pabyBuf, 8, 1, psTree->zfp );
    }

    if( psTree->needswap ) SwapWord( 4, pabyBuf );
    memcpy( &psTree->nShapes, pabyBuf, 4 );

    if( psTree->needswap ) SwapWord( 4, pabyBuf+4 );
    memcpy( &psTree->nDepth, pabyBuf+4, 4 );

    return( psTree );
}


void msSHPDiskTreeClose(SHPTreeHandle disktree)
{
    zzip_fclose( disktree->zfp );
    free( disktree );
}

static void searchDiskTreeNode(SHPTreeHandle disktree, rectObj aoi, char *status)
{
  int i;
  long offset;
  int numshapes, numsubnodes;
  rectObj rect;

  int *ids=NULL;

  zzip_fread( &offset, 4, 1, disktree->zfp );
  if ( disktree->needswap ) SwapWord ( 4, &offset );

  zzip_fread( &rect, sizeof(rectObj), 1, disktree->zfp );
  if ( disktree->needswap ) SwapWord ( 8, &rect.minx );
  if ( disktree->needswap ) SwapWord ( 8, &rect.miny );
  if ( disktree->needswap ) SwapWord ( 8, &rect.maxx );
  if ( disktree->needswap ) SwapWord ( 8, &rect.maxy );

  zzip_fread( &numshapes, 4, 1, disktree->zfp );
  if ( disktree->needswap ) SwapWord ( 4, &numshapes );

  if(!msRectOverlap(&rect, &aoi)) { // skip rest of this node and sub-nodes
    offset += numshapes*sizeof(int) + sizeof(int);
    fseek(disktree->fp, offset, SEEK_CUR);
    return;
  }
  if(numshapes > 0) {
    ids = (int *)malloc(numshapes*sizeof(int));

    zzip_fread( ids, numshapes*sizeof(int), 1, disktree->zfp );
    if (disktree->needswap )
    {
      for( i=0; i<numshapes; i++ )
      {
        SwapWord( 4, &ids[i] );
        msSetBit(status, ids[i], 1);
      }
    }
    else
    {
      for(i=0; i<numshapes; i++)
        msSetBit(status, ids[i], 1);
    }
    free(ids);
  }

  zzip_fread( &numsubnodes, 4, 1, disktree->zfp );
  if ( disktree->needswap ) SwapWord ( 4, &numsubnodes );

  for(i=0; i<numsubnodes; i++)
    searchDiskTreeNode(disktree, aoi, status);

  return;
}

char *msSearchDiskTree(const char *filename, rectObj aoi, int debug)
{
  SHPTreeHandle	disktree;
  char *status=NULL;

  disktree = msSHPDiskTreeOpen (filename, debug);
  if(!disktree) {

    // only set this error IF debugging is turned on, gets annoying otherwise
    if(debug) msSetError(MS_IOERR, "Unable to open spatial index for %s. In most cases you can safely ignore this message, otherwise check file names and permissions.", "msSearchDiskTree()", filename);

    return(NULL);
  }

  status = msAllocBitArray(disktree->nShapes);
  if(!status) {
    msSetError(MS_MEMERR, NULL, "msSearchDiskTree()");
    msSHPDiskTreeClose( disktree );
    return(NULL);
  }

  searchDiskTreeNode(disktree, aoi, status);

  msSHPDiskTreeClose( disktree );
  return(status);
}

// Function to filter search results further against feature bboxes
void msFilterTreeSearch(const shapefileObj *shp, char *status,
                        rectObj search_rect)
{
  int i;
  rectObj shape_rect;

  for(i=0;i<shp->numshapes;i++) { /* for each shape */
    if(msGetBit(status, i)) {
      if(!msSHPReadBounds(shp->hSHP, i, &shape_rect))
	if(msRectOverlap(&shape_rect, &search_rect) != MS_TRUE)
	  msSetBit(status, i, 0);
    }
  }
}
