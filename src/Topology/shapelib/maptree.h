#ifndef MAPTREE_H
#define MAPTREE_H

#include "mapprimitive.h"
#include "mapshape.h"

#ifdef __cplusplus
extern "C" {
#endif

/* this can be 2 or 4 for binary or quad tree */
#define MAX_SUBNODES 4

typedef struct shape_tree_node {
  // area covered by this node
  rectObj rect;

  // list of shapes stored at this node.
  int numshapes;
  int *ids;

  int numsubnodes;
  struct shape_tree_node *subnode[MAX_SUBNODES];
} treeNodeObj;

typedef struct {
  int numshapes;
  int maxdepth;
  treeNodeObj *root;
} treeObj;


typedef struct
{
    FILE        *fp;
  ZZIP_FILE     *zfp;
    char        signature[3];
    char	LSB_order;
    char        needswap;
    char	version;
    char	flags[3];

    int         nShapes;
    int         nDepth;
} SHPTreeInfo;
typedef SHPTreeInfo * SHPTreeHandle;

#define MS_LSB_ORDER -1
#define MS_MSB_ORDER -2
#define MS_NATIVE_ORDER 0
#define MS_NEW_LSB_ORDER 1
#define MS_NEW_MSB_ORDER 2


SHPTreeHandle msSHPDiskTreeOpen(const char * pszTree, int debug);
void msSHPDiskTreeClose(SHPTreeHandle disktree);

char *msSearchDiskTree(const char *filename, rectObj aoi, int debug);

void msFilterTreeSearch(const shapefileObj *shp, char *status,
                        rectObj search_rect);

#ifdef __cplusplus
}
#endif

#endif /* MAPTREE_H */
