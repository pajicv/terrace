/******************************************************************************
 * terracetrinalge.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Declarations from Triangle libarary used
 *           in terrace project.
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/
#ifndef TERRACE_TRIANGLE_H_INCLUDED
#define TERRACE_TRIANGLE_H_INCLUDED

#define ANSI_DECLARATORS
#define TRILIBRARY

//=====================================
// Triangle typedefs

/* #define SINGLE */

#ifdef SINGLE
#define REAL float
#else /* not SINGLE */
#define REAL double
#endif /* not SINGLE */

#define FILENAMESIZE 2048

#define VOID int

enum locateresult {INTRIANGLE, ONEDGE, ONVERTEX, OUTSIDE};

enum insertvertexresult {SUCCESSFULVERTEX, ENCROACHINGVERTEX, VIOLATINGVERTEX,
                         DUPLICATEVERTEX};

typedef REAL **triangle;

struct otri {
  triangle *tri;
  int orient;                                         
};

typedef REAL **subseg;

struct osub {
  subseg *ss;
  int ssorient;                                     
};

typedef REAL *vertex;

struct badsubseg {
  subseg encsubseg;                             
  vertex subsegorg, subsegdest;                        
};

struct badtriang {
  triangle poortri;                       
  REAL key;                           
  vertex triangorg, triangdest, triangapex;          
  struct badtriang *nexttriang;            
};

struct flipstacker {
  triangle flippedtri;                       
  struct flipstacker *prevflip;               
};

struct event {
  REAL xkey, ykey;                             
  VOID *eventptr;      
  int heapposition;              
};

struct splaynode {
  struct otri keyedge;                    
  vertex keydest;           
  struct splaynode *lchild, *rchild;              
};

struct memorypool {
  VOID **firstblock, **nowblock;
  VOID *nextitem;
  VOID *deaditemstack;
  VOID **pathblock;
  VOID *pathitem;
  int alignbytes;
  int itembytes;
  int itemsperblock;
  int itemsfirstblock;
  long items, maxitems;
  int unallocateditems;
  int pathitemsleft;
};

struct mesh {

  struct memorypool triangles;
  struct memorypool subsegs;
  struct memorypool vertices;
  struct memorypool viri;
  struct memorypool badsubsegs;
  struct memorypool badtriangles;
  struct memorypool flipstackers;
  struct memorypool splaynodes;

  struct badtriang *queuefront[4096];
  struct badtriang *queuetail[4096];
  int nextnonemptyq[4096];
  int firstnonemptyq;

  struct flipstacker *lastflip;

  REAL xmin, xmax, ymin, ymax;                            /* x and y bounds. */
  REAL xminextreme;      /* Nonexistent x value used as a flag in sweepline. */
  int invertices;                               /* Number of input vertices. */
  int inelements;                              /* Number of input triangles. */
  int insegments;                               /* Number of input segments. */
  int holes;                                       /* Number of input holes. */
  int regions;                                   /* Number of input regions. */
  int undeads;    /* Number of input vertices that don't appear in the mesh. */
  long edges;                                     /* Number of output edges. */
  int mesh_dim;                                /* Dimension (ought to be 2). */
  int nextras;                           /* Number of attributes per vertex. */
  int eextras;                         /* Number of attributes per triangle. */
  long hullsize;                          /* Number of edges in convex hull. */
  int steinerleft;                 /* Number of Steiner points not yet used. */
  int vertexmarkindex;         /* Index to find boundary marker of a vertex. */
  int vertex2triindex;     /* Index to find a triangle adjacent to a vertex. */
  int highorderindex;  /* Index to find extra nodes for high-order elements. */
  int elemattribindex;            /* Index to find attributes of a triangle. */
  int areaboundindex;             /* Index to find area bound of a triangle. */
  int checksegments;         /* Are there segments in the triangulation yet? */
  int checkquality;                  /* Has quality triangulation begun yet? */
  int readnodefile;                           /* Has a .node file been read? */
  long samples;              /* Number of random samples for point location. */

  long incirclecount;                 /* Number of incircle tests performed. */
  long counterclockcount;     /* Number of counterclockwise tests performed. */
  long orient3dcount;           /* Number of 3D orientation tests performed. */
  long hyperbolacount;      /* Number of right-of-hyperbola tests performed. */
  long circumcentercount;  /* Number of circumcenter calculations performed. */
  long circletopcount;       



  vertex infvertex1, infvertex2, infvertex3;

  triangle *dummytri;
  triangle *dummytribase;              

  subseg *dummysub;
  subseg *dummysubbase;    

  struct otri recenttri;

};                                                  

struct behavior {

  int poly, refine, quality, vararea, fixedarea, usertest;
  int regionattrib, convex, weighted, jettison;
  int firstnumber;
  int edgesout, voronoi, neighbors, geomview;
  int nobound, nopolywritten, nonodewritten, noelewritten, noiterationnum;
  int noholes, noexact, conformdel;
  int incremental, sweepline, dwyer;
  int splitseg;
  int docheck;
  int quiet, verbose;
  int usesegments;
  int order;
  int nobisect;
  int steiner;
  REAL minangle, goodangle, offconstant;
  REAL maxarea;

#ifndef TRILIBRARY
  char innodefilename[FILENAMESIZE];
  char inelefilename[FILENAMESIZE];
  char inpolyfilename[FILENAMESIZE];
  char areafilename[FILENAMESIZE];
  char outnodefilename[FILENAMESIZE];
  char outelefilename[FILENAMESIZE];
  char outpolyfilename[FILENAMESIZE];
  char edgefilename[FILENAMESIZE];
  char vnodefilename[FILENAMESIZE];
  char vedgefilename[FILENAMESIZE];
  char neighborfilename[FILENAMESIZE];
  char offfilename[FILENAMESIZE];
#endif /* not TRILIBRARY */

};  

struct triangulateio {
  REAL *pointlist;                                               /* In / out */
  REAL *pointattributelist;                                      /* In / out */
  int *pointmarkerlist;                                          /* In / out */
  int numberofpoints;                                            /* In / out */
  int numberofpointattributes;                                   /* In / out */

  int *trianglelist;                                             /* In / out */
  REAL *triangleattributelist;                                   /* In / out */
  REAL *trianglearealist;                                         /* In only */
  int *neighborlist;                                             /* Out only */
  int numberoftriangles;                                         /* In / out */
  int numberofcorners;                                           /* In / out */
  int numberoftriangleattributes;                                /* In / out */

  int *segmentlist;                                              /* In / out */
  int *segmentmarkerlist;                                        /* In / out */
  int numberofsegments;                                          /* In / out */

  REAL *holelist;                        /* In / pointer to array copied out */
  int numberofholes;                                      /* In / copied out */

  REAL *regionlist;                      /* In / pointer to array copied out */
  int numberofregions;                                    /* In / copied out */

  int *edgelist;                                                 /* Out only */
  int *edgemarkerlist;            /* Not used with Voronoi diagram; out only */
  REAL *normlist;                /* Used only with Voronoi diagram; out only */
  int numberofedges;                                             /* Out only */
};


//=====================================
// Triangle macros

//int plus1mod3[3] = {1, 2, 0};
//int minus1mod3[3] = {2, 0, 1};
//
//#define org(otri, vertexptr)                                                  \
//  vertexptr = (vertex) (otri).tri[plus1mod3[(otri).orient] + 3]
//
//#define dest(otri, vertexptr)                                                 \
//  vertexptr = (vertex) (otri).tri[minus1mod3[(otri).orient] + 3]
//
//#define apex(otri, vertexptr)                                                 \
//  vertexptr = (vertex) (otri).tri[(otri).orient + 3]

//=====================================
// Triangle functions

void triangleinit(struct mesh *m);

void parsecommandline(int argc, char **argv, struct behavior *b);

void triangledeinit(struct mesh *m, struct behavior *b);

void transfernodes(struct mesh *m, struct behavior *b, REAL *pointlist,
                   REAL *pointattriblist, int *pointmarkerlist,
                   int numberofpoints, int numberofpointattribs);

long delaunay(struct mesh *m, struct behavior *b);

void traversalinit(struct memorypool *pool);

triangle *triangletraverse(struct mesh *m);

enum locateresult preciselocate(struct mesh *m, struct behavior *b,
                                vertex searchpoint, struct otri *searchtri,
                                int stopatsubsegment);

enum insertvertexresult insertvertex(struct mesh *m, struct behavior *b,
                                     vertex newvertex, struct otri *searchtri,
                                     struct osub *splitseg,
                                     int segmentflaws, int triflaws);

 void deletevertex(struct mesh *m, struct behavior *b, struct otri *deltri);

 VOID *poolalloc(struct memorypool *pool);

 void pooldealloc(struct memorypool *pool, VOID *dyingitem);

 void boundingbox(struct mesh *m, struct behavior *b);

 long removebox(struct mesh *m, struct behavior *b);

#endif // TERRACE_TRIANGLE_H_INCLUDED