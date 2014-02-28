/******************************************************************************
 * tin.cpp
 *
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#include <limits> 

#include "tin.hpp"
#include "triangleiterator.hpp"

namespace terrace
{
namespace tin
{

TIN::TIN() : mMesh(NULL), mBehavior(NULL), mRecentTri(NULL), mMinZ(std::numeric_limits<double>::max()), mMaxZ(-1 * std::numeric_limits<double>::max())
{
	mMesh = new TMesh;
	mBehavior = new TBehavior;
	mRecentTri = new TOrientedTriangle;

	// Using incremental algorithm for triangulation
	char * switches = {"zQ"}; 

	triangleinit(mMesh);

	parsecommandline(1, &switches, mBehavior);

	mRecentTri->tri = NULL;
	mRecentTri->orient = 0;
}

TIN::~TIN()
{
	triangledeinit(mMesh, mBehavior);
	
	delete mRecentTri;
	delete mBehavior;
	delete mMesh;
}

void TIN::create(const mydefs::Points3d& thePoints3d)
{
	// Set input values for triangulation
	TTriangulationIO in;
	// Number of points
	in.numberofpoints = thePoints3d.size(); 
	// One attribute for Z coordinate
	in.numberofpointattributes = 1; 
	// Allocate memory for array of X and Y coordinates
	in.pointlist = (double *) malloc(in.numberofpoints * 2 * sizeof(double));
	// Allocate memory for array of Z coordinates
	in.pointattributelist = (double *) malloc(in.numberofpoints *
                                          in.numberofpointattributes *
                                          sizeof(double));
	in.pointmarkerlist = (int *) malloc(in.numberofpoints * sizeof(int));
	// Zero segments, holes and regions
	in.numberofsegments = 0;
	in.numberofholes = 0;
	in.numberofregions = 0;

	// Populate arrays of coordinates
	int i = 0;
	for(mydefs::Points3d::const_iterator iter = thePoints3d.begin(); 
		iter != thePoints3d.end(); 
		++iter)
	{
		in.pointlist[2*i] = (*iter)[0];
		in.pointlist[2*i+1] = (*iter)[1];
		in.pointattributelist[i] = (*iter)[2];
		
		if(mMinZ > (*iter)[2])
		{
			mMinZ = (*iter)[2];
		}
		if(mMaxZ < (*iter)[2])
		{
			mMaxZ = (*iter)[2];
		}

		++i;
	}
	
	// Prepare for triangulation
	transfernodes(mMesh, mBehavior, in.pointlist, in.pointattributelist,
			in.pointmarkerlist, in.numberofpoints,
			in.numberofpointattributes);

	// Triangulate points
	mMesh->hullsize = delaunay(mMesh, mBehavior);

	// Necessary maintenance 
	mMesh->infvertex1 = (TVertex) NULL;
	mMesh->infvertex2 = (TVertex) NULL;
	mMesh->infvertex3 = (TVertex) NULL;
	mMesh->edges = (3l * mMesh->triangles.items + mMesh->hullsize) / 2l;

	// Set most recent triangle
	traversalinit(&mMesh->triangles);
	mRecentTri->tri = triangletraverse(mMesh);
	mRecentTri->orient = 0;

	// Free memory
	free(in.pointmarkerlist);
	free(in.pointattributelist);
	free(in.pointlist);
}

double TIN::interpolate(double theX, double theY) const
{

////////////////////////////////////////////////////////////////////////////////
int plus1mod3[3] = {1, 2, 0};
int minus1mod3[3] = {2, 0, 1};

#define org(otri, vertexptr)                                                  \
  vertexptr = (vertex) (otri).tri[plus1mod3[(otri).orient] + 3]

#define dest(otri, vertexptr)                                                 \
  vertexptr = (vertex) (otri).tri[minus1mod3[(otri).orient] + 3]

#define apex(otri, vertexptr)                                                 \
  vertexptr = (vertex) (otri).tri[(otri).orient + 3]
////////////////////////////////////////////////////////////////////////////////
	
	double z;
	TVertex v;
	TVertex t1;
	TVertex t2;
	TVertex t3;
	wykobi::segment<double, 3> segment;
	wykobi::triangle<double, 3> triangle;

	v = static_cast<TVertex>(new double[2]);
	v[0] = theX;
	v[1] = theY;

	switch (preciselocate(mMesh, mBehavior, v, mRecentTri, 0)) {
		case ONVERTEX:
			org(*mRecentTri, t1);
			// Take elevation of vertex
			z = t1[2];
			// delete [] t1;
			break;
		case ONEDGE:
			// Interpolate elevation on edge
			org(*mRecentTri, t1);
			dest(*mRecentTri, t2);
			segment = wykobi::make_segment(t1[0], t1[1], t1[2], 
				t2[0], t2[1], t2[2]);
			z = interpolateOnSegment(segment, theX, theY);
			// delete [] t1;
			// delete [] t2;
			break;
		case INTRIANGLE:
			// Interpolate elevation on triangle
			org(*mRecentTri, t1);
			dest(*mRecentTri, t2);
			apex(*mRecentTri, t3);
			triangle = wykobi::make_triangle(t1[0], t1[1], t1[2], 
				t2[0], t2[1], t2[2], t3[0], t3[1], t3[2]); 
			z = interpolateInTriangle(triangle, theX, theY);
			// delete [] t1;
			// delete [] t2;
			// delete [] t3;
			break;
		case OUTSIDE:
			z = -1 * std::numeric_limits<double>::max(); 
			break;
	}

	delete [] v;

	return z;
}

bool TIN::findTriangle(double theX, double theY, mydefs::Triangle3d& theTriangle3d) const
{

////////////////////////////////////////////////////////////////////////////////
int plus1mod3[3] = {1, 2, 0};
int minus1mod3[3] = {2, 0, 1};

#define org(otri, vertexptr)                                                  \
  vertexptr = (vertex) (otri).tri[plus1mod3[(otri).orient] + 3]

#define dest(otri, vertexptr)                                                 \
  vertexptr = (vertex) (otri).tri[minus1mod3[(otri).orient] + 3]

#define apex(otri, vertexptr)                                                 \
  vertexptr = (vertex) (otri).tri[(otri).orient + 3]
////////////////////////////////////////////////////////////////////////////////

	bool result = false;

	TVertex v;
	TVertex t1;
	TVertex t2;
	TVertex t3;

	v = static_cast<TVertex>(new double[2]);
	v[0] = theX;
	v[1] = theY;

	if(preciselocate(mMesh, mBehavior, v, mRecentTri, 0) != OUTSIDE) 
	{
		org(*mRecentTri, t1);
		dest(*mRecentTri, t2);
		apex(*mRecentTri, t3);
		theTriangle3d = wykobi::make_triangle(t1[0], t1[1], t1[2],
			t2[0], t2[1], t2[2],
			t3[0], t3[1], t3[2]);
		result = true;
	} 

	delete [] v;

	return result;
}

//bool TIN::insertVertex(double theX, double theY, double theZ)
//{
//	bool result = false;
//
//	TVertex v;
//
//	//v = static_cast<TVertex>(new double[3]);
//	v = (TVertex) poolalloc(&mMesh->vertices);
//	v[0] = theX;
//	v[1] = theY;
//	v[2] = theZ;
//
//	if(insertvertex(mMesh, mBehavior, v, mRecentTri, NULL, 0, 0) == SUCCESSFULVERTEX)
//	{
//		result = true;
//	}
//
//	pooldealloc(&mMesh->vertices, reinterpret_cast<VOID *>(v));
//
//	return result; 
//}

//bool TIN::deleteVertex(double theX, double theY)
//{
//	bool result = false;
//
//	//TVertex v;
//	
//	//v = (TVertex) poolalloc(&mMesh->vertices);
//	//v[0] = theX;
//	//v[1] = theY;
//
//	//if(preciselocate(mMesh, mBehavior, v, mRecentTri, 0) == ONVERTEX)
//	//{
//	//	deletevertex(mMesh, mBehavior, mRecentTri);
//	//	result = true;
//	//}
//
//	//pooldealloc(&mMesh->vertices, reinterpret_cast<VOID *>(v));
//
//	TriangleIterator ti;
//	mydefs::Triangle3d currentTriangle;
//	for(ti = tiBegin(); ti != tiEnd(); ++ti)
//	{
//		currentTriangle = *ti;
//		if((currentTriangle[0].x == theX) && (currentTriangle[0].y == theY))
//		{
//			TOrientedTriangle delTri = ti.currentTriangle();
//			deletevertex(mMesh, mBehavior, &delTri);
//			result = true;
//			break;
//		}
//	}
//
//	return result;
//
//}

const mydefs::BoundingBox TIN::boundingBox() const
{
	mydefs::BoundingBox bb = wykobi::make_box(mMesh->xmin, mMesh->ymin, mMinZ,
		mMesh->xmax, mMesh->ymax, mMaxZ);
	return bb;
}

TriangleIterator TIN::tiBegin() const
{
	return TriangleIterator(this);
}

TriangleIterator TIN::tiEnd() const
{
	return TriangleIterator();
}

}
} //namespace terrace::tin