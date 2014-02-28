/******************************************************************************
 * tin.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Implementation of Triangular Irregular Network representation 
 *           of the terrain. This is based on Triangle libarary for Delaunay
 *           triangulation.
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_TIN_HPP_INCLUDED
#define TERRACE_TIN_HPP_INCLUDED

#include "terracedefs.hpp"
#include "wykobi.hpp"

namespace terrace
{
namespace tin
{

class TriangleIterator;

class TIN 
{
public:
	/** @name Constructor 
	*/
	/// Base constructor for TIN. Initializes mesh structure and sets 
	/// behavior parameters.
	TIN();

	/// Deinitializes mesh and behavior structure.
	~TIN();

	/// Creates TIN from the array of points usind 2D 
	/// delaunay triangluation.
	/// \param thePoints3d an vector of 3d coordinates of points
	/// in form [ [x1, y1, z1], [x2, y2, z2], . . .]
	void create(mydefs::Points3d const& thePoints3d);

	/// Interpolate elevation at the specified coordinates. 
	/// \param theX x coordinate
	/// \param theY y coordinate
	/// \return interpolated elevation
	double interpolate(double theX, double theY) const;

	/// Searches for the triangle that contains specified
	/// coordinates.
	/// \param theX x coordinate
	/// \param theY y coordinate
	/// \param[out] theTriangle coordinates of triangle vertices if 
	/// triangle is found
	/// \return true if triangle is found, false otherwise
	bool findTriangle(double theX, double theY, mydefs::Triangle3d& theTriangle3d) const;

	/// NOT IMPLEMENTED BECAUSE TRIANGLE HAS 
	/// UNDEFINED BEHAVIOR WHEN INSERTING VERTICES 
	/// AFTER MESH CREATION
	/// Inserts new vertex in TIN.
	/// \param theX x coordinate
	/// \param theY y coordinate
	/// \param theZ z coordinate (elevation)
	/// \return returns true if vertex is succesfuly inserted, otherwise returns false
	bool insertVertex(double theX, double theY, double theZ);

	/// NOT IMPLEMENTED BECAUSE TRIANGLE CANNOT DELETE VERTICES 
	/// ON MESH BOUNDARY
	/// Delete vertex if it exists on specified coordinates.
	/// \param theX x coordinate
	/// \param theY y coordinate
	/// \return true if the vertex is found at specified 
	/// coordinates and deleted, otherwise false
	bool deleteVertex(double theX, double theY);

	/// Calculates slope of trinagle that contains specified 
	/// coordinates.
	/// \param theX x coordinate
	/// \param theY y coordinate
	/// \return slope of trinagle
	double calculateSlope(double theX, double theY) const;

	/// Drops object represented by an array of 2D coordinates
	/// on surface of TIN. 
	/// \param vector of 2d points [[x1, y1], [x2, y2] , . . . ]
	/// \return vector of 3d points [ [x1, y1, z1], [x2, y2, z2], . . . ]
	/// \note the number of points in returned vector will be higher (rarely equal) 
	/// than number of input points because new points will be generated on TIN egdes
	/// in order to follow the surface
	const mydefs::Points3d& dropOnTIN(mydefs::Points2d const& thePoints2d) const;

	/// Get spatial bounding box of TIN.
	/// \return spatial bounding box of tin in form
	/// [ [ minx, minz, miny ], [maxx, maxy, maxz] ]
	const mydefs::BoundingBox boundingBox() const;

	/// Iterator to begining of list of triangles. 
	/// \return iterator to first triangle in TIN
	TriangleIterator tiBegin() const;

	/// Iterator to end of the list of triangles.
	/// \return empty iterator
	TriangleIterator tiEnd() const;

	/// Checks if TIN contains no points and triangles
	bool empty()
	{
		return mMesh->triangles.items == 0;
	}

	friend class TriangleIterator;

private:

	double interpolateOnSegment(const wykobi::segment<double, 3>& segment, double theX, double theY) const
	{
		double lineCoef = (theX - segment[0].x) / (segment[1].x - segment[0].x);
		return segment[0].z + (1 - lineCoef) * (segment[1].z - segment[0].z);
	}
	
	double interpolateInTriangle(const wykobi::triangle<double, 3>& theTriangle, double theX, double theY) const
	{
		wykobi::plane<double, 3> triPlane = wykobi::make_plane(theTriangle);
		return -1 * ((triPlane.normal.x * theX + triPlane.normal.y * theY + triPlane.constant) / triPlane.normal.z); 
	}

	/// Mesh data structure (from Triangle) 
	TMesh * mMesh;										// Contains triangles, vertices etc.
	
	/// Data structure for command line switches and file names (from Triangle) 
	TBehavior * mBehavior;								// Controls triangulation behaviour. 

	/// The most recently accessed triangle 
	TOrientedTriangle * mRecentTri;

	/// Minimal value of Z coordinate
	double mMinZ;

	/// Maximal value of Z coordinate
	double mMaxZ;

};

}
}//namespace terrace::tin

#endif //TERRACE_TIN_HPP_INCLUDED