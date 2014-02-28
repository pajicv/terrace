/******************************************************************************
 * terracedefs.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
             data.
 * Purpose:  Definitions of various types.
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_DEFS_HPP_INCLUDED
#define TERRACE_DEFS_HPP_INCLUDED

// Include std
//#include <limits> 
#include <vector>

// Include wykobi
#include "wykobi.hpp"

// Include Triangle
extern "C"
{
#include "terracetriangle.h"
}

/// Mesh type from Triangle
typedef struct mesh TMesh;

/// Behavior type from Triangle
typedef struct behavior TBehavior;

/// Oriented triangle type from Triangle
typedef struct otri TOrientedTriangle;

/// Triangle type from Triangle
typedef triangle TTriangle;

/// Vertex type from Triangle
typedef vertex TVertex;

/// Triangulation input and output structure
typedef struct triangulateio TTriangulationIO;


namespace terrace
{
namespace mydefs
{

/// No data value for elevations
//const double NODATA = -1 * std::numeric_limits<double>::max();

/// Vector of 3D points
typedef std::vector<wykobi::point3d<double>> Points3d;

/// Vector of 2D points
typedef std::vector<wykobi::point2d<double>> Points2d;

/// 3D triangle from wykobi
typedef wykobi::triangle3d Triangle3d;

/// 3D bounding box in form
/// [ Xmin, Ymin, Zmin, Xmax, Ymax, Zmax ]
typedef wykobi::box<double, 3> BoundingBox;

} // namespace mydefs
} // namespace terrace

#endif //TERRACE_DEFS_HPP_INCLUDED