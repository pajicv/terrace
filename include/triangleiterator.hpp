/******************************************************************************
 * triangleiterator.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Forward iterator through triangles of Triangular 
 *           Irregular Network.
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_TRIITER_HPP_INCLUDED
#define TERRACE_TRIITER_HPP_INCLUDED

#include "terracedefs.hpp"
#include "tin.hpp"
#include <iterator>

namespace terrace
{	
namespace tin
{

class TIN;

class TriangleIterator : public std::iterator<std::forward_iterator_tag, mydefs::Triangle3d>
{
private:

	TIN * mTIN;
	
	TOrientedTriangle mCurrentTriangle;

public:

	TriangleIterator();

	TriangleIterator(const TIN * theTIN);

	/// const TOrientedTriangle& currentTriangle() const;

	bool validTriangle() const;
	
	TriangleIterator& operator++();
	
	const mydefs::Triangle3d operator*(); 

	bool operator==(const TriangleIterator& theTI) const;

	bool operator!=(const TriangleIterator& theTriangleIterator) const;

};

}
}// namespace terrace::tin

#endif // TERRACE_TRIITER_HPP_INCLUDED