/******************************************************************************
 * gridindex.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Grid index on vector of lidar points. Each cell of 
 *			 grid index stores vector of LidarPoint iterators ordered by 
 *			 ascending elevation.
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_GRIDINDEX_HPP_INCLUDED
#define TERRACE_GRIDINDEX_HPP_INCLUDED

///////////////////////////////////////////////////////////////////////////////
// Forward declared dependacies

///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include "terracedefs.hpp"
#include "lidarpoint.hpp"
//#include "georaster.hpp"

///////////////////////////////////////////////////////////////////////////////
// Actual class

namespace terrace
{
namespace lidar
{

class LidarMetadata;

class GridIndex
{

public:
	
	typedef std::vector< LidarPoint::VectorIterator > Cell;
	typedef wykobi::rectangle<double> BoundingRectangle;

	GridIndex() : mCells(0), 
				  mCellSize(0.0), 
				  mRows(0), 
				  mCols(0),
				  mExtent(wykobi::make_rectangle(double(0.0), double(0.0), double(0.0), double(0.0)))
	{
	}

	~GridIndex()
	{
		delete mCells;
	}

	void create(const LidarMetadata& theMetadata, std::vector<LidarPoint>& thePoints, double theCellSize = 1.0);

	inline const BoundingRectangle& extent()
	{
		return mExtent;
	}

	inline double cellSize() const
	{
		return mCellSize;
	}
	
	inline unsigned int rows() const
	{
		return mRows;
	}

	inline unsigned int columns() const
	{
		return mCols;
	}

	inline const std::vector< Cell >* cells() const
	{
		return mCells;
	}

	inline unsigned int x2col(double theX) const
	{
		return unsigned int((theX - mExtent[0].x) / mCellSize);
	}

	inline unsigned int y2row(double theY) const
	{
		return unsigned int((mExtent[1].y - theY) / mCellSize);
	}

	inline unsigned int index(unsigned int r, unsigned int c) const
	{
		return r * mCols + c;
	}

	inline const Cell& operator[](unsigned int theIndex) const
	{
		return mCells->operator[](theIndex);
	}
	
	void elevationToRaster(const std::string& filename) const;

private:

	void clear()
	{
		if(mCells != 0)
		{
			delete mCells;
		}
		mRows = 0;
		mCols = 0;
		mExtent = wykobi::make_rectangle(double(0.0), double(0.0), double(0.0), double(0.0));
		mCellSize = 0;
	}

	/// 2D extent of grid
	BoundingRectangle mExtent;
	/// Size of grid cell
	double mCellSize;
	/// Number of rows in grid (along Y axis)
	unsigned int mRows;
	/// Number of cols in grid (along X axis)
	unsigned int mCols;
	/// Vector of cells
	std::vector< Cell >* mCells;

};

}
} // namespace terrace::lidar

#endif // TERRACE_LIDARHEADER_HPP_INCLUDED