/******************************************************************************
 * gridindex.cpp
 *
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#include "gridindex.hpp"
#include "lidarmetadata.hpp"
#include <iostream>

namespace terrace
{
namespace lidar
{

void GridIndex::create(const LidarMetadata& theMetadata, std::vector<LidarPoint>& thePoints, double theCellSize)
{
	clear();

	if(theCellSize > 0)
	{
		mCellSize = theCellSize;
	}
	else
	{
		std::cout << "WARNING: Cell size must be grater than 0. "
			<< "It is set to 1.0. \n";
		mCellSize = 1.0;
	}

	mExtent[0].x = theMetadata.boundingBox()[0].x;
	mExtent[0].y = theMetadata.boundingBox()[0].y;
	mExtent[1].x = theMetadata.boundingBox()[1].x;
	mExtent[1].y = theMetadata.boundingBox()[1].y;

	mRows =  unsigned int((mExtent[1].y - mExtent[0].y) / mCellSize) + 1;
	mCols =  unsigned int((mExtent[1].x - mExtent[0].x) / mCellSize) + 1;

	mCells = new std::vector<Cell>(mRows * mCols);
	
	for(LidarPoint::VectorIterator pointsIt = thePoints.begin(); 
		pointsIt != thePoints.end(); ++pointsIt)
	{
		unsigned int c = x2col((*pointsIt).realCoords().x);
		unsigned int r = y2row((*pointsIt).realCoords().y);
		mCells->operator[](index(r, c)).push_back(pointsIt);
	}
	//LidarPoint::VectorIterator pointsIt = thePoints.begin();
	//for(unsigned int i = 0; i < thePoints.size(); ++i)
	//{
	//	unsigned int c = x2col(thePoints[i].realCoords().x);
	//	unsigned int r = y2row(thePoints[i].realCoords().y);
	//	mCells->operator[](index(r, c)).push_back(pointsIt + i);
	//}
	
	for(std::vector< Cell >::iterator cellsIt = mCells->begin(); cellsIt != mCells->end(); ++cellsIt)
	{
		if( !(*cellsIt).empty() )
		{
			std::sort((*cellsIt).begin(), (*cellsIt).end(), compareZ);
		}
	}	

}

void GridIndex::elevationToRaster(const std::string& filename) const
{
	//terrace::georaster::Georaster<double>::Band band;
	//band.resize(mRows * mCols, 0);
	//int i = 0;
	//for(std::vector<Cell>::const_iterator cellsIt = mCells.begin(); cellsIt != mCells.end(); ++cellsIt)
	//{
	//	if(!(*cellsIt).empty())
	//	{
	//		band[i] = ((*(*cellsIt).front()).realCoords().z);
	//	}
	//	++i;
	//}

	//terrace::georaster::Georaster<double>::Image img;
	//img.push_back(band);

	//terrace::georaster::Georaster<double>::Georeference georef;

	//georef.x0 = mExtent[0].x;
	//georef.y0 = mExtent[1].y;
	//georef.dx = mCellSize;
	//georef.dy = - mCellSize;
	//georef.rotx = 0.0;
	//georef.roty = 0.0;

	//terrace::georaster::Georaster<double> geoRaster;
	//geoRaster.create(1, mRows, mCols, img, georef);

	//geoRaster.saveAs(filename);
}

}
} // namespace terrace::lidar