/******************************************************************************
 * lidardataset.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Lidar data set loaded from file. 
 *
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_LIDARDATASET_HPP_INCLUDED
#define TERRACE_LIDARDATASET_HPP_INCLUDED

///////////////////////////////////////////////////////////////////////////////
// Forward declared dependacies

///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include <string>
#include <vector>

#include "lidarpoint.hpp"
#include "lidarmetadata.hpp"
#include "gridindex.hpp"

///////////////////////////////////////////////////////////////////////////////
// Actual class

namespace terrace
{
namespace lidar
{

class LidarDataset
{
private:
	
	bool mLoaded;

	std::string mSource;
	
	std::vector<LidarPoint> mPoints;

	LidarMetadata mMetadata;

	GridIndex mGridIndex;

public:
	
	LidarDataset() : mLoaded(false), mSource(""), mPoints(), mMetadata()
	{
	}

	bool load(const std::string& theSource);

	bool loadFromXyz(const std::string& theSource);

	bool saveAs(const std::string& theDestination) const;

	double estimateDensity() const;

	unsigned int classifyGround(double blockSize, 
								double angleTreshold,
								double distanceTreshold, 
								double edgeLengthTreshold);

	const std::string& source() const
	{
		return mSource;
	}

	std::vector<LidarPoint>& points()
	{
		return mPoints;
	}

	const LidarMetadata& metadata() const
	{
		return mMetadata;
	}

	const GridIndex& gridIndex() const
	{
		return mGridIndex;
	}

}; // class LidarDataset

}
} // namespace terrace::lidar

#endif // TERRACE_LIDARDATASET_HPP_INCLUDED