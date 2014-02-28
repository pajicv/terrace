/******************************************************************************
 * lidarpoint.cpp
 *
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include "lidarpoint.hpp"

namespace terrace
{
namespace lidar
{

void LidarPoint::setFromRealCoords(wykobi::point3d<double> realPoint)
{
	if(realPoint.x >= mMetadata->offsets().x) 
	{
		mCoords.x = long((realPoint.x - mMetadata->offsets().x) / mMetadata->scales().x + 0.5);
	}
	else
	{
		mCoords.x = long((realPoint.x - mMetadata->offsets().x) / mMetadata->scales().x - 0.5);
	}
	
	if(realPoint.y >= mMetadata->offsets().y) 
	{
		mCoords.y = long((realPoint.y - mMetadata->offsets().y) / mMetadata->scales().y + 0.5);
	}
	else
	{
		mCoords.y = long((realPoint.y - mMetadata->offsets().y) / mMetadata->scales().y - 0.5);
	}

	if(realPoint.z >= mMetadata->offsets().z) 
	{
		mCoords.z = long((realPoint.z - mMetadata->offsets().z) / mMetadata->scales().z + 0.5);
	}
	else
	{
		mCoords.z = long((realPoint.z - mMetadata->offsets().z) / mMetadata->scales().z - 0.5);
	}
}

void LidarPoint::populateLasPoint(liblas::Point& lasPoint) const
{
	lasPoint.SetRawX(mCoords.x);
	lasPoint.SetRawY(mCoords.y);
	lasPoint.SetRawZ(mCoords.z);
	lasPoint.SetClassification(mCls);
}

}
} // namespace terrace::lidar
