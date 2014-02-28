/******************************************************************************
 * lidarheader.cpp
 *
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include "lidarmetadata.hpp"
#include "liblas/liblas.hpp"

namespace terrace
{
namespace lidar
{

void LidarMetadata::setFromLasHeader(const liblas::Header& theHdr)
{
	mNumberOfPoints = theHdr.GetPointRecordsCount(); 
	mBoundingBox = wykobi::make_box(theHdr.GetMinX(), theHdr.GetMinY(), theHdr.GetMinZ(), 
		theHdr.GetMaxX(), theHdr.GetMaxY(), theHdr.GetMaxZ());
	mOffsets = wykobi::make_vector(theHdr.GetOffsetX(), theHdr.GetOffsetY(), theHdr.GetOffsetZ());
	mScales = wykobi::make_vector(theHdr.GetScaleX(), theHdr.GetScaleY(),
		theHdr.GetScaleZ());
}

void LidarMetadata::populateLasHeader(liblas::Header& theHdr) const
{
	try
	{
	theHdr.SetDataFormatId(liblas::ePointFormat3);
	theHdr.SetCompressed(false);
	// lasHdr.SetSoftwareId(mGeneratingSoftware);
	//theHdr.SetSoftwareId("Terrace");
	theHdr.SetPointRecordsCount(mNumberOfPoints);
	theHdr.SetMin(mBoundingBox[0].x, mBoundingBox[0].y, mBoundingBox[0].z);
	theHdr.SetMax(mBoundingBox[1].x, mBoundingBox[1].y, mBoundingBox[1].z);
	theHdr.SetOffset(mOffsets.x, mOffsets.y, mOffsets.z);
	theHdr.SetScale(mScales.x, mScales.y, mScales.z);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
}

}
} // namespace terrace::lidar
