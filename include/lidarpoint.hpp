/******************************************************************************
 * lidarpoint.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Point measured by Lidar system. Currently
 *           only contains coordinates and classification
 *           attribute.
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_LIDARPOINT_HPP_INCLUDED
#define TERRACE_LIDARPOINT_HPP_INCLUDED

///////////////////////////////////////////////////////////////////////////////
// Forward declared dependacies

///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include "terracedefs.hpp"
#include "liblas\point.hpp"
#include "lidarmetadata.hpp"

///////////////////////////////////////////////////////////////////////////////
// Actual class

namespace terrace
{
namespace lidar
{

class LidarPoint 
{
public:

	/////////////////////// Type definitions ///////////////////////

	typedef std::vector<LidarPoint>::iterator VectorIterator;
	typedef std::vector<LidarPoint>::const_iterator ConstVectorIterator;

	/////////////////////////// Methods ////////////////////////////
	
	LidarPoint(const LidarMetadata& theMetadata, const wykobi::point3d<long> theCoords, unsigned char theCls = 0)	
		: mMetadata(&theMetadata), mCoords(theCoords), mCls(theCls)
	{
	} 

	LidarPoint(const LidarMetadata& theMetadata, long theX, long theY, long theZ, unsigned char theCls = 0)
		: mMetadata(&theMetadata), mCoords(wykobi::make_point(theX, theY, theZ)), mCls(theCls)
	{
	}

	LidarPoint(const LidarMetadata& theMetadata, const wykobi::point3d<double> theRealCoords, unsigned char theCls = 0)	
		: mMetadata(&theMetadata), mCls(theCls)
	{
		// Initialize mCoords
		setFromRealCoords(theRealCoords);
	} 

	LidarPoint(const LidarMetadata& theMetadata, double theX, double theY, double theZ, unsigned char theCls = 0)
		: mMetadata(&theMetadata), mCls(theCls)
	{
		//Initialize mCoords
		setFromRealCoords(wykobi::make_point(theX, theY, theZ));
	}

	LidarPoint(const LidarMetadata& theMetadata, const liblas::Point& theLasPoint)	
		: mMetadata(&theMetadata),
		  mCoords(wykobi::make_point(static_cast<long>(theLasPoint.GetRawX()), 
			static_cast<long>(theLasPoint.GetRawY()), 
			static_cast<long>(theLasPoint.GetRawZ()))),
		  mCls(theLasPoint.GetClassification().GetClass())
	{
	}

	inline wykobi::point3d<long> coords() const
	{
		return mCoords;
	}

	inline wykobi::point3d<double> realCoords() const
	{
		return wykobi::make_point(mCoords.x * mMetadata->scales().x + mMetadata->offsets().x, 
			mCoords.y * mMetadata->scales().y + mMetadata->offsets().y,
			mCoords.z * mMetadata->scales().z + mMetadata->offsets().z);
	}

	void setFromRealCoords(wykobi::point3d<double> realPoint);

	inline unsigned char classification() const
	{
		return mCls;
	}

	inline void setClassification(unsigned char theCls)
	{
		mCls = theCls;
	}

	inline void setZ(long theZ)
	{
		mCoords.z = theZ;
	}

	void populateLasPoint(liblas::Point& lasPoint) const;

private:

	const LidarMetadata* mMetadata;

	wykobi::point3d<long> mCoords;
	
	unsigned char mCls;
};

/// Comapares Z coordinate of points. Used for sorting points by Z.
/// \param p1 vector iterator to first point 
/// \param p2 vector iterator to second point
/// \return true if p1 has smaller Z than p2
inline bool compareZ(LidarPoint::VectorIterator p1, LidarPoint::VectorIterator p2)
{
	return ((*p1).coords().z < (*p2).coords().z);
}

}
} // namespace terrace::lidar

#endif // TERRACE_LIDARPOINT_HPP_INCLUDED