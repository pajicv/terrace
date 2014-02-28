/******************************************************************************
 * lidarpoint.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Information that describes lidar data set. 
 *
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_LIDARMETADATA_HPP_INCLUDED
#define TERRACE_LIDARMETADATA_HPP_INCLUDED

///////////////////////////////////////////////////////////////////////////////
// Forward declared dependacies
namespace liblas
{
	class Header;
}
///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include "terracedefs.hpp"
#include "liblas/liblas.hpp"

///////////////////////////////////////////////////////////////////////////////
// Actual class

namespace terrace
{
namespace lidar
{

class LidarMetadata
{
private:
	/// Software which generated lidar file.
	/// It will always be "Terrace"
	const std::string mGeneratingSoftware;
	/// Number of points in Lidar data set
	unsigned long mNumberOfPoints;
	/// Bounds of Lidar data set
	mydefs::BoundingBox mBoundingBox;
	/// Offsets of coordinates
	wykobi::vector3d<double> mOffsets;
	/// Scales of coordinates
	wykobi::vector3d<double> mScales;

	//CHECK: if creation year and day are set by LASwriter
public:
	LidarMetadata() : 
		mGeneratingSoftware("Terrace"), 
		mNumberOfPoints(0),
		mBoundingBox(wykobi::make_box(double(0.0), double(0.0), double(0.0), 
									  double(0.0), double(0.0), double(0.0))),
		mOffsets(wykobi::make_vector(double(0.0), double(0.0), double(0.0))),
		mScales(wykobi::make_vector(double(0.01), double(0.01), double(0.01)))
	{
	}

	/// Set values of attributes with data from liblas::Header 
	/// \param theHdr	LAS header from which the values are taken
	void setFromLasHeader(const liblas::Header& theHdr); 

	/// Creates liblas::Header to be saved in file
	/// \return LAS header to be saved in file
	void populateLasHeader(liblas::Header& theHdr) const;

	inline std::string generatingSoftware() const
	{
		return mGeneratingSoftware;
	}

	inline unsigned long numberOfPoints() const
	{
		return mNumberOfPoints;
	}

	inline void setNumberOfPoints(unsigned long n)
	{
		mNumberOfPoints = n;
	}

	inline mydefs::BoundingBox boundingBox() const
	{
		return mBoundingBox;
	}

	inline void setBoundingBox(const mydefs::BoundingBox& bb)
	{
		mBoundingBox = bb;
	}

	inline wykobi::vector3d<double> offsets() const
	{
		return mOffsets;
	}
	
	inline wykobi::vector3d<double> scales() const
	{
		return mScales;
	}

}; // class LidarMetadata

}
} // namespace terrace::lidar

#endif // TERRACE_LIDARMETADATA_HPP_INCLUDED