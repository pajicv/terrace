/******************************************************************************
 * groundclassifier.hpp
 *
 * Project:  terrace - A library for processing of Lidar 
 *           data.
 * Purpose:  Implementation of algorithm for classification
 *           of ground points. It is based on progessive TIN 
 *           densification algorithm by Axelsson (2000). 
 *
 * Author:   Vladimir Pajic, pajicv@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

#ifndef TERRACE_GROUNDCLASSIFIER_HPP_INCLUDED
#define TERRACE_GROUNDCLASSIFIER_HPP_INCLUDED

///////////////////////////////////////////////////////////////////////////////
// Included dependacies
#include "lidardataset.hpp"
#include "lidarpoint.hpp"
#include "tin.hpp"
#include "wykobi.hpp"

using terrace::lidar::LidarDataset;
using terrace::lidar::LidarPoint;
using terrace::tin::TIN;

namespace terrace
{
namespace lidar
{
namespace classification
{

///
/// Implementation of algorithm for classification
/// of ground points. It is based on progessive TIN 
/// densification algorithm by Axelsson (2000).
///
class GroundClassifier
{
public:

	struct Pyramid
	{
	public:

		class Level
		{
		public:
			std::vector<LidarPoint::VectorIterator> cells;
			unsigned int number;
			double cellSize;
			unsigned int rows;
			unsigned int columns;
		};

		std::vector<Level *> levels;
		Level* currentLevel;
		LidarPoint::VectorIterator empty;

		Pyramid(LidarDataset& lidarDs);
		~Pyramid();
	};

	/// Constructor
	GroundClassifier(LidarDataset& lidarDs, double angleTreshold,
		double distanceTreshold, double edgeLengthTreshold);

	/// Destructor
	~GroundClassifier()
	{
		delete mTIN;
	}

	/// Classifies ground points
	unsigned int classify();

private:

	/// Create pyramid levels of lidar points. The lowest points
	/// are on top and they are used to create initital TIN.
	void createPyramid();

	/// Find initial ground points (one per block)
	unsigned int findInitialGroundPoints();

	/// Creates tin and assigns it to mTIN
	void createTIN();

	/// Check angle constraint
	bool checkAngle(const wykobi::point3d<double>& point, const wykobi::triangle3d& triangle) const;

	/// Check angle for mirrored point (absolute value is checked)
	bool checkMirrorAngle(const wykobi::point3d<double>& point, const wykobi::triangle3d& triangle) const;

	/// Check distance constraint
	bool checkDistance(const wykobi::point3d<double>& point, const wykobi::triangle3d& triangle) const;

	/// Reduce angle if longest triangle edge is shorter than mEdgeLengthTreshold
	double reduceAngle(const wykobi::triangle3d& triangle) const;

	/// Find mirror point
	wykobi::point3d<double>* findMirrorPoint(const wykobi::point3d<double>& point, 
		const wykobi::triangle<double, 3>& triangle);

	/// Checks angle and distance constraints for point.
	/// If those are not satisfied finds mirror point and 
	/// checks constraint for mirrored point. 
	/// If point or mirrored point meets costraints returns true
	bool checkPoint(const wykobi::point3d<double>& point);

	/// Set classification for lidar points classified as ground
	void applyClassification();
	
	/// Pointer to LidarDataset being classified
	LidarDataset& mLidarDs;

	/// TIN of ground points
	TIN* mTIN;

	/// Classified points
	std::vector< LidarPoint::VectorIterator > mClassifiedPoints;

	Pyramid mPyramid;

	//============= Algorithm parameters ===========

	double mAngleTreshold;

	double mDistanceTreshold;

	double mEdgeLengthTreshold;

	double mEdgeDistance;
};

}
}
} // namespace terrace::lidar::classification

#endif //TERRACE_GROUNDCLASSIFIER_HPP_INCLUDED