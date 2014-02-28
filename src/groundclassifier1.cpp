/******************************************************************************
 * groundclassifier.cpp
 *
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Included dependacies
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include "wykobi.hpp"
#include "groundclassifier1.hpp"
#include "gridindex.hpp"

using terrace::lidar::GridIndex;

namespace terrace
{
namespace lidar
{
namespace classification
{

GroundClassifier::GroundClassifier(LidarDataset& lidarDs, 
								   double angleTreshold,
								   double distanceTreshold, 
								   double edgeLengthTreshold) : 
mLidarDs(lidarDs), 
mTIN(0),
mPyramid(lidarDs), 
mAngleTreshold(angleTreshold * wykobi::PI / 180), // Convert to radians 
mDistanceTreshold(distanceTreshold),
mEdgeLengthTreshold(edgeLengthTreshold * edgeLengthTreshold) // Square edge length threshold. It will save few sqrt operations.
{
	mEdgeDistance = 10 * lidarDs.gridIndex().cellSize();
}

unsigned int GroundClassifier::classify()
{
	unsigned int iteration = 0;
	unsigned int index = 1;

	std::cout << "Performing ground classifiaction.\n";

	// Discard previous classification
	for(std::vector<LidarPoint>::iterator pointsIt = mLidarDs.points().begin();
		pointsIt != mLidarDs.points().end();
		++pointsIt)
	{
		(*pointsIt).setClassification(1);
	}

	std::cout << "Taking points from pyramid level " <<  mPyramid.levels.size() 
			  << " as initial ground points.\n";

	findInitialGroundPoints();

	//std::cout << "Found " << mClassifiedPoints.size() << " ground points.\n";

	std::vector<Pyramid::Level*>::reverse_iterator levelsIt = mPyramid.levels.rbegin();

	// Last level (lowest resolution) is already processed go to previous
	++levelsIt;

	for( ; levelsIt != mPyramid.levels.rend(); ++levelsIt)
	{
		mPyramid.currentLevel = *levelsIt;
		std::cout << "Densifying TIN. Taking points from pyramid level " 
				  << mPyramid.currentLevel->number << std::endl;

		createTIN(); 

		std::vector<LidarPoint::VectorIterator>::iterator cellsIt;
		for(cellsIt = (*levelsIt)->cells.begin(); cellsIt != (*levelsIt)->cells.end(); ++cellsIt)
		{
			if((*cellsIt) != mPyramid.empty)
			{
				wykobi::point3d<double> point = (*(*cellsIt)).realCoords();

				if(checkPoint(point))
				{
					mClassifiedPoints.push_back(*cellsIt);
				}
			}
		}
	}

	std::cout << "Classified " << mClassifiedPoints.size() << " ground points.\n";

	applyClassification();

	std::cout << "Finished ground classification.\n";

	return mClassifiedPoints.size();
}

GroundClassifier::Pyramid::Pyramid(LidarDataset& lidarDs)
{
	std::cout << "Creating pyramid levels.\n";

	empty = const_cast<std::vector<LidarPoint>&>(lidarDs.points()).end();

	// Level 0
	Pyramid::Level* firstLevel = new Pyramid::Level;
	firstLevel->number = 1;
	firstLevel->rows = lidarDs.gridIndex().rows();
	firstLevel->columns = lidarDs.gridIndex().columns();
	firstLevel->cells.reserve(firstLevel->rows * firstLevel->columns);
	firstLevel->cellSize = lidarDs.gridIndex().cellSize();

	for(unsigned int i = 0; i < firstLevel->rows; ++i)
	{
		for(unsigned int j = 0; j < firstLevel->columns; ++j)
		{
			unsigned int gridCellIndex = lidarDs.gridIndex().index(i ,j);
			GridIndex::Cell gridCell = lidarDs.gridIndex()[gridCellIndex];
			if(!gridCell.empty())
			{
				firstLevel->cells.push_back(gridCell.front());
			}
			else
			{
				firstLevel->cells.push_back(empty);
			}
		}
	}

	levels.push_back(firstLevel);

	std::cout << "Created level 1.\n";

	do
	{
		Pyramid::Level* previousLevel = levels.back();
		currentLevel = new Pyramid::Level;
		currentLevel->number = previousLevel->number + 1;
		currentLevel->cells.reserve((previousLevel->rows / 2 + 1)
									* (previousLevel->columns / 2 + 1));
		currentLevel->cellSize = previousLevel->cellSize * 2;

		// Other levels
		unsigned int i;
		unsigned int j;
		for(i = 0; i < previousLevel->rows; i += 2)
		{
			for(j = 0; j < previousLevel->columns; j += 2)
			{
				// 4(or 2 on border; or 1 in corner) previous level cells are inspected
				std::vector<LidarPoint::VectorIterator> cell;
				cell.push_back(previousLevel->cells[i * previousLevel->columns + j]);
				if(j + 1 < previousLevel->columns)
				{
					cell.push_back(previousLevel->cells[i * previousLevel->columns + (j + 1)]);
				}
				else
				{
					cell.push_back(empty);
				}
				if(i + 1 < previousLevel->rows)
				{
					cell.push_back(previousLevel->cells[(i + 1) * previousLevel->columns + j]);
				}
				else
				{
					cell.push_back(empty);
				}
				if((i + 1 < previousLevel->rows) && (j + 1 < previousLevel->columns))
				{
					cell.push_back(previousLevel->cells[(i + 1) * previousLevel->columns + (j + 1)]);
				}
				else
				{
					cell.push_back(empty);
				}
				
				// Find the point with lowest elevation among them
				LidarPoint::VectorIterator lowestPointIt = empty;
				unsigned int minp = 0;
				unsigned int maxq = 0;
				for(unsigned int p = 0; p < 2; ++p)
				{
					for(unsigned int q = 0; q < 2; ++q)
					{
						if(lowestPointIt == empty)
						{
							lowestPointIt = cell[p * 2 + q];
						}
						else
						{
							if(cell[p * 2 + q] != empty)
							{
								if((*cell[p * 2 + q]).realCoords().z < (*lowestPointIt).realCoords().z)
								{
									lowestPointIt = cell[p * 2 + q];
									minp = p;
									maxq = q;
								}
							}
						}
					}
				}

				// Promote cell to this level
				currentLevel->cells.push_back(lowestPointIt);

				// Assign empty value to cell in previous level
				previousLevel->cells[(i + minp) * previousLevel->columns + (j + maxq)] = empty;

			}
		}

		// Half the resolution for new level
		currentLevel->rows = i / 2;
		currentLevel->columns = j / 2;

		levels.push_back(currentLevel);

		std::cout << "Created level " << currentLevel->number << ".\n";

	}
	while(levels.back()->rows * levels.back()->columns > 100);

	currentLevel = levels.back();

	std::cout << "\n\n";
}

GroundClassifier::Pyramid::~Pyramid()
{
	for(std::vector<Level*>::iterator levelsIt= levels.begin();
		levelsIt != levels.end();
		++levelsIt)
	{
		delete (*levelsIt);
	}
}

unsigned int GroundClassifier::findInitialGroundPoints()
{
	for(std::vector<LidarPoint::VectorIterator>::iterator cellsIt = mPyramid.levels.back()->cells.begin(); 
		cellsIt != mPyramid.levels.back()->cells.end(); 
		++cellsIt)
	{
		if((*cellsIt) != mPyramid.empty)
		{
			mClassifiedPoints.push_back(*cellsIt);
		}
	}

	return mClassifiedPoints.size();
}

void GroundClassifier::createTIN()
{
	std::vector<wykobi::point3d<double>> points;

	std::vector<LidarPoint::VectorIterator>::iterator classPointsIt;
	for(classPointsIt = mClassifiedPoints.begin(); 
		classPointsIt != mClassifiedPoints.end();
		++classPointsIt)
	{
		points.push_back((*(*classPointsIt)).realCoords());
	}

	delete mTIN;
	mTIN = new TIN;
	mTIN->create(points);
}

double vectorToPlaneAngle(const wykobi::vector3d<double>& v, const wykobi::plane<double, 3>& p)
{
	double nom = p.normal.x * v.x + p.normal.y * v.y + p.normal.z * v.z;
	double denom1 = p.normal.x * p.normal.x + p.normal.y * p.normal.y + p.normal.z * p.normal.z;
	double denom2 = v.x * v.x + v.y * v.y + v.z * v.z;
	return std::asin(nom / std::sqrt(denom1 * denom2)); 
}

double distanceToPlane(const wykobi::point3d<double>& point, const wykobi::plane<double, 3>& plane)
{
	double d = (plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z + plane.constant) 
		/ std::sqrt(plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y + plane.normal.z * plane.normal.z);
	return d;
}

bool GroundClassifier::checkAngle(const wykobi::point3d<double>& point, const wykobi::triangle3d& triangle) const
{
	bool result = false; 

	wykobi::vector3d<double> vect = point - triangle[0];
	wykobi::plane<double, 3> triPlane = wykobi::make_plane(triangle);
	
	double reducedAngle = reduceAngle(triangle);

	if(vectorToPlaneAngle(vect, triPlane) < reducedAngle)
	{
		vect = point - triangle[1];
		if(vectorToPlaneAngle(vect, triPlane) < reducedAngle)
		{
			vect = point - triangle[2];
			if(vectorToPlaneAngle(vect, triPlane) < reducedAngle)
			{
				result = true;
			}
		}
	}

	return result;
	
}

bool GroundClassifier::checkMirrorAngle(const wykobi::point3d<double>& point, const wykobi::triangle3d& triangle) const
{
	bool result = false; 

	wykobi::vector3d<double> vect = point - triangle[0];
	wykobi::plane<double, 3> triPlane = wykobi::make_plane(triangle);
	
	double reducedAngle = reduceAngle(triangle);

	if(std::abs(vectorToPlaneAngle(vect, triPlane)) < reducedAngle)
	{
		vect = point - triangle[1];
		if(std::abs(vectorToPlaneAngle(vect, triPlane)) < reducedAngle)
		{
			vect = point - triangle[2];
			if(std::abs(vectorToPlaneAngle(vect, triPlane)) < reducedAngle)
			{
				result = true;
			}
		}
	}

	return result;
	
}


bool GroundClassifier::checkDistance(const wykobi::point3d<double>& point, const wykobi::triangle3d& triangle) const
{
	wykobi::plane<double, 3> triPlane = wykobi::make_plane(triangle);
	return distanceToPlane(point, triPlane) < mDistanceTreshold;
}

double  GroundClassifier::reduceAngle(const wykobi::triangle3d& triangle) const
{
	double result = mAngleTreshold;

	double edge1 = (triangle[1].x - triangle[0].x) * (triangle[1].x - triangle[0].x)
				 + (triangle[1].y - triangle[0].y) * (triangle[1].y - triangle[0].y)
				 + (triangle[1].z - triangle[0].z) * (triangle[1].z - triangle[0].z);

	if(edge1 < mEdgeLengthTreshold)
	{
		double edge2 = (triangle[2].x - triangle[1].x) * (triangle[2].x - triangle[1].x)
					 + (triangle[2].y - triangle[1].y) * (triangle[2].y - triangle[1].y)
					 + (triangle[2].z - triangle[1].z) * (triangle[2].z - triangle[1].z);

		if(edge2 < mEdgeLengthTreshold)
		{
			double edge3 = (triangle[0].x - triangle[2].x) * (triangle[0].x - triangle[0].x)
						 + (triangle[0].y - triangle[2].y) * (triangle[0].y - triangle[0].y)
						 + (triangle[0].z - triangle[2].z) * (triangle[0].z - triangle[0].z);

			if(edge3 < mEdgeLengthTreshold)
			{
				double longest = edge1;

				if(edge2 > edge3)
				{
					if(edge2 > longest)
					{
						longest = edge2;
					}
				}
				else
				{
					if(edge3 > longest)
					{
						longest = edge3;
					}
				}

				result *= (longest / mEdgeLengthTreshold) * (longest / mEdgeLengthTreshold); 
			}

		}
	}

	return result;

}

wykobi::point3d<double>* GroundClassifier::findMirrorPoint(const wykobi::point3d<double>& point, 
														   const wykobi::triangle<double, 3>& triangle)
{
	wykobi::vector3d<double> mirrorAxis;

	wykobi::vector3d<double> p2v1  = triangle[0] - point;
	wykobi::vector3d<double> p2v2  = triangle[1] - point;
	wykobi::vector3d<double> p2v3  = triangle[2] - point;

	mirrorAxis = p2v1;
	if(wykobi::vector_norm(p2v2) < wykobi::vector_norm(p2v1))
	{
		mirrorAxis = p2v2;

		if(wykobi::vector_norm(p2v3) < wykobi::vector_norm(p2v2))
		{
			mirrorAxis = p2v3;
		}
	}
	else
	{
		if(wykobi::vector_norm(p2v3) < wykobi::vector_norm(p2v1))
		{
			mirrorAxis = p2v3;
		}
	}

	wykobi::point3d<double>* mirror = 0;
	if(vector_norm(mirrorAxis) < mPyramid.currentLevel->cellSize)
	{
		mirrorAxis = mirrorAxis * double(2);
		mirror = new wykobi::point3d<double>;
		*mirror = point + mirrorAxis;
	}
	return mirror;
}

bool GroundClassifier::checkPoint(const wykobi::point3d<double>& point)
{
	bool result = false;

	wykobi::triangle<double, 3> triangle;

	mTIN->findTriangle(point.x, point.y, triangle);

	// Have to check this because Triangle in some cases returns wrong triangle 
	bool realyInTriangle = wykobi::point_in_triangle(point.x, point.y,
													 triangle[0].x, triangle[0].y,
													 triangle[1].x, triangle[1].y,
													 triangle[2].x, triangle[2].y);
	
	if(realyInTriangle)
	{
		if(checkAngle(point, triangle) && checkDistance(point, triangle))
		{
			result = true;
		}
	}

	if(!result)
	{
		wykobi::point3d<double>* mirrorPoint = findMirrorPoint(point, triangle);
		
		if(mirrorPoint != 0)
		{
			mTIN->findTriangle(mirrorPoint->x, mirrorPoint->y, triangle);

			// Have to check this because Triangle in some cases returns wrong triangle
			bool realyInTriangle = wykobi::point_in_triangle(mirrorPoint->x, mirrorPoint->y,
													 triangle[0].x, triangle[0].y,
													 triangle[1].x, triangle[1].y,
													 triangle[2].x, triangle[2].y);
			if(realyInTriangle)
			{
				if(checkMirrorAngle(*mirrorPoint, triangle) && checkDistance(*mirrorPoint, triangle))
				{
					result = true;
				}
			}

			delete mirrorPoint;
		}
	}

	return result;
			
}

void GroundClassifier::applyClassification()
{
	std::fstream out;
	out.open("C:\\Temp\\terrace_gnd.xyz", std::fstream::out);
	out << std::setprecision(10);
	std::vector<LidarPoint::VectorIterator>::iterator classPointsIt;
	for(classPointsIt = mClassifiedPoints.begin(); classPointsIt != mClassifiedPoints.end(); ++classPointsIt)
	{
		out << (*(*classPointsIt)).realCoords().x << "\t" 
			<< (*(*classPointsIt)).realCoords().y << "\t"
			<< (*(*classPointsIt)).realCoords().z << "\n";
		(*(*classPointsIt)).setClassification(2);
	}
	out.close();
}

}
}
} // namespace terrace::lidar::classification