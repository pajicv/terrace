/******************************************************************************
 * lidarpoint.hpp
 *
 *
 ******************************************************************************
 * Copyright (c) 2013, Vladimir Pajic
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Included dependacies

#include "lidardataset.hpp"
#include "lidarpoint.hpp"
#include "lidarmetadata.hpp"
#include "groundclassifier1.hpp"

#include "liblas\liblas.hpp"

#include <fstream>  
#include <iostream>
#include <ios>
#include <map>
#include <string>
#include <sstream>

namespace terrace
{
namespace lidar
{

bool LidarDataset::load(const std::string& theSource)
{
	if( !mLoaded )
	{
		std::ifstream ifs;
		ifs.open(theSource.c_str(), std::ios::in | std::ios::binary);

		try 
		{
			liblas::ReaderFactory f;
			liblas::Reader reader = f.CreateWithStream(ifs);
				
			mSource = theSource;
			mMetadata.setFromLasHeader(reader.GetHeader());
			mPoints.reserve(mMetadata.numberOfPoints());
				
			for(unsigned long i = 0; i < mMetadata.numberOfPoints(); ++i)
			{
				reader.ReadNextPoint();
				mPoints.push_back( LidarPoint(mMetadata, reader.GetPoint()) );
			}

			std::cout << "Creating grid index.\n";

			mGridIndex.create(mMetadata, mPoints, 1);

			std::cout << "Estimating point density.\n";

			double pointDensity = estimateDensity();
			double pointSpacing = std::sqrt(1 / pointDensity);

			std::cout << "Point density is " << pointDensity 
				<< "\nPoint spacing is " << pointSpacing
				<< "\nRegenerating grid index with cell size of " << pointSpacing
				<< "\n";

			mGridIndex.create(mMetadata, mPoints, pointSpacing);

			std::cout << "Done.\n";

			mLoaded = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "Error: " << e.what() << std::endl;
		}
	}
	else
	{
		std::cerr << "Info: Cannot load more than one file." << std::endl;
	}
	return mLoaded;
}

//true if everything ok, false otherwise
bool extractCoordsFromString(const std::string& line, wykobi::point3d<double>& coords)
{
	bool result = false;
	// Extract x, y, and z strings
	std::istringstream lineStream(line);
	std::vector<std::string*> fields;
	int i = 0;
	while(!lineStream.eof())
	{
		std::string* field = new std::string;
		std::getline(lineStream, *field, '\t');
		fields.push_back(field);
	}

	// Convert from strings to values
	std::istringstream fieldStream;
	if(fields.size() == 3)
	{
		fieldStream.str(*fields[0]);
		if(fieldStream >> coords.x)
		{	
			fieldStream.str(*fields[1]);
			fieldStream.seekg(0, fieldStream.beg);
			if(fieldStream >> coords.y)
			{
				fieldStream.str(*fields[2]);
				fieldStream.seekg(0, fieldStream.beg);
				if(fieldStream >> coords.z)
				{
						result = true;
				}
			}
		}
	}

	if(!result)
	{
		std::cout << "ERROR: Parsing line \"" << line << "\"\n";
	}
	for(std::vector<std::string*>::iterator it = fields.begin();
		it != fields.end();
		++it)
	{
		delete *it;
	}

	return result;
}

bool LidarDataset::loadFromXyz(const std::string& theSource)
{
	double minx = std::numeric_limits<double>::max();
	double maxx = -1 * std::numeric_limits<double>::max();
	double miny = std::numeric_limits<double>::max();
	double maxy = -1 * std::numeric_limits<double>::max();
	double minz = std::numeric_limits<double>::max();
	double maxz = -1 * std::numeric_limits<double>::max();;

	std::ifstream fileStream;
	fileStream.open(theSource.c_str(), std::ifstream::in);
	
	if(!fileStream.good())
	{
		return false;
	}

	while(!fileStream.eof())
	{
		std::string line;
		std::getline(fileStream, line);

		wykobi::point3d<double> coords;
		if(extractCoordsFromString(line, coords))
		{
			LidarPoint lidarPoint(mMetadata, coords);
			mPoints.push_back(lidarPoint);
			
			// Update bounds
			if(coords.x < minx)
			{
				minx = coords.x;
			}
			else
			{
				if(coords.x > maxx)
				{
					maxx = coords.x;
				}
			}

			if(coords.y < miny)
			{
				miny = coords.y;
			}
			else
			{
				if(coords.y > maxy)
				{
					maxy = coords.y;
				}
			}

			if(coords.z < minz)
			{
				minz = coords.z;
			}
			else
			{
				if(coords.z > maxz)
				{
					maxz = coords.z;
				}
			}
		}
	}
	
	fileStream.close();

	mMetadata.setNumberOfPoints(mPoints.size());
	mMetadata.setBoundingBox(wykobi::make_box(minx, miny, minz, maxx, maxy, maxz));

	return true;

}

bool LidarDataset::saveAs(const std::string& theDestination) const
{
	bool result = false;

	if( mLoaded )
	{
		if (mSource.compare(theDestination) != 0)
		{
			try
			{
				std::ofstream ofs;
				liblas::Create(ofs, theDestination);
				// Create LAS header (version 1.2) from LidarMetadata
				liblas::Header lasHeader;
				lasHeader.SetCompressed(false);
				mMetadata.populateLasHeader(lasHeader);

				liblas::Writer writer(ofs, lasHeader);

				std::vector<LidarPoint>::const_iterator it;
				for(it = mPoints.begin(); it != mPoints.end(); ++it)
				{
					// Create LAS point (format 3) and populate 
					// with values from LidarPoint 
					liblas::Point lasPoint; 
					(*it).populateLasPoint(lasPoint);
					writer.WritePoint(lasPoint);
				}
				
				result = true;
				
				ofs.close();
			}
			catch (const std::exception& e)
			{
				std::cerr << "Error: " << e.what() << std::endl;
			}
		}
		else
		{
			std::cerr << "Info: Cannot save over the source file." << std::endl;
		}
	}
	else
	{
		std::cerr << "Info: Cannot save empty lidar data set." << std::endl;
	}

	return result;

}

double LidarDataset::estimateDensity() const
{
	std::map<unsigned int, unsigned int> histogram;
	std::map<unsigned int, unsigned int>::iterator histogramIt;

	for(std::vector< GridIndex::Cell >::const_iterator cellsIt = mGridIndex.cells()->begin(); cellsIt != mGridIndex.cells()->end(); ++cellsIt)
	{
		if( !(*cellsIt).empty() )
		{
			histogramIt = histogram.find((*cellsIt).size());
			
			if(histogramIt == histogram.end())
			{
				histogram[(*cellsIt).size()] = 1;
			}
			else
			{
				histogramIt->second++;
			}
		}
	}	

	double area = 0;
	double cellArea = mGridIndex.cellSize() * mGridIndex.cellSize();
	unsigned int numberOfPoints = 0;
	for(histogramIt = histogram.begin(); histogramIt != histogram.end(); ++histogramIt)
	{
		area += histogramIt->second * cellArea;
		numberOfPoints += histogramIt->first * histogramIt->second;
	}

	return double(numberOfPoints) / area;
}

unsigned int LidarDataset::classifyGround(double blockSize, 
										  double angleTreshold,
										  double distanceTreshold, 
										  double edgeLengthTreshold)
{
	terrace::lidar::classification::GroundClassifier classifier(*this,  
																angleTreshold, 
																distanceTreshold, 
																edgeLengthTreshold);
	return classifier.classify();
}

}
} // namespace terrace::lidar

