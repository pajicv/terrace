#ifndef TERRACE_GEORASTER_HPP_INCLUDED
#define TERRACE_GEORASTER_HPP_INCLUDED

#include <vector>
#include <string>
#include <typeinfo>

#include "gdal_priv.h"

namespace terrace
{
namespace georaster
{

template <class T>
class Georaster
{
public:
	typedef std::vector<T> Band;
	typedef std::vector<Band> Image;
	
	struct Georeference
	{
	public:	
		double x0;
		double dx;
		double rotx;
		double y0;
		double roty;
		double dy;

		Georeference() : x0(0.0), 
						 dx(0.0), 
						 rotx(0.0), 
						 y0(0.0), 
						 roty(0.0), 
						 dy(0.0)
		{
		}

	};

	Georaster() : mBands(0), 
				  mRows(0), 
				  mColumns(0), 
				  mImage(), 
				  mGeoref()
	{
	}

	bool create(unsigned int bands, 
				unsigned int rows, 
				unsigned int columns, 
				const Image& img, 
				const Georeference& georef);

	bool saveAs(const std::string& filename);

private:

	GDALDataType determineDataType() const;

	unsigned int mBands;
	unsigned int mRows;
	unsigned int mColumns;
	Image mImage;
	Georeference mGeoref;
};

///////////////////////////////////////////////////////////////////////////////
// Implementation

template <class T>
GDALDataType Georaster<T>::determineDataType() const
{
	GDALDataType result = GDT_Unknown;
	if(typeid(T) == typeid(unsigned char))
	{
		result = GDT_Byte;
	}
	else if(typeid(T) == typeid(unsigned short))
	{
		result = GDT_UInt16;
	}
	else if(typeid(T) == typeid(short))
	{
		result = GDT_Int16;
	}
	else if(typeid(T) == typeid(unsigned int))
	{
		result = GDT_UInt32;
	}
	else if(typeid(T) == typeid(int))
	{
		result = GDT_Int32;
	}
	else if(typeid(T) == typeid(float))
	{
		result = GDT_Float32;
	}
	else if(typeid(T) == typeid(double))
	{
		result = GDT_Float64;
	}
	return result;
}

template <class T>
bool Georaster<T>::create(unsigned int bands, 
					      unsigned int rows, 
					      unsigned int columns, 
					      const Image& img, 
					      const Georeference& georef)
{
	bool result = true;

	if(img.size() == bands)
	{
     	Georaster::Image::const_iterator imgIt; 
	    for(imgIt = img.begin(); imgIt != img.end(); ++imgIt)
	    {
			if((*imgIt).size() != rows * columns)
			{
				result = false;
				break;
			}
	    }
	}
	else
	{
		result = false;
	}

	if(result)
	{
		mBands = bands;
		mRows = rows;
		mColumns = columns;
		mImage = img;
		mGeoref = georef;
	}

	return result;
}

template <class T>
bool Georaster<T>::saveAs(const std::string& filename)
{
	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");

	if( driver == NULL )
        return false;

	//set options
	char ** options = NULL;
	options = CSLSetNameValue( options, "COMPRESS", "LZW" );

	//set correct compression options
	GDALDataType dt = determineDataType();
	if( dt == GDT_Byte )
	{
		options = CSLSetNameValue( options, "PREDICTOR", "1" );
	}
	else
	{
		if ( dt == GDT_UInt16 || dt == GDT_Int16  
			|| dt == GDT_UInt32 || dt == GDT_Int32 )
		{
			options = CSLSetNameValue( options, "PREDICTOR", "2" );
		} 
		else 
		{
			options = CSLSetNameValue( options, "PREDICTOR", "3" );
		}
	}

	//set filename
	GDALDataset* dstDS = driver->Create(filename.c_str(), 
										mColumns, 
										mRows, 
										mBands, 
										determineDataType(), 
										options);

	//set geotransform
	double geotransform[6];
	geotransform[0] = mGeoref.x0;
	geotransform[1] = mGeoref.dx;
	geotransform[2] = mGeoref.rotx;
	geotransform[3] = mGeoref.y0;
	geotransform[4] = mGeoref.roty;
	geotransform[5] = mGeoref.dy;

	dstDS->SetGeoTransform(geotransform);

	GDALRasterBand * band;

	for( unsigned int i=1; i <= mBands; i++ )
	{
		band = dstDS->GetRasterBand( i );

		T* data = &(mImage[i-1][0]);

		band->RasterIO( GF_Write, 0, 0, mColumns, mRows, static_cast<void *>(data), 
						mColumns, mRows, determineDataType(), 0, 0);

	}

	GDALClose( (GDALDatasetH) dstDS );

	return true;
}

}
} // namespace terrace::raster

#endif //TERRACE_RASTER_HPP_INCLUDED