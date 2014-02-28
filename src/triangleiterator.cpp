#include "triangleiterator.hpp"
#include "tin.hpp"

namespace terrace
{
namespace tin
{

TriangleIterator::TriangleIterator() : mTIN( NULL )
{
	mCurrentTriangle.tri = NULL;
	mCurrentTriangle.orient = 0;
}

TriangleIterator::TriangleIterator(const TIN * theTIN) : mTIN( const_cast<TIN *>(theTIN) )
{
	traversalinit( &(theTIN->mMesh->triangles) );
	mCurrentTriangle.tri = triangletraverse( theTIN->mMesh );
	mCurrentTriangle.orient = 0;
}

bool TriangleIterator::validTriangle() const
{
	return mCurrentTriangle.tri != NULL;
}

TriangleIterator& TriangleIterator::operator++()
{
	mCurrentTriangle.tri = triangletraverse( mTIN->mMesh );
	return *this;
}

const mydefs::Triangle3d TriangleIterator::operator*()
{
////////////////////////////////////////////////////////////////////////////////
int plus1mod3[3] = {1, 2, 0};
int minus1mod3[3] = {2, 0, 1};

#define org(otri, vertexptr)                                                  \
  vertexptr = (vertex) (otri).tri[plus1mod3[(otri).orient] + 3]

#define dest(otri, vertexptr)                                                 \
  vertexptr = (vertex) (otri).tri[minus1mod3[(otri).orient] + 3]

#define apex(otri, vertexptr)                                                 \
  vertexptr = (vertex) (otri).tri[(otri).orient + 3]
////////////////////////////////////////////////////////////////////////////////
	mydefs::Triangle3d triangle;
	TVertex v1;
	TVertex v2;
	TVertex v3;
	
	if( validTriangle() )
	{
		org(mCurrentTriangle, v1);
		dest(mCurrentTriangle, v2);
		apex(mCurrentTriangle, v3);
		triangle = wykobi::make_triangle( v1[0], v1[1], v1[2], 
			v2[0], v2[1], v2[2],
			v3[0], v3[1], v3[2]);
	}
	
	return triangle;
}

bool TriangleIterator::operator==(const TriangleIterator& theTI) const
{
	return mCurrentTriangle.tri == theTI.mCurrentTriangle.tri;
}

bool TriangleIterator::operator!=(const TriangleIterator& theTI) const
{
	return mCurrentTriangle.tri != theTI.mCurrentTriangle.tri;
}

}
}//namespace terrace::tin