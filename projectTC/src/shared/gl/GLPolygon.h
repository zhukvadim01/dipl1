//PACKAGE		:  GL
//FILE			:  GLPolygon.h
//AUTHOR		:
//DESCRIPTION	:  Header file for GLPoligon class.

#ifndef _GLPolygon_h_
#define _GLPolygon_h_

#include "GLGeometry.h"
#include <vector>

// PACKAGE		:  GL
// CLASS		:  GLPolygon
//DESCRIPTION	:  Class "Polygons".  

class GLPolygon : public std::vector<GLPointDouble2D>
{
public:

    //PACKAGE		:  GL
    //FUNCTION		:  GLPolygon::GLPolygon( qint32 size )
    //DESCRIPTION	:  Class constructor.
    //INPUTS		:  size - Size if array of vertex coordinates.
    //RETURNS		:  NONE

    GLPolygon( qint32 size );

    //PACKAGE		:  GL
    //FUNCTION		:  GLPolygon::GLPolygon( void )
    //DESCRIPTION	:  Class constructor.
    //INPUTS		:  size - Size if array of vertex coordinates.
    //RETURNS		:  NONE

    GLPolygon();

    //PACKAGE		:  GL
    //FUNCTION		:  GLPolygon::~GLPolygon( void )
    //DESCRIPTION	:  Class destructor.
    //INPUTS		:  NONE
    //RETURNS		:  NONE

    virtual ~GLPolygon();

//PACKAGE		:  GL
//FUNCTION		:  bool GLPolygon::ptInPolygon( long x, long y ) 
//DESCRIPTION	:  The method checks whether the point is inside the polygon.
//INPUTS		:  x - Point coordinate.
//              :  y - Point coordinate.
//RETURNS       :  True, if point in polygon

	bool ptInPolygon( long x, long y ) const;

    bool ptBeforePolygon( double x, double y, qint16 course, qint32& DistanceToZone ) const ;
	
//PACKAGE		:  GL
//FUNCTION		:  qint32 GLPolygon::getSize(void)
//DESCRIPTION	:  Returns size of array.
//INPUTS		:  NONE
//RETURNS       :  Size of array

  qint32 getSize(void);
};

#endif //_GLPolygon_h_