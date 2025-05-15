// PACKAGE		: Conversion
// FILE	        : Geodesic.cpp
//
// AUTHOR	: Marina Mukhortova, 2003
// DESCRIPTION		:Implementation file for CGeodesic class

#include "Geodesic.h"
#include "Geoline.h"
//#include "ConstRecalc.h"

// PACKAGE		: Conversion
// FUNCTION	: CGeodesic::CGeodesic
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: void
//
CGeodesic::CGeodesic( void )
{

}


// PACKAGE		: Conversion
// FUNCTION	: CGeodesic::~CGeodesic
//
// DESCRIPTION	: Destructor
//
// INPUTS 	: void
//
CGeodesic::~CGeodesic( void )
{
}


// PACKAGE		: Conversion
// FUNCTION	: CGeodesic::CGeodesic
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: dLatitude - Latitude radians;
//                dLongitude - Longitude radians ;
//                dAltitude - Altitude;
//
CGeodesic::CGeodesic( double dLatitude,
                      double dLongitude,
                      double dAltitude)
{
    m_dLatitude = dLatitude;
    m_dLongitude = dLongitude;
    m_dAltitude = dAltitude;
}


// PACKAGE		: Conversion
// FUNCTION	    : CGeodesic::operator=
//
// DESCRIPTION	: Overdetermination of operator "="
//
// INPUTS 	    : first struct;
//
// RETURNS	    : second struct;
//
CGeodesic& CGeodesic::operator= (const CGeodesic& sGeodesic)
{
    m_dLatitude = sGeodesic.m_dLatitude;
    m_dLongitude = sGeodesic.m_dLongitude;
    m_dAltitude = sGeodesic.m_dAltitude;
    return *this;
}

// PACKAGE		: Conversion
// FUNCTION	    : bool operator ==
//
// DESCRIPTION	: Overdetermination of operator "=="
//
// INPUTS 	    : first struct;
//
// RETURNS	    : Sign is true if equal, otherwise false;
//
bool CGeodesic::operator == (const CGeodesic& toCompare) const
{
    if( m_dLatitude  == toCompare.m_dLatitude  &&
            m_dLongitude == toCompare.m_dLongitude &&
            m_dAltitude  == toCompare.m_dAltitude )
    {
        return true;
    }

    return false;
}

// PACKAGE		: Conversion
// FUNCTION	    : CGeodesic::operator !=
//
// DESCRIPTION	: Overdetermination of operator "!="
//
// INPUTS 	    : first struct;
//
// RETURNS	    : Sign is false if equal, otherwise true;
//
bool CGeodesic::operator != (const CGeodesic& toCompare) const
{
    if( m_dLatitude  != toCompare.m_dLatitude  ||
            m_dLongitude != toCompare.m_dLongitude ||
            m_dAltitude  != toCompare.m_dAltitude )
    {
        return true;
    }

    return false;
}


// PACKAGE		: Conversion
// FUNCTION	    : CGeodesic::init
// DESCRIPTION	: initialization with given values
// INPUTS 	    : Latitide (radians), longitude (radians), altitude (meters)
// RETURNS	    : None
void CGeodesic::init(double dLatitude, double dLongitude, double dAltitude)
{
    m_dLatitude = dLatitude;
    m_dLongitude = dLongitude;
    m_dAltitude = dAltitude;
}


// PACKAGE		: Conversion
// FUNCTION	    : CGeodesic::GetGeodesicLine
//
// DESCRIPTION	    : Get the length of the geodesic line.
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : sGeoStart - geodesic coordinates of the begin of the curve;
//                    sGeoEnd - geodesic coordinates of the enf of the curve; 
// RETURNS	    : m_dLength - the length of the geodesic line;
//
double CGeodesic::GetGeodesicLine( CGeodesic sGeoStart,
                                   CGeodesic sGeoEnd)
{
    CGEOLINE sGeoLine; // the structure of the geodesic line

    sGeoLine.CalculationGEOLINE(&sGeoStart, &sGeoEnd);

    return sGeoLine.m_dLength;
}


double CGeodesic::GetGeodesicLine(CGeodesic sGeoEnd)
{
    CGEOLINE sGeoLine; //the structore of the geodesic line

    sGeoLine.CalculationGEOLINE(this, &sGeoEnd);

    return sGeoLine.m_dLength;
}


/*
// 
// OPERATOR		: operator << 
//
// DESCRIPTION	: serialization saving
//
CArchive& operator << ( CArchive& ar, CGeodesic& sVal)
{
    if(ar.IsStoring()) // Determines whether the archive is storing
    {
        ar << sVal.m_dLatitude;
        ar << sVal.m_dLongitude;
        ar << sVal.m_dAltitude;
    }
    return(ar); // archive
}


//
// OPERATOR		: operator >> 
//
// DESCRIPTION	: serialization reading
//
CArchive& operator >> ( CArchive& ar, CGeodesic& sVal)
{
    if(!ar.IsStoring()) // Determines whether the archive is not storing
    {
        ar >> sVal.m_dLatitude;
        ar >> sVal.m_dLongitude;
        ar >> sVal.m_dAltitude;
    }
    return(ar); // archive
}
*/
