// PACKAGE		: Conversion
// FILE		: Geodesic.h
//
// AUTHOR	: Marina Mukhortova,  2003
// DESCRIPTION		:Header file for CGeodesic class
//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __GEODESIC_H__
#define __GEODESIC_H__

//#include "Geocentric.h"


// PACKAGE		: Conversion
// CLASS        : CGeodesic
//
// DESCRIPTION  : class of geodesic system of coordinates
//
class   CGeodesic
{
public:
    double m_dLatitude{0.};  // Latitude, radians
    double m_dLongitude{0.}; // Longitude, radians
    double m_dAltitude{0.};  // Altitude, meters

    CGeodesic( void ); // constructor

    ~CGeodesic( void ); // destructor

    CGeodesic( double dLatitude, double dLongitude, double dAltitude); // constructor

    // *** Overdetermination of operator "=" ***
    CGeodesic& operator= (const CGeodesic& sGeodesic);

    // *** Overdetermination of operator "==" ***
    bool operator == (const CGeodesic& toCompare) const;

    // *** Overdetermination of operator "!=" ***
    bool operator != (const CGeodesic& toCompare) const;

    // PACKAGE		: Conversion
    // FUNCTION	    : CGeodesic::init
    // DESCRIPTION	: Initialization with given values
    // INPUTS 	    : Latitide (radians), longitude (radians), altitude (meters)
    // RETURNS	    : None
    void init(double dLatitude, double dLongitude, double dAltitude);

    // Get geodesic line
    double GetGeodesicLine( CGeodesic sGeoStart,
                            CGeodesic sGeoEnd);

    double GetGeodesicLine(CGeodesic sGeoEnd);

    //  friend CArchive& operator << ( CArchive& ar, CGeodesic& sVal); // *** Serialization saving ***
    //  friend CArchive& operator >> ( CArchive& ar, CGeodesic& sVal); // *** Serialization reading ***

};

//#include "Geodesic.inl"

#endif
