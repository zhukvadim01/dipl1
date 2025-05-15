// PACKAGE		: Conversion
// FILE      : Spherical.h
//
// AUTHOR    : Marina Mukhortova, 2003
// DESCRIPTION		:Header file for CSpherical class
//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __SPHERICAL_H__
#define __SPHERICAL_H__


// PACKAGE		: Conversion
// CLASS          : CSpherical
//
// DESCRIPTION    : class of the spherical system of coordinates
//
class   CSpherical
{
public:

    double m_dR{0.}; // coordinate R - slant range of the target, meters
    double m_dE{0.}; // coordinate E - elevation angle, radians
    double m_dB{0.}; // coordinate B - azimuth, radians

    CSpherical( void ); // constructor

    ~CSpherical( void ); // destructor

    CSpherical( double m_dR, double m_dE, double m_dB); // constructor

    // PACKAGE		: Conversion
    // FUNCTION	    : CSpherical::init
    // DESCRIPTION	: Initialization with given values
    // INPUTS 	    : Range (meters), elevation angle (radians), azimuth (radians)
    // RETURNS	    : None
    void init(double dR, double dE, double dB);

    // PACKAGE		: Conversion
    // FUNCTION	    : CSpherical::getHeight
    // DESCRIPTION	: Returns height, in assumption that the Earth is spherical
    // INPUTS 	    : Height of source stand point, m
    // RETURNS	    : Height of air object, m
    double getHeight(const double &SrcHeight);
};
//#include "Spherical.inl"

#endif
