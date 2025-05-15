// PACKAGE		: Conversion
// FILE		: Topocentric.h   
//
// AUTHOR	: Marina Mukhortova, 2003
// DESCRIPTION		:Header file for CTopocentric class

//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __TOPOCENTRIC_H__
#define __TOPOCENTRIC_H__


// PACKAGE		: Conversion
// CLASS	    : CTopocentric
//
// DESCRIPTION      : class of topocentric system of coordinates
//
class   CTopocentric
{
public:
    double m_dXt{0.}; // coordinate Xt, m
    double m_dYt{0.}; // coordinate Yt, m
    double m_dZt{0.}; // coordinate Zt, m

    CTopocentric( void ); // constructor

    ~CTopocentric( void ); // destructor

    CTopocentric( double m_dXt, double m_dYt, double m_dZt); // constructor

    CTopocentric( const CTopocentric& _posTopo);

    double mod( ) const;          //calculate vector module  value

    // PACKAGE		: Conversion
    // FUNCTION	    : CTopocentric::init
    // DESCRIPTION	: Initialization with given values
    // INPUTS 	    : Topocentric coordinates X, Y, Z (meters)
    // RETURNS	    : None
    void init(double dXt, double dYt, double dZt);

    // *** Overdetermination of operator "=" ***
    CTopocentric& operator=(const CTopocentric& sTopo);

    // *** Overdetermination of operator "+" ***
    CTopocentric operator+(const CTopocentric& sTopo) const;

    // *** Overdetermination of operator "+=" ***
    CTopocentric& operator+=(const CTopocentric& sTopo);

    // *** Overdetermination of operator "-" ***
    CTopocentric operator-(const CTopocentric& sTopo) const;

    // *** Overdetermination of operator "-=" ***
    CTopocentric& operator-=(const CTopocentric& sTopo);

    // *** Overdetermination of operator "*" ***
    CTopocentric operator*(const double mult) const;

    // *** Overdetermination of operator "*" ***
    CTopocentric& operator*=(const double mult);

};

//#include "Topocentric.inl"

#endif
