// PACKAGE		: Conversion
// FILE		: Topographic.h
//
// AUTHOR	: Lapushynski Aliaksandr, 2015
// DESCRIPTION		:Header file for CTopographic class

#ifndef TOPOGRAPHIC_H
#define TOPOGRAPHIC_H


// PACKAGE		: Conversion
// CLASS	    : CTopocentric
//
// DESCRIPTION      : class of topocentric system of coordinates
//
class   CTopographic
{
public:
    double m_dXt{0.}; // coordinate Xt
    double m_dHt{0.}; // height
    double m_dZt{0.}; // coordinate Zt
    // PACKAGE		: Conversion
    // FUNCTION	: CTopocentric::CTopocentric
    //
    // DESCRIPTION	: Constructor
    //
    // INPUTS 	: void
    // RETURNS      : None
    CTopographic( void ); // constructor
    // PACKAGE		: Conversion
    // FUNCTION	: CTopocentric::~CTopocentric
    //
    // DESCRIPTION	: Destructor
    //
    // INPUTS 	: void
    // RETURNS      : None
    ~CTopographic( void ); // destructor
    // PACKAGE		: Conversion
    // FUNCTION	: CTopocentric::CTopocentric
    //
    // DESCRIPTION	: Constructor
    //
    // INPUTS 	: dXt - coordinate Xt;
    //                dHt - height;
    //                dZt - coordinate Zt;
    // RETURNS      : None
    CTopographic( double m_dXt, double m_dZt, double m_dHt); // constructor
    // PACKAGE		: Conversion
    // FUNCTION	: CTopocentric::CTopocentric
    //
    // DESCRIPTION	: Constructor
    //
    // INPUTS 	: _posTopo - Topographic coordinate;
    // RETURNS      : None
    CTopographic( const CTopographic& _posTopo);


};

#endif // TOPOGRAPHIC_H




