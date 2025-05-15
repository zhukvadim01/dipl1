// PACKAGE		: Conversion
// FILE		: Topographic.h
//
// AUTHOR	: Lapushynski Aliaksandr, 2015
// DESCRIPTION		:Implementation file for CTopographic class

#include "Topographic.h"
#include "Altitude.h"


// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::CTopocentric
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: void
// RETURNS      : None
CTopographic::CTopographic( void )
{

}


// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::~CTopocentric
//
// DESCRIPTION	: Destructor
//
// INPUTS 	: void
// RETURNS      : None
CTopographic::~CTopographic( void )
{
}


// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::CTopocentric
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: dXt - coordinate Xt;
//                dHt - height;
//                dZt - coordinate Zt;
// RETURNS      : None
CTopographic::CTopographic( double dXt,
                            double dZt,
                            double dHt)
{
    m_dXt = dXt;
    m_dHt = dHt;
    m_dZt = dZt;
}

// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::CTopocentric
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: _posTopo - Topographic coordinate;
// RETURNS      : None
CTopographic::CTopographic(const CTopographic &_posTopo)
{
    m_dXt = _posTopo.m_dXt;
    m_dHt = _posTopo.m_dHt;
    m_dZt = _posTopo.m_dZt;
}
