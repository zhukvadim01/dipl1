// PACKAGE		: Conversion
// FILE		: Topocentric.cpp
//
// AUTHOR	: Marina Mukhortova, 2003
//
// DESCRIPTION		:Implementation file for CTopocentric class

#include "Topocentric.h"
#include "Altitude.h"
#include "math.h"

// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::CTopocentric
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: void
//
CTopocentric::CTopocentric( void )
{

}



// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::~CTopocentric
//
// DESCRIPTION	: Destructor
//
// INPUTS 	: void
//
CTopocentric::~CTopocentric( void )
{
}


// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::CTopocentric
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: dXt - coordinate Xt;
//                dYt - coordinate Yt;
//                dZt - coordinate Zt;
//
CTopocentric::CTopocentric( double dXt,
                            double dYt,
                            double dZt)
{
    m_dXt = dXt;
    m_dYt = dYt;
    m_dZt = dZt;
}

CTopocentric::CTopocentric(const CTopocentric& _posTopo)
{
    m_dXt = _posTopo.m_dXt;
    m_dYt = _posTopo.m_dYt;
    m_dZt = _posTopo.m_dZt;
}


void CTopocentric::init(double dXt, double dYt, double dZt)
{
    m_dXt = dXt;
    m_dYt = dYt;
    m_dZt = dZt;
}

// PACKAGE		: Conversion
// FUNCTION	: CTopocentric::mod
//
// DESCRIPTION	: vector module
//
// INPUTS 	:
//
// RETURNS	    : vector module value;
double CTopocentric::mod( ) const
{
    return sqrt(m_dXt*m_dXt + m_dYt*m_dYt + m_dZt*m_dZt);
}

CTopocentric& CTopocentric::operator=(const CTopocentric& sTopo)
{
    m_dXt = sTopo.m_dXt;
    m_dYt = sTopo.m_dYt;
    m_dZt = sTopo.m_dZt;
    return *this;
}
CTopocentric CTopocentric::operator+(const CTopocentric& sTopo) const
{
    CTopocentric l_data(*this);
    l_data.m_dXt += sTopo.m_dXt;
    l_data.m_dYt += sTopo.m_dYt;
    l_data.m_dZt += sTopo.m_dZt;
    return l_data;
}
CTopocentric& CTopocentric::operator+=(const CTopocentric& sTopo)
{
    this->m_dXt += sTopo.m_dXt;
    this->m_dYt += sTopo.m_dYt;
    this->m_dZt += sTopo.m_dZt;
    return *this;
}
CTopocentric CTopocentric::operator-(const CTopocentric& sTopo) const
{
    CTopocentric l_data(*this);
    l_data.m_dXt -= sTopo.m_dXt;
    l_data.m_dYt -= sTopo.m_dYt;
    l_data.m_dZt -= sTopo.m_dZt;
    return l_data;
}
CTopocentric& CTopocentric::operator-=(const CTopocentric& sTopo)
{
    this->m_dXt -= sTopo.m_dXt;
    this->m_dYt -= sTopo.m_dYt;
    this->m_dZt -= sTopo.m_dZt;
    return *this;
}
CTopocentric CTopocentric::operator*(const double mult) const
{
    CTopocentric l_data(*this);
    l_data.m_dXt *= mult;
    l_data.m_dYt *= mult;
    l_data.m_dZt *= mult;
    return l_data;
}
CTopocentric& CTopocentric::operator*=(const double mult)
{
    this->m_dXt *= mult;
    this->m_dYt *= mult;
    this->m_dZt *= mult;
    return *this;
}
