// PACKAGE		: Conversion
// FILE          : Spherical.cpp
//
// AUTHOR	 : Marina Mukhortova,  2003
//
// DESCRIPTION		:Implementation file for CSpherical class

#include <cmath>

#include "Spherical.h"
#include "ConstRecalc.h"


// PACKAGE		: Conversion
// FUNCTION	: CSpherical::CSpherical
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: void
//
CSpherical::CSpherical( void )
{

}


// PACKAGE		: Conversion
// FUNCTION	: CSpherical::~CSpherical
//
// DESCRIPTION	: Destructor
//
// INPUTS 	: void
//
CSpherical::~CSpherical( void )
{
}


// PACKAGE		: Conversion
// FUNCTION	: CSpherical::CSpherical
//
// DESCRIPTION	: Constructor
//
// INPUTS 	: dR - coordinate R - slant range of the target 
//                dE - coordinate E - elevation angle 
//                dB - coordinate B - azimuth
//
CSpherical::CSpherical( double dR,
                        double dE,
                        double dB)
{
    m_dR = dR;
    m_dE = dE;
    m_dB = dB;
}


void CSpherical::init(double dR, double dE, double dB)
{
    m_dR = dR;
    m_dE = dE;
    m_dB = dB;
}


double CSpherical::getHeight(const double &SrcHeight)
{
    double ResHeight = 0;
    if (m_dR < cdSPHERIC_R_MIN ||
            m_dR > cdSPHERIC_R_MAX ||
            m_dE < cdSPHERIC_E_MIN ||
            m_dE > cdSPHERIC_E_MAX ||
            m_dB < cdSPHERIC_B_MIN ||
            m_dB > cdSPHERIC_B_MAX ||
            SrcHeight < cd_H_MIN ||
            SrcHeight > cd_H_MAX)
    {
        ResHeight = -1;
    }
    else
    {
        double r1 = cdMainRadius + SrcHeight + m_dR*sin(m_dE);
        double p1 = m_dR*cos(m_dE);
        ResHeight = sqrt(r1*r1 + p1*p1) - cdMainRadius;
    }
    return ResHeight;
}
