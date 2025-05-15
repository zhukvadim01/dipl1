// AUTHOR	 : Marina Mukhortova, 2003

#include "Geocentric.h"
#include <cmath>

void CGeocentric::init(double dX, double dY, double dZ)
{
    m_dX = dX;
    m_dY = dY;
    m_dZ = dZ;
}

double CGeocentric::mod()
{
    return sqrt(m_dX * m_dX +
                m_dY * m_dY +
                m_dZ * m_dZ);
}

CGeocentric CGeocentric::operator+(CGeocentric sGeocentric)
{
    return CGeocentric(m_dX + sGeocentric.m_dX,
                       m_dY + sGeocentric.m_dY,
                       m_dZ + sGeocentric.m_dZ);
}

CGeocentric CGeocentric::operator-(CGeocentric sGeocentric)
{
    return CGeocentric(m_dX - sGeocentric.m_dX,
                       m_dY - sGeocentric.m_dY,
                       m_dZ - sGeocentric.m_dZ);
}

CGeocentric CGeocentric::operator*(double dValue)
{
    return CGeocentric(m_dX * dValue,
                       m_dY * dValue,
                       m_dZ * dValue);
}

CGeocentric CGeocentric::operator/(double dValue)
{
    return CGeocentric(m_dX / dValue,
                       m_dY / dValue,
                       m_dZ / dValue);
}

bool CGeocentric::operator==(const CGeocentric& toCompare) const
{
    if (   fabs(m_dX - toCompare.m_dX) < 1.
        && fabs(m_dY - toCompare.m_dY) < 1.
        && fabs(m_dZ - toCompare.m_dZ) < 1.)
    {
        return true;
    }
    return false;
}

bool CGeocentric::operator!=(const CGeocentric& toCompare) const
{
    return !operator==(toCompare);
}
