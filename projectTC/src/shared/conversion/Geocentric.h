// AUTHOR	: Marina Mukhortova, 2003
#pragma once

// DESCRIPTION  : class of geocentric system of coordinates
class CGeocentric
{
public:
    double m_dX = 0.0;  // coordinate X
    double m_dY = 0.0;  // coordinate Y
    double m_dZ = 0.0;  // coordinate Z

    ~CGeocentric() = default;
    CGeocentric() = default;
    CGeocentric(double a_dX, double a_dY, double a_dZ)
        : m_dX(a_dX)
        , m_dY(a_dY)
        , m_dZ(a_dZ)
    {}
    CGeocentric(const CGeocentric&) = default;
    CGeocentric(CGeocentric&&) = default;
    CGeocentric& operator=(const CGeocentric&) = default;
    CGeocentric& operator=(CGeocentric&&) = default;

    void init(double m_dX, double m_dY, double m_dZ);
    double mod();  // calculate vector module value

    CGeocentric operator+(CGeocentric sGeocentric);
    CGeocentric operator-(CGeocentric sGeocentric);
    CGeocentric operator*(double dValue);
    CGeocentric operator/(double dValue);
    bool operator==(const CGeocentric& toCompare) const;
    bool operator!=(const CGeocentric& toCompare) const;
};
