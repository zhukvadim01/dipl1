// PACKAGE		: Conversion
// FILE        : Geoline.cpp
//
// AUTHOR      : Marina Mukhortova, 2003
// DESCRIPTION		:Implementation file for CGEOLINE class

#include "Geoline.h"
#include "ConstRecalc.h"
#include "Geodesic.h"

#include <cmath>
//#include <cstdio>
//#include <cerrno>



// PACKAGE		: Conversion
// FUNCTION	    : CGEOLINE::CalculationGEOLINE
//
// DESCRIPTION      : The function of calculation of the length of the geodesic line
//                    All the parameters are in radians (angles) and metres (distanses).
//               
// INPUTS 	    : pGEO1 - geodesic coordinates of the first point;
//                    pGEO2 - geodesic coordinates of the second point;
//               
// RETURNS	    : m_dLength
//
bool CGEOLINE::CalculationGEOLINE (const CGeodesic *pGEO1, const CGeodesic *pGEO2)
{
    if((!pGEO1) ||
            (!pGEO2) ||
            (pGEO1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO1->m_dAltitude < cd_H_MIN) ||
            (pGEO1->m_dAltitude > cd_H_MAX) ||
            (pGEO2->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO2->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO2->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO2->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO2->m_dAltitude < cd_H_MIN) ||
            (pGEO2->m_dAltitude > cd_H_MAX))
    {
        return false;
    }

    // Intermediate variables
    double dW1;
    double dW2;
    double dSinU1;
    double dSinU2;
    double dCosU1;
    double dCosU2;
    double da1;
    double da2;
    double db1;
    double db2;
    double dp;
    double dq;
    double dSinSIG;
    double dCosSIG;
    double dA0;
    double dX;
    double dY;
    double dA;
    double dB;
    double dC;

    double dSIG;              // Spherical distance
    double dA1;               // Start azimuth
    double dDifferenceLongit; // The difference between longitudes


    // Intermediate variables
    // Comment Goga 12.02.2007 dW1 = sqrt(1-cdFirstEccentricity*pow(sin(pGEO1->m_dLatitude),2));
    // Comment Goga 12.02.2007 dW2 = sqrt(1-cdFirstEccentricity*pow(sin(pGEO2->m_dLatitude),2));

    dW1 = sqrt(1-cdFirstEccentricity*sin(pGEO1->m_dLatitude)); // Goga 12.02.2007
    dW2 = sqrt(1-cdFirstEccentricity*sin(pGEO2->m_dLatitude)); // Goga 12.02.2007

    dSinU1 = ((sin(pGEO1->m_dLatitude)*sqrt(1-cdFirstEccentricity))/dW1);
    dSinU2 = ((sin(pGEO2->m_dLatitude)*sqrt(1-cdFirstEccentricity))/dW2);

    dCosU1 = cos(pGEO1->m_dLatitude)/dW1;
    dCosU2 = cos(pGEO2->m_dLatitude)/dW2;

    dDifferenceLongit = pGEO2->m_dLongitude-pGEO1->m_dLongitude;

    da1 = dSinU1*dSinU2;
    da2 = dCosU1*dCosU2;
    db1 = dCosU1*dSinU2;
    db2 = dSinU1*dCosU2;

    dp = dCosU2*sin(dDifferenceLongit);
    dq = db1-db2*cos(dDifferenceLongit);

    // Start azimuth
    dA1 = atan2(dp,dq);

    if (dA1<0)
    {
        dA1 = dA1 + 2*cdPi;
    }


    dSinSIG = dp*sin(dA1)+dq*cos(dA1);
    dCosSIG = da1+da2*cos(dDifferenceLongit);

    // Spherical distance
    dSIG =atan2(dSinSIG,dCosSIG);

    if (dSIG>0)
    {
        dSIG = fabs(dSIG);
    }
    
    if (dSIG<0)
    {
        dSIG = cdPi - fabs(dSIG);
    }

    dA0 = asin(dCosU1*sin(dA1));

    // Comment Goga 25.11.2008 dX = 2*da1 - pow(cos(dA0),2)*dCosSIG;
    dX = 2*da1 - pow(cos(dA0),2)*cos(dSIG);                 // Goga 25.11.2008
    // Comment Goga 25.11.2008 dY = (pow(cos(dA0),4)-2*dX*dX)*dCosSIG;
    dY = (pow(cos(dA0),4)-2*dX*dX)*cos(dSIG);               // Goga 25.11.2008
    dA = 6356863.02 + (10708.949 - 13.474*pow(cos(dA0),2))*pow(cos(dA0),2);

    dB = 10708.938 - 17.956*pow(cos(dA0),2);

    dC = 4.487;


    // Count of the length of the geodesic line
    // Comment Goga 25.11.2008 m_dLength = dA*dSIG + (dB*dX + dC*dY)*dSinSIG;
    m_dLength = dA*dSIG + (dB*dX + dC*dY)*sin(dSIG);       // Goga 25.11.2008

    return true;


}


bool CGEOLINE::CalculationGEOLINE(const CGeocentric *pGEOCENTRIC1, const CGeocentric *pGEOCENTRIC2)
{
    CGeodesic GEO1, GEO2;

    GEOCENTRIC_GEO(pGEOCENTRIC1, &GEO1);
    GEOCENTRIC_GEO(pGEOCENTRIC2, &GEO2);

    if (! CalculationGEOLINE(&GEO1, &GEO2))
    {
        return false;
    }

    return true;
}


bool CGEOLINE::CalculationGEOLINE(const CGeodesic *pGEO1, const CGeodesic *pGEO2, double &GeoLine)
{    
    if (!CalculationGEOLINE(pGEO1, pGEO2))
    {
        return false;
    }

    GeoLine = m_dLength;
    return true;
}


bool CGEOLINE::CalculationGEOLINE(const CGeocentric *pGEOCENTRIC1, const CGeocentric *pGEOCENTRIC2, double &GeoLine)
{    
    if (!CalculationGEOLINE(pGEOCENTRIC1, pGEOCENTRIC2))
    {
        return false;
    }

    GeoLine = m_dLength;
    return true;
}
