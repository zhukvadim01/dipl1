// PACKAGE		: Conversion
// FILE          : GEOCENTRIC_GEO.cpp
//
// AUTHOR	 : Marina Mukhortova,  2004
//
//DESCRIPTION :Implementation file for function of recalculation geocentric coordinates system and geodesic
//                   coordinates system

#include "ConstRecalc.h"
#include "Geocentric_Geo.h"
#include "Geocentric.h"
#include "Geodesic.h"
#include "Geocentric_Topo.h"

#include <cmath>
#include <cstdio>
#include <cerrno>
#include <cassert>



// PACKAGE		: Conversion
// FUNCTION	    : GEOCENTRIC_GEO
//
// DESCRIPTION	    : Function of recalculation from the geocentric system of coordinates
//                    to the geodesic system of coordinates (parameters of WGS-84)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pWGS - geocentric coordinates;
//
// RETURNS	    : pGEO - geodesic coordinates;
//
bool   GEOCENTRIC_GEO(const CGeocentric *pWGS,
                      CGeodesic *pGEO)
{
    if((!pWGS) ||
            (!pGEO) ||
            (pWGS->m_dX < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
            (pWGS->m_dY < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
            (pWGS->m_dZ < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dZ > cdGEOCEN_X_Y_Z_MAX))
    {
        return false;
    }

    if(pWGS->m_dX == 0 && pWGS->m_dY == 0)
    {
        pGEO->m_dLatitude = 0;
        pGEO->m_dLongitude = 0;
        pGEO->m_dAltitude = fabs(pWGS->m_dZ)-cdEquatorAxis;
        return true;
    }
    if(sqrt(pow((pWGS->m_dX),2.0)+pow((pWGS->m_dY),2.0)+pow((pWGS->m_dZ),2.0))<cdPolarAxis*0.9)
    {
        return GEOCENTRIC_GEO_it(pWGS,pGEO);
    }
    //Intermediate variables
    double dP;
    //Intermediate variables
    dP = sqrt(pow((pWGS->m_dX),2.0)+pow((pWGS->m_dY),2.0));

    if (dP == 0)
    {
        if (pWGS->m_dZ == 0)
        {
            assert(0);
        }
        pGEO->m_dLatitude = ((pWGS->m_dZ>0) ? cdPi/2 : -cdPi/2);
        pGEO->m_dLongitude = 0;
        pGEO->m_dAltitude = ((pWGS->m_dZ>0) ? pWGS->m_dZ : -pWGS->m_dZ)-cdMainRadius;
    }
    else
    {
        double dQ = atan(((pWGS->m_dZ)*cdEquatorAxis)/(dP*cdPolarAxis));


        // Geodesic coordinates - Latitude
        pGEO->m_dLatitude = atan(((pWGS->m_dZ)+cdSecondEccentricity*cdPolarAxis*pow(sin(dQ),3.0))/(dP-
                                                                                                   cdFirstEccentricity*cdEquatorAxis*pow(cos(dQ),3.0)));


        //Intermediate variables
        double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO->m_dLatitude),2.0));


        // Geodesic coordinates - Longitude
        pGEO->m_dLongitude = atan2((pWGS->m_dY),(pWGS->m_dX));


        // Geodesic coordinates - Altitude
        pGEO->m_dAltitude = dP/cos(pGEO->m_dLatitude)-dN;
    }

    return true;

}

// PACKAGE		: Conversion
// FUNCTION	    : GEOCENTRIC_GEO_it
//
// DESCRIPTION	    : Function of recalculation from the geocentric system of coordinates
//                    to the geodesic system of coordinates (parameters of WGS-84)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pWGS - geocentric coordinates;
//
// RETURNS	    : pGEO - geodesic coordinates;
//
bool   GEOCENTRIC_GEO_it(const CGeocentric *pWGS,
                         CGeodesic *pGEO)
{
    if((!pWGS) ||
            (!pGEO) ||
            (pWGS->m_dX < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
            (pWGS->m_dY < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
            (pWGS->m_dZ < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dZ > cdGEOCEN_X_Y_Z_MAX))
    {
        return false;
    }
    if(pWGS->m_dX == 0 && pWGS->m_dY == 0)
    {
        pGEO->m_dLatitude = 0;
        pGEO->m_dLongitude = 0;
        pGEO->m_dAltitude = fabs(pWGS->m_dZ) - cdEquatorAxis;
        return true;
    }
    double lat_first = atan((pWGS->m_dZ/sqrt(pWGS->m_dX*pWGS->m_dX+pWGS->m_dY*pWGS->m_dY))*pow((1-cdFirstEccentricity),(-1)));
    double v = cdEquatorAxis/sqrt( 1.0-cdFirstEccentricity * pow( sin(lat_first) , 2.0) );
    double h = sqrt(pWGS->m_dX*pWGS->m_dX+pWGS->m_dY*pWGS->m_dY)/cos(lat_first)-v;

    double d_lat = 10;
    int counter=0;

    while (fabs(d_lat)>pow(10,-20))
    {
        counter++;
        if(counter > NMaxCycle)
        {
            return false;
        }
        double lat_prev = lat_first;
        lat_first = atan((pWGS->m_dZ / sqrt(pWGS->m_dX*pWGS->m_dX+pWGS->m_dY*pWGS->m_dY))*pow((1-cdFirstEccentricity*v/(v+h)),(-1)));

        v = cdEquatorAxis/sqrt( 1.0-cdFirstEccentricity * pow( sin(lat_first) , 2.0) );
        h = sqrt(pWGS->m_dX*pWGS->m_dX+pWGS->m_dY*pWGS->m_dY)/cos(lat_first)-v;

        d_lat = lat_prev-lat_first;
    }
    pGEO->m_dLatitude = lat_first;
    pGEO->m_dLongitude = atan2((pWGS->m_dY),(pWGS->m_dX));
    pGEO->m_dAltitude = h;
    return true;
}

// PACKAGE		: Conversion
// FUNCTION	    : GEO_GEOCENTRIC
//
// DESCRIPTION	    : Function of recalculation from the geodesic  system of coordinates
//                    to  the geocentric system of coordinates (parameters of WGS-84)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates;
//
// RETURNS	    : pWGS - geocentric coordinates;
//
bool   GEO_GEOCENTRIC (const CGeodesic *pGEO,
                       CGeocentric *pWGS)
{
    if((!pGEO) ||
            (!pWGS) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))

    {
        return false;
    }

    //Intermediate variables
    double dN;     // The radius of the main vertical curve


    // The radius of the main vertical curve
    dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO->m_dLatitude),2.0));


    // Geocentric coordinates
    pWGS->m_dX = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLatitude)*cos(pGEO->m_dLongitude);

    pWGS->m_dY = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLatitude)*sin(pGEO->m_dLongitude);

    pWGS->m_dZ = (dN*(1.0-cdFirstEccentricity)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);

    return true;

}



// PACKAGE		: Conversion
// FUNCTION	    : Kr_GEOCENTRIC_GEO
//
// DESCRIPTION	    : Function of recalculation from the geocentric system of coordinates
//                    to the geodesic system of coordinates (Krasovsky parameters)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pWGS - geocentric coordinates;
//
// RETURNS	    : pGEO - geodesic coordinates;
//
bool   Kr_GEOCENTRIC_GEO(const CGeocentric *pWGS,
                         CGeodesic *pGEO)
{
    if((!pWGS) ||
            (!pGEO) ||
            (pWGS->m_dX < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
            (pWGS->m_dY < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
            (pWGS->m_dZ < cdGEOCEN_X_Y_Z_MIN) ||
            (pWGS->m_dZ > cdGEOCEN_X_Y_Z_MAX))
    {
        return false;
    }

    //Intermediate variables
    double dP;    
    //Intermediate variables
    dP = sqrt(pow((pWGS->m_dX),2.0)+pow((pWGS->m_dY),2.0));

    if (dP == 0)
    {
        if (pWGS->m_dZ == 0) assert(0);
        pGEO->m_dLatitude = ((pWGS->m_dZ>0) ? cdPi/2 : -cdPi/2);
        pGEO->m_dLongitude = 0;
        pGEO->m_dAltitude = ((pWGS->m_dZ>0) ? pWGS->m_dZ : -pWGS->m_dZ)-cdMainRadius;
    }
    else
    {
        double dQ = atan(((pWGS->m_dZ)*cdEquatorAxisKr)/(dP*cdPolarAxisKr));

        // Geodesic coordinates - Latitude
        pGEO->m_dLatitude = atan(((pWGS->m_dZ)+
                                  cdSecondEccentricityKr*cdPolarAxisKr*pow(sin(dQ),3.0))/(dP-
                                                                                          cdFirstEccentricityKr*cdEquatorAxisKr*pow(cos(dQ),3.0)));


        //Intermediate variables
        double dN = cdEquatorAxisKr/sqrt(1.0-cdFirstEccentricityKr*pow(sin(pGEO->m_dLatitude),2.0));


        // Geodesic coordinates - Longitude
        pGEO->m_dLongitude = atan2((pWGS->m_dY),(pWGS->m_dX));


        // Geodesic coordinates - Altitude
        pGEO->m_dAltitude = dP/cos(pGEO->m_dLatitude)-dN;
    }

    return true;


}


// PACKAGE		: Conversion
// FUNCTION	    : Kr_GEO_GEOCENTRIC
//
// DESCRIPTION	    : Function of recalculation from the geodesic  system of coordinates
//                    to  the geocentric system of coordinates (Krasovsky parameters)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates;
//
// RETURNS	    : pWGS - geocentric coordinates;
//
bool   Kr_GEO_GEOCENTRIC (const CGeodesic *pGEO,
                          CGeocentric *pWGS)
{
    if((!pGEO) ||
            (!pWGS) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))
    {
        return false;
    }

    //Intermediate variables
    double dN;     // The radius of the main vertical curve


    // The radius of the main vertical curve
    dN = cdEquatorAxisKr/sqrt(1.0-cdFirstEccentricityKr*pow(sin(pGEO->m_dLatitude),2.0));


    // Geocentric coordinates
    pWGS->m_dX = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLatitude)*cos(pGEO->m_dLongitude);

    pWGS->m_dY = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLatitude)*sin(pGEO->m_dLongitude);

    pWGS->m_dZ = (dN*(1.0-cdFirstEccentricityKr)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);

    return true;

}

bool  GEO_TOPO(const CGeodesic *pGEO_in, const CGeodesic *pGEO_base, CTopocentric& pointTOPO)
{
    CGeocentric l_In_geocenric;
    bool res = GEO_GEOCENTRIC( pGEO_in, &l_In_geocenric) ;

    res |= GEOCENTRIC_TOPO(pGEO_base, &l_In_geocenric, &pointTOPO);
    return res;
}


bool Recount_CovMatr_GeocentricToGEO(const CGeocentric *pWGS, Square_Matrix<3> *pCovWGS, Square_Matrix<3> *pCovGEO)
{
    bool bOK = true;
    if (pWGS && pCovWGS && pCovGEO)
    {
        if ((pWGS->m_dX < cdGEOCEN_X_Y_Z_MIN) ||
                (pWGS->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
                (pWGS->m_dY < cdGEOCEN_X_Y_Z_MIN) ||
                (pWGS->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
                (pWGS->m_dZ < cdGEOCEN_X_Y_Z_MIN) ||
                (pWGS->m_dZ > cdGEOCEN_X_Y_Z_MAX))
        {
            bOK = false;
        }
        else
        {
            double a = cdEquatorAxis;
            double b = cdPolarAxis;
            double X2 = pWGS->m_dX * pWGS->m_dX;
            double Y2 = pWGS->m_dY * pWGS->m_dY;
            double p2 = X2 + Y2;
            double p = sqrt(p2);
            double a2 = a*a;
            double a3 = a2*a;
            double b2 = b*b;
            double b3 = b2*b;
            double b5 = b3*b2;
            double p3 = p2*p;
            double Z2 = pWGS->m_dZ*pWGS->m_dZ;
            double Z3 = Z2*pWGS->m_dZ;
            double m0 = sqrt(b2*p2 + a2*Z2);
            double m02 = m0*m0;
            double m03 = m02*m0;
            double m05 = m02*m03;
            if (m0 > 0)
            {
                double u0 = pWGS->m_dZ + cdSecondEccentricity*b*a3*Z3/m03;
                double v0 = p - cdFirstEccentricity*a*b3*p3/m03;
                if (v0 > 0)
                {
                    double g0 = u0/v0;
                    double g02 = g0*g0;
                    double q0 = (1.+g02)*v0;

                    double r2 = p2 + Z2;
                    double r = sqrt(r2);
                    double r3 = r2*r;
                    if (fabs(r) > 0 &&  fabs(q0) > 0 && fabs(p) > 0)
                    {
                        double Coef_dLatXY = - (3.*cdSecondEccentricity*a3*b3*Z3/m05
                                                + g0*(1./p - 3*cdFirstEccentricity*a*b3*p/m03
                                                      + 3.*cdFirstEccentricity*a*b5*p3/m05)) / q0;

                        double S = (b/a)*cdFirstEccentricity / pow((b2/a2 + cdFirstEccentricity*Z2/r2), 1.5);

                        Square_Matrix<3> Jmatr(3,3);
                        Jmatr.M[0][0] = pWGS->m_dX * Coef_dLatXY;
                        Jmatr.M[0][1] = pWGS->m_dY * Coef_dLatXY;
                        Jmatr.M[0][2] = ((1.+3.*cdSecondEccentricity*b*Z2*a3*(1.-Z2*a2/m02)/m03)
                                         - 3.*u0*cdFirstEccentricity*a3*b3*p3*pWGS->m_dZ/(v0*m05)) / q0;
                        Jmatr.M[1][0] = -pWGS->m_dY / p2;
                        Jmatr.M[1][1] =  pWGS->m_dX / p2;
                        Jmatr.M[1][2] = 0.;

                        Jmatr.M[2][0] = pWGS->m_dX*(1. - a*Z2*S/r3)/r;
                        Jmatr.M[2][1] = pWGS->m_dY*(1. - a*Z2*S/r3)/r;
                        Jmatr.M[2][2] = pWGS->m_dZ*(1. + a*p2*S/r3)/r;

                        Square_Matrix<3> JmatrT(3,3); //J transposed
                        Jmatr.transp(&JmatrT);

                        pCovGEO->Reset(3, 3);
                        bOK = Jmatr.MatrXMatrXMatr(pCovWGS, &JmatrT, pCovGEO);
                    }
                    else
                    {
                        bOK = false;
                    }
                }
                else
                {
                    bOK = false;
                }
            }
            else
            {
                bOK = false;
            }
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}

bool Recount_CovMatr_GeocentricToGeodesic(const CGeocentric *pGEO, Square_Matrix<3> *pCovGEO, Square_Matrix<3> *pCovGDS)
{

    if ( !pGEO || !pCovGDS || !pCovGEO)
        return false;

    if ((pGEO->m_dX < cdGEOCEN_X_Y_Z_MIN) || (pGEO->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
            (pGEO->m_dY < cdGEOCEN_X_Y_Z_MIN) || (pGEO->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
            (pGEO->m_dZ < cdGEOCEN_X_Y_Z_MIN) || (pGEO->m_dZ > cdGEOCEN_X_Y_Z_MAX))
        return false;

    double X(pGEO->m_dX), Y(pGEO->m_dY), Z(pGEO->m_dZ);
    double p2(X*X + Y*Y), p(sqrt(p2));

    if ( fabs(p) < 0.1E-14 )
        return false;

    double a(cdEquatorAxis), b(cdPolarAxis), a_b(a/b), k(a_b*(Z/p)), k_1(1./(1+k*k));
    double ef2(cdFirstEccentricity), es2(cdSecondEccentricity);

    double Q(atan(k)), sinQ(sin(Q)), cosQ(cos(Q));
    double sinQ2(sinQ*sinQ), cosQ2(cosQ*cosQ);
    double sinQ3(sinQ*sinQ2), cosQ3(cosQ*cosQ2);

    double D(Z+b*es2*sinQ3), S(p-a*ef2*cosQ3);

    double dpdx(X/p), dpdy(Y/p);
    double dQdx(-k_1*a_b*(Z/p)*(X/p2)), dQdy(-k_1*a_b*(Z/p)*(Y/p2)), dQdz(k_1*a_b/p);

    double mD(3.*b*es2*sinQ2*cosQ), dDdx(mD*dQdx), dDdy(mD*dQdy), dDdz(1+mD*dQdz);
    double mS(3.*a*ef2*cosQ2*sinQ), dSdx(dpdx+mS*dQdx), dSdy(dpdy+mS*dQdy), dSdz(mS*dQdz);

    if ( fabs(S) < 0.1E-14 )
        return false;

    double D_S(D/S), D_S_1(1/(1+D_S*D_S)), B(atan(D_S));
    double sinB(sin(B)), cosB(cos(B)), cosB2(cosB*cosB);

    if ( fabs(cosB) < 0.1E-14 )
        return false;

    double dBdx(D_S_1 * (dDdx /S - D * dSdx /S /S));
    double dBdy(D_S_1 * (dDdy /S - D * dSdy /S /S));
    double dBdz(D_S_1 * (dDdz /S - D * dSdz /S /S));

    double N(a/sqrt(1-ef2*sinB*sinB)), dNdB(a*ef2*pow(N/a,3.)*sinB*cosB);

    Square_Matrix<3> Jmatr(3,3);
    Jmatr.M[0][0] = dBdx;
    Jmatr.M[0][1] = dBdy;
    Jmatr.M[0][2] = dBdz;

    Jmatr.M[1][0] = -Y / p2;
    Jmatr.M[1][1] =  X / p2;
    Jmatr.M[1][2] =  0.;

    Jmatr.M[2][0] = dpdx /cosB + p * (sinB /cosB2) * dBdx - dNdB * dBdx;
    Jmatr.M[2][1] = dpdy /cosB + p * (sinB /cosB2) * dBdy - dNdB * dBdy;
    Jmatr.M[2][2] = p * (sinB /cosB2) * dBdz - dNdB * dBdz;

    Square_Matrix<3> JmatrT(3,3); //J transposed
    Jmatr.transp(&JmatrT);

    pCovGDS->Reset(3, 3);
    bool bOK = Jmatr.MatrXMatrXMatr(pCovGEO, &JmatrT, pCovGDS);

    return bOK;
}

bool Recount_CovMatr_GeodesicToGeocentric(const CGeodesic *pGEO, Square_Matrix<3> *pCovGEO, Square_Matrix<3> *pCovWGS)
{
    if( !pGEO || !pCovGEO || !pCovWGS )
    {
        return false;
    }
    if( (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) || (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) || (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) || (pGEO->m_dAltitude > cd_H_MAX) )
    {
        return false;
    }

    //temp calculation
    double sinB(sin(pGEO->m_dLatitude)), cosB(cos(pGEO->m_dLatitude));
    double sinL(sin(pGEO->m_dLongitude)), cosL(cos(pGEO->m_dLongitude));
    double dH(pGEO->m_dAltitude), dEccent(1. - cdFirstEccentricity);


    // The radius of the main vertical curve
    double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sinB, 2.));
    double dNdB = cdEquatorAxis * cdFirstEccentricity * pow(dN/cdEquatorAxis, 3.) * sinB * cosB;

    Square_Matrix<3> Jmatr(3,3);
    Jmatr.M[0][0] = dNdB * cosB * cosL - (dN + dH) * sinB * cosL;
    Jmatr.M[0][1] = -(dN + dH) * cosB * sinL;
    Jmatr.M[0][2] = cosB * cosL;

    Jmatr.M[1][0] = dNdB * cosB * sinL - (dN + dH) * sinB * sinL;
    Jmatr.M[1][1] = (dN + dH) * cosB * cosL;
    Jmatr.M[1][2] = cosB * sinL;

    Jmatr.M[2][0] = dEccent * dNdB * sinB + (dN * dEccent + dH) * cosB;
    Jmatr.M[2][1] = 0;
    Jmatr.M[2][2] = sinB;

    Square_Matrix<3> JmatrT(3,3); //J transposed
    Jmatr.transp(&JmatrT);

    pCovWGS->Reset(3, 3);
    bool bOK = Jmatr.MatrXMatrXMatr(pCovGEO, &JmatrT, pCovWGS);

    return bOK;
}
