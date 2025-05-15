// PACKAGE		: Conversion
// FILE         : Topo1_Topo2.cpp
//
// AUTHOR	: Marina Mukhortova,  2003
//
//DESCRIPTION :Implementation file for function of recalculation topocentric coordinates system

#include <cmath>
#include <cstdio>
#include <cerrno>

#include "ConstRecalc.h"
#include "Topo1_Topo2.h"
#include "Structures.h"


// PACKAGE		: Conversion
// FUNCTION        : InitTopo1_Topo2_SHIFT
//
// DESCRIPTION     : Function of initialization factors in recalculation from the TOPO1 to the TOPO2 
//                   All the parameters are in radians (angles) and metres (distanses).
//                   (using parameters of WGS-84)
//                  
// INPUTS          : pGEO1 - geodesic coordinates of the first TPSK origin;
//                   pGEO2 - geodesic coordinates of the second  TPSK origin;
//                   (WITH SHIFT)
//                   
// RETURNS         : pRecal - coefficients
//
bool   InitTopo1_Topo2_SHIFT (const CGeodesic *pGEO1,
                              const CGeodesic *pGEO2,
                              SKoefRecal  *pRecal)
{
    if((!pGEO1) ||
            (!pGEO2) ||
            (!pRecal) ||
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


    //CGeodesic sGEO1; // geodesic coordinates
    //CGeodesic sGEO2; // geodesic coordinates

    // Function of recalculation from GAUS to the geodesic system of coordinates (for the first
    // origin)      (WITH SHIFT)
    //GAUS_GEO_SHIFT(pGAUS1,
    //	           &sGEO1);

    // Function of recalculation from GAUS to the geodesic system of coordinates (for the second
    // origin)      (WITH SHIFT)
    //GAUS_GEO_SHIFT(pGAUS2,
    //	           &sGEO2);

    // Intermediate variables
    double dN1;                // The radius of the main vertical curve
    double dN2;                // The radius of the main vertical curve
    double dDifferenceLongit;  // The difference between longitudes


    // The radius of the main vertical curve
    dN1 = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO1->m_dLatitude),2.0));


    // The radius of the main vertical curve
    dN2 = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO2->m_dLatitude),2.0));


    // The difference between longitudes
    dDifferenceLongit = (pGEO2->m_dLongitude)-(pGEO1->m_dLongitude);


    // Coefficients
    pRecal->m_dKx1 = sin(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude)*cos(dDifferenceLongit)+
            cos(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude);
    pRecal->m_dKx2 = -sin(pGEO2->m_dLatitude)*cos(pGEO1->m_dLatitude)*cos(dDifferenceLongit)+
            sin(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude);
    pRecal->m_dKx3 = -sin(pGEO2->m_dLatitude)*sin(dDifferenceLongit);

    pRecal->m_dKy1 = -sin(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude)*cos(dDifferenceLongit)+
            cos(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude);
    pRecal->m_dKy2 = cos(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude)*cos(dDifferenceLongit)+
            sin(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude);
    pRecal->m_dKy3 = cos(pGEO2->m_dLatitude)*sin(dDifferenceLongit);

    pRecal->m_dKz1 = sin(pGEO1->m_dLatitude)*sin(dDifferenceLongit);
    pRecal->m_dKz2 = -cos(pGEO1->m_dLatitude)*sin(dDifferenceLongit);
    pRecal->m_dKz3 = cos(dDifferenceLongit);

    pRecal->m_dKx0 = (dN1+(pGEO1->m_dAltitude))*(sin(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude)-
                                                 cos(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude)*cos(dDifferenceLongit))+
            cdFirstEccentricity*cos(pGEO2->m_dLatitude)*(dN2*sin(pGEO2->m_dLatitude)-
                                                         dN1*sin(pGEO1->m_dLatitude));
    pRecal->m_dKy0 = (dN1+(pGEO1->m_dAltitude))*(pRecal->m_dKy2)-(dN2+(pGEO2->m_dAltitude))+
            cdFirstEccentricity*sin(pGEO2->m_dLatitude)*(dN2*sin(pGEO2->m_dLatitude)-
                                                         dN1*sin(pGEO1->m_dLatitude));
    pRecal->m_dKz0 = -(dN1+(pGEO1->m_dAltitude))*cos(pGEO1->m_dLatitude)*sin(dDifferenceLongit);

    return true;

}



// PACKAGE		: Conversion
// FUNCTION       : Topo1_Topo2_SHIFT
//
// DESCRIPTION    : Function of recalculation from the first topocentric system of coordinates
//                  to the second
//                  All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS         : Topocentric coordinates:
//                  pPositionTOPO1 - position,
//		    pVelocityTOPO1 - velocity,
//		    pAccelerationTOPO1 - acceleration,
//                  sRecal - coefficients
//
// RETURNS        : Topocentric coordinates:
//                  pPositionTOPO2 - position,
//		    pVelocityTOPO2 - velocity,
//		    pAccelerationTOPO2 - acceleration;
//                  
//
bool   Topo1_Topo2_SHIFT(const CTopocentric *pPositionTOPO1,
                         const CTopocentric *pVelocityTOPO1,
                         const CTopocentric *pAccelerationTOPO1,
                         SKoefRecal  sRecal,
                         CTopocentric *pPositionTOPO2,
                         CTopocentric *pVelocityTOPO2,
                         CTopocentric *pAccelerationTOPO2)
{
    
    if (pPositionTOPO1 || pVelocityTOPO1 || pAccelerationTOPO1)
    {

        if (pPositionTOPO1)
        {
            if ((pPositionTOPO1->m_dXt < cdTOPO_X_Y_Z_MIN) ||
                    (pPositionTOPO1->m_dXt > cdTOPO_X_Y_Z_MAX) ||
                    (pPositionTOPO1->m_dYt < cdTOPO_X_Y_Z_MIN) ||
                    (pPositionTOPO1->m_dYt > cdTOPO_X_Y_Z_MAX) ||
                    (pPositionTOPO1->m_dZt < cdTOPO_X_Y_Z_MIN) ||
                    (pPositionTOPO1->m_dZt > cdTOPO_X_Y_Z_MAX))
            {
                return false;
            }

            // Coordinates in the second topocentric system
            pPositionTOPO2->m_dXt = (sRecal.m_dKx1)*(pPositionTOPO1->m_dXt)+
                    (sRecal.m_dKx2)*(pPositionTOPO1->m_dYt)+
                    (sRecal.m_dKx3)*(pPositionTOPO1->m_dZt)+sRecal.m_dKx0;
            pPositionTOPO2->m_dYt = (sRecal.m_dKy1)*(pPositionTOPO1->m_dXt)+
                    (sRecal.m_dKy2)*(pPositionTOPO1->m_dYt)+
                    (sRecal.m_dKy3)*(pPositionTOPO1->m_dZt)+sRecal.m_dKy0;
            pPositionTOPO2->m_dZt = (sRecal.m_dKz1)*(pPositionTOPO1->m_dXt)+
                    (sRecal.m_dKz2)*(pPositionTOPO1->m_dYt)+
                    (sRecal.m_dKz3)*(pPositionTOPO1->m_dZt)+sRecal.m_dKz0;
        }
        else
        {
            pPositionTOPO2 = nullptr;
        }


        if (pVelocityTOPO1)
        {

            // Coordinates of the speed in the second topocentric system
            pVelocityTOPO2->m_dXt = (pVelocityTOPO1->m_dXt)*(sRecal.m_dKx1)+
                    (pVelocityTOPO1->m_dYt)*(sRecal.m_dKx2)+
                    (pVelocityTOPO1->m_dZt)*(sRecal.m_dKx3);
            pVelocityTOPO2->m_dYt = (pVelocityTOPO1->m_dXt)*(sRecal.m_dKy1)+
                    (pVelocityTOPO1->m_dYt)*(sRecal.m_dKy2)+
                    (pVelocityTOPO1->m_dZt)*(sRecal.m_dKy3);
            pVelocityTOPO2->m_dZt = (pVelocityTOPO1->m_dXt)*(sRecal.m_dKz1)+
                    (pVelocityTOPO1->m_dYt)*(sRecal.m_dKz2)+
                    (pVelocityTOPO1->m_dZt)*(sRecal.m_dKz3);
        }
        else
        {
            pVelocityTOPO2 = nullptr;
        }


        if (pAccelerationTOPO1)
        {
            // Coordinates of the acceleration in the second topocentric system
            pAccelerationTOPO2->m_dXt = (sRecal.m_dKx1)*(pAccelerationTOPO1->m_dXt)+
                    (sRecal.m_dKx2)*(pAccelerationTOPO1->m_dYt)+
                    (sRecal.m_dKx3)*(pAccelerationTOPO1->m_dZt);
            pAccelerationTOPO2->m_dYt = (sRecal.m_dKy1)*(pAccelerationTOPO1->m_dXt)+
                    (sRecal.m_dKy2)*(pAccelerationTOPO1->m_dYt)+
                    (sRecal.m_dKy3)*(pAccelerationTOPO1->m_dZt);
            pAccelerationTOPO2->m_dZt = (sRecal.m_dKz1)*(pAccelerationTOPO1->m_dXt)+
                    (sRecal.m_dKz2)*(pAccelerationTOPO1->m_dYt)+
                    (sRecal.m_dKz3)*(pAccelerationTOPO1->m_dZt);
        }
        else
        {
            pAccelerationTOPO2 = nullptr;
        }
    }
    else
    {
        pPositionTOPO2 = nullptr;
        pVelocityTOPO2 = nullptr;
        pAccelerationTOPO2 = nullptr;
    }

    return true;

}


// PACKAGE		: Conversion
// FUNCTION       : Topo_TopoAB
//
// DESCRIPTION    : Function of recount from the topo system to the topo
//                  in compliance with levelling coefficients
//                  All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS         : Topocentric coordinates:
//                  pPositionTOPO - position,
//		    pVelocityTOPO - velocity,
//		    pAccelerationTOPO - acceleration,
//                  pCoef_a, pCoef_b - levelling coefficients
//
// RETURNS        : Topocentric coordinates:
//                  pPositionTOPO_AB - position,
//		    pVelocityTOPO_AB - velocity,
//		    pAccelerationTOPO_AB - acceleration;
//
bool   Topo_TopoAB(const CTopocentric *pPositionTOPO,
                   const CTopocentric *pVelocityTOPO,
                   const CTopocentric *pAccelerationTOPO,
                   const SLevelCoef_A *pCoef_a,
                   const SLevelCoef_B *pCoef_b,
                   CTopocentric *pPositionTOPO_AB,
                   CTopocentric *pVelocityTOPO_AB,
                   CTopocentric *pAccelerationTOPO_AB)
{
    if (pPositionTOPO || pVelocityTOPO || pAccelerationTOPO)
    {

        if (pPositionTOPO)
        {
            if ((pPositionTOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
                    (pPositionTOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
                    (pPositionTOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
                    (pPositionTOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
                    (pPositionTOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
                    (pPositionTOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
            {
                return false;
            }
            // topo coordinates in compliance with levelling coefficients
            pPositionTOPO_AB->m_dXt = (pCoef_a->m_dA11)*(pPositionTOPO->m_dXt)+
                    (pCoef_a->m_dA12)*(pPositionTOPO->m_dYt)+
                    (pCoef_a->m_dA13)*(pPositionTOPO->m_dZt)+pCoef_b->m_dDeltaX;
            pPositionTOPO_AB->m_dYt = (pCoef_a->m_dA21)*(pPositionTOPO->m_dXt)+
                    (pCoef_a->m_dA22)*(pPositionTOPO->m_dYt)+
                    (pCoef_a->m_dA23)*(pPositionTOPO->m_dZt)+pCoef_b->m_dDeltaY;
            pPositionTOPO_AB->m_dZt = (pCoef_a->m_dA31)*(pPositionTOPO->m_dXt)+
                    (pCoef_a->m_dA32)*(pPositionTOPO->m_dYt)+
                    (pCoef_a->m_dA33)*(pPositionTOPO->m_dZt)+pCoef_b->m_dDeltaZ;
        }
        else
        {
            pPositionTOPO_AB = nullptr;
        }

        if (pVelocityTOPO)
        {
            // topo coorditates of velocity in compliance with levelling coefficients
            pVelocityTOPO_AB->m_dXt = (pVelocityTOPO->m_dXt)*(pCoef_a->m_dA11)+
                    (pVelocityTOPO->m_dYt)*(pCoef_a->m_dA12)+
                    (pVelocityTOPO->m_dZt)*(pCoef_a->m_dA13);
            pVelocityTOPO_AB->m_dYt = (pVelocityTOPO->m_dXt)*(pCoef_a->m_dA21)+
                    (pVelocityTOPO->m_dYt)*(pCoef_a->m_dA22)+
                    (pVelocityTOPO->m_dZt)*(pCoef_a->m_dA23);
            pVelocityTOPO_AB->m_dZt = (pVelocityTOPO->m_dXt)*(pCoef_a->m_dA31)+
                    (pVelocityTOPO->m_dYt)*(pCoef_a->m_dA32)+
                    (pVelocityTOPO->m_dZt)*(pCoef_a->m_dA33);
        }
        else
        {
            pVelocityTOPO_AB = nullptr;
        }


        if (pAccelerationTOPO)
        {
            // topo coordinates of acceleration in compliance with levelling
            // coefficients
            pAccelerationTOPO_AB->m_dXt = (pCoef_a->m_dA11)*(pAccelerationTOPO->m_dXt)+
                    (pCoef_a->m_dA12)*(pAccelerationTOPO->m_dYt)+
                    (pCoef_a->m_dA13)*(pAccelerationTOPO->m_dZt);
            pAccelerationTOPO_AB->m_dYt = (pCoef_a->m_dA21)*(pAccelerationTOPO->m_dXt)+
                    (pCoef_a->m_dA22)*(pAccelerationTOPO->m_dYt)+
                    (pCoef_a->m_dA23)*(pAccelerationTOPO->m_dZt);
            pAccelerationTOPO_AB->m_dZt = (pCoef_a->m_dA31)*(pAccelerationTOPO->m_dXt)+
                    (pCoef_a->m_dA32)*(pAccelerationTOPO->m_dYt)+
                    (pCoef_a->m_dA33)*(pAccelerationTOPO->m_dZt);
        }
        else
        {
            pAccelerationTOPO_AB = nullptr;
        }
    }
    else
    {
        pPositionTOPO_AB = nullptr;
        pVelocityTOPO_AB = nullptr;
        pAccelerationTOPO_AB = nullptr;
    }

    return true;

}


// PACKAGE		: Conversion
// FUNCTION        : InitTopo1_Topo2_SHIFT_KR
//
// DESCRIPTION     : Function of initialization factors in recalculation from the TOPO1 to the TOPO2 
//                   All the parameters are in radians (angles) and metres (distanses).
//                   (using Krasovsky parameters)   
//                  
// INPUTS          : pGEO1 - geodesic coordinates of the first TPSK origin;
//                   pGEO2 - geodesic coordinates of the second  TPSK origin;
//                   (WITH SHIFT)
//                   
// RETURNS         : pRecal - coefficients
//
bool   InitTopo1_Topo2_SHIFT_KR (const CGeodesic *pGEO1,
                                 const CGeodesic *pGEO2,
                                 SKoefRecal  *pRecal)
{
    if((!pGEO1) ||
            (!pGEO2) ||
            (!pRecal) ||
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


    //CGeodesic sGEO1; // geodesic coordinates
    //CGeodesic sGEO2; // geodesic coordinates

    // Function of recalculation from GAUS to the geodesic system of coordinates (for the first
    // origin)      (WITH SHIFT)
    //GAUS_GEO_SHIFT(pGAUS1,
    //	           &sGEO1);

    // Function of recalculation from GAUS to the geodesic system of coordinates (for the second
    // origin)      (WITH SHIFT)
    //GAUS_GEO_SHIFT(pGAUS2,
    //	           &sGEO2);

    // Intermediate variables
    double dN1;                // The radius of the main vertical curve
    double dN2;                // The radius of the main vertical curve
    double dDifferenceLongit;  // The difference between longitudes


    // The radius of the main vertical curve
    dN1 = cdEquatorAxisKr/sqrt(1.0-cdFirstEccentricityKr*pow(sin(pGEO1->m_dLatitude),2.0));


    // The radius of the main vertical curve
    dN2 = cdEquatorAxisKr/sqrt(1.0-cdFirstEccentricityKr*pow(sin(pGEO2->m_dLatitude),2.0));


    // The difference between longitudes
    dDifferenceLongit = (pGEO2->m_dLongitude)-(pGEO1->m_dLongitude);


    // Coefficients
    pRecal->m_dKx1 = sin(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude)*cos(dDifferenceLongit)+
            cos(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude);
    pRecal->m_dKx2 = -sin(pGEO2->m_dLatitude)*cos(pGEO1->m_dLatitude)*cos(dDifferenceLongit)+
            sin(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude);
    pRecal->m_dKx3 = -sin(pGEO2->m_dLatitude)*sin(dDifferenceLongit);

    pRecal->m_dKy1 = -sin(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude)*cos(dDifferenceLongit)+
            cos(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude);
    pRecal->m_dKy2 = cos(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude)*cos(dDifferenceLongit)+
            sin(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude);
    pRecal->m_dKy3 = cos(pGEO2->m_dLatitude)*sin(dDifferenceLongit);

    pRecal->m_dKz1 = sin(pGEO1->m_dLatitude)*sin(dDifferenceLongit);
    pRecal->m_dKz2 = -cos(pGEO1->m_dLatitude)*sin(dDifferenceLongit);
    pRecal->m_dKz3 = cos(dDifferenceLongit);

    pRecal->m_dKx0 = (dN1+(pGEO1->m_dAltitude))*(sin(pGEO1->m_dLatitude)*cos(pGEO2->m_dLatitude)-
                                                 cos(pGEO1->m_dLatitude)*sin(pGEO2->m_dLatitude)*cos(dDifferenceLongit))+
            cdFirstEccentricityKr*cos(pGEO2->m_dLatitude)*(dN2*sin(pGEO2->m_dLatitude)-
                                                           dN1*sin(pGEO1->m_dLatitude));
    pRecal->m_dKy0 = (dN1+(pGEO1->m_dAltitude))*(pRecal->m_dKy2)-(dN2+(pGEO2->m_dAltitude))+
            cdFirstEccentricityKr*sin(pGEO2->m_dLatitude)*(dN2*sin(pGEO2->m_dLatitude)-
                                                           dN1*sin(pGEO1->m_dLatitude));
    pRecal->m_dKz0 = -(dN1+(pGEO1->m_dAltitude))*cos(pGEO1->m_dLatitude)*sin(dDifferenceLongit);

    return true;

}

// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO_vel_accel
//
// DESCRIPTION     : Function of recalculation from the topocentric system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1, pCenter2 - Geodesic coordinates; pPositionTopo1 - topocentric coordinates;
//
//
// RETURNS	   : pPositionTopo2 - topocentric coordinates;
//

bool Topo1_Topo2(const CGeodesic *pCenter1,const CGeodesic *pCenter2,
                 const CTopocentric *pPositionTopo1,CTopocentric *pPositionTopo2)
{
    SKoefRecal pRecal;
    if(!InitTopo1_Topo2_SHIFT(pCenter1, pCenter2, &pRecal))
        return false;

    Square_Matrix<3> Matr;
    Matr.s_m=3;
    Matr.s_n=3;

    Matr.M[0][0]=pRecal.m_dKx1;
    Matr.M[0][1]=pRecal.m_dKx2;
    Matr.M[0][2]=pRecal.m_dKx3;

    Matr.M[1][0]=pRecal.m_dKy1;
    Matr.M[1][1]=pRecal.m_dKy2;
    Matr.M[1][2]=pRecal.m_dKy3;

    Matr.M[2][0]=pRecal.m_dKz1;
    Matr.M[2][1]=pRecal.m_dKz2;
    Matr.M[2][2]=pRecal.m_dKz3;


    Square_Matrix<3> Matr_pos=Square_Matrix<3>(3,1);
    Matr_pos.M[0][0]=pPositionTopo1->m_dXt;
    Matr_pos.M[1][0]=pPositionTopo1->m_dYt;
    Matr_pos.M[2][0]=pPositionTopo1->m_dZt;

    Square_Matrix<3> Matr_Res=Square_Matrix<3>(3,1);
    Matr.MatrXMatr(&Matr_pos,&Matr_Res);
    pPositionTopo2->m_dXt=Matr_Res.M[0][0]+pRecal.m_dKx0;
    pPositionTopo2->m_dYt=Matr_Res.M[1][0]+pRecal.m_dKy0;
    pPositionTopo2->m_dZt=Matr_Res.M[2][0]+pRecal.m_dKz0;

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO_vel_accel
//
// DESCRIPTION     : Function of recalculation from the topocentric system of coordinates to the
//                   topocentric system of coordinates, velocity;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1, pCenter2 - Geodesic coordinates; pPositionTopo1 - topocentric coordinates;
//                   pVelocityTopo1 - topocentric velocity;
//
// RETURNS	   : pPositionTopo2 - topocentric coordinates, pVelocityTopo2 - topocentric velocity;
//

bool Topo1_Topo2(const CGeodesic *pCenter1, const CGeodesic *pCenter2,
                 const CTopocentric *pPositionTopo1, CTopocentric *pPositionTopo2,
                 const CTopocentric *pVelocityTopo1, CTopocentric *pVelocityTopo2)
{
    SKoefRecal pRecal;
    if(!InitTopo1_Topo2_SHIFT(pCenter1, pCenter2, &pRecal))
        return false;

    Square_Matrix<3> Matr;
    Matr.s_m=3;
    Matr.s_n=3;

    Matr.M[0][0]=pRecal.m_dKx1;
    Matr.M[0][1]=pRecal.m_dKx2;
    Matr.M[0][2]=pRecal.m_dKx3;

    Matr.M[1][0]=pRecal.m_dKy1;
    Matr.M[1][1]=pRecal.m_dKy2;
    Matr.M[1][2]=pRecal.m_dKy3;

    Matr.M[2][0]=pRecal.m_dKz1;
    Matr.M[2][1]=pRecal.m_dKz2;
    Matr.M[2][2]=pRecal.m_dKz3;

    Topo1_Topo2(pCenter1,pCenter2, pPositionTopo1,pPositionTopo2);

    Square_Matrix<3> Matr_Vel=Square_Matrix<3>(3,1);
    Matr_Vel.M[0][0]=pVelocityTopo1->m_dXt;
    Matr_Vel.M[1][0]=pVelocityTopo1->m_dYt;
    Matr_Vel.M[2][0]=pVelocityTopo1->m_dZt;

    Square_Matrix<3> Matr_Res_Vel=Square_Matrix<3>(3,1);
    Matr.MatrXMatr(&Matr_Vel,&Matr_Res_Vel);
    pVelocityTopo2->m_dXt=Matr_Res_Vel.M[0][0];
    pVelocityTopo2->m_dYt=Matr_Res_Vel.M[1][0];
    pVelocityTopo2->m_dZt=Matr_Res_Vel.M[2][0];


    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO_vel_accel
//
// DESCRIPTION     : Function of recalculation from the topocentric system of coordinates to the
//                   topocentric system of coordinates, velocity and acceleration;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1, pCenter2 - Geodesic coordinates; pPositionTopo1 - topocentric coordinates;
//                   pVelocityTopo1 - topocentric velocity and pAccelerationTopo1 - topocentric acceleration;
//
// RETURNS	   : pPositionTopo2 - topocentric coordinates, pVelocityTopo2 - topocentric velocity  and pAccelerationTopo2 - topocentric acceleration;
//

bool Topo1_Topo2(const CGeodesic *pCenter1, const CGeodesic *pCenter2,
                 const CTopocentric *pPositionTopo1, CTopocentric *pPositionTopo2,
                 const CTopocentric *pVelocityTopo1, CTopocentric *pVelocityTopo2,
                 const CTopocentric *pAccelerationTopo1, CTopocentric *pAccelerationTopo2)
{
    SKoefRecal pRecal;
    if(!InitTopo1_Topo2_SHIFT(pCenter1, pCenter2, &pRecal))
    {
        return false;
    }

    Square_Matrix<3> Matr;
    Matr.s_m=3;
    Matr.s_n=3;

    Matr.M[0][0]=pRecal.m_dKx1;
    Matr.M[0][1]=pRecal.m_dKx2;
    Matr.M[0][2]=pRecal.m_dKx3;

    Matr.M[1][0]=pRecal.m_dKy1;
    Matr.M[1][1]=pRecal.m_dKy2;
    Matr.M[1][2]=pRecal.m_dKy3;

    Matr.M[2][0]=pRecal.m_dKz1;
    Matr.M[2][1]=pRecal.m_dKz2;
    Matr.M[2][2]=pRecal.m_dKz3;

    Topo1_Topo2(pCenter1,pCenter2, pPositionTopo1,pPositionTopo2,
                pVelocityTopo1,pVelocityTopo2);

    Square_Matrix<3> Matr_Acc=Square_Matrix<3>(3,1);
    Matr_Acc.M[0][0]=pAccelerationTopo1->m_dXt;
    Matr_Acc.M[1][0]=pAccelerationTopo1->m_dYt;
    Matr_Acc.M[2][0]=pAccelerationTopo1->m_dZt;

    Square_Matrix<3> Matr_Res_Acc=Square_Matrix<3>(3,1);
    Matr.MatrXMatr(&Matr_Acc,&Matr_Res_Acc);
    pAccelerationTopo2->m_dXt=Matr_Res_Acc.M[0][0];
    pAccelerationTopo2->m_dYt=Matr_Res_Acc.M[1][0];
    pAccelerationTopo2->m_dZt=Matr_Res_Acc.M[2][0];

    return true;
}

// PACKAGE		: Conversion
// FUNCTION       : Recount_CovMatr_TopoToTopo
// DESCRIPTION    : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   topocentric system of coordinates
// INPUTS         : pGEO_1 - Geodesic coordinates of the center topo_1;
//                  pGEO_2 - Geodesic coordinates of the center topo_1
//                  CM_NUE_1 - the covariance matrix of errors in the determination of topocentric
//                  coordinates;
// RETURNS        : CM_NUE_1 - the covariance matrix of errors in the determination of topocentric
//                  coordinates;


bool Recount_CovMatr_TopoToTopo(const CGeodesic *pGEO_1,const CGeodesic *pGEO_2,
                                Square_Matrix<3> *CM_NUE_1, Square_Matrix<3> *CM_NUE_2){
    int dim = CM_NUE_1->s_m;
    if (CM_NUE_1->s_n != dim)
    {
        return false;
    }

    const int c_dim = 3;
    if (dim != c_dim)
    {
        return false;
    }


    SKoefRecal pRecal;
    if(!InitTopo1_Topo2_SHIFT(pGEO_1, pGEO_2, &pRecal))
        return false;

    CM_NUE_2->Reset(3,3);

    Square_Matrix<c_dim> Matr;
    Matr.s_m=c_dim;
    Matr.s_n=c_dim;

    Matr.M[0][0]=pRecal.m_dKx1;
    Matr.M[0][1]=pRecal.m_dKx2;
    Matr.M[0][2]=pRecal.m_dKx3;

    Matr.M[1][0]=pRecal.m_dKy1;
    Matr.M[1][1]=pRecal.m_dKy2;
    Matr.M[1][2]=pRecal.m_dKy3;

    Matr.M[2][0]=pRecal.m_dKz1;
    Matr.M[2][1]=pRecal.m_dKz2;
    Matr.M[2][2]=pRecal.m_dKz3;

    Square_Matrix<c_dim> Matr_tr;
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_NUE_1,&Matr_tr,CM_NUE_2);
    return true;
}

bool Recount_CovMatr_TopoToTopo(const CGeodesic *pGEO_1,const CGeodesic *pGEO_2, Square_Matrix<6> *CM_NUE_1, Square_Matrix<6> *CM_NUE_2){
    int dim = CM_NUE_1->s_m;
    if (CM_NUE_1->s_n != dim)
    {
        return false;
    }

    const int c_dim = 6;
    if (dim != c_dim)
    {
        return false;
    }

    SKoefRecal pRecal;
    if(!InitTopo1_Topo2_SHIFT(pGEO_1, pGEO_2, &pRecal))
    {
        return false;
    }

    CM_NUE_2->Reset(6,6);

    Square_Matrix<c_dim> Matr;
    Matr.s_m=c_dim;
    Matr.s_n=c_dim;

    Matr.M[0][0]=pRecal.m_dKx1;
    Matr.M[0][1]=pRecal.m_dKx2;
    Matr.M[0][2]=pRecal.m_dKx3;

    Matr.M[1][0]=pRecal.m_dKy1;
    Matr.M[1][1]=pRecal.m_dKy2;
    Matr.M[1][2]=pRecal.m_dKy3;

    Matr.M[2][0]=pRecal.m_dKz1;
    Matr.M[2][1]=pRecal.m_dKz2;
    Matr.M[2][2]=pRecal.m_dKz3;

    Matr.M[3][3]=pRecal.m_dKx1;
    Matr.M[3][4]=pRecal.m_dKx2;
    Matr.M[3][5]=pRecal.m_dKx3;

    Matr.M[4][3]=pRecal.m_dKy1;
    Matr.M[4][4]=pRecal.m_dKy2;
    Matr.M[4][5]=pRecal.m_dKy3;

    Matr.M[5][3]=pRecal.m_dKz1;
    Matr.M[5][4]=pRecal.m_dKz2;
    Matr.M[5][5]=pRecal.m_dKz3;

    Square_Matrix<c_dim> Matr_tr;
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_NUE_1,&Matr_tr,CM_NUE_2);

    return true;
}

bool Recount_CovMatr_TopoToTopo(const CGeodesic *pGEO_1,const CGeodesic *pGEO_2,
                                Square_Matrix<9> *CM_NUE_1, Square_Matrix<9> *CM_NUE_2){
    int dim = CM_NUE_1->s_m;
    if (CM_NUE_1->s_n != dim)
    {
        return false;
    }

    const int c_dim = 9;
    if (dim != c_dim)
    {
        return false;
    }

    SKoefRecal pRecal;
    if(!InitTopo1_Topo2_SHIFT(pGEO_1, pGEO_2, &pRecal))
    {
        return false;
    }

    CM_NUE_2->Reset(9,9);

    Square_Matrix<c_dim> Matr;
    Matr.s_m=c_dim;
    Matr.s_n=c_dim;

    Matr.M[0][0]=pRecal.m_dKx1;
    Matr.M[0][1]=pRecal.m_dKx2;
    Matr.M[0][2]=pRecal.m_dKx3;

    Matr.M[1][0]=pRecal.m_dKy1;
    Matr.M[1][1]=pRecal.m_dKy2;
    Matr.M[1][2]=pRecal.m_dKy3;

    Matr.M[2][0]=pRecal.m_dKz1;
    Matr.M[2][1]=pRecal.m_dKz2;
    Matr.M[2][2]=pRecal.m_dKz3;

    Matr.M[3][3]=pRecal.m_dKx1;
    Matr.M[3][4]=pRecal.m_dKx2;
    Matr.M[3][5]=pRecal.m_dKx3;

    Matr.M[4][3]=pRecal.m_dKy1;
    Matr.M[4][4]=pRecal.m_dKy2;
    Matr.M[4][5]=pRecal.m_dKy3;

    Matr.M[5][3]=pRecal.m_dKz1;
    Matr.M[5][4]=pRecal.m_dKz2;
    Matr.M[5][5]=pRecal.m_dKz3;

    Matr.M[6][6]=pRecal.m_dKx1;
    Matr.M[6][7]=pRecal.m_dKx2;
    Matr.M[6][8]=pRecal.m_dKx3;

    Matr.M[7][6]=pRecal.m_dKy1;
    Matr.M[7][7]=pRecal.m_dKy2;
    Matr.M[7][8]=pRecal.m_dKy3;

    Matr.M[8][6]=pRecal.m_dKz1;
    Matr.M[8][7]=pRecal.m_dKz2;
    Matr.M[8][8]=pRecal.m_dKz3;

    Square_Matrix<c_dim> Matr_tr;
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_NUE_1,&Matr_tr,CM_NUE_2);

    return true;
}
