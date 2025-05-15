// PACKAGE		: Conversion
// FILE      : WGS_TOPO.cpp
// AUTHOR    : Marina Mukhortova, 2003
//
//DESCRIPTION :Implementation file for function of recalculation Geocentric coordinates system and geodesic
//                   topocentric system

#include "ConstRecalc.h"
#include "Geocentric_Topo.h"
#include "Structures.h"
#include "Topocentric.h"
#include "Geodesic.h"
#include "Geocentric.h"

#include <cmath>
#include <cstdio>
#include <cerrno>


// PACKAGE		: Conversion
// FUNCTION	    : TOPO_GEOCENTRIC_SHIFT
//
// DESCRIPTION      : Function of recalculation from the topocentric system of coordinates
//                    to the geocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : Topocentric coordinates:
//                    pPositionTOPO - position,
//		      pVelocityTOPO - velocity,
//		      pAccelerationTOPO - acceleration,
//                    sKoef - coefficients of the matrix of rotational cosines,
//                    sKoefTurn - the matrix of transfer coefficients from TOPO to WGS;
//
// RETURNS	    : Geocentric coordinates:
//                    pPositionWGS - position,
//		      pVelocityWGS - velocity,
//		      pAccelerationWGS - acceleration;   
//
bool   TOPO_GEOCENTRIC_SHIFT(const CTopocentric *pPositionTOPO,
                             const CTopocentric *pVelocityTOPO,
                             const CTopocentric *pAccelerationTOPO,
                             SKoefMatrRotCos sKoef,
                             SKoef_TurnTOPOtoGEOCENTRIC sKoefTurn,
                             CGeocentric *pPositionWGS,
                             CGeocentric *pVelocityWGS,
                             CGeocentric *pAccelerationWGS)
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

            // Geocentric coordinates
            pPositionWGS->m_dX = (sKoef.m_dKx1)*(pPositionTOPO->m_dXt)+
                    (sKoef.m_dKx2)*(pPositionTOPO->m_dYt)+
                    (sKoef.m_dKx3)*(pPositionTOPO->m_dZt)+sKoefTurn.m_dKx0;
            pPositionWGS->m_dY = (sKoef.m_dKy1)*(pPositionTOPO->m_dXt)+
                    (sKoef.m_dKy2)*(pPositionTOPO->m_dYt)+
                    (sKoef.m_dKy3)*(pPositionTOPO->m_dZt)+sKoefTurn.m_dKy0;
            pPositionWGS->m_dZ = (sKoef.m_dKz1)*(pPositionTOPO->m_dXt)+
                    (sKoef.m_dKz2)*(pPositionTOPO->m_dYt)+
                    (sKoef.m_dKz3)*(pPositionTOPO->m_dZt)+sKoefTurn.m_dKz0;

        }
        else
        {
            pPositionWGS = nullptr;
        }



        if (pVelocityTOPO)
        {
            // Coordinates of the speed
            pVelocityWGS->m_dX = (sKoef.m_dKx1)*(pVelocityTOPO->m_dXt)+
                    (sKoef.m_dKx2)*(pVelocityTOPO->m_dYt)+
                    (sKoef.m_dKx3)*(pVelocityTOPO->m_dZt);
            pVelocityWGS->m_dY = (sKoef.m_dKy1)*(pVelocityTOPO->m_dXt)+
                    (sKoef.m_dKy2)*(pVelocityTOPO->m_dYt)+
                    (sKoef.m_dKy3)*(pVelocityTOPO->m_dZt);
            pVelocityWGS->m_dZ = (sKoef.m_dKz1)*(pVelocityTOPO->m_dXt)+
                    (sKoef.m_dKz2)*(pVelocityTOPO->m_dYt)+
                    (sKoef.m_dKz3)*(pVelocityTOPO->m_dZt);
        }
        else
        {
            pVelocityWGS = nullptr;
        }



        if (pAccelerationTOPO)
        {
            // Coordinates of the acceleration
            pAccelerationWGS->m_dX = (sKoef.m_dKx1)*(pAccelerationTOPO->m_dXt)+
                    (sKoef.m_dKx2)*(pAccelerationTOPO->m_dYt)+
                    (sKoef.m_dKx3)*(pAccelerationTOPO->m_dZt);
            pAccelerationWGS->m_dY = (sKoef.m_dKy1)*(pAccelerationTOPO->m_dXt)+
                    (sKoef.m_dKy2)*(pAccelerationTOPO->m_dYt)+
                    (sKoef.m_dKy3)*(pAccelerationTOPO->m_dZt);
            pAccelerationWGS->m_dZ = (sKoef.m_dKz1)*(pAccelerationTOPO->m_dXt)+
                    (sKoef.m_dKz2)*(pAccelerationTOPO->m_dYt)+
                    (sKoef.m_dKz3)*(pAccelerationTOPO->m_dZt);
        }
        else
        {
            pAccelerationWGS = nullptr;
        }
    }
    else
    {
        pPositionWGS = nullptr;
        pVelocityWGS = nullptr;
        pAccelerationWGS = nullptr;
    }

    return true;

}


// PACKAGE		: Conversion
// FUNCTION      : GEOCENTRIC_TOPO_SHIFT
//
// DESCRIPTION   : Recalculation from the geocentric system of coordinates
//                 to the topocentric system of coordinates
//                 All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS        : Geocentric coordinates:
//                 pPositionWGS - position,
//		   pVelocityWGS - velocity,
//		   pAccelerationWGS - acceleration;  
//                 sKoefTR - coefficients of the transpose of a matrix of rotational cosines,
//                 sKoefTurn - the matrix of transfer of coefficients from WGS to TOPO;
//
// RETURNS	 : Topocentric coordinates:
//                 pPositionTOPO - position,
//		   pVelocityTOPO - velocity,
//		   pAccelerationTOPO - acceleration;
//
bool   GEOCENTRIC_TOPO_SHIFT(const CGeocentric *pPositionWGS,
                             const CGeocentric *pVelocityWGS,
                             const CGeocentric *pAccelerationWGS,
                             SKoefTransposeMatr sKoefTR,
                             SKoef_TurnGEOCENTRICtoTOPO sKoefTurn,
                             CTopocentric *pPositionTOPO,
                             CTopocentric *pVelocityTOPO,
                             CTopocentric *pAccelerationTOPO)
{

    
    if (pPositionWGS || pVelocityWGS || pAccelerationWGS)
    {

        if (pPositionWGS)
        {
            if ((pPositionWGS->m_dX < cdGEOCEN_X_Y_Z_MIN) ||
                    (pPositionWGS->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
                    (pPositionWGS->m_dY < cdGEOCEN_X_Y_Z_MIN) ||
                    (pPositionWGS->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
                    (pPositionWGS->m_dZ < cdGEOCEN_X_Y_Z_MIN) ||
                    (pPositionWGS->m_dZ > cdGEOCEN_X_Y_Z_MAX))
            {
                return false;
            }

            // Topocentric coordinates
            pPositionTOPO->m_dXt = (sKoefTR.m_dKx1)*(pPositionWGS->m_dX)+
                    (sKoefTR.m_dKx2)*(pPositionWGS->m_dY)+
                    (sKoefTR.m_dKx3)*(pPositionWGS->m_dZ)-sKoefTurn.m_dKx0;
            pPositionTOPO->m_dYt = (sKoefTR.m_dKy1)*(pPositionWGS->m_dX)+
                    (sKoefTR.m_dKy2)*(pPositionWGS->m_dY)+
                    (sKoefTR.m_dKy3)*(pPositionWGS->m_dZ)-sKoefTurn.m_dKy0;
            pPositionTOPO->m_dZt = (sKoefTR.m_dKz1)*(pPositionWGS->m_dX)+
                    (sKoefTR.m_dKz2)*(pPositionWGS->m_dY)+
                    (sKoefTR.m_dKz3)*(pPositionWGS->m_dZ)-sKoefTurn.m_dKz0;
        }
        else
        {
            pPositionTOPO = nullptr;
        }


        if (pVelocityWGS)
        {
            // Coordinates of the speed
            pVelocityTOPO->m_dXt = (sKoefTR.m_dKx1)*(pVelocityWGS->m_dX)+
                    (sKoefTR.m_dKx2)*(pVelocityWGS->m_dY)+
                    (sKoefTR.m_dKx3)*(pVelocityWGS->m_dZ);
            pVelocityTOPO->m_dYt = (sKoefTR.m_dKy1)*(pVelocityWGS->m_dX)+
                    (sKoefTR.m_dKy2)*(pVelocityWGS->m_dY)+
                    (sKoefTR.m_dKy3)*(pVelocityWGS->m_dZ);
            pVelocityTOPO->m_dZt = (sKoefTR.m_dKz1)*(pVelocityWGS->m_dX)+
                    (sKoefTR.m_dKz2)*(pVelocityWGS->m_dY)+
                    (sKoefTR.m_dKz3)*(pVelocityWGS->m_dZ);
        }
        else
        {
            pVelocityTOPO = nullptr;
        }


        if (pAccelerationWGS)
        {
            // Coordinates of the a cceleration
            pAccelerationTOPO->m_dXt = (sKoefTR.m_dKx1)*(pAccelerationWGS->m_dX)+
                    (sKoefTR.m_dKx2)*(pAccelerationWGS->m_dY)+
                    (sKoefTR.m_dKx3)*(pAccelerationWGS->m_dZ);
            pAccelerationTOPO->m_dYt = (sKoefTR.m_dKy1)*(pAccelerationWGS->m_dX)+
                    (sKoefTR.m_dKy2)*(pAccelerationWGS->m_dY)+
                    (sKoefTR.m_dKy3)*(pAccelerationWGS->m_dZ);
            pAccelerationTOPO->m_dZt = (sKoefTR.m_dKz1)*(pAccelerationWGS->m_dX)+
                    (sKoefTR.m_dKz2)*(pAccelerationWGS->m_dY)+
                    (sKoefTR.m_dKz3)*(pAccelerationWGS->m_dZ);
        }
        else
        {
            pAccelerationTOPO = nullptr;
        }
    }
    else
    {
        pPositionTOPO = nullptr;
        pVelocityTOPO = nullptr;
        pAccelerationTOPO = nullptr;
    }

    return true;

}



// PACKAGE		: Conversion
// FUNCTION	    : Init_TOPO_GEOCENTRIC_SHIFT
//
// DESCRIPTION	    : Function of initialization in recalculation from the Topocentric system of
//                    coordinates to the Geocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//                    
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                
// RETURNS	    : pKoef - coefficients of the matrix of rotational cosines,
//                    pKoefTurnT_G - the matrix of transfer coefficients from TOPO to WGS;
//
bool   Init_TOPO_GEOCENTRIC_SHIFT (const CGeodesic *pGEO,
                                   SKoefMatrRotCos *pKoef,
                                   SKoef_TurnTOPOtoGEOCENTRIC *pKoefTurnT_G)
{
    if((!pGEO) ||
            (!pKoef) ||
            (!pKoefTurnT_G) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))
    {
        return false;
    }


    // Intermediate variables
    double dN;       // The radius of the main vertical curve


    // The radius of the main vertical curve
    dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO->m_dLatitude),2.0));


    // Coefficients of the matrix of rotational displacement
    pKoef->m_dKx1 = -cos(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoef->m_dKx2 = cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoef->m_dKx3 = -sin(pGEO->m_dLongitude);

    pKoef->m_dKy1 = -sin(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoef->m_dKy2 = sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoef->m_dKy3 = cos(pGEO->m_dLongitude);

    pKoef->m_dKz1 = cos(pGEO->m_dLatitude);
    pKoef->m_dKz2 = sin(pGEO->m_dLatitude);
    pKoef->m_dKz3 = 0;


    // Coefficients of the matrix of transposition
    pKoefTurnT_G->m_dKx0 = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTurnT_G->m_dKy0 = (dN+(pGEO->m_dAltitude))*sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTurnT_G->m_dKz0 = (dN*(1.0-cdFirstEccentricity)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);

    return true;


}

// PACKAGE		: Conversion
// FUNCTION	    : Init_TOPO_GEOCENTRIC_SHIFT_FOR_BALL
//
// DESCRIPTION	    : Function of initialization in recalculation from the Topocentric system of
//                    coordinates to the Geocentric system of coordinates. Earth is ball.
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                  dEarthRadius - earth radius;
// RETURNS	    : pKoef - coefficients of the matrix of rotational cosines,
//                    pKoefTurnT_G - the matrix of transfer coefficients from TOPO to WGS;
//
bool   Init_TOPO_GEOCENTRIC_SHIFT_FOR_BALL (const CGeodesic *pGEO,
                                            SKoefMatrRotCos *pKoef,
                                            SKoef_TurnTOPOtoGEOCENTRIC *pKoefTurnT_G,double dEarthRadius)
{
    if((!pGEO) ||
            (!pKoef) ||
            (!pKoefTurnT_G) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))
    {
        return false;
    }


    // Intermediate variables
    double dN;       // The radius of the main vertical curve


    // The radius of the main vertical curve
    dN = dEarthRadius; //cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO->m_dLatitude),2.0));


    // Coefficients of the matrix of rotational displacement
    pKoef->m_dKx1 = -cos(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoef->m_dKx2 = cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoef->m_dKx3 = -sin(pGEO->m_dLongitude);

    pKoef->m_dKy1 = -sin(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoef->m_dKy2 = sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoef->m_dKy3 = cos(pGEO->m_dLongitude);

    pKoef->m_dKz1 = cos(pGEO->m_dLatitude);
    pKoef->m_dKz2 = sin(pGEO->m_dLatitude);
    pKoef->m_dKz3 = 0;


    // Coefficients of the matrix of transposition
    pKoefTurnT_G->m_dKx0 = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTurnT_G->m_dKy0 = (dN+(pGEO->m_dAltitude))*sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTurnT_G->m_dKz0 = (dN*(1.0)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);

    return true;

}

// PACKAGE		: Conversion
// FUNCTION	    : Init_GEOCENTRIC_TOPO_SHIFT 
//
// DESCRIPTION      : Function of initialization in recalculation from the Geocentric system of 
//                    coordinates to the Topocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//                
// INPUTS 	    : pGEO - geodesic coordinates of the  origin;
//                
// RETURNS	    : pKoefTR - coefficients of the transpose of a matrix of rotational cosines,
//                    pKoefTurnG_T - the matrix of transfer of coefficients from WGS to TOPO;
//
bool   Init_GEOCENTRIC_TOPO_SHIFT (const CGeodesic  *pGEO,
                                   SKoefTransposeMatr *pKoefTR,
                                   SKoef_TurnGEOCENTRICtoTOPO *pKoefTurnG_T)
{
    if((!pGEO) ||
            (!pKoefTR) ||
            (!pKoefTurnG_T) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))
    {
        return false;
    }


    // Intermediate variables
    double dN;           // The radius of the main vertical curve
    double dK1,dK2,dK3;  // Coefficients


    // The radius of the main vertical curve
    dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pGEO->m_dLatitude),2.0));


    // Coefficients of the transpose of a matrix of rotational cosines
    pKoefTR->m_dKx1 = -cos(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoefTR->m_dKx2 = -sin(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoefTR->m_dKx3 = cos(pGEO->m_dLatitude);

    pKoefTR->m_dKy1 = cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTR->m_dKy2 = sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTR->m_dKy3 = sin(pGEO->m_dLatitude);

    pKoefTR->m_dKz1 = -sin(pGEO->m_dLongitude);
    pKoefTR->m_dKz2 = cos(pGEO->m_dLongitude);
    pKoefTR->m_dKz3 = 0;


    //  Coefficients of the matrix of transposition
    dK1 = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    dK2 = (dN+(pGEO->m_dAltitude))*sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    dK3 = (dN*(1.0-cdFirstEccentricity)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);


    pKoefTurnG_T->m_dKx0 = (pKoefTR->m_dKx1)*dK1+(pKoefTR->m_dKx2)*dK2+(pKoefTR->m_dKx3)*dK3;
    pKoefTurnG_T->m_dKy0 = (pKoefTR->m_dKy1)*dK1+(pKoefTR->m_dKy2)*dK2+(pKoefTR->m_dKy3)*dK3;
    pKoefTurnG_T->m_dKz0 = (pKoefTR->m_dKz1)*dK1+(pKoefTR->m_dKz2)*dK2+(pKoefTR->m_dKz3)*dK3;

    return true;


}


// PACKAGE		: Conversion
// FUNCTION	    : Init_TOPO_GEOCENTRIC_SHIFT_Kr
//
// DESCRIPTION      : Function of initialization in recalculation from the Topocentric system of
//                    coordinates to the Geocentric system of coordinates
//                    (using Krasovsky parameters)
//                    All the parameters are in radians (angles) and metres (distanses).
//                
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                    (WITH SHIFT)
//                
// RETURNS	    : pKoef - coefficients of the matrix of rotational cosines,
//                    pKoefTurnT_G - the matrix of transfer coefficients from TOPO to WGS;
//
bool   Init_TOPO_GEOCENTRIC_SHIFT_Kr (const CGeodesic *pGEO,
                                      SKoefMatrRotCos *pKoef,
                                      SKoef_TurnTOPOtoGEOCENTRIC *pKoefTurnT_G)
{
    if((!pGEO) ||
            (!pKoef) ||
            (!pKoefTurnT_G) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))
    {
        return false;
    }

    // Intermediate variables
    double dN;       // The radius of the main vertical curve

    // The radius of the main vertical curve
    dN = cdEquatorAxisKr/sqrt(1.0-cdFirstEccentricityKr*pow(sin(pGEO->m_dLatitude),2.0));

    // Coefficients of the matrix of rotational displacement
    pKoef->m_dKx1 = -cos(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoef->m_dKx2 = cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoef->m_dKx3 = -sin(pGEO->m_dLongitude);

    pKoef->m_dKy1 = -sin(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoef->m_dKy2 = sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoef->m_dKy3 = cos(pGEO->m_dLongitude);

    pKoef->m_dKz1 = cos(pGEO->m_dLatitude);
    pKoef->m_dKz2 = sin(pGEO->m_dLatitude);
    pKoef->m_dKz3 = 0;

    // Coefficients of the matrix of transposition
    pKoefTurnT_G->m_dKx0 = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTurnT_G->m_dKy0 = (dN+(pGEO->m_dAltitude))*sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTurnT_G->m_dKz0 = (dN*(1.0-cdFirstEccentricityKr)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);

    return true;
}


// PACKAGE		: Conversion
// FUNCTION	    : Init_GEOCENTRIC_TOPO_SHIFT_Kr 
//
// DESCRIPTION	    : Function of initialization in recalculation from the Geocentric system of 
//                    coordinates to the Topocentric system of coordinates
//                    (using Krasovsky parameters)
//                    All the parameters are in radians (angles) and metres (distanses).
//                
// INPUTS 	    : pGEO - geodesic coordinates of the  origin;
//                    (WITH SHIFT)
//                
// RETURNS	    : pKoefTR - coefficients of the transpose of a matrix of rotational cosines,
//                    pKoefTurnG_T - the matrix of transfer of coefficients from WGS to TOPO;
//
bool   Init_GEOCENTRIC_TOPO_SHIFT_Kr (const CGeodesic *pGEO,
                                      SKoefTransposeMatr *pKoefTR,
                                      SKoef_TurnGEOCENTRICtoTOPO *pKoefTurnG_T)
{
    if((!pGEO) ||
            (!pKoefTR) ||
            (!pKoefTurnG_T) ||
            (pGEO->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pGEO->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pGEO->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pGEO->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pGEO->m_dAltitude < cd_H_MIN) ||
            (pGEO->m_dAltitude > cd_H_MAX))
    {
        return false;
    }

    // Intermediate variables
    double dN;           // The radius of the main vertical curve
    double dK1,dK2,dK3;  // Coefficients

    // The radius of the main vertical curve
    dN = cdEquatorAxisKr/sqrt(1.0-cdFirstEccentricityKr*pow(sin(pGEO->m_dLatitude),2.0));

    // Coefficients of the transpose of a matrix of rotational cosines
    pKoefTR->m_dKx1 = -cos(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoefTR->m_dKx2 = -sin(pGEO->m_dLongitude)*sin(pGEO->m_dLatitude);
    pKoefTR->m_dKx3 = cos(pGEO->m_dLatitude);

    pKoefTR->m_dKy1 = cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTR->m_dKy2 = sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    pKoefTR->m_dKy3 = sin(pGEO->m_dLatitude);

    pKoefTR->m_dKz1 = -sin(pGEO->m_dLongitude);
    pKoefTR->m_dKz2 = cos(pGEO->m_dLongitude);
    pKoefTR->m_dKz3 = 0;

    //  Coefficients of the matrix of transposition
    dK1 = (dN+(pGEO->m_dAltitude))*cos(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    dK2 = (dN+(pGEO->m_dAltitude))*sin(pGEO->m_dLongitude)*cos(pGEO->m_dLatitude);
    dK3 = (dN*(1.0-cdFirstEccentricityKr)+(pGEO->m_dAltitude))*sin(pGEO->m_dLatitude);

    pKoefTurnG_T->m_dKx0 = (pKoefTR->m_dKx1)*dK1+(pKoefTR->m_dKx2)*dK2+(pKoefTR->m_dKx3)*dK3;
    pKoefTurnG_T->m_dKy0 = (pKoefTR->m_dKy1)*dK1+(pKoefTR->m_dKy2)*dK2+(pKoefTR->m_dKy3)*dK3;
    pKoefTurnG_T->m_dKz0 = (pKoefTR->m_dKz1)*dK1+(pKoefTR->m_dKz2)*dK2+(pKoefTR->m_dKz3)*dK3;

    return true;
}


bool   GEOCENTRIC_TOPO(const CGeodesic  *pGEO, const CGeocentric *pPositionWGS,	CTopocentric *pPositionTOPO)
{
    SKoefTransposeMatr l_KoefTR;
    SKoef_TurnGEOCENTRICtoTOPO l_KoefTurnG_T;

    if(!Init_GEOCENTRIC_TOPO_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    if(!GEOCENTRIC_TOPO_SHIFT(pPositionWGS, /*NULL*/0, /*NULL*/0, l_KoefTR, l_KoefTurnG_T, pPositionTOPO, /*NULL*/0, /*NULL*/0))
        return false;
    return true;
}

bool   GEOCENTRIC_TOPO(const CGeodesic *pGEO,
                       const CGeocentric *pPositionWGS,	const CGeocentric *pVelocityWGS, const CGeocentric *pAccelerationWGS,
                       CTopocentric *pPositionTOPO, CTopocentric *pVelocityTOPO, CTopocentric *pAccelerationTOPO)
{
    SKoefTransposeMatr l_KoefTR;
    SKoef_TurnGEOCENTRICtoTOPO l_KoefTurnG_T;

    if(!Init_GEOCENTRIC_TOPO_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    if(!GEOCENTRIC_TOPO_SHIFT(pPositionWGS, pVelocityWGS, pAccelerationWGS, l_KoefTR, l_KoefTurnG_T, pPositionTOPO, pVelocityTOPO, pAccelerationTOPO))
        return false;
    return true;
}

bool   TOPO_GEOCENTRIC(const CGeodesic  *pGEO, const CTopocentric *pPositionTOPO, CGeocentric *pPositionWGS)
{

    SKoefMatrRotCos l_Koef;
    SKoef_TurnTOPOtoGEOCENTRIC l_KoefTurnT_G;

    if(!Init_TOPO_GEOCENTRIC_SHIFT(pGEO, &l_Koef, &l_KoefTurnT_G))
        return false;

    if(!TOPO_GEOCENTRIC_SHIFT(pPositionTOPO, 0, 0, l_Koef, l_KoefTurnT_G, pPositionWGS, 0, 0))
        return false;

    return true;
}

bool   TOPO_GEOCENTRIC_FOR_BALL(const CGeodesic  *pGEO, const CTopocentric *pPositionTOPO, CGeocentric *pPositionWGS, double dEarthRadius)
{
    SKoefMatrRotCos l_Koef;
    SKoef_TurnTOPOtoGEOCENTRIC l_KoefTurnT_G;

    if(!Init_TOPO_GEOCENTRIC_SHIFT_FOR_BALL(pGEO, &l_Koef, &l_KoefTurnT_G,dEarthRadius))
        return false;

    if(!TOPO_GEOCENTRIC_SHIFT(pPositionTOPO, 0, 0, l_Koef, l_KoefTurnT_G, pPositionWGS, 0, 0))
        return false;

    return true;
}

bool   TOPO_GEOCENTRIC(const CGeodesic *pGEO,
                       const CTopocentric *pPositionTOPO, const CTopocentric *pVelocityTOPO, const CTopocentric *pAccelerationTOPO,
                       CGeocentric *pPositionWGS, CGeocentric *pVelocityWGS, CGeocentric *pAccelerationWGS)
{

    SKoefMatrRotCos l_Koef;
    SKoef_TurnTOPOtoGEOCENTRIC l_KoefTurnT_G;

    if(!Init_TOPO_GEOCENTRIC_SHIFT(pGEO, &l_Koef, &l_KoefTurnT_G))
        return false;

    if(!TOPO_GEOCENTRIC_SHIFT(pPositionTOPO, pVelocityTOPO, pAccelerationTOPO, l_Koef, l_KoefTurnT_G, pPositionWGS, pVelocityWGS, pAccelerationWGS))
        return false;

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_GeocentricToTopo
//
// DESCRIPTION     : Function of recalculation covariance matrix from the Geocentric system of coordinates to the
//                   topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pGEO - Geodesic coordinates;
//                  CM_Geo - the covariance matrix of errors in the determination of Geodesic
//                  coordinates;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates;
//
bool Recount_CovMatr_GeocentricToTopo(const CGeodesic *pGEO, Square_Matrix<3> *CM_Geo, Square_Matrix<3> * CM_NUE )
{

    SKoefTransposeMatr l_KoefTR;
    SKoef_TurnGEOCENTRICtoTOPO l_KoefTurnG_T;

    if(!Init_GEOCENTRIC_TOPO_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    CM_NUE->Reset(3,3);

    Square_Matrix<3> Matr;
    Matr.s_m=3;
    Matr.s_n=3;

    Matr.M[0][0]=l_KoefTR.m_dKx1;
    Matr.M[0][1]=l_KoefTR.m_dKx2;
    Matr.M[0][2]=l_KoefTR.m_dKx3;

    Matr.M[1][0]=l_KoefTR.m_dKy1;
    Matr.M[1][1]=l_KoefTR.m_dKy2;
    Matr.M[1][2]=l_KoefTR.m_dKy3;

    Matr.M[2][0]=l_KoefTR.m_dKz1;
    Matr.M[2][1]=l_KoefTR.m_dKz2;
    Matr.M[2][2]=l_KoefTR.m_dKz3;


    Square_Matrix<3> Matr_tr=Square_Matrix<3>(3,3);
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_Geo,&Matr_tr,CM_NUE);

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_GeocentricToTopo
//
// DESCRIPTION     : Function of recalculation covariance matrix from the Geocentric system of coordinates to the
//                   topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pGEO - Geodesic coordinates;
//                  CM_Geo - the covariance matrix of errors in the determination of Geodesic
//                  coordinates, velocity;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity;
//

bool Recount_CovMatr_GeocentricToTopo(const CGeodesic *pGEO, Square_Matrix<6> *CM_Geo, Square_Matrix<6> * CM_NUE )
{
    SKoefTransposeMatr l_KoefTR;
    SKoef_TurnGEOCENTRICtoTOPO l_KoefTurnG_T;

    if(!Init_GEOCENTRIC_TOPO_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    CM_NUE->Reset(6,6);

    Square_Matrix<6> Matr;
    Matr.s_m=6;
    Matr.s_n=6;

    Matr.M[0][0]=l_KoefTR.m_dKx1;
    Matr.M[0][1]=l_KoefTR.m_dKx2;
    Matr.M[0][2]=l_KoefTR.m_dKx3;

    Matr.M[1][0]=l_KoefTR.m_dKy1;
    Matr.M[1][1]=l_KoefTR.m_dKy2;
    Matr.M[1][2]=l_KoefTR.m_dKy3;

    Matr.M[2][0]=l_KoefTR.m_dKz1;
    Matr.M[2][1]=l_KoefTR.m_dKz2;
    Matr.M[2][2]=l_KoefTR.m_dKz3;

    Matr.M[3][3]=l_KoefTR.m_dKx1;
    Matr.M[3][4]=l_KoefTR.m_dKx2;
    Matr.M[3][5]=l_KoefTR.m_dKx3;

    Matr.M[4][3]=l_KoefTR.m_dKy1;
    Matr.M[4][4]=l_KoefTR.m_dKy2;
    Matr.M[4][5]=l_KoefTR.m_dKy3;

    Matr.M[5][3]=l_KoefTR.m_dKz1;
    Matr.M[5][4]=l_KoefTR.m_dKz2;
    Matr.M[5][5]=l_KoefTR.m_dKz3;

    Square_Matrix<6> Matr_tr=Square_Matrix<6>(6,6);
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_Geo,&Matr_tr,CM_NUE);



    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_GeocentricToTopo
//
// DESCRIPTION     : Function of recalculation covariance matrix from the Geocentric system of coordinates to the
//                   topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pGEO - Geodesic coordinates;
//                  CM_Geo - the covariance matrix of errors in the determination of Geodesic
//                  coordinates, velocity and acceleration components;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity and acceleration components;
//

bool Recount_CovMatr_GeocentricToTopo(const CGeodesic *pGEO, Square_Matrix<9> *CM_Geo, Square_Matrix<9> * CM_NUE )
{
    SKoefTransposeMatr l_KoefTR;
    SKoef_TurnGEOCENTRICtoTOPO l_KoefTurnG_T;

    if(!Init_GEOCENTRIC_TOPO_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    CM_NUE->Reset(9,9);

    Square_Matrix<9> Matr;
    Matr.s_m=9;
    Matr.s_n=9;

    Matr.M[0][0]=l_KoefTR.m_dKx1;
    Matr.M[0][1]=l_KoefTR.m_dKx2;
    Matr.M[0][2]=l_KoefTR.m_dKx3;

    Matr.M[1][0]=l_KoefTR.m_dKy1;
    Matr.M[1][1]=l_KoefTR.m_dKy2;
    Matr.M[1][2]=l_KoefTR.m_dKy3;

    Matr.M[2][0]=l_KoefTR.m_dKz1;
    Matr.M[2][1]=l_KoefTR.m_dKz2;
    Matr.M[2][2]=l_KoefTR.m_dKz3;

    Matr.M[3][3]=l_KoefTR.m_dKx1;
    Matr.M[3][4]=l_KoefTR.m_dKx2;
    Matr.M[3][5]=l_KoefTR.m_dKx3;

    Matr.M[4][3]=l_KoefTR.m_dKy1;
    Matr.M[4][4]=l_KoefTR.m_dKy2;
    Matr.M[4][5]=l_KoefTR.m_dKy3;

    Matr.M[5][3]=l_KoefTR.m_dKz1;
    Matr.M[5][4]=l_KoefTR.m_dKz2;
    Matr.M[5][5]=l_KoefTR.m_dKz3;

    Matr.M[6][6]=l_KoefTR.m_dKx1;
    Matr.M[6][7]=l_KoefTR.m_dKx2;
    Matr.M[6][8]=l_KoefTR.m_dKx3;

    Matr.M[7][6]=l_KoefTR.m_dKy1;
    Matr.M[7][7]=l_KoefTR.m_dKy2;
    Matr.M[7][8]=l_KoefTR.m_dKy3;

    Matr.M[8][6]=l_KoefTR.m_dKz1;
    Matr.M[8][7]=l_KoefTR.m_dKz2;
    Matr.M[8][8]=l_KoefTR.m_dKz3;

    Square_Matrix<9> Matr_tr=Square_Matrix<9>(9,9);
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_Geo,&Matr_tr,CM_NUE);

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_TopoToGeocentric
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   geocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pGEO - Geodesic coordinates;
//                  CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of geocentric
//                  coordinates;
//

bool Recount_CovMatr_TopoToGeocentric(const CGeodesic *pGEO, Square_Matrix<3> *CM_NUE, Square_Matrix<3> *CM_Geo  )
{
    SKoefMatrRotCos l_KoefTR;
    SKoef_TurnTOPOtoGEOCENTRIC l_KoefTurnG_T;

    if(!Init_TOPO_GEOCENTRIC_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    CM_Geo->Reset(3,3);

    Square_Matrix<3> Matr;
    Matr.s_m=3;
    Matr.s_n=3;

    Matr.M[0][0]=l_KoefTR.m_dKx1;
    Matr.M[0][1]=l_KoefTR.m_dKx2;
    Matr.M[0][2]=l_KoefTR.m_dKx3;

    Matr.M[1][0]=l_KoefTR.m_dKy1;
    Matr.M[1][1]=l_KoefTR.m_dKy2;
    Matr.M[1][2]=l_KoefTR.m_dKy3;

    Matr.M[2][0]=l_KoefTR.m_dKz1;
    Matr.M[2][1]=l_KoefTR.m_dKz2;
    Matr.M[2][2]=l_KoefTR.m_dKz3;



    Square_Matrix<3> Matr_tr=Square_Matrix<3>(3,3);
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_NUE,&Matr_tr,CM_Geo);

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_TopoToGeocentric
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   geocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pGEO - Geodesic coordinates;
//                  CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of geocentric
//                  coordinates, velocity;
//

bool Recount_CovMatr_TopoToGeocentric(const CGeodesic *pGEO, Square_Matrix<6> *CM_NUE, Square_Matrix<6> *CM_Geo  )
{
    SKoefMatrRotCos l_KoefTR;
    SKoef_TurnTOPOtoGEOCENTRIC l_KoefTurnG_T;

    if(!Init_TOPO_GEOCENTRIC_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    CM_Geo->Reset(6,6);

    Square_Matrix<6> Matr;
    Matr.s_m=6;
    Matr.s_n=6;

    Matr.M[0][0]=l_KoefTR.m_dKx1;
    Matr.M[0][1]=l_KoefTR.m_dKx2;
    Matr.M[0][2]=l_KoefTR.m_dKx3;

    Matr.M[1][0]=l_KoefTR.m_dKy1;
    Matr.M[1][1]=l_KoefTR.m_dKy2;
    Matr.M[1][2]=l_KoefTR.m_dKy3;

    Matr.M[2][0]=l_KoefTR.m_dKz1;
    Matr.M[2][1]=l_KoefTR.m_dKz2;
    Matr.M[2][2]=l_KoefTR.m_dKz3;

    Matr.M[3][3]=l_KoefTR.m_dKx1;
    Matr.M[3][4]=l_KoefTR.m_dKx2;
    Matr.M[3][5]=l_KoefTR.m_dKx3;

    Matr.M[4][3]=l_KoefTR.m_dKy1;
    Matr.M[4][4]=l_KoefTR.m_dKy2;
    Matr.M[4][5]=l_KoefTR.m_dKy3;

    Matr.M[5][3]=l_KoefTR.m_dKz1;
    Matr.M[5][4]=l_KoefTR.m_dKz2;
    Matr.M[5][5]=l_KoefTR.m_dKz3;

    Square_Matrix<6> Matr_tr=Square_Matrix<6>(6,6);
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_NUE,&Matr_tr,CM_Geo);

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_TopoToGeocentric
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   geocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pGEO - Geodesic coordinates;
//                  CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity and acceleration components;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of geocentric
//                  coordinates, velocity and acceleration components;
//

bool Recount_CovMatr_TopoToGeocentric(const CGeodesic *pGEO, Square_Matrix<9> *CM_NUE, Square_Matrix<9> *CM_Geo  )
{
    SKoefMatrRotCos l_KoefTR;
    SKoef_TurnTOPOtoGEOCENTRIC l_KoefTurnG_T;

    if(!Init_TOPO_GEOCENTRIC_SHIFT(pGEO, &l_KoefTR, &l_KoefTurnG_T))
        return false;

    CM_Geo->Reset(9,9);

    Square_Matrix<9> Matr;
    Matr.s_m=9;
    Matr.s_n=9;

    Matr.M[0][0]=l_KoefTR.m_dKx1;
    Matr.M[0][1]=l_KoefTR.m_dKx2;
    Matr.M[0][2]=l_KoefTR.m_dKx3;

    Matr.M[1][0]=l_KoefTR.m_dKy1;
    Matr.M[1][1]=l_KoefTR.m_dKy2;
    Matr.M[1][2]=l_KoefTR.m_dKy3;

    Matr.M[2][0]=l_KoefTR.m_dKz1;
    Matr.M[2][1]=l_KoefTR.m_dKz2;
    Matr.M[2][2]=l_KoefTR.m_dKz3;

    Matr.M[3][3]=l_KoefTR.m_dKx1;
    Matr.M[3][4]=l_KoefTR.m_dKx2;
    Matr.M[3][5]=l_KoefTR.m_dKx3;

    Matr.M[4][3]=l_KoefTR.m_dKy1;
    Matr.M[4][4]=l_KoefTR.m_dKy2;
    Matr.M[4][5]=l_KoefTR.m_dKy3;

    Matr.M[5][3]=l_KoefTR.m_dKz1;
    Matr.M[5][4]=l_KoefTR.m_dKz2;
    Matr.M[5][5]=l_KoefTR.m_dKz3;

    Matr.M[6][6]=l_KoefTR.m_dKx1;
    Matr.M[6][7]=l_KoefTR.m_dKx2;
    Matr.M[6][8]=l_KoefTR.m_dKx3;

    Matr.M[7][6]=l_KoefTR.m_dKy1;
    Matr.M[7][7]=l_KoefTR.m_dKy2;
    Matr.M[7][8]=l_KoefTR.m_dKy3;

    Matr.M[8][6]=l_KoefTR.m_dKz1;
    Matr.M[8][7]=l_KoefTR.m_dKz2;
    Matr.M[8][8]=l_KoefTR.m_dKz3;

    Square_Matrix<9> Matr_tr=Square_Matrix<9>(9,9);
    Matr.transp(&Matr_tr);
    Matr.MatrXMatrXMatr(CM_NUE,&Matr_tr,CM_Geo);

    return true;
}
