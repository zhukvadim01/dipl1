// PACKAGE		: Conversion
// FILE     : Altitude.cpp
//
// AUTHOR   : Marina Mukhortova, 2003
// DESCRIPTION	:implementation file for function of calculation of the altitude

#include <cmath>
#include <cstdio>
#include <cerrno>

#include "Altitude.h"
#include "ConstRecalc.h"




// PACKAGE		: Conversion
// FUNCTION	    : CalculationALTITUDE
//
// DESCRIPTION      : The function of calculation of the altitude using topocentric coordinates.
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pTOPO - topocentric coordinates:
//                    dAltitude_ref - altitude of the fixation point of the reference point
//                                
// RETURNS	    : pdAltitude
//
bool   CalculationALTITUDE (const CTopocentric *pTOPO,
                            double dAltitude_ref,
                            double *pdAltitude)
{
    if((!pTOPO) ||
            (pTOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pTOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (pTOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dZt > cdTOPO_X_Y_Z_MAX))

    {
        return false;
    }

    // Intermediate variables
    double dD; // distance
    double dAlt;

    dD = sqrt(pTOPO->m_dXt*pTOPO->m_dXt + pTOPO->m_dZt*pTOPO->m_dZt); // distance

    // Count of the altitude
    dAlt = sqrt(pow(dD,2)+pow((pTOPO->m_dYt+dAltitude_ref+cdMainRadius),2)) - cdMainRadius;

    *pdAltitude = dAlt;

    return true;
}



// PACKAGE		: Conversion
// FUNCTION	    : CalculationTOPO_Y_Coordinate
//
// DESCRIPTION	    : The function of calculation of the topocentric coordinates (Y)
//                    using given altitude.
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pTOPO - topocentric coordinates:
//                    dAltitude_ref - altitude of the fixation point of the reference point
//                    dAltitude - altitude of the point above sea level
//                               
// RETURNS	    : pTOPO - topocentric coordinates
//
bool   CalculationTOPO_Y_Coordinate (CTopocentric *pTOPO,
                                     double dAltitude_ref,
                                     double dAltitude)
{
    if ((!pTOPO) ||
            (pTOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pTOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }


    // Topocentric coordinates

    pTOPO->m_dYt = sqrt(pow((dAltitude+cdMainRadius),2)-pTOPO->m_dXt*pTOPO->m_dXt-
                        pTOPO->m_dZt*pTOPO->m_dZt)-dAltitude_ref-cdMainRadius;


    return true;

}
