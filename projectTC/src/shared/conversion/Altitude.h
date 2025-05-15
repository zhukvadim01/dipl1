// PACKAGE		: Conversion
// FILE         : Altitude.h  
// AUTHOR       : Marina Mukhortova, 2003
// DESCRIPTION	:Header file for function of calculation of the altitude

//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __ALTITUDE_H__
#define __ALTITUDE_H__

#include "Topocentric.h"


// PACKAGE		: Conversion
// 
// FUNCTION	    : CalculationALTITUDE
//
// DESCRIPTION      : Function of calculation of the altitude using topocentric coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//                
// INPUTS 	    : pTopo - topocentric coordinates;
//                    dAltitude_ref - altitude of the fixation point of the reference point;
//            
// RETURNS	    : pdAltitude
//
bool   CalculationALTITUDE (const CTopocentric *pTopo,
                            double dAltitude_ref,
                            double *pdAltitude);


// PACKAGE		: Conversion
//
// FUNCTION	    : CalculationTOPO_Y_Coordinate
//
// DESCRIPTION      : The function of calculation of the topocentric coordinates
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
                                     double dAltitude);


#endif
