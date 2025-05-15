// PACKAGE		: Conversion
// FILE         : Topo1_Topo2.h   
// 
// AUTHOR	: Marina Mukhortova, 2003
//
//DESCRIPTION :Header file for function of recalculation topocentric coordinates system

//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __Topo1_Topo2_H__
#define __Topo1_Topo2_H__

#include "Structures.h"
#include "Topocentric.h"
#include "Geodesic.h"


// PACKAGE		: Conversion
// FUNCTION        : InitTopo1_Topo2_SHIFT
//
// DESCRIPTION     : Function of initialization factors in recalculation from the TOPO1 to the TOPO2
//                   All the parameters are in radians (angles) and metres (distanses).
//                   
// INPUTS          : pGEO1 - geodesic coordinates of the first TPSK origin;
//                   pGEO2 - geodesic coordinates of the second  TPSK origin;
//                   (WITH SHIFT)
//                   
// RETURNS         : pRecal - coefficients
//
bool   InitTopo1_Topo2_SHIFT (const CGeodesic *pGEO1,
                              const CGeodesic *pGEO2,
                              SKoefRecal  *pRecal);



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
bool   Topo1_Topo2_SHIFT(const CTopocentric *pPositionTOPO1,
                         const CTopocentric *pVelocityTOPO1,
                         const CTopocentric *pAccelerationTOPO1,
                         SKoefRecal  sRecal,
                         CTopocentric *pPositionTOPO2,
                         CTopocentric *pVelocityTOPO2,
                         CTopocentric *pAccelerationTOPO2);

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
                   CTopocentric *pAccelerationTOPO_AB);

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
                                 SKoefRecal  *pRecal);

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

bool Topo1_Topo2(const CGeodesic *pCenter1,const CGeodesic *pCenter2, const CTopocentric *pPositionTopo1,CTopocentric *pPositionTopo2);

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

bool Topo1_Topo2(const CGeodesic *pCenter1,const CGeodesic *pCenter2,
                 const CTopocentric *pPositionTopo1, CTopocentric *pPositionTopo2,
                 const CTopocentric *pVelocityTopo1, CTopocentric *pVelocityTopo2);

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
                 const CTopocentric *pAccelerationTopo1, CTopocentric *pAccelerationTopo2);

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


bool Recount_CovMatr_TopoToTopo(const CGeodesic *pGEO_1,const CGeodesic *pGEO_2, Square_Matrix<3> *CM_NUE_1, Square_Matrix<3> *CM_NUE_2);

bool Recount_CovMatr_TopoToTopo(const CGeodesic *pGEO_1,const CGeodesic *pGEO_2, Square_Matrix<6> *CM_NUE_1, Square_Matrix<6> *CM_NUE_2);

bool Recount_CovMatr_TopoToTopo(const CGeodesic *pGEO_1,const CGeodesic *pGEO_2, Square_Matrix<9> *CM_NUE_1, Square_Matrix<9> *CM_NUE_2);

#endif
