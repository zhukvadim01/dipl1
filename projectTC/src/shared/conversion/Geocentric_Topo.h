// PACKAGE		: Conversion
// FILE         : Geocentric_Topo.h     
// AUTHOR	: Marina Mukhortova, 2003
//DESCRIPTION :Header file for function of recalculation Geocentric coordinates system and geodesic
//                   topocentric system
//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __Geocentric_Topo_H__
#define __Geocentric_Topo_H__

#include "Structures.h"
#include "Geocentric.h"
#include "Topocentric.h"
#include "Geodesic.h"

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
                             CTopocentric *pAccelerationTOPO);


// PACKAGE		: Conversion
// FUNCTION	    : TOPO_GEOCENTRIC_SHIFT
//
// DESCRIPTION	    : Function of recalculation from the topocentric system of coordinates
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
                             CGeocentric *pAccelerationWGS);



// PACKAGE		: Conversion
// FUNCTION	    : Init_GEOCENTRIC_TOPO_SHIFT 
//
// DESCRIPTION	    : Function of initialization in recalculation from the Geocentric system of 
//                    coordinates to the Topocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//                
// INPUTS 	    : pGEO - geodesic coordinates of the  origin;
//                
// RETURNS	    : pKoefTR - coefficients of the transpose of a matrix of rotational cosines,
//                    pKoefTurnG_T - the matrix of transfer of coefficients from WGS to TOPO;
//
bool   Init_GEOCENTRIC_TOPO_SHIFT (const CGeodesic *pGEO,
                                   SKoefTransposeMatr *pKoefTR,
                                   SKoef_TurnGEOCENTRICtoTOPO *pKoefTurnG_T);



// PACKAGE		: Conversion
// FUNCTION	    : Init_TOPO_GEOCENTRIC_SHIFT_FOR_BALL
//
// DESCRIPTION	    : Function of initialization in recalculation from the Topocentric system of
//                    coordinates to the Geocentric system of coordinates. Earth is ball.
//                    All the parameters are in radians (angles) and metres (distanses).
//                
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                dEarthRadius - earth radius;
// RETURNS	    : pKoef - coefficients of the matrix of rotational cosines,
//                    pKoefTurnT_G - the matrix of transfer coefficients from TOPO to WGS;
//	
bool   Init_TOPO_GEOCENTRIC_SHIFT_FOR_BALL (const CGeodesic *pGEO,
                                            SKoefMatrRotCos *pKoef,
                                            SKoef_TurnTOPOtoGEOCENTRIC *pKoefTurnT_G, double dEarthRadius = 6372000.0);

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
                                   SKoef_TurnTOPOtoGEOCENTRIC *pKoefTurnT_G);


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
                                      SKoef_TurnTOPOtoGEOCENTRIC *pKoefTurnT_G);

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
                                      SKoef_TurnGEOCENTRICtoTOPO *pKoefTurnG_T);


// PACKAGE		: Conversion
// FUNCTION      : GEOCENTRIC_TOPO
//
// DESCRIPTION   : Recalculation from the geocentric system of coordinates
//                 to the topocentric system of coordinates
//                 All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS        : pGEO - geodesic coordinates of the  origin;
//                 Geocentric coordinates:
//                 pPositionWGS - position,
// RETURNS       : Topocentric coordinates:
//                 pPositionTOPO - position,
bool   GEOCENTRIC_TOPO(const CGeodesic *pGEO, const CGeocentric *pPositionWGS,	CTopocentric *pPositionTOPO);

// PACKAGE		: Conversion
// FUNCTION      : GEOCENTRIC_TOPO_SHIFT
//
// DESCRIPTION   : Recalculation from the geocentric system of coordinates
//                 to the topocentric system of coordinates
//                 All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS        : pGEO - geodesic coordinates of the  origin;
//                 Geocentric coordinates:
//                 pPositionWGS - position,
//                 pVelocityWGS - velocity,
//                 pAccelerationWGS - acceleration;
//
// RETURNS	 : Topocentric coordinates:
//                 pPositionTOPO - position,
//                 pVelocityTOPO - velocity,
//                 pAccelerationTOPO - acceleration;
//
bool   GEOCENTRIC_TOPO(const CGeodesic  *pGEO,
                       const CGeocentric *pPositionWGS,const 	CGeocentric *pVelocityWGS, const CGeocentric *pAccelerationWGS,
                       CTopocentric *pPositionTOPO, CTopocentric *pVelocityTOPO, CTopocentric *pAccelerationTOPO);


// PACKAGE		: Conversion
// FUNCTION	    : TOPO_GEOCENTRIC
//
// DESCRIPTION	    : Function of recalculation from the topocentric system of coordinates
//                    to the geocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                  Topocentric coordinates:
//                    pPositionTOPO - position,
//
// RETURNS	    : Geocentric coordinates:
//                    pPositionWGS - position,
//
bool   TOPO_GEOCENTRIC(const CGeodesic *pGEO, const CTopocentric *pPositionTOPO, CGeocentric *pPositionWGS);

// PACKAGE		: Conversion
// FUNCTION	    : TOPO_GEOCENTRIC_FOR_BALL
//
// DESCRIPTION	    : Function of recalculation from the topocentric system of coordinates
//                    to the geocentric system of coordinates. Р•arth is ball.
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                  Topocentric coordinates:
//                    pPositionTOPO - position;
//                    dEarthRadius - earth radius;
//
// RETURNS	    : Geocentric coordinates:
//                    pPositionWGS - position,
//
bool   TOPO_GEOCENTRIC_FOR_BALL(const CGeodesic *pGEO, const CTopocentric *pPositionTOPO,
                                CGeocentric *pPositionWGS, double dEarthRadius = 6372000.0);

// PACKAGE		: Conversion
// FUNCTION	    : TOPO_GEOCENTRIC_SHIFT
//
// DESCRIPTION	    : Function of recalculation from the topocentric system of coordinates
//                    to the geocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates of the origin ;
//                  Topocentric coordinates:
//                      pPositionTOPO - position,
//                      pVelocityTOPO - velocity,
//                      pAccelerationTOPO - acceleration,
//
// RETURNS	    : Geocentric coordinates:
//                      pPositionWGS - position,
//                      pVelocityWGS - velocity,
//                      pAccelerationWGS - acceleration;
//
bool   TOPO_GEOCENTRIC(const CGeodesic  *pGEO,
                       const CTopocentric *pPositionTOPO, const CTopocentric *pVelocityTOPO, const CTopocentric *pAccelerationTOPO,
                       CGeocentric *pPositionWGS, CGeocentric *pVelocityWGS, CGeocentric *pAccelerationWGS);


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

bool Recount_CovMatr_GeocentricToTopo(const CGeodesic *pGEO, Square_Matrix<3> *CM_Geo, Square_Matrix<3> * CM_NUE );


// PACKAGE		: Conversion
//
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

bool Recount_CovMatr_GeocentricToTopo(const CGeodesic *pGEO, Square_Matrix<6> *CM_Geo, Square_Matrix<6> * CM_NUE );



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

bool Recount_CovMatr_GeocentricToTopo(const CGeodesic *pGEO, Square_Matrix<9> *CM_Geo, Square_Matrix<9> * CM_NUE );


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

bool Recount_CovMatr_TopoToGeocentric(const CGeodesic *pGEO, Square_Matrix<3> *CM_NUE, Square_Matrix<3> *CM_Geo  );


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

bool Recount_CovMatr_TopoToGeocentric(const CGeodesic *pGEO, Square_Matrix<6> *CM_NUE, Square_Matrix<6> *CM_Geo  );



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

bool Recount_CovMatr_TopoToGeocentric(const CGeodesic *pGEO, Square_Matrix<9> *CM_NUE, Square_Matrix<9> *CM_Geo  );

#endif
