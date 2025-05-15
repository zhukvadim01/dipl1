// PACKAGE		: Conversion
// FILE         : Geocentric_GEO.h   
//
// AUTHOR	: Marina Mukhortova, 2003
//DESCRIPTION :Header file for function of recalculation geocentric coordinates system and geodesic
//                   coordinates system
//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __Geocentric_Geo_H__
#define __Geocentric_Geo_H__

#include "Structures.h"
#include "Geocentric.h"
#include "Geodesic.h"
#include "Topocentric.h"


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
                      CGeodesic *pGEO);

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
                         CGeodesic *pGEO);


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
                       CGeocentric *pWGS);


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
                         CGeodesic *pGEO);



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
                          CGeocentric *pWGS);

bool  GEO_TOPO(const CGeodesic  *pGEO_in, const CGeodesic  *pGEO_base, CTopocentric& pointTOPO);


// PACKAGE		: Conversion
// FUNCTION	    : Recount_CovMatr_GeocentricToGEO
//
// DESCRIPTION	    : Function of covariance matrix recalculation from the geocentric system of coordinates
//                    to the geodesic system of coordinates (parameters of WGS-84)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pWGS - geocentric coordinates;
//              : pCovWGS - covariance matrix of the geocentric coordinates;
//
// RETURNS	    : pCovGEO - covariance matrix of the geodesic coordinates
//              : (order: Latitude, radians; Longitude, radians; Altitude, metres)
//
bool   Recount_CovMatr_GeocentricToGEO(const CGeocentric *pWGS,
                                       Square_Matrix<3> *pCovWGS,
                                       Square_Matrix<3> *pCovGEO);

// PACKAGE		: Conversion
// FUNCTION	    : Recount_CovMatr_GeocentricToGEO
//
// DESCRIPTION	    : Function of covariance matrix recalculation from the geocentric system of coordinates
//                    to the geodesic system of coordinates (parameters of WGS-84)
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geocentric coordinates;
//              : pCovGEO - covariance matrix of the geocentric coordinates;
//
// RETURNS	    : pCovGDS - covariance matrix of the geodesic coordinates
//              : (order: Latitude, radians; Longitude, radians; Altitude, metres)
bool Recount_CovMatr_GeocentricToGeodesic(const CGeocentric *pGEO,
                                          Square_Matrix<3> *pCovGEO,
                                          Square_Matrix<3> *pCovGDS);

// PACKAGE		: Conversion
// FUNCTION	    : Recount_CovMatr_GEOToGeocentric
//
// DESCRIPTION	    : Function of covariance matrix recalculation from the geodesic system of coordinates (parameters of WGS-84)
//                    to the geodesic system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS 	    : pGEO - geodesic coordinates;
//              : pCovGEO - covariance matrix of the geodesic coordinates;
//              : (order: Latitude, radians; Longitude, radians; Altitude, metres)
//
// RETURNS	    : pCovWGS - covariance matrix of the geocentric coordinates
//
bool Recount_CovMatr_GeodesicToGeocentric(const CGeodesic *pGEO,
                                          Square_Matrix<3> *pCovGEO,
                                          Square_Matrix<3> *pCovWGS);
#endif
