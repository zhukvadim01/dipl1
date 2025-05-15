// PACKAGE		: Conversion
// FILE         :   Spherical_Topo.h   
// 
// AUTHOR	: Marina Mukhortova, 2003
//DESCRIPTION :Header file for function of recalculation spherical coordinates system and topocentric
//                   coordinates system

//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __Spherical_Topo_H__
#define __Spherical_Topo_H__


#include "Topocentric.h"
#include "Spherical.h"
#include "Structures.h"

// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO
//
// DESCRIPTION     : Function of recalculation from the spherical system of coordinates to the
//                   topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pSF - spherical coordinates;
//
// RETURNS	   : pTOPO - topocentric coordinates;
//
bool   SPHERICAL_TOPO(const CSpherical *pSF,
                      CTopocentric *pTOPO);


// PACKAGE		: Conversion
// FUNCTION        : TOPO_SPHERICAL
//
// DESCRIPTION     : Function of recalculation from the topocentric system of coordinates to the
//                   spherical system of coordinates 
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pTOPO - topocentric coordinates;
//
// RETURNS	   : pSF - spherical coordinates;
//
bool   TOPO_SPHERICAL(const CTopocentric *pTOPO,
                      CSpherical *pSF);

// PACKAGE		: Conversion
// FUNCTION        : TOPO_SPHERICAL_coord_vel
//
// DESCRIPTION     : Function of recalculation from the topocentric system of coordinates to the
//                   spherical system of coordinates and velocity
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity;
//
// RETURNS	   : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity;
//
bool TOPO_SPHERICAL_coord_vel(const CTopocentric *Coord_TOPO,const  CTopocentric *Vel_TOPO,CSpherical *Coord_Sf,CSpherical *Vel_Sf);

// PACKAGE		: Conversion
// FUNCTION        : TOPO_SPHERICAL_coord_vel_accel
//
// DESCRIPTION     : Function of recalculation from the topocentric system of coordinates to the
//                   spherical system of coordinates, velocity and acceleration
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity  and Acc_TOPO - topocentric acceleration;
//
// RETURNS	   : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity and Acc_Sf - spherical acceleration;
//
bool TOPO_SPHERICAL_coord_vel_accel(const CTopocentric *Coord_TOPO, const CTopocentric *Vel_TOPO,const CTopocentric *Acc_TOPO,
                                    CSpherical *Coord_Sf,CSpherical *Vel_Sf,CSpherical *Acc_Sf);


// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO_vel
//
// DESCRIPTION     : Function of recalculation from the spherical system of coordinates to the
//                   topocentric system of coordinates and velocity
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity;
//
// RETURNS	   : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity;
//
bool SPHERICAL_TOPO_vel( const CSpherical *Coord_Sf,const CSpherical *Vel_Sf,CTopocentric *Coord_TOPO, CTopocentric *Vel_TOPO);

// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO_vel_accel
//
// DESCRIPTION     : Function of recalculation from the spherical system of coordinates to the
//                   topocentric system of coordinates, velocity and acceleration;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity and Acc_Sf - spherical acceleration;
//
// RETURNS	       : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity  and Acc_TOPO - topocentric acceleration;
//
bool SPHERICAL_TOPO_vel_accel( const CSpherical *Coord_Sf,const CSpherical *Vel_Sf,const CSpherical *Acc_Sf,
                               CTopocentric *Coord_TOPO, CTopocentric *Vel_TOPO, CTopocentric *Acc_TOPO);



// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   spherical system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates; CM_NUE - the covariance matrix of the measurement errors topocentric coordinates,
//                 : order: x, y, z
//
// RETURNS	       : CM_SPH - the covariance matrix of the measurement errors of spherical coordinates,
//                 : order: R, Beta, Epsilon
//
bool Recount_CovMatr_NUEtoSPHCS_coord(const CTopocentric *Coord_TOPO,Square_Matrix<3> *CM_NUE, Square_Matrix<3> *CM_SPH);


// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   spherical system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity; CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates and velocity components, order: x, y, z, vx, vy, vz
//
// RETURNS	       : CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates and velocity components, order: R, Beta, Epsilon, VR, VBeta, VEpsilon


bool Recount_CovMatr_NUEtoSPHCS_coord_vel(const CTopocentric *Coord_TOPO,const CTopocentric *Vel_TOPO,Square_Matrix<6> *CM_NUE, Square_Matrix<6> *CM_SPH);

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel_accel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   spherical system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity  and Acc_TOPO - topocentric acceleration;
//                  CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity and acceleration components, order: x, y, z, vx, vy, vz, ax, ay, az
//
// RETURNS	       : CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates, velocity and acceleration components, order: R, Beta, Epsilon, VR, VBeta, VEpsilon, AR, ABeta, AEpsilon
//
bool Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(const CTopocentric *Coord_TOPO,const CTopocentric *Vel_TOPO,const CTopocentric *Acc_TOPO,
                                                Square_Matrix<9> *CM_NUE, Square_Matrix<9> *CM_SPH);

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_SPHCStoNUEcoord
//
// DESCRIPTION     : Function of recalculation covariance matrix from the spherical system of coordinates to the
//                   topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates; CM_SPH - the spherical matrix of the measurement errors spherical coordinates,
//                 : order: R, Beta, Epsilon
//
// RETURNS	       : CM_NUE - the covariance matrix of the measurement errors of topocentric coordinates,
//                 : order: x, y, z
//
bool Recount_CovMatr_SPHCStoNUEcoord(const CSpherical *Coord_Sf, Square_Matrix<3> *CM_SPH, Square_Matrix<3> *CM_NUE);


// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the spherical system of coordinates to the
//                  topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity; CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates and velocity components, order: R, Beta, Epsilon, VR, VBeta, VEpsilon
//
// RETURNS         : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates and velocity components, order: x, y, z, vx, vy, vz
//
bool Recount_CovMatr_SPHCStoNUEcoord_vel(const CSpherical *Coord_Sf,const CSpherical *Vel_Sf, Square_Matrix<6> *CM_SPH, Square_Matrix<6> *CM_NUE);

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel_accel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the spherical system of coordinates to the
//                  topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity and Acc_Sf - spherical acceleration;
//                  CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates, velocity and acceleration components, order: R, Beta, Epsilon, VR, VBeta, VEpsilon, AR, ABeta, AEpsilon
//
// RETURNS          : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity and acceleration components, order: x, y, z, vx, vy, vz, ax, ay, az
//
bool Recount_CovMatr_SPHCStoNUEcoord_vel_acc(const CSpherical *Coord_Sf,const CSpherical *Vel_Sf,const CSpherical *Acc_Sf,
                                             Square_Matrix<9> *CM_SPH, Square_Matrix<9> *CM_NUE);

// PACKAGE          : Conversion
// FUNCTION         : TOPO_GET_AZIMUTH
// DESCRIPTION      : Function of azimuth calculation for the point in the topocentric system of coordinates
//                    All the parameters are in radians (angles) and metres (distanses).
// INPUTS           : pTOPO - topocentric coordinates; pointer to the resulting value of azimuth
// RETURNS          : True if result is OK
bool TOPO__GET_AZIMUTH(const CTopocentric *pTOPO, double *pAzimuth);


#endif
