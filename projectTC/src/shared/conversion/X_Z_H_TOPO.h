#ifndef X_Z_H_TOPO_H
#define X_Z_H_TOPO_H


#include "Structures.h"
#include "Geocentric.h"
#include "Topocentric.h"
#include "Geodesic.h"
#include "Spherical_Topo.h"


// PACKAGE		: Conversion
// FUNCTION        : X_Z_H_TOPO
//
// DESCRIPTION     : Function of recalculation from the X_Z_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1 - geodesic coordinates of the origin ;
//                  pPosition_X_Z_H - coordinates, where m_dXt and m_dZt - to the
//                   topocentric system of coordinates. m_dYt - Altitude to Geodesic system of coordinates ;
//
//
// RETURNS	   : pPositionTopo - topocentric coordinates;
//
bool X_Z_H_TOPO(const CGeodesic *pCenter1, const CTopocentric *pPosition_X_Z_H,CTopocentric *pPositionTopo);


// PACKAGE		: Conversion
// FUNCTION     : X_Z_H_TOPO
//
// DESCRIPTION  : Function of recalculation from the X_Z_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : pCenter1 - geodesic coordinates of the origin ;
//                pVelocity_X_Z_H - velocity components, where m_dXt and m_dZt - velocity components in the
//                topocentric system of coordinates. m_dYt - Altitude rate in the Geodesic system of coordinates ;
//
//
// RETURNS	    : pVelocityTopo - topocentric velocity;
//
bool Vel_X_Z_H_TOPO(const CGeodesic *pCenter1, const CGeocentric *pPositionGEO, const CTopocentric *pVelocity_X_Z_H, CTopocentric *pVelocityTopo);


// PACKAGE		: Conversion
// FUNCTION        : X_Z_H_TOPO_It
//
// DESCRIPTION     : Function of iteratively recalculation from the X_Z_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1 - geodesic coordinates of the origin ;
//                  pPosition_X_Z_H - coordinates, where m_dXt and m_dZt - to the
//                   topocentric system of coordinates. m_dYt - Altitude to Geodesic system of coordinates ;
//
//
// RETURNS	   : pPositionTopo - topocentric coordinates;
bool X_Z_H_TOPO_It(const CGeodesic *pCenter1, const CTopocentric *pPosition_X_Z_H,CTopocentric *pPositionTopo);


// PACKAGE		: Conversion
// FUNCTION        : R_cos_R_sin_H_TOPO_It
//
// DESCRIPTION     : Function of iteratively recalculation from the X_Z_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1 - geodesic coordinates of the origin ;
//                  pPosition_X_Z_H - coordinates, where m_dXt = range*cos(azimuth), m_dZt = range*sin(azimuth).
//                      m_dYt - Altitude to Geodesic system of coordinates ;
//
//
// RETURNS	   : pPositionTopo - topocentric coordinates;
bool R_cos_R_sin_H_TOPO_It(const CGeodesic *pCenter1, const CTopocentric *pPosition_X_Z_H,CTopocentric *pPositionTopo);

// PACKAGE		: Conversion
// FUNCTION        : D_AZ_H_SFSC
//
// DESCRIPTION     : Function of iteratively recalculation from the D_AZ_H system of coordinates to the
//                   Spherical system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          :pCenter1 - geodesic coordinates of the origin ;
//                  D - slant range of the target;
//                  Az - azimuth;
//                  H - Altitude to Geodesic system of coordinates ;//                  
// RETURNS	   : pSF - Spherical coordinates;
bool D_AZ_H_SFSC(const CGeodesic *pCenter1,  double D, double Az, double H, CSpherical *pSF);

// PACKAGE		: Conversion
// FUNCTION        : D_AZ_H_TOPO_It
//
// DESCRIPTION     : Function of iteratively recalculation from the D_AZ_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1 -  geodesic coordinates of the origin ;
//                  D - slant range of the target;
//                  Az - azimuth;
//                  H - Altitude to Geodesic system of coordinates ;
//
// RETURNS	   : pPositionTopo - topocentric coordinates;
bool D_AZ_H_TOPO_It(const CGeodesic *pCenter1, double D, double Az, double H, CTopocentric *pPositionTopo, CSpherical *pPositionSF = 0);

// PACKAGE		   : Conversion
// FUNCTION        : X_Z_H_TOPO_coord_vel
//
// DESCRIPTION     : Function of recalculation from the X_Z_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1 - geodesic coordinates of the origin ;
//                   pPosition_X_Z_H - coordinates, where m_dXt and m_dZt corresponds to the
//                   topocentric system of coordinates, m_dYt - Altitude in Geodesic system of coordinates;
//                   pVel_X_Z_H - velocity components, where m_dXt and m_dZt corresponds to the
//                   topocentric system of coordinates, m_dYt - vertical velocity in Geodesic system of coordinates;
//
// RETURNS         : pPositionTopo - topocentric coordinates;
//                   pVelTopo - topocentric velocity components
bool X_Z_H_TOPO_coord_vel(const CGeodesic *pCenter1,
                          const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H,
                          CTopocentric *pPositionTopo, CTopocentric *pVelTopo);

// PACKAGE         : Conversion
// FUNCTION        : X_Z_H_TOPO_coord_vel_acc
//
// DESCRIPTION     : Function of recalculation from the X_Z_H system of coordinates to the
//                   topocentric system of coordinates;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pCenter1 - geodesic coordinates of the origin ;
//                   pPosition_X_Z_H - coordinates, where m_dXt and m_dZt corresponds to the
//                   topocentric system of coordinates, m_dYt - Altitude in Geodesic system of coordinates;
//                   pVel_X_Z_H - velocity components, where m_dXt and m_dZt corresponds to the
//                   topocentric system of coordinates, m_dYt - vertical velocity in Geodesic system of coordinates;
//                   pAcc_X_Z_H - acceleration components, where m_dXt and m_dZt corresponds to the
//                   topocentric system of coordinates, m_dYt - vertical acceleration in Geodesic system of coordinates;
//
// RETURNS         : pPositionTopo - topocentric coordinates;
//                   pVelTopo - topocentric velocity components
//                   pAccTopo - topocentric acceleration components
bool X_Z_H_TOPO_coord_vel_acc(const CGeodesic *pCenter1,
                              const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H, const CTopocentric *pAcc_X_Z_H,
                              CTopocentric *pPositionTopo, CTopocentric *pVelTopo, CTopocentric *pAccTopo);

// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_D_AZ_H_SFSC
//
// DESCRIPTION  : Function of covariance matrix recalculation from the D_AZ_H system of coordinates to the
//                Spherical system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : D - slant range of the target;
//                H - Altitude in Geodesic system of coordinates;
//                pCM_DAzH - covariance matrix in the D_AZ_H system, order: D, Az, H
//
// RETURNS	   : pCM_Sph - covariance matrix in the Spherical coordinates, order: D, Az, Epsilon
//             : true if result is OK
bool Recount_CovMatr_D_AZ_H_SFSC(const CGeodesic *pCenter1, double D, double H,
                                 Square_Matrix<3> *pCM_DAzH, Square_Matrix<3> *pCM_Sph);

// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_D_AZ_H_TOPO
//
// DESCRIPTION  : Function of covariance matrix recalculation from the D_AZ_H system of coordinates to the
//                Topocentric system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : D - slant range of the target;
//                H - Altitude in Geodesic system of coordinates;
//                pCM_DAzH - covariance matrix in the D_AZ_H system, order: D, Az, H
//
// RETURNS      : pCM_Topo - covariance matrix in the Topocentric coordinates;
//              : true if result is OK
bool Recount_CovMatr_D_AZ_H_TOPO(const CGeodesic *pCenter1, double D, double Az, double H,
                                 Square_Matrix<3> *pCM_DAzH, Square_Matrix<3> *pCM_Topo);


// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_X_H_Z_TOPO
//
// DESCRIPTION  : Function of covariance matrix recalculation from the X_Z_H system of coordinates to the
//                Topocentric system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : pCenter1 - geodesic coordinates of the origin ;
//                pPositionGEO - point geocentric coordinates
//                pCM_XHZ - covariance matrix in the X_H_Z system;
//
// RETURNS	    : pCM_Topo - covariance matrix in the Topocentric coordinates
//
bool Recount_CovMatr_X_H_Z_TOPO(const CGeodesic *pCenter1, const CGeocentric *pPositionGEO,
                                Square_Matrix<3> *pCM_XHZ, Square_Matrix<3> *pCM_Topo);


// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_X_H_Z_TOPO
//
// DESCRIPTION  : Function of covariance matrix recalculation from the X_Z_H system of coordinates to the
//                Topocentric system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : pCenter1 - geodesic coordinates of the origin ;
//                pPositionGEO - point geocentric coordinates
//                pCM_XHZ - covariance matrix in the X_H_Z system;
//
// RETURNS	    : pCM_Topo - covariance matrix in the Topocentric coordinates
//
bool Recount_CovMatr_Vel_X_H_Z_TOPO(const CGeodesic *pCenter1, const CGeocentric *pPositionGEO,
                                    Square_Matrix<3> *pCM_XHZ, Square_Matrix<3> *pCM_Topo);
// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_X_Z_H_TOPO_coord
//
// DESCRIPTION  : Function of 3x3 covariance matrix recalculation from the X_Z_H system of coordinates to the
//                Topocentric system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : pCenter1 - geodesic coordinates of the origin;
//                pPosition_X_Z_H - position in X_Z_H coordinate system, where m_dXt and m_dZt corresponds to the
//                topocentric system of coordinates, m_dYt - Altitude in Geodesic system of coordinates;
//                pCM_X_Z_H - 3x3 covariance matrix in the X_Z_H coordinate system, order: X, Z, H
//
// RETURNS      : pCM_Topo - 3x3 covariance matrix in the Topocentric coordinates;
//              : true if result is OK
bool Recount_CovMatr_X_Z_H_TOPO_coord(const CGeodesic *pCenter1,const CTopocentric *pPosition_X_Z_H,
                                      Square_Matrix<3> *pCM_X_Z_H, Square_Matrix<3> *pCM_Topo);

// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_X_Z_H_TOPO_coord_vel
//
// DESCRIPTION  : Function of 6x6 covariance matrix recalculation from the X_Z_H system of coordinates to the
//                Topocentric system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : pCenter1 - geodesic coordinates of the origin;
//                pPosition_X_Z_H - position in X_Z_H coordinate system, where m_dXt and m_dZt corresponds to the
//                topocentric system of coordinates, m_dYt - Altitude in Geodesic system of coordinates;
//                pVel_X_Z_H - velocity components, where m_dXt and m_dZt corresponds to the
//                topocentric system of coordinates, m_dYt - vertical velocity in Geodesic system of coordinates;
//                pCM_X_Z_H - 6x6 covariance matrix in the X_Z_H coordinate system, order: X, Z, H, VX, VZ, VH
//
// RETURNS      : pCM_Topo - 6x6 covariance matrix in the Topocentric coordinates;
//              : true if result is OK
bool Recount_CovMatr_X_Z_H_TOPO_coord_vel(const CGeodesic *pCenter1,
                                          const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H,
                                          Square_Matrix<6> *pCM_X_Z_H, Square_Matrix<6> *pCM_Topo);

// PACKAGE		: Conversion
// FUNCTION     : Recount_CovMatr_X_Z_H_TOPO_coord_vel_acc
//
// DESCRIPTION  : Function of 9x9 covariance matrix recalculation from the X_Z_H system of coordinates to the
//                Topocentric system of coordinates;
//                All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS       : pCenter1 - geodesic coordinates of the origin;
//                pPosition_X_Z_H - position in X_Z_H coordinate system, where m_dXt and m_dZt corresponds to the
//                topocentric system of coordinates, m_dYt - Altitude in Geodesic system of coordinates;
//                pVel_X_Z_H - velocity components, where m_dXt and m_dZt corresponds to the
//                topocentric system of coordinates, m_dYt - vertical velocity in Geodesic system of coordinates;
//                pAcc_X_Z_H - acceleration components, where m_dXt and m_dZt corresponds to the
//                topocentric system of coordinates, m_dYt - vertical velocity in Geodesic system of coordinates;
//                pCM_X_Z_H - 9x9 covariance matrix in the X_Z_H coordinate system, order: X, Z, H, VX, VZ, VH, AX, AZ, AH
//
// RETURNS      : pCM_Topo - 9x9 covariance matrix in the Topocentric coordinates;
//              : true if result is OK
bool Recount_CovMatr_X_Z_H_TOPO_coord_vel_acc(const CGeodesic *pCenter1,
                                              const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H, const CTopocentric *pAcc_X_Z_H,
                                              Square_Matrix<9> *pCM_X_Z_H, Square_Matrix<9> *pCM_Topo);
#endif // X_Z_H_TOPO_H
