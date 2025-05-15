//PACKAGE		:   GL
//FILE			:   GLConvertMatrices.h
//AUTHOR		:
//DESCRIPTION	:   Conversion of matrices in the form TMatrix from one coordinate system to another

#ifndef CONVERT_MATRICES_H
#define CONVERT_MATRICES_H

#include "conversion/Structures.h"
#include "conversion/Geocentric_Topo.h"
#include "conversion/Spherical_Topo.h"
#include "conversion/Topo1_Topo2.h"

#include "GLMatrix.h"

namespace GL_MATR
{
    // FUNCTION       : MatrMatching_3x3
    // DESCRIPTION    : Matching TMatrix<3> (GL) to Square_Matrix<3> (Conversion)
    // INPUTS         : Matrix in the form given in GL
    // RETURNS        : Matrix in the form given in Conversion
    void MatrMatching_3x3(TMatrix<3> &M_inp, Square_Matrix<3> &M_out);

    // FUNCTION       : MatrMatching_3x3
    // DESCRIPTION    : Matching Square_Matrix<3> (Conversion) to TMatrix<3> (GL)
    // INPUTS         : Matrix in the form given in Conversion
    // RETURNS        : Matrix in the form given in GL
    void MatrMatching_3x3(Square_Matrix<3> &M_inp, TMatrix<3> &M_out);

    // FUNCTION       : MatrMatching_6x6
    // DESCRIPTION    : Matching TMatrix<6> (GL) to Square_Matrix<6> (Conversion)
    // INPUTS         : Matrix in the form given in GL
    // RETURNS        : Matrix in the form given in Conversion
    void MatrMatching_6x6(TMatrix<6> &M_inp, Square_Matrix<6> &M_out);

    // FUNCTION       : MatrMatching_6x6
    // DESCRIPTION    : Matching Square_Matrix<6> (Conversion) to TMatrix<6> (GL)
    // INPUTS         : Matrix in the form given in Conversion
    // RETURNS        : Matrix in the form given in GL
    void MatrMatching_6x6(Square_Matrix<6> &M_inp, TMatrix<6> &M_out);

    // FUNCTION       : MatrMatching_9x9
    // DESCRIPTION    : Matching TMatrix<9> (GL) to Square_Matrix<9> (Conversion)
    // INPUTS         : Matrix in the form given in GL
    // RETURNS        : Matrix in the form given in Conversion
    void MatrMatching_9x9(TMatrix<9> &M_inp, Square_Matrix<9> &M_out);

    // FUNCTION       : MatrMatching_9x9
    // DESCRIPTION    : Matching Square_Matrix<9> (Conversion) to TMatrix<9> (GL)
    // INPUTS         : Matrix in the form given in Conversion
    // RETURNS        : Matrix in the form given in GL
    void MatrMatching_9x9(Square_Matrix<9> &M_inp, TMatrix<9> &M_out);

    // FUNCTION       : MatrMatching_SIZE_Mto3
    // DESCRIPTION    : Matching GLMatrix (GL) to Square_Matrix<3> (Conversion)
    // INPUTS         : Matrix in the form given in GL
    // RETURNS        : Matrix in the form given in Conversion
    bool MatrMatching_SIZE_Mto3(GLMatrix &M_inp, Square_Matrix<3> &M_out);

    // FUNCTION       : MatrMatching_3toSIZE_M
    // DESCRIPTION    : Matching Square_Matrix<3> (Conversion) to GLMatrix (GL)
    // INPUTS         : Matrix in the form given in Conversion
    // RETURNS        : Matrix in the form given in GL
    bool MatrMatching_3toSIZE_M(Square_Matrix<3> &M_inp, GLMatrix &M_out);

    // FUNCTION       : MatrMatching_SIZE_Mto6
    // DESCRIPTION    : Matching GLMatrix (GL) to Square_Matrix<6> (Conversion)
    // INPUTS         : Matrix in the form given in GL
    // RETURNS        : Matrix in the form given in Conversion
    bool MatrMatching_SIZE_Mto6(GLMatrix &M_inp, Square_Matrix<6> &M_out);

    // FUNCTION       : MatrMatching_6toSIZE_M
    // DESCRIPTION    : Matching Square_Matrix<6> (Conversion) to GLMatrix (GL)
    // INPUTS         : Matrix in the form given in Conversion
    // RETURNS        : Matrix in the form given in GL
    bool MatrMatching_6toSIZE_M(Square_Matrix<6> &M_inp, GLMatrix &M_out);

    // FUNCTION       : MatrMatching_SIZE_Mto9
    // DESCRIPTION    : Matching GLMatrix (GL) to Square_Matrix<9> (Conversion)
    // INPUTS         : Matrix in the form given in GL
    // RETURNS        : Matrix in the form given in Conversion
    bool MatrMatching_SIZE_Mto9(GLMatrix &M_inp, Square_Matrix<9> &M_out);

    // FUNCTION       : MatrMatching_9toSIZE_M
    // DESCRIPTION    : Matching Square_Matrix<9> (Conversion) to GLMatrix (GL)
    // INPUTS         : Matrix in the form given in Conversion
    // RETURNS        : Matrix in the form given in GL
    bool MatrMatching_9toSIZE_M(Square_Matrix<9> &M_inp, GLMatrix &M_out);

    // FUNCTION       : Recount_CovMatr_GeocentricToTopo
    // DESCRIPTION    : Function of recalculation covariance matrix from the Geocentric system of coordinates to the
    //                   topocentric system of coordinates
    // INPUTS         : pGEO - Geodesic coordinates;
    //                  CM_Geo - the covariance matrix of errors in the determination of Geodesic
    //                  coordinates;
    // RETURNS        : CM_NUE - the covariance matrix of errors in the determination of topocentric
    //                  coordinates;
    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, TMatrix<3> *CM_Geo, TMatrix<3> * CM_NUE);

    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, TMatrix<6> *CM_Geo, TMatrix<6> * CM_NUE);

    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, TMatrix<9> *CM_Geo, TMatrix<9> * CM_NUE);

    bool Recount_CovMatr_GeocentricToTopo(CGeodesic *pGEO, GLMatrix *CM_Geo, GLMatrix *CM_NUE);

    // FUNCTION       : Recount_CovMatr_TopoToGeocentric
    // DESCRIPTION    : Function of recalculation covariance matrix from the topocentric system of coordinates to the
    //                   geocentric system of coordinates
    // INPUTS         : pGEO - Geodesic coordinates;
    //                  CM_NUE - the covariance matrix of errors in the determination of topocentric
    //                  coordinates;
    // RETURNS        : CM_Geo - the covariance matrix of errors in the determination of Geodesic
    //                  coordinates;


    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, TMatrix<3> *CM_NUE, TMatrix<3> *CM_Geo);

    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, TMatrix<6> *CM_NUE, TMatrix<6> *CM_Geo);

    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, TMatrix<9> *CM_NUE, TMatrix<9> *CM_Geo);

    bool Recount_CovMatr_TopoToGeocentric(CGeodesic *pGEO, GLMatrix *CM_NUE, GLMatrix *CM_Geo);

    // FUNCTION       : Recount_CovMatr_NUEtoSPHCS_coord
    // DESCRIPTION    : Function of recalculation covariance matrix from the topocentric system of coordinates to the
    //                   spherical  system of coordinates
    // INPUTS         : Coord_TOPO - topocentric coordinates;
    //                  CM_NUE - the covariance matrix of errors in the determination of topocentric
    //                  coordinates; order: x, y, z
    // RETURNS        : CM_SPH  - the covariance matrix of errors in the determination of spherical
    //                  coordinates; order: R, Beta, Epsilon

    bool Recount_CovMatr_NUEtoSPHCS_coord(CTopocentric *Coord_TOPO,TMatrix<3> *CM_NUE, TMatrix<3> *CM_SPH);

    bool Recount_CovMatr_NUEtoSPHCS_coord(CTopocentric *Coord_TOPO,GLMatrix *CM_NUE, GLMatrix *CM_SPH);

    // FUNCTION       : Recount_CovMatr_NUEtoSPHCS_coord_vel
    // DESCRIPTION    : Function of recalculation covariance matrix from the topocentric system of coordinates to the
    //                   spherical  system of coordinates
    // INPUTS         : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity;
    //                  CM_NUE - the covariance matrix of errors in the determination of topocentric
    //                  coordinates; order: x, y, z, vx, vy, vz
    // RETURNS        : CM_SPH  - the covariance matrix of errors in the determination of spherical
    //                  coordinates; order: R, Beta, Epsilon, VR, VBeta, VEpsilon

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,TMatrix<6> *CM_NUE, TMatrix<6> *CM_SPH);

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,GLMatrix *CM_NUE, GLMatrix *CM_SPH);

    // FUNCTION       : Recount_CovMatr_NUEtoSPHCS_coord_vel_accel
    // DESCRIPTION    : Function of recalculation covariance matrix from the topocentric system of coordinates to the
    //                   spherical  system of coordinates
    // INPUTS         : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity  and Acc_TOPO - topocentric acceleration;
    //                  CM_NUE - the covariance matrix of errors in the determination of topocentric
    //                  coordinates; order: x, y, z, vx, vy, vz, ax, ay, az
    // RETURNS        : CM_SPH  - the covariance matrix of errors in the determination of spherical
    //                  coordinates; order: R, Beta, Epsilon, VR, VBeta, VEpsilon, AR, ABeta, AEpsilon

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,CTopocentric *Acc_TOPO,
                                                                TMatrix<9> *CM_NUE, TMatrix<9> *CM_SPH);

    bool Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(CTopocentric *Coord_TOPO,CTopocentric *Vel_TOPO,CTopocentric *Acc_TOPO,
                                                                GLMatrix *CM_NUE, GLMatrix *CM_SPH);

    // FUNCTION       : Recount_CovMatr_SPHCStoNUEcoord
    // DESCRIPTION    : Function of recalculation covariance matrix from the spherical system of coordinates to the
    //                   topocentric  system of coordinates
    // INPUTS         : Coord_Sf - spherical coordinates;
    //                  CM_SPH - the covariance matrix of errors in the determination of spherical
    //                  coordinates; order: R, Beta, Epsilon
    // RETURNS        : CM_NUE  - the covariance matrix of errors in the determination of topocentric
    //                  coordinates; order: x, y, z

    bool Recount_CovMatr_SPHCStoNUEcoord(CSpherical *Coord_Sf, TMatrix<3> *CM_SPH, TMatrix<3> *CM_NUE);

    bool Recount_CovMatr_SPHCStoNUEcoord(CSpherical *Coord_Sf, GLMatrix *CM_SPH, GLMatrix *CM_NUE);

    // FUNCTION       : Recount_CovMatr_SPHCStoNUEcoord_vel
    // DESCRIPTION    : Function of recalculation covariance matrix from the spherical system of coordinates to the
    //                   topocentric  system of coordinates
    // INPUTS         : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity;
    //                  CM_SPH - the covariance matrix of errors in the determination of spherical
    //                  coordinates; order: R, Beta, Epsilon, VR, VBeta, VEpsilon
    // RETURNS        : CM_NUE  - the covariance matrix of errors in the determination of topocentric
    //                  coordinates; order: x, y, z, vx, vy, vz

    bool Recount_CovMatr_SPHCStoNUEcoord_vel(CSpherical *Coord_Sf,CSpherical *Vel_Sf, TMatrix<6> *CM_SPH, TMatrix<6> *CM_NUE);

    bool Recount_CovMatr_SPHCStoNUEcoord_vel(CSpherical *Coord_Sf,CSpherical *Vel_Sf, GLMatrix *CM_SPH, GLMatrix *CM_NUE);

    // FUNCTION       : Recount_CovMatr_SPHCStoNUEcoord_vel_acc
    // DESCRIPTION    : Function of recalculation covariance matrix from the spherical system of coordinates to the
    //                   topocentric  system of coordinates
    // INPUTS         : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity and Acc_Sf - spherical acceleration;
    //                  CM_SPH - the covariance matrix of errors in the determination of spherical
    //                  coordinates; order: R, Beta, Epsilon, VR, VBeta, VEpsilon, AR, ABeta, AEpsilon
    // RETURNS        : CM_NUE  - the covariance matrix of errors in the determination of topocentric
    //                  coordinates; order: x, y, z, vx, vy, vz, ax, ay, az

    bool Recount_CovMatr_SPHCStoNUEcoord_vel_acc(CSpherical *Coord_Sf,CSpherical *Vel_Sf,CSpherical *Acc_Sf,
                                                             TMatrix<9> *CM_SPH, TMatrix<9> *CM_NUE);

    bool Recount_CovMatr_SPHCStoNUEcoord_vel_acc(CSpherical *Coord_Sf,CSpherical *Vel_Sf,CSpherical *Acc_Sf,
                                                            GLMatrix *CM_SPH,GLMatrix *CM_NUE);

    // FUNCTION       : Recount_CovMatr_TopoToTopo
    // DESCRIPTION    : Function of recalculation covariance matrix from the topocentric system of coordinates to the
    //                   topocentric system of coordinates
    // INPUTS         : pGEO_1 - Geodesic coordinates of the center topo_1;
    //                  pGEO_2 - Geodesic coordinates of the center topo_1
    //                  CM_NUE_1 - the covariance matrix of errors in the determination of topocentric
    //                  coordinates;
    // RETURNS        : CM_NUE_1 - the covariance matrix of errors in the determination of topocentric
    //                  coordinates;


    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, TMatrix<3> *CM_NUE_1, TMatrix<3> *CM_NUE_2);

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, TMatrix<6> *CM_NUE_1, TMatrix<6> *CM_NUE_2);

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, TMatrix<9> *CM_NUE_1, TMatrix<9> *CM_NUE_2);

    bool Recount_CovMatr_TopoToTopo(CGeodesic *pGEO_1,CGeodesic *pGEO_2, GLMatrix *CM_NUE_1, GLMatrix *CM_NUE_2);



    bool Recount_TVec_GeocentricToTopo(CGeodesic *pGEO, TVector<SIZE_M> *CM_Geo, TVector<SIZE_M> *CM_Topo);

    bool Recount_TVec_TopocentricToGeo(CGeodesic *pGEO, TVector<SIZE_M> *CM_Topo, TVector<SIZE_M> *CM_Geo);
}

#endif // CONVERT_MATRICES_H
