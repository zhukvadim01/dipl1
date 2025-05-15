// PACKAGE		: Conversion
// FILE         : SPHERICALtoTOPO.cpp
//
// AUTHOR	: Marina Mukhortova, 2003
//
//DESCRIPTION :Implementation file for function of recalculation spherical coordinates system and topocentric
//                   coordinates system

#include <cmath>
#include <cstdio>
#include <cerrno>
#include <stdlib.h>

#include "Spherical_Topo.h"
#include "ConstRecalc.h"


// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO
//
// DESCRIPTION     : Function of recalculation from the spherical system of coordinates to the topocentric 
//                   system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : pSF - spherical coordinates;
//
// RETURNS	   : pTOPO - topocentric coordinates;
//
bool   SPHERICAL_TOPO(const CSpherical *pSF, CTopocentric *pTOPO)
{
    if((!pSF) ||
            (!pTOPO) ||
            (pSF->m_dR < cdSPHERIC_R_MIN) ||
            (pSF->m_dR > cdSPHERIC_R_MAX) ||
            (pSF->m_dE < cdSPHERIC_E_MIN) ||
            (pSF->m_dE > cdSPHERIC_E_MAX) ||
            (pSF->m_dB < cdSPHERIC_B_MIN) ||
            (pSF->m_dB > cdSPHERIC_B_MAX))
    {
        return false;
    }

    pTOPO->m_dXt = (pSF->m_dR)*cos(pSF->m_dE)*cos(pSF->m_dB);
    pTOPO->m_dYt = (pSF->m_dR)*sin(pSF->m_dE);
    pTOPO->m_dZt = (pSF->m_dR)*cos(pSF->m_dE)*sin(pSF->m_dB);

    return true;

}



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
bool   TOPO_SPHERICAL(const CTopocentric *pTOPO, CSpherical *pSF)
{
    if((!pTOPO) ||
            (!pSF) ||
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
    double dD;  // The auxiliary variable
    
    dD = sqrt(pow((pTOPO->m_dXt),2.0)+pow((pTOPO->m_dZt),2.0));  // The auxiliary variable


    // Spherical coordinate R - slant range of the target
    pSF->m_dR = sqrt(pow((pTOPO->m_dXt),2.0)+pow((pTOPO->m_dYt),2.0)+pow((pTOPO->m_dZt),2.0));


    // Spherical coordinate B - azimuth
    if ((pTOPO->m_dXt) != 0)
    {
        pSF->m_dB = atan((pTOPO->m_dZt)/(pTOPO->m_dXt));
    }
    else
    {
        pSF->m_dB = 0;
    }


    pSF->m_dB = fabs(pSF->m_dB);


    /*if (((pTOPO->m_dZt) > 0) && ((pTOPO->m_dXt) > 0))
    {
        pSF->m_dB = pSF->m_dB; error 264
    }*/

    if (((pTOPO->m_dZt) > 0) && ((pTOPO->m_dXt) < 0))
    {
        pSF->m_dB = cdPi-(pSF->m_dB);
    }

    if (((pTOPO->m_dZt) < 0) && ((pTOPO->m_dXt) > 0))
    {
        pSF->m_dB = 2*cdPi-(pSF->m_dB);
    }

    if (((pTOPO->m_dZt) < 0) && ((pTOPO->m_dXt) < 0))
    {
        pSF->m_dB = cdPi+(pSF->m_dB);
    }

    if (((pTOPO->m_dZt) > 0) && ((pTOPO->m_dXt) == 0))
    {
        pSF->m_dB = cdPi/2;
    }

    if (((pTOPO->m_dZt) == 0) && ((pTOPO->m_dXt) == 0))
    {
        pSF->m_dB = 0;
    }

    if (((pTOPO->m_dZt) < 0) && ((pTOPO->m_dXt) == 0))
    {
        pSF->m_dB = 3*cdPi/2;
    }

    if (((pTOPO->m_dZt) == 0) && ((pTOPO->m_dXt) > 0))
    {
        pSF->m_dB = 0;
    }

    if (((pTOPO->m_dZt) == 0) && ((pTOPO->m_dXt) < 0))
    {
        pSF->m_dB = cdPi;
    }


    // Spherical coordinate E - elevation angle
    if (dD != 0)
    {
        pSF->m_dE = atan((pTOPO->m_dYt)/dD);
    }
    else
    {
        if ((pTOPO->m_dYt) != 0)
        {
            pSF->m_dE = cdPi/2;
        }
        else
        {
            pSF->m_dE = 0;
        }
    }


    return true;


}

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

bool TOPO_SPHERICAL_coord_vel(const CTopocentric *Coord_TOPO, const CTopocentric *Vel_TOPO,
                              CSpherical *Coord_Sf, CSpherical *Vel_Sf)
{


    if((!Coord_TOPO) ||
            (!Coord_Sf) ||
            (Coord_TOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }
    double x=Coord_TOPO->m_dXt;
    double y=Coord_TOPO->m_dYt;
    double z=Coord_TOPO->m_dZt;
    double vX=Vel_TOPO->m_dXt;
    double vY=Vel_TOPO->m_dYt;
    double vZ=Vel_TOPO->m_dZt;

    double r=sqrt(x*x+y*y+z*z);
    double epsilon;
    double betta;
    double vR;
    double vEpsilon;
    double vBetta;

    if (z > 0.0)
    {
        betta = acos(x / sqrt(x * x + z * z));
    }
    else
        if (z < 0.0)
        {
            betta = 2*cdPi - acos(x / sqrt(x * x + z * z));
        }
        else
            if (x > 0.0)
            {
                betta = 0.0;
            }
            else
                if (x < 0.0)
                {
                    betta = cdPi;
                }
                else
                {
                    betta = 0.0;
                }


    if (sqrt(x * x + z * z) != 0.0)
    {
        epsilon = atan(y / sqrt(x * x + z * z));
    }
    else
        if (y > 0.0)
        {
            epsilon = cdPi/2;
        }
        else
        {
            epsilon = -cdPi/2;
        }

    if (r != 0.0)
    {
        vR = ((x * vX + y * vY) + z * vZ) / r;
    }
    else
    {
        vR = 0.0;
    }

    if (x * x + z * z != 0.0)
    {
        vBetta = (x * vZ - z * vX) / (x * x + z * z);
    }
    else
    {
        vBetta = 0.0;
    }

    if ((r != 0.0) && (x * x + z * z != 0.0))
    {
        vEpsilon = (r*vY-y*vR)/(r*sqrt(x*x+z*z));
    }
    else
    {
        vEpsilon = 0.0;
    }

    Coord_Sf->m_dR=r;
    Coord_Sf->m_dB=betta;
    Coord_Sf->m_dE=epsilon;
    Vel_Sf->m_dR=vR;
    Vel_Sf->m_dB=vBetta;
    Vel_Sf->m_dE=vEpsilon;

    return true;

}


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

bool TOPO_SPHERICAL_coord_vel_accel(const CTopocentric *Coord_TOPO, const CTopocentric *Vel_TOPO, const CTopocentric *Acc_TOPO,
                                    CSpherical *Coord_Sf, CSpherical *Vel_Sf, CSpherical *Acc_Sf)
{
    if((!Coord_TOPO) ||
            (!Coord_Sf) ||
            (Coord_TOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }

    double x=Coord_TOPO->m_dXt;
    double y=Coord_TOPO->m_dYt;
    double z=Coord_TOPO->m_dZt;
    double vX=Vel_TOPO->m_dXt;
    double vY=Vel_TOPO->m_dYt;
    double vZ=Vel_TOPO->m_dZt;
    double aX=Acc_TOPO->m_dXt;
    double aY=Acc_TOPO->m_dYt;
    double aZ=Acc_TOPO->m_dZt;

    double r=sqrt(x*x+y*y+z*z);
    double epsilon;
    double betta;
    double vR;
    double vEpsilon;
    double vBetta;
    double aR;
    double aEpsilon;
    double aBetta;

    if (z > 0.0)
    {
        betta = acos(x / sqrt(x * x + z * z));
    }
    else
        if (z < 0.0)
        {
            betta = 2*cdPi - acos(x / sqrt(x * x + z * z));
        }
        else
            if (x > 0.0)
            {
                betta = 0.0;
            }
            else
                if (x < 0.0)
                {
                    betta = cdPi;
                }
                else
                {
                    betta = 0.0;
                }

    if (sqrt(x * x + z * z) != 0.0)
    {
        epsilon = atan(y / sqrt(x * x + z * z));
    }
    else
        if (y > 0.0)
        {
            epsilon = cdPi/2;
        }
        else
        {
            epsilon = -cdPi/2;
        }

    if (r != 0.0)
    {
        vR = ((x * vX + y * vY) + z * vZ) / r;
    }
    else
    {
        vR = 0.0;
    }

    if (x * x + z * z != 0.0)
    {
        vBetta = (x * vZ - z * vX) / (x * x + z * z);
    }
    else
    {
        vBetta = 0.0;
    }


    if ((r != 0.0) && (x * x + z * z != 0.0))
    {
        vEpsilon = (r*vY-y*vR)/(r*sqrt(x*x+z*z));
    }
    else
    {
        vEpsilon = 0.0;
    }

    if (r != 0.0)
    {
        aR = ((((((x * aX + y * aY) + z * aZ) + vX * vX) + vY * vY) + vZ * vZ) - vR *
              vR) / r;
    }
    else
    {
        aR = 0.0;
    }

    if (x * x + z * z != 0.0)
    {
        aBetta = ((x * aZ - z * aX) - 2.0 * vBetta * (x * vX + z * vZ)) / (x * x + z
                                                                           * z);
    }
    else
    {
        aBetta = 0.0;
    }

    if (r * r * sqrt(x * x + z * z) != 0.0)
    {
        aEpsilon = (r * aY - y * aR) / (r * sqrt(x * x + z * z)) - vEpsilon*((x*vX+z*vZ)/(x*x+z*z)+vR/r);
    }
    else
    {
        aEpsilon = 0.0;
    }


    Coord_Sf->m_dR=r;
    Coord_Sf->m_dB=betta;
    Coord_Sf->m_dE=epsilon;
    Vel_Sf->m_dR=vR;
    Vel_Sf->m_dB=vBetta;
    Vel_Sf->m_dE=vEpsilon;
    Acc_Sf->m_dR=aR;
    Acc_Sf->m_dB=aBetta;
    Acc_Sf->m_dE=aEpsilon;

    return true;

}


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

bool SPHERICAL_TOPO_vel( const CSpherical *Coord_Sf,const CSpherical *Vel_Sf,
                         CTopocentric *Coord_TOPO, CTopocentric *Vel_TOPO)
{
    if((!Coord_Sf) ||
            (!Coord_TOPO) ||
            (Coord_Sf->m_dR < cdSPHERIC_R_MIN) ||
            (Coord_Sf->m_dR > cdSPHERIC_R_MAX) ||
            (Coord_Sf->m_dE < cdSPHERIC_E_MIN) ||
            (Coord_Sf->m_dE > cdSPHERIC_E_MAX) ||
            (Coord_Sf->m_dB < cdSPHERIC_B_MIN) ||
            (Coord_Sf->m_dB > cdSPHERIC_B_MAX))
    {
        return false;
    }

    double r=Coord_Sf->m_dR;
    double betta=Coord_Sf->m_dB;
    double epsilon=Coord_Sf->m_dE;
    double vR=Vel_Sf->m_dR;
    double vBetta=Vel_Sf->m_dB;
    double vEpsilon=Vel_Sf->m_dE;
    double x= r*cos(betta) * cos(epsilon);;
    double y=r*sin(epsilon);
    double z=r * sin(betta) * cos(epsilon);;
    double vX=vR * cos(betta) * cos(epsilon) - r*(sin(betta)*cos(epsilon)*vBetta+cos(betta)*sin(epsilon)*vEpsilon);
    double vY=vR * sin(epsilon) + r * cos(epsilon) * vEpsilon;
    double vZ=vR * sin(betta) * cos(epsilon) + r * (cos(betta) * cos(epsilon) *vBetta - sin(betta) * sin(epsilon) * vEpsilon);

    Coord_TOPO->m_dXt=x;
    Coord_TOPO->m_dYt=y;
    Coord_TOPO->m_dZt=z;
    Vel_TOPO->m_dXt=vX;
    Vel_TOPO->m_dYt=vY;
    Vel_TOPO->m_dZt=vZ;

    return true;
}

// PACKAGE		: Conversion
// FUNCTION        : SPHERICAL_TOPO_vel_accel
//
// DESCRIPTION     : Function of recalculation from the spherical system of coordinates to the
//                   topocentric system of coordinates, velocity and acceleration;
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity and Acc_Sf - spherical acceleration;
//
// RETURNS	   : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity  and Acc_TOPO - topocentric acceleration;
//

bool SPHERICAL_TOPO_vel_accel(const CSpherical *Coord_Sf, const CSpherical *Vel_Sf, const CSpherical *Acc_Sf,
                               CTopocentric *Coord_TOPO, CTopocentric *Vel_TOPO, CTopocentric *Acc_TOPO)
{
    if((!Coord_Sf) ||
            (!Coord_TOPO) ||
            (Coord_Sf->m_dR < cdSPHERIC_R_MIN) ||
            (Coord_Sf->m_dR > cdSPHERIC_R_MAX) ||
            (Coord_Sf->m_dE < cdSPHERIC_E_MIN) ||
            (Coord_Sf->m_dE > cdSPHERIC_E_MAX) ||
            (Coord_Sf->m_dB < cdSPHERIC_B_MIN) ||
            (Coord_Sf->m_dB > cdSPHERIC_B_MAX))
    {
        return false;
    }

    double r=Coord_Sf->m_dR;
    double betta=Coord_Sf->m_dB;
    double epsilon=Coord_Sf->m_dE;
    double vR=Vel_Sf->m_dR;
    double vBetta=Vel_Sf->m_dB;
    double vEpsilon=Vel_Sf->m_dE;
    double aR=Acc_Sf->m_dR;
    double aBetta=Acc_Sf->m_dB;
    double aEpsilon=Acc_Sf->m_dE;
    Coord_TOPO->m_dXt= r*cos(betta) * cos(epsilon);
    Coord_TOPO->m_dYt=r*sin(epsilon);
    Coord_TOPO->m_dZt=r * sin(betta) * cos(epsilon);
    //
    Vel_TOPO->m_dXt=vR * cos(betta) * cos(epsilon) - r*(sin(betta)*cos(epsilon)*vBetta+cos(betta)*sin(epsilon)*vEpsilon);
    Vel_TOPO->m_dYt=vR * sin(epsilon) + r * cos(epsilon) * vEpsilon;
    Vel_TOPO->m_dZt=vR * sin(betta) * cos(epsilon) + r * (cos(betta) * cos(epsilon) *vBetta - sin(betta) * sin(epsilon) * vEpsilon);
    Acc_TOPO->m_dXt=((cos(betta) * cos(epsilon) * (aR - r*(vBetta*vBetta+vEpsilon*vEpsilon)) + sin(betta) * sin(epsilon) *
                      (2.0 * r * vBetta * vEpsilon)) + cos(betta) * sin(epsilon) * (-r *
                                                                                    aEpsilon - 2.0 * vR * vEpsilon)) + sin(betta) * cos(epsilon) * (-r *
                                                                                                                                                    aBetta - 2.0 * vR * vBetta);
    Acc_TOPO->m_dYt=cos(epsilon) * (2.0 * vR * vEpsilon + r * aEpsilon) + sin(epsilon) *
            (aR - r * (vEpsilon * vEpsilon));
    Acc_TOPO->m_dZt=((cos(betta) * cos(epsilon) * (r * aBetta + 2.0 * vR * vBetta) + sin
                      (betta) * sin(epsilon) * (-r * aEpsilon - 2.0 * vR * vEpsilon)) +
                     cos(betta) * sin(epsilon) * (-2.0 * r * vBetta * vEpsilon)) + sin
            (betta) * cos(epsilon) * (aR - r * (vBetta * vBetta + vEpsilon * vEpsilon));

    return true;

}
// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   spherical system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates; CM_NUE - the covariance matrix of the measurement errors topocentric coordinates;
//
// RETURNS	   : CM_SPH - the covariance matrix of the measurement errors of spherical coordinates;
//
bool Recount_CovMatr_NUEtoSPHCS_coord(const CTopocentric *Coord_TOPO,Square_Matrix<3> *CM_NUE, Square_Matrix<3> *CM_SPH)
{
    if((!Coord_TOPO) ||
            (!CM_NUE) ||
            (Coord_TOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }

    CM_SPH->Reset(3,3);

    CSpherical *pSF=new CSpherical();

    double r_XZ=sqrt(pow(Coord_TOPO->m_dXt,2)+pow(Coord_TOPO->m_dZt,2));

    bool bResult = true;

    TOPO_SPHERICAL( Coord_TOPO, pSF);

    if (pSF->m_dR > 0 && r_XZ > 0)
    {
        Square_Matrix<3> J_SPH=Square_Matrix<3>(3,3);
        J_SPH.M[0][0]=Coord_TOPO->m_dXt /pSF->m_dR;
        J_SPH.M[0][1]=Coord_TOPO->m_dYt /pSF->m_dR;
        J_SPH.M[0][2]=Coord_TOPO->m_dZt /pSF->m_dR;

        J_SPH.M[1][0]=-Coord_TOPO->m_dZt /pow(r_XZ,2);
        J_SPH.M[1][1]=0;
        J_SPH.M[1][2]=Coord_TOPO->m_dXt /pow(r_XZ,2);

        J_SPH.M[2][0]=(-Coord_TOPO->m_dXt*Coord_TOPO->m_dYt) /(r_XZ*pow(pSF->m_dR,2));
        J_SPH.M[2][1]=r_XZ /(pow(pSF->m_dR,2));
        J_SPH.M[2][2]=(-Coord_TOPO->m_dYt*Coord_TOPO->m_dZt) /(r_XZ*pow(pSF->m_dR,2));

        Square_Matrix<3> J_SPH_tr=Square_Matrix<3>(3,3);
        J_SPH.transp(&J_SPH_tr);
        J_SPH.MatrXMatrXMatr(CM_NUE,&J_SPH_tr,CM_SPH);
    }
    else
    {
        bResult = false;
    }

    delete pSF;
    return bResult;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   spherical system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity; CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates and velocity components;
//
// RETURNS	   : CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates and velocity components;


bool Recount_CovMatr_NUEtoSPHCS_coord_vel(const CTopocentric *Coord_TOPO,const CTopocentric *Vel_TOPO,
                                          Square_Matrix<6> *CM_NUE, Square_Matrix<6> *CM_SPH)
{

    if((!Coord_TOPO) ||
            (!CM_NUE) ||
            (Coord_TOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }

    CM_SPH->Reset(6,6);

    bool bResult = true;
    CSpherical *Coord_Sf=new CSpherical();
    CSpherical *Vel_Sf=new CSpherical();
    double x=Coord_TOPO->m_dXt;
    double y=Coord_TOPO->m_dYt;
    double z=Coord_TOPO->m_dZt;
    double vX=Vel_TOPO->m_dXt;
    double vY=Vel_TOPO->m_dYt;
    double vZ=Vel_TOPO->m_dZt;
    double r_XZ=sqrt(pow(x,2)+pow(z,2));
    TOPO_SPHERICAL_coord_vel(Coord_TOPO,Vel_TOPO,Coord_Sf,Vel_Sf);
    double r=Coord_Sf->m_dR;
    //   double betta=Coord_Sf->m_dB;
    //   double epsilon=Coord_Sf->m_dE;
    double vR=Vel_Sf->m_dR;
    double vBetta=Vel_Sf->m_dB;
    double vEpsilon=Vel_Sf->m_dE;

    if (r > 0 && r_XZ > 0)
    {
        Square_Matrix<6> J_SPH=Square_Matrix<6>(6,6);
        J_SPH.s_m=6;
        J_SPH.s_n=6;
        J_SPH.M[0][0]=x /r;
        J_SPH.M[0][1]=y /r;
        J_SPH.M[0][2]=z /r;
        J_SPH.M[0][3]=0;
        J_SPH.M[0][4]=0;
        J_SPH.M[0][5]=0;

        J_SPH.M[1][0]=-z/pow(r_XZ,2);
        J_SPH.M[1][1]=0;
        J_SPH.M[1][2]=x /pow(r_XZ,2);
        J_SPH.M[1][3]=0;
        J_SPH.M[1][4]=0;
        J_SPH.M[1][5]=0;

        J_SPH.M[2][0]=(-x*y) /(r_XZ*pow(r,2));
        J_SPH.M[2][1]=r_XZ /(pow(r,2));
        J_SPH.M[2][2]=(-y*z) /(r_XZ*pow(r,2));
        J_SPH.M[2][3]=0;
        J_SPH.M[2][4]=0;
        J_SPH.M[2][5]=0;


        J_SPH.M[3][0]=vX/r-vR*x/pow(r,2);
        J_SPH.M[3][1]=vY/r-vR*y/pow(r,2);
        J_SPH.M[3][2]=vZ/r-vR*z/pow(r,2);
        J_SPH.M[3][3]=x/r;
        J_SPH.M[3][4]=y/r;
        J_SPH.M[3][5]=z/r;

        J_SPH.M[4][0]=(vZ-2*vBetta*x)/pow(r_XZ,2);
        J_SPH.M[4][1]=0;
        J_SPH.M[4][2]=(-vX-2*vBetta*z)/pow(r_XZ,2);
        J_SPH.M[4][3]=-z/pow(r_XZ,2);
        J_SPH.M[4][4]=0;
        J_SPH.M[4][5]=x/pow(r_XZ,2);

        J_SPH.M[5][0]=-y*(vX-2*x*vR/r)/(pow(r,2)*r_XZ)-vEpsilon*x/pow(r_XZ,2);
        J_SPH.M[5][1]=-(x*vX+z*vZ)/(pow(r,2)*r_XZ)-2*vEpsilon*y/pow(r,2);
        J_SPH.M[5][2]=-y/(pow(r,2)*r_XZ)*(vZ-2*z*vR/r)-vEpsilon*z/pow(r_XZ,2);
        J_SPH.M[5][3]=(-x*y) /(r_XZ*pow(r,2));
        J_SPH.M[5][4]=r_XZ /(pow(r,2));
        J_SPH.M[5][5]=(-y*z) /(r_XZ*pow(r,2));

        Square_Matrix<6> J_SPH_tr=Square_Matrix<6>(6,6);
        J_SPH.transp(&J_SPH_tr);
        J_SPH.MatrXMatrXMatr(CM_NUE,&J_SPH_tr,CM_SPH);
    }
    else
    {
        bResult = false;
    }

    delete Coord_Sf;
    delete Vel_Sf;

    return  bResult;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel_accel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the topocentric system of coordinates to the
//                   spherical system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_TOPO - topocentric coordinates, Vel_TOPO - topocentric velocity  and Acc_TOPO - topocentric acceleration;
//                  CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity and acceleration components;
//
// RETURNS	   : CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates, velocity and acceleration components;
//

bool Recount_CovMatr_NUEtoSPHCS_coord_vel_accel(const CTopocentric *Coord_TOPO,const CTopocentric *Vel_TOPO,const CTopocentric *Acc_TOPO,
                                                Square_Matrix<9> *CM_NUE, Square_Matrix<9> *CM_SPH)
{
    if((!Coord_TOPO) ||
            (!CM_NUE) ||
            (Coord_TOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (Coord_TOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (Coord_TOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }

    CM_SPH->Reset(9,9);

    bool bResult = true;
    CSpherical Coord_Sf;
    CSpherical Vel_Sf;
    CSpherical Acc_Sf;
    double x=Coord_TOPO->m_dXt;
    double y=Coord_TOPO->m_dYt;
    double z=Coord_TOPO->m_dZt;
    double vX=Vel_TOPO->m_dXt;
    double vY=Vel_TOPO->m_dYt;
    double vZ=Vel_TOPO->m_dZt;
    double aX=Acc_TOPO->m_dXt;
    double aY=Acc_TOPO->m_dYt;
    double aZ=Acc_TOPO->m_dZt;

    double r_XZ=sqrt(pow(x,2)+pow(z,2));
    TOPO_SPHERICAL_coord_vel_accel(Coord_TOPO,Vel_TOPO,Acc_TOPO,&Coord_Sf,&Vel_Sf,&Acc_Sf);
    double r=Coord_Sf.m_dR;
    //    double betta=Coord_Sf.m_dB;
    //    double epsilon=Coord_Sf.m_dE;
    double vR=Vel_Sf.m_dR;
    double vBetta=Vel_Sf.m_dB;
    double vEpsilon=Vel_Sf.m_dE;
    double aR=Acc_Sf.m_dR;
    double aBetta=Acc_Sf.m_dB;
    double aEpsilon=Acc_Sf.m_dE;

    if (r > 0 && r_XZ > 0)
    {
        Square_Matrix<9> j_SPH=Square_Matrix<9>(9,9);

        j_SPH.s_n=9;
        j_SPH.s_m=9;

        j_SPH.M[0][0]=x/r;
        j_SPH.M[0][1]=y/r;
        j_SPH.M[0][2]=z/r;
        j_SPH.M[0][3]=0;
        j_SPH.M[0][4]=0;
        j_SPH.M[0][5]=0;
        j_SPH.M[0][6]=0;
        j_SPH.M[0][7]=0;
        j_SPH.M[0][8]=0;

        j_SPH.M[1][0]=-z/pow(r_XZ,2);
        j_SPH.M[1][1]=0;
        j_SPH.M[1][2]=x /pow(r_XZ,2);
        j_SPH.M[1][3]=0;
        j_SPH.M[1][4]=0;
        j_SPH.M[1][5]=0;
        j_SPH.M[1][6]=0;
        j_SPH.M[1][7]=0;
        j_SPH.M[1][8]=0;

        j_SPH.M[2][0]=(-x*y) /(r_XZ*pow(r,2));
        j_SPH.M[2][1]=r_XZ /(pow(r,2));
        j_SPH.M[2][2]=(-y*z) /(r_XZ*pow(r,2));
        j_SPH.M[2][3]=0;
        j_SPH.M[2][4]=0;
        j_SPH.M[2][5]=0;
        j_SPH.M[2][6]=0;
        j_SPH.M[2][7]=0;
        j_SPH.M[2][8]=0;

        j_SPH.M[3][0]=vX/r-vR*x/pow(r,2);
        j_SPH.M[3][1]=vY/r-vR*y/pow(r,2);
        j_SPH.M[3][2]=vZ/r-vR*z/pow(r,2);
        j_SPH.M[3][3]=x/r;
        j_SPH.M[3][4]=y/r;
        j_SPH.M[3][5]=z/r;
        j_SPH.M[3][6]=0;
        j_SPH.M[3][7]=0;
        j_SPH.M[3][8]=0;

        j_SPH.M[4][0]=(vZ-2*vBetta*x)/pow(r_XZ,2);
        j_SPH.M[4][1]=0;
        j_SPH.M[4][2]=(-vX-2*vBetta*z)/pow(r_XZ,2);
        j_SPH.M[4][3]=-z/pow(r_XZ,2);
        j_SPH.M[4][4]=0;
        j_SPH.M[4][5]=x/pow(r_XZ,2);
        j_SPH.M[4][6]=0;
        j_SPH.M[4][7]=0;
        j_SPH.M[4][8]=0;

        j_SPH.M[5][0]=-y/(pow(r,2)*r_XZ)*(vX-2*x*vR/r)-vEpsilon*x/pow(r_XZ,2);
        j_SPH.M[5][1]=-(x*vX+z*vZ)/(pow(r,2)*r_XZ)-2*vEpsilon*y/pow(r,2);
        j_SPH.M[5][2]=-y/(pow(r,2)*r_XZ)*(vZ-2*z*vR/r)-vEpsilon*z/pow(r_XZ,2);
        j_SPH.M[5][3]=(-x*y) /(r_XZ*pow(r,2));
        j_SPH.M[5][4]=r_XZ /(pow(r,2));
        j_SPH.M[5][5]=(-y*z) /(r_XZ*pow(r,2));
        j_SPH.M[5][6]=0;
        j_SPH.M[5][7]=0;
        j_SPH.M[5][8]=0;


        j_SPH.M[6][0]=aX/r-2*vR*vX/pow(r,2)+2*pow(vR,2)*x/pow(r,3)-aR*x/pow(r,2);
        j_SPH.M[6][1]=aY/r-2*vR*vY/pow(r,2)+2*pow(vR,2)*y/pow(r,3)-aR*y/pow(r,2);
        j_SPH.M[6][2]=aZ/r-2*vR*vZ/pow(r,2)+2*pow(vR,2)*z/pow(r,3)-aR*z/pow(r,2);
        j_SPH.M[6][3]=2*(vX/r-vR*x/pow(r,2));
        j_SPH.M[6][4]=2*(vY/r-vR*y/pow(r,2));
        j_SPH.M[6][5]=2*(vZ/r-vR*z/pow(r,2));
        j_SPH.M[6][6]=x/r;
        j_SPH.M[6][7]=y/r;
        j_SPH.M[6][8]=z/r;


        j_SPH.M[7][0]=(aZ-2*aBetta*x-2*vBetta*vX)/pow(r_XZ,2)+2*((x*vX+z*vZ)*(2*vBetta*x-vZ))/pow(r_XZ,4);
        j_SPH.M[7][1]=0;
        j_SPH.M[7][2]=(-aX-2*aBetta*z-2*vBetta*vZ)/pow(r_XZ,2)+2*((x*vX+z*vZ)*(2*vBetta*z+vX))/pow(r_XZ,4);
        j_SPH.M[7][3]=2*(vZ-2*vBetta*x)/pow(r_XZ,2);
        j_SPH.M[7][4]=0;
        j_SPH.M[7][5]=2*((-vX-2*vBetta*z)/pow(r_XZ,2));
        j_SPH.M[7][6]=-z/pow(r_XZ,2);
        j_SPH.M[7][7]=0;
        j_SPH.M[7][8]=x /pow(r_XZ,2);


        j_SPH.M[8][0]=-aEpsilon*x*(2/pow(r,2)+1/pow(r_XZ,2))+(2*x*aY+vX*vY-y*aX)/(pow(r,2)*r_XZ)+(y*vX-2*x*vY)/(pow(r,2)*r_XZ)*((x*vX+z*vZ)/pow(r_XZ,2)+2*vR/r)-vEpsilon*(vX*(2/pow(r,2)+1/pow(r_XZ,2))-2*x*(x*vX+z*vZ)/pow(r_XZ,4)-4*x*vR/pow(r,3));
        j_SPH.M[8][1]=-2*aEpsilon*y/pow(r,2)-(x*aX+z*aZ+pow(vX,2)+pow(vZ,2))/(pow(r,2)*r_XZ)+(x*vX+z*vZ)/(pow(r,2)*r_XZ)*((x*vX+z*vZ)/pow(r_XZ,2)+2*vR/r)+2*vEpsilon*(2*y*vR/pow(r,3)-vY/pow(r,2));
        j_SPH.M[8][2]=-aEpsilon*z*(2/pow(r,2)+1/pow(r_XZ,2))+(2*z*aY+vZ*vY-y*aZ)/(pow(r,2)*r_XZ)+(y*vZ-2*z*vY)/(pow(r,2)*r_XZ)*((x*vX+z*vZ)/pow(r_XZ,2)+2*vR/r)-vEpsilon*(vZ*(2/pow(r,2)+1/pow(r_XZ,2))-2*z*(x*vX+z*vZ)/pow(r_XZ,4)-4*z*vR/pow(r,3));
        j_SPH.M[8][3]=2*(-y/(pow(r,2)*r_XZ)*(vX-2*x*vR/r)-vEpsilon*x/pow(r_XZ,2));
        j_SPH.M[8][4]=2*(-(x*vX+z*vZ)/(pow(r,2)*r_XZ)-2*vEpsilon*y/pow(r,2));
        j_SPH.M[8][5]=2*(-y/(pow(r,2)*r_XZ)*(vZ-2*z*vR/r)-vEpsilon*z/pow(r_XZ,2));
        j_SPH.M[8][6]=(-x*y) /(r_XZ*pow(r,2));
        j_SPH.M[8][7]=r_XZ /(pow(r,2));
        j_SPH.M[8][8]=(-y*z) /(r_XZ*pow(r,2));

        Square_Matrix<9> J_SPH_tr=Square_Matrix<9>(9,9);
        j_SPH.transp(&J_SPH_tr);
        j_SPH.MatrXMatrXMatr(CM_NUE,&J_SPH_tr,CM_SPH);
    }
    else
    {
        bResult = false;
    }

    return bResult;
}

// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_SPHCStoNUEcoord
//
// DESCRIPTION     : Function of recalculation covariance matrix from the spherical system of coordinates to the
//                   topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates; CM_SPH - the spherical matrix of the measurement errors spherical coordinates;
//
// RETURNS	       : CM_NUE - the covariance matrix of the measurement errors of topocentric coordinates;
//

bool Recount_CovMatr_SPHCStoNUEcoord(const CSpherical *Coord_Sf, Square_Matrix<3> *CM_SPH, Square_Matrix<3> *CM_NUE)
{
    if((!Coord_Sf) ||
            (!CM_SPH) ||
            (Coord_Sf->m_dR < cdSPHERIC_R_MIN) ||
            (Coord_Sf->m_dR > cdSPHERIC_R_MAX) ||
            (Coord_Sf->m_dE < cdSPHERIC_E_MIN) ||
            (Coord_Sf->m_dE > cdSPHERIC_E_MAX) ||
            (Coord_Sf->m_dB < cdSPHERIC_B_MIN) ||
            (Coord_Sf->m_dB > cdSPHERIC_B_MAX))
    {
        return false;
    }

    CM_NUE->Reset(3,3);

    double r=Coord_Sf->m_dR;
    double betta=Coord_Sf->m_dB;
    double epsilon=Coord_Sf->m_dE;

    //    CTopocentric *Coord_TOPO;
    //    SPHERICAL_TOPO(*Coord_Sf,*Coord_TOPO);
    Square_Matrix<3> j_NUE=Square_Matrix<3>(3,3);

    j_NUE.M[0][0]=cos(betta)*cos(epsilon);
    j_NUE.M[0][1]=-r*sin(betta)*cos(epsilon);
    j_NUE.M[0][2]=-r*cos(betta)*sin(epsilon);

    j_NUE.M[1][0]=sin(epsilon);
    j_NUE.M[1][1]=0;
    j_NUE.M[1][2]=r*cos(epsilon);

    j_NUE.M[2][0]=sin(betta)*cos(epsilon);
    j_NUE.M[2][1]=r*cos(betta)*cos(epsilon);
    j_NUE.M[2][2]=-r*sin(betta)*sin(epsilon);

    Square_Matrix<3> j_NUE_tr=Square_Matrix<3>(3,3);
    j_NUE.transp(&j_NUE_tr);
    j_NUE.MatrXMatrXMatr(CM_SPH,&j_NUE_tr,CM_NUE);

    return true;

}



// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the spherical system of coordinates to the
//                  topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity; CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates and velocity components;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates and velocity components;
//

bool Recount_CovMatr_SPHCStoNUEcoord_vel(const CSpherical *Coord_Sf,const CSpherical *Vel_Sf,
                                         Square_Matrix<6> *CM_SPH, Square_Matrix<6> *CM_NUE)
{
    if((!Coord_Sf) ||
            (!CM_SPH) ||
            (Coord_Sf->m_dR < cdSPHERIC_R_MIN) ||
            (Coord_Sf->m_dR > cdSPHERIC_R_MAX) ||
            (Coord_Sf->m_dE < cdSPHERIC_E_MIN) ||
            (Coord_Sf->m_dE > cdSPHERIC_E_MAX) ||
            (Coord_Sf->m_dB < cdSPHERIC_B_MIN) ||
            (Coord_Sf->m_dB > cdSPHERIC_B_MAX))
    {
        return false;
    }

    CM_NUE->Reset(6,6);

    double r=Coord_Sf->m_dR;
    double betta=Coord_Sf->m_dB;
    double epsilon=Coord_Sf->m_dE;
    double vR=Vel_Sf->m_dR;
    double vBetta=Vel_Sf->m_dB;
    double vEpsilon=Vel_Sf->m_dE;

    Square_Matrix<6> j_NUE=Square_Matrix<6>(6,6);

    j_NUE.M[0][0]=cos(betta)*cos(epsilon);
    j_NUE.M[0][1]=-r*sin(betta)*cos(epsilon);
    j_NUE.M[0][2]=-r*cos(betta)*sin(epsilon);
    j_NUE.M[0][3]=0;
    j_NUE.M[0][4]=0;
    j_NUE.M[0][5]=0;

    j_NUE.M[1][0]=sin(epsilon);
    j_NUE.M[1][1]=0;
    j_NUE.M[1][2]=r*cos(epsilon);
    j_NUE.M[1][3]=0;
    j_NUE.M[1][4]=0;
    j_NUE.M[1][5]=0;

    j_NUE.M[2][0]=sin(betta)*cos(epsilon);
    j_NUE.M[2][1]=r*cos(betta)*cos(epsilon);
    j_NUE.M[2][2]=-r*sin(betta)*sin(epsilon);
    j_NUE.M[2][3]=0;
    j_NUE.M[2][4]=0;
    j_NUE.M[2][5]=0;

    j_NUE.M[3][0]=-vBetta*sin(betta)*cos(epsilon)-vEpsilon*cos(betta)*sin(epsilon);
    j_NUE.M[3][1]=-r*vBetta*cos(betta)*cos(epsilon)+r*vEpsilon*sin(betta)*sin(epsilon)-vR*sin(betta)*cos(epsilon);
    j_NUE.M[3][2]=r*vBetta*sin(betta)*sin(epsilon)-r*vEpsilon*cos(betta)*cos(epsilon)-vR*cos(betta)*sin(epsilon);
    j_NUE.M[3][3]=cos(betta)*cos(epsilon);
    j_NUE.M[3][4]=-r*sin(betta)*cos(epsilon);
    j_NUE.M[3][5]=-r*cos(betta)*sin(epsilon);


    j_NUE.M[4][0]=vEpsilon*cos(epsilon);
    j_NUE.M[4][1]=0;
    j_NUE.M[4][2]=vR*cos(epsilon)-r*vEpsilon*sin(epsilon);
    j_NUE.M[4][3]=sin(epsilon);
    j_NUE.M[4][4]=0;
    j_NUE.M[4][5]=r*cos(epsilon);


    j_NUE.M[5][0]=vBetta*cos(betta)*cos(epsilon)-vEpsilon*sin(betta)*sin(epsilon);
    j_NUE.M[5][1]=-r*vBetta*sin(betta)*cos(epsilon)-r*vEpsilon*cos(betta)*sin(epsilon)+vR*cos(betta)*cos(epsilon);
    j_NUE.M[5][2]=-r*vBetta*cos(betta)*sin(epsilon)-r*vEpsilon*sin(betta)*cos(epsilon)-vR*sin(betta)*sin(epsilon);
    j_NUE.M[5][3]=sin(betta)*cos(epsilon);
    j_NUE.M[5][4]=r*cos(betta)*cos(epsilon);
    j_NUE.M[5][5]=-r*sin(betta)*sin(epsilon);


    Square_Matrix<6> j_NUE_tr=Square_Matrix<6>(6,6);
    j_NUE.transp(&j_NUE_tr);
    j_NUE.MatrXMatrXMatr(CM_SPH,&j_NUE_tr,CM_NUE);

    return true;
}


// PACKAGE		: Conversion
// FUNCTION        : Recount_CovMatr_NUEtoSPHCS_coord_vel_accel
//
// DESCRIPTION     : Function of recalculation covariance matrix from the spherical system of coordinates to the
//                  topocentric system of coordinates
//                   All the parameters are in radians (angles) and metres (distanses).
//
// INPUTS          : Coord_Sf - spherical coordinates, Vel_Sf - spherical velocity and Acc_Sf - spherical acceleration;
//                  CM_SPH - the covariance matrix of errors in the determination of spherical
//                  coordinates, velocity and acceleration components;
//
// RETURNS	   : CM_NUE - the covariance matrix of errors in the determination of topocentric
//                  coordinates, velocity and acceleration components;
//

bool Recount_CovMatr_SPHCStoNUEcoord_vel_acc(const CSpherical *Coord_Sf,const CSpherical *Vel_Sf,const CSpherical *Acc_Sf,
                                             Square_Matrix<9> *CM_SPH, Square_Matrix<9> *CM_NUE)
{
    if((!Coord_Sf) ||
            (!CM_SPH) ||
            (Coord_Sf->m_dR < cdSPHERIC_R_MIN) ||
            (Coord_Sf->m_dR > cdSPHERIC_R_MAX) ||
            (Coord_Sf->m_dE < cdSPHERIC_E_MIN) ||
            (Coord_Sf->m_dE > cdSPHERIC_E_MAX) ||
            (Coord_Sf->m_dB < cdSPHERIC_B_MIN) ||
            (Coord_Sf->m_dB > cdSPHERIC_B_MAX))
    {
        return false;
    }

    CM_NUE->Reset(9,9);

    double r=Coord_Sf->m_dR;
    double betta=Coord_Sf->m_dB;
    double epsilon=Coord_Sf->m_dE;
    double vR=Vel_Sf->m_dR;
    double vBetta=Vel_Sf->m_dB;
    double vEpsilon=Vel_Sf->m_dE;
    double aR=Acc_Sf->m_dR;
    double aBetta=Acc_Sf->m_dB;
    double aEpsilon=Acc_Sf->m_dE;

    Square_Matrix<9> j_NUE=Square_Matrix<9>(9,9);
    j_NUE.s_m=9;
    j_NUE.s_n=9;
    j_NUE.M[0][0]=cos(betta)*cos(epsilon);
    j_NUE.M[0][1]=-r*sin(betta)*cos(epsilon);
    j_NUE.M[0][2]=-r*cos(betta)*sin(epsilon);
    j_NUE.M[0][3]=0;
    j_NUE.M[0][4]=0;
    j_NUE.M[0][5]=0;
    j_NUE.M[0][6]=0;
    j_NUE.M[0][7]=0;
    j_NUE.M[0][8]=0;

    j_NUE.M[1][0]=sin(epsilon);
    j_NUE.M[1][1]=0;
    j_NUE.M[1][2]=r*cos(epsilon);
    j_NUE.M[1][3]=0;
    j_NUE.M[1][4]=0;
    j_NUE.M[1][5]=0;
    j_NUE.M[1][6]=0;
    j_NUE.M[1][7]=0;
    j_NUE.M[1][8]=0;

    j_NUE.M[2][0]=sin(betta)*cos(epsilon);
    j_NUE.M[2][1]=r*cos(betta)*cos(epsilon);
    j_NUE.M[2][2]=-r*sin(betta)*sin(epsilon);
    j_NUE.M[2][3]=0;
    j_NUE.M[2][4]=0;
    j_NUE.M[2][5]=0;
    j_NUE.M[2][6]=0;
    j_NUE.M[2][7]=0;
    j_NUE.M[2][8]=0;

    j_NUE.M[3][0]=-vBetta*sin(betta)*cos(epsilon)-vEpsilon*cos(betta)*sin(epsilon);
    j_NUE.M[3][1]=-vR*sin(betta)*cos(epsilon)-r*vBetta*cos(betta)*cos(epsilon)+r*vEpsilon*sin(betta)*sin(epsilon);
    j_NUE.M[3][2]=r*vBetta*sin(betta)*sin(epsilon)-r*vEpsilon*cos(betta)*cos(epsilon)-vR*cos(betta)*sin(epsilon);
    j_NUE.M[3][3]=cos(betta)*cos(epsilon);
    j_NUE.M[3][4]=-r*sin(betta)*cos(epsilon);
    j_NUE.M[3][5]=-r*cos(betta)*sin(epsilon);
    j_NUE.M[3][6]=0;
    j_NUE.M[3][7]=0;
    j_NUE.M[3][8]=0;

    j_NUE.M[4][0]=vEpsilon*cos(epsilon);
    j_NUE.M[4][1]=0;
    j_NUE.M[4][2]=vR*cos(epsilon)-r*vEpsilon*sin(epsilon);
    j_NUE.M[4][3]=sin(epsilon);
    j_NUE.M[4][4]=0;
    j_NUE.M[4][5]=r*cos(epsilon);
    j_NUE.M[4][6]=0;
    j_NUE.M[4][7]=0;
    j_NUE.M[4][8]=0;

    j_NUE.M[5][0]=vBetta*cos(betta)*cos(epsilon)-vEpsilon*sin(betta)*sin(epsilon);
    j_NUE.M[5][1]=-r*vBetta*sin(betta)*cos(epsilon)-r*vEpsilon*cos(betta)*sin(epsilon)+vR*cos(betta)*cos(epsilon);
    j_NUE.M[5][2]=-r*vBetta*cos(betta)*sin(epsilon)-r*vEpsilon*sin(betta)*cos(epsilon)-vR*sin(betta)*sin(epsilon);
    j_NUE.M[5][3]=sin(betta)*cos(epsilon);
    j_NUE.M[5][4]=r*cos(betta)*cos(epsilon);
    j_NUE.M[5][5]=-r*sin(betta)*sin(epsilon);
    j_NUE.M[5][6]=0;
    j_NUE.M[5][7]=0;
    j_NUE.M[5][8]=0;

    j_NUE.M[6][0]=-(pow(vBetta,2)+pow(vEpsilon,2))*cos(betta)*cos(epsilon)+2*vBetta*vEpsilon*sin(betta)*sin(epsilon)
            -aEpsilon*cos(betta)*sin(epsilon)-aBetta*sin(betta)*cos(epsilon);//+
    j_NUE.M[6][1]=-(aR-r*(pow(vBetta,2)+pow(vEpsilon,2)))*sin(betta)*cos(epsilon)+2*r*vBetta*vEpsilon*cos(betta)*sin(epsilon)-
            (-r*aEpsilon-2*vR*vEpsilon)*sin(betta)*sin(epsilon)+(-r*aBetta-2*vR*vBetta)*cos(betta)*cos(epsilon);//+
    j_NUE.M[6][2]= -(aR-r*(pow(vBetta,2)+pow(vEpsilon,2)))*cos(betta)*sin(epsilon)+2*r*vBetta*vEpsilon*sin(betta)*cos(epsilon)+
            (-r*aEpsilon-2*vR*vEpsilon)*cos(betta)*cos(epsilon)-(-r*aBetta-2*vR*vBetta)*sin(betta)*sin(epsilon);//+
    j_NUE.M[6][3]=(-2*vBetta*sin(betta)*cos(epsilon)-2*vEpsilon*cos(betta)*sin(epsilon));
    j_NUE.M[6][4]=(-2*r*vBetta*cos(betta)*cos(epsilon)+2*r*vEpsilon*sin(betta)*sin(epsilon)-2*vR*sin(betta)*cos(epsilon));
    j_NUE.M[6][5]=(2*r*vBetta*sin(betta)*sin(epsilon)-2*r*vEpsilon*cos(betta)*cos(epsilon)-2*vR*cos(betta)*sin(epsilon));
    j_NUE.M[6][6]=cos(betta)*cos(epsilon);
    j_NUE.M[6][7]=-r*sin(betta)*cos(epsilon);
    j_NUE.M[6][8]=-r*cos(betta)*sin(epsilon);
    //
    j_NUE.M[7][0]=aEpsilon*cos(epsilon)-pow(vEpsilon,2.0)*sin(epsilon);
    j_NUE.M[7][1]=0;
    j_NUE.M[7][2]=(aR-r*pow(vEpsilon,2))*cos(epsilon)-(r*aEpsilon+2*vR*vEpsilon)*sin(epsilon);
    j_NUE.M[7][3]=2*vEpsilon*cos(epsilon);
    j_NUE.M[7][4]=0;
    j_NUE.M[7][5]=(2*vR*cos(epsilon)-2*r*vEpsilon*sin(epsilon));
    j_NUE.M[7][6]=sin(epsilon);
    j_NUE.M[7][7]=0;
    j_NUE.M[7][8]=r*cos(epsilon);
    //
    j_NUE.M[8][0]=aBetta*cos(betta)*cos(epsilon)-aEpsilon*sin(betta)*sin(epsilon)-
            2*vBetta*vEpsilon*cos(betta)*sin(epsilon)-(pow(vBetta,2)+pow(vEpsilon,2))*sin(betta)*cos(epsilon);//+
    j_NUE.M[8][1]=-(r*aBetta+2*vR*vBetta)*sin(betta)*cos(epsilon)-(r*aEpsilon+2*vR*vEpsilon)*cos(betta)*sin(epsilon)+
            2*r*vBetta*vEpsilon*sin(betta)*sin(epsilon)+(aR-r*(pow(vBetta,2)+pow(vEpsilon,2)))*cos(betta)*cos(epsilon);//+
    j_NUE.M[8][2]=-(r*aBetta+2*vR*vBetta)*cos(betta)*sin(epsilon)-(r*aEpsilon+2*vR*vEpsilon)*sin(betta)*cos(epsilon)-
            2*r*vBetta*vEpsilon*cos(betta)*cos(epsilon)-(aR-r*(pow(vBetta,2)+pow(vEpsilon,2)))*sin(betta)*sin(epsilon);
    j_NUE.M[8][3]=2*(vBetta*cos(betta)*cos(epsilon)-vEpsilon*sin(betta)*sin(epsilon));
    j_NUE.M[8][4]=2*(-r*vBetta*sin(betta)*cos(epsilon)-r*vEpsilon*cos(betta)*sin(epsilon)+vR*cos(betta)*cos(epsilon));
    j_NUE.M[8][5]=2*(-r*vBetta*cos(betta)*sin(epsilon)-r*vEpsilon*sin(betta)*cos(epsilon)-vR*sin(betta)*sin(epsilon));
    j_NUE.M[8][6]=sin(betta)*cos(epsilon);
    j_NUE.M[8][7]=r*cos(betta)*cos(epsilon);
    j_NUE.M[8][8]=-r*sin(betta)*sin(epsilon);

    Square_Matrix<9> j_NUE_tr=Square_Matrix<9>(9,9);
    j_NUE.transp(&j_NUE_tr);
    j_NUE.MatrXMatrXMatr(CM_SPH,&j_NUE_tr,CM_NUE);

    return true;
}


bool TOPO__GET_AZIMUTH(const CTopocentric *pTOPO, double *pAzimuth)
{
    bool bOK = true;
    if((!pTOPO) ||
            (!pAzimuth) ||
            (pTOPO->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pTOPO->m_dYt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dYt > cdTOPO_X_Y_Z_MAX) ||
            (pTOPO->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pTOPO->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        bOK = false;
    }
    else
    {
        *pAzimuth = 0;

        if ((pTOPO->m_dXt) != 0)
        {
            *pAzimuth = atan((pTOPO->m_dZt)/(pTOPO->m_dXt));
        }

        *pAzimuth = fabs(*pAzimuth);

        if (((pTOPO->m_dZt) > 0) && ((pTOPO->m_dXt) < 0))
        {
            *pAzimuth = cdPi-(*pAzimuth);
        }

        if (((pTOPO->m_dZt) < 0) && ((pTOPO->m_dXt) > 0))
        {
            *pAzimuth = 2*cdPi-(*pAzimuth);
        }

        if (((pTOPO->m_dZt) < 0) && ((pTOPO->m_dXt) < 0))
        {
            *pAzimuth = cdPi+(*pAzimuth);
        }

        if (((pTOPO->m_dZt) >= 0) && ((pTOPO->m_dXt) == 0))
        {
            *pAzimuth = cdPi/2;
        }

        if (((pTOPO->m_dZt) < 0) && ((pTOPO->m_dXt) == 0))
        {
            *pAzimuth = 3*cdPi/2;
        }

        if (((pTOPO->m_dZt) == 0) && ((pTOPO->m_dXt) > 0))
        {
            *pAzimuth = 0;
        }

        if (((pTOPO->m_dZt) == 0) && ((pTOPO->m_dXt) < 0))
        {
            *pAzimuth = cdPi;
        }
    }
    return bOK;
}
