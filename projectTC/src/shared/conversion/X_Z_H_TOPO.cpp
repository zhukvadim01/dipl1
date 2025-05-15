#include "X_Z_H_TOPO.h"
#include "ConstRecalc.h"
#include "Geocentric_Geo.h"
#include "Geocentric.h"
#include "Geodesic.h"
#include "Geocentric_Topo.h"
#include "Topo1_Topo2.h"

#include <cmath>
#include <cstdio>
#include <cerrno>
#include <cassert>

bool X_Z_H_TOPO(const CGeodesic *pCenter1, const CTopocentric *pPosition_X_Z_H, CTopocentric *pPositionTopo)
{
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }

    //double Re = 6372000.0;
    double Re = cdEquatorAxis*(1-cdFirstEccentricity)/pow(sqrt((1-cdFirstEccentricity*sin(pCenter1->m_dLatitude))),3);
    double h = pCenter1->m_dAltitude;
    double dist = pow((Re+pPosition_X_Z_H->m_dYt),2) -
            pPosition_X_Z_H->m_dXt*pPosition_X_Z_H->m_dXt -
            pPosition_X_Z_H->m_dZt*pPosition_X_Z_H->m_dZt;
    if( dist < 0 ) {
        return false;
    }
    pPositionTopo->m_dXt = pPosition_X_Z_H->m_dXt;
    pPositionTopo->m_dYt = sqrt(dist)-Re-h;
    pPositionTopo->m_dZt = pPosition_X_Z_H->m_dZt;

    return true;
}


bool X_Z_H_TOPO_It(const CGeodesic *pCenter1, const CTopocentric *pPosition_X_Z_H, CTopocentric *pPositionTopo)
{
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pPositionTopo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }

    CTopocentric pCoordTopo;
    if (!X_Z_H_TOPO(pCenter1, pPosition_X_Z_H,&pCoordTopo))
    {
        return false;
    }

    double h_Topo_temp = pCoordTopo.m_dYt;
    CTopocentric pCoordTopoTemp;
    pCoordTopoTemp.m_dXt = pPosition_X_Z_H->m_dXt;
    pCoordTopoTemp.m_dYt = pCoordTopo.m_dYt;
    pCoordTopoTemp.m_dZt = pPosition_X_Z_H->m_dZt;
    CGeocentric pPositionWGSTemp;
    CGeodesic pPositionGeoTemp;
    if (!TOPO_GEOCENTRIC(pCenter1, &pCoordTopoTemp, &pPositionWGSTemp))
    {
        return false;
    }
    if (!GEOCENTRIC_GEO(&pPositionWGSTemp, &pPositionGeoTemp))
    {
        return false;
    }
    double h_Geo_temp = pPositionGeoTemp.m_dAltitude;

    double deltaH = h_Geo_temp-pPosition_X_Z_H->m_dYt;
    double first_step = fabs(deltaH);
    int counter=0;

    while (fabs(deltaH) > cdXZH_TOPO_DELTA)
    {
        counter++;
        if(counter > NMaxCycle)
        {
            return false;
        }

        if(deltaH>0)
        {
            h_Topo_temp-=first_step;
        }
        else
        {
            h_Topo_temp+=first_step;
        }

        pCoordTopoTemp.m_dYt = h_Topo_temp;
        if (!TOPO_GEOCENTRIC(pCenter1, &pCoordTopoTemp, &pPositionWGSTemp))
        {
            return false;
        }
        if (!GEOCENTRIC_GEO(&pPositionWGSTemp, &pPositionGeoTemp))
        {
            return false;
        }

        h_Geo_temp = pPositionGeoTemp.m_dAltitude;
        double deltaH_temp = h_Geo_temp-pPosition_X_Z_H->m_dYt;
        if (std::signbit(deltaH) != std::signbit(deltaH_temp))
        {
            if(deltaH>0)
            {
                h_Topo_temp+=first_step;
                first_step=fabs(deltaH);
            }
            else
            {
                h_Topo_temp-=first_step;
                first_step=fabs(deltaH);
            }
        }
        else
        {
            deltaH = deltaH_temp;
        }
    }

    pPositionTopo->m_dXt = pPosition_X_Z_H->m_dXt;
    pPositionTopo->m_dYt = h_Topo_temp;
    pPositionTopo->m_dZt = pPosition_X_Z_H->m_dZt;
    return true;
}

bool Vel_X_Z_H_TOPO(const CGeodesic *pCenter1, const CGeocentric *pPositionGEO, const CTopocentric *pVelocity_X_Z_H, CTopocentric *pVelocityTopo)
{
    if ((!pCenter1) ||
            (!pVelocity_X_Z_H) ||
            (!pPositionGEO) ||
            (!pVelocityTopo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) )
    {
        return false;
    }

    CGeodesic l_positionGDS;
    if ( !GEOCENTRIC_GEO(pPositionGEO, &l_positionGDS) )
    {
        return false;
    }

    SKoefRecal  l_RecalcCoeff;
    if ( !InitTopo1_Topo2_SHIFT (pCenter1, &l_positionGDS, &l_RecalcCoeff) )
    {
        return false;
    }

    double Vy = 0.;
    Vy += l_RecalcCoeff.m_dKy1 * pVelocity_X_Z_H->m_dXt;
    Vy += l_RecalcCoeff.m_dKy3 * pVelocity_X_Z_H->m_dZt;
    Vy -= pVelocity_X_Z_H->m_dYt;
    Vy /= l_RecalcCoeff.m_dKy2;

    pVelocityTopo->init(pVelocity_X_Z_H->m_dXt, Vy, pVelocity_X_Z_H->m_dZt);

    return true;
}

bool D_AZ_H_SFSC(const CGeodesic *pCenter1,  double D, double Az, double H, CSpherical *pSF)
{
    if((!pSF) ||
            (D < cdSPHERIC_R_MIN) ||
            (D > cdSPHERIC_R_MAX) ||
            (H < cd_H_MIN) ||
            (H > cd_H_MAX) ||
            (Az< cdSPHERIC_B_MIN) ||
            (Az > cdSPHERIC_B_MAX))
    {
        return false;
    }


    //double Re = 6372000;
    double Re = cdEquatorAxis*(1-cdFirstEccentricity)/pow(sqrt((1-cdFirstEccentricity*sin(pCenter1->m_dLatitude))),3);
    pSF->m_dR = D;
    double Alt = pCenter1->m_dAltitude;
    if(D==0)
    {
        pSF->m_dE = 0;
    }
    else
        if(fabs((Re+Alt)*(Re+Alt)+D*D-(Re+H)*(Re+H))>(2*(Re+Alt)*D))
        {
            return false;
        }
        else
        {
            pSF->m_dE = acos(((Re+Alt)*(Re+Alt)+D*D-(Re+H)*(Re+H))/(2*(Re+Alt)*D))-cdPi/2;
        }

    pSF->m_dB = Az;

    return true;
}

bool D_AZ_H_TOPO_It(const CGeodesic *pCenter1, double D, double Az, double H, CTopocentric *pPositionTopo, CSpherical *pPositionSF)
{
    if((!pPositionTopo) ||
            (!pCenter1) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (D < cdSPHERIC_R_MIN) ||
            (D > cdSPHERIC_R_MAX) ||
            (H < cd_H_MIN) ||
            (H > cd_H_MAX) ||
            (Az< cdSPHERIC_B_MIN) ||
            (Az > cdSPHERIC_B_MAX))
    {
        return false;
    }
    double first_step = 0.004;
    double division = 5;
    int counter=0;
    CSpherical pCoordSF;
    if(!D_AZ_H_SFSC(pCenter1, D,  Az,  H, &pCoordSF))
    {
        return false;
    }
    double elev = pCoordSF.m_dE;

    CTopocentric pPositionTopoTemp;
    CGeocentric pPositionWGSTemp;
    CGeodesic pPositionGeoTemp;
    if(!SPHERICAL_TOPO(&pCoordSF,&pPositionTopoTemp))
    {
        return false;
    }
    if(!TOPO_GEOCENTRIC(pCenter1, &pPositionTopoTemp, &pPositionWGSTemp))
    {
        return false;
    }
    if(!GEOCENTRIC_GEO(&pPositionWGSTemp, &pPositionGeoTemp))
    {
        return false;
    }

    double h_geo_temp = pPositionGeoTemp.m_dAltitude;
    double deltaH = h_geo_temp-H;

    while (fabs(deltaH)>1)
    {
        counter++;
        if(counter > NMaxCycle)
        {
            return false;
        }

        if(deltaH>0)
        {
            elev=elev-first_step;
        }
        else
        {
            elev=elev+first_step;
        }

        pCoordSF.m_dE = elev;
        if(!SPHERICAL_TOPO(&pCoordSF,&pPositionTopoTemp))
        {
            return false;
        }
        if(!TOPO_GEOCENTRIC(pCenter1, &pPositionTopoTemp, &pPositionWGSTemp))
        {
            return false;
        }
        if(!GEOCENTRIC_GEO(&pPositionWGSTemp, &pPositionGeoTemp))
        {
            return false;
        }
        h_geo_temp = pPositionGeoTemp.m_dAltitude;
        double deltaH_temp = h_geo_temp-H;

        if (std::signbit(deltaH) != std::signbit(deltaH_temp))
        {
            if(deltaH>0)
            {
                elev = elev+first_step;
                first_step = first_step/division;
            }
            else
            {
                elev = elev-first_step;
                first_step = first_step/division;
            }
        }
        else
        {
            deltaH = deltaH_temp;
        }
    }

    pCoordSF.m_dE = elev;
    if(!SPHERICAL_TOPO(&pCoordSF,pPositionTopo))
    {
        return false;
    }
    if ( pPositionSF != 0 )
    {
        pPositionSF->m_dR = pCoordSF.m_dR;
        pPositionSF->m_dB = pCoordSF.m_dB;
        pPositionSF->m_dE = pCoordSF.m_dE;
    }
    return true;
}

bool R_cos_R_sin_H_TOPO_It(const CGeodesic *pCenter1,
                           const CTopocentric *pPosition_X_Z_H,
                           CTopocentric *pPositionTopo)
{
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pPositionTopo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        return false;
    }
    CTopocentric pPositionTopoTemp;
    pPositionTopoTemp.m_dXt = pPosition_X_Z_H->m_dXt;
    pPositionTopoTemp.m_dYt = 0;
    pPositionTopoTemp.m_dZt = pPosition_X_Z_H->m_dZt;
    CSpherical pCoordSFTemp;
    if(!TOPO_SPHERICAL(&pPositionTopoTemp,&pCoordSFTemp))
    {
        return false;
    }
    double D  = pCoordSFTemp.m_dR;
    double Az = pCoordSFTemp.m_dB;
    double H  = pPosition_X_Z_H->m_dYt;
    if(!(D_AZ_H_TOPO_It(pCenter1, D,  Az,  H, pPositionTopo)))
    {
        return false;
    }
    return true;
}


bool X_Z_H_TOPO_coord_vel(const CGeodesic *pCenter1,
                          const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H,
                          CTopocentric *pPositionTopo, CTopocentric *pVelTopo)
{
    bool bOK = true;
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pPositionTopo) ||
            (!pVel_X_Z_H) ||
            (!pVelTopo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dYt < cd_H_MIN) ||
            (pVel_X_Z_H->m_dYt > cd_H_MAX) ||
            (pVel_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX)   )
    {
        bOK = false;
    }
    else
    {
        bOK = X_Z_H_TOPO_It(pCenter1, pPosition_X_Z_H, pPositionTopo);
        if (bOK)
        {
            // The radius of the main vertical curve
            double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));

            double RE = dN;
            double X = pPosition_X_Z_H->m_dXt;
            double Z = pPosition_X_Z_H->m_dZt;
            double H = pPosition_X_Z_H->m_dYt;
            double VX = pVel_X_Z_H->m_dXt;
            double VZ = pVel_X_Z_H->m_dZt;
            double VH = pVel_X_Z_H->m_dYt;
            double Q2 = (RE+H)*(RE+H) - X*X - Z*Z;
            if (Q2 > 0)
            {
                double Q = sqrt(Q2);
                double VY = ((RE+H)*VH - X*VX - Z*VZ) / Q;
                pVelTopo->m_dXt = VX;
                pVelTopo->m_dYt = VY;
                pVelTopo->m_dZt = VZ;
            }
            else
            {
                bOK = false;
            }
        }
    }
    return bOK;
}


bool X_Z_H_TOPO_coord_vel_acc(const CGeodesic *pCenter1,
                              const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H, const CTopocentric *pAcc_X_Z_H,
                              CTopocentric *pPositionTopo, CTopocentric *pVelTopo, CTopocentric *pAccTopo)
{
    bool bOK = true;
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pPositionTopo) ||
            (!pVel_X_Z_H) ||
            (!pVelTopo) ||
            (!pAcc_X_Z_H) ||
            (!pAccTopo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dYt < cd_H_MIN) ||
            (pVel_X_Z_H->m_dYt > cd_H_MAX) ||
            (pVel_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) ||
            (pAcc_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pAcc_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pAcc_X_Z_H->m_dYt < cd_H_MIN) ||
            (pAcc_X_Z_H->m_dYt > cd_H_MAX) ||
            (pAcc_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pAcc_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX))
    {
        bOK = false;
    }
    else
    {
        bOK = X_Z_H_TOPO_It(pCenter1, pPosition_X_Z_H, pPositionTopo);
        if (bOK)
        {
            // The radius of the main vertical curve
            double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));

            double RE = dN;
            double X = pPosition_X_Z_H->m_dXt;
            double Z = pPosition_X_Z_H->m_dZt;
            double H = pPosition_X_Z_H->m_dYt;
            double VX = pVel_X_Z_H->m_dXt;
            double VZ = pVel_X_Z_H->m_dZt;
            double VH = pVel_X_Z_H->m_dYt;
            double AX = pAcc_X_Z_H->m_dXt;
            double AZ = pAcc_X_Z_H->m_dZt;
            double AH = pAcc_X_Z_H->m_dYt;
            double Q2 = (RE+H)*(RE+H) - X*X - Z*Z;

            if (Q2 > 0)
            {
                double Q = sqrt(Q2);
                double VY = ((RE+H)*VH - X*VX - Z*VZ) / Q;
                pVelTopo->m_dXt = VX;
                pVelTopo->m_dYt = VY;
                pVelTopo->m_dZt = VZ;
                double AY = ((RE+H)*AH - X*AX - Z*AZ + VH*VH - VX*VX - VZ*VZ - VY*VY) / Q;
                pAccTopo->m_dXt = AX;
                pAccTopo->m_dYt = AY;
                pAccTopo->m_dZt = AZ;
            }
            else
            {
                bOK = false;
            }
        }
    }
    return bOK;
}


bool Recount_CovMatr_D_AZ_H_SFSC(const CGeodesic *pCenter1, double D, double H,
                                 Square_Matrix<3> *pCM_DAzH, Square_Matrix<3> *pCM_Sph)
{
    if((!pCM_DAzH) ||
            (!pCM_Sph) ||
            (!pCenter1) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (D < cdSPHERIC_R_MIN) ||
            (D > cdSPHERIC_R_MAX) ||
            (H < cd_H_MIN) ||
            (H > cd_H_MAX))
    {
        return false;
    }
    bool bOK = true;

    // The radius of the main vertical curve
    double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));

    double RE = dN;
    double h0 = pCenter1->m_dAltitude;

    double q = ((RE+h0)*(RE+h0) + D*D - (RE+H)*(RE+H)) / (2.*(RE+h0)*D);
    if (fabs(q) < 1.)
    {
        double denominator = (RE+h0)*D*sqrt(1.-q*q);
        if (fabs(denominator) > 0.)
        {
            double dEps_dD = (q*(RE+h0) - D)/denominator;
            double dEps_dH = (RE+H)/denominator;

            Square_Matrix<3> J(3,3); //Jacobi matrix
            J.M[0][0] = 1.;
            J.M[1][1] = 1.;
            J.M[2][2] = dEps_dH;
            J.M[2][0] = dEps_dD;

            Square_Matrix<3> JT(3,3);
            J.transp(&JT);
            bOK = J.MatrXMatrXMatr(pCM_DAzH, &JT, pCM_Sph);
        }
        else
        {
            bOK = false;
        }
    }
    else
    {
        bOK = false;
    }
    return bOK;
}


bool Recount_CovMatr_D_AZ_H_TOPO(const CGeodesic *pCenter1, double D, double Az, double H,
                                 Square_Matrix<3> *pCM_DAzH, Square_Matrix<3> *pCM_Topo)
{
    bool bOK = true;
    Square_Matrix<3> CM_Sph;
    bOK = Recount_CovMatr_D_AZ_H_SFSC(pCenter1, D, H, pCM_DAzH, &CM_Sph);
    if (bOK)
    {
        CSpherical PosSph;
        bOK = D_AZ_H_SFSC(pCenter1, D, Az, H, &PosSph);
        if (bOK)
        {
            bOK = Recount_CovMatr_SPHCStoNUEcoord(&PosSph, &CM_Sph, pCM_Topo);
        }
    }
    return bOK;
}

bool Recount_CovMatr_X_H_Z_TOPO(const CGeodesic *pCenter1, const CGeocentric *pPositionGEO,
                                Square_Matrix<3> *pCM_XHZ, Square_Matrix<3> *pCM_Topo)
{
    if ((!pCenter1) ||
            (!pPositionGEO) ||
            (!pCM_XHZ) ||
            (!pCM_Topo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPositionGEO->m_dX < cdGEOCEN_X_Y_Z_MIN) ||
            (pPositionGEO->m_dX > cdGEOCEN_X_Y_Z_MAX) ||
            (pPositionGEO->m_dY < cdGEOCEN_X_Y_Z_MIN) ||
            (pPositionGEO->m_dY > cdGEOCEN_X_Y_Z_MAX) ||
            (pPositionGEO->m_dZ < cdGEOCEN_X_Y_Z_MIN) ||
            (pPositionGEO->m_dZ > cdGEOCEN_X_Y_Z_MAX))
    {
        return false;
    }

    bool bOK = false;

    // The radius of the main vertical curve
    double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));
    double RE = dN;

    CGeodesic positionGDS;
    if ( !GEOCENTRIC_GEO(pPositionGEO, &positionGDS) )
    {
        return false;
    }

    CTopocentric positionTOPO;
    if ( !GEOCENTRIC_TOPO(pCenter1, pPositionGEO, &positionTOPO) )
    {
        return false;
    }

    double X = positionTOPO.m_dXt;
    double Z = positionTOPO.m_dZt;
    double H = positionGDS.m_dAltitude;
    double Q2 = (RE+H)*(RE+H) - X*X - Z*Z;
    if (Q2 > 0.)
    {
        double denominator = sqrt(Q2);
        if (fabs(denominator) > 0.)
        {
            double dY_dH = (RE+H)/denominator;
            double dY_dX = X/denominator;
            double dY_dZ = Z/denominator;

            Square_Matrix<3> J(3,3); //Jacobi matrix
            J.M[0][0] = 1.;
            J.M[2][2] = 1.;
            J.M[1][0] = dY_dX;
            J.M[1][1] = dY_dH;
            J.M[1][2] = dY_dZ;

            Square_Matrix<3> JT(3,3);
            J.transp(&JT);
            bOK = J.MatrXMatrXMatr(pCM_XHZ, &JT, pCM_Topo);
        }
    }
    return bOK;
}

bool Recount_CovMatr_Vel_X_H_Z_TOPO(const CGeodesic *pCenter1, const CGeocentric *pPositionGEO,
                                    Square_Matrix<3> *pCM_XHZ, Square_Matrix<3> *pCM_Topo)
{
    if ((!pCenter1) ||
            (!pPositionGEO) ||
            (!pCM_XHZ) ||
            (!pCM_Topo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) )
    {
        return false;
    }

    bool bOK = false;

    CGeodesic l_positionGDS;
    if ( !GEOCENTRIC_GEO(pPositionGEO, &l_positionGDS) )
    {
        return false;
    }

    SKoefRecal  l_RecalcCoeff;
    if ( !InitTopo1_Topo2_SHIFT (pCenter1, &l_positionGDS, &l_RecalcCoeff) )
    {
        return false;
    }

    ////    used formulas
    //    Vy += l_RecalcCoeff.m_dKy1 * pVelocity_X_Z_H->m_dXt;
    //    Vy += l_RecalcCoeff.m_dKy3 * pVelocity_X_Z_H->m_dZt;
    //    Vy -= pVelocity_X_Z_H->m_dYt;
    //    Vy /= l_RecalcCoeff.m_dKy2;

    if (fabs(l_RecalcCoeff.m_dKy2) > 0.)
    {
        double dVy_dVh = l_RecalcCoeff.m_dKy1/l_RecalcCoeff.m_dKy2;
        double dVy_dVx = -1./l_RecalcCoeff.m_dKy2;
        double dVy_dVz = l_RecalcCoeff.m_dKy3/l_RecalcCoeff.m_dKy2;

        Square_Matrix<3> J(3,3); //Jacobi matrix
        J.M[0][0] = 1.;
        J.M[2][2] = 1.;
        J.M[1][0] = dVy_dVx;
        J.M[1][1] = dVy_dVh;
        J.M[1][2] = dVy_dVz;

        Square_Matrix<3> JT(3,3);
        J.transp(&JT);
        bOK = J.MatrXMatrXMatr(pCM_XHZ, &JT, pCM_Topo);
    }
    return bOK;
}

bool Recount_CovMatr_X_Z_H_TOPO_coord(const CGeodesic *pCenter1, const CTopocentric *pPosition_X_Z_H,
                                      Square_Matrix<3> *pCM_X_Z_H, Square_Matrix<3> *pCM_Topo)
{
    bool bOK = true;
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pCM_X_Z_H) ||
            (!pCM_Topo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) )
    {
        bOK = false;
    }
    else
    {
        pCM_Topo->Reset(3,3);
        // The radius of the main vertical curve
        double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));

        double RE = dN;
        double X = pPosition_X_Z_H->m_dXt;
        double Z = pPosition_X_Z_H->m_dZt;
        double H = pPosition_X_Z_H->m_dYt;
        double Q2 = (RE+H)*(RE+H) - X*X - Z*Z;
        if (Q2 > 0)
        {
            double Q = sqrt(Q2);
            double dYdX = -X / Q;
            double dYdZ = -Z / Q;
            double dYdH = (RE+H) / Q;

            Square_Matrix<3> J(3,3); //Jacobi matrix: |dXdX dXdZ dXdH|
            J.M[0][0] = 1.;          //               |dYdX dYdZ dYdH|
            J.M[2][1] = 1.;          //               |dZdX dZdZ dZdH|
            J.M[1][0] = dYdX;
            J.M[1][1] = dYdZ;
            J.M[1][2] = dYdH;

            Square_Matrix<3> JT(3,3); //J transposed
            J.transp(&JT);
            bOK = J.MatrXMatrXMatr(pCM_X_Z_H, &JT, pCM_Topo);
        }
        else
        {
            bOK = false;
        }
    }
    return bOK;
}


bool Recount_CovMatr_X_Z_H_TOPO_coord_vel(const CGeodesic *pCenter1,
                                          const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H,
                                          Square_Matrix<6> *pCM_X_Z_H, Square_Matrix<6> *pCM_Topo)
{
    bool bOK = true;
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pVel_X_Z_H) ||
            (!pCM_X_Z_H) ||
            (!pCM_Topo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dYt < cd_H_MIN) ||
            (pVel_X_Z_H->m_dYt > cd_H_MAX) ||
            (pVel_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) )
    {
        bOK = false;
    }
    else
    {
        pCM_Topo->Reset(6,6);
        // The radius of the main vertical curve
        double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));

        double RE = dN;
        double X = pPosition_X_Z_H->m_dXt;
        double Z = pPosition_X_Z_H->m_dZt;
        double H = pPosition_X_Z_H->m_dYt;
        double VX = pVel_X_Z_H->m_dXt;
        double VZ = pVel_X_Z_H->m_dZt;
        double VH = pVel_X_Z_H->m_dYt;
        double Q2 = (RE+H)*(RE+H) - X*X - Z*Z;
        if (Q2 > 0)
        {
            double Q = sqrt(Q2);
            double VY = ((RE+H)*VH - X*VX - Z*VZ) / Q;

            double dYdX = -X / Q;
            double dYdZ = -Z / Q;
            double dYdH = (RE+H) / Q;
            double dVYdX = (X*VY/Q - VX) / Q;
            double dVYdZ = (Z*VY/Q - VZ) / Q;
            double dVYdH = (VH - ((RE+H)*VY/Q)) / Q;
            //                   0     1     2     3      4      5
            Square_Matrix<6> J(6,6); //Jacobi matrix: 0 |dXdX  dXdZ  dXdH  dXdVX  dXdVZ  dXdVH |
            J.M[0][0] = 1.;          //               1 |dYdX  dYdZ  dYdH  dYdVX  dYdVZ  dYdVH |
            J.M[2][1] = 1.;          //               2 |dZdX  dZdZ  dZdH  dZdVX  dZdVY  dZdVH |
            J.M[1][0] = dYdX;        //               3 |dVXdX dVXdZ dVXdH dVXdVX dVXdVZ dVXdVH|
            J.M[1][1] = dYdZ;        //               4 |dVYdX dVYdZ dVYdH dVYdVX dVYdVZ dVYdVH|
            J.M[1][2] = dYdH;        //               5 |dVZdX dVZdY dVZdH dVZdVX dVZdVZ dVZdVH|
            J.M[3][3] = 1.;
            J.M[5][4] = 1.;
            J.M[4][0] = dVYdX;
            J.M[4][1] = dVYdZ;
            J.M[4][2] = dVYdH;
            J.M[4][3] = dYdX;
            J.M[4][4] = dYdZ;
            J.M[4][5] = dYdH;

            Square_Matrix<6> JT(6,6); //J transposed
            J.transp(&JT);
            bOK = J.MatrXMatrXMatr(pCM_X_Z_H, &JT, pCM_Topo);
        }
        else
        {
            bOK = false;
        }
    }
    return bOK;
}


bool Recount_CovMatr_X_Z_H_TOPO_coord_vel_acc(const CGeodesic *pCenter1,
                                              const CTopocentric *pPosition_X_Z_H, const CTopocentric *pVel_X_Z_H, const CTopocentric *pAcc_X_Z_H,
                                              Square_Matrix<9> *pCM_X_Z_H, Square_Matrix<9> *pCM_Topo)
{
    bool bOK = true;
    if ((!pCenter1) ||
            (!pPosition_X_Z_H) ||
            (!pVel_X_Z_H) ||
            (!pAcc_X_Z_H) ||
            (!pCM_X_Z_H) ||
            (!pCM_Topo) ||
            (pCenter1->m_dLatitude < cdGEODES_LATITUDE_MIN) ||
            (pCenter1->m_dLatitude > cdGEODES_LATITUDE_MAX) ||
            (pCenter1->m_dLongitude < cdGEODES_LONGITUDE_MIN) ||
            (pCenter1->m_dLongitude > cdGEODES_LONGITUDE_MAX) ||
            (pCenter1->m_dAltitude < cd_H_MIN) ||
            (pCenter1->m_dAltitude > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pPosition_X_Z_H->m_dYt < cd_H_MIN) ||
            (pPosition_X_Z_H->m_dYt > cd_H_MAX) ||
            (pPosition_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pPosition_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pVel_X_Z_H->m_dYt < cd_H_MIN) ||
            (pVel_X_Z_H->m_dYt > cd_H_MAX) ||
            (pVel_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pVel_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) ||
            (pAcc_X_Z_H->m_dXt < cdTOPO_X_Y_Z_MIN) ||
            (pAcc_X_Z_H->m_dXt > cdTOPO_X_Y_Z_MAX) ||
            (pAcc_X_Z_H->m_dYt < cd_H_MIN) ||
            (pAcc_X_Z_H->m_dYt > cd_H_MAX) ||
            (pAcc_X_Z_H->m_dZt < cdTOPO_X_Y_Z_MIN) ||
            (pAcc_X_Z_H->m_dZt > cdTOPO_X_Y_Z_MAX) )
    {
        bOK = false;
    }
    else
    {
        pCM_Topo->Reset(6,6);
        // The radius of the main vertical curve
        double dN = cdEquatorAxis/sqrt(1.0-cdFirstEccentricity*pow(sin(pCenter1->m_dLatitude),2.0));

        double RE = dN;
        double X = pPosition_X_Z_H->m_dXt;
        double Z = pPosition_X_Z_H->m_dZt;
        double H = pPosition_X_Z_H->m_dYt;
        double VX = pVel_X_Z_H->m_dXt;
        double VZ = pVel_X_Z_H->m_dZt;
        double VH = pVel_X_Z_H->m_dYt;
        double AX = pAcc_X_Z_H->m_dXt;
        double AZ = pAcc_X_Z_H->m_dZt;
        double AH = pAcc_X_Z_H->m_dYt;
        double Q2 = (RE+H)*(RE+H) - X*X - Z*Z;
        if (Q2 > 0)
        {
            double Q = sqrt(Q2);
            double VY = ((RE+H)*VH - X*VX - Z*VZ) / Q;
            double AY = ((RE+H)*AH - X*AX - Z*AZ + VH*VH - VX*VX - VZ*VZ - VY*VY) / Q;

            double dYdX = -X / Q;
            double dYdZ = -Z / Q;
            double dYdH = (RE+H) / Q;
            double dVYdX = (X*VY/Q - VX) / Q;
            double dVYdZ = (Z*VY/Q - VZ) / Q;
            double dVYdH = (VH - ((RE+H)*VY/Q)) / Q;
            double dAYdX = (-AX - 2.*VY*dVYdX)/Q + (AY*X/Q2);
            double dAYdZ = (-AZ - 2.*VY*dVYdZ)/Q + (AY*Z/Q2);
            double dAYdH = ( AH - 2.*VY*dVYdH)/Q - (AY*(RE+H)/Q2);
            //                   0     1     2     3      4      5      6      7      8
            Square_Matrix<9> J(9,9); //Jacobi matrix: 0 |dXdX  dXdZ  dXdH  dXdVX  dXdVZ  dXdVH  dXdAX  dXdAZ  dXdAH |
            J.M[0][0] = 1.;          //               1 |dYdX  dYdZ  dYdH  dYdVX  dYdVZ  dYdVH  dYdAX  dYdAZ  dYdAH |
            J.M[2][1] = 1.;          //               2 |dZdX  dZdZ  dZdH  dZdVX  dZdVY  dZdVH  dZdAX  dZdAZ  dZdAH |
            J.M[1][0] = dYdX;        //               3 |dVXdX dVXdZ dVXdH dVXdVX dVXdVZ dVXdVH dVXdAX dVXdAZ dVXdAH|
            J.M[1][1] = dYdZ;        //               4 |dVYdX dVYdZ dVYdH dVYdVX dVYdVZ dVYdVH dVYdAX dVYdAZ dVYdAH|
            J.M[1][2] = dYdH;        //               5 |dVZdX dVZdY dVZdH dVZdVX dVZdVZ dVZdVH dVZdAX dVZdAZ dVZdAH|
            J.M[3][3] = 1.;          //               6 |dAXdX dAXdZ dAXdH dAXdVX dAXdVZ dAXdVH dAXdAX dAXdAZ dAXdAH|
            J.M[5][4] = 1.;          //               7 |dAYdX dAYdZ dAYdH dAYdVX dAYdVZ dAYdVH dAYdAX dAYdAZ dAYdAH|
            J.M[4][0] = dVYdX;       //               8 |dAZdX dAZdZ dAZdH dAZdVX dAZdVZ dAZdVH dAZdAX dAZdAZ dAZdAH|
            J.M[4][1] = dVYdZ;
            J.M[4][2] = dVYdH;
            J.M[4][3] = dYdX;
            J.M[4][4] = dYdZ;
            J.M[4][5] = dYdH;
            J.M[6][6] = 1.;
            J.M[8][7] = 1.;
            J.M[7][0] = dAYdX;
            J.M[7][1] = dAYdZ;
            J.M[7][2] = dAYdH;
            J.M[7][3] = 2.*dVYdX;
            J.M[7][4] = 2.*dVYdZ;
            J.M[7][5] = 2.*dVYdH;
            J.M[7][6] = dYdX;
            J.M[7][7] = dYdZ;
            J.M[7][8] = dYdH;

            Square_Matrix<9> JT(9,9); //J transposed
            J.transp(&JT);
            bOK = J.MatrXMatrXMatr(pCM_X_Z_H, &JT, pCM_Topo);
        }
        else
        {
            bOK = false;
        }
    }
    return bOK;
}
