//
//FILE			:rip_tc_prog_act.cpp
//
//AUTHOR		:tania
//

#include "CBallist_ActiveProgn_M.h"
#include "air_ballist_literal.h"
#include "conversion/Geocentric_Geo.h"
#include "conversion/Geocentric_Topo.h"
#include "conversion/Spherical_Topo.h"
#include "gl/GLConvertMatrices.h"

#include<math.h>
#include<vector>

CBallist_ActiveProgn_M::CBallist_ActiveProgn_M()
{
//    m_pAPrInput = nullptr;
}

CBallist_ActiveProgn_M::~CBallist_ActiveProgn_M()
{
}


void CBallist_ActiveProgn_M::InitConst(const Pr_Act_Const &cnst)
{
	m_APrConst = cnst;
}


void CBallist_ActiveProgn_M::GetActiveProgn(const Pr_Act_Input* pInData,
                                            Pr_Act_Out	&rOutData)
{    
    rOutData.Reset();
    const qreal dt_est = pInData->tL - pInData->tLastActProgn;
    const qreal modSP = pow(pInData->EstStartPoint0.x,2) + pow(pInData->EstStartPoint0.y,2) + pow(pInData->EstStartPoint0.z,2);
    if (dt_est < m_APrConst.TPeriod && modSP > con_par_eps && !pInData->ForcedActPr)
	{        
        rOutData.ActPrInfo.New_ActPrognoz = AIR_BALL::PREVIOUS_ACT_PROGN;
		return;
	}

    rOutData.ActPrInfo.New_ActPrognoz = AIR_BALL::NEW_ACT_PROGN;
    rOutData.ActPrInfo.Trapeze.AzFly = 0;
    rOutData.ActPrInfo.Trapeze.SigmaAzFly = c360;
    rOutData.ActPrInfo.Trapeze.SStrAzFly = c360;
    rOutData.ActPrInfo.Trapeze.MaxRange = m_APrConst.MaxumumOfRange;
    rOutData.ActPrInfo.Trapeze.MinRange = m_APrConst.MinimumOfRange;
    rOutData.ActPrInfo.EstStartPoint.x = 0;
    rOutData.ActPrInfo.EstStartPoint.y = 0;
    rOutData.ActPrInfo.EstStartPoint.z = 0;
    rOutData.ActPrInfo.EstFallPoint.x = 0;
    rOutData.ActPrInfo.EstFallPoint.y = 0;
    rOutData.ActPrInfo.EstFallPoint.z = 0;
    rOutData.ActPrInfo.TimeStartEst = 0;
    rOutData.ActPrInfo.NumsThreat.clear();
    //Out.ActPrInfo.t_calc_act = m_pAPrInput->tL;

    CGeodesic GCenter;
    CGeodesic GCenterSP;
    CGeocentric PointW;
    CGeocentric CurPointW(pInData->X, pInData->Y, pInData->Z);
    CGeocentric SpeedW;
    CGeocentric AccelW;
    CGeocentric CurVelW(pInData->VX, pInData->VY, pInData->VZ);
    CTopocentric PointT;
    CTopocentric PointTSP;
    CTopocentric CurPointT;
    CTopocentric CurVelT;
    CTopocentric SpeedT;
    CTopocentric SpeedTSP;
    CTopocentric AccelT;
    CTopocentric AccelTSP;
    //TMatrix<3>	Agm(3,3);
    //TMatrix<3> AgmT(3,3);
//    TMatrix<3> MSigmaV_Topo(3,3);
    TMatrix<3> MSigmaV_WGS(3,3); //M1,
    TMatrix<3> MSigmaV_TopoSP(3,3);
    CGeocentric CenterSP_WGS;
    CGeodesic CenterSP_GEO;

    GCenter = pInData->Center;

    bool bRecountOK = GEOCENTRIC_TOPO(&GCenter, &CurPointW, &CurVelW, 0, &CurPointT, &CurVelT, 0);
	
    bool P_first_SP_calc = false; //sign of the first correctly calculated start point
    qreal t;
	
    if (modSP > con_par_eps)
    {
        rOutData.ActPrInfo.EstStartPoint = pInData->EstStartPoint0;
        rOutData.ActPrInfo.TimeStartEst = pInData->TimeStartEst0;
    }
	else
	{
        if (pInData->Vh < 0) {
			return;
        }

        bool bStartOK = StartTimeEstimation(pInData, CurPointT, CurVelT, t, P_first_SP_calc);
        if (!bStartOK)
        {
            return;
        }
	}
					
	if (P_first_SP_calc)
	{	
        PointT.m_dXt = CurPointT.m_dXt + CurVelT.m_dXt * t;
        PointT.m_dYt = CurPointT.m_dYt + CurVelT.m_dYt * t;
        PointT.m_dZt = CurPointT.m_dZt + CurVelT.m_dZt * t;
        rOutData.ActPrInfo.TimeStartEst = pInData->tL + t;
	}
	else
	{
        PointT.m_dXt = 0;
        PointT.m_dYt = 0;
        PointT.m_dZt = 0;
	}

    SpeedT.m_dXt = CurVelT.m_dXt;
    SpeedT.m_dYt = CurVelT.m_dYt;
    SpeedT.m_dZt = CurVelT.m_dZt;
			
    AccelT.m_dXt = 0;
    AccelT.m_dYt = 0;
    AccelT.m_dZt = 0;
				
	//-----Recalculation of estimated Start Point and Velocity from TPSC to WGS-----
    bRecountOK = bRecountOK && TOPO_GEOCENTRIC(&GCenter, &PointT, &SpeedT, &AccelT, &PointW, &SpeedW, &AccelW);

	//-----Recalculation of current point from TPSC to WGS-----	

	if (!P_first_SP_calc)
	{
        PointW.m_dX = pInData->EstStartPoint0.x;
        PointW.m_dY = pInData->EstStartPoint0.y;
        PointW.m_dZ = pInData->EstStartPoint0.z;
        rOutData.ActPrInfo.TimeStartEst = pInData->TimeStartEst0;
	}
	
    rOutData.ActPrInfo.EstStartPoint.x = PointW.m_dX;
    rOutData.ActPrInfo.EstStartPoint.y = PointW.m_dY;
    rOutData.ActPrInfo.EstStartPoint.z = PointW.m_dZ;

    rOutData.ActPrInfo.Trapeze.P_base = rOutData.ActPrInfo.EstStartPoint;
		
    //-----Velocity covariance matrix in WGS-----
    MSigmaV_WGS.M[0][0]	= pow(pInData->SigVX,2);
    MSigmaV_WGS.M[1][1]	= pow(pInData->SigVY,2);
    MSigmaV_WGS.M[2][2]	= pow(pInData->SigVZ,2);
    MSigmaV_WGS.M[0][1]	= 0;
    MSigmaV_WGS.M[1][0]	= 0;
    MSigmaV_WGS.M[0][2]	= 0;
    MSigmaV_WGS.M[2][0]	= 0;
    MSigmaV_WGS.M[2][1]	= 0;
    MSigmaV_WGS.M[1][2]	= 0;
				
//    GL_MATR::Recount_CovMatr_TopoToGeocentric(&GCenter, &MSigmaV_Topo, &MSigmaV_WGS);
				
	//------Recalculation of Start Point from WGS to GEO (degrees)------
    CenterSP_WGS = PointW;
    bRecountOK = bRecountOK && GEOCENTRIC_GEO(&CenterSP_WGS, &CenterSP_GEO);

    GCenterSP = CenterSP_GEO;
				
	//-----Recalculation of Velocity from WGS to TPSC with center in Start Point-----
    bRecountOK = bRecountOK && GEOCENTRIC_TOPO(&GCenterSP, &PointW, &SpeedW, &AccelW, &PointTSP, &SpeedTSP, &AccelTSP);

	//-----Velocity RMSE recalculation from WGS to TPSC with center in Start Point----- 			
    bRecountOK = bRecountOK && GL_MATR::Recount_CovMatr_GeocentricToTopo(&GCenterSP, &MSigmaV_WGS, &MSigmaV_TopoSP);
				
    GLPointDouble3D	SigmaV_TSP;
    SigmaV_TSP.x = sqrt(MSigmaV_TopoSP.M[0][0]);
    SigmaV_TSP.y = sqrt(MSigmaV_TopoSP.M[1][1]);
    SigmaV_TSP.z = sqrt(MSigmaV_TopoSP.M[2][2]);
				
	//-----Calculation of Azimuth of direction-----
    CSpherical Spher;
    bRecountOK = TOPO_SPHERICAL(&SpeedTSP, &Spher);
    if (bRecountOK)
    {
        rOutData.ActPrInfo.Trapeze.AzFly = Spher.m_dB * con_180_div_PI;

        bool bAzIncreased = false; //true if azimuth is increased
        //-----Calculation of Azimuth RMSE-----
        rOutData.ActPrInfo.Trapeze.SigmaAzFly = ( ( sqrt( pow(SpeedTSP.m_dZt,2)*pow(SigmaV_TSP.x,2) + pow(SpeedTSP.m_dXt,2)*pow(SigmaV_TSP.z,2) ) ) / ( pow(SpeedTSP.m_dXt,2) + pow(SpeedTSP.m_dZt,2) ) ) * 180/con_pi ;
        if (pInData->SigAzPrev > con_par_eps && rOutData.ActPrInfo.Trapeze.SigmaAzFly > pInData->SigAzPrev)
        {
            //correction of Azimuth RMSE
            //        Out.ActPrInfo.Trapeze.SigmaAzFly = cKExpSmSigAz * Out.ActPrInfo.Trapeze.SigmaAzFly
            //                + (AIR_BALL::c1 - cKExpSmSigAz) * m_pAPrInput->SigAzPrev;
            bAzIncreased = true;
        }
        rOutData.ActPrInfo.Trapeze.SStrAzFly = m_APrConst.KSigmaAz * rOutData.ActPrInfo.Trapeze.SigmaAzFly +
                m_APrConst.DeltaAzCorr;

        //-----Calculation of Flying Range-----
        if (pInData->Pbs == AIR_BALL::ACTIVE_PATH)
        {	//minimum range by table of H and V
            //        if (m_pAPrInput->H > a_LHV[11][1] || m_pAPrInput->V > a_LHV[11][2])
            //        {
            //            Out.ActPrInfo.Trapeze.MinRange = c1000 * (a_LHV[11][0]) * 0.4;
            //        }
            //		else
            //		{
            bool p_break = false;
            for (qint16 i=21; i>0; i--)
            {
                if (pInData->H > a_LHV[i][1] + AIR_BALL::K_Sigma * pInData->SigH
                        && pInData->V > a_LHV[i][2] + AIR_BALL::K_Sigma * pInData->SigV)
                {
                    rOutData.ActPrInfo.Trapeze.MinRange = c1000 * a_LHV[i][0] * 0.4;
                    p_break = true;
                    break;
                }
            }
            if (!p_break)
            {
                if (pInData->H > a_LHV[0][1] + AIR_BALL::K_Sigma * pInData->SigH
                        || pInData->V > a_LHV[0][2] + AIR_BALL::K_Sigma * pInData->SigV)
                {
                    rOutData.ActPrInfo.Trapeze.MinRange = c1000*(a_LHV[0][0])*0.2;
                }
            }
            //		}
        }

        //minimum range by distance to the start point
        const qreal d2start = sqrt(pow(CurPointW.m_dX - rOutData.ActPrInfo.EstStartPoint.x, 2) +
                                       pow(CurPointW.m_dY - rOutData.ActPrInfo.EstStartPoint.y, 2) +
                                       pow(CurPointW.m_dZ - rOutData.ActPrInfo.EstStartPoint.z, 2) );

        rOutData.ActPrInfo.Trapeze.MinRange = std::max(rOutData.ActPrInfo.Trapeze.MinRange,
                                                       d2start + m_APrConst.MinimumOfRange);

        //minimum range by calculation of the drop point based on the present position and velocity
        qreal Vmin, VHmin, Hmin; //minimum admissible values of V, VH, H
        Vmin = pInData->V - AIR_BALL::K_Sigma * pInData->SigV;
        VHmin = pInData->Vh - AIR_BALL::K_Sigma * pInData->SigVH;
        Hmin = pInData->H - AIR_BALL::K_Sigma * pInData->SigH;
        const qreal VL = sqrt(pow(Vmin, 2) - pow(VHmin, 2)); //horizontal velocity

        const qreal tf = ( VHmin + sqrt( pow(VHmin, 2) + 2. * con_g * Hmin ) ) / con_g; //fall time
        if (tf > con_par_eps)
        {
            qreal k_braking;
            if (pInData->H < cHboundBraking[0]) //30000
            {
                k_braking = cCoeffBraking[0]; //0.5
            }
            else if (pInData->H < cHboundBraking[1]) //40000
            {
                k_braking = cCoeffBraking[1]; //0.7
            }
            else
            {
                k_braking = cCoeffBraking[2]; //0.9
            }

            const qreal L = d2start + k_braking*VL*tf; //estimated range
            rOutData.ActPrInfo.Trapeze.MinRange = std::max(rOutData.ActPrInfo.Trapeze.MinRange, L);
        }

        OutputParamClarification(pInData, bAzIncreased, rOutData);
    }
    else
    {
        rOutData.ActPrInfo.New_ActPrognoz = AIR_BALL::UNRELIABLE_ACT_PROGN;
    }
    rOutData.P_Changed = true;
}


bool CBallist_ActiveProgn_M::StartTimeEstimation(const Pr_Act_Input* pInData,
                                                 const CTopocentric &CurPointT, const CTopocentric &CurVelT,
                                                 qreal &t_res, bool &P_first_SP_calc)
{
    bool bRes = false;
    P_first_SP_calc = false;

    const qreal a = pow(CurVelT.m_dXt,2)+pow(CurVelT.m_dYt,2)+pow(CurVelT.m_dZt,2);
    const qreal b = 2*(CurPointT.m_dXt * CurVelT.m_dXt + (CurPointT.m_dYt + AIR_BALL::RZ +
                                                          pInData->Center.m_dAltitude) * CurVelT.m_dYt +
                       CurPointT.m_dZt * CurVelT.m_dZt);

    const qreal c = pow(CurPointT.m_dXt,2) + pow(CurPointT.m_dYt + AIR_BALL::RZ + pInData->Center.m_dAltitude,2) +
            pow(CurPointT.m_dZt,2) - pow(AIR_BALL::RZ,2);

    const qreal D = b*b - 4*a*c;

    if (D > con_par_eps)
    {
        const qreal t1 = (-b + sqrt(D))/(2*a);
        const qreal t2 = (-b - sqrt(D))/(2*a);
        if (std::min(t1, t2) < 0)
        {
            if (t1 < 0)
            {
                t_res = t1;
            }
            else
            {
                t_res = t2;
            }
            bRes = true;
            P_first_SP_calc = true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    return bRes;
}


void CBallist_ActiveProgn_M::OutputParamClarification(const Pr_Act_Input *pInData,
                                                      bool bAzIncreased,
                                                      Pr_Act_Out	&rOutData)
{
    if (pInData->p_arCovObj != nullptr) //limitation of maximum range using distance to covered objects
    {
        qreal DMaxCovObj = - con_large_value;
        GLTrapeze ExtTrapeze = rOutData.ActPrInfo.Trapeze; //extended trapeze for comparison
        ExtTrapeze.MaxRange = m_APrConst.MaximumOfRangeExt;

        for (qint32 i=0; i<pInData->p_arCovObj->N_obj; i++)
        {
            AIR_BALL::sCovObj *pCurCov = &pInData->p_arCovObj->arCovObj[i];
            CGeocentric CurCenterGC;
            GEO_GEOCENTRIC(&pCurCov->CoordGeodez, &CurCenterGC);
            GLPointDouble3D CurCenter3D(CurCenterGC.m_dX, CurCenterGC.m_dY, CurCenterGC.m_dZ);

            if (ExtTrapeze.IntersectsWithCircle(CurCenter3D, pCurCov->Radius))
            {
                rOutData.ActPrInfo.NumsThreat.insert(pCurCov->Num);

                qreal dCurr = rOutData.ActPrInfo.Trapeze.P_base.getDistance(CurCenter3D)
                        + pCurCov->Radius;
                DMaxCovObj = std::max(DMaxCovObj, dCurr);
            }
        }

        if (DMaxCovObj > con_par_eps)
        {
            rOutData.ActPrInfo.Trapeze.MaxRange = std::max(cCoefDMax * DMaxCovObj, m_APrConst.MaxumumOfRange);
        }
    }

    if (rOutData.ActPrInfo.Trapeze.MinRange > m_APrConst.MaxumumOfRange)
    {
        rOutData.ActPrInfo.Trapeze.MaxRange = m_APrConst.MaximumOfRangeExt;
    }

    if (rOutData.ActPrInfo.Trapeze.MinRange > rOutData.ActPrInfo.Trapeze.MaxRange)
    {
        rOutData.ActPrInfo.Trapeze.MaxRange = cCoefDMinMax * rOutData.ActPrInfo.Trapeze.MinRange;
    }

    if(rOutData.ActPrInfo.Trapeze.SigmaAzFly > cConf_Angle || rOutData.ActPrInfo.EstStartPoint.module() < con_par_eps
            || rOutData.ActPrInfo.Trapeze.MinRange < con_par_eps || rOutData.ActPrInfo.Trapeze.MaxRange < con_par_eps
            || bAzIncreased)
    {
        rOutData.ActPrInfo.New_ActPrognoz = AIR_BALL::UNRELIABLE_ACT_PROGN;
    }
}
