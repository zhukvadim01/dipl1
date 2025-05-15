//
//FILE			:rip_tc_prog_act.cpp
//
//AUTHOR		:tania
//

#include "CBallist_ActiveProgn.h"
#include "air_ballist_literal.h"
#include "conversion/Geocentric_Geo.h"
#include "conversion/Geocentric_Topo.h"
#include "conversion/Spherical_Topo.h"
#include "gl/GLConvertMatrices.h"

#include<math.h>
#include<vector>

CBallist_ActiveProgn::CBallist_ActiveProgn()
{
    m_pAPrInput = nullptr;
}

CBallist_ActiveProgn::~CBallist_ActiveProgn()
{
}


void CBallist_ActiveProgn::InitConst(const Pr_Act_Const &cnst)
{
	m_APrConst = cnst;
}


void CBallist_ActiveProgn::InitInput(Pr_Act_Input &inp)
{
    m_pAPrInput = &inp;
}


void CBallist_ActiveProgn::GetActiveProgn(Pr_Act_Out	&Out)
{    
    Out.Reset();
    const qreal dt_est = m_pAPrInput->tL - m_pAPrInput->tLastActProgn;
    const qreal modSP = pow(m_pAPrInput->EstStartPoint0.x,2) + pow(m_pAPrInput->EstStartPoint0.y,2) + pow(m_pAPrInput->EstStartPoint0.z,2);
    if (dt_est < m_APrConst.TPeriod && modSP > con_par_eps && !m_pAPrInput->ForcedActPr)
	{        
        Out.ActPrInfo.New_ActPrognoz = AIR_BALL::PREVIOUS_ACT_PROGN;
		return;
	}

    Out.ActPrInfo.New_ActPrognoz = AIR_BALL::NEW_ACT_PROGN;
    Out.ActPrInfo.Trapeze.AzFly = 0;
    Out.ActPrInfo.Trapeze.SigmaAzFly = c360;
    Out.ActPrInfo.Trapeze.SStrAzFly = c360;
    Out.ActPrInfo.Trapeze.MaxRange = m_APrConst.MaxumumOfRange;
    Out.ActPrInfo.Trapeze.MinRange = m_APrConst.MinimumOfRange;
    Out.ActPrInfo.EstStartPoint.x = 0;
    Out.ActPrInfo.EstStartPoint.y = 0;
    Out.ActPrInfo.EstStartPoint.z = 0;
    Out.ActPrInfo.EstFallPoint.x = 0;
    Out.ActPrInfo.EstFallPoint.y = 0;
    Out.ActPrInfo.EstFallPoint.z = 0;
    Out.ActPrInfo.TimeStartEst = 0;
    Out.ActPrInfo.NumsThreat.clear();
    //Out.ActPrInfo.t_calc_act = m_pAPrInput->tL;

    CGeodesic GCenter;
    CGeodesic GCenterSP;
    CGeocentric PointW;
    CGeocentric CurPointW(m_pAPrInput->X, m_pAPrInput->Y, m_pAPrInput->Z);
    CGeocentric SpeedW;
    CGeocentric AccelW;
    CGeocentric CurVelW(m_pAPrInput->VX, m_pAPrInput->VY, m_pAPrInput->VZ);
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

    GCenter = m_pAPrInput->Center;

    bool bRecountOK = GEOCENTRIC_TOPO(&GCenter, &CurPointW, &CurVelW, 0, &CurPointT, &CurVelT, 0);
	
    bool P_first_SP_calc = false; //sign of the first correctly calculated start point
    qreal t;
	
    if (modSP > con_par_eps)
    {
        Out.ActPrInfo.EstStartPoint = m_pAPrInput->EstStartPoint0;
        Out.ActPrInfo.TimeStartEst = m_pAPrInput->TimeStartEst0;
    }
	else
	{
        if (m_pAPrInput->Vh < 0)
        {
			return;
        }

        bool bStartOK = StartTimeEstimation(CurPointT, CurVelT, t, P_first_SP_calc);
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
        Out.ActPrInfo.TimeStartEst = m_pAPrInput->tL + t;
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
        PointW.m_dX = m_pAPrInput->EstStartPoint0.x;
        PointW.m_dY = m_pAPrInput->EstStartPoint0.y;
        PointW.m_dZ = m_pAPrInput->EstStartPoint0.z;
        Out.ActPrInfo.TimeStartEst = m_pAPrInput->TimeStartEst0;
	}
	
    Out.ActPrInfo.EstStartPoint.x = PointW.m_dX;
    Out.ActPrInfo.EstStartPoint.y = PointW.m_dY;
    Out.ActPrInfo.EstStartPoint.z = PointW.m_dZ;

    Out.ActPrInfo.Trapeze.P_base = Out.ActPrInfo.EstStartPoint;
		
    //-----Velocity covariance matrix in WGS-----
    MSigmaV_WGS.M[0][0]	= pow(m_pAPrInput->SigVX,2);
    MSigmaV_WGS.M[1][1]	= pow(m_pAPrInput->SigVY,2);
    MSigmaV_WGS.M[2][2]	= pow(m_pAPrInput->SigVZ,2);
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
        Out.ActPrInfo.Trapeze.AzFly = Spher.m_dB * con_180_div_PI;

        bool bAzIncreased = false; //true if azimuth is increased
        //-----Calculation of Azimuth RMSE-----
        Out.ActPrInfo.Trapeze.SigmaAzFly = ( ( sqrt( pow(SpeedTSP.m_dZt,2)*pow(SigmaV_TSP.x,2) + pow(SpeedTSP.m_dXt,2)*pow(SigmaV_TSP.z,2) ) ) / ( pow(SpeedTSP.m_dXt,2) + pow(SpeedTSP.m_dZt,2) ) ) * 180/con_pi ;
        if (m_pAPrInput->SigAzPrev > con_par_eps && Out.ActPrInfo.Trapeze.SigmaAzFly > m_pAPrInput->SigAzPrev)
        {       //correction of Azimuth RMSE
            //        Out.ActPrInfo.Trapeze.SigmaAzFly = cKExpSmSigAz * Out.ActPrInfo.Trapeze.SigmaAzFly
            //                + (AIR_BALL::c1 - cKExpSmSigAz) * m_pAPrInput->SigAzPrev;
            bAzIncreased = true;
        }
        Out.ActPrInfo.Trapeze.SStrAzFly = m_APrConst.KSigmaAz * Out.ActPrInfo.Trapeze.SigmaAzFly + m_APrConst.DeltaAzCorr;

        //-----Calculation of Flying Range-----
        if (m_pAPrInput->Pbs == AIR_BALL::ACTIVE_PATH)
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
                if (m_pAPrInput->H > a_LHV[i][1] + AIR_BALL::K_Sigma * m_pAPrInput->SigH
                        && m_pAPrInput->V > a_LHV[i][2] + AIR_BALL::K_Sigma * m_pAPrInput->SigV)
                {
                    Out.ActPrInfo.Trapeze.MinRange = c1000 * a_LHV[i][0] * 0.4;
                    p_break = true;
                    break;
                }
            }
            if (!p_break)
            {
                if (m_pAPrInput->H > a_LHV[0][1] + AIR_BALL::K_Sigma * m_pAPrInput->SigH
                        || m_pAPrInput->V > a_LHV[0][2] + AIR_BALL::K_Sigma * m_pAPrInput->SigV)
                {
                    Out.ActPrInfo.Trapeze.MinRange = c1000*(a_LHV[0][0])*0.2;
                }
            }
            //		}
        }

        //minimum range by distance to the start point
        const qreal d2start = sqrt(pow(CurPointW.m_dX - Out.ActPrInfo.EstStartPoint.x, 2) +
                                       pow(CurPointW.m_dY - Out.ActPrInfo.EstStartPoint.y, 2) +
                                       pow(CurPointW.m_dZ - Out.ActPrInfo.EstStartPoint.z, 2) );

        Out.ActPrInfo.Trapeze.MinRange = std::max(Out.ActPrInfo.Trapeze.MinRange, d2start + m_APrConst.MinimumOfRange);

        //minimum range by calculation of the drop point based on the present position and velocity
        qreal Vmin, VHmin, Hmin; //minimum admissible values of V, VH, H
        Vmin = m_pAPrInput->V - AIR_BALL::K_Sigma * m_pAPrInput->SigV;
        VHmin = m_pAPrInput->Vh - AIR_BALL::K_Sigma * m_pAPrInput->SigVH;
        Hmin = m_pAPrInput->H - AIR_BALL::K_Sigma * m_pAPrInput->SigH;
        const qreal VL = sqrt(pow(Vmin, 2) - pow(VHmin, 2)); //horizontal velocity

        const qreal tf = ( VHmin + sqrt( pow(VHmin, 2) + 2. * con_g * Hmin ) ) / con_g; //fall time
        if (tf > con_par_eps)
        {
            qreal k_braking;
            if (m_pAPrInput->H < cHboundBraking[0]) //30000
            {
                k_braking = cCoeffBraking[0]; //0.5
            }
            else if (m_pAPrInput->H < cHboundBraking[1]) //40000
            {
                k_braking = cCoeffBraking[1]; //0.7
            }
            else
            {
                k_braking = cCoeffBraking[2]; //0.9
            }

            const qreal L = d2start + k_braking*VL*tf; //estimated range
            Out.ActPrInfo.Trapeze.MinRange = std::max(Out.ActPrInfo.Trapeze.MinRange, L);
        }

        OutputParamClarification(bAzIncreased, Out);
    }
    else
    {
        Out.ActPrInfo.New_ActPrognoz = AIR_BALL::UNRELIABLE_ACT_PROGN;
    }
    Out.P_Changed = true;
}


bool CBallist_ActiveProgn::StartTimeEstimation(const CTopocentric &CurPointT, const CTopocentric &CurVelT,
                                               qreal &t_res, bool &P_first_SP_calc)
{
    bool bRes = false;
    P_first_SP_calc = false;

    const qreal a = pow(CurVelT.m_dXt,2)+pow(CurVelT.m_dYt,2)+pow(CurVelT.m_dZt,2);
    const qreal b = 2*(CurPointT.m_dXt * CurVelT.m_dXt + (CurPointT.m_dYt + AIR_BALL::RZ + m_pAPrInput->Center.m_dAltitude) * CurVelT.m_dYt +
           CurPointT.m_dZt * CurVelT.m_dZt);
    const qreal c = pow(CurPointT.m_dXt,2) + pow(CurPointT.m_dYt + AIR_BALL::RZ + m_pAPrInput->Center.m_dAltitude,2) +
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


void CBallist_ActiveProgn::OutputParamClarification(bool bAzIncreased, Pr_Act_Out &Out)
{
    if (m_pAPrInput->p_arCovObj != nullptr) //limitation of maximum range using distance to covered objects
    {
        qreal DMaxCovObj = - con_large_value;
        GLTrapeze ExtTrapeze = Out.ActPrInfo.Trapeze; //extended trapeze for comparison
        ExtTrapeze.MaxRange = m_APrConst.MaximumOfRangeExt;

        for (qint32 i=0; i<m_pAPrInput->p_arCovObj->N_obj; i++)
        {
            AIR_BALL::sCovObj *pCurCov = &m_pAPrInput->p_arCovObj->arCovObj[i];
            CGeocentric CurCenterGC;
            GEO_GEOCENTRIC(&pCurCov->CoordGeodez, &CurCenterGC);
            GLPointDouble3D CurCenter3D(CurCenterGC.m_dX, CurCenterGC.m_dY, CurCenterGC.m_dZ);

            if (ExtTrapeze.IntersectsWithCircle(CurCenter3D, pCurCov->Radius))
            {
                Out.ActPrInfo.NumsThreat.insert(pCurCov->Num);

                qreal dCurr = Out.ActPrInfo.Trapeze.P_base.getDistance(CurCenter3D)
                        + pCurCov->Radius;
                DMaxCovObj = std::max(DMaxCovObj, dCurr);
            }
        }

        if (DMaxCovObj > con_par_eps)
        {
            Out.ActPrInfo.Trapeze.MaxRange = std::max(cCoefDMax * DMaxCovObj, m_APrConst.MaxumumOfRange);
        }
    }

    if (Out.ActPrInfo.Trapeze.MinRange > m_APrConst.MaxumumOfRange)
    {
        Out.ActPrInfo.Trapeze.MaxRange = m_APrConst.MaximumOfRangeExt;
    }

    if (Out.ActPrInfo.Trapeze.MinRange > Out.ActPrInfo.Trapeze.MaxRange)
    {
        Out.ActPrInfo.Trapeze.MaxRange = cCoefDMinMax * Out.ActPrInfo.Trapeze.MinRange;
    }

    if(Out.ActPrInfo.Trapeze.SigmaAzFly > cConf_Angle || Out.ActPrInfo.EstStartPoint.module() < con_par_eps
            || Out.ActPrInfo.Trapeze.MinRange < con_par_eps || Out.ActPrInfo.Trapeze.MaxRange < con_par_eps
            || bAzIncreased)
    {
        Out.ActPrInfo.New_ActPrognoz = AIR_BALL::UNRELIABLE_ACT_PROGN;
    }
}
