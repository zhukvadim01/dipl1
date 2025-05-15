//
//FILE			:rip_tc_prognoz.cpp
//
//AUTHOR		:tatka,	13 May 2006
//

#include "ball_one_prognoz.h"
#include "air_ballist_literal.h"

#include "gl/GLGeometry.h"
#include "gl/GLConvertMatrices.h"
#include "gl/GLFileLog.h"
#include "gl/Constants.h"

#include "conversion/Geocentric_Topo.h"
#include "conversion/Geocentric_Geo.h"
#include "conversion/Topo_Topographic.h"

#include<string>
#include<iomanip>
#include<fstream>
#include<ios>
#include<iostream>
#include <limits>

// using namespace std;
using namespace GLFileLog;

namespace AIR_BALL{
    static std::ofstream out_param(qPrintable(QString("%1/progn_traj.txt").arg(log_path)));
}


COnePrognosis::COnePrognosis()
{
//    Reset();
    std::vector<M10>().swap(Param);
    Param.reserve(1);
    std::vector<qreal>().swap(VectH);
    memset(&RK, 0, sizeof(RK));
    memset(&F, 0, sizeof(F));
    memset(&UK, 0, sizeof(UK));
}


COnePrognosis::~COnePrognosis()
{}


void COnePrognosis::Reset()
{   
    std::vector<M10>().swap(Param);
    Param.reserve(1);
    std::vector<qreal>().swap(VectH);

    dim = 0;
    memset(&RK, 0, sizeof(RK));
    memset(&F, 0, sizeof(F));
    memset(&UK, 0, sizeof(UK));
    AK = 0;
    BK = 0;
    CK = 0;
    H0 = 0;
    Hmax = 0;
    tApogee = AIR_BALL::Timer_INI;
    VK = 0;
    RoK = 0;
    T = 0;
    Step = 0;
    CalcStartPoint.clear();
    Tpass=0;
    SignCorrectProl = true;
    PolynomComb.Reset();
    PolynomCombNUE.Reset();
    m_pLogEll = nullptr;

}


//FUNCTION		:COnePrognosis::InitConst
//
//DESCRIPTION	:Initialization of constants
//
//INPUTS		:Reference to structure for constants space
//
//RETURNS		:void
//
void COnePrognosis::InitConst(const One_Pr_Const &cnst)
{
	m_PrConst = cnst;
}


//function of prolonged track calculation
void COnePrognosis::CalcTrack(const qreal t0,const qreal x0,const qreal y0,const qreal z0,const qreal v0x,const qreal v0y,const qreal v0z,const qreal gamma,const bool P_Inverse)
{
	this->UK[0] = v0x;
	this->UK[1] = v0y;
	this->UK[2] = v0z;
	this->UK[3] = x0;
	this->UK[4] = y0;
	this->UK[5] = z0;

	Step = m_PrConst.step;

	RoK = GetDensity(x0,y0,z0);
	VK = GetAbsVector(v0x,v0y,v0z);
	H0 = GetHeight(x0,y0,z0);
    if (H0 > Hmax)
    {
        Hmax = H0;
        tApogee = t0;
    }
	
	RecalcSystemCoefs(x0,y0,z0,CK,BK,AK);
	T = t0;
	dim = 0;

	if (!P_Inverse)
    {
        F_Diff_Eq(UK,AK,BK,CK,RoK,VK,gamma);
    }
	else
    {
        F_Diff_Eq_Inverse(UK,AK,BK,CK,RoK,VK,gamma);
    }
	M10 M;

	M.M_info[0] = t0;
	M.M_info[1] = x0;
	M.M_info[2] = y0;
	M.M_info[3] = z0;
	M.M_info[4] = v0x;
	M.M_info[5] = v0y;
	M.M_info[6] = v0z;
	M.M_info[7] = F[0];
	M.M_info[8] = F[1];
	M.M_info[9] = F[2];

	Param.push_back(M);
    VectH.push_back(H0);
	qint16 detector = 0;

    qint32 counter1=0, counter2=0;
    SignCorrectProl = true;
    while(H0 > m_PrConst.h0 && T - t0 <= m_PrConst.Tcontr && Step > m_PrConst.min_step && counter1 < 3 * AIR_BALL::MaxCycleIter)
	{
		detector = 1;
        counter1++;
        Step = m_PrConst.step;
        counter2 = 0;
        while(detector == 1 && Step > m_PrConst.min_step && counter2 < AIR_BALL::MaxCycleIter)
		{
			detector = Solution(P_Inverse, gamma);
            counter2++;
		}
	}

    if (T-t0 - m_PrConst.Tcontr >  -con_eps
            || (Step - m_PrConst.min_step <  con_par_eps && !P_Inverse))
    {
        SignCorrectProl = false;
        return;
    }

	if(dim == 0)
    {
		return;
    }
	dim--;
	Param.pop_back();
    VectH.pop_back();

    qreal H_comp = GetHeight(Param[dim].M_info[1], Param[dim].M_info[2], Param[dim].M_info[3]);

    T = T - Step;
    Step = Step/2;//*******
    qreal controlStepValue = Step;//*******
    bool p_continue = true;
    counter1 = 0;
    while (p_continue
           && counter1 < AIR_BALL::MaxCycleIter
           && controlStepValue > m_PrConst.min_step_CloseToEarth)
    {
        //Step = Step/2;
        Step = controlStepValue;//*********
        counter1++;

        this->UK[0] = Param[dim].M_info[4];
        this->UK[1] = Param[dim].M_info[5];
        this->UK[2] = Param[dim].M_info[6];
        this->UK[3] = Param[dim].M_info[1];
        this->UK[4] = Param[dim].M_info[2];
        this->UK[5] = Param[dim].M_info[3];

        detector = 1;
        counter2 = 0;
        while(detector == 1 && counter2 < AIR_BALL::MaxCycleIter)
        {
            detector = Solution(P_Inverse, gamma);
            counter2++;
        }

//        if (fabs(H_comp-H0) < m_PrConst.H_Eps ||  T - t0 > m_PrConst.Tcontr)
//            p_continue = false;
//        if (fabs(H0 - 0) < m_PrConst.H_Eps ||  T - t0 > m_PrConst.Tcontr)//******
        if (fabs(H_comp-H0) < m_PrConst.H_Eps ||  T - t0 > m_PrConst.Tcontr)
        {
              p_continue = false;//*******
        }

        if (H0 < m_PrConst.h0)
        {
            dim--;
            T = T - Step;
            Param.pop_back();
            VectH.pop_back();
            controlStepValue = Step / 2;//********
        }
        H_comp = GetHeight(Param[dim].M_info[1], Param[dim].M_info[2], Param[dim].M_info[3]);
    }
    Step = m_PrConst.step; //changed for comparison of Step

    if (T - t0 > m_PrConst.Tcontr)
    {
        SignCorrectProl = false;
    }

    if ( static_cast<qint32>( Param.size() ) > dim + 1 )
    {
        Param.resize(dim + 1);
    }

	return;

}


bool COnePrognosis::CalcTrack_OnTime(const qreal t0,const qreal t_end,const qreal x0,const qreal y0,const qreal z0,const qreal v0x,const qreal v0y,const qreal v0z,const qreal gamma,const bool P_Inverse)
{
    this->UK[0] = v0x; //fill initial parameters
    this->UK[1] = v0y;
    this->UK[2] = v0z;
    this->UK[3] = x0;
    this->UK[4] = y0;
    this->UK[5] = z0;

    Step = m_PrConst.step;

    RoK = GetDensity(x0,y0,z0);
    VK = GetAbsVector(v0x,v0y,v0z);
    H0 = GetHeight(x0,y0,z0);    
    if (H0 > Hmax)
    {
        Hmax = H0;
        tApogee = t0;
    }

    RecalcSystemCoefs(x0,y0,z0,CK,BK,AK);
    T = t0;
    dim = 0;

    if (!P_Inverse)
    {
        F_Diff_Eq(UK,AK,BK,CK,RoK,VK,gamma);
    }
    else
    {
        F_Diff_Eq_Inverse(UK,AK,BK,CK,RoK,VK,gamma);
    }
    M10 M;

    M.M_info[0] = t0;
    M.M_info[1] = x0;
    M.M_info[2] = y0;
    M.M_info[3] = z0;
    M.M_info[4] = v0x;
    M.M_info[5] = v0y;
    M.M_info[6] = v0z;
    M.M_info[7] = F[0];
    M.M_info[8] = F[1];
    M.M_info[9] = F[2];

    Param.push_back(M);
    VectH.push_back(H0);
    qint16 detector = 0;
    SignCorrectProl = true;
    qint32 counter1=0, counter2=0;
    qint32 MaxIter = static_cast<qint32>((t_end-t0)/m_PrConst.min_step) + AIR_BALL::MaxCycleIter;
    while(T < t_end && Step > m_PrConst.min_step && counter1 < MaxIter) //prolongation with check of step
    {
        detector = 1;
        Step = m_PrConst.step;
        counter1++;
        counter2=0;
        while(detector == 1 && Step > m_PrConst.min_step && counter2 < AIR_BALL::MaxCycleIter)
        {
            detector = Solution(P_Inverse, gamma);
            counter2++;
        }
    }

    if (Step - m_PrConst.min_step<  con_par_eps) //result is unsatisfactory if Step is very small
    {
        SignCorrectProl = false;
        return false;
    }

    if(dim == 0 && !SignCorrectProl)
    {
        return false;
    }

    if (T - t_end >  con_par_eps)
    {
        dim--;
        Param.pop_back();
        VectH.pop_back();
        T = T - Step;

        Step = t_end - T;

        bool p_continue = true;
        counter1 = 0;
        while (p_continue && counter1 < MaxIter)
        {
            counter1++;
            this->UK[0] = Param[dim].M_info[4];
            this->UK[1] = Param[dim].M_info[5];
            this->UK[2] = Param[dim].M_info[6];
            this->UK[3] = Param[dim].M_info[1];
            this->UK[4] = Param[dim].M_info[2];
            this->UK[5] = Param[dim].M_info[3];

            detector = 1;
            counter2 = 0;
            while(detector == 1 && counter2 < AIR_BALL::MaxCycleIter)
            {
                detector = Solution(P_Inverse, gamma);
                counter2++;
            }

            if (fabs(T - t_end) < con_par_eps || Step < m_PrConst.min_step)
            {
                p_continue = false;
            }
        }
    }

//    if (Step < m_PrConst.min_step + con_par_eps)
//    {
//        SignCorrectProl = false;
//        return false;
//    }

    Step = m_PrConst.step; //changed for comparison of Step
    return true;
}


bool COnePrognosis::CalcTrack_Aeroball(const qreal t0, const qreal t0_aeroball, const qreal x0, const qreal y0, const qreal z0, const qreal v0x, const qreal v0y, const qreal v0z, const qreal gamma, const qreal delta)
{
    this->UK[0] = v0x;
    this->UK[1] = v0y;
    this->UK[2] = v0z;
    this->UK[3] = x0;
    this->UK[4] = y0;
    this->UK[5] = z0;

    Step = m_PrConst.step;

    RoK = GetDensity(x0,y0,z0);
    VK = GetAbsVector(v0x,v0y,v0z);
    H0 = GetHeight(x0,y0,z0);
    if (H0 > Hmax)
    {
        Hmax = H0;
        tApogee = t0;
    }

    RecalcSystemCoefs(x0,y0,z0,CK,BK,AK);
    T = t0;
    dim = 0;

    if (T < t0_aeroball + cDTMaxPrognAeroball && H0 > AIR_BALL::MIN_H_AB_MANEUVER && H0 < AIR_BALL::MAX_H_APOGEE_QBM)
    {
        F_Diff_Eq_Aeroball(UK,AK,BK,CK,RoK,VK,gamma, delta);
    }
    else
    {
        F_Diff_Eq(UK,AK,BK,CK,RoK,VK,gamma);
    }
    M10 M;

    M.M_info[0] = t0;
    M.M_info[1] = x0;
    M.M_info[2] = y0;
    M.M_info[3] = z0;
    M.M_info[4] = v0x;
    M.M_info[5] = v0y;
    M.M_info[6] = v0z;
    M.M_info[7] = F[0];
    M.M_info[8] = F[1];
    M.M_info[9] = F[2];

    Param.push_back(M);
    VectH.push_back(H0);
    GLPointDouble3D PosPrev(M.M_info[1], M.M_info[2], M.M_info[3]);
    GLPointDouble3D Dir;
    GLPointDouble3D DirPrev;
    qint16 detector = 0;

    qint32 counter1=0, counter2=0;
    SignCorrectProl = true;
    while(H0 > m_PrConst.h0 && T - t0 <= m_PrConst.Tcontr && Step > m_PrConst.min_step && counter1 < 3 * AIR_BALL::MaxCycleIter)
    {
        detector = 1;
        counter1++;
        Step = m_PrConst.step;
        counter2 = 0;
        while(detector == 1 && Step > m_PrConst.min_step && counter2 < AIR_BALL::MaxCycleIter)
        {
            if (T < t0_aeroball + cDTMaxPrognAeroball && H0 > AIR_BALL::MIN_H_AB_MANEUVER && H0 < AIR_BALL::MAX_H_APOGEE_QBM)
            {
                detector = SolutionAeroball(gamma, delta, M);

                //flight direction check
                GLPointDouble3D Pos(M.M_info[1], M.M_info[2], M.M_info[3]);
                if (counter1 > 1)
                {
                    double ModPos = Pos.moduleFloat();
                    double ModPosPrev = PosPrev.moduleFloat();
                    if (ModPos > con_eps2 && ModPosPrev > con_eps2)
                    {
                        GLPointDouble3D ae = PosPrev / ModPosPrev;
                        GLPointDouble3D be = Pos / ModPos;
                        double CosAlpha = ae.ScalarProduct(be);
                        Dir = (be*CosAlpha - ae) * ModPosPrev; //perpendicular from previous position vector to the current
                        if (counter1 > 2)
                        {
                            double ScalProd = Dir.ScalarProduct(DirPrev);
                            if (ScalProd < con_eps2)
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                PosPrev = Pos;
                DirPrev = Dir;
            }
            else
            {
                detector = Solution(false, gamma);
            }
            counter2++;
        }
    }

    if (T-t0 - m_PrConst.Tcontr >  -con_eps
            || Step - m_PrConst.min_step <  con_par_eps)
    {
        SignCorrectProl = false;
        return SignCorrectProl;
    }

    if(dim == 0)
    {
        return SignCorrectProl;
    }
    dim--;
    Param.pop_back();
    VectH.pop_back();

    qreal H_comp = GetHeight(Param[dim].M_info[1], Param[dim].M_info[2], Param[dim].M_info[3]);

    T = T - Step;
    Step = Step/2;//*******
    qreal controlStepValue = Step;//*******
    bool p_continue = true;
    counter1 = 0;
    while (p_continue
           && counter1 < AIR_BALL::MaxCycleIter
           && controlStepValue > m_PrConst.min_step_CloseToEarth)
    {
        //Step = Step/2;
        Step = controlStepValue;//*********
        counter1++;

        this->UK[0] = Param[dim].M_info[4];
        this->UK[1] = Param[dim].M_info[5];
        this->UK[2] = Param[dim].M_info[6];
        this->UK[3] = Param[dim].M_info[1];
        this->UK[4] = Param[dim].M_info[2];
        this->UK[5] = Param[dim].M_info[3];

        detector = 1;
        counter2 = 0;
        while(detector == 1 && counter2 < AIR_BALL::MaxCycleIter)
        {
            detector = Solution(false, gamma);
            counter2++;
        }

        if (fabs(H_comp-H0) < m_PrConst.H_Eps ||  T - t0 > m_PrConst.Tcontr)
        {
              p_continue = false;//*******
        }

        if (H0 < m_PrConst.h0)
        {
            dim--;
            T = T - Step;
            Param.pop_back();
            VectH.pop_back();
            controlStepValue = Step / 2;//********
        }
        H_comp = GetHeight(Param[dim].M_info[1], Param[dim].M_info[2], Param[dim].M_info[3]);
    }
    Step = m_PrConst.step; //changed for comparison of Step

    if (T - t0 > m_PrConst.Tcontr || H0 > c_H_tolerance)
    {
        SignCorrectProl = false;
    }

    if ( static_cast<qint32>( Param.size() ) > dim + 1 )
    {
        Param.resize(dim + 1);
    }

    return SignCorrectProl;
}


//composition of a log-file for review CalcTrack function results
void COnePrognosis::OutputTrack()
{
    std::streamsize ss = AIR_BALL::out_param.precision();
    AIR_BALL::out_param << std::setprecision(10);

    for (qint32 i = 0; i < static_cast<qint32> (Param.size()); i++)
    {
        for (qint32 j = 0; j < 10; j++)
        {
            AIR_BALL::out_param << Param[i].M_info[j] << "\t";
        }
        AIR_BALL::out_param << std::endl;
    }
    AIR_BALL::out_param.precision(ss);
}


void COnePrognosis::ClearTrack()
{
    //Param.clear();
    std::vector<M10>().swap(Param);
    std::vector<qreal>().swap(VectH);
    dim = 0;
    Tpass = 0;
}


bool COnePrognosis::OutputLastPoint(qreal &Time, qint32 &Size, GLPointDouble3D &Coord, GLPointDouble3D &Vel, GLPointDouble3D &Acc)
{
    if (Param.size() < 1 || dim < 1)
    {
        return false;
    }

    Time = Param[dim].M_info[0];
    Size = Param.size();

    Coord.x = Param[dim].M_info[1];
    Coord.y = Param[dim].M_info[2];
    Coord.z = Param[dim].M_info[3];

    Vel.x = Param[dim].M_info[4];
    Vel.y = Param[dim].M_info[5];
    Vel.z = Param[dim].M_info[6];

    Acc.x = Param[dim].M_info[7];
    Acc.y = Param[dim].M_info[8];
    Acc.z = Param[dim].M_info[9];

    return true;
}


bool COnePrognosis::OutputLastPoint(qreal &Time, qint32 &Size, GLPointDouble3D &Coord, GLPointDouble3D &Vel)
{
    if (Param.size() < 1 || dim < 1)
    {
        return false;
    }

    Time = Param[dim].M_info[0];
    Size = Param.size();

    Coord.x = Param[dim].M_info[1];
    Coord.y = Param[dim].M_info[2];
    Coord.z = Param[dim].M_info[3];

    Vel.x = Param[dim].M_info[4];
    Vel.y = Param[dim].M_info[5];
    Vel.z = Param[dim].M_info[6];

    return true;
}


void COnePrognosis::GetProlongedTrack(std::vector<M10> &ProlongedTrack, qint32 &dimension)
{
    std::vector<M10>().swap(ProlongedTrack);
    ProlongedTrack = Param;
    dimension = dim;
}


bool COnePrognosis::OutputStartPointByTables(const qreal &CurrTime,const Str_Ar_EAP_Param_1Table *pEAP_table, qreal &StartTime, GLPointDouble3D &Coord)
{
    if (pEAP_table != nullptr)
    {
        StartTime = CurrTime - Tpass - pEAP_table->t_EAP;
    }
    else
    {
        StartTime = CurrTime - Tpass - m_PrConst.T_act_def;
    }

    Coord.x = CalcStartPoint.x;
    Coord.y = CalcStartPoint.y;
    Coord.z = CalcStartPoint.z;

    if (fabs(Tpass) + fabs(Coord.x) + fabs(Coord.y) + fabs(Coord.z) < con_par_eps)
    {
        return false;
    }

    return true;
}


bool COnePrognosis::OutputStartPointByTables(const qreal &CurrTime,const Str_Ar_EAP_Param_1Table *pEAP_table, qreal &StartTime, GLPointDouble3D &CoordSP, qreal &T_passive, GLPointDouble3D &CoordEAP, GLPointDouble3D &VelEAP)
{
    T_passive = 0.;
    const bool ItsOK = OutputStartPointByTables(CurrTime, pEAP_table, StartTime, CoordSP);
    if (ItsOK)
    {
        T_passive = Tpass;
        CoordEAP.x = Param[dim].M_info[1];
        CoordEAP.y = Param[dim].M_info[2];
        CoordEAP.z = Param[dim].M_info[3];
        VelEAP.x = Param[dim].M_info[4];
        VelEAP.y = Param[dim].M_info[5];
        VelEAP.z = Param[dim].M_info[6];
        return true;
    }
    else
    {
        return false;
    }
}


bool COnePrognosis::OutputLastPoint(qreal &Time, qint32 &Size, TMatrix<6> &ColumnCV)
{
    if (Param.size() < 1 || dim < 1)
    {
        return false;
    }

    Time = Param[dim].M_info[0];
    Size = Param.size();

    ColumnCV.Reset(6,1);

    ColumnCV.M[0][0] = Param[dim].M_info[1];
    ColumnCV.M[1][0] = Param[dim].M_info[2];
    ColumnCV.M[2][0] = Param[dim].M_info[3];

    ColumnCV.M[3][0] = Param[dim].M_info[4];
    ColumnCV.M[4][0] = Param[dim].M_info[5];
    ColumnCV.M[5][0] = Param[dim].M_info[6];

    return true;
}


//Function for solving system of differential equations
qint16 COnePrognosis::Solution(const bool P_Inverse, const qreal gamma)
{
	RungeKutta(UK,AK,BK,CK,RoK,VK,gamma,Step,P_Inverse);//->RK[6]
	qreal UK1[6],UK11[6];
	qint16 j;
	for(j=0;j<6;j++)
	{
		UK1[j] = UK[j] + ( RK[0][j] + 2*RK[1][j] + 2*RK[2][j] + RK[3][j] )/6.;
		UK11[j] = UK[j];
	}
	for(qint16 n = 1;n<=2;n++)
	{
		RungeKutta(UK11,AK,BK,CK,RoK,VK,gamma,Step/2,P_Inverse);
		for(j=0;j<6;j++)
        {
			UK11[j] = UK11[j] + ( RK[0][j] + 2*RK[1][j] + 2*RK[2][j] + RK[3][j] )/6.;
        }
	}
	qreal Accuracy = 0;
	for(j=3;j<6;j++)
	{
		if(Accuracy < fabs(UK11[j] - UK1[j]))
        {
			Accuracy = fabs(UK11[j] - UK1[j]);
        }
	}
	if(Accuracy > m_PrConst.Delta_Eps)
	{
		Step /= 2;
		return 1; //return 0; //changed for comparison of Step
	}
	else
	{
		T += Step;
		this->dim++;
		RoK = GetDensity(UK11[3],UK11[4],UK11[5]);
		VK = GetAbsVector(UK11[0],UK11[1],UK11[2]);
		H0 = GetHeight(UK11[3],UK11[4],UK11[5]);
        if (H0 > Hmax)
        {
            Hmax = H0;
            tApogee = T;
        }
		RecalcSystemCoefs(UK11[3],UK11[4],UK11[5],CK,BK,AK);

		M10 M;
		if (!P_Inverse)
        {
            F_Diff_Eq(UK11,AK,BK,CK,RoK,VK,gamma);
        }
		else
        {
            F_Diff_Eq_Inverse(UK11,AK,BK,CK,RoK,VK,gamma);
        }
		M.M_info[0] = T;
		M.M_info[1] = UK11[3];
		M.M_info[2] = UK11[4];
		M.M_info[3] = UK11[5];
		M.M_info[4] = UK11[0];
		M.M_info[5] = UK11[1];
		M.M_info[6] = UK11[2];
		M.M_info[7] = F[0];
		M.M_info[8] = F[1];
		M.M_info[9] = F[2];

		Param.push_back(M);
        VectH.push_back(H0);
//AIR_BALL::out_param << setprecision(10) << dim << "\t" << Param.size() << std::endl;
		for(j=0;j<6;j++)
        {
			UK[j] = UK11[j];
        }
		return 0;
	}
}


//Function for solving system of differential equations for aeroballistic missile
qint16 COnePrognosis::SolutionAeroball(const qreal gamma, const qreal delta, M10 &M)
{
    RungeKutta_Aeroball(UK, AK, BK, CK, RoK, VK, gamma, delta, Step);
    qreal UK1[6],UK11[6];
    qint16 j;
    for(j=0;j<6;j++)
    {
        UK1[j] = UK[j] + ( RK[0][j] + 2*RK[1][j] + 2*RK[2][j] + RK[3][j] )/6.;
        UK11[j] = UK[j];
    }
    for(qint16 n = 1;n<=2;n++)
    {
        RungeKutta_Aeroball(UK11,AK,BK,CK,RoK,VK,gamma, delta,Step/2);
        for(j=0;j<6;j++)
        {
            UK11[j] = UK11[j] + ( RK[0][j] + 2*RK[1][j] + 2*RK[2][j] + RK[3][j] )/6.;
        }
    }
    qreal Accuracy = 0;
    for(j=3;j<6;j++)
    {
        if(Accuracy < fabs(UK11[j] - UK1[j]))
        {
            Accuracy = fabs(UK11[j] - UK1[j]);
        }
    }
    if(Accuracy > m_PrConst.Delta_Eps)
    {
        Step /= 2;
        return 1; //return 0; //changed for comparison of Step
    }
    else
    {
        T += Step;
        this->dim++;
        RoK = GetDensity(UK11[3],UK11[4],UK11[5]);
        VK = GetAbsVector(UK11[0],UK11[1],UK11[2]);
        H0 = GetHeight(UK11[3],UK11[4],UK11[5]);
        if (H0 > Hmax)
        {
            Hmax = H0;
            tApogee = T;
        }
        RecalcSystemCoefs(UK11[3],UK11[4],UK11[5],CK,BK,AK);

        //M10 M;
        F_Diff_Eq_Aeroball(UK11,AK,BK,CK,RoK,VK,gamma, delta);

        M.M_info[0] = T;
        M.M_info[1] = UK11[3];
        M.M_info[2] = UK11[4];
        M.M_info[3] = UK11[5];
        M.M_info[4] = UK11[0];
        M.M_info[5] = UK11[1];
        M.M_info[6] = UK11[2];
        M.M_info[7] = F[0];
        M.M_info[8] = F[1];
        M.M_info[9] = F[2];

        Param.push_back(M);
        VectH.push_back(H0);

        for(j=0;j<6;j++)
        {
            UK[j] = UK11[j];
        }
        return 0;
    }
}


//Function for finding start point by end of active leg characteristics
bool COnePrognosis::FindStartByEAP(Str_Ar_EAP_Param_1Table* pEAP_table, const qreal t0, const qreal x0, const qreal y0, const qreal z0,
                                   const qreal v0x, const qreal v0y, const qreal v0z, const qreal gamma, qreal &Theta,
                                   qreal &modV, qreal &DeltaL, qreal &ResDeltaV,
                                   qreal &ResHBoost1, qreal &ResLBoost1, qreal &ResVBoost1,
                                   qreal &ResHBoost2, qreal &ResLBoost2, qreal &ResVBoost2)
{
	bool P_find_kau = false;
    ResDeltaV = 0;
    ResHBoost1 = 0;
    ResLBoost1 = 0;
    ResVBoost1 = 0;
    ResHBoost2 = 0;
    ResLBoost2 = 0;
    ResVBoost2 = 0;

    if (pEAP_table == nullptr)
    {
        return false;
    }

//    Point_3Coord KAU_Point, KAU_V;
	Theta = 0;

    qint32 i;
	qreal Hmax = 0;

    qreal L; qreal L_pred; qreal H_comp;    
	L_pred = -1;
	L = -1;
    DeltaL = 0;
	bool p_final_progn = false;

    for (i=0; i<pEAP_table->N_el; i++)
    {
        if (pEAP_table->Items[i].H > Hmax)
        {
            Hmax = pEAP_table->Items[i].H;
        }
    }
	
	this->UK[0] = v0x;
	this->UK[1] = v0y;
	this->UK[2] = v0z;
	this->UK[3] = x0;
	this->UK[4] = y0;
	this->UK[5] = z0;

	Step = m_PrConst.step;

	RoK = GetDensity(x0,y0,z0);
	VK = GetAbsVector(v0x,v0y,v0z);
	H0 = GetHeight(x0,y0,z0);
//    Hmax = std::max(Hmax, H0);
	
	RecalcSystemCoefs(x0,y0,z0,CK,BK,AK);
	T = t0;
	dim = 0;

    F_Diff_Eq_Inverse(UK,AK,BK,CK,RoK,VK,gamma);
	M10 M;

	M.M_info[0] = t0;
	M.M_info[1] = x0;
	M.M_info[2] = y0;
	M.M_info[3] = z0;
	M.M_info[4] = v0x;
	M.M_info[5] = v0y;
	M.M_info[6] = v0z;
	M.M_info[7] = F[0];
	M.M_info[8] = F[1];
	M.M_info[9] = F[2];

	Param.push_back(M);
    VectH.push_back(H0);
	qint16 detector = 0;

	bool p_end_progn = false;
    qint32 counter1=0, counter2=0;
	//while(H0 > m_PrConst.h0 && T - t0 <= m_PrConst.Tcontr)
    SignCorrectProl = true;
    while (!p_end_progn && counter1 < AIR_BALL::MaxCycleIter)
	{
        counter1++;
		detector = 1;
        counter2 = 0;
        while(detector == 1 && counter2 < AIR_BALL::MaxCycleIter)
		{
			detector = Solution(true, gamma);
            counter2++;
		}
		
		if (H0 < 0 || T - t0 > m_PrConst.Tcontr)
		{
			dim--;
			T = T - Step;
			Param.pop_back();
            VectH.pop_back();
			p_end_progn = true;

            if (T - t0 > m_PrConst.Tcontr)
            {
                SignCorrectProl = false;
            }
		}
		else
		{			
            H_comp = GetHeight(Param[dim].M_info[1], Param[dim].M_info[2], Param[dim].M_info[3]);
			if (H_comp > 0 && H_comp < Hmax + m_PrConst.H_tolerance)
			{
                const qreal Vh_curr = (Param[dim].M_info[1]*Param[dim].M_info[4] + Param[dim].M_info[2]*Param[dim].M_info[5] + Param[dim].M_info[3]*Param[dim].M_info[6]) /
								 GetAbsVector(Param[dim].M_info[1], Param[dim].M_info[2], Param[dim].M_info[3]);
				if (Vh_curr < m_PrConst.VH_tolerance)
                {
                    modV = GetAbsVector(Param[dim].M_info[4], Param[dim].M_info[5], Param[dim].M_info[6]);
                    Theta = (con_half_pi - acos(fabs(Vh_curr)/modV))*180./con_pi;
					L_pred = L;                    
                    L = Check_EAP(pEAP_table, H_comp, Theta, modV, DeltaL, p_final_progn, ResDeltaV,
                                  ResHBoost1, ResLBoost1, ResVBoost1, ResHBoost2, ResLBoost2, ResVBoost2);
					if (L < 0 && L_pred > 0)
					{
						p_end_progn = true;
						P_find_kau = true;

						dim--;
						T = T - Step;
						Param.pop_back();
                        VectH.pop_back();
					}
					if (L > 0)
					{
						//P1 ~ (Xkau, Ykau, Zkau), P2 ~ (Xkau+VXkau, Ykau+VYkau, Zkau+VZkau): WGS to GEO
                        CGeocentric P1_wgs_init;
                        CGeocentric P2_wgs_init;
                        CGeocentric P2_wgs;
                        CGeocentric PStart_wgs;

                        CGeodesic P1_geo;
                        CGeodesic P2_geo;
	
                        P1_wgs_init.m_dX = Param[dim].M_info[1];
                        P1_wgs_init.m_dY = Param[dim].M_info[2];
                        P1_wgs_init.m_dZ = Param[dim].M_info[3];

                        P2_wgs_init.m_dX = Param[dim].M_info[1] + Param[dim].M_info[4];
                        P2_wgs_init.m_dY = Param[dim].M_info[2] + Param[dim].M_info[5];
                        P2_wgs_init.m_dZ = Param[dim].M_info[3] + Param[dim].M_info[6];

                        bool bOK = GEOCENTRIC_GEO(&P1_wgs_init, &P1_geo);
                        bOK = bOK && GEOCENTRIC_GEO(&P2_wgs_init, &P2_geo);

						//P2: H = 0; GEO to WGS
                        P1_geo.m_dAltitude = 0;
                        P2_geo.m_dAltitude = 0;
												
                        bOK = bOK && GEO_GEOCENTRIC(&P2_geo, &P2_wgs);

						//P2: WGS to TOPO
                        CTopocentric P2_topo;
                        CTopocentric PStart_topo;
                        CTopographic P2_topogr;
                        CTopographic PStart_topogr;

						//Center ~ P1
                        bOK = bOK && GEOCENTRIC_TOPO(&P1_geo, &P2_wgs, &P2_topo);
						
						//P2: TOPO (= TOPOCENTRIC) to TOPOGRAPHIC
//                        AIR_BALL::CTOPOCENTRICtoTOPOGRAPHIC CTT;
//                        AIR_BALL::TOPOCENTRIC P2_topoc, PStart_topoc;
//                        AIR_BALL::TOPOGRAPHIC P2_topog, PStart_topog;

//                        P2_topoc.X = P2_topo.m_dXt;
//                        P2_topoc.Y = P2_topo.m_dYt;
//                        P2_topoc.Z = P2_topo.m_dZt;

//						CTT.RecountTOPOCENTRICtoTOPOGRAPHIC_coord(&P2_topoc, &P2_topog);
                        bOK = bOK && RecountTOPOCENTRICtoTOPOGRAPHIC_coord(&P2_topo, &P2_topogr);

                        const qreal L0 = sqrt( pow(P2_topogr.m_dXt, 2) + pow(P2_topogr.m_dZt, 2) );

						//Start Point in TOPOGRAPHIC
                        PStart_topogr.m_dXt = P2_topogr.m_dXt * L/L0;
                        PStart_topogr.m_dZt = P2_topogr.m_dZt * L/L0;
                        PStart_topogr.m_dHt = 0;

						//Start Point in TOPOCENTRIC
//						CTT.RecountTOPOGRAPHICtoTOPOCENTRIC_coord(&PStart_topog, &PStart_topoc);
                        bOK = bOK && RecountTOPOGRAPHICtoTOPOCENTRIC_coord(&PStart_topogr, &PStart_topo);

//                        PStart_topo.m_dXt = PStart_topoc.X;
//                        PStart_topo.m_dYt = PStart_topoc.Y;
//                        PStart_topo.m_dZt = PStart_topoc.Z;

						//Start Point in WGS
                        bOK = bOK && TOPO_GEOCENTRIC(&P1_geo, &PStart_topo, &PStart_wgs);

                        CalcStartPoint.x = PStart_wgs.m_dX;
                        CalcStartPoint.y = PStart_wgs.m_dY;
                        CalcStartPoint.z = PStart_wgs.m_dZ;

                        if (p_final_progn && bOK)
						{
							p_end_progn = true; 
							P_find_kau = true;
						}

						Tpass = Param[dim].M_info[0] - t0;						
					}
				}
			}
		}
	}

    if(dim == 0 || !SignCorrectProl)
    {
        P_find_kau = false;
    }
    else
    {
        dim--;
        Param.pop_back();
        VectH.pop_back();
    }

	return P_find_kau;
}

//Function for finding of end of active leg
qreal COnePrognosis::Check_EAP(const Str_Ar_EAP_Param_1Table *pEAP_table, const qreal currH, const qreal currTheta,
                                   const qreal currV, qreal &DeltaL, bool &p_final, qreal &ResDeltaV,
                                   qreal &ResHBoost1, qreal &ResLBoost1, qreal &ResVBoost1,
                                   qreal &ResHBoost2, qreal &ResLBoost2, qreal &ResVBoost2)
{
    qreal L; qreal L_n; qreal L_k; qreal Theta_kau_n; qreal Theta_kau_k;
    qint32 i;
    DeltaL = 0;
	p_final = false;
    ResDeltaV = 0;
    ResHBoost1 = 0;
    ResLBoost1 = 0;
    ResVBoost1 = 0;
    ResHBoost2 = 0;
    ResLBoost2 = 0;
    ResVBoost2 = 0;

	L = -1;
    if (pEAP_table != nullptr)
    {
        if (currH <= pEAP_table->Items[0].H && currH > m_PrConst.H_Eps)
        {
            Theta_kau_n = pEAP_table->Items[0].theta;
            Theta_kau_k = pEAP_table->Items[1].theta;
            L_n = pEAP_table->Items[0].L;
            L_k = pEAP_table->Items[1].L;
            DeltaL = L_k - L_n;
            L = L_n + DeltaL  * (currTheta - Theta_kau_n) / (Theta_kau_k - Theta_kau_n);
            CalculateSepBoostParam(pEAP_table, 0, currTheta, ResHBoost1, ResLBoost1, ResVBoost1, ResHBoost2, ResLBoost2, ResVBoost2);
            ResDeltaV = fabs(currV - pEAP_table->Items[0].V);
            p_final = true;
        }
        else
        {
            for (i=0; i < pEAP_table->N_el-1; i++)
            {
                if (currH > pEAP_table->Items[i].H && currH <= pEAP_table->Items[i+1].H)
                {
                    if (currTheta >= pEAP_table->Items[i].theta - m_PrConst.dTheta1
                            && currTheta <= pEAP_table->Items[i+1].theta + m_PrConst.dTheta2
                            && (pEAP_table->Items[i].theta < pEAP_table->Items[i+1].theta
                                || pEAP_table->Items[i+1].theta < con_eps)) //this condition takes into account possible switch between 2 branches on graph "Theta-H"
                    {
                        Theta_kau_n = pEAP_table->Items[i].theta;
                        Theta_kau_k = pEAP_table->Items[i+1].theta;
                        L_n = pEAP_table->Items[i].L;
                        L_k = pEAP_table->Items[i+1].L;
                        DeltaL = L_k - L_n;
                        L = L_n + DeltaL * (currTheta - Theta_kau_n) / (Theta_kau_k - Theta_kau_n);
                        CalculateSepBoostParam(pEAP_table, i, currTheta, ResHBoost1, ResLBoost1, ResVBoost1, ResHBoost2, ResLBoost2, ResVBoost2);

                        if (currV + c_dModVel >= pEAP_table->Items[i+1].V || currV + c_dModVel >= pEAP_table->Items[i].V)
                        {
                            p_final = true;
                            ResDeltaV = std::min(fabs(currV - pEAP_table->Items[i+1].V), fabs(currV - pEAP_table->Items[i].V));
                        }
                        break;
                    }
                    //				break;
                }
            }
        }
    }
	return L;
}


void COnePrognosis::CalculateSepBoostParam(const Str_Ar_EAP_Param_1Table *pEAP_table, const qint16 i_row, const qreal CurrTheta,
                                           qreal &ResHBoost1, qreal &ResLBoost1, qreal &ResVBoost1,
                                           qreal &ResHBoost2, qreal &ResLBoost2, qreal &ResVBoost2)
{
    ResHBoost1 = 0;
    ResLBoost1 = 0;
    ResVBoost1 = 0;
    ResHBoost2 = 0;
    ResLBoost2 = 0;
    ResVBoost2 = 0;

    if (pEAP_table != nullptr && 0 <= i_row && i_row < AIR_BALL::N_ELEM_KAU_PARAM-1)
    {
        if (pEAP_table->QuantityBoost >= 2)
        {
            qreal Theta_eap_n, Theta_eap_k, Proportion;

            Theta_eap_n = pEAP_table->Items[i_row].theta;
            Theta_eap_k = pEAP_table->Items[i_row+1].theta;
            Proportion = (CurrTheta - Theta_eap_n) / (Theta_eap_k - Theta_eap_n);

            ResHBoost1 = pEAP_table->ItemsBoost1[i_row].H + (pEAP_table->ItemsBoost1[i_row+1].H - pEAP_table->ItemsBoost1[i_row].H) * Proportion;
            ResLBoost1 = pEAP_table->ItemsBoost1[i_row].L + (pEAP_table->ItemsBoost1[i_row+1].L - pEAP_table->ItemsBoost1[i_row].L) * Proportion;
            ResVBoost1 = pEAP_table->ItemsBoost1[i_row].V + (pEAP_table->ItemsBoost1[i_row+1].V - pEAP_table->ItemsBoost1[i_row].V) * Proportion;

            if (pEAP_table->QuantityBoost >= 3)
            {
                ResHBoost2 = pEAP_table->ItemsBoost2[i_row].H + (pEAP_table->ItemsBoost2[i_row+1].H - pEAP_table->ItemsBoost2[i_row].H) * Proportion;
                ResLBoost2 = pEAP_table->ItemsBoost2[i_row].L + (pEAP_table->ItemsBoost2[i_row+1].L - pEAP_table->ItemsBoost2[i_row].L) * Proportion;
                ResVBoost2 = pEAP_table->ItemsBoost2[i_row].V + (pEAP_table->ItemsBoost2[i_row+1].V - pEAP_table->ItemsBoost2[i_row].V) * Proportion;
            }
        }
    }
}


//Function for calculation of density
qreal COnePrognosis::GetDensity(const qreal x, const qreal y, const qreal z)
{
	qint16 i = 0;
    qreal ro; qreal h; qreal A; qreal k1; qreal k2; qreal hi;
	ro = 0; // VV_1034:Corrected
	h = GetHeight(x,y,z);
	while(i < 8)
	{
		if(i < 7)
		{
			if(DensCoef[i][0]< h && h <= DensCoef[i+1][0])
			{
				A = DensCoef[i][1];
				k1 = DensCoef[i][2];
				k2 = DensCoef[i][3];
				hi = DensCoef[i][0];
				ro = A*exp(k1*pow(h - hi,2) - k2*( h- hi));	
				break;
			}
		}
		else
		{

			A = DensCoef[7][1];
			k1 = DensCoef[7][2];
			k2 = DensCoef[7][3];
			hi = DensCoef[7][0];
			ro = A*exp(k1*pow(h - hi,2) - k2*( h- hi));
		}
		i++;
	}
	return ro;
}

//Function for calculation of absolute meaning of vector
qreal COnePrognosis::GetAbsVector(const qreal x, const qreal y, const qreal z)
{
    return(sqrt(x*x + y*y + z*z));
}

//Function for calculation of the height above ellipsoid surface
qreal COnePrognosis::GetHeight(const qreal x, const qreal y, const qreal z)
{
    const qreal r = GetAbsVector(x,y,z);
    const qreal h1 = AIR_BALL::cd_a*(1-AIR_BALL::cd_Alpha);
    const qreal h2 = pow(1-AIR_BALL::cd_Alpha,2) + (2*AIR_BALL::cd_Alpha - pow(AIR_BALL::cd_Alpha,2))*pow(z/r,2);
    const qreal h = r - h1/sqrt(h2);
	return h;
}


//Function for calculation of system coefficients
void COnePrognosis::RecalcSystemCoefs(const qreal x, const qreal y, const qreal z,
		qreal &C, qreal &B, qreal &A)
{
    const qreal r = GetAbsVector(x,y,z);
    const qreal DABC = 5*pow(z/r,2);
    const qreal alpha00 = AIR_BALL::cd_b0/AIR_BALL::RZ;
    const qreal alpha20 = -AIR_BALL::cd_b2/pow(AIR_BALL::RZ,3);
    C = 1.5*alpha20*pow(AIR_BALL::RZ/r,2);
    B = AIR_BALL::RZ/pow(r,3);
	A = B*(alpha00 + C*(DABC - 1));
}


//Function for calculation Runge-Kutta coefficients
void COnePrognosis::RungeKutta(const qreal uk[6], const qreal Ak, const qreal Bk, const qreal Ck,
                                             const qreal Rok,  const qreal Vk, const qreal gam, const qreal h, const bool P_Inverse)
{
	qint16 j;
	qreal uk1[6];
	if (P_Inverse)
    {
        F_Diff_Eq_Inverse(uk,Ak,Bk,Ck,Rok,Vk,gam);
    }
	else
    {
        F_Diff_Eq(uk,Ak,Bk,Ck,Rok,Vk,gam);
    }
	for(j=0;j<6;j++)
    {
		RK[0][j] = h*F[j];
    }

	uk1[0] = uk[0] + RK[0][0]/2;
	uk1[1] = uk[1] + RK[0][1]/2;
	uk1[2] = uk[2] + RK[0][2]/2;
	uk1[3] = uk[3] + RK[0][3]/2;
	uk1[4] = uk[4] + RK[0][4]/2;
	uk1[5] = uk[5] + RK[0][5]/2;
	if (P_Inverse)
    {
        F_Diff_Eq_Inverse(uk1,Ak,Bk,Ck,Rok,Vk,gam);
    }
	else
    {
        F_Diff_Eq(uk1,Ak,Bk,Ck,Rok,Vk,gam);
    }
	for(j=0;j<6;j++)
    {
		RK[1][j] = h*F[j];
    }

	uk1[0] = uk[0] + RK[1][0]/2;
	uk1[1] = uk[1] + RK[1][1]/2;
	uk1[2] = uk[2] + RK[1][2]/2;
	uk1[3] = uk[3] + RK[1][3]/2;
	uk1[4] = uk[4] + RK[1][4]/2;
	uk1[5] = uk[5] + RK[1][5]/2;
	if (P_Inverse)
    {
        F_Diff_Eq_Inverse(uk1,Ak,Bk,Ck,Rok,Vk,gam);
    }
	else
    {
        F_Diff_Eq(uk1,Ak,Bk,Ck,Rok,Vk,gam);
    }
	for(j=0;j<6;j++)
    {
		RK[2][j] = h*F[j];
    }

	uk1[0] = uk[0] + RK[2][0];
	uk1[1] = uk[1] + RK[2][1];
	uk1[2] = uk[2] + RK[2][2];
	uk1[3] = uk[3] + RK[2][3];
	uk1[4] = uk[4] + RK[2][4];
	uk1[5] = uk[5] + RK[2][5];
	if (P_Inverse)
    {
        F_Diff_Eq_Inverse(uk1,Ak,Bk,Ck,Rok,Vk,gam);
    }
	else
    {
        F_Diff_Eq(uk1,Ak,Bk,Ck,Rok,Vk,gam);
    }
	for(j=0;j<6;j++)
    {
		RK[3][j] = h*F[j];
    }
}


//Function for calculation Runge-Kutta coefficients for aeroballistic missile
void COnePrognosis::RungeKutta_Aeroball(const qreal uk[], const qreal Ak, const qreal Bk, const qreal Ck, const qreal Rok, const qreal Vk, const qreal gam, const qreal delta, const qreal h)
{
    qint16 j;
    qreal uk1[6];

    F_Diff_Eq_Aeroball(uk,Ak,Bk,Ck,Rok,Vk,gam, delta);

    for(j=0;j<6;j++)
    {
        RK[0][j] = h*F[j];
    }

    uk1[0] = uk[0] + RK[0][0]/2;
    uk1[1] = uk[1] + RK[0][1]/2;
    uk1[2] = uk[2] + RK[0][2]/2;
    uk1[3] = uk[3] + RK[0][3]/2;
    uk1[4] = uk[4] + RK[0][4]/2;
    uk1[5] = uk[5] + RK[0][5]/2;

    F_Diff_Eq_Aeroball(uk1,Ak,Bk,Ck,Rok,Vk,gam, delta);

    for(j=0;j<6;j++)
    {
        RK[1][j] = h*F[j];
    }

    uk1[0] = uk[0] + RK[1][0]/2;
    uk1[1] = uk[1] + RK[1][1]/2;
    uk1[2] = uk[2] + RK[1][2]/2;
    uk1[3] = uk[3] + RK[1][3]/2;
    uk1[4] = uk[4] + RK[1][4]/2;
    uk1[5] = uk[5] + RK[1][5]/2;

    F_Diff_Eq_Aeroball(uk1,Ak,Bk,Ck,Rok,Vk,gam, delta);

    for(j=0;j<6;j++)
    {
        RK[2][j] = h*F[j];
    }

    uk1[0] = uk[0] + RK[2][0];
    uk1[1] = uk[1] + RK[2][1];
    uk1[2] = uk[2] + RK[2][2];
    uk1[3] = uk[3] + RK[2][3];
    uk1[4] = uk[4] + RK[2][4];
    uk1[5] = uk[5] + RK[2][5];

    F_Diff_Eq_Aeroball(uk1,Ak,Bk,Ck,Rok,Vk,gam, delta);

    for(j=0;j<6;j++)
    {
        RK[3][j] = h*F[j];
    }
}


//Function defining the system of differential equations
void COnePrognosis::F_Diff_Eq(const qreal U1[6], const qreal A1, const qreal B1, const qreal C1,
                                      const qreal Ro1, const qreal V1, const qreal Gam1)
{
    const qreal x = U1[3];
    const qreal y = U1[4];
    const qreal z = U1[5];
    const qreal vx = U1[0];
    const qreal vy = U1[1];
    const qreal vz = U1[2];
    this->F[0] = ( pow(AIR_BALL::OmegaZ,2) - A1 )*x + 2*AIR_BALL::OmegaZ*vy - Gam1*Ro1*V1*vx;
    this->F[1] = ( pow(AIR_BALL::OmegaZ,2) - A1 )*y - 2*AIR_BALL::OmegaZ*vx - Gam1*Ro1*V1*vy;
	this->F[2] = (2*B1*C1 - A1)*z - Gam1*Ro1*V1*vz;
	this->F[3] = vx;
	this->F[4] = vy;
	this->F[5] = vz;
}


//Function defining the system of differential equation of ballistic target movement in inverse prognosis
void COnePrognosis::F_Diff_Eq_Inverse(const qreal U1[6],const qreal A1,const qreal B1,const qreal C1,
                                      const qreal Ro1,const qreal V1,const qreal Gam1)
{
    const qreal x = U1[3];
    const qreal y = U1[4];
    const qreal z = U1[5];
    const qreal vx = U1[0];
    const qreal vy = U1[1];
    const qreal vz = U1[2];
    this->F[0] = ( pow(AIR_BALL::OmegaZ,2) - A1 )*x - 2*AIR_BALL::OmegaZ*vy + Gam1*Ro1*V1*vx;
    this->F[1] = ( pow(AIR_BALL::OmegaZ,2) - A1 )*y + 2*AIR_BALL::OmegaZ*vx + Gam1*Ro1*V1*vy;
	this->F[2] = (2*B1*C1 - A1)*z + Gam1*Ro1*V1*vz;
	this->F[3] = vx;
	this->F[4] = vy;
	this->F[5] = vz;
}


//Function defining the system of differential equations during aeroballistic maneuver
void COnePrognosis::F_Diff_Eq_Aeroball(const qreal U1[6], const qreal A1, const qreal B1, const qreal C1,
                                      const qreal Ro1, const qreal V1, const qreal Gam1, const qreal Delta1)
{
    const qreal x = U1[3];
    const qreal y = U1[4];
    const qreal z = U1[5];
    const qreal vx = U1[0];
    const qreal vy = U1[1];
    const qreal vz = U1[2];
    double RoV = Ro1*V1;
    this->F[0] = ( pow(AIR_BALL::OmegaZ,2) - A1 )*x + 2*AIR_BALL::OmegaZ*vy - Gam1*RoV*vx;
    this->F[1] = ( pow(AIR_BALL::OmegaZ,2) - A1 )*y - 2*AIR_BALL::OmegaZ*vx - Gam1*RoV*vy;
    this->F[2] = (2*B1*C1 - A1)*z - Gam1*RoV*vz;
    this->F[3] = vx;
    this->F[4] = vy;
    this->F[5] = vz;

    qreal Q = sqrt(sqr(x*vy-y*vx) + sqr(y*vz-z*vy) + sqr(z*vx-x*vz));
    if (fabs(Q) > con_eps2)
    {
        qreal QXlift = (x*(vy*vy + vz*vz) + vx*(y*vy + z*vz)) / Q;
        qreal QYlift = (y*(vz*vz + vx*vx) + vy*(z*vz + x*vx)) / Q;
        qreal QZlift = (z*(vx*vx + vy*vy) + vz*(x*vx + y*vy)) / Q;

        this->F[0] += Delta1 * RoV * QXlift;
        this->F[1] += Delta1 * RoV * QYlift;
        this->F[2] += Delta1 * RoV * QZlift;
    }
}


//Function for alternative calculation of the height
//qreal COnePrognosis::convertGSCtoGeODeZ_hight(const qreal X, const qreal Y, const qreal Z)
//{
//    const qreal ra = 6378245.0;									// major and minor semiaxis
//    const qreal rb = 6356863.0;									// of Krassovski ellipsoid
//    const qreal ra_ra = 0.000001 * ra * ra;
//    const qreal rb_rb = 0.000001 * rb * rb;
	
//    qreal real_a = 0.001 * Z;
//	if (Z < 0)
//    {
//		real_a *= -1.0;
//    }
//    const qreal dist = sqrt((qreal)(0.000001*X*X + 0.000001*Y*Y + 0.000001*Z*Z));
//    const qreal	real_b = sqrt((qreal)(0.000001*X*X + 0.000001*Y*Y));

//    const qreal	cos_f = real_b / dist;
//    const qreal	sin_f = real_a / dist;
//	qreal	Ro = (ra_ra * rb_rb) / (cos_f * cos_f * rb_rb + sin_f * sin_f * ra_ra);
//	Ro = sqrt(Ro);
//	return	((dist - Ro) * 1000);
//}


//FUNCTION		:COnePrognosis::CalcEllipse
//
//DESCRIPTION	:Dispersion ellipse calculation
//
//INPUTS		:Structure of covariance matrix, qreal time of location and fall,
//					structures for Fall point and velocities in Fall point,
//					references for qreal ellipses characteristics
//
//RETURNS		:void
//
void COnePrognosis::CalcEllipse(const TMatrix<DIM_CV> &K, const qreal t,
                                const qreal tp,const GLPointDouble3D &SP,const  GLPointDouble3D &SV,
                                qreal &aI, qreal &bI, qreal &betaI)
{
	aI = 0;
	bI = 0;
	betaI = 0;

    if (m_pLogEll != nullptr)
    {
        tLogMatrix(m_pLogEll, "Input covariance matrix", K);
    }
	    
    const qreal tau = fabs(tp - t);//1


    TMatrix<DIM_C> K2e(DIM_C,DIM_C);
    TMatrix<DIM_C> K2eTPSC(DIM_C,DIM_C);

	//2
    const qreal sxe2 = K.M[0][0] + 2*K.M[0][3]*tau + K.M[3][3]*pow(tau,2);
    const qreal sye2 = K.M[1][1] + 2*K.M[1][4]*tau + K.M[4][4]*pow(tau,2);
    const qreal sze2 = K.M[2][2] + 2*K.M[2][5]*tau + K.M[5][5]*pow(tau,2);

    const qreal sxye = K.M[0][1] + (K.M[0][4] + K.M[1][3])*tau + K.M[3][4]*pow(tau,2);
    const qreal sxze = K.M[0][2] + (K.M[0][5] + K.M[2][3])*tau + K.M[3][5]*pow(tau,2);
    const qreal syze = K.M[1][2] + (K.M[1][5] + K.M[2][4])*tau + K.M[4][5]*pow(tau,2);

    K2e.M[0][0] = sxe2;
    K2e.M[1][1] = sye2;
    K2e.M[2][2] = sze2;
    K2e.M[0][1] = sxye;
    K2e.M[1][0] = sxye;
    K2e.M[0][2] = sxze;
    K2e.M[2][0] = sxze;
    K2e.M[1][2] = syze;
    K2e.M[2][1] = syze;//2

    CGeocentric CGC_P(SP.x, SP.y, SP.z);
    CGeocentric CGC_V(SV.x, SV.y, SV.z);
    CGeocentric CGC_P1;
    CTopocentric CTC_P;
    CTopocentric CTC_V;
    CTopocentric CTC_A;
    CGeodesic CGd;
    CGeodesic CGd1;

    bool bOK = GEOCENTRIC_GEO(&CGC_P, &CGd);

    bOK = bOK && GL_MATR::Recount_CovMatr_GeocentricToTopo(&CGd, &K2e, &K2eTPSC);
    if (m_pLogEll != nullptr)
    {
        tLogMatrix(m_pLogEll, "Covariance matrix in TPCS", K2eTPSC);
    }

    bOK = bOK && GEOCENTRIC_TOPO(&CGd, &CGC_P, &CGC_V, &CGC_V, &CTC_P, &CTC_V, &CTC_A);

	//6
    qreal beta2 = 0; qreal eps2 = 0; //,VT2;
    CalcBetaEpsilon(CTC_V, beta2, eps2);

//    if(CTC_V.m_dXt == 0)
//	{
//        if(CTC_V.m_dZt >= 0)
//            beta2 = AIR_BALL::PI/2.;
//		else
//            beta2 = 3*AIR_BALL::PI/2.;
//	}
//    else if(CTC_V.m_dZt == 0)
//	{
//        if(CTC_V.m_dXt > 0)
//			beta2 = 0;
//		else
//            beta2 = AIR_BALL::PI;
//	}
//	else
//	{
//        beta2 = atan(CTC_V.m_dZt / CTC_V.m_dXt);
//        if(CTC_V.m_dXt > 0 && CTC_V.m_dZt > 0)
//			beta2 = fabs(beta2);
//        else if(CTC_V.m_dXt < 0 && CTC_V.m_dZt > 0)
//            beta2 = AIR_BALL::PI - fabs(beta2);
//        else if(CTC_V.m_dXt > 0 && CTC_V.m_dZt < 0)
//            beta2 = 2*AIR_BALL::PI - fabs(beta2);
//		else //z<0,x<0
//            beta2 = AIR_BALL::PI + fabs(beta2);
//    }
//    if (fabs(CTC_V.m_dXt) > con_par_eps || fabs(CTC_V.m_dZt) > con_par_eps)
//        eps2 = atan(CTC_V.m_dYt / sqrt(pow(CTC_V.m_dXt,2) + pow(CTC_V.m_dZt,2)));
//	else
//	{
//        if (CTC_V.m_dYt > con_par_eps)
//            eps2 = con_pi / 2;
//        else if (CTC_V.m_dYt < con_par_eps)
//            eps2 = - con_pi / 2;
//		else
//			return;
	
//	}

    if (!bOK || (fabs(pow(CTC_V.m_dXt,2) + pow(CTC_V.m_dYt,2) + pow(CTC_V.m_dZt,2)) < con_par_eps))
    {
		return;
    }

    TMatrix<3> A(3,3);
    TMatrix<3> AT(3,3);
    TMatrix<3> K2(3,3);
    TMatrix<3> M2(3,3);
    TCMatrixFunc<3> CM3;

    A.M[0][0] = cos(-eps2)*cos(-beta2);
    A.M[0][1] = -sin(-eps2);
    A.M[0][2] = -cos(-eps2)*sin(-beta2);
    A.M[1][0] = sin(-eps2)*cos(-beta2);
    A.M[1][1] = cos(-eps2);
    A.M[1][2] = -sin(-eps2)*sin(-beta2);
    A.M[2][0] = sin(-beta2);
    A.M[2][1] = 0;
    A.M[2][2] = cos(-beta2);//7

    bOK = CM3.Transpon(A,AT); // VV_1053:Not Required

    bOK = bOK && CM3.MatrXMatr(A,K2eTPSC,M2); // VV_1056:Not Required
    bOK = bOK && CM3.MatrXMatr(M2,AT,K2);//8  // VV_1057:Not Required

    if (m_pLogEll != nullptr)
    {
        tLogMatrix(m_pLogEll, "Matrix after rotation", K2);
    }

	qreal lambda=0; // VV_1112: Corrected

    if (K2.M[1][1] < 0 || K2.M[2][2] < 0 || !bOK)
    {
		return;
    }

    const qreal sy1 = sqrt(K2.M[1][1]);
    const qreal sz1 = sqrt(K2.M[2][2]);
    const qreal syz1 = K2.M[1][2];

    if(fabs(sy1 - sz1) < con_eps2 && syz1 > 0)
    {
        lambda = PI/4.;
    }
    else if(fabs(sy1 - sz1) < con_eps2 && syz1 < 0)
    {
        lambda = -PI/4.;
    }
    else if(fabs(sy1 - sz1) < con_eps2 && fabs(syz1) < con_eps2)
    {
		lambda = 0;
    }
    else if (fabs(pow(sy1,2) - pow(sz1,2)) < con_eps2)
    {
		return;
    }
	else if(sy1 > sz1)
    {
        lambda = 0.5*atan(2.*syz1 / (pow(sy1,2) - pow(sz1,2)));
    }
	else if(sy1 < sz1 && syz1 >= 0)
    {
        lambda = 0.5*atan(2.*syz1 / (pow(sy1,2) - pow(sz1,2))) + PI/2.;
    }
	else if(sy1 < sz1 && syz1 < 0)
    {
        lambda = 0.5*atan(2.*syz1 / (pow(sy1,2) - pow(sz1,2))) - PI/2.;//9
    }
    else
    {
    }


    if ( pow(K2.M[1][1] - K2.M[2][2],2) + pow(2.*K2.M[1][2],2) < 0
        || 0.5*(K2.M[1][1] + K2.M[2][2] - sqrt(pow(K2.M[1][1] - K2.M[2][2],2) + pow(2*K2.M[1][2],2))) < 0)
    {
		return;
    }

    const qreal a = sqrt( 0.5*( K2.M[1][1] + K2.M[2][2] +
        sqrt( pow(K2.M[1][1] - K2.M[2][2],2) + pow(2.*K2.M[1][2],2) ) ) );

    const qreal b = sqrt(0.5*(K2.M[1][1] + K2.M[2][2] -
        sqrt(pow(K2.M[1][1] - K2.M[2][2],2) + pow(2.*K2.M[1][2],2))));//10

    const qreal H1p = CGd.m_dAltitude + 100000;

    CGd1.m_dLatitude = CGd.m_dLatitude;
    CGd1.m_dLongitude = CGd.m_dLongitude;
    CGd1.m_dAltitude = H1p;

    bOK = GEO_GEOCENTRIC(&CGd1, &CGC_P1);

    const qreal XG = CGC_P1.m_dX - CGC_P.m_dX;
    const qreal YG = CGC_P1.m_dY - CGC_P.m_dY;
    const qreal ZG = CGC_P1.m_dZ - CGC_P.m_dZ;//11

    if (pow(SV.x,2) + pow(SV.y,2) + pow(SV.z,2) < con_par_eps
        || XG*XG + YG*YG + ZG*ZG < con_par_eps || !bOK)
    {
		return;
    }

    const qreal cos_eta = (SV.x*XG + SV.y*YG + SV.z*ZG) /
                ( sqrt( pow(SV.x,2) + pow(SV.y,2) + pow(SV.z,2) ) *
				sqrt(XG*XG + YG*YG + ZG*ZG) );//12

	const qreal alpha = fabs(lambda);
    const qreal c = a*sin(PI/2. - alpha);
    const qreal d = a*cos(PI/2. - alpha);
    const qreal e = c/cos_eta;
    const qreal f = b*sin(alpha);
    const qreal g = b*cos(alpha);
    const qreal h = f/cos_eta;
    const qreal d_beta = atan(e/d);//13

	aI = sqrt(d*d + e*e);
	bI = sqrt(g*g + h*h);//14

	if(lambda > 0)
    {
        betaI = SB_2PI(beta2 , (PI/2. - d_beta));
    }
	else
    {
        betaI = AB_2PI(beta2 , (PI/2. - d_beta));//15
    }

	if (bI > aI)
	{
        const qreal sax0 = aI;
		aI = bI;
		bI = sax0;
        betaI = AB_2PI(betaI, PI/2.);
	}

    if (m_pLogEll != nullptr)
    {
        Log(m_pLogEll, "   Lambda %f  alpha %f  a %f  b %f  c %f  d %f  "
                       "e %f  f %f  g %f  h %f  d_beta %f  aI %f  bI %f  betaI %f",
            lambda, alpha, a, b, c, d,
            e, f, g, h, d_beta, aI, bI, betaI);
        Log(m_pLogEll, "\n");
    }
}


void COnePrognosis::CalcEllipse(const TMatrix<DIM_CV> &K, const GLPointDouble3D &SP, const GLPointDouble3D &SV, qreal &aI, qreal &bI, qreal &betaI)
{
    CalcEllipse(K, 0, 0, SP, SV, aI, bI, betaI);
}


void COnePrognosis::CalcEllipse(GLMatrix &K, const qreal t, const qreal tp, const GLPointDouble3D &SP, const GLPointDouble3D &SV, qreal &aI, qreal &bI, qreal &betaI)
{
    CMatrix fMatr;
    TMatrix<DIM_CV> K1;
    fMatr.Copy(K1, K, 0, DIM_CV-1, 0, DIM_CV-1);
    CalcEllipse(K1, t, tp, SP, SV, aI, bI, betaI);
}


void COnePrognosis::CalcBetaEpsilon(const CTopocentric &CTC_V, qreal &beta2, qreal &eps2)
{
    beta2 = 0.;
    eps2 = 0.;

    if( fabs(CTC_V.m_dXt) <= std::numeric_limits<double>::epsilon() )
    {
        if(CTC_V.m_dZt >= 0)
        {
            beta2 = con_half_pi;
        }
        else
        {
            beta2 = 3.*con_half_pi;
        }
    }
    else if( fabs(CTC_V.m_dZt) <= std::numeric_limits<double>::epsilon() )
    {
        if(CTC_V.m_dXt > 0)
        {
            beta2 = 0;
        }
        else
        {
            beta2 = con_pi;
        }
    }
    else
    {
        beta2 = atan(CTC_V.m_dZt / CTC_V.m_dXt);
        if(CTC_V.m_dXt > 0 && CTC_V.m_dZt > 0)
        {
            beta2 = fabs(beta2);
        }
        else if(CTC_V.m_dXt < 0 && CTC_V.m_dZt > 0)
        {
            beta2 = con_pi - fabs(beta2);
        }
        else if(CTC_V.m_dXt > 0 && CTC_V.m_dZt < 0)
        {
            beta2 = con_2pi - fabs(beta2);
        }
        else //z<0,x<0
        {
           beta2 = con_pi + fabs(beta2);
        }
    }

    if (fabs(CTC_V.m_dXt) > con_par_eps || fabs(CTC_V.m_dZt) > con_par_eps)
    {
        eps2 = atan(CTC_V.m_dYt / sqrt(pow(CTC_V.m_dXt,2) + pow(CTC_V.m_dZt,2)));
    }
    else
    {
        if (CTC_V.m_dYt > con_par_eps)
        {
            eps2 = con_pi / 2.;
        }
        else if (CTC_V.m_dYt < con_par_eps)
        {
            eps2 = - con_pi / 2.;
        }
        else
        {            
        }
    }
}


//void COnePrognosis::CalcEllipse(GLMatrix &K, qreal t, qreal tp, GLPointDouble3D &SP, GLPointDouble3D &SV, qreal &aI, qreal &bI, qreal &betaI)
//{
//    SPoint SP_curr;
//    SVel   SV_curr;

//    SP_curr.X = SP.x;
//    SP_curr.Y = SP.y;
//    SP_curr.Z = SP.z;

//    SV_curr.VX = SV.x;
//    SV_curr.VY = SV.y;
//    SV_curr.VZ = SV.z;

//    CalcEllipse(K, t, tp, SP_curr, SV_curr, aI, bI, betaI);
//}


void COnePrognosis::GetPolinom_4deg(Pred_Polinom &Pol)
{
    qreal arrT[5];
	qreal A[5][3];
	qreal a0[3];
	qreal a1[3];
	qreal a2[3];
	qreal a3[3];
	qreal a4[3];
    qreal X1;
    qreal Y1;
    qreal Z1;

    const qreal del_T = (Param[dim].M_info[0] - Param[0].M_info[0])/4;
    const qreal tp = (Param[dim].M_info[0] + Param[0].M_info[0])/2;

    arrT[0] = tp - 2*del_T;
    arrT[1] = tp - del_T;
    arrT[2] = tp;
    arrT[3] = tp + del_T;
    arrT[4] = tp + 2*del_T;

    qint32 k = 0;
	qint16 i;
	for(i = 0;i < 5;i++)
	{
        while( k < AIR_BALL::MaxCycleIter && Param[k].M_info[0] < arrT[i] )
		{
			k++;
		}
        if(fabs(Param[k].M_info[0] - arrT[i]) <1e-4)
		{
			X1 = Param[k].M_info[1];
			Y1 = Param[k].M_info[2];
			Z1 = Param[k].M_info[3];
		}
		else
		{
//			qreal tt1 = Param[k].M_info[0];
//			qreal tt2 = Param[k-1].M_info[0];
			X1 = Param[k-1].M_info[1] + 
                (arrT[i] - Param[k-1].M_info[0])*(Param[k].M_info[1] -
				Param[k-1].M_info[1])/(Param[k].M_info[0] - Param[k-1].M_info[0]);
			Y1 = Param[k-1].M_info[2] + 
                (arrT[i] - Param[k-1].M_info[0])*(Param[k].M_info[2] -
				Param[k-1].M_info[2])/(Param[k].M_info[0] - Param[k-1].M_info[0]);
			Z1 = Param[k-1].M_info[3] + 
                (arrT[i] - Param[k-1].M_info[0])*(Param[k].M_info[3] -
				Param[k-1].M_info[3])/(Param[k].M_info[0] - Param[k-1].M_info[0]);
		}
		A[i][0] = X1;
		A[i][1] = Y1;
		A[i][2] = Z1;
	}

	for(i=0;i<3;i++)
	{
		a0[i] = A[2][i];
		a1[i] = ( A[0][i] - A[4][i] + 8*( A[3][i] - A[1][i] ) )/(12*del_T);
		a2[i] = ( 16*( A[3][i] + A[1][i] ) - A[4][i] - A[0][i] - 30*A[2][i])/
			(24*pow(del_T,2));
		a3[i] = ( 2*( A[1][i] - A[3][i] ) + A[4][i] - A[0][i] )/(12*pow(del_T,3));
		a4[i] = (-4*(A[3][i] + A[1][i]) + A[4][i] + A[0][i] + 6*A[2][i])/
			(24*pow(del_T,4));
	}

    Pol.Part0.x = a0[0];
    Pol.Part0.y = a0[1];
    Pol.Part0.z = a0[2];

    Pol.Part1.x = a1[0];
    Pol.Part1.y = a1[1];
    Pol.Part1.z = a1[2];

    Pol.Part2.x = a2[0];
    Pol.Part2.y = a2[1];
    Pol.Part2.z = a2[2];

    Pol.Part3.x = a3[0];
    Pol.Part3.y = a3[1];
    Pol.Part3.z = a3[2];

    Pol.Part4.x = a4[0];
    Pol.Part4.y = a4[1];
    Pol.Part4.z = a4[2];

	Pol.AnchorTime = tp;

}


bool COnePrognosis::GetSignCorrectProgn()
{
    return SignCorrectProl;
}


bool COnePrognosis::GetPolinom_Ndeg(const qint16 N, std::vector<M10> &ProlTrack, const qint32 ind_begin, const qint32 ind_end,
                                    CGLArrayFix<GLPointDouble3D, SIZE_ARR_POLYNOM> &PolData, qreal &AnchorTime, qreal &FixTime)
{
    bool bOK = true;
    if (N % 2 != 0 || dim < con_par_eps || ind_begin >= ind_end) //N must be an even number
    {
        return false;
    }

    qint32 i;
    qint32 j;
    qint16 k;

    const qreal del_T = (ProlTrack[ind_end].M_info[0] - ProlTrack[ind_begin].M_info[0]) / static_cast<qreal>(N);
    if (fabs(del_T) < con_eps)
    {
        bOK = false;
    }
    else
    {
        AnchorTime = ProlTrack[ind_begin].M_info[0]; //(ProlTrack[ind_end].M_info[0] + ProlTrack[ind_begin].M_info[0]) / 2.;

        CGLArrayFix<qreal, SIZE_ARR_POLYNOM> T_poly;
        TMatrix<SIZE_ARR_POLYNOM> Coeffs_A(N+1,3);
        TMatrix<SIZE_ARR_POLYNOM> ArrT(N+1,N+1);
        TMatrix<SIZE_ARR_POLYNOM> InvArrT(N+1, N+1);
        TMatrix<SIZE_ARR_POLYNOM> Coeffs0(N+1,3);
        TCMatrixFunc<SIZE_ARR_POLYNOM> clMatrFunc;

        //    const qint32 N2 = N/2;
        //    T_poly[N2] = AnchorTime;

        //    for (k=0; k<N2; k++) //fill values of time of N+1 points
        //    {
        //        T_poly[k] = AnchorTime - (N2 - k)*del_T;
        //        T_poly[N-k] = AnchorTime + (N2 - k)*del_T;
        //    }


        for (k=0; k<=N; k++) //fill values of time of N+1 points
        {
            T_poly[k] = AnchorTime + k*del_T;
        }

        //attachment FixTime to the array of times
        if (FixTime > T_poly[0] + con_eps && FixTime < T_poly[N] - con_eps)
        {
            qint16 k_tFix = 0;
            qreal deltaT_fix = con_large_value;
            for (k=0; k<=N; k++)
            {
                qreal deltaT_curr = fabs(T_poly[k] - FixTime);
                if (deltaT_curr < deltaT_fix)
                {
                    deltaT_fix = deltaT_curr;
                    k_tFix = k;
                }
            }

            if (k_tFix == 0)
            {
                k_tFix = 1;
            }
            else
            {
                if (k_tFix == N)
                {
                    k_tFix = N-1;
                }
            }

            T_poly[k_tFix] = FixTime;
        }

        k=0;
        for (i=0; i<=N; i++) //fill values of coordinates of N+1 points
        {
            while ( k < AIR_BALL::MaxCycleIter && ProlTrack[k].M_info[0] - T_poly[i] <  -con_eps2 )
            {
                k++;
            }

            if (k==0 || fabs(ProlTrack[k].M_info[0] - T_poly[i]) < con_eps2)
            {
                for (j=0; j<=2; j++)
                {
                    Coeffs_A.M[i][j] = ProlTrack[k].M_info[j+1];
                }
            }
            else
            {
                for (j=0; j<=2; j++)
                {
                    Coeffs_A.M[i][j] = ProlTrack[k-1].M_info[j+1] + (T_poly[i] - ProlTrack[k-1].M_info[0]) *
                            (ProlTrack[k].M_info[j+1] - ProlTrack[k-1].M_info[j+1]) /
                            (ProlTrack[k].M_info[0] - ProlTrack[k-1].M_info[0]);
                }
            }
        }

        for (i=0; i<=N; i++)
        {
            for (j=0; j<=N; j++)
            {
                ArrT.M[i][j] = pow((T_poly[i] - AnchorTime) / del_T, N-j);
            }
        }

        bOK = clMatrFunc.InvMatrix(ArrT, InvArrT); //InvArrT = ArrT^-1
        bOK = bOK && clMatrFunc.MatrXMatr(InvArrT, Coeffs_A, Coeffs0); //Coeffs0 = InvArrT * Coeffs_A
        TMatrix<SIZE_ARR_POLYNOM> ForTest(N+1, N+1);
        bOK = bOK && clMatrFunc.MatrXMatr(ArrT, Coeffs0, ForTest);

        qreal Divisor = 1.;
        for (k=0; k<=N; k++)
        {
            if (k>0)
            {
                Divisor *= del_T;
            }
            if (fabs(Divisor) > con_par_eps)
            {
                PolData[k].x = Coeffs0.M[N-k][0] / Divisor;
                PolData[k].y = Coeffs0.M[N-k][1] / Divisor;
                PolData[k].z = Coeffs0.M[N-k][2] / Divisor;
            }
            else
            {
                bOK = false;
            }
        }
    }

    return bOK;
}


void COnePrognosis::GetM10_ForPoint(const qreal T, const GLPointDouble3D &Coord, const GLPointDouble3D &Vel, const qreal Gamma, M10 &Result)
{
    Result.Reset();
    Result.M_info[0] = T;
    Result.M_info[1] = Coord.x;
    Result.M_info[2] = Coord.y;
    Result.M_info[3] = Coord.z;
    Result.M_info[4] = Vel.x;
    Result.M_info[5] = Vel.y;
    Result.M_info[6] = Vel.z;

    UK[0] = Vel.x;
    UK[1] = Vel.y;
    UK[2] = Vel.z;
    UK[3] = Coord.x;
    UK[4] = Coord.y;
    UK[5] = Coord.z;

    RoK = GetDensity(Coord.x, Coord.y, Coord.z);
    VK = GetAbsVector(Vel.x, Vel.y, Vel.z);
    H0 = GetHeight(Coord.x, Coord.y, Coord.z);    
    RecalcSystemCoefs(Coord.x, Coord.y, Coord.z, CK, BK, AK);
    F_Diff_Eq(UK, AK, BK, CK, RoK, VK, Gamma);

    Result.M_info[7] = F[0];
    Result.M_info[8] = F[1];
    Result.M_info[9] = F[2];
}


bool COnePrognosis::GetPolinom(std::vector<M10> &ProlTrack, Polynoms_data &Pol)
{
    Pol.Reset();

    if (dim <= 0)
    {
        return false;
    }

    bool ItsOK;
    qint32 i;
    qreal ind_begin;
    qreal ind_end;
    CGLArrayFix<GLPointDouble3D, SIZE_ARR_POLYNOM> CurrPolData;
    qreal AnchorTime = 0.;

    //finding the apogee and the TimeChange
    qreal H_prev;
    qreal H_curr;
    qreal H_max;
    qint32 ind_apogee;
    qint32 ind_change;  //indexes at apogee and at change of polynom

    H_prev = GetHeight(ProlTrack[0].M_info[1], ProlTrack[0].M_info[2], ProlTrack[0].M_info[3]);
    H_max = H_prev;
    ind_apogee = 0;
    ind_change = 0;

    for (i=1; i<=dim; i++)
    {
        H_curr = GetHeight(ProlTrack[i].M_info[1], ProlTrack[i].M_info[2], ProlTrack[i].M_info[3]);

        if (H_curr > H_max)
        {
            H_max = H_curr;
            ind_apogee = i;
        }

        if (H_curr < H_prev) //descending
        {
            if (H_max < H_CHANGE_POLYNOM) //apogee is low
            {
                ind_change = ind_apogee;
                break;
            }
            else if (H_prev >= H_CHANGE_POLYNOM && H_curr <= H_CHANGE_POLYNOM)
            {
                ind_change = i;
                break;
            }
            else
            {
            }
        }
        H_prev = H_curr;
    }

    Pol.TimeChange = ProlTrack[ind_change].M_info[0];

    if (ind_change > 1)
    {
        //begin and end indexes for polynom of 4 degree
        ind_begin = 0;
//        ind_end = ind_change;
        ind_end = dim; //for tacking into account all array of times for 4deg polynom construction
    }
    else
    {   //simplified polynom of 4 deg in the case of descending branch in the atmosphere
        //begin and end indexes for polynom of 4 degree
        ind_begin = 0;
        ind_end = dim;
        Pol.TimeChange = ProlTrack[0].M_info[0] - con_eps2;
    }

    //calculation of polynom of 4 degree
    ItsOK = GetPolinom_Ndeg(4, ProlTrack, ind_begin, ind_end, CurrPolData, AnchorTime, Pol.TimeChange);
    if (!ItsOK)
    {
        return false;
    }
    Pol.Part0_4deg = CurrPolData[0];
    Pol.Part1_4deg = CurrPolData[1];
    Pol.Part2_4deg = CurrPolData[2];
    Pol.Part3_4deg = CurrPolData[3];
    Pol.Part4_4deg = CurrPolData[4];

    Pol.AnchorTime_4deg = AnchorTime;

    if (ind_change < dim-1)
    {
        ind_begin = ind_change;
        ind_end = dim;
        AnchorTime = 0;

        ItsOK = GetPolinom_Ndeg(8, ProlTrack, ind_begin, ind_end, CurrPolData, AnchorTime, Pol.TimeChange);
        if (!ItsOK)
        {
            return false;
        }

        Pol.Part0_8deg = CurrPolData[0];
        Pol.Part1_8deg = CurrPolData[1];
        Pol.Part2_8deg = CurrPolData[2];
        Pol.Part3_8deg = CurrPolData[3];
        Pol.Part4_8deg = CurrPolData[4];
        Pol.Part5_8deg = CurrPolData[5];
        Pol.Part6_8deg = CurrPolData[6];
        Pol.Part7_8deg = CurrPolData[7];
        Pol.Part8_8deg = CurrPolData[8];

        Pol.AnchorTime_8deg = AnchorTime;
    }
    else
    {
        Pol.TimeChange = ProlTrack[dim].M_info[0] + con_par_eps;
    }

    //polynom of 2 degree
    ind_begin = 0;
    ind_end = dim;
    CurrPolData.Reset();
    AnchorTime = 0;
    qreal t_fix2 = ProlTrack[0].M_info[0] - con_eps;
    if (ind_apogee > 0)
    {
        qreal t_0 = ProlTrack[0].M_info[0];
        qreal t_apogee = ProlTrack[ind_apogee].M_info[0];
        qreal t_end = ProlTrack[0].M_info[dim-1];
        if (t_apogee > t_0 && t_apogee - t_0 > c_CoefDT_Apogee*(t_end - t_0))
        {
            t_fix2 = t_apogee;
        }
    }

    ItsOK = GetPolinom_Ndeg(2, ProlTrack, ind_begin, ind_end, CurrPolData, AnchorTime, t_fix2);
    if (ItsOK)
    {
        Pol.Part0_2deg = CurrPolData[0];
        Pol.Part1_2deg = CurrPolData[1];
        Pol.Part2_2deg = CurrPolData[2];

        Pol.AnchorTime_2deg = AnchorTime;
    }
    else
    {
        return false;
    }

    return true;
}


bool COnePrognosis::GetPolinom(Polynoms_data &Pol)
{
    bool bRes = true;

    const qint32 Size = static_cast<int>(Param.size());
    if (Size > 0)
    {
        bRes = GetPolinom(Param, Pol);
    }
    else
    {
        Pol.Reset();
        bRes = false;
    }

    return bRes;
}


Polynoms_data COnePrognosis::GetPolynomECEF()
{
    return PolynomComb;
}


Polynoms_data COnePrognosis::GetPolynomNUE()
{
    return PolynomCombNUE;
}


bool COnePrognosis::CalcPolynom()
{
    bool bRes = GetPolinom(PolynomComb);
    return bRes;
}


bool COnePrognosis::RecalcPolynomToNUE(CGeodesic &Center, const Polynoms_data &PolECEF, Polynoms_data &PolNUE)
{
    bool bRes = true;

    SKoefTransposeMatr l_KoefTR;
    SKoef_TurnGEOCENTRICtoTOPO l_KoefTurnG_T;

    bRes = Init_GEOCENTRIC_TOPO_SHIFT(&Center, &l_KoefTR, &l_KoefTurnG_T);
    if (bRes)
    {
        bool bOK = true;

        CGeocentric CoefGeocentric;
        CTopocentric CoefTopocentric(0.,0.,0.);

        //4 degree
        CoefGeocentric.init(PolECEF.Part0_4deg.x, PolECEF.Part0_4deg.y, PolECEF.Part0_4deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(&CoefGeocentric, 0, 0, l_KoefTR, l_KoefTurnG_T, &CoefTopocentric, 0, 0); bRes = bRes && bOK;
        PolNUE.Part0_4deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part1_4deg.x, PolECEF.Part1_4deg.y, PolECEF.Part1_4deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part1_4deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part2_4deg.x, PolECEF.Part2_4deg.y, PolECEF.Part2_4deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part2_4deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part3_4deg.x, PolECEF.Part3_4deg.y, PolECEF.Part3_4deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part3_4deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part4_4deg.x, PolECEF.Part4_4deg.y, PolECEF.Part4_4deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part4_4deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        //8 degree
        CoefGeocentric.init(PolECEF.Part0_8deg.x, PolECEF.Part0_8deg.y, PolECEF.Part0_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(&CoefGeocentric, 0, 0, l_KoefTR, l_KoefTurnG_T, &CoefTopocentric, 0, 0); bRes = bRes && bOK;
        PolNUE.Part0_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part1_8deg.x, PolECEF.Part1_8deg.y, PolECEF.Part1_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part1_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part2_8deg.x, PolECEF.Part2_8deg.y, PolECEF.Part2_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part2_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part3_8deg.x, PolECEF.Part3_8deg.y, PolECEF.Part3_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part3_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part4_8deg.x, PolECEF.Part4_8deg.y, PolECEF.Part4_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part4_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part5_8deg.x, PolECEF.Part5_8deg.y, PolECEF.Part5_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part5_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part6_8deg.x, PolECEF.Part6_8deg.y, PolECEF.Part6_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part6_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part7_8deg.x, PolECEF.Part7_8deg.y, PolECEF.Part7_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part7_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        CoefGeocentric.init(PolECEF.Part8_8deg.x, PolECEF.Part8_8deg.y, PolECEF.Part8_8deg.z);
        bOK = GEOCENTRIC_TOPO_SHIFT(0, &CoefGeocentric, 0, l_KoefTR, l_KoefTurnG_T, 0, &CoefTopocentric, 0); bRes = bRes && bOK;
        PolNUE.Part8_8deg.init(CoefTopocentric.m_dXt, CoefTopocentric.m_dYt, CoefTopocentric.m_dZt);

        PolNUE.AnchorTime_4deg  = PolECEF.AnchorTime_4deg;
        PolNUE.AnchorTime_8deg  = PolECEF.AnchorTime_8deg;
        PolNUE.TimeChange       = PolECEF.TimeChange;
    }

    return bRes;
}


bool COnePrognosis::RecalcPolynomToNUE(CGeodesic &Center)
{
    return RecalcPolynomToNUE(Center, PolynomComb, PolynomCombNUE);
}


//void COnePrognosis::SetHmax(qreal NewHmax)
//{
//    Hmax = NewHmax;
//}


qreal COnePrognosis::GetHmax()
{
    return Hmax;
}


//void COnePrognosis::SetTApogee(qreal NewTApogee)
//{
//    tApogee = NewTApogee;
//}


qreal COnePrognosis::GetTApogee()
{
    return tApogee;
}


bool COnePrognosis::extrCoordByCombPolynom(const qreal _time, const Polynoms_data &Pol, GLPointDouble3D &ResPoint)
{
    bool bRes = true;
    try
    {        
        qreal DeltaT;
        if (_time > Pol.TimeChange) //polynom of 8 deg
        {
            DeltaT = _time - Pol.AnchorTime_8deg;
            const qreal DeltaT_2 = DeltaT * DeltaT;
            const qreal DeltaT_3 = DeltaT_2 * DeltaT;
            const qreal DeltaT_4 = DeltaT_3 * DeltaT;
            const qreal DeltaT_5 = DeltaT_4 * DeltaT;
            const qreal DeltaT_6 = DeltaT_5 * DeltaT;
            const qreal DeltaT_7 = DeltaT_6 * DeltaT;
            const qreal DeltaT_8 = DeltaT_7 * DeltaT;

            ResPoint =    Pol.Part0_8deg
                        + Pol.Part1_8deg * DeltaT
                        + Pol.Part2_8deg * DeltaT_2
                        + Pol.Part3_8deg * DeltaT_3
                        + Pol.Part4_8deg * DeltaT_4
                        + Pol.Part5_8deg * DeltaT_5
                        + Pol.Part6_8deg * DeltaT_6
                        + Pol.Part7_8deg * DeltaT_7
                        + Pol.Part8_8deg * DeltaT_8;
        }
        else //polynom of 4 deg
        {
            DeltaT = _time - Pol.AnchorTime_4deg;
            const qreal DeltaT_2 = DeltaT * DeltaT;
            const qreal DeltaT_3 = DeltaT_2 * DeltaT;
            const qreal DeltaT_4 = DeltaT_3 * DeltaT;

            ResPoint =    Pol.Part0_4deg
                        + Pol.Part1_4deg * DeltaT
                        + Pol.Part2_4deg * DeltaT_2
                        + Pol.Part3_4deg * DeltaT_3
                        + Pol.Part4_4deg * DeltaT_4;
        }
    }
    catch(...)
    {
        DPS_ASSERT(0);
        bRes = false;
    }
    return bRes;
}


bool COnePrognosis::extrCoordByCombPolynom(const qreal _time, GLPointDouble3D &ResPoint)
{
    return extrCoordByCombPolynom(_time, PolynomComb, ResPoint);
}


bool COnePrognosis::extrCoordByCombPolynomNUE(const qreal _time, GLPointDouble3D &ResPoint)
{
    return extrCoordByCombPolynom(_time, PolynomCombNUE, ResPoint);
}


bool COnePrognosis::extrCoordVelByCombPolynom(const qreal _time, const Polynoms_data &Pol, GLPointDouble3D &ResPoint, GLPointDouble3D &ResVel)
{
    bool bRes = true;
    try
    {        
        qreal DeltaT;
        if (_time > Pol.TimeChange) //polynom of 8 deg
        {
            DeltaT = _time - Pol.AnchorTime_8deg;
            const qreal DeltaT_2 = DeltaT * DeltaT;
            const qreal DeltaT_3 = DeltaT_2 * DeltaT;
            const qreal DeltaT_4 = DeltaT_3 * DeltaT;
            const qreal DeltaT_5 = DeltaT_4 * DeltaT;
            const qreal DeltaT_6 = DeltaT_5 * DeltaT;
            const qreal DeltaT_7 = DeltaT_6 * DeltaT;
            const qreal DeltaT_8 = DeltaT_7 * DeltaT;

            ResPoint =    Pol.Part0_8deg
                        + Pol.Part1_8deg * DeltaT
                        + Pol.Part2_8deg * DeltaT_2
                        + Pol.Part3_8deg * DeltaT_3
                        + Pol.Part4_8deg * DeltaT_4
                        + Pol.Part5_8deg * DeltaT_5
                        + Pol.Part6_8deg * DeltaT_6
                        + Pol.Part7_8deg * DeltaT_7
                        + Pol.Part8_8deg * DeltaT_8;

            ResVel =      Pol.Part1_8deg
                        + Pol.Part2_8deg * (DeltaT * AIR_BALL::c2)
                        + Pol.Part3_8deg * (DeltaT_2 * AIR_BALL::c3)
                        + Pol.Part4_8deg * (DeltaT_3 * AIR_BALL::c4)
                        + Pol.Part5_8deg * (DeltaT_4 * AIR_BALL::c5)
                        + Pol.Part6_8deg * (DeltaT_5 * AIR_BALL::c6)
                        + Pol.Part7_8deg * (DeltaT_6 * AIR_BALL::c7)
                        + Pol.Part8_8deg * (DeltaT_7 * AIR_BALL::c8);
        }
        else //polynom of 4 deg
        {
            DeltaT = _time - Pol.AnchorTime_4deg;
            const qreal DeltaT_2 = DeltaT * DeltaT;
            const qreal DeltaT_3 = DeltaT_2 * DeltaT;
            const qreal DeltaT_4 = DeltaT_3 * DeltaT;

            ResPoint =    Pol.Part0_4deg
                        + Pol.Part1_4deg * DeltaT
                        + Pol.Part2_4deg * DeltaT_2
                        + Pol.Part3_4deg * DeltaT_3
                        + Pol.Part4_4deg * DeltaT_4;

            ResVel =      Pol.Part1_4deg
                        + Pol.Part2_4deg * (DeltaT * AIR_BALL::c2)
                        + Pol.Part3_4deg * (DeltaT_2 * AIR_BALL::c3)
                        + Pol.Part4_4deg * (DeltaT_3 * AIR_BALL::c4);
        }
    }
    catch(...)
    {
        DPS_ASSERT(0);
        bRes = false;
    }
    return bRes;

}


bool COnePrognosis::extrCoordVelByCombPolynom(const qreal _time, GLPointDouble3D &ResPoint, GLPointDouble3D &ResVel)
{
    return extrCoordVelByCombPolynom(_time, PolynomComb, ResPoint, ResVel);
}


bool COnePrognosis::extrCoordVelByCombPolynomNUE(const qreal _time, GLPointDouble3D &ResPoint, GLPointDouble3D &ResVel)
{
    return extrCoordVelByCombPolynom(_time, PolynomCombNUE, ResPoint, ResVel);
}


void COnePrognosis::GetTPSCPolinom(const std::vector<M10> P,const CGeodesic &Ctpsc, Pred_Polinom &TPol)
{
    qreal arrT[3];
	qreal A[3][3];
	qreal a0[3];
	qreal a1[3];
	qreal a2[3];
	qint16 i;
//	qreal t1,t2;
//	t1 = P[0].M_info[0];
//	t2 = P[dim].M_info[0];
    const qreal del_T = (P[dim].M_info[0] - P[0].M_info[0])/2;
    const qreal tp = (P[dim].M_info[0] + P[0].M_info[0])/2;

    arrT[0] = P[0].M_info[0];
    arrT[1] = P[dim].M_info[0] - del_T;
    arrT[2] = P[dim].M_info[0];

	//to tpsc!!!!!
	
//    AIR_BALL::CWGStoTOPO	CWT;
//    AIR_BALL::_GEO				Gcenter;
//    AIR_BALL::_WGS				PointW;
//    AIR_BALL::SPEED_WGS			SpeedW;
//    AIR_BALL::ACCELERATION_WGS	AccelW;
//    AIR_BALL::TOPO				PointT,PointTK,PointTK1;
//    AIR_BALL::SPEED_TOPO			SpeedT;
//    AIR_BALL::ACCELERATION_TOPO	AccelT;
    CGeodesic Gcenter;
    CGeocentric PointW;
    CGeocentric SpeedW;
    CGeocentric AccelW;
    CTopocentric PointT;
    CTopocentric SpeedT;
    CTopocentric AccelT;
    CTopocentric PointTK;
    CTopocentric PointTK1;
//    Gcenter.m_dLatitude	= Ctpsc.B*180./PI;
//    Gcenter.m_dLongitude	= Ctpsc.L*180./PI;
//    Gcenter.m_dAltitude	= Ctpsc.H;
    Gcenter = Ctpsc;
//	CWT.InitWGStoTOPO(&Gcenter);
	
    PointW.m_dX	= P[0].M_info[1];
    PointW.m_dY	= P[0].M_info[2];
    PointW.m_dZ	= P[0].M_info[3];
    SpeedW.m_dX	= P[0].M_info[4];
    SpeedW.m_dY	= P[0].M_info[5];
    SpeedW.m_dZ	= P[0].M_info[6];
    AccelW.m_dX	= P[0].M_info[7];
    AccelW.m_dY	= P[0].M_info[8];
    AccelW.m_dZ	= P[0].M_info[9];
	
	// VV_1030:Not Required
//	CWT.RecountWGStoTOPO(&PointW, &PointT, &SpeedT, &AccelT, &SpeedW, &AccelW);
    bool bOK = GEOCENTRIC_TOPO(&Gcenter, &PointW, &SpeedW, &AccelW, &PointT, &SpeedT, &AccelT);

    A[0][0] = PointT.m_dXt;
    A[0][1] = PointT.m_dYt;
    A[0][2] = PointT.m_dZt;

    PointW.m_dX	= P[dim].M_info[1];
    PointW.m_dY	= P[dim].M_info[2];
    PointW.m_dZ	= P[dim].M_info[3];
    SpeedW.m_dX	= P[dim].M_info[4];
    SpeedW.m_dY	= P[dim].M_info[5];
    SpeedW.m_dZ	= P[dim].M_info[6];
    AccelW.m_dX	= P[dim].M_info[7];
    AccelW.m_dY	= P[dim].M_info[8];
    AccelW.m_dZ	= P[dim].M_info[9];
	  
//	CWT.RecountWGStoTOPO(&PointW, &PointT, &SpeedT, &AccelT, &SpeedW, &AccelW);
    bOK = bOK && GEOCENTRIC_TOPO(&Gcenter, &PointW, &SpeedW, &AccelW, &PointT, &SpeedT, &AccelT);

    A[2][0] = PointT.m_dXt;
    A[2][1] = PointT.m_dYt;
    A[2][2] = PointT.m_dZt;
	
    const qint32 k1 = (qint32)(dim/2);
//	qreal t3 = Param[k1].M_info[0];
    if(fabs(Param[k1].M_info[0] - arrT[1]) <1e-4)
	{
        PointW.m_dX	= P[k1].M_info[1];
        PointW.m_dY	= P[k1].M_info[2];
        PointW.m_dZ	= P[k1].M_info[3];
        SpeedW.m_dX	= P[k1].M_info[4];
        SpeedW.m_dY	= P[k1].M_info[5];
        SpeedW.m_dZ	= P[k1].M_info[6];
        AccelW.m_dX	= P[k1].M_info[7];
        AccelW.m_dY	= P[k1].M_info[8];
        AccelW.m_dZ	= P[k1].M_info[9];
		
//		CWT.RecountWGStoTOPO(&PointW, &PointT, &SpeedT, &AccelT, &SpeedW, &AccelW);
        bOK = bOK && GEOCENTRIC_TOPO(&Gcenter, &PointW, &SpeedW, &AccelW, &PointT, &SpeedT, &AccelT);
		
        A[1][0] = PointT.m_dXt;
        A[1][1] = PointT.m_dYt;
        A[1][2] = PointT.m_dZt;

	}
	else
	{
        const qint32 k = k1;
		i = 1;
//		qreal tt1 = P[k+1].M_info[0];
//		qreal tt2 = P[k].M_info[0];


        PointW.m_dX	= P[k].M_info[1];
        PointW.m_dY	= P[k].M_info[2];
        PointW.m_dZ	= P[k].M_info[3];
        SpeedW.m_dX	= P[k].M_info[4];
        SpeedW.m_dY	= P[k].M_info[5];
        SpeedW.m_dZ	= P[k].M_info[6];
        AccelW.m_dX	= P[k].M_info[7];
        AccelW.m_dY	= P[k].M_info[8];
        AccelW.m_dZ	= P[k].M_info[9];
		
//		CWT.RecountWGStoTOPO(&PointW, &PointTK, &SpeedT, &AccelT, &SpeedW, &AccelW); // VV_1069:Not Required
        bOK = bOK && GEOCENTRIC_TOPO(&Gcenter, &PointW, &SpeedW, &AccelW, &PointTK, &SpeedT, &AccelT);

        PointW.m_dX	= P[k+1].M_info[1];
        PointW.m_dY	= P[k+1].M_info[2];
        PointW.m_dZ	= P[k+1].M_info[3];
        SpeedW.m_dX	= P[k+1].M_info[4];
        SpeedW.m_dY	= P[k+1].M_info[5];
        SpeedW.m_dZ	= P[k+1].M_info[6];
        AccelW.m_dX	= P[k+1].M_info[7];
        AccelW.m_dY	= P[k+1].M_info[8];
        AccelW.m_dZ	= P[k+1].M_info[9];
		
//		CWT.RecountWGStoTOPO(&PointW, &PointTK1, &SpeedT, &AccelT, &SpeedW, &AccelW); // VV_1080:Not Required
        bOK = bOK && GEOCENTRIC_TOPO(&Gcenter, &PointW, &SpeedW, &AccelW, &PointTK1, &SpeedT, &AccelT);

		/*A[1][0] = P[k].M_info[1] + 
			(T[i] - P[k].M_info[0])*(P[k+1].M_info[1] - 
			P[k].M_info[1])/(P[k+1].M_info[0] - P[k].M_info[0]);
		A[1][1] = P[k].M_info[2] + 
			(T[i] - P[k].M_info[0])*(P[k+1].M_info[2] - 
			P[k].M_info[2])/(P[k+1].M_info[0] - P[k].M_info[0]);
		A[1][2] = P[k].M_info[3] + 
			(T[i] - P[k].M_info[0])*(P[k+1].M_info[3] - 
			P[k].M_info[3])/(P[k+1].M_info[0] - P[k].M_info[0]);*/

        A[1][0] = PointTK.m_dXt +
            (arrT[i] - P[k].M_info[0])*(PointTK1.m_dXt -
            PointTK.m_dXt)/(P[k+1].M_info[0] - P[k].M_info[0]);
        A[1][1] = PointTK.m_dYt +
            (arrT[i] - P[k].M_info[0])*(PointTK1.m_dYt -
            PointTK.m_dYt)/(P[k+1].M_info[0] - P[k].M_info[0]);
        A[1][2] = PointTK.m_dZt +
            (arrT[i] - P[k].M_info[0])*(PointTK1.m_dZt -
            PointTK.m_dZt)/(P[k+1].M_info[0] - P[k].M_info[0]);
	}

	for(i=0;i<3;i++)
	{
		a2[i] = (A[2][i] - 2*A[1][i] + A[0][i])/
            (arrT[2]*arrT[2] - 2*arrT[1]*arrT[1] + arrT[0]*arrT[0]);
        a1[i] = (A[1][i] - A[0][i])/del_T - a2[i]*(arrT[1] + arrT[0]);
        a0[i] = A[0][i] - arrT[0]*arrT[0]*a2[i] - arrT[0]*a1[i];
	}

    TPol.Part0.x = a0[0];
    TPol.Part0.y = a0[1];
    TPol.Part0.z = a0[2];

    TPol.Part1.x = a1[0];
    TPol.Part1.y = a1[1];
    TPol.Part1.z = a1[2];

    TPol.Part2.x = a2[0];
    TPol.Part2.y = a2[1];
    TPol.Part2.z = a2[2];

    TPol.Part3.x = 0;
    TPol.Part3.y = 0;
    TPol.Part3.z = 0;

    TPol.Part4.x = 0;
    TPol.Part4.y = 0;
    TPol.Part4.z = 0;

	TPol.AnchorTime = tp;
}


void COnePrognosis::TransmitPointerToTheLogFiles(FILE *_pLogEll)
{
    m_pLogEll = _pLogEll;
}
