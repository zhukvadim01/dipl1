#include <math.h>
#include <fstream>
#include <iomanip>

#include "gl/GLMath.h"
#include "gl/Constants.h"

#include "CBallSubclassDetermination.h"
#include "air_ballist_literal.h"
#include "air_ballist_constants.h"

// using namespace std;

#ifdef KEEP_LOG
namespace AIR_BALL{
    static std::ofstream out_TType_info(qPrintable(QString("%1/TrajTypeDet_Info.txt").arg(log_path)));
    static std::ofstream out_BMMark_info(qPrintable(QString("%1/BMMark_Info.txt").arg(log_path)));
}
#endif //KEEP_LOG


CBallSubclassDetermination::CBallSubclassDetermination()
{
//#ifdef KEEP_LOG
//    AIR_BALL::out_info_subclass << "Constructor: CBallSubclassDetermination  " << this << std::endl;
//#endif //KEEP_LOG

    m_arTTypeData.resize(AIR_BALL::BALL_FORM_AMOUNT);
    m_arMarkData.resize(AIR_BALL::BALL_FORM_AMOUNT);
//    Reset();
}


CBallSubclassDetermination::~CBallSubclassDetermination()
{
//#ifdef KEEP_LOG
//    AIR_BALL::out_info_subclass << "Destructor: CBallSubclassDetermination  " << this << std::endl;
//#endif //KEEP_LOG
}


void CBallSubclassDetermination::Reset()
{
    m_pTTypeInpData = nullptr;
    m_pMarkInpData  = nullptr;

    ClearInnerData_Subcl();
    ClearInnerData_TType();

    m_ar_BM_Diapasons.Reset();

//#ifdef KEEP_LOG
//    AIR_BALL::out_info_subclass << "Reset CBallSubclassDetermination:  " << this << std::endl;
//#endif
}


void CBallSubclassDetermination::FillInputData_TType(TrajType_InputData &InData)
{
    m_pTTypeInpData = &InData;
}


void CBallSubclassDetermination::FillInputData_BTMark(BTMark_InputData &InData)
{
    m_pMarkInpData = &InData;
}


void CBallSubclassDetermination::TrajTypeDet_Realization()
{    
    const qint32 BallIndex = m_pTTypeInpData->IndBall;

    m_OutTrajType.P_rewrite_TT = false;

    bool p_check = false; //sign of check
    if (fabs(m_pTTypeInpData->tLoc - m_arTTypeData[BallIndex].t_prev_TType_check) > cTTypeRecount) //more than cTTypeRecount s passed after last TType determination
    {
        p_check = true;
    }

    if ( (m_arTTypeData[BallIndex].PrTTypeD && !p_check)
           || fabs(m_pTTypeInpData->tLoc - m_arTTypeData[BallIndex].t_prev_TType_calc) < dT_TType )
    {
        return;
    }

    qint16 currTType = AIR_BALL::UNDEFINED_TRAJ_TYPE;

    const qreal A = fabs(m_pTTypeInpData->VH*m_pTTypeInpData->VH/(2*m_pTTypeInpData->AH)) + m_pTTypeInpData->H;
    const qreal Ac_pred = m_arTTypeData[BallIndex].Ac;
    const qreal Ac = (1-KsiAh)*A + KsiAh*Ac_pred;
    const qreal delta_Ac = fabs((Ac-Ac_pred)/(std::max(std::min(Ac, Ac_pred), epsAh)));
    m_arTTypeData[BallIndex].Ac = Ac;

#ifdef KEEP_LOG
    std::streamsize ss = AIR_BALL::out_TType_info.precision();
    AIR_BALL::out_TType_info << std::setprecision(10) << "IndBall = " << BallIndex << "\t| H = " << m_pTTypeInpData->H
            << "\tVH = " << m_pTTypeInpData->VH  << "\t AH = " << m_pTTypeInpData->AH
            << " |\tA = " << A << "\tAc = " << Ac << "\tAc_pred = " << Ac_pred
            << "\tdelta_Ac = " << delta_Ac << "\tD = " << m_pTTypeInpData->D;
#endif //KEEP_LOG

    if (delta_Ac < kAh && Ac_pred > con_par_eps
            && m_pTTypeInpData->D > con_par_eps)
    {
        //trajectory type determination using information about apogee
        const qreal lambda = Ac/pow(m_pTTypeInpData->D, sAh);
        if (lambda < lam1Ah)
        {
            currTType = AIR_BALL::FLAT;
        }
        else if (lambda < lam2Ah)
        {
            currTType = AIR_BALL::OPTIMAL;
        }
        else
        {
           currTType = AIR_BALL::LOFTED;
        }
#ifdef KEEP_LOG
        AIR_BALL::out_TType_info << "\tlambda = " << lambda << "\tTType = " << currTType;
#endif //KEEP_LOG
    }
    else
    {
        //trajectory type determination using information about throw angle
        if (m_pTTypeInpData->bound_flat_tr > con_par_eps
                && m_pTTypeInpData->bound_lofted_tr > con_par_eps
                && m_pTTypeInpData->throw_angle > con_par_eps)
        {
            if (m_pTTypeInpData->throw_angle < m_pTTypeInpData->bound_flat_tr)
            {
                currTType = AIR_BALL::FLAT;
            }
            else if (m_pTTypeInpData->throw_angle < m_pTTypeInpData->bound_lofted_tr)
            {
                currTType = AIR_BALL::OPTIMAL;
            }
            else
            {
                currTType = AIR_BALL::FLAT;
            }
        }
#ifdef KEEP_LOG
        AIR_BALL::out_TType_info << "\tthrow_angle = " << m_pTTypeInpData->throw_angle << "\tTType = " << currTType;
#endif //KEEP_LOG
    }

#ifdef KEEP_LOG
    AIR_BALL::out_TType_info << std::endl;
    AIR_BALL::out_TType_info.precision(ss);
#endif //KEEP_LOG

    if (currTType != AIR_BALL::UNDEFINED_TRAJ_TYPE)
    {
        if (p_check && m_arTTypeData[BallIndex].TType == currTType)
        {
            m_arTTypeData[BallIndex].t_prev_TType_check = m_pTTypeInpData->tLoc;
        }
        else
        {
            if ( (m_arTTypeData[BallIndex].prevTType == AIR_BALL::UNDEFINED_TRAJ_TYPE)
                    || currTType == m_arTTypeData[BallIndex].prevTType)
            {
                m_arTTypeData[BallIndex].counterTType++;
            }
            else
            {
                m_arTTypeData[BallIndex].counterTType = 0;
            }

            if (m_arTTypeData[BallIndex].counterTType >= NminTType)
            {
                m_OutTrajType.P_rewrite_TT = true;

                m_arTTypeData[BallIndex].PrTTypeD = true;
                m_arTTypeData[BallIndex].TType = currTType;
                m_OutTrajType.TrajType = currTType;

                m_arTTypeData[BallIndex].t_prev_TType_calc = m_pTTypeInpData->tLoc;
                m_arTTypeData[BallIndex].prevTType = currTType;
            }
        }
    }

}


void CBallSubclassDetermination::BMMarkDet_Realization()
{
    if (m_pMarkInpData->IndBall < 0 || m_pMarkInpData->IndBall >= AIR_BALL::BALL_FORM_AMOUNT
            || m_pMarkInpData->pTrParameters == nullptr)
    {
        return;
    }

    bool p_calc_mark=false;   //signs of calculation and check of the mark
    bool p_check_mark=false;  //signs of calculation and check of the mark

    const qint32 BallIndex = m_pMarkInpData->IndBall;

    m_OutMark.Reset();

    if (fabs(m_pMarkInpData->tLoc - m_arMarkData[BallIndex].PrevTLoc) > con_eps)
    {
        m_arMarkData[BallIndex].PrevTLoc = m_pMarkInpData->tLoc;
        const qreal H_lim = CalculateLimitingHeight(); //conditional height of atmosphere to restrict definition of mark

        if (/*!m_arMarkData[CurIndex].BMMark.IsSingleValued*/
                !m_arMarkData[BallIndex].SignFinalDecision)
        {
            if ( m_pMarkInpData->Branch == AIR_BALL::ASCENDING_BRANCH
                 || m_pMarkInpData->pTrParameters->H > H_lim
                 || !m_arMarkData[BallIndex].SignConfirmedMin
                 || !m_arMarkData[BallIndex].SignConfirmedMax)
            {
                p_calc_mark = true;
            }
        }
        else
        {
            if (m_arMarkData[BallIndex].SignConfrimation
                    || ( m_pMarkInpData->tLoc - m_arMarkData[BallIndex].tCheck > PERIOD_MARK_CHECK
                        && m_pMarkInpData->tFall - m_pMarkInpData->tLoc > DT_REFUSE_MARK_RECOUNT
                        && m_pMarkInpData->Branch != AIR_BALL::DESCENDING_BRANCH))
            {
                p_check_mark = true;
            }

            qreal dDmax = GetNominalRange(m_arMarkData[BallIndex].BMMark.maxBMMark);
            qreal dDmin = GetNominalRange(m_arMarkData[BallIndex].BMMark.minBMMark);
            if ((m_pMarkInpData->D > dDmax * K_NomDistance + m_pMarkInpData->SigmaSP
                    || m_pMarkInpData->D < dDmin * K_NomMinDistance - m_pMarkInpData->SigmaSP)
                && m_pMarkInpData->tFall - m_pMarkInpData->tLoc > DT_REFUSE_MARK_RECOUNT)
            {
                p_check_mark = true;
            }
        }

#ifdef KEEP_LOG
        std::streamsize ss = AIR_BALL::out_BMMark_info.precision();
        AIR_BALL::out_BMMark_info << std::setprecision(10) << m_pMarkInpData->tLoc << "\t" << BallIndex << "\t"
                                  << ((qint16)p_calc_mark) << "\t" << ((qint16)p_check_mark) << "\t";
#endif //KEEP_LOG

        if (p_calc_mark || p_check_mark)
        {
            CalculateAndCheckMark(p_calc_mark);
        }

        m_OutMark.BMMark = m_arMarkData[BallIndex].BMMark;

        if (!m_arMarkData[BallIndex].BMMark.IsSingleValued)
        {
            //output of confirmed values of minimal and maximal mark
            if ( (m_arMarkData[BallIndex].MinMark_Confirmed > AIR_BALL::UNDEFINED_BM_MARK)
                 && (m_arMarkData[BallIndex].MinMark_Confirmed != m_OutMark.BMMark.minBMMark))
            {
                m_OutMark.BMMark.minBMMark = m_arMarkData[BallIndex].MinMark_Confirmed;
            }

            if ((m_arMarkData[BallIndex].MaxMark_Confirmed > AIR_BALL::UNDEFINED_BM_MARK)
                    && (m_arMarkData[BallIndex].MaxMark_Confirmed != m_OutMark.BMMark.maxBMMark))
            {
                m_OutMark.BMMark.maxBMMark = m_arMarkData[BallIndex].MaxMark_Confirmed;
            }

            m_OutMark.BMMark.MarkValue = m_OutMark.BMMark.maxBMMark;
        }

#ifdef KEEP_LOG
        AIR_BALL::out_BMMark_info << "Mark is changed? " << ((qint16)m_OutMark.P_MarkIsChanged) << "\t"
                                  << "Final decision? " << m_arMarkData[BallIndex].SignFinalDecision << "\t"
                                  << "Need confirmation? " << m_arMarkData[BallIndex].SignConfrimation << "\t"
                                  << "Begin of confirmation: " << m_arMarkData[BallIndex].tBeginConfirmation << "\t"
                                  << "Number of confirmations: " << m_arMarkData[BallIndex].n_Confirmation << "\t"
                                  << "Time of check: " << m_arMarkData[BallIndex].tCheck << "\t"
                                  << "Min is confirmed? " << m_arMarkData[BallIndex].SignConfirmedMin << "\t"
                                  << "Max is confirmed? " << m_arMarkData[BallIndex].SignConfirmedMax << "\t"
                                  << "Begin min confirm.: " << m_arMarkData[BallIndex].tBeginConfirm_min << "\t"
                                  << "Begin max confirm.: " << m_arMarkData[BallIndex].tBeginConfirm_max << "\t"
                                  << "Numb. min confirm.: " << m_arMarkData[BallIndex].n_Confirm_min << "\t"
                                  << "Numb. max confirm.: " << m_arMarkData[BallIndex].n_Confirm_max << "\t"
                                  << "Confirmed min: " << m_arMarkData[BallIndex].MinMark_Confirmed << "\t"
                                  << "Confirmed max: " << m_arMarkData[BallIndex].MaxMark_Confirmed << "\t"
                                  << "Lim. height: " << H_lim << "\t"
                                  << m_arMarkData[BallIndex].BMMark.IsSingleValued << "\t"
                                  << m_arMarkData[BallIndex].BMMark.MarkValue << "\t"
                                  << m_arMarkData[BallIndex].BMMark.minBMMark << "\t"
                                  << m_arMarkData[BallIndex].BMMark.maxBMMark << "\t" << std::endl;
        AIR_BALL::out_BMMark_info.precision(ss);
#endif //KEEP_LOG
    }
}


void CBallSubclassDetermination::FillHeBoundaryValues(Cells_BMMark_Data *pArrCells)
{
    if (pArrCells != nullptr)
    {
        m_ar_BM_Diapasons.Reset();
        for (qint32 diapason = 0; diapason < 4; diapason++)
        {
            qint32 nMin, nMax;
            nMax = Cells_H_He_Bounds[diapason] / Cells_sizes[diapason];
            if (diapason == 0)
            {
                nMin = 1;
            }
            else
            {
                nMin = Cells_H_He_Bounds[diapason-1] / Cells_sizes[diapason] + 1;
            }

            for (qint32 i=1; i<=nMax; i++)
            {
//                qreal Hmax = static_cast<qreal>(i) * Cells_sizes[diapason];
//                qreal Hmin = static_cast<qreal>(i-1) * Cells_sizes[diapason] + con_eps + con_eps;

                qint32 jMin = (i >= nMin ? 1 : nMin);
                for(qint32 j=jMin; j<=nMax; j++)
                {
                    qreal HEmax = static_cast<qreal>(j) * Cells_sizes[diapason];
                    qreal HEmin = static_cast<qreal>(j-1) * Cells_sizes[diapason] + con_eps + con_eps;

                    qint32 CellNum = CalculateCellNumber(i, j ,diapason);
                    qint32 Code_ListBMMarks = pArrCells->ArrCells[CellNum-1];

                    BMMark_AndDiapason BMMarkCur;
                    GetBMMarkByCode(Code_ListBMMarks, BMMarkCur);

                    for (qint16 MarkCur = BMMarkCur.minBMMark; MarkCur <= BMMarkCur.maxBMMark; MarkCur++)
                    {
                        if (MarkCur > AIR_BALL::UNKNOWN_OF_SHORT_RANGE
                                && MarkCur < AIR_BALL::UNKNOWN_OF_LONG_RANGE)
                        {
                            if (fabs(m_ar_BM_Diapasons[MarkCur].HeMax) < con_eps
                                    || m_ar_BM_Diapasons[MarkCur].HeMax < HEmax)
                            {
                                m_ar_BM_Diapasons[MarkCur].HeMax = HEmax;
                            }

                            if (fabs(m_ar_BM_Diapasons[MarkCur].HeMin) < con_eps
                                    || m_ar_BM_Diapasons[MarkCur].HeMin > HEmin)
                            {
                                m_ar_BM_Diapasons[MarkCur].HeMin = HEmin;
                            }
                        }
                    }
                }
            }
        }
    }
}


void CBallSubclassDetermination::CalculateAndCheckMark(const bool p_calc_mark)
{
    const qint32 BallIndex = m_pMarkInpData->IndBall;

    BMMark_AndDiapason BMMark;

    OnceMarkDet(BMMark);

    if (BMMark.IsSingleValued)
    {
        if (BMMark.MarkValue == m_arMarkData[BallIndex].BMMark.MarkValue)
        {
            if(fabs(m_arMarkData[BallIndex].tBeginConfirmation - AIR_BALL::Timer_INI) < con_par_eps)
            {
                m_arMarkData[BallIndex].tBeginConfirmation = m_pMarkInpData->tLoc;
            }

            m_arMarkData[BallIndex].n_Confirmation ++;

            if (m_arMarkData[BallIndex].n_Confirmation > NUMB_MARK_CONFIRM
                    && m_pMarkInpData->tLoc - m_arMarkData[BallIndex].tBeginConfirmation > PERIOD_MARK_CONFIRM)
            {
                //Mark is confirmed
                m_arMarkData[BallIndex].SignConfrimation = false;
                m_arMarkData[BallIndex].tCheck = m_pMarkInpData->tLoc;
                m_arMarkData[BallIndex].SignFinalDecision = true;

                //save MarkValue also as confirmed minimal and maximal mark values
                m_arMarkData[BallIndex].SignConfirmedMin = true;
                m_arMarkData[BallIndex].SignConfirmedMax = true;
                m_arMarkData[BallIndex].MinMark_Confirmed = BMMark.MarkValue;
                m_arMarkData[BallIndex].MaxMark_Confirmed = BMMark.MarkValue;

                if (m_arMarkData[BallIndex].BMMark.minBMMark != BMMark.minBMMark
                        || m_arMarkData[BallIndex].BMMark.maxBMMark != BMMark.maxBMMark
                        || m_arMarkData[BallIndex].BMMark.MarkValue != BMMark.MarkValue)
                {
                    m_OutMark.P_MarkIsChanged = true;
                }

                m_arMarkData[BallIndex].BMMark = BMMark;
            }
        }
        else
        {
            if (p_calc_mark)
            {
                m_arMarkData[BallIndex].tCheck = m_pMarkInpData->tLoc;
            }

            m_arMarkData[BallIndex].BMMark = BMMark;

            m_arMarkData[BallIndex].SignConfrimation = true;
            m_arMarkData[BallIndex].tBeginConfirmation = AIR_BALL::Timer_INI;
            m_arMarkData[BallIndex].n_Confirmation = 1;

            m_OutMark.P_MarkIsChanged = true;
        }
    }
    else
    {
        m_arMarkData[BallIndex].SignConfrimation = true;

        bool SignRewriteMark = false;

        if (m_arMarkData[BallIndex].BMMark.IsSingleValued)
        {
            m_arMarkData[BallIndex].tBeginConfirmation = AIR_BALL::Timer_INI;
            m_arMarkData[BallIndex].n_Confirmation = 0;

            if (m_arMarkData[BallIndex].BMMark.MarkValue < BMMark.minBMMark
                    || m_arMarkData[BallIndex].BMMark.MarkValue > BMMark.maxBMMark)
            {
                m_arMarkData[BallIndex].SignFinalDecision = false;
                SignRewriteMark = true;
            }
        }
        else //previous mark is not single valued
        {
            //if minimal mark is already confirmed, assume that minimal mark can only increase
            if ((m_arMarkData[BallIndex].MinMark_Confirmed == AIR_BALL::UNDEFINED_BM_MARK)
                    || (BMMark.minBMMark > m_arMarkData[BallIndex].MinMark_Confirmed
                        && BMMark.minBMMark <= m_arMarkData[BallIndex].MaxMark_Confirmed))
            {
                //confirmation of minimal mark
                if (m_arMarkData[BallIndex].BMMark.minBMMark == BMMark.minBMMark)
                {
                    if (fabs(m_arMarkData[BallIndex].tBeginConfirm_min - AIR_BALL::Timer_INI) < con_par_eps)
                    {
                        m_arMarkData[BallIndex].tBeginConfirm_min = m_pMarkInpData->tLoc;
                    }

                    m_arMarkData[BallIndex].n_Confirm_min ++;

                    if (m_arMarkData[BallIndex].n_Confirm_min > NUMB_MARK_CONFIRM
                            && m_pMarkInpData->tLoc - m_arMarkData[BallIndex].tBeginConfirm_min > PERIOD_MARK_CONFIRM)
                    {
                        m_arMarkData[BallIndex].SignConfirmedMin = true;
                        m_arMarkData[BallIndex].MinMark_Confirmed = BMMark.minBMMark;
                    }
                }
                else
                {
                    m_arMarkData[BallIndex].tBeginConfirm_min = AIR_BALL::Timer_INI;
                    m_arMarkData[BallIndex].n_Confirm_min = 0;
                }
            }

            //if maximal mark is already confirmed, assume that maximal mark can only decrease
            if ((m_arMarkData[BallIndex].MaxMark_Confirmed == AIR_BALL::UNDEFINED_BM_MARK)
                    || (BMMark.maxBMMark < m_arMarkData[BallIndex].MaxMark_Confirmed
                        && BMMark.maxBMMark >= m_arMarkData[BallIndex].MinMark_Confirmed))
            {
                //confirmation of maximal mark
                if (m_arMarkData[BallIndex].BMMark.maxBMMark == BMMark.maxBMMark)
                {
                    if (fabs(m_arMarkData[BallIndex].tBeginConfirm_max - AIR_BALL::Timer_INI) < con_par_eps)
                    {
                        m_arMarkData[BallIndex].tBeginConfirm_max = m_pMarkInpData->tLoc;
                    }

                    m_arMarkData[BallIndex].n_Confirm_max ++;

                    if (m_arMarkData[BallIndex].n_Confirm_max > NUMB_MARK_CONFIRM
                            && m_pMarkInpData->tLoc - m_arMarkData[BallIndex].tBeginConfirm_max > PERIOD_MARK_CONFIRM)
                    {
                        m_arMarkData[BallIndex].SignConfirmedMax = true;
                        m_arMarkData[BallIndex].MaxMark_Confirmed = BMMark.maxBMMark;
                    }
                }
                else
                {
                    m_arMarkData[BallIndex].tBeginConfirm_max = AIR_BALL::Timer_INI;
                    m_arMarkData[BallIndex].n_Confirm_max = 0;
                }
            }
        }

        if (p_calc_mark || SignRewriteMark)
        {
            if (m_arMarkData[BallIndex].BMMark.minBMMark != BMMark.minBMMark
                    || m_arMarkData[BallIndex].BMMark.maxBMMark != BMMark.maxBMMark
                    || m_arMarkData[BallIndex].BMMark.MarkValue != BMMark.MarkValue)
            {
                m_OutMark.P_MarkIsChanged = true;
            }

            m_arMarkData[BallIndex].BMMark = BMMark;
        }
    }
}


void CBallSubclassDetermination::Drop_BallTrack(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        ClearInnerData_Subcl_1Track(ind_Ball);
        ClearInnerData_TType_1Track(ind_Ball);
    }
}


//void CBallSubclassDetermination::Drop_ST(qint32 ind_ST)
//{
//    ;
//}


void CBallSubclassDetermination::OnceMarkDet(BMMark_AndDiapason &BMMark)
{
    BMMark.Reset();

    MarkDet_H_He(BMMark);

    if (BMMark.IsEmpty())
    {
        MarkDet_He(BMMark);
    }

    if (!BMMark.IsSingleValued)
    {
        if (fabs(m_pMarkInpData->Theta) > con_par_eps)
        {
            MarkCorr_Theta_He(BMMark);

            if (!BMMark.IsSingleValued)
            {
                if (fabs(m_pMarkInpData->Veap) > con_par_eps)
                {
                    MarkCorr_Theta_Veap(BMMark);
                }
            }
        }
    }

//    if (!BMMark.IsSingleValued)
//    {
//        MarkCorr_Distance(BMMark);
//    }

    MarkCorr_Distance(BMMark);

}


void CBallSubclassDetermination::MarkDet_H_He(BMMark_AndDiapason &BMMark)
{
    if (m_pMarkInpData == nullptr)
    {
        return;
    }
    if (m_pMarkInpData->Cells_BMMark == nullptr)
    {
        return;
    }
    if (m_pMarkInpData->pTrParameters == nullptr)
    {
        return;
    }

    bool flag_isOk;

    TMatrix<2> J_H_He(2,2);
    flag_isOk = true;
    TMatrix<2> J_H_He_Tr(2,2);
    TMatrix<2> KovMatr_initial(2,2);
    TMatrix<2> KovMatr(2,2);
    TMatrix<2> InverseKovMatr(2,2);
    TCMatrixFunc<2> MatrFunc;

    //Construction jacobian matrix for functions H(h,v) = h and He(h,v)
    J_H_He.M[0][0] = 1;
    J_H_He.M[0][1] = 0;
    J_H_He.M[1][0] = pow(con_Grav_Const * con_Earth_Mass /
                       ((m_pMarkInpData->pTrParameters->H + con_Eath_middle_radius) *
                        (con_Grav_Const * con_Earth_Mass / (m_pMarkInpData->pTrParameters->H + con_Eath_middle_radius) -
                         pow(m_pMarkInpData->pTrParameters->V,2) / 2)),2);

    J_H_He.M[1][1] = con_Grav_Const * con_Earth_Mass * m_pMarkInpData->pTrParameters->V /
            pow(con_Grav_Const * con_Earth_Mass / (m_pMarkInpData->pTrParameters->H + con_Eath_middle_radius) -
                pow(m_pMarkInpData->pTrParameters->V, 2.) / 2, 2.);
    //covariance matrix for H and V
    KovMatr_initial.M[0][0] = sqr(m_pMarkInpData->pTrParameters->SigH);
    KovMatr_initial.M[0][1] = 0;
    KovMatr_initial.M[1][0] = 0;
    KovMatr_initial.M[1][1] = sqr(m_pMarkInpData->pTrParameters->SigV);

    flag_isOk = MatrFunc.Transpon(J_H_He, J_H_He_Tr); //J_H_He_Tr is J_H_He transposed
    if (!flag_isOk)
    {
        return;
    }

    //finding covariance matrix for H and He
    flag_isOk = MatrFunc.MatrXMatrXMatr(J_H_He, KovMatr_initial, J_H_He_Tr, KovMatr); //D_Matr = J*K*JT
    if (!flag_isOk)
    {
        return;
    }

    flag_isOk = MatrFunc.InvMatrixSymmPos(KovMatr,InverseKovMatr);
    if (!flag_isOk)
    {
        return;
    }

    const qreal ELLIPSE_SIZE_CONST = 3;
    //calculating such values MaxHDist and MaxHeDist that if point (h_c,he_c) will be the center of the ellipse than lines
    //h = h_c + MaxHDist, h = h_c - MaxHDist, he = he_c + MaxHeDist, he = he_c - MaxHeDist will be the tangent lines of the ellipse
    const qreal MaxHDist = ELLIPSE_SIZE_CONST / sqrt(InverseKovMatr.M[0][0] - pow(InverseKovMatr.M[0][1],2) / InverseKovMatr.M[1][1]);
    const qreal MaxHeDist = ELLIPSE_SIZE_CONST / sqrt(InverseKovMatr.M[1][1] - pow(InverseKovMatr.M[0][1],2) / InverseKovMatr.M[0][0]);


    qreal maxH;

    qreal maxHe;

    qreal LineH;
    qreal LineHe;

    qreal HeP_curr;

    qreal HP_curr;

    std::set<qint32> SetOfCellsNumbers;
    qint32 CurrCellNumber;

    const qreal minH = m_pMarkInpData->pTrParameters->H - MaxHDist;
    maxH = m_pMarkInpData->pTrParameters->H + MaxHDist;
    const qreal minHe = m_pMarkInpData->pTrParameters->He - MaxHeDist;
    maxHe = m_pMarkInpData->pTrParameters->He + MaxHeDist;
    const qreal gridScale = GridScaleCalculation(std::max(minH,minHe));
    if (minH < gridScale)
    {
        LineH = gridScale;
    }
    else
    {
        LineH = ceil(minH / gridScale) * gridScale;
    }
    if(maxH > Cells_H_He_Bounds[3])
    {
        maxH = Cells_H_He_Bounds[3] + 10;
    }

    CurrCellNumber = CalculateCellNumber_H_He(m_pMarkInpData->pTrParameters->H, m_pMarkInpData->pTrParameters->He);
    SetOfCellsNumbers.insert(CurrCellNumber);

    for (qint32 i_JastInCase = 0; (LineH < maxH) && (i_JastInCase < 10000); i_JastInCase++)
    {
        //finding the closest to the ellipse center point on line (h = LineH); //algorithm discription
        //this point will have coordinates (LineH,MiddlePHe) on H-He plane  //algorithm discription
        const qreal H0 = m_pMarkInpData->pTrParameters->H;
        const qreal He0 = m_pMarkInpData->pTrParameters->He;
        const qreal MiddlePHe = He0 - ((LineH-H0) * InverseKovMatr.M[0][1]) / InverseKovMatr.M[1][1];
        //finding the distence between this closest point and crossing point of ellipse boundary with this line  //algorithm discription
        const qreal HalfHeDist = sqrt((pow(ELLIPSE_SIZE_CONST,2) - pow(LineH-H0, 2.) * (InverseKovMatr.M[0][0] -
                          pow(InverseKovMatr.M[0][1],2.) / InverseKovMatr.M[1][1])) / InverseKovMatr.M[1][1]);
        if ((MiddlePHe + HalfHeDist < 0) || (MiddlePHe - HalfHeDist > Cells_H_He_Bounds[3]))
        {
            LineH += gridScale;
            continue;
        }

        HeP_curr = ceil((MiddlePHe - HalfHeDist) / gridScale) * gridScale;
        if(HeP_curr < 0)
        {
            HeP_curr = gridScale;
        }

        for (qint32 j_JastInCase = 0; (HeP_curr < MiddlePHe) && (j_JastInCase < 10000); j_JastInCase++)
        {
            CurrCellNumber = CalculateCellNumber_H_He( LineH - gridScale / 2, HeP_curr - gridScale / 2);
            SetOfCellsNumbers.insert(CurrCellNumber);
            HeP_curr += gridScale;
        }
        HeP_curr -= gridScale;
        for (qint32 j_JastInCase = 0; (HeP_curr < MiddlePHe + HalfHeDist) && (j_JastInCase < 10000); j_JastInCase++)
        {
            CurrCellNumber = CalculateCellNumber_H_He( LineH - gridScale / 2, HeP_curr + gridScale / 2);
            SetOfCellsNumbers.insert(CurrCellNumber);
            HeP_curr += gridScale;
        }
        LineH += gridScale;
    }

    if (minHe < gridScale)
    {
        LineHe = gridScale;
    }
    else
    {
        LineHe = ceil(minHe / gridScale) * gridScale;
    }
    if(maxHe > Cells_H_He_Bounds[3])
    {
        maxHe = Cells_H_He_Bounds[3] + 10;
    }

    for (qint32 i_JastInCase = 0; (LineHe < maxHe) && (i_JastInCase < 10000); i_JastInCase++)
    {
        const qreal H0 = m_pMarkInpData->pTrParameters->H;
        const qreal He0 = m_pMarkInpData->pTrParameters->He;
        const qreal MiddlePH = H0 - ((LineHe-He0) * InverseKovMatr.M[0][1]) / InverseKovMatr.M[0][0];
        const qreal HalfHDist = sqrt((pow(ELLIPSE_SIZE_CONST,2) - pow((LineHe-He0), 2) * (InverseKovMatr.M[1][1] -
                          pow(InverseKovMatr.M[0][1],2) / InverseKovMatr.M[0][0])) / InverseKovMatr.M[0][0]);
        HP_curr = ceil((MiddlePH - HalfHDist) / gridScale) * gridScale;
        if (HP_curr > 0 && HP_curr < Cells_H_He_Bounds[3])
        {
            CurrCellNumber = CalculateCellNumber_H_He( HP_curr - gridScale / 2, LineHe - gridScale / 2);
            SetOfCellsNumbers.insert(CurrCellNumber);
        }
        HP_curr = floor((MiddlePH + HalfHDist) / gridScale) * gridScale;
        if (HP_curr > 0 && HP_curr < Cells_H_He_Bounds[3])
        {
            CurrCellNumber = CalculateCellNumber_H_He( HP_curr + gridScale / 2, LineHe - gridScale / 2);
            SetOfCellsNumbers.insert(CurrCellNumber);
        }

        LineHe += gridScale;
    }

    qint32 Code_ListBMMarks;
    BMMark_AndDiapason BMMark_temp;
    std::set<qint32>::iterator it = SetOfCellsNumbers.begin();
    if (it == SetOfCellsNumbers.end())
    {
        return;
    }
    if ((*it)==0)
    {
        it++;
        if (it == SetOfCellsNumbers.end())
        {
            return;
        }
    }
    Code_ListBMMarks = m_pMarkInpData->Cells_BMMark->ArrCells[(*it)-1]; //numeric correspondence between cell and list of BT marks
    GetBMMarkByCode(Code_ListBMMarks, BMMark_temp);
    BMMark.maxBMMark = BMMark_temp.maxBMMark;
    BMMark.minBMMark = BMMark_temp.minBMMark;
    while(it != SetOfCellsNumbers.end())
    {
        Code_ListBMMarks = m_pMarkInpData->Cells_BMMark->ArrCells[(*it)-1]; //numeric correspondence between cell and list of BT marks
        GetBMMarkByCode(Code_ListBMMarks, BMMark_temp);
        if (BMMark.maxBMMark < BMMark_temp.maxBMMark)
        {
            BMMark.maxBMMark = BMMark_temp.maxBMMark;
        }
        if (BMMark.minBMMark > BMMark_temp.minBMMark && BMMark_temp.maxBMMark > 0)
        {
            BMMark.minBMMark = BMMark_temp.minBMMark;
        }

        it++;
    }

    if ((BMMark.minBMMark > AIR_BALL::UNDEFINED_BM_MARK)
            && (BMMark.minBMMark == BMMark.maxBMMark))
    {
        BMMark.IsSingleValued = true;
        BMMark.MarkValue = BMMark.maxBMMark;
    }
    else
    {
        BMMark.IsSingleValued = false;
        BMMark.MarkValue = BMMark.maxBMMark;
    }

#ifdef KEEP_LOG
    AIR_BALL::out_BMMark_info << "Cell number: " << CurrCellNumber << "\t"
                              << "Cell code: " << Code_ListBMMarks << "\t";
#endif //KEEP_LOG

}


void CBallSubclassDetermination::MarkDet_He(BMMark_AndDiapason &BMMark)
{
    if (BMMark.IsEmpty())
    {
        qint32 HeMin = m_pMarkInpData->pTrParameters->He
                - AIR_BALL::K_Sigma * m_pMarkInpData->pTrParameters->SigHe;
        qint32 HeMax = m_pMarkInpData->pTrParameters->He
                + AIR_BALL::K_Sigma * m_pMarkInpData->pTrParameters->SigHe;

        for (qint16 CurMark = AIR_BALL::UNKNOWN_OF_SHORT_RANGE+1;
             CurMark < AIR_BALL::UNKNOWN_OF_LONG_RANGE; CurMark++)
        {
            if (HeMax > m_ar_BM_Diapasons[CurMark].HeMin - 3.*con_eps
                    && HeMin < m_ar_BM_Diapasons[CurMark].HeMax + 3.*con_eps)
            {
                if (BMMark.minBMMark <= AIR_BALL::UNKNOWN_OF_SHORT_RANGE
                        || BMMark.minBMMark > CurMark)
                {
                    BMMark.minBMMark = CurMark;
                }

                if (BMMark.maxBMMark <= AIR_BALL::UNKNOWN_OF_SHORT_RANGE
                        || BMMark.maxBMMark < CurMark)
                {
                    BMMark.maxBMMark = CurMark;
                }
            }
        }

        if (BMMark.minBMMark > AIR_BALL::UNKNOWN_OF_SHORT_RANGE+1)
        {
            BMMark.minBMMark --;
        }
    }
}


qint32 CBallSubclassDetermination::GridScaleCalculation(const qint32 H_He_value)
{
    if (H_He_value < Cells_H_He_Bounds[0])
    {
        return Cells_sizes[0];
    }
    else if (H_He_value < Cells_H_He_Bounds[1])
    {
        return Cells_sizes[1];
    }
    else if (H_He_value < Cells_H_He_Bounds[2])
    {
        return Cells_sizes[2];
    }
    else
    {
       return Cells_sizes[3];
    }
}


void CBallSubclassDetermination::MarkCorr_Theta_He(BMMark_AndDiapason &BMMark)
{
    if (!BMMark.IsEmpty())
    {
        ;
    }
}


void CBallSubclassDetermination::MarkCorr_Theta_Veap(BMMark_AndDiapason &BMMark)
{
    if (!BMMark.IsEmpty())
    {
        ;
    }
}


void CBallSubclassDetermination::MarkCorr_Distance(BMMark_AndDiapason &BMMark)
{
    if (m_pMarkInpData->D > con_par_eps)
    {
        qint32 NewMarkMax = BMMark.maxBMMark;

        for (qint32 j = BMMark.maxBMMark; j < AIR_BALL::UNKNOWN_OF_LONG_RANGE; j++)
        {
            const qreal NomRangeMax_curr = GetNominalRange(j);
            if (NomRangeMax_curr > con_par_eps)
            {
                if (m_pMarkInpData->D > NomRangeMax_curr * K_NomDistance + m_pMarkInpData->SigmaSP)
                {
                    if (NewMarkMax < j)
                    {
                        NewMarkMax = j;
                    }

                    NewMarkMax ++;
                }
                else
                {
                    break;
                }
            }
        }

        qint32 NewMarkMin = BMMark.minBMMark;

        for (qint32 j = BMMark.minBMMark; j <= NewMarkMax; j++)
        {
            const qreal NomRangeMin_curr = GetNominalRange(j);
            if (NomRangeMin_curr > con_par_eps)
            {
                if (m_pMarkInpData->D > NomRangeMin_curr * K_NomDistance + m_pMarkInpData->SigmaSP)
                {
                    if (NewMarkMin < j)
                    {
                        NewMarkMin = j;
                    }

                    NewMarkMin ++;
                }
                else
                {
                    break;
                }
            }
        }

        if (NewMarkMin > BMMark.minBMMark)
        {
            BMMark.minBMMark = NewMarkMin;
        }

        if (NewMarkMax > BMMark.maxBMMark)
        {
            BMMark.maxBMMark = NewMarkMax;
            BMMark.MarkValue = NewMarkMax;

            if (BMMark.IsSingleValued && BMMark.minBMMark < BMMark.maxBMMark)
            {
                BMMark.IsSingleValued = false;
            }            
        }

        if (!BMMark.IsSingleValued
                && BMMark.minBMMark > AIR_BALL::UNDEFINED_BM_MARK
                && BMMark.minBMMark == BMMark.maxBMMark)
        {
            BMMark.IsSingleValued = true;
        }        
    }
}


void CBallSubclassDetermination::GetBMMarkByCode(const qint32 Code_ListBMMarks, BMMark_AndDiapason &BMMark)
{
    BMMark.Reset();
    qint32 remainder;
    qint32 quotient = Code_ListBMMarks;
    const qint16 COUNTING_START = AIR_BALL::UNKNOWN_OF_SHORT_RANGE;
    for (qint32 i=0; i< AIR_BALL::NUMB_BM_MARKS; i++)
    {
        remainder = quotient % 2;
        quotient = (quotient - remainder) / 2;

        if (remainder == 1)
        {
            if (BMMark.minBMMark == AIR_BALL::UNDEFINED_BM_MARK)
            {
                BMMark.minBMMark = COUNTING_START + (i+1);
            }
            BMMark.maxBMMark = COUNTING_START + (i+1);
        }
    }

    if ( (BMMark.minBMMark > AIR_BALL::UNDEFINED_BM_MARK)
            && (BMMark.minBMMark == BMMark.maxBMMark))
    {
        BMMark.IsSingleValued = true;
        BMMark.MarkValue = BMMark.minBMMark;
    }
}



qreal CBallSubclassDetermination::GetNominalRange(const qint32 MarkValue)
{
    qreal D_Nominal = 0;

    switch (MarkValue)
    {
    case AIR_BALL::BM60:
        D_Nominal = 60000;
        break;
    case AIR_BALL::BM100:
        D_Nominal = 100000;
        break;
    case AIR_BALL::BM200:
        D_Nominal = 200000;
        break;
    case AIR_BALL::BM300:
        D_Nominal = 300000;
        break;
    case AIR_BALL::BM600:
        D_Nominal = 600000;
        break;
    case AIR_BALL::BM750:
        D_Nominal = 750000;
        break;
    case AIR_BALL::BM1000:
        D_Nominal = 1000000;
        break;
    case AIR_BALL::BM1500:
        D_Nominal = 1500000;
        break;
    case AIR_BALL::BM1800:
        D_Nominal = 1800000;
        break;
    case AIR_BALL::BM2000:
        D_Nominal = 2000000;
        break;
    case AIR_BALL::BM2500:
        D_Nominal = 2500000;
        break;
    case AIR_BALL::BM2800:
        D_Nominal = 2800000;
        break;
    case AIR_BALL::BM3000:
        D_Nominal = 3000000;
        break;
    case AIR_BALL::BM3500:
        D_Nominal = 3500000;
        break;
    case AIR_BALL::BM5000:
        D_Nominal = 5000000;
        break;
    case AIR_BALL::BM6000:
        D_Nominal = 6000000;
        break;
    case AIR_BALL::BM7000:
        D_Nominal = 7000000;
        break;
    case AIR_BALL::BM8000:
        D_Nominal = 8000000;
        break;
    case AIR_BALL::BM9000:
        D_Nominal = 9000000;
        break;
    case AIR_BALL::BM10000:
        D_Nominal = 10000000;
        break;
    case AIR_BALL::BM11000:
        D_Nominal = 11000000;
        break;
    case AIR_BALL::BM12000:
        D_Nominal = 12000000;
        break;
    case AIR_BALL::UNKNOWN_OF_LONG_RANGE:
        D_Nominal = 14000000;
        break;
    default:
        D_Nominal = 0;
        break;
    }

    return D_Nominal;
}


qint32 CBallSubclassDetermination::CalculateCellNumber(const qint32 i, const qint32 j, const qint32 diapason)
{
    const qint32 max_curr = std::max(i, j);
    const qint32 Res_numb = (max_curr-1)*(max_curr-1) + max_curr + i - j + Cells_N0_diapason[diapason];
    return Res_numb;
}


qint32 CBallSubclassDetermination::CalculateCellNumber_H_He(const qreal H, const qreal He)
{
    const qreal Max_cmp = std::max(H, He);
    qint32 Res_numb = 0;
    qint32 diapason = 0; //numeration from 0

    if (Max_cmp <= Cells_H_He_Bounds[0])
    {
        diapason = 0;
    }
    else if (Max_cmp <= Cells_H_He_Bounds[1])
    {
        diapason = 1;
    }
    else if (Max_cmp < Cells_H_He_Bounds[2])
    {
        diapason = 2;
    }
    else if (Max_cmp < Cells_H_He_Bounds[3])
    {
        diapason = 3;
    }
    else
    {
        diapason = 4;
    }

    if (diapason < 4)
    {


        const qint32 i = static_cast<qint32> (ceil(H / Cells_sizes[diapason]));
        const qint32 j = static_cast<qint32> (ceil(He / Cells_sizes[diapason]));  //numeration from 1

        Res_numb = CalculateCellNumber(i, j, diapason);
    }

    return Res_numb;
}


qreal CBallSubclassDetermination::CalculateLimitingHeight()
{
    qreal H_lim = 0;

    const qint32 BallIndex = m_pMarkInpData->IndBall;
    const qint32 min_mark = m_arMarkData[BallIndex].BMMark.minBMMark;
    const qint32 max_mark = m_arMarkData[BallIndex].BMMark.maxBMMark;
    qint32 j;
    qint32 j_min=0;
    qint32 j_max=0;

    for (j=1; j<N_LIMITS_H_MARK; j++)
    {
        if (H_LIM_MARK[j-1][0] < min_mark && min_mark <= H_LIM_MARK[j][0])
        {
            j_min = j;
        }

        if (H_LIM_MARK[j-1][0] < max_mark && max_mark <= H_LIM_MARK[j][0])
        {
            j_max = j;
        }
    }

    for (j = j_min; j <= j_max; j++)
    {
        if (H_LIM_MARK[j][1] > H_lim)
        {
            H_lim = H_LIM_MARK[j][1];
        }
    }

    return H_lim;
}


void CBallSubclassDetermination::ClearInnerData_Subcl()
{
    for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
    {
        ClearInnerData_Subcl_1Track(i);
    }
}


void CBallSubclassDetermination::ClearInnerData_TType()
{
    for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
    {
        ClearInnerData_TType_1Track(i);
    }
}


void CBallSubclassDetermination::ClearInnerData_Subcl_1Track(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_arMarkData[ind_Ball].Reset();
    }
}


void CBallSubclassDetermination::ClearInnerData_TType_1Track(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_arTTypeData[ind_Ball].Reset();
    }
}
