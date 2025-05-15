#include <math.h>
#include <fstream>
#include <iomanip>

#include "gl/GLMath.h"
#include "gl/Constants.h"

#include "CBallSubclassDetermination_M.h"
#include "air_ballist_literal.h"
#include "air_ballist_constants.h"

// using namespace std;

#ifdef KEEP_LOG
namespace AIR_BALL{
//    static std::ofstream out_TType_info(qPrintable(QString("%1/TrajTypeDet_Info_M.txt").arg(log_path)));
//    static std::ofstream out_BMMark_info(qPrintable(QString("%1/BMMark_Info_M.txt").arg(log_path)));
    static FILE* out_TType_info = nullptr;
    static FILE* out_BMMark_info = nullptr;
}
#endif //KEEP_LOG


CBallSubclassDetermination_M::CBallSubclassDetermination_M()
{
//#ifdef KEEP_LOG
//    AIR_BALL::out_info_subclass << "Constructor: CBallSubclassDetermination_M  " << this << std::endl;
//#endif //KEEP_LOG

    m_arTTypeData.resize(AIR_BALL::BALL_FORM_AMOUNT);
    m_arMarkData.resize(AIR_BALL::BALL_FORM_AMOUNT);

#ifdef KEEP_LOG
        AIR_BALL::out_TType_info = GLFileLog::OpenLog("TrajTypeDet_Info_M", AIR_BALL::log_path);
        AIR_BALL::out_BMMark_info = GLFileLog::OpenLog("BMMark_Info_M", AIR_BALL::log_path);
#endif //KEEP_LOG

//    Reset();
}


CBallSubclassDetermination_M::~CBallSubclassDetermination_M()
{
#ifdef KEEP_LOG
    if( AIR_BALL::out_TType_info ) {
        fclose(AIR_BALL::out_TType_info);
    }
    if( AIR_BALL::out_BMMark_info ) {
        fclose(AIR_BALL::out_BMMark_info);
    }
#endif //KEEP_LOG

//#ifdef KEEP_LOG
//    AIR_BALL::out_info_subclass << "Destructor: CBallSubclassDetermination_M  " << this << std::endl;
//#endif //KEEP_LOG
}


void CBallSubclassDetermination_M::Reset()
{
//    m_pTTypeInpData = nullptr;
//    m_pMarkInpData  = nullptr;

    ClearInnerData_Subcl();
    ClearInnerData_TType();

    m_ar_BM_Diapasons.Reset();

//#ifdef KEEP_LOG
//    AIR_BALL::out_info_subclass << "Reset CBallSubclassDetermination_M:  " << this << std::endl;
//#endif
}


void CBallSubclassDetermination_M::TrajTypeDet_Realization(const TrajType_InputData* pInData,
                                                           TrajType_OutputData& rOutData)
{    
    const qint32 BallIndex = pInData->IndBall;

    rOutData.P_rewrite_TT = false;

    bool p_check = false; //sign of check
    if (fabs(pInData->tLoc - m_arTTypeData[BallIndex].t_prev_TType_check) > cTTypeRecount) //more than cTTypeRecount s passed after last TType determination
    {
        p_check = true;
    }

    if ( (m_arTTypeData[BallIndex].PrTTypeD && !p_check)
           || fabs(pInData->tLoc - m_arTTypeData[BallIndex].t_prev_TType_calc) < dT_TType )
    {
        return;
    }

    qint16 currTType = AIR_BALL::UNDEFINED_TRAJ_TYPE;

    const qreal A = fabs(pInData->VH*pInData->VH/(2*pInData->AH)) + pInData->H;
    const qreal Ac_pred = m_arTTypeData[BallIndex].Ac;
    const qreal Ac = (1-KsiAh)*A + KsiAh*Ac_pred;
    const qreal delta_Ac = fabs((Ac-Ac_pred)/(std::max(std::min(Ac, Ac_pred), epsAh)));
    m_arTTypeData[BallIndex].Ac = Ac;

#ifdef KEEP_LOG
    QString l_log = QString::asprintf("IndBall = %d\t| H = %.5f\t VH = %.5f\t AH = %.5f "
                                      "|\tA = %.5f\tAc = %.5f\tAc_pred = %.5f\tdelta_Ac = %.5f\tD = %.5f",
                                      BallIndex, pInData->H, pInData->VH, pInData->AH,
                                      A, Ac, Ac_pred, delta_Ac, pInData->D);
#endif //KEEP_LOG

    if (delta_Ac < kAh && Ac_pred > con_par_eps
            && pInData->D > con_par_eps)
    {
        //trajectory type determination using information about apogee
        const qreal lambda = Ac/pow(pInData->D, sAh);
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
        l_log += QString("\tlambda = %1\tTType = %2").arg(lambda, 0, 'f', 5).arg(currTType);
#endif //KEEP_LOG
    }
    else
    {
        //trajectory type determination using information about throw angle
        if (pInData->bound_flat_tr > con_par_eps
                && pInData->bound_lofted_tr > con_par_eps
                && pInData->throw_angle > con_par_eps)
        {
            if (pInData->throw_angle < pInData->bound_flat_tr)
            {
                currTType = AIR_BALL::FLAT;
            }
            else if (pInData->throw_angle < pInData->bound_lofted_tr)
            {
                currTType = AIR_BALL::OPTIMAL;
            }
            else
            {
                currTType = AIR_BALL::FLAT;
            }
        }
#ifdef KEEP_LOG
        l_log += QString("\tthrow_angle = %1\tTType = %2").arg(pInData->throw_angle, 0, 'f', 5).arg(currTType);
#endif //KEEP_LOG
    }

#ifdef KEEP_LOG
    GLFileLog::tLog(AIR_BALL::out_TType_info, "%s", qPrintable(l_log));
#endif //KEEP_LOG

    if (currTType != AIR_BALL::UNDEFINED_TRAJ_TYPE)
    {
        if (p_check && m_arTTypeData[BallIndex].TType == currTType)
        {
            m_arTTypeData[BallIndex].t_prev_TType_check = pInData->tLoc;
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
                rOutData.P_rewrite_TT = true;

                m_arTTypeData[BallIndex].PrTTypeD = true;
                m_arTTypeData[BallIndex].TType = currTType;
                rOutData.TrajType = currTType;

                m_arTTypeData[BallIndex].t_prev_TType_calc = pInData->tLoc;
                m_arTTypeData[BallIndex].prevTType = currTType;
            }
        }
    }

}


void CBallSubclassDetermination_M::BMMarkDet_Realization(const BTMark_InputData* pInData,
                                                         BTMark_OutputData& rOutData)
{
    if (pInData->IndBall < 0 || pInData->IndBall >= AIR_BALL::BALL_FORM_AMOUNT
            || pInData->pTrParameters == nullptr)
    {
        return;
    }

    bool p_calc_mark=false;   //signs of calculation and check of the mark
    bool p_check_mark=false;  //signs of calculation and check of the mark

    const qint32 BallIndex = pInData->IndBall;
    rOutData.Reset();

    if (fabs(pInData->tLoc - m_arMarkData[BallIndex].PrevTLoc) > con_eps)
    {
        m_arMarkData[BallIndex].PrevTLoc = pInData->tLoc;
        const qreal H_lim = CalculateLimitingHeight(pInData); //conditional height of atmosphere to restrict definition of mark

        if (/*!m_arMarkData[CurIndex].BMMark.IsSingleValued*/
                !m_arMarkData[BallIndex].SignFinalDecision)
        {
            if ( pInData->Branch == AIR_BALL::ASCENDING_BRANCH
                 || pInData->pTrParameters->H > H_lim
                 || !m_arMarkData[BallIndex].SignConfirmedMin
                 || !m_arMarkData[BallIndex].SignConfirmedMax)
            {
                p_calc_mark = true;
            }
        }
        else
        {
            if (m_arMarkData[BallIndex].SignConfrimation
                    || ( pInData->tLoc - m_arMarkData[BallIndex].tCheck > PERIOD_MARK_CHECK
                        && pInData->tFall - pInData->tLoc > DT_REFUSE_MARK_RECOUNT
                        && pInData->Branch != AIR_BALL::DESCENDING_BRANCH))
            {
                p_check_mark = true;
            }

            qreal dDmax = GetNominalRange(m_arMarkData[BallIndex].BMMark.maxBMMark);
            qreal dDmin = GetNominalRange(m_arMarkData[BallIndex].BMMark.minBMMark);
            if ((pInData->D > dDmax * K_NomDistance + pInData->SigmaSP
                    || pInData->D < dDmin * K_NomMinDistance - pInData->SigmaSP)
                && pInData->tFall - pInData->tLoc > DT_REFUSE_MARK_RECOUNT)
            {
                p_check_mark = true;
            }
        }

#ifdef KEEP_LOG
        QString l_log = QString("\t%1\t%2\t%3\t%4\t").arg(pInData->tLoc, 0, 'f', 5)
                .arg(BallIndex).arg((qint16)p_calc_mark).arg((qint16)p_check_mark);
#endif //KEEP_LOG

        if (p_calc_mark || p_check_mark)
        {
#ifdef KEEP_LOG
            l_log += CalculateAndCheckMark(p_calc_mark, pInData, rOutData);
#else
            CalculateAndCheckMark(p_calc_mark, pInData, rOutData);
#endif
        }

        rOutData.BMMark = m_arMarkData[BallIndex].BMMark;

        if (!m_arMarkData[BallIndex].BMMark.IsSingleValued)
        {
            //output of confirmed values of minimal and maximal mark
            if ( (m_arMarkData[BallIndex].MinMark_Confirmed > AIR_BALL::UNDEFINED_BM_MARK)
                 && (m_arMarkData[BallIndex].MinMark_Confirmed != rOutData.BMMark.minBMMark))
            {
                rOutData.BMMark.minBMMark = m_arMarkData[BallIndex].MinMark_Confirmed;
            }

            if ((m_arMarkData[BallIndex].MaxMark_Confirmed > AIR_BALL::UNDEFINED_BM_MARK)
                    && (m_arMarkData[BallIndex].MaxMark_Confirmed != rOutData.BMMark.maxBMMark))
            {
                rOutData.BMMark.maxBMMark = m_arMarkData[BallIndex].MaxMark_Confirmed;
            }

            rOutData.BMMark.MarkValue = rOutData.BMMark.maxBMMark;
        }

#ifdef KEEP_LOG
        l_log += QString("Mark is changed? %1\t").arg((qint16)rOutData.P_MarkIsChanged);
        l_log += QString("Final decision? %1\t").arg(m_arMarkData[BallIndex].SignFinalDecision);
        l_log += QString("Need confirmation? %1\t").arg(m_arMarkData[BallIndex].SignConfrimation);
        l_log += QString("Begin of confirmation: %1\t").arg(m_arMarkData[BallIndex].tBeginConfirmation, 0, 'f', 3);
        l_log += QString("Number of confirmations: %1\t").arg(m_arMarkData[BallIndex].n_Confirmation);
        l_log += QString("Time of check: %1\t").arg(m_arMarkData[BallIndex].tCheck, 0, 'f', 3);
        l_log += QString("Min is confirmed? %1\t").arg(m_arMarkData[BallIndex].SignConfirmedMin);
        l_log += QString("Max is confirmed? %1\t").arg(m_arMarkData[BallIndex].SignConfirmedMax);
        l_log += QString("Begin min confirm.: %1\t").arg(m_arMarkData[BallIndex].tBeginConfirm_min, 0, 'f', 3);
        l_log += QString("Begin max confirm.: %1\t").arg(m_arMarkData[BallIndex].tBeginConfirm_max, 0, 'f', 3);
        l_log += QString("Numb. min confirm.: %1\t").arg(m_arMarkData[BallIndex].n_Confirm_min);
        l_log += QString("Numb. max confirm.: %1\t").arg(m_arMarkData[BallIndex].n_Confirm_max);
        l_log += QString("Confirmed min: %1\t").arg(m_arMarkData[BallIndex].MinMark_Confirmed);
        l_log += QString("Confirmed max:%1\t").arg(m_arMarkData[BallIndex].MaxMark_Confirmed);
        l_log += QString("Lim. height: %1\t").arg(H_lim, 0, 'f', 3);
        l_log += QString("%1\t").arg(m_arMarkData[BallIndex].BMMark.IsSingleValued);
        l_log += QString("%1\t").arg(m_arMarkData[BallIndex].BMMark.MarkValue);
        l_log += QString("%1\t").arg(m_arMarkData[BallIndex].BMMark.minBMMark);
        l_log += QString("%1\t").arg(m_arMarkData[BallIndex].BMMark.maxBMMark);
        GLFileLog::tLog(AIR_BALL::out_BMMark_info, "%s", qPrintable(l_log));
#endif //KEEP_LOG
    }
}


void CBallSubclassDetermination_M::FillHeBoundaryValues(Cells_BMMark_Data *pArrCells)
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


QString CBallSubclassDetermination_M::CalculateAndCheckMark(const bool p_calc_mark,
                                                            const BTMark_InputData* pInData,
                                                            BTMark_OutputData &rOutData)
{
    const qint32 BallIndex = pInData->IndBall;

    BMMark_AndDiapason BMMark;
    auto l_log = OnceMarkDet(pInData, BMMark);

    if (BMMark.IsSingleValued)
    {
        if (BMMark.MarkValue == m_arMarkData[BallIndex].BMMark.MarkValue)
        {
            if(fabs(m_arMarkData[BallIndex].tBeginConfirmation - AIR_BALL::Timer_INI) < con_par_eps)
            {
                m_arMarkData[BallIndex].tBeginConfirmation = pInData->tLoc;
            }

            m_arMarkData[BallIndex].n_Confirmation ++;

            if (m_arMarkData[BallIndex].n_Confirmation > NUMB_MARK_CONFIRM
                    && pInData->tLoc - m_arMarkData[BallIndex].tBeginConfirmation > PERIOD_MARK_CONFIRM)
            {
                //Mark is confirmed
                m_arMarkData[BallIndex].SignConfrimation = false;
                m_arMarkData[BallIndex].tCheck = pInData->tLoc;
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
                    rOutData.P_MarkIsChanged = true;
                }

                m_arMarkData[BallIndex].BMMark = BMMark;
            }
        }
        else
        {
            if (p_calc_mark)
            {
                m_arMarkData[BallIndex].tCheck = pInData->tLoc;
            }

            m_arMarkData[BallIndex].BMMark = BMMark;

            m_arMarkData[BallIndex].SignConfrimation = true;
            m_arMarkData[BallIndex].tBeginConfirmation = AIR_BALL::Timer_INI;
            m_arMarkData[BallIndex].n_Confirmation = 1;

            rOutData.P_MarkIsChanged = true;
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
                        m_arMarkData[BallIndex].tBeginConfirm_min = pInData->tLoc;
                    }

                    m_arMarkData[BallIndex].n_Confirm_min ++;

                    if (m_arMarkData[BallIndex].n_Confirm_min > NUMB_MARK_CONFIRM
                            && pInData->tLoc - m_arMarkData[BallIndex].tBeginConfirm_min > PERIOD_MARK_CONFIRM)
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
                        m_arMarkData[BallIndex].tBeginConfirm_max = pInData->tLoc;
                    }

                    m_arMarkData[BallIndex].n_Confirm_max ++;

                    if (m_arMarkData[BallIndex].n_Confirm_max > NUMB_MARK_CONFIRM
                            && pInData->tLoc - m_arMarkData[BallIndex].tBeginConfirm_max > PERIOD_MARK_CONFIRM)
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
                rOutData.P_MarkIsChanged = true;
            }

            m_arMarkData[BallIndex].BMMark = BMMark;
        }
    }
    return l_log;
}


void CBallSubclassDetermination_M::Drop_BallTrack(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        ClearInnerData_Subcl_1Track(ind_Ball);
        ClearInnerData_TType_1Track(ind_Ball);
    }
}


//void CBallSubclassDetermination_M::Drop_ST(qint32 ind_ST)
//{
//    ;
//}


QString CBallSubclassDetermination_M::OnceMarkDet(const BTMark_InputData* pInData,
                                                  BMMark_AndDiapason &BMMark)
{
    BMMark.Reset();

    QString l_log;
    MarkDet_H_He(pInData, BMMark, l_log);

    if( BMMark.IsEmpty() ) {
        MarkDet_He(pInData, BMMark);
    }

    if (!BMMark.IsSingleValued)
    {
        if (fabs(pInData->Theta) > con_par_eps)
        {
            MarkCorr_Theta_He(BMMark);

            if (!BMMark.IsSingleValued)
            {
                if (fabs(pInData->Veap) > con_par_eps)
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

    MarkCorr_Distance(pInData, BMMark);
    return l_log;
}


void CBallSubclassDetermination_M::MarkDet_H_He(const BTMark_InputData* pInData,
                                                BMMark_AndDiapason &BMMark, QString& log)
{
    if (pInData == nullptr) {
        return;
    }
    if (pInData->Cells_BMMark == nullptr) {
        return;
    }
    if (pInData->pTrParameters == nullptr) {
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
                       ((pInData->pTrParameters->H + con_Eath_middle_radius) *
                        (con_Grav_Const * con_Earth_Mass / (pInData->pTrParameters->H + con_Eath_middle_radius) -
                         pow(pInData->pTrParameters->V,2) / 2)),2);

    J_H_He.M[1][1] = con_Grav_Const * con_Earth_Mass * pInData->pTrParameters->V /
            pow(con_Grav_Const * con_Earth_Mass / (pInData->pTrParameters->H + con_Eath_middle_radius) -
                pow(pInData->pTrParameters->V, 2.) / 2, 2.);
    //covariance matrix for H and V
    KovMatr_initial.M[0][0] = sqr(pInData->pTrParameters->SigH);
    KovMatr_initial.M[0][1] = 0;
    KovMatr_initial.M[1][0] = 0;
    KovMatr_initial.M[1][1] = sqr(pInData->pTrParameters->SigV);

    flag_isOk = MatrFunc.Transpon(J_H_He, J_H_He_Tr); //J_H_He_Tr is J_H_He transposed
    if (!flag_isOk) {
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

    qreal LineH;
    qreal LineHe;

    qreal HeP_curr;

    qreal HP_curr;

    std::set<qint32> SetOfCellsNumbers;
    qint32 CurrCellNumber;

    const qreal minH = pInData->pTrParameters->H - MaxHDist;
    qreal maxH = pInData->pTrParameters->H + MaxHDist;
    const qreal minHe = pInData->pTrParameters->He - MaxHeDist;
    qreal maxHe = pInData->pTrParameters->He + MaxHeDist;
    const qreal gridScale = GridScaleCalculation(std::max(minH,minHe));

    if (minH < gridScale) {
        LineH = gridScale;
    }
    else {
        LineH = ceil(minH / gridScale) * gridScale;
    }

    if( maxH > Cells_H_He_Bounds[3] ) {
        maxH = Cells_H_He_Bounds[3] + 10;
    }

    CurrCellNumber = CalculateCellNumber_H_He(pInData->pTrParameters->H, pInData->pTrParameters->He);
    SetOfCellsNumbers.insert(CurrCellNumber);

    for (qint32 i_JastInCase = 0; (LineH < maxH) && (i_JastInCase < 10000); i_JastInCase++)
    {
        //finding the closest to the ellipse center point on line (h = LineH); //algorithm discription
        //this point will have coordinates (LineH,MiddlePHe) on H-He plane  //algorithm discription
        const qreal H0 = pInData->pTrParameters->H;
        const qreal He0 = pInData->pTrParameters->He;
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

    if (minHe < gridScale) {
        LineHe = gridScale;
    }
    else {
        LineHe = ceil(minHe / gridScale) * gridScale;
    }

    if(maxHe > Cells_H_He_Bounds[3]) {
        maxHe = Cells_H_He_Bounds[3] + 10;
    }

    for (qint32 i_JastInCase = 0; (LineHe < maxHe) && (i_JastInCase < 10000); i_JastInCase++)
    {
        const qreal H0 = pInData->pTrParameters->H;
        const qreal He0 = pInData->pTrParameters->He;
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

    std::set<qint32>::iterator it = SetOfCellsNumbers.begin();
    if (it == SetOfCellsNumbers.end()) {
        return;
    }
    if ((*it)==0)
    {
        ++it;
        if (it == SetOfCellsNumbers.end()) {
            return;
        }
    }

    BMMark_AndDiapason BMMark_temp;
    qint32 Code_ListBMMarks = pInData->Cells_BMMark->ArrCells[(*it)-1]; //numeric correspondence between cell and list of BT marks
    GetBMMarkByCode(Code_ListBMMarks, BMMark_temp);

    BMMark.maxBMMark = BMMark_temp.maxBMMark;
    BMMark.minBMMark = BMMark_temp.minBMMark;
    while(it != SetOfCellsNumbers.end())
    {
        Code_ListBMMarks = pInData->Cells_BMMark->ArrCells[(*it)-1]; //numeric correspondence between cell and list of BT marks
        GetBMMarkByCode(Code_ListBMMarks, BMMark_temp);
        if (BMMark.maxBMMark < BMMark_temp.maxBMMark)
        {
            BMMark.maxBMMark = BMMark_temp.maxBMMark;
        }
        if (BMMark.minBMMark > BMMark_temp.minBMMark && BMMark_temp.maxBMMark > 0)
        {
            BMMark.minBMMark = BMMark_temp.minBMMark;
        }
        ++it;
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
    log += QString("Cell number: %1\tCell code: %2\t").arg(CurrCellNumber).arg(Code_ListBMMarks);
#endif //KEEP_LOG
}


void CBallSubclassDetermination_M::MarkDet_He(const BTMark_InputData* pInData,
                                              BMMark_AndDiapason &BMMark)
{
    if (BMMark.IsEmpty())
    {
        qint32 HeMin = pInData->pTrParameters->He
                - AIR_BALL::K_Sigma * pInData->pTrParameters->SigHe;
        qint32 HeMax = pInData->pTrParameters->He
                + AIR_BALL::K_Sigma * pInData->pTrParameters->SigHe;

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


qint32 CBallSubclassDetermination_M::GridScaleCalculation(const qint32 H_He_value)
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


void CBallSubclassDetermination_M::MarkCorr_Theta_He(BMMark_AndDiapason &BMMark)
{
    if (!BMMark.IsEmpty())
    {
        ;
    }
}


void CBallSubclassDetermination_M::MarkCorr_Theta_Veap(BMMark_AndDiapason &BMMark)
{
    if (!BMMark.IsEmpty())
    {
        ;
    }
}


void CBallSubclassDetermination_M::MarkCorr_Distance(const BTMark_InputData* pInData,
                                                     BMMark_AndDiapason &BMMark)
{
    if (pInData->D > con_par_eps)
    {
        qint32 NewMarkMax = BMMark.maxBMMark;

        for (qint32 j = BMMark.maxBMMark; j < AIR_BALL::UNKNOWN_OF_LONG_RANGE; j++)
        {
            const qreal NomRangeMax_curr = GetNominalRange(j);
            if (NomRangeMax_curr > con_par_eps)
            {
                if (pInData->D > NomRangeMax_curr * K_NomDistance + pInData->SigmaSP)
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
                if (pInData->D > NomRangeMin_curr * K_NomDistance + pInData->SigmaSP)
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


void CBallSubclassDetermination_M::GetBMMarkByCode(const qint32 Code_ListBMMarks, BMMark_AndDiapason &BMMark)
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



qreal CBallSubclassDetermination_M::GetNominalRange(const qint32 MarkValue)
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


qint32 CBallSubclassDetermination_M::CalculateCellNumber(const qint32 i, const qint32 j, const qint32 diapason)
{
    const qint32 max_curr = std::max(i, j);
    const qint32 Res_numb = (max_curr-1)*(max_curr-1) + max_curr + i - j + Cells_N0_diapason[diapason];
    return Res_numb;
}


qint32 CBallSubclassDetermination_M::CalculateCellNumber_H_He(const qreal H, const qreal He)
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


qreal CBallSubclassDetermination_M::CalculateLimitingHeight(const BTMark_InputData* pInData)
{
    qreal H_lim = 0;

    const qint32 BallIndex = pInData->IndBall;
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


void CBallSubclassDetermination_M::ClearInnerData_Subcl()
{
    for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
    {
        ClearInnerData_Subcl_1Track(i);
    }
}


void CBallSubclassDetermination_M::ClearInnerData_TType()
{
    for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
    {
        ClearInnerData_TType_1Track(i);
    }
}


void CBallSubclassDetermination_M::ClearInnerData_Subcl_1Track(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_arMarkData[ind_Ball].Reset();
    }
}


void CBallSubclassDetermination_M::ClearInnerData_TType_1Track(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_arTTypeData[ind_Ball].Reset();
    }
}
