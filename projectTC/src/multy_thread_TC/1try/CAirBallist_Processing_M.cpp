#include "CAirBallist_Processing_M.h"
#include "air_ballist_literal.h"
#include "CtrlAir_Ballist_M.h"

#include "common/GDEnum.h"

#include "conversion/Geoline.h"
#include "conversion/Geocentric_Geo.h"
#include "conversion/Spherical_Topo.h"

#include "gl/GLFileLog.h"
#include "gl/GLEllipse2D.h"
#include "gl/GLCalcTrajParam.h"
#include "gl/GLConvertMatrices.h"

#include <fstream>
#include <iomanip>
// using namespace std;
using namespace GLFileLog;

#ifdef KEEP_LOG
namespace AIR_BALL
{
    std::ofstream out_ball_traj_param(qPrintable(QString("%1/out_ball_traj_param.txt").arg(log_path)));  //logging trajectory parameters
    std::ofstream out_IDs(qPrintable(QString("%1/out_ball_IDs.txt").arg(log_path)));                     //logging identificators
}
#endif //KEEP_LOG


CAirBallist_Processing_GD::CAirBallist_Processing_GD()
{
    InitConst();
    ClearInnerData();

    QMutexLocker l_lock(&m_mutexGD);
    m_pLogProlTrack = OpenLog("ProlongedTrack", AIR_BALL::log_path);
}


CAirBallist_Processing_GD::~CAirBallist_Processing_GD()
{
    if (m_pLogProlTrack)
    {
        fclose(m_pLogProlTrack);
    }
    m_pLogProlTrack = nullptr;
}


void CAirBallist_Processing_GD::Reset()
{
    ClearInnerData();
    InitConst();

    QMutexLocker l_lock(&m_mutexGD);

    m_pEAP_Tables = nullptr;

    m_PathBranchDet.Reset();
    m_BallSubclDet.Reset();

    m_BallProlong.InitPointerToBallProcArray(&m_arProcData);
    m_PathBranchDet.InitPointerToBallProcArray(&m_arProcData);

    try {
        if( m_pLogProlTrack ) {
            fclose(m_pLogProlTrack);
        }
        m_pLogProlTrack = OpenLog("ProlongedTrack", AIR_BALL::log_path);
    }
    catch(...)
    {
        DPS_ASSERT(false);
    }
}


void CAirBallist_Processing_GD::SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables, Str_Ar_BallCoef_Param_AllTables *p_arBallCoef_Tables)
{
    QMutexLocker l_lock(&m_mutexGD);
    m_pEAP_Tables = p_arEAP_Tables;
    m_BallProlong.SetPersistentData(p_arEAP_Tables, p_arBallCoef_Tables);
}


void CAirBallist_Processing_GD::SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables)
{
    QMutexLocker l_lock(&m_mutexGD);
    m_pEAP_Tables = p_arEAP_Tables;
    m_BallProlong.SetPersistentData(p_arEAP_Tables);
}


void CAirBallist_Processing_GD::FillHeBoundaryValues(Cells_BMMark_Data *pArrCells)
{
    QMutexLocker l_lock(&m_mutexGD);
    if (pArrCells != nullptr) {
        m_BallSubclDet.FillHeBoundaryValues(pArrCells);
    }
}


void CAirBallist_Processing_GD::SetActionPhase(const qint32 NumbGT, const qint16 ActionPhase, qreal CurTime)
{
    if (0 < NumbGT && NumbGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);
        CAirBall_Proc_DataForGTNumber *pCurData = &m_arProcData.arData__NumbGT[NumbGT];

        if (ActionPhase >= NO_ACTION && ActionPhase <= REASSIGNED
                && ActionPhase != IC_EXPLOSION /*MEET_POINT*/)
        {
            if (pCurData->ActionPhase < KILL_ASSESSMENT || pCurData->ActionPhase == REASSIGNED
                    || ActionPhase >= KILL_ASSESSMENT)
            {
                if (ActionPhase == REASSIGNED)
                {                    
                    if (pCurData->ActionPhase < KILL_ASSESSMENT || pCurData->ActionPhase == REASSIGNED
                            || CurTime - pCurData->tObtainActPhase > c_ControlTimeNearMeet)
                    {
                        pCurData->ActionPhase = ActionPhase;
                        pCurData->tObtainActPhase = CurTime;
                    }
                }
                else
                {
                    pCurData->ActionPhase = ActionPhase;
                    pCurData->tObtainActPhase = CurTime;
                }
            }
        }
    }
}


void CAirBallist_Processing_GD::SetBallCoeffSrc(const qint32 NumbGT, const qreal Gamma)
{
    if (0 < NumbGT && NumbGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);
        m_arProcData.arData__NumbGT[NumbGT].GammaSrc = Gamma;
    }
}


void CAirBallist_Processing_GD::SetNumbGT(const qint32 IndGT, const qint32 NumbGT)
{
    if (0 <= IndGT && IndGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        if (0 < NumbGT && NumbGT < AIR_BALL::GT_FORM_AMOUNT)
        {
            QMutexLocker l_lock(&m_mutexGD);
            m_arProcData.arData_GT[IndGT].NumbGT = NumbGT;
        }
    }
}


qreal CAirBallist_Processing_GD::GetMinActPathTimeByMark(qint32 indGT)
{
    qreal TimeAP = AIR_BALL::Timer_INI;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);
        CAirBall_Proc_InnerData_GT *pCurData = &m_arProcData.arData_GT[indGT]; //pointer to the data for current GT

        qreal TimeMin = con_large_value; //minimum AP duration time value
        for (qint16 i = pCurData->BMMark.minBMMark; i<= pCurData->BMMark.maxBMMark; i++)
        {
            if (AIR_BALL::UNKNOWN_OF_SHORT_RANGE < i && i < AIR_BALL::UNKNOWN_OF_LONG_RANGE)
            {
                Str_Ar_EAP_Param_1Table *pEAP_Table = m_pEAP_Tables->getCellMark(i); //pointer to the current EAP table
                if (pEAP_Table != nullptr)
                {
                    if (pEAP_Table->t_EAP > con_par_eps && pEAP_Table->t_EAP < TimeMin)
                    {
                        TimeMin = pEAP_Table->t_EAP;
                    }
                }
            }
        }

        if (TimeMin > con_par_eps && TimeMin - con_large_value<  -con_par_eps)
        {
            TimeAP = TimeMin;
        }
    }
    return TimeAP;
}


qreal CAirBallist_Processing_GD::Estimate_AB_Distance(qint32 indGT)
{
    qreal Res_AB_Distance = -1; //resulting value of distance

    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);

        const CAirBall_Proc_InnerData_GT *pCurData = & m_arProcData.arData_GT[indGT]; //pointer to the currend GT data
        if (pCurData->D > con_par_eps)
        {
            qreal MultCoef;
            if (pCurData->Hapogee > AIR_BALL::MAX_H_APOGEE_QBM || pCurData->D > AIR_BALL::MAX_DIST_QBM)
            {
                //possible MaRV
                MultCoef = AIR_BALL::MAX_D_PERCENTAGE_MAN_MARV / AIR_BALL::c100;
            }
            else
            {
                //possible QBM
                MultCoef = AIR_BALL::MAX_D_PERCENTAGE_MAN_QBM / AIR_BALL::c100;
            }

            Res_AB_Distance = pCurData->D * MultCoef;
        }
    }

    return Res_AB_Distance;
}


void CAirBallist_Processing_GD::GetFlatLoftBound_Mark(qreal &bound_flat_tr, qreal &bound_lofted_tr)
{
    bound_flat_tr = 0.;
    bound_lofted_tr = 0.;
}


void CAirBallist_Processing_GD::Drop_GT(const qint32 ind_GT, const qint32 ind_Ball)
{    
    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        ClstClearForGT(ind_GT, true);

        QMutexLocker l_lock(&m_mutexGD);
        m_PathBranchDet.Drop_GT(ind_GT);

        if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
        {
            m_BallSubclDet.Drop_BallTrack(ind_Ball);
            m_BallProlong.Drop_BallTrack(ind_Ball);
        }

        qint32 NumGT = m_arProcData.arData_GT[ind_GT].NumbGT;
        if (0 < NumGT && NumGT < AIR_BALL::GT_FORM_AMOUNT)
        {
            m_arProcData.arData__NumbGT[NumGT].Reset();
        }
        l_lock.unlock();

        ClearInnerData_1Track(ind_GT);
    }
}


void CAirBallist_Processing_GD::Drop_ST(const qint32 ind_ST)
{
    if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);
        m_PathBranchDet.Drop_ST(ind_ST);
        m_arProcData.arData_ST[ind_ST].Reset();
    }
}


void CAirBallist_Processing_GD::ClearBallisticData(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);
        m_BallSubclDet.Drop_BallTrack(ind_Ball);
        m_BallProlong.Drop_BallTrack(ind_Ball);
    }
}


void CAirBallist_Processing_GD::ClearInnerData()
{
    for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++)
    {
        ClearInnerData_1Track(i);
    }
}


void CAirBallist_Processing_GD::ClearInnerData_1Track(const qint32 index)
{
    if (0 <= index && index < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGD);
        m_arProcData.arData_GT[index].Reset();
    }
}


void CAirBallist_Processing_GD::InitConst()
{
    //prediction
    Pr_Const Const;

    Const.PrognPeriod_WH_Usual = c_PrognPeriod_WH_Usual;
    Const.PrognPeriod_WH_Usual_HI_LOAD = c_PrognPeriod_WH_Usual_HI_LOAD;
    Const.PrognPeriod_WH_Urgent = c_PrognPeriod_WH_Urgent;
    Const.PrognPeriod_WH_Urgent_HI_LOAD = c_PrognPeriod_WH_Urgent_HI_LOAD;
    Const.PrognPeriod_Boost = c_PrognPeriod_Boost;
    Const.PrognPeriod_Boost_HI_LOAD = c_PrognPeriod_Boost_HI_LOAD;
    Const.OnePrConsts.Tcontr = c_Tcontr;
    Const.Gamma_st = c_Gamma_st;
    Const.OnePrConsts.h0 = c_h0;
    Const.OnePrConsts.Delta_Eps = c_Delta_Eps;
    Const.OnePrConsts.step = c_step;
    Const.OnePrConsts.min_step = c_min_step;
    Const.OnePrConsts.min_step_CloseToEarth = c_min_step_CloseToEarth;
    Const.K_Gamma = c_K_Gamma;
    Const.OnePrConsts.H_Eps = c_H_Eps;
    Const.OnePrConsts.H_tolerance = c_H_tolerance;
    Const.OnePrConsts.VH_tolerance = c_VH_tolerance;
    Const.OnePrConsts.T_act_def = c_T_act_def;
    Const.dT_def_start_2method = c_dT_def_start_2method;
    Const.OnePrConsts.dTheta1 = c_dTheta1;
    Const.OnePrConsts.dTheta2 = c_dTheta2;
    Const.dT_corr_progn = c_dT_corr_progn;

    //prediction on active path
    Pr_Act_Const ConstAct;

    ConstAct.TPeriod = cTPeriod_ActPr;
    ConstAct.MinimumOfRange = cMinRange_ActPr;
    ConstAct.MaxumumOfRange = cMaxRange_ActPr_default;
    ConstAct.MaximumOfRangeExt = cMaxRange_ActPr_extended;
    ConstAct.DeltaAzCorr = cDeltaAzCorr;
    ConstAct.KSigmaAz = cKSigmaAz;

    QMutexLocker l_lock(&m_mutexGD);
    m_BallProlong.InitConst(Const);
    m_BallActPred.InitConst(ConstAct);
}


void CAirBallist_Processing_GD::ClstClearForGT(qint32 indGT, bool bGTDroped)
{
    QMutexLocker l_lock(&m_mutexGD);
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        CAirBall_Proc_InnerData_GT *pGTData = &m_arProcData.arData_GT[indGT];

        qint32 ClstNum = pGTData->ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            CAirBall_Proc_ClstGTData *pClstData = &m_arProcData.arData__Clst[ClstNum]; //pointer to the cluster data
            std::set<qint32>::iterator it_search = pClstData->SetGTInd.find(indGT);
            if (it_search != pClstData->SetGTInd.end())
            {
                pClstData->SetGTInd.erase(it_search);
            }

            if (pClstData->SetGTInd.empty())
            {
                pClstData->Reset();
            }
            else
            {
                if (pClstData->FstWHInd == indGT)
                {
                    m_arProcData.DetermineFstWHInClst(ClstNum);

                    if ((pClstData->ParentIndGT == indGT && bGTDroped)
                            || (pClstData->ParentIndGT != indGT && pClstData->ParentIndGT != pClstData->FstWHInd))
                    {
                        UpdateArrPointsForClst(ClstNum);
                    }
                }

                if (pClstData->ParentIndGT == indGT)
                {                    
                    m_arProcData.DetermineParentInClst(ClstNum);

                    if (bGTDroped)
                    {
                        pClstData->bParentDropped = true; //keep previous start point if previous parent track is dropped
                    }
                    else
                    {
                        //change start point and array of points if the parent track joins to the another cluster
                        if (fabs(pClstData->tStart - AIR_BALL::Timer_INI) > con_eps
                                && fabs(pClstData->tStart - pGTData->tStart) < con_eps)
                        {
                            pClstData->tStart = AIR_BALL::Timer_INI; //clear old data
                            pClstData->StartPoint.clear();
                            pClstData->StartEll.Reset();

                            if (!pClstData->InitialArrPoints.empty())
                            {
                                std::vector<TracksPoints>().swap(pClstData->InitialArrPoints);
                            }
                            pClstData->bArrPointsFilled = false;

                            if(pClstData->ParentIndGT >=0 && pClstData->ParentIndGT < AIR_BALL::GT_FORM_AMOUNT)
                            {
                                //update data for new GT
                                CAirBall_Proc_InnerData_GT *pNewParentGTData = &m_arProcData.arData_GT[pClstData->ParentIndGT];

                                if (fabs(pNewParentGTData->tStart - AIR_BALL::Timer_INI) > con_eps
                                        && !pNewParentGTData->StartPoint.IsZero()
                                        && !pNewParentGTData->StartEll.IsZero())
                                {
                                    pClstData->tStart = pNewParentGTData->tStart;
                                    pClstData->StartPoint = pNewParentGTData->StartPoint;
                                    pClstData->StartEll = pNewParentGTData->StartEll;

                                    if (pClstData->FstWHInd >= 0 && pClstData->FstWHInd < AIR_BALL::GT_FORM_AMOUNT
                                            && pClstData->FstWHInd != pClstData->ParentIndGT)
                                    {
                                        UpdateArrPointsForClst(ClstNum);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


void CAirBallist_Processing_GD::UpdateArrPointsForClst(qint32 ClstNum)
{
    QMutexLocker l_lock(&m_mutexGD);
    if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
    {
        qreal tEnd = AIR_BALL::Timer_INI;
        CAirBall_Proc_ClstGTData *pClst = &m_arProcData.arData__Clst[ClstNum];


        if (pClst->ParentIndGT >= 0 && pClst->ParentIndGT < AIR_BALL::GT_FORM_AMOUNT                
                && pClst->ParentIndGT != pClst->FstWHInd)
        {
            CAirBall_Proc_InnerData_GT *pParentGT = &m_arProcData.arData_GT[pClst->ParentIndGT]; //pointer to the GT data of parent track

            //choise of tEnd: tEnd if the appearance time of first WH if first WH exist; otherwise tEnd is the appearance time of 1st object in the cluster different from parent
            if (pClst->FstWHInd >= 0 && pClst->FstWHInd < AIR_BALL::GT_FORM_AMOUNT)
            {
                CAirBall_Proc_InnerData_GT *pFstWHGT  = &m_arProcData.arData_GT[pClst->FstWHInd]; //pointer to the GT data of track of first WH
                tEnd = pFstWHGT->tFirst;
            }
            else
            {
                std::set<qint32>::iterator it;
                for (it = pClst->SetGTInd.begin(); it != pClst->SetGTInd.end(); it++)
                {
                    qint32 GTind = (*it);
                    if (0 <= GTind && GTind < AIR_BALL::GT_FORM_AMOUNT
                            && GTind != pClst->ParentIndGT)
                    {
                        qreal tFirstCurr = m_arProcData.arData_GT[GTind].tFirst;
                        if (fabs(tFirstCurr - AIR_BALL::Timer_INI) > con_eps
                                && tFirstCurr > pParentGT->tFirst)
                        {
                            if (fabs(tEnd - AIR_BALL::Timer_INI) < con_eps)
                            {
                                tEnd = tFirstCurr;
                            }
                            else
                            {
                                if (tEnd > tFirstCurr)
                                {
                                    tEnd = tFirstCurr;
                                }
                            }
                        }
                    }
                }
            }

            if (fabs(tEnd - AIR_BALL::Timer_INI) > con_eps
                    && pParentGT->IndBall >= 0 && pParentGT->IndBall < AIR_BALL::BALL_FORM_AMOUNT)
            {
                CBallProl_Output ProlData;
                m_BallProlong.GetOutPoints(pClst->ParentIndGT, pParentGT->IndBall, ProlData);

                if (tEnd > pParentGT->tFirst
                        && ProlData.bArrPointsFilled)
                {
                    qreal tBegin = pParentGT->tStart;                    

                    if (!pClst->InitialArrPoints.empty())
                    {
                        qreal t0 = pClst->InitialArrPoints.back().time_point;
                        if (t0 > tBegin && tEnd > t0)
                        {
                            tBegin = t0;
                        }
                    }

                    for (qint16 i=0; i<AIR_BALL::N_OUT_POINTS; i++)
                    {
                        TracksPoints CurrEl = ProlData.arrPoints[i];
                        if (CurrEl.time_point > tBegin-con_eps)
                        {
                            if (tEnd > CurrEl.time_point)
                            {
                                pClst->InitialArrPoints.push_back(CurrEl);
                                pClst->bArrPointsFilled = true;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }

                    //correction by height in the cluster
                    if (!pClst->InitialArrPoints.empty()
                            && static_cast<qint16>(pClst->SetGTInd.size()) > 2)
                    {
                        std::set<qint32>::iterator it;
                        for (it = pClst->SetGTInd.begin(); it != pClst->SetGTInd.end(); it++)
                        {
                            qint32 GTIndCmp = (*it);
                            if (0 <= GTIndCmp && GTIndCmp < AIR_BALL::GT_FORM_AMOUNT
                                    && GTIndCmp != pClst->ParentIndGT
                                    && GTIndCmp != pClst->FstWHInd)
                            {
                                CAirBall_Proc_InnerData_GT *pGTCmp = &m_arProcData.arData_GT[GTIndCmp];
                                if (fabs(pGTCmp->tFirst - AIR_BALL::Timer_INI) > con_eps
                                        && tEnd > pGTCmp->tFirst)
                                {
                                    //if current GT in the cluster appeared earlier than first WH
                                    CBallistProlong_InnerData *pProlCmp = &m_BallProlong.m_arProlData[pGTCmp->IndBall];

                                    std::vector<TracksPoints>::iterator it_vec;
                                    for (it_vec = pClst->InitialArrPoints.begin(); it_vec != pClst->InitialArrPoints.end(); it_vec++)
                                    {
                                        TracksPoints *pCurPoint = it_vec.operator->();

                                        if (pCurPoint->time_point > pGTCmp->tFirst)
                                        {
                                            GLCalcTrajParam TrCalc;
                                            qreal HPoint = TrCalc.CalcHeight(pCurPoint->P);

                                            GLPointDouble3D PointCmp;
                                            bool bOK = pProlCmp->InterpolationProlongAtTime(pCurPoint->time_point, PointCmp);
                                            if (bOK)
                                            {
                                                qreal HCmp = TrCalc.CalcHeight(PointCmp);
                                                if (HCmp > HPoint)
                                                {
                                                    pCurPoint->P = PointCmp;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


//==================================================================================================

CAirBallist_Processing_M::CAirBallist_Processing_M(std::shared_ptr<CAirBallist_Processing_GD> pGD) :
    m_pGlobalData(pGD)
{
    m_pLogProlTrack = m_pGlobalData->m_pLogProlTrack;
}


CAirBallist_Processing_M::~CAirBallist_Processing_M() {
}


void CAirBallist_Processing_M::Reset()
{
    m_pInData = nullptr;
    m_pInData_ST = nullptr;

    m_OutData_Ballist.Reset();
    m_TrPar.Reset();

    m_pGlobalData->Reset();
    m_pLogProlTrack = m_pGlobalData->m_pLogProlTrack;
}


void CAirBallist_Processing_M::SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables,
                                                 Str_Ar_BallCoef_Param_AllTables *p_arBallCoef_Tables)
{
    m_pGlobalData->SetPersistentData(p_arEAP_Tables, p_arBallCoef_Tables);
}


void CAirBallist_Processing_M::SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables)
{
    m_pGlobalData->SetPersistentData(p_arEAP_Tables);
}


void CAirBallist_Processing_M::FillHeBoundaryValues(Cells_BMMark_Data *pArrCells)
{
    m_pGlobalData->FillHeBoundaryValues(pArrCells);
}


void CAirBallist_Processing_M::FillInputData(CAirBall_Proc_InputData_GT  &InData)
{
    m_pInData = &InData;
}


void CAirBallist_Processing_M::FillInputData(CAirBall_Proc_InputData_ST &InData)
{
    m_pInData_ST = &InData;
}


void CAirBallist_Processing_M::AirBallist_Realization()
{
    if (m_pInData == nullptr) {
        return;
    }

    m_OutData_Ballist.Reset();
    const qint32 CurIndex = m_pInData->IF_GT;

#ifdef KEEP_LOG
    AIR_BALL::out_IDs << std::setprecision(10) << "ID (function 'Realisation') "
                      << m_pInData->IF_GT << "\t" << m_pInData->tLoc << std::endl;
#endif //KEEP_LOG

    if (CurIndex < 0 || CurIndex >= AIR_BALL::GT_FORM_AMOUNT)
    {
#ifdef KEEP_LOG
        AIR_BALL::out_IDs << "   !!! WRONG ID !!!" <<std::endl;
#endif //KEEP_LOG
        return;
    }

    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    CAirBall_Proc_InnerData_GT *pDataGT = &m_pGlobalData->m_arProcData.arData_GT[CurIndex];

    if (!pDataGT->IsTaken) {
        NewTrackProcessing_GT();
    }

    GLPointDouble3D VelPrev = pDataGT->Vel;
    pDataGT->tLoc = m_pInData->tLoc;
    pDataGT->BallType = m_pInData->BallType;
    pDataGT->IndBall = m_pInData->IF_BALL;
    pDataGT->Vel = m_pInData->Vel;
    pDataGT->bAeroballistic = m_pInData->bAeroballistic;
    if (m_pInData->bAeroballistic && (pDataGT->t0_Aeroballistic - AIR_BALL::Timer_INI) < con_eps )
    {
        pDataGT->t0_Aeroballistic = m_pInData->tLoc;
    }

    UpdateCluster();

    if (fabs(m_pInData->tLoc - pDataGT->PrevTLoc) > con_eps2) {
        pDataGT->VelPrev = VelPrev;
    }

    if (m_pInData->Class != CLASS_BALLISTIC) {
        return;
    }

    const qreal tEAP_saved = pDataGT->tEAP;
    bool bEmptySP = pDataGT->StartPoint.IsZero();
    l_lock.unlock();

    CalcTrajParam();

    Path_Branch_Processing_GT();

    qint16 ActionPhase = NO_ACTION;
    qreal tObtainActPhase = AIR_BALL::Timer_INI;
    qint32 NumGT = m_pGlobalData->m_arProcData.arData_GT[CurIndex].NumbGT;
    if (0 < NumGT && NumGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        ActionPhase = m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].ActionPhase;
        tObtainActPhase = m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].tObtainActPhase;
    }

    bool l_toPredict = CurrGTIsWellSmoothed()
            || (m_pInData->tLoc - pDataGT->tLastWellSmoothed > AIR_BALL::cDTContrCheckWellSm
                && fabs(AIR_BALL::Timer_INI - pDataGT->tLastWellSmoothed) > con_eps)
            || ( (ActionPhase == KILL_ASSESSMENT || ActionPhase == ASSESSMENT_RESULT || ActionPhase == MEET_POINT
                     || ActionPhase == DESTROYED || ActionPhase == MISSED )
                 && fabs(m_pInData->tLoc - tObtainActPhase) < c_ControlTimeNearMeet);

    if( l_toPredict )
    {
        if( (pDataGT->Path == AIR_BALL::UNDEFINED_PATH)
                || (pDataGT->Path == AIR_BALL::ACTIVE_PATH) )
        {
            Active_Prediction_Processing();
        }
        else //passive path or end of active path
        {
            DeterminationBallCoeff();

            //qreal PathTime = (m_TrPar.Vh + sqrt(sqr(m_TrPar.Vh) + m_TrPar.H * con_g)) / con_g;
            qreal TBound = 0;//PathTime > c_ProlDelay ? std::min (c_ProlDelay*1.5, .2 * PathTime) : 0.;
            if ((pDataGT->tLoc - pDataGT->tEAP) > TBound) //pDataGT->tFirst > 60)
            {
              Prediction_Processing();
              TrajType_Processing();
              BT_Mark_Processing();
            }
        }
    }

    HorizontalManeuverDetection();
    TimeEAP_Updating();

    if ((pDataGT->Path == AIR_BALL::PASSIVE_PATH || pDataGT->Path == AIR_BALL::END_OF_ACTIVE_PATH)
            && (fabs(pDataGT->tEAP - tEAP_saved) > con_par_eps
                || (bEmptySP && !pDataGT->StartPoint.IsZero()
                    && pDataGT->T_EAP_finding_method != AIR_BALL::UNDEFINED_TEAP_METHOD)))
    {
        m_OutData_Ballist.bNewTEAP = true;
        ParametersEAP_Updating();
    }

    pDataGT->PrevTLoc = m_pInData->tLoc;
}


void CAirBallist_Processing_M::AirBallist_Realization_ST()
{
    if (m_pInData_ST == nullptr) {
        return;
    }

    const qint32 CurIndex = m_pInData_ST->IF_ST;
    if (CurIndex < 0 || CurIndex >= AIR_BALL::ST_FORM_AMOUNT) {
        return;
    }

    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    if (!m_pGlobalData->m_arProcData.arData_ST[CurIndex].IsTaken )
    {
        NewTrackProcessing_ST();
    }
    else
    {
        qreal dt = m_pInData_ST->tLoc - m_pGlobalData->m_arProcData.arData_ST[CurIndex].tLoc;
        if (m_pGlobalData->m_arProcData.arData_ST[CurIndex].MeanPeriod > con_par_eps)
        {
            m_pGlobalData->m_arProcData.arData_ST[CurIndex].MeanPeriod =
                    AIR_BALL::cLambdaCalcPeriod * m_pGlobalData->m_arProcData.arData_ST[CurIndex].MeanPeriod +
                    (1. - AIR_BALL::cLambdaCalcPeriod) * dt;
        }
        else {
            m_pGlobalData->m_arProcData.arData_ST[CurIndex].MeanPeriod = dt;
        }
    }

    if (fabs(m_pInData_ST->tLoc - m_pGlobalData->m_arProcData.arData_ST[CurIndex].tLoc) > con_eps) {
        m_pGlobalData->m_arProcData.arData_ST[CurIndex].NUpdates ++;
    }

    m_pGlobalData->m_arProcData.arData_ST[CurIndex].tLoc = m_pInData_ST->tLoc;
    m_pGlobalData->m_arProcData.arData_ST[CurIndex].RMSE_mean =
            (m_pInData_ST->CovMatr.M[0][0] + m_pInData_ST->CovMatr.M[1][1] + m_pInData_ST->CovMatr.M[2][2]) / AIR_BALL::c3;
    m_pGlobalData->m_arProcData.arData_ST[CurIndex].Gamma = m_pInData_ST->Gamma;
    m_pGlobalData->m_arProcData.arData_ST[CurIndex].N_StepSmooth = m_pInData_ST->N_StepSmooth;

    m_pGlobalData->m_arProcData.arData_ST[CurIndex].bWellSmoothed = CurrSTIsWellSmoothed();
    l_lock.unlock();

    CalcTrajParam(false);
    Path_Branch_Processing_ST();
}


void CAirBallist_Processing_M::AirBallist_Realization_Satellite(CAirBall_Proc_OutData &OutData)
{
    OutData.Reset();
    qint32 IndGT = m_pInData->IF_GT;
    if (0 <= IndGT && IndGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        CAirBall_Proc_InnerData_GT *pDataGT = &m_pGlobalData->m_arProcData.arData_GT[IndGT];

        if (fabs(pDataGT->tPredictSatellite - AIR_BALL::Timer_INI) < con_eps
                || m_pInData->tLoc - pDataGT->tPredictSatellite > c_PrognPeriod_Satellite)
        {
            CBallProl_Input Inp;
            CBallProl_Output ProlOut;
            Inp.IF_GT = IndGT;
            Inp.tLoc = m_pInData->tLoc;
            Inp.Coord = m_pInData->Coord;
            Inp.Vel = m_pInData->Vel;
            Inp.Acc = m_pInData->Acc;

            m_pGlobalData->m_BallProlong.FillInputData(Inp);
            m_pGlobalData->m_BallProlong.CalcPolynomSatellite(ProlOut);

            if (ProlOut.SignProgn == AIR_BALL::NEW_PROGNOSIS)
            {
                OutData.bNewPrediction = true;
                OutData.Polynoms = ProlOut.Polynoms;
                pDataGT->tPredictSatellite = m_pInData->tLoc;
            }
        }
    }
}


void CAirBallist_Processing_M::CalcTrajParam(const bool IsGT)
{
    GLCalcTrajParam cl_CalcParam;

    GLVector Pos(9);
    GLMatrix *pCurrMatr;

    m_TrPar.Reset();

    if (IsGT)
    {       //for generalized track
        Pos.Vec[0] = m_pInData->Coord.x;
        Pos.Vec[1] = m_pInData->Coord.y;
        Pos.Vec[2] = m_pInData->Coord.z;
        Pos.Vec[3] = m_pInData->Vel.x;
        Pos.Vec[4] = m_pInData->Vel.y;
        Pos.Vec[5] = m_pInData->Vel.z;
        Pos.Vec[6] = m_pInData->Acc.x;
        Pos.Vec[7] = m_pInData->Acc.y;
        Pos.Vec[8] = m_pInData->Acc.z;
        pCurrMatr  = &m_pInData->CovMatr;
    }
    else
    {       //for single track
        Pos.Vec[0] = m_pInData_ST->Coord.x;
        Pos.Vec[1] = m_pInData_ST->Coord.y;
        Pos.Vec[2] = m_pInData_ST->Coord.z;
        Pos.Vec[3] = m_pInData_ST->Vel.x;
        Pos.Vec[4] = m_pInData_ST->Vel.y;
        Pos.Vec[5] = m_pInData_ST->Vel.z;
        Pos.Vec[6] = m_pInData_ST->Acc.x;
        Pos.Vec[7] = m_pInData_ST->Acc.y;
        Pos.Vec[8] = m_pInData_ST->Acc.z;
        pCurrMatr  = &m_pInData_ST->CovMatr;
    }

    cl_CalcParam.Initialize(Pos);

    GLMatrix Kc(3,3), Kv(3,3), Kcv(6,6), Kca(6,6), Ka(3,3);
    CMatrix cl_Matr;
    bool bOK = cl_Matr.Submatrix(*pCurrMatr, 0, 2, 0, 2, Kc);
    bOK = bOK && cl_Matr.Submatrix(*pCurrMatr, 3, 5, 3, 5, Kv);
    bOK = bOK && cl_Matr.Submatrix(*pCurrMatr, 0, 5, 0, 5, Kcv);
    bOK = bOK && cl_Matr.Submatrix(*pCurrMatr, 6, 8, 6, 8, Ka);
    for (qint32 i=0; i<3; i++)
    {
        for (qint32 j=0; j<3; j++)
        {
            Kca.M[i][j] = pCurrMatr->M[i][j];
            Kca.M[i][j+3] = pCurrMatr->M[i][j+6];
            Kca.M[i+3][j] = pCurrMatr->M[i+6][j];
            Kca.M[i+3][j+3] = pCurrMatr->M[i+6][j+6];
        }
    }

    m_TrPar.H = cl_CalcParam.CalcHeight(Pos);
    m_TrPar.V = cl_CalcParam.CalcAbsVelocity(Pos);
    m_TrPar.Vh = cl_CalcParam.CalcVertVel(Pos);
    m_TrPar.He = cl_CalcParam.CalcEnHeight_EarthCenter(Pos);
    m_TrPar.ah = cl_CalcParam.CalcVertAccel(Pos);
    m_TrPar.a = cl_CalcParam.CalcAbsAcceleration(Pos);
    m_TrPar.ah_noC = cl_CalcParam.CalcVertAccelWithoutCoriolis(Pos);

    m_TrPar.SigH = cl_CalcParam.CalcRMSE_Height(Pos, Kc);
    m_TrPar.SigV = cl_CalcParam.CalcRMSE_AbsVelocity(Pos, Kv);
    m_TrPar.SigVh = cl_CalcParam.CalcRMSE_VertVel(Pos, Kcv);
    m_TrPar.SigHe = cl_CalcParam.CalcRMSE_EnHeight_EarthCenter(Pos, Kcv);
    m_TrPar.SigAh = cl_CalcParam.CalcRMSE_VertAccel(Pos, Kca);
    if(IsGT)
    {
        m_TrPar.SigA = cl_CalcParam.CalcRMSE_AbsAccel(m_pInData->Coord.x, m_pInData->Coord.y,
                                                m_pInData->Coord.z, Ka);
    }
    else
    {
        m_TrPar.SigA = cl_CalcParam.CalcRMSE_AbsAccel(m_pInData_ST->Coord.x, m_pInData_ST->Coord.y,
                                                       m_pInData_ST->Coord.z, Ka);
    }

#ifdef KEEP_LOG
    std::streamsize ss = AIR_BALL::out_ball_traj_param.precision();
    AIR_BALL::out_ball_traj_param << std::setprecision(10)
                            << (IsGT ? m_pInData->tLoc : m_pInData_ST->tLoc) << "\t"
                            << (IsGT ? "GT #" : "ST #") << "\t"
                            << (IsGT ? m_pInData->IF_GT : m_pInData_ST->IF_ST) << "\t"
                            << m_TrPar.H << "\t" << m_TrPar.V << "\t" << m_TrPar.Vh << "\t"
                            << m_TrPar.He << "\t" <<m_TrPar.ah << "\t" << m_TrPar.a << "\t"
                            << m_TrPar.SigH << "\t" << m_TrPar.SigV << "\t"
                            << m_TrPar.SigVh << "\t" << m_TrPar.SigHe << "\t"
                            << m_TrPar.SigAh << "\t" << m_TrPar.SigA << std::endl;
    AIR_BALL::out_ball_traj_param.precision(ss);
#endif //KEEP_LOG
}


bool CAirBallist_Processing_M::CurrSTIsWellSmoothed()
{
    bool bRes = false;

    const qint32 CurIndex = m_pInData_ST->IF_ST;
    if (CurIndex < 0 || CurIndex >= AIR_BALL::ST_FORM_AMOUNT)
    {
        return bRes;
    }

    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    auto l_NUpdates = m_pGlobalData->m_arProcData.arData_ST[CurIndex].NUpdates;
    l_lock.unlock();

    if (m_pInData_ST->N_StepSmooth >= AIR_BALL::cN_Well_Sm && l_NUpdates >= AIR_BALL::cN_Well_Sm)
    {
        bRes = true;
    }
    else
    {
        if (m_pInData_ST->CovMatr.s_m >= 9)
        {
            qreal maxSigmaC, maxSigmaV, maxSigmaA;
            maxSigmaC = sqrt(std::max(m_pInData_ST->CovMatr.M[0][0], std::max(m_pInData_ST->CovMatr.M[1][1], m_pInData_ST->CovMatr.M[2][2])));
            if (maxSigmaC < AIR_BALL::cRMSE_Coord_Well_Sm)
            {
                maxSigmaV = sqrt(std::max(m_pInData_ST->CovMatr.M[3][3], std::max(m_pInData_ST->CovMatr.M[4][4], m_pInData_ST->CovMatr.M[5][5])));
                if (maxSigmaV < AIR_BALL::cRMSE_Vel_Well_Sm)
                {
                    maxSigmaA = sqrt(std::max(m_pInData_ST->CovMatr.M[6][6], std::max(m_pInData_ST->CovMatr.M[7][7], m_pInData_ST->CovMatr.M[8][8])));
                    if (maxSigmaA < AIR_BALL::cRMSE_Acc_Well_Sm)
                    {
                        bRes = true;
                    }
                }
            }
        }
    }

    return bRes;
}


bool CAirBallist_Processing_M::CurrGTIsWellSmoothed()
{
    bool bRes = true;

    qint32 GTind = m_pInData->IF_GT;
    if (0 <= GTind && GTind < AIR_BALL::GT_FORM_AMOUNT)
    {
        std::set<qint32>::iterator it;
        qreal meanSrcPeriod = 0;
        qint16 counter = 0; //counter of ST
        for (it = m_pInData->SetST.begin(); it != m_pInData->SetST.end(); it++)
        {
            qint32 indST = (*it);
            if (0 <= indST && indST < AIR_BALL::ST_FORM_AMOUNT)
            {
                QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
                meanSrcPeriod += m_pGlobalData->m_arProcData.arData_ST[indST].MeanPeriod;
                counter++;
            }
        }

        if (counter > 0)
        {
            meanSrcPeriod /= counter;

            for (it = m_pInData->SetST.begin(); it != m_pInData->SetST.end(); it++)
            {
                qint32 indST = (*it);
                if (0 <= indST && indST < AIR_BALL::ST_FORM_AMOUNT)
                {
                    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
                    if (m_pGlobalData->m_arProcData.arData_ST[indST].MeanPeriod < AIR_BALL::cCoefCmpPeriod * meanSrcPeriod
                            && !m_pGlobalData->m_arProcData.arData_ST[indST].bWellSmoothed)
                    {
                        bRes = false;
                        break;
                    }
                }
            }
        }
        else {
            bRes = false;
        }

        if (bRes)
        {
            QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
            m_pGlobalData->m_arProcData.arData_GT[GTind].tLastWellSmoothed = m_pInData->tLoc;
        }
    }
    else {
        bRes = false;
    }

    return bRes;
}


void CAirBallist_Processing_M::Path_Branch_Processing_GT()
{
    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

    const qint32 CurIndex = m_pInData->IF_GT;
    CAirBall_Proc_InnerData_GT *pGTData = &m_pGlobalData->m_arProcData.arData_GT[CurIndex]; //pointer to the GT data in the array
    qint16 PrevPath = pGTData->Path;

    //fill input data
    CPathBranch_InputData_GT InpPathBranch;

    InpPathBranch.IF_GT = CurIndex;
    InpPathBranch.tLoc  = m_pInData->tLoc;
    InpPathBranch.SetST.clear();
    std::set<qint32>::iterator it;
    for (it = m_pInData->SetST.begin(); it != m_pInData->SetST.end(); ++it)
    {
        InpPathBranch.SetST.insert((*it));
    }
    InpPathBranch.TypeGT = m_pInData->BallType;
    InpPathBranch.tPrevPathDet = pGTData->tPrevPathDet;

    m_pGlobalData->m_PathBranchDet.FillInputData_GT(InpPathBranch);
    m_pGlobalData->m_PathBranchDet.PathBranchDet_Realization_GT();

    pGTData->tPrevPathDet = m_pInData->tLoc;
    const CPathBranch_OutputData_GT *pOutDataPBr = &m_pGlobalData->m_PathBranchDet.m_OutData_PathBranch_GT; //output data from path/branch determination algorithm
    const CPathBranch_SavedData_GT *pDataPBr = &m_pGlobalData->m_PathBranchDet.m_arPathBrData_GT[CurIndex]; //saved data in the path/branch determination algorithm

    if (pOutDataPBr->bPathBranchChanged)
    {
        m_OutData_Ballist.bNewPathBranch = true;
        pGTData->Path = pDataPBr->P_Path;
        pGTData->Branch = pDataPBr->P_Branch;
        if (pDataPBr->P_Path == AIR_BALL::END_OF_ACTIVE_PATH)
        {
            if (fabs(pDataPBr->T_eap - AIR_BALL::Timer_INI) > con_par_eps) //transition from active to passive path
            {
                pGTData->tEAP = pDataPBr->T_eap;
                pGTData->T_EAP_finding_method = AIR_BALL::TEAP_BY_TRANSITION_ACT_PASS;

                pGTData->EAP_Point.time     = SEC70_to_MSEC70(pGTData->tEAP);
                pGTData->EAP_Point.coord    = m_pInData->Coord;
                pGTData->EAP_Point.vel      = m_pInData->Vel;
                pGTData->EAP_Point.accel    = m_pInData->Acc;

                GLCalcTrajParam GeomCalc;
                pGTData->EAP_parameters.H = m_TrPar.H;
                pGTData->EAP_parameters.V = m_TrPar.V;
                pGTData->EAP_parameters.theta = con_180_div_PI * GeomCalc.CalcAngleVelHorizont(pGTData->EAP_Point.coord, pGTData->EAP_Point.vel);
                pGTData->EAP_parameters.L = pGTData->FirstPoint.getDistanceOnSurface(pGTData->EAP_Point.coord);
            }

            pGTData->tLastWellSmoothed = m_pInData->tLoc;
        }
        l_lock.unlock();

        if ( ( (PrevPath == AIR_BALL::PASSIVE_PATH)
                    || (PrevPath == AIR_BALL::END_OF_ACTIVE_PATH))
             && (pDataPBr->P_Path == AIR_BALL::ACTIVE_PATH))
        {       //if previous path is passive and current path is active
            ReturnFromPassToAct_Processing();
        }
    }
}


void CAirBallist_Processing_M::Path_Branch_Processing_ST()
{
    const qint32 CurIndex = m_pInData_ST->IF_ST;

    //fill input data
    CPathBranch_InputData_ST InpPathBranch;

    InpPathBranch.t_Loc = m_pInData_ST->tLoc;
    InpPathBranch.IF_ST = CurIndex;
    InpPathBranch.Type = m_pInData_ST->BallType;
    InpPathBranch.TypeGT = TYPE_UNDEFINED;
    if (m_pInData_ST->IF_GT >= 0 && m_pInData_ST->IF_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        InpPathBranch.TypeGT = m_pGlobalData->m_arProcData.arData_GT.at(m_pInData_ST->IF_GT).BallType;
        InpPathBranch.bAeroballistic = m_pGlobalData->m_arProcData.arData_GT.at(m_pInData_ST->IF_GT).bAeroballistic;
    }

//    InpPathBranch.TrParameters.H = m_TrPar.H;
//    InpPathBranch.TrParameters.Vh = m_TrPar.Vh;
//    InpPathBranch.TrParameters.ah = m_TrPar.ah;
//    InpPathBranch.TrParameters.SigH = m_TrPar.SigH;
//    InpPathBranch.TrParameters.SigVh = m_TrPar.SigVh;
//    InpPathBranch.TrParameters.SigAh = m_TrPar.SigAh;
    InpPathBranch.TrParameters = m_TrPar;
    InpPathBranch.pInDataST = m_pInData_ST;
    InpPathBranch.Prob_IMM_small_coeff = m_pInData_ST->Prob_IMM_small_coeff;
    InpPathBranch.Prob_IMM_large_coeff = m_pInData_ST->Prob_IMM_large_coeff;
    InpPathBranch.R_rdr_obj = m_pInData_ST->R_rdr_obj;

    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    m_pGlobalData->m_PathBranchDet.FillInputData_ST(InpPathBranch);
    m_pGlobalData->m_PathBranchDet.PathBranchDet_Realization_ST();

//    m_OutData_Ballist.P_Save_prev_PathBranchData = m_PathBranchDet.m_OutData_PathBranch.P_Save_prev_PathBranchData;
    if (!m_pGlobalData->m_PathBranchDet.m_OutData_PathBranch_ST.P_Save_prev_PathBranchData)
    {
        m_pGlobalData->m_arProcData.arData_ST[CurIndex].Path = m_pGlobalData->m_PathBranchDet.m_OutData_PathBranch_ST.P_Path;
        m_pGlobalData->m_arProcData.arData_ST[CurIndex].Branch = m_pGlobalData->m_PathBranchDet.m_OutData_PathBranch_ST.P_Branch;

//        m_OutData_Ballist.bNewPathBranch = true;
    }

//    m_OutData_Ballist.Path = m_arProcData.Form[CurIndex].Path;
//    m_OutData_Ballist.Branch = m_arProcData.Form[CurIndex].Branch;
}


void CAirBallist_Processing_M::Active_Prediction_Processing()
{
    const qint32 ind_GT = m_pInData->IF_GT;

#ifdef KEEP_LOG
    AIR_BALL::out_IDs << std::setprecision(10) << "ID (function 'Active prediction') "
                      << m_pInData->IF_GT << "\t" << m_pInData->tLoc << std::endl;
#endif //KEEP_LOG

    if (ind_GT < 0 || ind_GT >= AIR_BALL::GT_FORM_AMOUNT)
    {
#ifdef KEEP_LOG
        AIR_BALL::out_IDs << "   !!! Wrong ID !!!" << std::endl;
#endif
        return;
    }

    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    CAirBall_Proc_InnerData_GT* pCurrData = &m_pGlobalData->m_arProcData.arData_GT[ind_GT];

    Pr_Act_Input Inp;
    Pr_Act_Out Out;
    Inp.tL = m_pInData->tLoc;
    //Inp.tLastActProgn = pCurrData->ActPredictInfo.t_calc_act;
    Inp.tLastActProgn = pCurrData->t_calc_act;
    Inp.X = m_pInData->Coord.x;
    Inp.Y = m_pInData->Coord.y;
    Inp.Z = m_pInData->Coord.z;
    Inp.VX = m_pInData->Vel.x;
    Inp.VY = m_pInData->Vel.y;
    Inp.VZ = m_pInData->Vel.z;
    Inp.SigVX = sqrt(m_pInData->CovMatr.M[3][3]);
    Inp.SigVY = sqrt(m_pInData->CovMatr.M[4][4]);
    Inp.SigVZ = sqrt(m_pInData->CovMatr.M[5][5]);
    Inp.H = m_TrPar.H;
    Inp.V = m_TrPar.V;
    Inp.Vh = m_TrPar.Vh;
    Inp.SigH = m_TrPar.SigH;
    Inp.SigV = m_TrPar.SigV;
    Inp.SigVH = m_TrPar.SigVh;
    Inp.SigAzPrev = pCurrData->CorrectSigAzPrev;
    Inp.EstStartPoint0 = pCurrData->ActPredictInfo.EstStartPoint;
    Inp.TimeStartEst0 = pCurrData->ActPredictInfo.TimeStartEst;
    Inp.Pbs = pCurrData->Path;
    Inp.ForcedActPr = false;
    Inp.p_arCovObj = m_pInData->p_arCovObj;
    CGeocentric Center_ECEF(pCurrData->FirstPoint.x, pCurrData->FirstPoint.y, pCurrData->FirstPoint.z);
    CGeodesic Center_GD;
    GEOCENTRIC_GEO(&Center_ECEF, &Center_GD);
    Inp.Center = Center_GD;

    m_pGlobalData->m_BallActPred.InitInput(Inp);
    m_pGlobalData->m_BallActPred.GetActiveProgn(Out);

    if (Out.P_Changed)
    {
        pCurrData->t_calc_act = m_pInData->tLoc;

        if (Out.ActPrInfo.New_ActPrognoz == AIR_BALL::NEW_ACT_PROGN)
        {
            pCurrData->ActPredictInfo = Out.ActPrInfo;
            pCurrData->CorrectSigAzPrev = Out.ActPrInfo.Trapeze.SigmaAzFly;
        }

        m_OutData_Ballist.bNewActPred = true;
    }
}


void CAirBallist_Processing_M::DeterminationBallCoeff()
{
    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

    qint32 IndGT = m_pInData->IF_GT;
    qreal GammaRes = m_pGlobalData->m_arProcData.arData_GT[IndGT].Gamma; //resulting value of Gamma
    std::set<qint32>::iterator it;
    qreal RMSEmin = con_large_value; //minimum value of RMSE, for comparison

    for (it = m_pInData->SetST.begin(); it != m_pInData->SetST.end(); it++)
    {       //ballistic coefficient obtained from filter, using single tracks
        qint32 IndST = (*it);
        CAirBall_Proc_InnerData_ST *pCurDataST = &m_pGlobalData->m_arProcData.arData_ST[IndST]; //pointer to the data for current ST

        if (pCurDataST->N_StepSmooth > c_NumSmoothUseGamma && pCurDataST->Gamma > con_par_eps)
        {
            if (pCurDataST->RMSE_mean > con_par_eps && pCurDataST->RMSE_mean < RMSEmin)
            {
                RMSEmin = pCurDataST->RMSE_mean;
                GammaRes = pCurDataST->Gamma;
            }
        }
    }

    if (GammaRes < con_par_eps)
    {       //ballistic coefficient obtained from source
        qint32 NumGT = m_pGlobalData->m_arProcData.arData_GT[IndGT].NumbGT;
        if (NumGT > 0 && NumGT < AIR_BALL::GT_FORM_AMOUNT)
        {
            if (m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].GammaSrc > con_par_eps)
            {
                GammaRes = m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].GammaSrc;
            }
        }

        if (GammaRes < con_par_eps)
        {       //ballistic coefficient obtained using BM Mark
            qint16 CurMark = m_pGlobalData->m_arProcData.arData_GT[IndGT].BMMark.MarkValue;
            if (CurMark != AIR_BALL::UNDEFINED_BM_MARK)
            {
               Str_Ar_EAP_Param_1Table *pCurTable = m_pGlobalData->m_pEAP_Tables->getCellMark(CurMark);
               if (pCurTable != nullptr)
               {
                   if (pCurTable->Gamma > con_par_eps)
                   {
                       GammaRes = pCurTable->Gamma;
                   }
               }
            }
        }

        if (GammaRes < con_par_eps)
        {
            GammaRes = c_Gamma_st; //default value of ballistic coefficient
        }
    }

    qreal GammaMax = c_Gamma_max; //maximum admissible value of ballistic coefficient
    if (m_pInData->BallType == SUBCLASS_WARHEAD)
    {
        GammaMax = c_Gamma_max_WH;
    }

    if ((GammaRes > con_par_eps) && (GammaRes - GammaMax < con_par_eps))
    {
        m_pGlobalData->m_arProcData.arData_GT[IndGT].Gamma = GammaRes;
    }
}


void CAirBallist_Processing_M::Prediction_Processing()
{
    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;
    if (0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT
            && 0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CBallistProlong_InnerData* pProlData = &m_pGlobalData->m_BallProlong.m_arProlData[ind_ball]; //pointer to the array of prolongated data
        CBallProl_Output ProlOut; //prolongation output data
        CAirBall_Proc_InnerData_GT *pGTData = &m_pGlobalData->m_arProcData.arData_GT[ind_GT];

        CBallProl_Input Inp;

        Inp.IF_GT = m_pInData->IF_GT;
        Inp.IF_BALL = m_pInData->IF_BALL;
        Inp.tLoc = m_pInData->tLoc;
        Inp.Coord = m_pInData->Coord;
        Inp.Vel = m_pInData->Vel;
        Inp.Acc = m_pInData->Acc;
        Inp.SignQuickReaction = m_pInData->SignQuickReaction;
        Inp.Sign_large_load = m_pInData->Sign_large_load;
        Inp.BallType = m_pInData->BallType;
        Inp.CovMatr = m_pInData->CovMatr;
        Inp.H = m_TrPar.H;
        Inp.bAeroballistic = m_pInData->bAeroballistic;

        qint32 NumGT = pGTData->NumbGT;
        if (0 < NumGT && NumGT < AIR_BALL::GT_FORM_AMOUNT)
        {
            Inp.ActionPhase = m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].ActionPhase;
            Inp.tObtainActionPhase = m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].tObtainActPhase;
        }
        //    Inp.bPossibleAB = m_pInData->bPossibleAB;
        //    Inp.AB_MaxDistance = m_pInData->AB_MaxDistance;

        m_pGlobalData->m_BallProlong.FillInputData(Inp);

        bool bStartProl = true;
        if (!pGTData->VelPrev.IsZero())
        {
            qreal AngleVel = m_pInData->Vel.getAngle(pGTData->VelPrev); //angle between current and previous velocity vectors
            if (fabs(AngleVel) > c_AngleVcurrVprev)
            {
                bStartProl = false;
            }
        }

        if (bStartProl) {
            m_pGlobalData->m_BallProlong.CalcStartPoint(ProlOut);
        }

        bool ForcedProlong = false;
        if (ProlOut.p_StartIsChanged)
        {
            //m_arProcData.Form[CurIndex].StartPoint = m_BallProlong.m_OutData.StartPoint;
            pGTData->StartPoint = pProlData->StartPoint;
            pGTData->tStart = pProlData->tStart;
            pGTData->StartEll = pProlData->StartEll;
            if (fabs(pProlData->Teap_byTable - AIR_BALL::Timer_INI) > con_par_eps)
            {
                pGTData->tEAP = pProlData->Teap_byTable;
                pGTData->T_EAP_finding_method = AIR_BALL::TEAP_BY_TABLE_EAP;
            }

            if (ProlOut.bBoostSepPointsChanged)
            {
                pGTData->PointSepBoost1 = ProlOut.PointSepBoost1;
                pGTData->PointSepBoost2 = ProlOut.PointSepBoost2;
                m_OutData_Ballist.bNewTEAP = true;
            }

            if (m_pGlobalData->m_arProcData.GTIsParentInCluster(ind_GT))
            {
                qint32 ClstNum = pGTData->ClstNum;
                CAirBall_Proc_ClstGTData *pClst = &m_pGlobalData->m_arProcData.arData__Clst[ClstNum];

                if (!pClst->bParentDropped
                        || fabs(pClst->tStart - AIR_BALL::Timer_INI) < con_eps
                        || pClst->StartPoint.IsZero()
                        || pClst->StartEll.IsZero())
                {
                    pClst->tStart = pProlData->tStart;
                    pClst->StartPoint = pProlData->StartPoint;
                    pClst->StartEll = pProlData->StartEll;
                }
            }

            ForcedProlong = true;

            m_OutData_Ballist.bNewSP = true;
        }

        bool bPredictConditions = CheckConditionsForPredict();

        if (bPredictConditions || ForcedProlong)
        {
            bool bPredictionDeviation = CheckPredictionDeviation();

            ForcedProlong = ForcedProlong || bPredictionDeviation;

            m_pGlobalData->m_BallProlong.CalcFallPoint(ProlOut, ForcedProlong);
        }

        if (ProlOut.p_FallIsChanged)
        {
            //m_arProcData.Form[CurIndex].FallPoint = m_BallProlong.m_OutData.FallPoint;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].FallPoint = pProlData->FallPoint;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].tFall = pProlData->tFall;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].FallEll = pProlData->FallEll;

            if (ProlOut.SignProgn == AIR_BALL::NEW_PROGNOSIS)
            {
                m_OutData_Ballist.bNewPrediction = true;
            }

            if ( (ProlOut.SignProgn == AIR_BALL::NEW_PROGNOSIS)
                 || (ProlOut.SignProgn == AIR_BALL::PROGNOSIS_WITH_LARGE_ELLIPSE))
            {
                m_OutData_Ballist.bNewPred_InternalUse = true;
                m_OutData_Ballist.Polynoms = ProlOut.Polynoms;

                if (m_pGlobalData->m_arProcData.GTIsParentInCluster(ind_GT)
                        && !m_pGlobalData->m_arProcData.GTIsFirstWHInCluster(ind_GT))
                {
                    qint32 ClstNum = pGTData->ClstNum;
                    CAirBall_Proc_ClstGTData *pClst = &m_pGlobalData->m_arProcData.arData__Clst[ClstNum];

                    if (!pClst->bArrPointsFilled)
                    {
                        m_pGlobalData->UpdateArrPointsForClst(ClstNum);
                    }
                }

                if ( pGTData->tEAP > con_eps && (m_pInData->tLoc - pGTData->tEAP) > c_OutPointsDelay
                        && pProlData->D > c_MinDistOutPnts)
                {
                    m_OutData_Ballist.arrPoints = ProlOut.arrPoints;
                }
            }

            const bool bThreat = Estimate_Threat(ind_ball); //estimation of direct threat
            if (!bThreat)
            {       //calculation of trapeze in the case of absence of direct threat
                Inp.AB_MaxDistance = m_pGlobalData->Estimate_AB_Distance(ind_GT);

                m_OutData_Ballist.bNewABTrapeze = m_pGlobalData->m_BallProlong.CalculateTrapezeAB();

                if (m_OutData_Ballist.bNewABTrapeze)
                {
                    m_pGlobalData->m_arProcData.arData_GT[ind_GT].AB_Trapeze = ProlOut.AB_Trapeze;
                }
            }
        }

        if (ProlOut.p_StartIsChanged || ProlOut.p_FallIsChanged)
        {
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].D = pProlData->D;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].Hapogee = pProlData->Hapogee;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].tApogee = pProlData->tApogee;

#ifdef LOG_PROL_TRACK
            if (NumGT == 70 || NumGT == 71){
            pProlData->LogProlongedTrack(m_pLogProlTrack, m_pInData->tLoc, NumGT);
            ProlOut.LogTrackPoints(m_pLogProlTrack, m_pInData->tLoc, NumGT);}
#endif //LOG_PROL_TRACK
        }
    }
}


void CAirBallist_Processing_M::TrajType_Processing()
{

    if (0 <= m_pInData->IF_BALL && m_pInData->IF_BALL < AIR_BALL::BALL_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        const qint32 CurIndex = m_pInData->IF_GT;

        //fill input data
        TrajType_InputData InpTrajType;

        InpTrajType.IndBall = m_pInData->IF_BALL;
        InpTrajType.tLoc = m_pInData->tLoc;
        InpTrajType.H = m_TrPar.H;
        InpTrajType.VH = m_TrPar.Vh;
        InpTrajType.AH = m_TrPar.ah;
        InpTrajType.D = m_pGlobalData->m_arProcData.arData_GT[CurIndex].D;
        InpTrajType.throw_angle = m_pGlobalData->m_arProcData.arData_GT[CurIndex].EAP_parameters.theta; //.ThrowAngle;

        qreal bound_flat_tr = 0., bound_lofted_tr = 0.;
        m_pGlobalData->GetFlatLoftBound_Mark(bound_flat_tr, bound_lofted_tr);
        InpTrajType.bound_flat_tr = bound_flat_tr;
        InpTrajType.bound_lofted_tr = bound_lofted_tr;

        m_pGlobalData->m_BallSubclDet.FillInputData_TType(InpTrajType);
        m_pGlobalData->m_BallSubclDet.TrajTypeDet_Realization();

        if (m_pGlobalData->m_BallSubclDet.m_OutTrajType.P_rewrite_TT)
        {
            if (m_pGlobalData->m_arProcData.arData_GT[CurIndex].TrajType != m_pGlobalData->m_BallSubclDet.m_OutTrajType.TrajType)
            {
                m_pGlobalData->m_arProcData.arData_GT[CurIndex].TrajType = m_pGlobalData->m_BallSubclDet.m_OutTrajType.TrajType;
                m_OutData_Ballist.bNewTrajType = true;
            }
        }
        //    m_OutData_Ballist.TrajType = m_arProcData.Form[CurIndex].TrajType;
    }
}


void CAirBallist_Processing_M::BT_Mark_Processing()
{
    if (m_pInData->IF_BALL >= 0 && m_pInData->IF_BALL < AIR_BALL::BALL_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        const qint32 CurIndex = m_pInData->IF_GT;

        //fill input data
        BTMark_InputData InpBTMark;
        InpBTMark.IndBall = m_pInData->IF_BALL;
        InpBTMark.tLoc = m_pInData->tLoc;
        InpBTMark.tFall = m_pGlobalData->m_arProcData.arData_GT[CurIndex].tFall;
        //    InpBTMark.H = m_TrPar.H;
        //    InpBTMark.He = m_TrPar.He;
        InpBTMark.pTrParameters = &m_TrPar;
        InpBTMark.D = m_pGlobalData->m_arProcData.arData_GT[CurIndex].D;
        InpBTMark.SigmaSP = m_pGlobalData->m_arProcData.arData_GT[CurIndex].StartEll.aI +
                m_pGlobalData->m_arProcData.arData_GT[CurIndex].FallEll.aI;
        InpBTMark.Theta = m_pGlobalData->m_arProcData.arData_GT[CurIndex].EAP_parameters.theta; //.ThrowAngle;
        InpBTMark.Veap = m_pGlobalData->m_arProcData.arData_GT[CurIndex].EAP_parameters.V; //.Veap;
        InpBTMark.Branch = m_pGlobalData->m_arProcData.arData_GT[CurIndex].Branch;
        InpBTMark.Cells_BMMark = m_pInData->Cells_BMMark;

        m_pGlobalData->m_BallSubclDet.FillInputData_BTMark(InpBTMark);
        m_pGlobalData->m_BallSubclDet.BMMarkDet_Realization();

        if (m_pGlobalData->m_BallSubclDet.m_OutMark.P_MarkIsChanged)
        {
            const BMMark_AndDiapason *pNewMark = &m_pGlobalData->m_BallSubclDet.m_OutMark.BMMark;
            const BMMark_AndDiapason *pSavedMark = &m_pGlobalData->m_arProcData.arData_GT[CurIndex].BMMark;

            if (pNewMark->IsSingleValued != pSavedMark->IsSingleValued
                    || pNewMark->MarkValue != pSavedMark->MarkValue
                    || pNewMark->minBMMark != pSavedMark->minBMMark
                    || pNewMark->maxBMMark != pSavedMark->maxBMMark)
            {
                m_pGlobalData->m_arProcData.arData_GT[CurIndex].BMMark = m_pGlobalData->m_BallSubclDet.m_OutMark.BMMark;
                m_OutData_Ballist.bNewSubcl = true;
                m_pGlobalData->m_BallProlong.SetSignBMMarkChanged(m_pInData->IF_BALL);
            }
        }
        //    m_OutData_Ballist.BMMark = m_arProcData.Form[CurIndex].BMMark;
    }
}


void CAirBallist_Processing_M::HorizontalManeuverDetection()
{
    bool bOK = true;
    const qint32 indGT = m_pInData->IF_GT;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CAirBall_Proc_InnerData_GT *pCurData = &m_pGlobalData->m_arProcData.arData_GT[indGT]; //pointer to the data for current GT

        if (fabs(m_pInData->tLoc - pCurData->PrevTLoc) > con_eps
                && !pCurData->FirstPoint.IsZero()
                && !m_pInData->Vel.IsZero())
        {
            CGeocentric FirstP_GC(pCurData->FirstPoint.x, pCurData->FirstPoint.y, pCurData->FirstPoint.z); //first point
            CGeodesic FirstP_GD;
            CGeocentric CurrV_GC(m_pInData->Vel.x, m_pInData->Vel.y, m_pInData->Vel.z); //velocity vector
            CTopocentric CurrV_TP;
            CSpherical CurrV_Sph;

            bOK = GEOCENTRIC_GEO(&FirstP_GC, &FirstP_GD);
            bOK = bOK && GEOCENTRIC_TOPO(&FirstP_GD, 0, &CurrV_GC, 0, 0, &CurrV_TP, 0);
            bOK = bOK && TOPO_SPHERICAL(&CurrV_TP, &CurrV_Sph); //velocity vector in the spherical coordinate system, relative to the first point of the track

            TMatrix<3> CovV_gc;
            TMatrix<3> CovV_tp;
            TMatrix<3> CovV_sph;
            TCMatrixFunc<3> FuncMatr;
            bOK = bOK && FuncMatr.Copy(CovV_gc, m_pInData->CovMatr, 3, 5, 3, 5);
            bOK = bOK && GL_MATR::Recount_CovMatr_GeocentricToTopo(&FirstP_GD, &CovV_gc, &CovV_tp);
            bOK = bOK && GL_MATR::Recount_CovMatr_NUEtoSPHCS_coord(&CurrV_TP, &CovV_tp, &CovV_sph); //function Recount_CovMatr_NUEtoSPHCS_coord is used
                                                           //because it is required to estimate RMSE of azimuth of velocity vector, not rate of azimuth

            T_U_SU DirCurr(m_pInData->tLoc, CurrV_Sph.m_dB, sqrt(CovV_sph.M[1][1])); //current direction: time, azimuth of velocity vector,
            //RMSE of azimuth of velocity vector
            if (bOK && !DirCurr.IsZero()
                    && fabs(CurrV_Sph.m_dE) < con_half_pi - cAngleDirMan*con_pi_div_180) //check for not vertical movement
            {
                if (!pCurData->DirPrev.IsZero() && !pCurData->DirPrev_1.IsZero() && !pCurData->DirPrev_2.IsZero())
                {
                    qreal DeltaBoundCurrPrev  = std::max(cCoefDirMan * AIR_BALL::K_Sigma * (DirCurr.SigU + pCurData->DirPrev.SigU),   cAngleDirMan*con_pi_div_180);
                    qreal DeltaBoundCurrPrev1 = std::max(cCoefDirMan * AIR_BALL::K_Sigma * (DirCurr.SigU + pCurData->DirPrev_1.SigU), cAngleDirMan*con_pi_div_180);
                    qreal DeltaBoundCurrPrev2 = std::max(cCoefDirMan * AIR_BALL::K_Sigma * (DirCurr.SigU + pCurData->DirPrev_2.SigU), cAngleDirMan*con_pi_div_180);

                    qreal DeltaAzCurrPrev  = SMODB_2PI(DirCurr.U, pCurData->DirPrev.U);
                    qreal DeltaAzCurrPrev1 = SMODB_2PI(DirCurr.U, pCurData->DirPrev_1.U);
                    qreal DeltaAzCurrPrev2 = SMODB_2PI(DirCurr.U, pCurData->DirPrev_2.U);

                    if (DeltaAzCurrPrev > DeltaBoundCurrPrev
                            || DeltaAzCurrPrev1 > DeltaBoundCurrPrev1
                            || DeltaAzCurrPrev2 > DeltaBoundCurrPrev2)
                    {
                        pCurData->ManTime   = m_pInData->tLoc;
                        pCurData->ManPoint  = m_pInData->Coord;
                    }
                }

                pCurData->DirPrev_2 = pCurData->DirPrev_1;
                pCurData->DirPrev_1 = pCurData->DirPrev;
                pCurData->DirPrev   = DirCurr;
            }
        }
    }
}


void CAirBallist_Processing_M::TimeEAP_Updating()
{
    const qint32 indGT = m_pInData->IF_GT;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CAirBall_Proc_InnerData_GT *pCurData = &m_pGlobalData->m_arProcData.arData_GT[indGT]; //pointer to the data for current GT
        if ((pCurData->Path == AIR_BALL::PASSIVE_PATH || pCurData->Path == AIR_BALL::END_OF_ACTIVE_PATH)
                && (m_OutData_Ballist.bNewSP
                    || m_OutData_Ballist.bNewSubcl
                    || (m_OutData_Ballist.bNewPathBranch && pCurData->Path == AIR_BALL::PASSIVE_PATH)))
        {
            if (pCurData->T_EAP_finding_method == AIR_BALL::UNDEFINED_TEAP_METHOD
                    || pCurData->T_EAP_finding_method == AIR_BALL::TEAP_BY_START_TIME)
            {
                if (fabs(pCurData->tStart - AIR_BALL::Timer_INI) > con_par_eps)
                {
                    qreal TimeMinAP = GetMinActPathTimeByMark(indGT); //minimum value of EAP time from tables
                    if (fabs(TimeMinAP - AIR_BALL::Timer_INI) > con_eps)
                    {
                        pCurData->tEAP = pCurData->tStart + TimeMinAP;
                    }
                    else
                    {       //default value of EAP time if value from table is undefined
                        pCurData->tEAP = pCurData->tStart + c_T_act_def;
                    }

                    pCurData->T_EAP_finding_method = AIR_BALL::TEAP_BY_START_TIME;
                }
            }

            const CPathBranch_SavedData_GT *pDataPBr = &m_pGlobalData->m_PathBranchDet.m_arPathBrData_GT[indGT]; //saved data in the path/branch determination algorithm
            if (fabs(pDataPBr->T_def_eap - AIR_BALL::Timer_INI) > con_eps)
            {
                if (pCurData->tEAP > pDataPBr->T_def_eap)
                {           //comparison with time of passive path determination
                    pCurData->tEAP = pDataPBr->T_def_eap;
                }

                if (pDataPBr->bActObserved && fabs(pDataPBr->T_def_eap - pCurData->tEAP) > DT_CTRL_PATH_REDETERM)
                {
                    if (pDataPBr->T_def_eap - pCurData->tEAP > cDT_EAP_margin)
                    {
                        pCurData->tEAP = pDataPBr->T_def_eap - cDT_EAP_margin;
                    }

                    std::set<qint32>::iterator it;
                    for (it = m_pInData->SetST.begin(); it != m_pInData->SetST.end(); it++)
                    {
                        qint32 STInd = (*it);
                        if (0 <= STInd && STInd < AIR_BALL::ST_FORM_AMOUNT)
                        {
                            qreal tActProuved = m_pGlobalData->m_PathBranchDet.m_arPathBrData_ST[STInd].T_act_prouved;
                            if (fabs(tActProuved - AIR_BALL::Timer_INI) > con_eps
                                    && fabs(tActProuved - pDataPBr->T_def_eap) < cDT_EAP_margin
                                    && tActProuved > pCurData->tEAP)
                            {
                                pCurData->tEAP = tActProuved;
                            }
                        }
                    }
                }
            }
        }
    }
}


void CAirBallist_Processing_M::ParametersEAP_Updating()
{
    const qint32 indGT = m_pInData->IF_GT;
    bool bOK = true;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CAirBall_Proc_InnerData_GT *pCurData = &m_pGlobalData->m_arProcData.arData_GT[indGT]; //pointer to the data for current GT
        const qint32 ind_ball = m_pInData->IF_BALL;
        if (0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT)
        {
            std::vector<M10> *pProlTrack = &m_pGlobalData->m_BallProlong.m_arProlData[ind_ball].ProlongedTrack; //pointer to the prolongated track

            qreal tEAP = pCurData->tEAP;

            const qint32 TrSize = pProlTrack->size();
            if (TrSize > 1)
            {
                qint32 k=0;
                std::vector<M10>::iterator it;
                for (it = pProlTrack->begin(); it != pProlTrack->end(); it++)
                {
                    M10 CurEl = (*it);
                    if (tEAP > CurEl.M_info[0])
                    {
                        k++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (k == 0)
                {
                    k = 1;
                }

                M10 PrevP = pProlTrack->at(k-1),
                    NextP = pProlTrack->at(k);

                pCurData->EAP_Point.time = SEC70_to_MSEC70(tEAP);
                if (fabs(tEAP - PrevP.M_info[0]) < con_par_eps)
                {
                    pCurData->EAP_Point.coord.x = PrevP.M_info[1];
                    pCurData->EAP_Point.coord.y = PrevP.M_info[2];
                    pCurData->EAP_Point.coord.z = PrevP.M_info[3];
                    pCurData->EAP_Point.vel.x = PrevP.M_info[4];
                    pCurData->EAP_Point.vel.y = PrevP.M_info[5];
                    pCurData->EAP_Point.vel.z = PrevP.M_info[6];
                    pCurData->EAP_Point.accel.x = PrevP.M_info[7];
                    pCurData->EAP_Point.accel.y = PrevP.M_info[8];
                    pCurData->EAP_Point.accel.z = PrevP.M_info[9];
                }
                else
                {
                    if (fabs(NextP.M_info[0] - PrevP.M_info[0]) > con_eps)
                    {
                        const qreal rel_T = (tEAP - PrevP.M_info[0])
                                                / (NextP.M_info[0] - PrevP.M_info[0]);

                        pCurData->EAP_Point.coord.x = PrevP.M_info[1] + rel_T * (NextP.M_info[1] - PrevP.M_info[1]);
                        pCurData->EAP_Point.coord.y = PrevP.M_info[2] + rel_T * (NextP.M_info[2] - PrevP.M_info[2]);
                        pCurData->EAP_Point.coord.z = PrevP.M_info[3] + rel_T * (NextP.M_info[3] - PrevP.M_info[3]);
                        pCurData->EAP_Point.vel.x = PrevP.M_info[4] + rel_T * (NextP.M_info[4] - PrevP.M_info[4]);
                        pCurData->EAP_Point.vel.y = PrevP.M_info[5] + rel_T * (NextP.M_info[5] - PrevP.M_info[5]);
                        pCurData->EAP_Point.vel.z = PrevP.M_info[6] + rel_T * (NextP.M_info[6] - PrevP.M_info[6]);
                        pCurData->EAP_Point.accel.x = PrevP.M_info[7] + rel_T * (NextP.M_info[7] - PrevP.M_info[7]);
                        pCurData->EAP_Point.accel.y = PrevP.M_info[8] + rel_T * (NextP.M_info[8] - PrevP.M_info[8]);
                        pCurData->EAP_Point.accel.z = PrevP.M_info[9] + rel_T * (NextP.M_info[9] - PrevP.M_info[9]);
                    }
                    else
                    {
                        bOK = false;
                    }
                }

                if (bOK)
                {
                    GLCalcTrajParam fCalc;
                    pCurData->EAP_parameters.H = fCalc.CalcHeight(pCurData->EAP_Point.coord);
                    pCurData->EAP_parameters.V = fCalc.CalcAbsVelocity(pCurData->EAP_Point.vel);
                    pCurData->EAP_parameters.theta = con_180_div_PI * fCalc.CalcAngleVelHorizont(pCurData->EAP_Point.coord, pCurData->EAP_Point.vel);

                    if (!pCurData->StartPoint.IsZero() && !pCurData->EAP_Point.coord.IsZero())
                    {
                        CGeocentric SPgc(pCurData->StartPoint.x, pCurData->StartPoint.y, pCurData->StartPoint.z),
                                EAPgc(pCurData->EAP_Point.coord.x, pCurData->EAP_Point.coord.y, pCurData->EAP_Point.coord.z);
                        CGeodesic SPgd, EAPgd;

                        bOK = GEOCENTRIC_GEO(&SPgc, &SPgd);
                        bOK = bOK && GEOCENTRIC_GEO(&EAPgc, &EAPgd);

                        if (bOK)
                        {
                            pCurData->EAP_parameters.L = SPgd.GetGeodesicLine(EAPgd);
                        }
                    }
                }
            }
        }
    }
}


qreal CAirBallist_Processing_M::GetMinActPathTimeByMark(qint32 indGT)
{
    qreal TimeAP = AIR_BALL::Timer_INI;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        CAirBall_Proc_InnerData_GT *pCurData = &m_pGlobalData->m_arProcData.arData_GT[indGT]; //pointer to the data for current GT

        qreal TimeMin = con_large_value; //minimum AP duration time value
        for (qint16 i = pCurData->BMMark.minBMMark; i<= pCurData->BMMark.maxBMMark; i++)
        {
            if (AIR_BALL::UNKNOWN_OF_SHORT_RANGE < i && i < AIR_BALL::UNKNOWN_OF_LONG_RANGE)
            {
                Str_Ar_EAP_Param_1Table *pEAP_Table = m_pGlobalData->m_pEAP_Tables->getCellMark(i); //pointer to the current EAP table
                if (pEAP_Table != nullptr)
                {
                    if (pEAP_Table->t_EAP > con_par_eps && pEAP_Table->t_EAP < TimeMin)
                    {
                        TimeMin = pEAP_Table->t_EAP;
                    }
                }
            }
        }

        if (TimeMin > con_par_eps && TimeMin - con_large_value<  -con_par_eps)
        {
            TimeAP = TimeMin;
        }
    }
    return TimeAP;
}


bool CAirBallist_Processing_M::CheckConditionsForPredict()
{
    bool bRes = true; //true for perform prolongation
    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;

    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        CAirBall_Proc_InnerData_GT *pCurData = &m_pGlobalData->m_arProcData.arData_GT[ind_GT];

        bool bActionPhase = false; //sign of action phase presence
        qint16 NumGT = m_pGlobalData->m_arProcData.arData_GT[ind_GT].NumbGT;
        if (0 < NumGT && NumGT < AIR_BALL::GT_FORM_AMOUNT)
        {
            if (m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].ActionPhase > NO_ACTION
                    && m_pGlobalData->m_arProcData.arData__NumbGT[NumGT].ActionPhase != CANCEL_ASSIGN)
            {
                bActionPhase = true;
            }
        }

        if (pCurData->Branch == AIR_BALL::DESCENDING_BRANCH)
        {
            CBallistProlong_InnerData* pProlData = &m_pGlobalData->m_BallProlong.m_arProlData[ind_ball]; //pointer to the array of prolongated data
            if (pProlData->SignLastProgn == AIR_BALL::NEW_PROGNOSIS)
            {
                if (!bActionPhase)
                {
                    if (pCurData->Hapogee > cHeightLowApogee)
                    {
                        if (m_TrPar.H < cHeightStopProlong_HiApogee)
                        {
                            bRes = false;
                        }
                    }
                    else
                    {
                        if (pCurData->Hapogee > con_par_eps
                                && m_TrPar.H < pCurData->Hapogee
                                && m_TrPar.H < cHeightStopProlong_LowApogee)
                        {
                            bRes = false;
                        }
                    }
                }
            }
        }

        if (!pCurData->VelPrev.IsZero() && !bActionPhase)
        {
            qreal AngleVel = m_pInData->Vel.getAngle(pCurData->VelPrev); //angle between current and previous velocity vectors
            if (fabs(AngleVel) > c_AngleVcurrVprev)
            {
                bRes = false;
            }
        }
    }
    return bRes;
}


bool CAirBallist_Processing_M::CheckPredictionDeviation()
{
    bool bDeviation = false;
    qint32 GTInd = m_pInData->IF_GT;
    qint32 BallInd = m_pInData->IF_BALL;
    if (0 <= GTInd && GTInd < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= BallInd && BallInd < AIR_BALL::BALL_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CAirBall_Proc_InnerData_GT *pGTData = &m_pGlobalData->m_arProcData.arData_GT[GTInd];
        CBallistProlong_InnerData *pProlData = &m_pGlobalData->m_BallProlong.m_arProlData[BallInd]; //pointer to the array of prolongated data

        if (fabs(pGTData->tLoc - pGTData->PrevTLoc) > con_eps)
        {
            GLPointDouble3D CoordPredict;
            bool bOK = pProlData->InterpolationProlongAtTime(pGTData->tLoc, CoordPredict);
            if (bOK)
            {
                qreal Dist = m_pInData->Coord.getDistance(CoordPredict);
                qreal RStrobe = sqrt(m_pInData->CovMatr.GetMaxDiag3());
                if (Dist > cCoefPredictionDeviation * AIR_BALL::K_Sigma * RStrobe)
                {
                    bDeviation = true;
                }
            }
            else
            {
                bDeviation = true;
            }
        }
    }
    return bDeviation;
}


bool CAirBallist_Processing_M::Estimate_Threat(qint32 indBall)
{
    bool bThreat = false;

    if (m_pInData->p_arCovObj != nullptr)
    {
        if (0 <= indBall && indBall < AIR_BALL::BALL_FORM_AMOUNT)
        {
            QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
            const CBallistProlong_InnerData* pProlData = &m_pGlobalData->m_BallProlong.m_arProlData[indBall]; //pointer to the array of prolongated data

            CGeocentric FPgc(pProlData->FallPoint.x, pProlData->FallPoint.y, pProlData->FallPoint.z); //fall point in ECEF
            CGeodesic FPgd; //fall point in the geodesic coordinate system
            GEOCENTRIC_GEO(&FPgc, &FPgd); //recalculation to the geodesic

            GLEllipse2D EllFP(pProlData->FallEll.bI, pProlData->FallEll.aI,
                              pProlData->FallEll.BetaI, 0., 0.); //ellipse in the format GLEllipse2D

            for (qint32 i = 0; i < m_pInData->p_arCovObj->N_obj; i++)
            {
                AIR_BALL::sCovObj *pObj = &m_pInData->p_arCovObj->arCovObj[i]; //pointer to the covered object
                CGeocentric ObjC_gc; //center of covered object in the geocentric coordinate system
                GEO_GEOCENTRIC(&pObj->CoordGeodez, &ObjC_gc); //recalculation to the geocentric

                CTopocentric ObjC_tp; //center of covered object in the topocentric CS with the center in fall point
                GEOCENTRIC_TOPO(&FPgd, &ObjC_gc, &ObjC_tp); //recalculation to the topocentric

                if (EllFP.IsPointInside(ObjC_tp.m_dZt, ObjC_tp.m_dXt) //center of covered object is inside the ellipse
                        || (EllFP.DistanceToPoint(ObjC_tp.m_dZt, ObjC_tp.m_dXt) < pObj->Radius) ) //or distance from center to the ellipse is less than radius
                {
                    bThreat = true;
                    break;
                }
            }
        }
    }

    return bThreat;
}


void CAirBallist_Processing_M::NewTrackProcessing_GT()
{
    const qint32 ind_GT = m_pInData->IF_GT;
    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
        if (!m_pGlobalData->m_arProcData.arData_GT[ind_GT].IsTaken)
        {
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].IsTaken = true;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].FirstPoint = m_pInData->Coord;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].tFirst = m_pInData->tLoc;
            m_pGlobalData->m_arProcData.arData_GT[ind_GT].RMSEs_FirstPoint.init(sqrt(m_pInData->CovMatr.M[0][0]),
                    sqrt(m_pInData->CovMatr.M[1][1]),
                    sqrt(m_pInData->CovMatr.M[2][2]));
        }
    }
}


void CAirBallist_Processing_M::NewTrackProcessing_ST()
{
    const qint32 ind_ST = m_pInData_ST->IF_ST;
    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

    if (!m_pGlobalData->m_arProcData.arData_ST[ind_ST].IsTaken)
    {
        m_pGlobalData->m_arProcData.arData_ST[ind_ST].IsTaken = true;
    }
}


void CAirBallist_Processing_M::ReturnFromPassToAct_Processing()
{
    const qint32 CurIndex = m_pInData->IF_GT;
    if (0 <= CurIndex && CurIndex < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CAirBall_Proc_InnerData_GT *pDataGT = &m_pGlobalData->m_arProcData.arData_GT[CurIndex];
        m_pGlobalData->m_BallSubclDet.ClearInnerData_Subcl_1Track(m_pInData->IF_BALL);
        m_pGlobalData->m_BallSubclDet.ClearInnerData_TType_1Track(m_pInData->IF_BALL);
        pDataGT->BMMark.Reset();
        pDataGT->TrajType = AIR_BALL::UNDEFINED_TRAJ_TYPE;

        m_pGlobalData->m_BallProlong.ClearInnerData_1Track(m_pInData->IF_BALL);
        pDataGT->tStart = AIR_BALL::Timer_INI;
        pDataGT->StartPoint.clear();
        pDataGT->StartEll.Reset();
        pDataGT->tFall = AIR_BALL::Timer_INI;
        pDataGT->FallPoint.clear();
        pDataGT->FallEll.Reset();
        pDataGT->D = 0;
        pDataGT->tApogee = AIR_BALL::Timer_INI;
        pDataGT->Hapogee = 0;

        pDataGT->tEAP = AIR_BALL::Timer_INI;
        pDataGT->T_EAP_finding_method = AIR_BALL::UNDEFINED_TEAP_METHOD;
        pDataGT->EAP_parameters.Reset();
        pDataGT->EAP_Point.clear();

        m_OutData_Ballist.bNewSubcl = true;
        m_OutData_Ballist.bNewTrajType = true;
        m_OutData_Ballist.bNewTEAP = false;
    }
}


void CAirBallist_Processing_M::UpdateCluster()
{
    qint32 indGT = m_pInData->IF_GT;
    qint32 ClstNum = m_pInData->ClstNum;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);

        CAirBall_Proc_InnerData_GT *pGTData = &m_pGlobalData->m_arProcData.arData_GT[indGT];
        qint32 PrevClstNum = pGTData->ClstNum; //previous cluster number

        if (ClstNum != PrevClstNum)
        {
            if (0 < PrevClstNum && PrevClstNum < AIR_BALL::CLST_FORM_AMOUNT)
            {
                //if previous cluster is not empty
                m_pGlobalData->ClstClearForGT(indGT, false);
            }

            pGTData->ClstNum = ClstNum;

            if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
//                    && PrevClstNum == 0)
            {
                //ClstNum is new for input GT
                CAirBall_Proc_ClstGTData *pInpClstData = &m_pGlobalData->m_arProcData.arData__Clst[ClstNum];

                if (pInpClstData->bBusy)
                {
                    std::set<qint32>::iterator it_search = pInpClstData->SetGTInd.find(indGT);
                    if (it_search == pInpClstData->SetGTInd.end()) //indGT is not found
                    {
                        pInpClstData->SetGTInd.insert(indGT);

                        m_pGlobalData->m_arProcData.DetermineParentInClst(ClstNum);
                        m_pGlobalData->m_arProcData.DetermineFstWHInClst(ClstNum);

                        bool bParent = m_pGlobalData->m_arProcData.GTIsParentInCluster(indGT);
                        bool bFirstWH = m_pGlobalData->m_arProcData.GTIsFirstWHInCluster(indGT);

                        if (bParent)
                        {
                            if (pInpClstData->ParentIndGT >= 0 && pInpClstData->ParentIndGT < AIR_BALL::GT_FORM_AMOUNT)
                            {
                                CAirBall_Proc_InnerData_GT *pNewParentGT = &m_pGlobalData->m_arProcData.arData_GT[pInpClstData->ParentIndGT];

                                if (fabs(pNewParentGT->tStart - AIR_BALL::Timer_INI) > con_eps
                                        && !pNewParentGT->StartPoint.IsZero()
                                        && (fabs(pInpClstData->tStart - AIR_BALL::Timer_INI) < con_eps
                                            || pInpClstData->StartPoint.IsZero()
                                            || pInpClstData->StartEll.IsZero()
                                            || pInpClstData->tStart > pNewParentGT->tStart))
                                {
                                    pInpClstData->tStart = pNewParentGT->tStart;
                                    pInpClstData->StartPoint = pNewParentGT->StartPoint;
                                    pInpClstData->StartEll = pNewParentGT->StartEll;
                                    pInpClstData->bParentDropped = false;
                                    std::vector<TracksPoints>().swap(pInpClstData->InitialArrPoints); //clear InitialArrPoints if start point is joint to the new parent track
                                }
                            }
                        }

                        if ((bParent && !bFirstWH) || (!bParent && bFirstWH))
                        {
                            m_pGlobalData->UpdateArrPointsForClst(ClstNum);
                        }
                    }
                }
                else
                {       //cluster is new
                    pInpClstData->Reset();
                    pInpClstData->bBusy = true;
                    pInpClstData->SetGTInd.insert(indGT);
                    m_pGlobalData->m_arProcData.DetermineParentInClst(ClstNum);
                    m_pGlobalData->m_arProcData.DetermineFstWHInClst(ClstNum);

                    if (pInpClstData->ParentIndGT >= 0 && pInpClstData->ParentIndGT < AIR_BALL::GT_FORM_AMOUNT)
                    {
                        CAirBall_Proc_InnerData_GT *pNewParentGT = &m_pGlobalData->m_arProcData.arData_GT[pInpClstData->ParentIndGT];
                        if (fabs(pNewParentGT->tStart - AIR_BALL::Timer_INI) > con_eps
                                && !pNewParentGT->StartPoint.IsZero()
                                && !pNewParentGT->StartEll.IsZero())
                        {
                            pInpClstData->tStart = pNewParentGT->tStart;
                            pInpClstData->StartPoint = pNewParentGT->StartPoint;
                            pInpClstData->StartEll = pNewParentGT->StartEll;
                        }
                    }
                }
            }
        }

        //check for first WH reclassification
        if (pGTData->ClstNum > 0 && pGTData->ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            CAirBall_Proc_ClstGTData *pClstData = &m_pGlobalData->m_arProcData.arData__Clst[pGTData->ClstNum];
            if (pClstData->FstWHInd >= 0 && pClstData->FstWHInd < AIR_BALL::GT_FORM_AMOUNT)
            {
                qint16 TypeCurr = m_pGlobalData->m_arProcData.arData_GT[pClstData->FstWHInd].BallType;
                if (TypeCurr != SUBCLASS_WARHEAD /*&& TypeCurr != TYPE_MaCRV && TypeCurr != TYPE_QBM && TypeCurr != TYPE_MaRV*/)
                {
                    m_pGlobalData->m_arProcData.DetermineFstWHInClst(pGTData->ClstNum);
                    m_pGlobalData->UpdateArrPointsForClst(pGTData->ClstNum);
                }
            }
        }
    }
}
void CAirBallist_Processing_M::SetNumbGT(const qint32 IndGT, const qint32 NumbGT)
{
    m_pGlobalData->SetNumbGT(IndGT, NumbGT);
}

void CAirBallist_Processing_M::SetActionPhase(const qint32 NumbGT, const qint16 ActionPhase, qreal CurTime)
{
    m_pGlobalData->SetActionPhase(NumbGT, ActionPhase, CurTime);
}

CAirBall_Proc_InnerData_GT CAirBallist_Processing_M::get_Proc_InnerData_GT(const qint32 ind_GT)
{
    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    return m_pGlobalData->m_arProcData.arData_GT[ind_GT];
}

void CAirBallist_Processing_M::set_Proc_InnerData_GT(const qint32 ind_GT, const CAirBall_Proc_InnerData_GT& data)
{
    QMutexLocker l_lock(&m_pGlobalData->m_mutexGD);
    m_pGlobalData->m_arProcData.arData_GT[ind_GT] = data;
}

