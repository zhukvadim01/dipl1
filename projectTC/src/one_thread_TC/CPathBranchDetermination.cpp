#include <fstream>
#include <iomanip>

#include "gl/GLFileLog.h"
#include "gl/GLCalcTrajParam.h"

#include "CPathBranchDetermination.h"
#include "air_ballist_literal.h"

// using namespace std;
using namespace GLFileLog;


CPathBranchDetermination::CPathBranchDetermination()
{
//#ifdef KEEP_LOG
//    AIR_CLS::out_info_path_branch << "Constructor: CPathBranchDetermination  " << this << std::endl;
//#endif //KEEP_LOG

    //Reset();
    m_arPathBrData_GT.resize(AIR_BALL::GT_FORM_AMOUNT);
    m_arPathBrData_ST.resize(AIR_BALL::ST_FORM_AMOUNT);
    try
    {
        m_pLogPBr = OpenLog("out_PathBranch", AIR_BALL::log_path);
    }
    catch(...)
    {
        DPS_ASSERT(false);
    }
}


CPathBranchDetermination::~CPathBranchDetermination()
{
    if (m_pLogPBr)
    {
        fclose(m_pLogPBr);
    }
    m_pLogPBr = nullptr;
//#ifdef KEEP_LOG
//    AIR_CLS::out_info_path_branch << "Destructor: CPathBranchDetermination  " << this << std::endl;
//#endif //KEEP_LOG
}


void CPathBranchDetermination::Reset()
{
    m_pInData_ST = nullptr;
    m_pInData_GT = nullptr;
    m_p_Arr_BallProc = nullptr;
    ClearInnerData_GT();
    ClearInnerData_ST();

    try
    {
        m_pLogPBr = OpenLog("out_PathBranch", AIR_BALL::log_path);
    }
    catch(...)
    {
        DPS_ASSERT(false);
    }

//#ifdef KEEP_LOG
//    AIR_CLS::out_info_path_branch << "Reset CPathBranchDetermination  " << this << std::endl;
//#endif //KEEP_LOG
}


void CPathBranchDetermination::FillInputData_ST(CPathBranch_InputData_ST &InData)
{
    m_pInData_ST = &InData;
}


void CPathBranchDetermination::FillInputData_GT(CPathBranch_InputData_GT &InData)
{
    m_pInData_GT = &InData;
}


void CPathBranchDetermination::InitPointerToBallProcArray(CAirBall_Proc_Inner_Arr *Arr)
{
    m_p_Arr_BallProc = Arr;
}


void CPathBranchDetermination::PathBranchDet_Realization_ST()
{
    const qint32 CurIndex = m_pInData_ST->IF_ST; //index of current general track

    m_OutData_PathBranch_ST.Reset();

    const qint16 PrevPath = m_arPathBrData_ST[CurIndex].P_Path;
    const qint16 PrevBranch = m_arPathBrData_ST[CurIndex].P_Branch;

    VhArrayFilling();

    if (m_arPathBrData_ST[CurIndex].P_Branch == AIR_BALL::DESCENDING_BRANCH)
    {
        Branch_Check_ST();
    }
    else
    {
        Branch_Determination_ST();
    }

    if (m_arPathBrData_ST[CurIndex].P_Branch == AIR_BALL::DESCENDING_BRANCH)
    {
        if ((m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::UNDEFINED_PATH)
                || (m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::ACTIVE_PATH))
        {
            m_arPathBrData_ST[CurIndex].P_Path = AIR_BALL::END_OF_ACTIVE_PATH;
            m_arPathBrData_ST[CurIndex].T_def_eap = m_pInData_ST->t_Loc;
            tLog(m_pLogPBr, " %f ST #%d Passive path by descending branch", m_pInData_ST->t_Loc, CurIndex);
        }
        else if (m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::END_OF_ACTIVE_PATH)
        {
            m_arPathBrData_ST[CurIndex].P_Path = AIR_BALL::PASSIVE_PATH;
            tLog(m_pLogPBr, " %f ST #%d Passive path by descending branch", m_pInData_ST->t_Loc, CurIndex);
        }
        else
        {
        }
    }
    else //ascending or undefined branch
    {
        if (!m_pInData_ST->bAeroballistic/*m_pInData_ST->Type != TYPE_QBM && m_pInData_ST->Type != TYPE_MaRV*/
                /*&& m_pInData_ST->TypeGT != TYPE_QBM && m_pInData_ST->TypeGT != TYPE_MaRV*/)
        {
            Path_Determination_ST();
        }
        else
        {       //for aeroballistic objects only
            if (m_arPathBrData_ST[CurIndex].P_Path != AIR_BALL::PASSIVE_PATH)
            {
                m_arPathBrData_ST[CurIndex].P_Path = AIR_BALL::PASSIVE_PATH;
                m_OutData_PathBranch_ST.P_Save_prev_PathBranchData = false;
                m_OutData_PathBranch_ST.P_Path = AIR_BALL::PASSIVE_PATH;
                tLog(m_pLogPBr, " %f ST #%d Passive path by aeroballistic", m_pInData_ST->t_Loc, CurIndex);
            }
        }

    }

    if (m_arPathBrData_ST[CurIndex].P_Path == PrevPath
            && m_arPathBrData_ST[CurIndex].P_Branch == PrevBranch)
    {
        m_OutData_PathBranch_ST.P_Save_prev_PathBranchData = true;
    }
    else //path or branch has changed
    {
        m_OutData_PathBranch_ST.P_Save_prev_PathBranchData = false;

        m_OutData_PathBranch_ST.P_Path = m_arPathBrData_ST[CurIndex].P_Path;
        m_OutData_PathBranch_ST.P_Branch = m_arPathBrData_ST[CurIndex].P_Branch;
    }

    tLog(m_pLogPBr, " %f ST #%d Path %d Branch %d", m_pInData_ST->t_Loc, CurIndex,
         m_arPathBrData_ST[CurIndex].P_Path, m_arPathBrData_ST[CurIndex].P_Branch);
}


void CPathBranchDetermination::PathBranchDet_Realization_GT()
{
    m_OutData_PathBranch_GT.Reset();
    const qint32 indGT = m_pInData_GT->IF_GT;

    qint16 MaxPathValue = -1;
    qint16 MaxBranchValue = -1;
    std::set<qint32>::const_iterator it;
    for (it = m_pInData_GT->SetST.begin(); it != m_pInData_GT->SetST.end(); it++)
    {
        const qint32 ind_ST = (*it);
        if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
        {
            if (m_arPathBrData_ST[ind_ST].currentAhArraySize > 0)
            {
                if (m_arPathBrData_ST[ind_ST].P_Path > MaxPathValue)
                {
                    MaxPathValue = m_arPathBrData_ST[ind_ST].P_Path;
                }
                if (m_arPathBrData_ST[ind_ST].P_Branch > MaxBranchValue)
                {
                    MaxBranchValue = m_arPathBrData_ST[ind_ST].P_Branch;
                }
            }
        }
    }

    if (MaxBranchValue >= 0 && MaxBranchValue != m_arPathBrData_GT[indGT].P_Branch)
    {
        m_OutData_PathBranch_GT.bPathBranchChanged = true;        
        m_arPathBrData_GT[indGT].P_Branch = MaxBranchValue;
    }

    if (!m_p_Arr_BallProc->arData_GT.at(indGT).bAeroballistic/*m_pInData_GT->TypeGT != TYPE_QBM && m_pInData_GT->TypeGT != TYPE_MaRV*/)
    {
        Path_Fusion_GT();

        if (m_arPathBrData_GT[indGT].P_Path == AIR_BALL::END_OF_ACTIVE_PATH)
        {
            m_arPathBrData_GT[indGT].T_def_eap = m_pInData_GT->tLoc;
            if (m_p_Arr_BallProc->arData_GT[indGT].Path == AIR_BALL::ACTIVE_PATH)
            {
                m_arPathBrData_GT[indGT].T_eap = m_pInData_GT->tLoc;
            }
        }
        else
        {
            if (m_arPathBrData_GT[indGT].P_Path == AIR_BALL::ACTIVE_PATH)
            {
                m_arPathBrData_GT[indGT].bActObserved = true;
            }
        }
    }
    else
    {       //for aeroballistic objects only
        if (m_arPathBrData_GT[indGT].P_Path != AIR_BALL::PASSIVE_PATH)
        {
            m_arPathBrData_GT[indGT].P_Path = AIR_BALL::PASSIVE_PATH;
            tLog(m_pLogPBr, " %f GT #%d Passive path by aeroballistic", m_pInData_GT->tLoc, indGT);
            m_OutData_PathBranch_GT.bPathBranchChanged = true;
        }
    }


    tLog(m_pLogPBr, " %f GT #%d Path %d Branch %d", m_pInData_GT->tLoc, indGT,
         m_arPathBrData_GT[indGT].P_Path, m_arPathBrData_GT[indGT].P_Branch);
}

void CPathBranchDetermination::Path_Fusion_GT()
{
//    m_OutData_PathBranch_GT.Reset();
    const qint32 indGT = m_pInData_GT->IF_GT;
    qreal meanOverSoursesRMSE = 0, meanOverSoursesRMSE_SmallPer = 0;
    qreal meanSrcPeriod = 0;
//    const qreal EAP_FUSION_COEFFICIENT = 2;
    qint32 numberOfSourses = 0, numberOfSources_SmallPer=0;
    qint32 numberSrcCalcPeriod = 0;
    bool allVoteForPassive = true;
    bool allVoteForActive = true;

    std::set<qint32>::iterator it;
    for (it = m_pInData_GT->SetST.begin(); it != m_pInData_GT->SetST.end(); it++)
    {
        const qint32 ind_ST = (*it);
        if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
        {
            if (m_arPathBrData_ST[ind_ST].currentAhArraySize > 0)
            {
                meanOverSoursesRMSE += m_p_Arr_BallProc->arData_ST[ind_ST].RMSE_mean;
                numberOfSourses++;

                if (m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod > con_par_eps)
                {
                    meanSrcPeriod += m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod;
                    numberSrcCalcPeriod ++;
                }
            }
        }
    }
    meanOverSoursesRMSE /= numberOfSourses;
    if (numberSrcCalcPeriod > 0)
    {
        meanSrcPeriod /= numberSrcCalcPeriod;

        for (it = m_pInData_GT->SetST.begin(); it != m_pInData_GT->SetST.end(); it++)
        {
            const qint32 ind_ST = (*it);
            if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
            {
                if (m_arPathBrData_ST[ind_ST].currentAhArraySize > 0
                        && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod > con_par_eps
                        && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod < EAP_FUSION_COEFFICIENT * meanSrcPeriod)
                {
                    meanOverSoursesRMSE_SmallPer += m_p_Arr_BallProc->arData_ST[ind_ST].RMSE_mean;
                    numberOfSources_SmallPer ++;
                }
            }
        }

        meanOverSoursesRMSE_SmallPer /= numberOfSources_SmallPer;
    }
    else
    {
        meanSrcPeriod = c_step;
    }

    for (it = m_pInData_GT->SetST.begin(); it != m_pInData_GT->SetST.end(); it++)
    {
        const qint32 ind_ST = (*it);
        if ((0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
                && m_arPathBrData_ST[ind_ST].currentAhArraySize > 0
                && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod > con_par_eps
                && m_p_Arr_BallProc->arData_ST[ind_ST].RMSE_mean < EAP_FUSION_COEFFICIENT * meanOverSoursesRMSE
                && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod < EAP_FUSION_COEFFICIENT * meanSrcPeriod)
        {
            allVoteForPassive = allVoteForPassive && ((m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::PASSIVE_PATH) ||
                                  (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::END_OF_ACTIVE_PATH) ||
                                  (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::UNDEFINED_PATH));
            allVoteForActive = allVoteForActive && ((m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::ACTIVE_PATH) ||
                                 (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::UNDEFINED_PATH));
        }
    }

    if (!allVoteForActive && !allVoteForPassive)
    {
        allVoteForActive = true;
        allVoteForPassive = true;

        for (it = m_pInData_GT->SetST.begin(); it != m_pInData_GT->SetST.end(); it++)
        {
            const qint32 ind_ST = (*it);
            if ((0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
                    && m_arPathBrData_ST[ind_ST].currentAhArraySize > 0
                    && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod > con_par_eps
                    && m_p_Arr_BallProc->arData_ST[ind_ST].tLoc > m_pInData_GT->tPrevPathDet - meanSrcPeriod - con_eps
                    && m_p_Arr_BallProc->arData_ST[ind_ST].RMSE_mean < EAP_FUSION_COEFFICIENT * meanOverSoursesRMSE
                    && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod < EAP_FUSION_COEFFICIENT * meanSrcPeriod)
            {
                allVoteForPassive = allVoteForPassive && ((m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::PASSIVE_PATH) ||
                                      (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::END_OF_ACTIVE_PATH) ||
                                      (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::UNDEFINED_PATH));
                allVoteForActive = allVoteForActive && ((m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::ACTIVE_PATH) ||
                                     (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::UNDEFINED_PATH));
            }
        }
    }

    if (allVoteForPassive && allVoteForActive)
    {
        for (it = m_pInData_GT->SetST.begin(); it != m_pInData_GT->SetST.end(); it++)
        {
            const qint32 ind_ST = (*it);
            if ((0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
                    && m_arPathBrData_ST[ind_ST].currentAhArraySize > 0
                    && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod > con_par_eps
                    && m_p_Arr_BallProc->arData_ST[ind_ST].tLoc > m_pInData_GT->tPrevPathDet - meanSrcPeriod - con_eps
                    && m_p_Arr_BallProc->arData_ST[ind_ST].RMSE_mean < EAP_FUSION_COEFFICIENT * meanOverSoursesRMSE_SmallPer
                    && m_p_Arr_BallProc->arData_ST[ind_ST].MeanPeriod < EAP_FUSION_COEFFICIENT * meanSrcPeriod)
            {
                allVoteForPassive = allVoteForPassive && ((m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::PASSIVE_PATH) ||
                                      (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::END_OF_ACTIVE_PATH) ||
                                      (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::UNDEFINED_PATH));
                allVoteForActive = allVoteForActive && ((m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::ACTIVE_PATH) ||
                                     (m_arPathBrData_ST[ind_ST].P_Path == AIR_BALL::UNDEFINED_PATH));
            }
        }

        if (allVoteForPassive && allVoteForActive)
        {
            return;
        }
    }

    if ((allVoteForPassive) && (m_arPathBrData_GT[indGT].P_Path != AIR_BALL::PASSIVE_PATH))
    {
        if((m_arPathBrData_GT[indGT].P_Path == AIR_BALL::ACTIVE_PATH) ||
                (m_arPathBrData_GT[indGT].P_Path == AIR_BALL::UNDEFINED_PATH))
        {
            m_arPathBrData_GT[indGT].P_Path = AIR_BALL::END_OF_ACTIVE_PATH;
            tLog(m_pLogPBr, " %f GT #%d End of active path", m_pInData_GT->tLoc, indGT);
            m_OutData_PathBranch_GT.bPathBranchChanged = true;
            m_arPathBrData_GT[indGT].T_def_eap = m_pInData_GT->tLoc;
            if (m_p_Arr_BallProc->arData_GT[indGT].Path == AIR_BALL::ACTIVE_PATH)
            {
                m_arPathBrData_GT[indGT].T_eap = m_pInData_GT->tLoc;
            }
        }
        else
        {
            m_arPathBrData_GT[indGT].P_Path = AIR_BALL::PASSIVE_PATH;
            m_OutData_PathBranch_GT.bPathBranchChanged = true;
        }
    }

    if ((allVoteForActive) && (m_arPathBrData_GT[indGT].P_Path != AIR_BALL::ACTIVE_PATH))
    {
        m_arPathBrData_GT[indGT].P_Path = AIR_BALL::ACTIVE_PATH;
        m_OutData_PathBranch_GT.bPathBranchChanged = true;
    }
}

void CPathBranchDetermination::Branch_Determination_ST()
{
    const qint32 CurIndex = m_pInData_ST->IF_ST;
    qint32 CurrBranch = AIR_BALL::UNDEFINED_BRANCH;
    Branch_Det_forCurrentUpd_ST(CurrBranch);
    m_arPathBrData_ST[CurIndex].P_Branch = CurrBranch;
}


void CPathBranchDetermination::Path_Determination_ST()
{
     const qint32 CurIndex = m_pInData_ST->IF_ST;
     qint32 CurrPath = AIR_BALL::UNDEFINED_PATH; //value of path for current update
     Path_Det_forCurrentUpd_ST(CurrPath);

     if (CurrPath == AIR_BALL::ACTIVE_PATH && m_pInData_ST->pInDataST != nullptr)
     {
         GLCalcTrajParam TrajCalc;
         qreal ALong = TrajCalc.CalcLongitudinalAccel(m_pInData_ST->pInDataST->Vel, m_pInData_ST->pInDataST->Acc);
         qreal SigmaALong = TrajCalc.CalcRMSE_LongitudinalAccel(m_pInData_ST->pInDataST->Vel, m_pInData_ST->pInDataST->Acc,
                                                                    m_pInData_ST->pInDataST->CovMatr);
         if (ALong > AIR_BALL::K_Sigma * SigmaALong)
         {
             //longitudinal accelecation = time derivative of absolute velocity is positive => absolute velocity increases
             m_arPathBrData_ST[CurIndex].T_act_prouved = m_pInData_ST->t_Loc;
         }
     }

//     if (CurrPath == AIR_BALL::UNDEFINED_PATH)
//     {
//         TryDetEAP_forGTusingST();
//     }

     if ((CurrPath == AIR_BALL::UNDEFINED_PATH)
             &&(m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::ACTIVE_PATH))
     {
         m_arPathBrData_ST[CurIndex].P_Path = AIR_BALL::ACTIVE_PATH;
         return;
     }

     if ((CurrPath == AIR_BALL::UNDEFINED_PATH)
             &&((m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::PASSIVE_PATH)
                 ||(m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::END_OF_ACTIVE_PATH)))
     {
         m_arPathBrData_ST[CurIndex].P_Path = AIR_BALL::PASSIVE_PATH;
         return;
     }

     if ((CurrPath == AIR_BALL::PASSIVE_PATH)
             &&((m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::ACTIVE_PATH)
                 ||(m_arPathBrData_ST[CurIndex].P_Path == AIR_BALL::UNDEFINED_PATH)))
     {
         m_arPathBrData_ST[CurIndex].P_Path = AIR_BALL::END_OF_ACTIVE_PATH;
         m_arPathBrData_ST[CurIndex].T_def_eap = m_pInData_ST->t_Loc;
         return;
     }

     m_arPathBrData_ST[CurIndex].P_Path = CurrPath;

}


void CPathBranchDetermination::Branch_Det_forCurrentUpd_ST(qint32 &CurrBranch)
{
    const qint32 CurIndex = m_pInData_ST->IF_ST;

    qint32 Curr_Vh_Ind = m_arPathBrData_ST[CurIndex].currentVhIndex -1;
    if (Curr_Vh_Ind == -1)
    {
        Curr_Vh_Ind = DEPTH_VH_MAX - 1;
    }
    qint32 depth_Vh_current;
    qint32 ind;
    qreal sigmaVh = 0;

    if( m_arPathBrData_ST[CurIndex].currentVhArraySize < DEPTH_VH)
    {
        CurrBranch = AIR_BALL::UNDEFINED_BRANCH;
        return;
    }

    for(ind = 0; ind < DEPTH_VH; ind++)
    {
        sigmaVh += m_arPathBrData_ST[CurIndex].arr_SigVh[(Curr_Vh_Ind - ind + DEPTH_VH_MAX)%DEPTH_VH_MAX];
    }
    sigmaVh = sigmaVh / DEPTH_VH;

    if (sigmaVh / DEPTH_VH < MAX_ALLOWABLE_VH_RMSE)
    {
        depth_Vh_current = DEPTH_VH;
    }
    else
    {
        if(sigmaVh / DEPTH_VH_MAX > MAX_ALLOWABLE_VH_RMSE)
        {
            CurrBranch = AIR_BALL::UNDEFINED_BRANCH;
            return;
        }
        else
        {
            depth_Vh_current = DEPTH_VH;
            while ((depth_Vh_current < DEPTH_VH_MAX) && (sigmaVh / depth_Vh_current > MAX_ALLOWABLE_VH_RMSE)
                   && (depth_Vh_current < m_arPathBrData_ST[CurIndex].currentVhArraySize))
            {
                depth_Vh_current++;
                sigmaVh = (sigmaVh * (depth_Vh_current - 1)  + m_arPathBrData_ST[CurIndex].arr_SigVh[(Curr_Vh_Ind - ind + DEPTH_VH_MAX)%DEPTH_VH_MAX])/ depth_Vh_current;
                ind++;
            }

            if(sigmaVh / depth_Vh_current > MAX_ALLOWABLE_VH_RMSE)
            {
                CurrBranch = AIR_BALL::UNDEFINED_BRANCH;
                return;
            }
        }
    }


    qreal Vh = 0;

    for(qint32 i = 0; i < depth_Vh_current; i++)
    {
        Vh += m_arPathBrData_ST[CurIndex].arr_Vh[(Curr_Vh_Ind - i + DEPTH_VH_MAX)%DEPTH_VH_MAX];
    }
    Vh = Vh / depth_Vh_current;

    const qreal criterion_Value = depth_Vh_current * pow(Vh,2.) / pow(sigmaVh,2.);


    if( criterion_Value > BOUND_VH_ACCEPT )
    {
        if( Vh > 0)
        {
            CurrBranch = AIR_BALL::ASCENDING_BRANCH;
        }
        else
        {
            CurrBranch =AIR_BALL::DESCENDING_BRANCH;
        }
    }
    else
    {
        CurrBranch = AIR_BALL::TRANSITION_BRANCHES;
    }
}


void CPathBranchDetermination::Path_Det_forCurrentUpd_ST(qint32 &CurrPath)
{
    const qint32 CurIndex = m_pInData_ST->IF_ST;

    qint16 Path_VH = EstimatePathByVertVel(); //preliminary estimated path, using vertical velocity

    if( m_arPathBrData_ST[CurIndex].currentFilterArraySize >= MIN_OVERWEIGHT_POINT_NUMBER)
    {
        const qreal sigma_h_for_filter = sqrt(pow(SIGMA_RANGE,2.) + pow(m_pInData_ST->R_rdr_obj,2.) * pow(SIGMA_EPSILON,2.));

        if(sigma_h_for_filter < MAX_ALLOWABLE_H_RMSE_FOR_FILTER)
        {
            qint32 Curr_filter_Ind = m_arPathBrData_ST[CurIndex].Curr_filter_Ind -1;
            if (Curr_filter_Ind == -1)
            {
                Curr_filter_Ind = MIN_OVERWEIGHT_POINT_NUMBER - 1;
            }

            bool flag_filter = true;

            for(qint32 i = 0; i < MIN_OVERWEIGHT_POINT_NUMBER; i++)
            {
                flag_filter = flag_filter &&
                        (m_arPathBrData_ST[CurIndex].arr_Prob_IMM_large_coeff[(Curr_filter_Ind - i + MIN_OVERWEIGHT_POINT_NUMBER) % MIN_OVERWEIGHT_POINT_NUMBER]
                         > m_arPathBrData_ST[CurIndex].arr_Prob_IMM_small_coeff[(Curr_filter_Ind - i + MIN_OVERWEIGHT_POINT_NUMBER) % MIN_OVERWEIGHT_POINT_NUMBER]);
            }
            if(flag_filter)
            {
                CurrPath = AIR_BALL::PASSIVE_PATH;
                tLog(m_pLogPBr, " %f ST #%d Passive path by the filter", m_pInData_ST->t_Loc, CurIndex);
                return;
            }
        }
    }

    if( m_arPathBrData_ST[CurIndex].currentAhArraySize < DEPTH_AH)
    {
        CurrPath = AIR_BALL::UNDEFINED_PATH;
        return;
    }

    qint32 Curr_Ah_Ind = m_arPathBrData_ST[CurIndex].currentAhIndex -1;
    if (Curr_Ah_Ind == -1)
    {
        Curr_Ah_Ind = DEPTH_AH_MAX - 1;
    }


    qint32 depth_Ah_current;
    qint32 ind;

    qreal sigmaAh = 0;

    for(ind = 0; ind < DEPTH_AH; ind++)
    {
        sigmaAh += m_arPathBrData_ST[CurIndex].arr_SigAh[(Curr_Ah_Ind - ind + DEPTH_AH_MAX)%DEPTH_AH_MAX];
    }
    sigmaAh = sigmaAh / DEPTH_AH;

    if (sigmaAh / DEPTH_AH < MAX_ALLOWABLE_AH_RMSE)
    {
        depth_Ah_current = DEPTH_AH;
    }
    else
    {
        if(sigmaAh / DEPTH_AH_MAX > MAX_ALLOWABLE_AH_RMSE)
        {
            CurrPath = AIR_BALL::UNDEFINED_PATH;
            return;
        }
        else
        {
            depth_Ah_current = DEPTH_AH ;
            while ((depth_Ah_current < DEPTH_AH_MAX) && (sigmaAh / depth_Ah_current > MAX_ALLOWABLE_AH_RMSE)
                   && (depth_Ah_current < m_arPathBrData_ST[CurIndex].currentAhArraySize))
            {
                depth_Ah_current++;
                sigmaAh = (sigmaAh * (depth_Ah_current - 1)  + m_arPathBrData_ST[CurIndex].arr_SigAh[(Curr_Ah_Ind - ind + DEPTH_AH_MAX)%DEPTH_AH_MAX])/ depth_Ah_current;
                ind++;
            }

            if (sigmaAh / depth_Ah_current > MAX_ALLOWABLE_AH_RMSE)
            {
                CurrPath = AIR_BALL::UNDEFINED_PATH;
                return;
            }
        }
    }

    qreal Ah = 0;

    const qreal g_h = con_g * pow(con_Eath_middle_radius / ( con_Eath_middle_radius + m_pInData_ST->TrParameters.H), 2.);

    for(qint32 i = 0; i < depth_Ah_current; i++)
    {
        Ah += m_arPathBrData_ST[CurIndex].arr_Ah[(Curr_Ah_Ind - i + DEPTH_AH_MAX)%DEPTH_AH_MAX];
    }
    Ah = Ah / depth_Ah_current;

    if (Ah < Ah_FOR_PASSIVE_PATH_ACCEPT
            && m_pInData_ST->TrParameters.H < H_ATMOSPHERE)
    {
        CurrPath = AIR_BALL::PASSIVE_PATH;
        tLog(m_pLogPBr, " %f ST #%d Passive path by the threshold", m_pInData_ST->t_Loc, CurIndex);
        return;
    }

    //const qreal criterion_Value = depth_Ah_current * pow(Ah + g_h,2.) / pow(sigmaAh,2.);
    const qreal criterion_Value = sqr(Ah + g_h) / sqr(AIR_BALL::K_Sigma * sigmaAh);

    if( criterion_Value > BOUND_AH_EAP_ACCEPT
            || m_pInData_ST->TrParameters.a > g_h + cCoef_a_g_sigA * AIR_BALL::K_Sigma * m_pInData_ST->TrParameters.SigA)
    {
        bool bRedetermination = true; //sign of path redetermination

        if (fabs(m_arPathBrData_ST[CurIndex].T_def_eap - AIR_BALL::Timer_INI) > con_eps
                && m_pInData_ST->t_Loc - m_arPathBrData_ST[CurIndex].T_def_eap > DT_PATH_REDETERMINATION_CONTROL)
        {
            bRedetermination = false;
            m_arPathBrData_ST[CurIndex].counter_active_redetermination ++;
            if (fabs(m_arPathBrData_ST[CurIndex].t0_active_redetermination - AIR_BALL::Timer_INI) < con_eps)
            {
                m_arPathBrData_ST[CurIndex].t0_active_redetermination = m_pInData_ST->t_Loc;
            }
            else
            {
                if (m_pInData_ST->t_Loc - m_arPathBrData_ST[CurIndex].t0_active_redetermination > DT_CTRL_PATH_REDETERM
                        && m_arPathBrData_ST[CurIndex].counter_active_redetermination > N_CTRL_PATH_REDETERM)
                {
                    bRedetermination = true;
                }
            }
        }

        if (bRedetermination)
        {
            if( criterion_Value > BOUND_Ah_ACT_PATH_ACCEPT)
            {
                CurrPath = AIR_BALL::ACTIVE_PATH;
            }
            else
            {
                CurrPath = AIR_BALL::UNDEFINED_PATH;
            }
        }
    }
    else
    {
        if (Path_VH == AIR_BALL::END_OF_ACTIVE_PATH
                || Path_VH == AIR_BALL::PASSIVE_PATH)
        {
            CurrPath = AIR_BALL::PASSIVE_PATH;
            tLog(m_pLogPBr, " %f ST #%d Passive path by the criterion", m_pInData_ST->t_Loc, CurIndex);

            m_arPathBrData_ST[CurIndex].t0_active_redetermination = AIR_BALL::Timer_INI;
            m_arPathBrData_ST[CurIndex].counter_active_redetermination = 0;
        }
    }
}


qint16 CPathBranchDetermination::EstimatePathByVertVel()
{
    qint16 Path = AIR_BALL::UNDEFINED_PATH;
    const qint32 CurIndex = m_pInData_ST->IF_ST;
    CPathBranch_InnerData_ST *pDataST = &m_arPathBrData_ST[CurIndex];
    if (pDataST->currentVhArraySize > 1)
    {
        qint32 Curr_Vh_Ind = pDataST->currentVhIndex -1;
        if (Curr_Vh_Ind == -1)
        {
            Curr_Vh_Ind = DEPTH_VH_MAX - 1;
        }

        qint32 Prev_Vh_Ind = Curr_Vh_Ind - 1;
        qreal dt = 0;
        bool bExit = false; //sign of the exit from "while" cycle
        bool bIndexFound = false; //true if the index of compared element in the arrays of Vh is found
        while (!bExit)
        {
            if (Prev_Vh_Ind < 0 && pDataST->currentVhArraySize >= DEPTH_VH_MAX)
            {
                Prev_Vh_Ind = DEPTH_VH_MAX -1;
            }

            if (Prev_Vh_Ind >= 0)
            {
                dt = pDataST->arr_TVh[Curr_Vh_Ind] - pDataST->arr_TVh[Prev_Vh_Ind];
                qreal SigVMean = sqrt(0.5*(sqr(pDataST->arr_SigVh[Curr_Vh_Ind]) + sqr(pDataST->arr_SigVh[Prev_Vh_Ind])));
                if (dt >= SigVMean/cK_VT && dt > con_eps)
                {
                    bIndexFound = true;
                    bExit = true;
                }
                else
                {
                    Prev_Vh_Ind --;
                    if ((Curr_Vh_Ind - Prev_Vh_Ind)%DEPTH_VH_MAX == 0)
                    {
                        bExit = true;
                    }
                }
            }
            else
            {
                bExit = true;
            }
        }

        if (bIndexFound && dt > con_eps)
        {
            pDataST->NumAhExpSm ++;
            qreal AhCurr = (pDataST->arr_Vh[Curr_Vh_Ind] - pDataST->arr_Vh[Prev_Vh_Ind]) / dt;
            if (pDataST->NumAhExpSm == 1)
            {
                pDataST->AhExpSm = AhCurr;
                //pDataST->SigAhExpSm = sqrt(sqr(pDataST->arr_SigVh[Curr_Vh_Ind])
                //                           + sqr(pDataST->arr_SigVh[Prev_Vh_Ind])) / dt;
            }
            else
            {
                pDataST->AhExpSm = (1.-cEAP_Ksi)*AhCurr + cEAP_Ksi*pDataST->AhExpSm;
                if (pDataST->NumAhExpSm >= cEAP_m0)
                {
                    qreal Thr=0;
                    if (pDataST->NumAhExpSm == cEAP_m0)
                    {
                        Thr = cEAP_P0;
                    }
                    else if (pDataST->NumAhExpSm < cEAP_m0 + 8)
                    {
                        qreal SigmaA = 2. * cK_VT
                                * sqrt((pow(cEAP_Ksi, 2.*static_cast<qreal>(pDataST->NumAhExpSm) - 2.)*(3.*cEAP_Ksi-1.)
                                        + sqr(cEAP_Ksi) - 2.*cEAP_Ksi + 1) / (cEAP_Ksi + 1.));
                        Thr = (cEAP_P1 - cEAP_P0)*(SigmaA-SigmaA0)/(SigmaA1-SigmaA0) + cEAP_P0;
                    }
                    else
                    {
                        Thr = cEAP_P1;
                    }

                    if (pDataST->AhExpSm < Thr)
                    {
                        if (pDataST->P_Path == AIR_BALL::UNDEFINED_PATH
                                || pDataST->P_Path == AIR_BALL::ACTIVE_PATH)
                        {
                            Path = AIR_BALL::END_OF_ACTIVE_PATH;
                        }
                        else if (pDataST->P_Path == AIR_BALL::END_OF_ACTIVE_PATH
                                 || pDataST->P_Path == AIR_BALL::PASSIVE_PATH)
                        {
                            Path = AIR_BALL::PASSIVE_PATH;
                        }
                        else
                        {
                            ;
                        }
                    }
                    else
                    {
                        if (pDataST->P_Path == AIR_BALL::ACTIVE_PATH)
                        {
                            Path = AIR_BALL::ACTIVE_PATH;
                        }
                    }
                }
            }
        }
    }
    return Path;
}


void CPathBranchDetermination::Branch_Check_ST()
{
    const qint32 CurIndex = m_pInData_ST->IF_ST;
    qint32 CurrBranch = AIR_BALL::UNDEFINED_BRANCH;
    Branch_Det_forCurrentUpd_ST(CurrBranch);

    if ((CurrBranch != AIR_BALL::UNDEFINED_BRANCH)
          && (CurrBranch != m_arPathBrData_ST[CurIndex].P_Branch))
    {
        if (CurrBranch == m_arPathBrData_ST[CurIndex].Branch_saved_for_check)
        {
            if (fabs(m_arPathBrData_ST[CurIndex].t_begin_check_branch - AIR_BALL::Timer_INI) < con_par_eps)
            {
                m_arPathBrData_ST[CurIndex].t_begin_check_branch = m_pInData_ST->t_Loc;
            }

            m_arPathBrData_ST[CurIndex].counter_check_branch ++;

            if (m_pInData_ST->t_Loc - m_arPathBrData_ST[CurIndex].t_begin_check_branch > PERIOD_BRANCH_CHECK_CONFIRM
                    && m_arPathBrData_ST[CurIndex].counter_check_branch >= NUMB_BRANCH_CHECK_CONFIRM)
            {
                m_arPathBrData_ST[CurIndex].P_Branch = CurrBranch;
            }
        }
        else
        {
            m_arPathBrData_ST[CurIndex].counter_check_branch = 0;
            m_arPathBrData_ST[CurIndex].t_begin_check_branch = AIR_BALL::Timer_INI;
        }
    }
    else
    {
        m_arPathBrData_ST[CurIndex].counter_check_branch = 0;
        m_arPathBrData_ST[CurIndex].t_begin_check_branch = AIR_BALL::Timer_INI;
    }

    m_arPathBrData_ST[CurIndex].Branch_saved_for_check = CurrBranch;
}


//void CPathBranchDetermination::EAP_Det_VDoppl_ST()
//{
//    ;
//}


void CPathBranchDetermination::VhArrayFilling()
{
    const qint32 CurIndex = m_pInData_ST->IF_ST;
    const qint32 Curr_Vh_Ind = m_arPathBrData_ST[CurIndex].currentVhIndex;
    const qint32 Curr_Ah_Ind = m_arPathBrData_ST[CurIndex].currentAhIndex;
    const qint32 Curr_filter_Ind = m_arPathBrData_ST[CurIndex].Curr_filter_Ind;

    m_arPathBrData_ST[CurIndex].arr_TVh[Curr_Vh_Ind] = m_pInData_ST->t_Loc;
    m_arPathBrData_ST[CurIndex].arr_Vh[Curr_Vh_Ind] = m_pInData_ST->TrParameters.Vh;
    m_arPathBrData_ST[CurIndex].arr_SigVh[Curr_Vh_Ind] = m_pInData_ST->TrParameters.SigVh;

    m_arPathBrData_ST[CurIndex].arr_Ah[Curr_Ah_Ind] = m_pInData_ST->TrParameters.ah_noC; //ah;
    m_arPathBrData_ST[CurIndex].arr_SigAh[Curr_Ah_Ind] = m_pInData_ST->TrParameters.SigAh + CORR_RMSE_AH;

    m_arPathBrData_ST[CurIndex].arr_Prob_IMM_small_coeff[Curr_filter_Ind] = m_pInData_ST->Prob_IMM_small_coeff;
    m_arPathBrData_ST[CurIndex].arr_Prob_IMM_large_coeff[Curr_filter_Ind] = m_pInData_ST->Prob_IMM_large_coeff;


    m_arPathBrData_ST[CurIndex].currentVhIndex = (Curr_Vh_Ind + 1) % DEPTH_VH_MAX;
    m_arPathBrData_ST[CurIndex].currentAhIndex = (Curr_Ah_Ind + 1) % DEPTH_AH_MAX;
    m_arPathBrData_ST[CurIndex].Curr_filter_Ind = (Curr_filter_Ind + 1) % MIN_OVERWEIGHT_POINT_NUMBER;

    m_arPathBrData_ST[CurIndex].currentVhArraySize = m_arPathBrData_ST[CurIndex].currentVhArraySize + 1;
    m_arPathBrData_ST[CurIndex].currentAhArraySize = m_arPathBrData_ST[CurIndex].currentAhArraySize + 1;
    m_arPathBrData_ST[CurIndex].currentFilterArraySize = m_arPathBrData_ST[CurIndex].currentFilterArraySize +1;

}


void CPathBranchDetermination::DepthFindingInVhCalculating()
{
    ;
}


void CPathBranchDetermination::ClarifyEAP_Time()
{
    ;
}


void CPathBranchDetermination::Drop_GT(const qint32 ind_GT)
{
    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        ClearInnerData_GT_1Track(ind_GT);
    }
}


void CPathBranchDetermination::Drop_ST(const qint32 ind_ST)
{
    if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
    {
        ClearInnerData_ST_1Track(ind_ST);
    }
}


void CPathBranchDetermination::ClearInnerData_GT()
{
    for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++)
    {
        ClearInnerData_GT_1Track(i);
    }
}


void CPathBranchDetermination::ClearInnerData_ST()
{
    for (qint32 i=0; i<AIR_BALL::ST_FORM_AMOUNT; i++)
    {
        ClearInnerData_ST_1Track(i);
    }
}


void CPathBranchDetermination::ClearInnerData_GT_1Track(const qint32 index)
{
    if (0 <= index && index < AIR_BALL::GT_FORM_AMOUNT)
    {
        m_arPathBrData_GT[index].Reset();
    }
}


void CPathBranchDetermination::ClearInnerData_ST_1Track(const qint32 index)
{
    if (0 <= index && index < AIR_BALL::ST_FORM_AMOUNT)
    {
        m_arPathBrData_ST[index].Reset();
    }
}
