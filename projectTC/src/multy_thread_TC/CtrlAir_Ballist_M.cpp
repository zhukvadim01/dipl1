#include <fstream>
#include <iomanip>

#include <QCoreApplication>
#include <QDir>

#include "common/GDEnum.h"

#include "common/DefaultsConsts.h"

#include "gl/GLFileLog.h"
#include "gl/GLLogSpecForm.h"

#include "CtrlAir_Ballist_M.h"
#include "air_ballist_constants.h"

// using namespace std;
using namespace GLFileLog;

namespace AIR_BALL
{
//    const char* log_path = "../data/view/log";
//    const char* hhe_ini_file = "typeBM_HHe.ini";

#ifdef KEEP_LOG
//    static std::ofstream out_BallResult(qPrintable(QString("%1/BallistResult_M.txt").arg(log_path))); //logging result of classification
    static FILE* out_BallResult = nullptr;
#endif //KEEP_LOG

    static std::ifstream cells_ini; //stream for read values of cells for BM-Mark classification

//    qint32 SRC_AMOUNT        = MAX_ABONENTS /4;
//    qint32 ST_SRC_AMOUNT     = MAX_SRC_TRACKS;
//    qint32 ST_FORM_AMOUNT    = SRC_AMOUNT * ST_SRC_AMOUNT;

//    qint32 GT_FORM_AMOUNT    = MAX_GEN_TRACKS;
//    qint32 BALL_FORM_AMOUNT  = GT_FORM_AMOUNT;
}

CtrlAIR_BALLIST_M::CtrlAIR_BALLIST_M()
{
//#ifdef KEEP_LOG
//    AIR_BALL::out_info << "Constructor: CtrlAIR_BALLIST_M   " << this << std::endl;
//#endif //KEEP_LOG

    ID_Values_GT.resize(AIR_BALL::GT_FORM_AMOUNT+1);
    Ball_IDs_forGT.resize(AIR_BALL::GT_FORM_AMOUNT, nullptr);
    m_arDataCtrl.resize(AIR_BALL::GT_FORM_AMOUNT);

    ID_Values_ST.resize(AIR_BALL::ST_FORM_AMOUNT+1);
    ID_Values_Ball.resize(AIR_BALL::BALL_FORM_AMOUNT+1);
    GT_IDs.resize(AIR_BALL::GT_FORM_AMOUNT);
    m_arSrcInnEnumeration.resize(AIR_BALL::SRC_AMOUNT);

    ST_IDs.resize(AIR_BALL::SRC_AMOUNT);
    for(quint16 i = 0; i < ST_IDs.size(); ++i ) {
        ST_IDs[i].resize(AIR_BALL::ST_SRC_AMOUNT);
    }

//    m_ST_Data.resize(AIR_BALL::ST_FORM_AMOUNT+1);
//    m_BallInpData_ST.resize(AIR_BALL::ST_FORM_AMOUNT+1);

#ifdef KEEP_LOG
    AIR_BALL::out_BallResult = OpenLog("BallistResult_M", AIR_BALL::log_path);
#endif //KEEP_LOG

    Reset();
}


CtrlAIR_BALLIST_M::~CtrlAIR_BALLIST_M()
{
#ifdef KEEP_LOG
    if( AIR_BALL::out_BallResult ) {
        fclose(AIR_BALL::out_BallResult);
    }
#endif //KEEP_LOG

    if( m_pLogIDs ) {
        fclose(m_pLogIDs);
        m_pLogIDs = nullptr;
    }

    if( m_pLogProgn ) {
        fclose(m_pLogProgn);
        m_pLogProgn = nullptr;
    }

    if( m_pLogInp ) {
        fclose(m_pLogInp);
        m_pLogInp = nullptr;
    }

    if( m_pLogRDID ) {
        fclose(m_pLogRDID);
        m_pLogRDID = nullptr;
    }

    if( m_pLogTest ) {
        fclose(m_pLogTest);
        m_pLogTest = nullptr;
    }

    if( m_pLogOut ) {
        fclose(m_pLogOut);
        m_pLogOut = nullptr;
    }
}

void CtrlAIR_BALLIST_M::initConsts(quint16 SRC_amount, quint16 SRC_TRCK_amount, quint16 GT_amount)
{
    AIR_BALL::GT_FORM_AMOUNT        = GT_amount;

    AIR_BALL::SRC_AMOUNT            = SRC_amount;
    AIR_BALL::ST_SRC_AMOUNT         = SRC_TRCK_amount;
    AIR_BALL::ST_FORM_AMOUNT        = AIR_BALL::SRC_AMOUNT * AIR_BALL::ST_SRC_AMOUNT;

    AIR_BALL::GT_FORM_AMOUNT        = GT_amount;
    AIR_BALL::BALL_FORM_AMOUNT      = AIR_BALL::GT_FORM_AMOUNT;
}

void CtrlAIR_BALLIST_M::Reset()
{
    m_BO_quantity = 0;
    m_CurTime = -1;

    m_InpMsgsBuf.Reset();
    m_arCovObj.Reset();

    m_EAP_Tables.Reset();
    m_BallCoef_Tables.Reset();

    //fill arrays of identificators
    for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++)
    {
        ID_Values_GT[i].Index = i;
        ID_Values_GT[i].SignBusy = false;
    }
    ID_Values_GT[AIR_BALL::GT_FORM_AMOUNT].Index = -1;
    ID_Values_GT[AIR_BALL::GT_FORM_AMOUNT].SignBusy = false;

    for(qint32 i=0; i<AIR_BALL::ST_FORM_AMOUNT; i++)
    {
        ID_Values_ST[i].Index = i;
        ID_Values_ST[i].SignBusy = false;
    }
    ID_Values_ST[AIR_BALL::ST_FORM_AMOUNT].Index = -1;
    ID_Values_ST[AIR_BALL::ST_FORM_AMOUNT].SignBusy = false;

    for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
    {
        ID_Values_Ball[i].Index = i;
        ID_Values_Ball[i].SignBusy = false;
    }
    ID_Values_Ball[AIR_BALL::BALL_FORM_AMOUNT].Index = -1;
    ID_Values_Ball[AIR_BALL::BALL_FORM_AMOUNT].SignBusy = false;

    for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++) {
        GT_IDs[i] = nullptr;
    }

    m_NumGTEnumeration.Reset(AIR_BALL::GT_FORM_AMOUNT);

    for (qint32 i=0; i<AIR_BALL::SRC_AMOUNT; i++)
    {
        m_arSrcInnEnumeration[i].Reset(AIR_BALL::ST_SRC_AMOUNT);

        for (qint32 j=0; j<AIR_BALL::ST_SRC_AMOUNT; j++) {
            ST_IDs[i][j] = nullptr;
        }
    }

    for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++) {
        Ball_IDs_forGT[i] = nullptr;
    }

    m_CellsBMMark.ArrCells.Reset();
    ReadCellsBMMark();

    m_CAirBallProc.Reset();
    m_p_Arr_BallProc = &m_CAirBallProc.m_arProcData;
    ClearInnerData_GT();
    m_CAirBallProc.FillHeBoundaryValues(&m_CellsBMMark);

    try
    {
        m_pLogIDs   = OpenLog("IDs_Ball_M", AIR_BALL::log_path);
        m_pLogProgn = OpenLog("Prediction_Data_M", AIR_BALL::log_path);
        m_pLogInp   = OpenLog("Input_AIR_BALL_M", AIR_BALL::log_path);
        m_pLogTest  = OpenLog("TestDataBallST_M", AIR_BALL::log_path);
        m_pLogRDID  = OpenLog("RD_ID_AIR_BALL_M", AIR_BALL::log_path);
        m_pLogOut   = OpenLog("Output_AIR_BALL_M", AIR_BALL::log_path);
    }
    catch(...) {
        DPS_ASSERT(false);
    }
}


void CtrlAIR_BALLIST_M::SetPersistentData(const Str_Ar_EAP_Param_AllTables &arrEAP_Tables,
                                        const Str_Ar_BallCoef_Param_AllTables &arrBallCoef_Tables)
{
    m_EAP_Tables.Reset();
    m_BallCoef_Tables.Reset();

    m_EAP_Tables = arrEAP_Tables;
    m_BallCoef_Tables = arrBallCoef_Tables;

    qint32 i;
    if (m_EAP_Tables.N_el > 0)
    {
        for (i=0; i<m_EAP_Tables.N_el; i++)
        {
            m_EAP_Tables.arData[i].BallMark_inner = ConvertBallSubcl_GDtoAirBall(m_EAP_Tables.arData[i].BallSubclass);
        }
    }

    if (m_BallCoef_Tables.N_el > 0)
    {
        for (i=0; i<m_BallCoef_Tables.N_el; i++)
        {
            m_BallCoef_Tables.arData[i].BallMark_inner = ConvertBallSubcl_GDtoAirBall(m_BallCoef_Tables.arData[i].BallSubclass);
        }
    }

    m_EAP_Tables.LogData(m_pLogRDID);
    m_BallCoef_Tables.LogData(m_pLogRDID);

    m_CAirBallProc.SetPersistentData(&m_EAP_Tables, &m_BallCoef_Tables);    
}

void CtrlAIR_BALLIST_M::SetPersistentData(const Str_Ar_EAP_Param_AllTables &arrEAP_Tables)
{
//    QMutexLocker LockBallist(&m_CritSect);
    m_EAP_Tables.Reset();

    m_EAP_Tables = arrEAP_Tables;

    qint32 i;
    if (m_EAP_Tables.N_el > 0)
    {
        for (i=0; i<m_EAP_Tables.N_el; i++)
        {
            m_EAP_Tables.arData[i].BallMark_inner = ConvertBallSubcl_GDtoAirBall(m_EAP_Tables.arData[i].BallSubclass);
        }
    }

    m_EAP_Tables.LogData(m_pLogRDID);

    m_CAirBallProc.SetPersistentData(&m_EAP_Tables);
//    LockBallist.unlock();
}


void CtrlAIR_BALLIST_M::FillCovObj(const AIR_BALL::s_arCovObj &arr_CovObj)
{
//    QMutexLocker LockBallist(&m_CritSect);
    m_arCovObj = arr_CovObj;
    m_arCovObj.LogData(m_pLogRDID);
//    LockBallist.unlock();
}


qint32 CtrlAIR_BALLIST_M::FillTrajParam(const CtrlAIR_BALL_Input_GT_Data &inp_data)
{
    qint32 NumGTInner = GetInnerTrackNum(inp_data.NumbGT);
    const qint32 ind_GT = GetIdGT(inp_data.tLoc, inp_data.NumbGT, NumGTInner); //m_GT_Data.ID;
//    m_GT_Data[ind_GT] = inp_data;

    tLog(m_pLogIDs, "Mode: %d", AIR_BALL::GT_DATA);
//    m_GT_Data[ind_GT].LogData(m_pLogInp);
    const_cast<CtrlAIR_BALL_Input_GT_Data&>(inp_data).LogData(m_pLogInp);
    return ind_GT;
}


qint32 CtrlAIR_BALLIST_M::FillTrajParam(const CtrlAIR_BALL_Input_ST_Data &inp_data)
{
    const qint32 IndST = GetIdST(inp_data.tLoc, inp_data.SrcInd, inp_data.NtrInSrc);
//    m_ST_Data[IndST] = inp_data;

//    m_ST_Data[IndST].CovMatr.ReflectNonZeroRelativeDiag(); //Reflect non-zero elements relative to the matrix diagonal
//    m_ST_Data[IndST].CovMatr.FCovMatrAdaptation(DIM_CVA, cMinRMSE, cCovCorr);

    tLog(m_pLogIDs, "Mode: %d", AIR_BALL::ST_DATA);
//    m_ST_Data[IndST].LogData(m_pLogInp);
    const_cast<CtrlAIR_BALL_Input_ST_Data&>(inp_data).LogData(m_pLogInp);
    return IndST;
}


void CtrlAIR_BALLIST_M::FillTrajParam(const CtrlAIR_BALL_traj_param &tr_param)
{
//    m_TrajParam  = tr_param;
//    m_InDataMode = AIR_BALL::TEST_DATA;
    tLog(m_pLogIDs, "Mode: %d", AIR_BALL::TEST_DATA);
}


void CtrlAIR_BALLIST_M::SetCurTime(qreal _CurTime)
{
    m_CurTime = _CurTime;
}


void CtrlAIR_BALLIST_M::ProcessingTrajParam(qint16 InDataMode, void* pInData,
                                            CtrlAIR_BALL_Output_GT_Data *pOutGT)
{
    if (pOutGT != nullptr) {
        pOutGT->Reset();
    }

    try
    {
        if( !InDataMode ) {
            return;
        }

        InpMsgsProcessing(); //processing of input messages

        CtrlAIR_BALL_Input_GT_Data* l_pInDataGT{nullptr};
        CtrlAIR_BALL_Input_ST_Data* l_pInDataST{nullptr};

        if (InDataMode == AIR_BALL::GT_DATA)
        {
            l_pInDataGT = static_cast<CtrlAIR_BALL_Input_GT_Data*>(pInData);
            if (l_pInDataGT->TrackingSign == DROP_COORD) //drop of GT
            {
                Drop_GT(l_pInDataGT);
                return;
            }
            else
            {
                const qint32 NumGTInner = GetInnerTrackNum(l_pInDataGT->NumbGT);
                const qint32 ind_GT = GetIdGT(l_pInDataGT->tLoc, l_pInDataGT->NumbGT, NumGTInner);
                bool bBadInput = (fabs(l_pInDataGT->X - 0xffff) < con_par_eps
                                  && fabs(l_pInDataGT->Y - 0xffff) < con_par_eps
                                  && fabs(l_pInDataGT->Z - 0xffff) < con_par_eps);

                CtrlAIR_BALL_Output_GT_Data l_CAirBallOut;
                if (ind_GT >= 0 && ind_GT < AIR_BALL::GT_FORM_AMOUNT && !bBadInput)
                {
                    m_arDataCtrl[ind_GT].Class = l_pInDataGT->Class;

                    if (l_pInDataGT->Class == CLASS_BALLISTIC)
                    {
                        CAirBall_Proc_InputData_GT l_InDataProcGT;
                        FillBallProcInData(AIR_BALL::GT_DATA, l_pInDataGT, &l_InDataProcGT);

                        CAirBall_Proc_OutData l_OutData_Ballist;
                        m_CAirBallProc.AirBallist_Realization(&l_InDataProcGT, l_OutData_Ballist);

                        FormOutputData_GT(ind_GT, l_OutData_Ballist, l_CAirBallOut);

                        if (l_CAirBallOut.AnyChanges()) {
                            l_CAirBallOut.LogData(m_pLogOut, l_pInDataGT->NumbGT, ind_GT, l_pInDataGT->tLoc, false, true);
                        }

    #ifdef KEEP_LOG
                        QString l_log = QString("%1\t %2\t %3\t %4\t %5\t %6\t %7\t %8")
                                .arg(ind_GT, 5).arg(l_pInDataGT->tLoc, 0, 'f', 5)
                                .arg(m_p_Arr_BallProc->arData_GT[ind_GT].Path).arg(m_p_Arr_BallProc->arData_GT[ind_GT].Branch)
                                .arg(m_p_Arr_BallProc->arData_GT[ind_GT].TrajType).arg(m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.MarkValue)
                                .arg(m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.minBMMark).arg(m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.maxBMMark);
                        GLFileLog::tLog(AIR_BALL::out_BallResult, "%s", qPrintable(l_log));
    #endif //KEEP_LOG
                        if (fabs(m_arDataCtrl[ind_GT].t_waiting_begin - AIR_BALL::Timer_INI) > con_eps)
                        {
                            m_arDataCtrl[ind_GT].t_waiting_begin = AIR_BALL::Timer_INI;
                        }
                    }
                    else //not ballistic at the input
                    {
                        NonBallGTProcessing(l_pInDataGT, ind_GT);

                        if (l_pInDataGT->Class == CLASS_SATELITE)
                        {
                            CAirBall_Proc_InputData_GT l_InDataProcGT;
                            FillBallProcInData(AIR_BALL::GT_DATA, l_pInDataGT, &l_InDataProcGT);

                            CAirBall_Proc_OutData OutSat;
                            m_CAirBallProc.AirBallist_Realization_Satellite(&l_InDataProcGT, OutSat);

                            if (OutSat.bNewPrediction)
                            {
                                l_CAirBallOut.bNewPolynomSat = true;
                                l_CAirBallOut.Predict.Pol = OutSat.Polynoms;

                                l_CAirBallOut.Predict.tFall = l_pInDataGT->tLoc + c_TimeProlSatellite;

                                tLog(m_pLogOut, "===========================================================");
                                tLog(m_pLogOut, "NumGT: %d ID_GT: %d tLoc: %lf tEndProgn: %lf",
                                     l_pInDataGT->NumbGT, ind_GT, l_pInDataGT->tLoc, l_CAirBallOut.Predict.tFall);
                                l_CAirBallOut.Predict.Pol.LogData(m_pLogOut, "POLYNOM FOR SATELLITE: ");
                            }
                        }
                    }
                }

                if (pOutGT != nullptr && l_CAirBallOut.AnyChanges()) {
                    *pOutGT = l_CAirBallOut;
                }
            }
        }
        else if (InDataMode == AIR_BALL::ST_DATA)
        {
            l_pInDataST = static_cast<CtrlAIR_BALL_Input_ST_Data*>(pInData);
            if (l_pInDataST->TrackingSign == DROP_COORD) //drop of ST
            {
                Drop_ST(l_pInDataST);
                return;
            }
            else
            {
                CAirBall_Proc_InputData_ST l_InDataProcST;
                FillBallProcInData(AIR_BALL::ST_DATA, l_pInDataST, &l_InDataProcST);

                bool bBadInput = (fabs(l_InDataProcST.Coord.x - 0xffff) < con_par_eps
                                   && fabs(l_InDataProcST.Coord.y - 0xffff) < con_par_eps
                                   && fabs(l_InDataProcST.Coord.z - 0xffff) < con_par_eps);

                if (l_InDataProcST.IF_ST >= 0
                        && l_InDataProcST.IF_ST < AIR_BALL::ST_FORM_AMOUNT
                        && !bBadInput)
                {
                    m_CAirBallProc.AirBallist_Realization_ST(&l_InDataProcST);
                }
            }
        }
        else {
        }

        if (InDataMode == AIR_BALL::TEST_DATA) //test data processing
        {
            // TODO check and correct later
            /*
            qint32 NumGTInner = GetInnerTrackNum(m_BallInpData.IF_GT);
            FillBallProcInData(AIR_BALL::TEST_DATA, -1, NumGTInner);

            const qint32 ind_GT = m_BallInpData.IF_GT;
//            const qint32 ind_ST = m_BallInpData_ST.IF_ST;

            if (ind_GT >= 0 && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
            {
                m_arDataCtrl[ind_GT].Class = m_BallInpData.Class;
            }

            m_CAirBallProc.AirBallist_Realization_ST();

//            m_p_Arr_BallProc->arData_GT[ind_GT].Path = m_p_Arr_BallProc->arData_ST[ind_ST].Path;
//            m_p_Arr_BallProc->arData_GT[ind_GT].Branch = m_p_Arr_BallProc->arData_ST[ind_ST].Branch;

            m_CAirBallProc.SetNumbGT(ind_GT, ind_GT);
            m_CAirBallProc.SetActionPhase(ind_GT, ASSESSMENT_RESULT, m_BallInpData.tLoc);

            m_CAirBallProc.AirBallist_Realization();

            FormOutputData_GT(ind_GT);

            if (m_CAirBallOut.AnyChanges())
            {
                m_CAirBallOut.LogData(m_pLogOut, m_BallInpData.IF_GT, ind_GT, m_BallInpData.tLoc, false, false);
            }

#ifdef KEEP_LOG
            std::streamsize ss = AIR_BALL::out_BallResult.precision();
            AIR_BALL::out_BallResult << std::setprecision(10)
                                     << m_TrajParam.ID << "\t" << m_TrajParam.tLoc << "\t"
                                     << m_p_Arr_BallProc->arData_GT[ind_GT].Path << "\t"
                                     << m_p_Arr_BallProc->arData_GT[ind_GT].Branch << "\t"
                                     << m_p_Arr_BallProc->arData_GT[ind_GT].TrajType << "\t"
                                     << m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.MarkValue << "\t"
                                     << m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.minBMMark << "\t"
                                     << m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.maxBMMark << std::endl;
            AIR_BALL::out_BallResult.precision(ss);
#endif //KEEP_LOG
            */
        }
//        lockBuffer.unlock();
    }
    catch (...)
    {
    }
}


bool CtrlAIR_BALLIST_M::ProcessingAirBallist(const CtrlAIR_BALL_Input_ST_Data &InData)
{
    try
    {
        FillTrajParam(InData);
        ProcessingTrajParam(AIR_BALL::ST_DATA, const_cast<CtrlAIR_BALL_Input_ST_Data*>(&InData), nullptr);
    }
    catch(...) {
    }
    return true;
}


bool CtrlAIR_BALLIST_M::ProcessingAirBallist(const CtrlAIR_BALL_Input_GT_Data &InData,
                                             CtrlAIR_BALL_Output_GT_Data &OutData)
{
    QMutexLocker l_lock(&m_mutexProc);
    // track is processing right now
    if( m_setProcGT.find(InData.NumbGT) != m_setProcGT.end()) {
        return false;
    }
    // add track to processing set
    m_setProcGT.insert(InData.NumbGT);
    l_lock.unlock();

    try
    {
        OutData.Reset();

        FillTrajParam(InData);
        ProcessingTrajParam(AIR_BALL::GT_DATA, const_cast<CtrlAIR_BALL_Input_GT_Data*>(&InData), &OutData);
    }
    catch(...) {
    }

    // remove track from processing set
    l_lock.relock();
    m_setProcGT.erase(InData.NumbGT);
    l_lock.unlock();

    return true;
}


void CtrlAIR_BALLIST_M::InpMsgsProcessing()
{
    QMutexLocker lockBuffer(&m_InpMsgsBuf);

    qreal l_CurTime = m_CurTime; //current time, seconds from the day's beginning
    if (l_CurTime < 0) {
        l_CurTime = CURR_TIME_OF_70_SEC;
    }

    if (m_InpMsgsBuf.MsgsAmount > 0 && m_InpMsgsBuf.MsgsAmount <= AIR_BALL::INP_MSGS_AMOUNT)
    {
        m_InpMsgsBuf.LogData(m_pLogInp);

        for (qint32 i=0; i<m_InpMsgsBuf.MsgsAmount; i++)
        {
            if (m_InpMsgsBuf.MsgsAr[i].MsgType == AIR_BALL::Str_InpMsg::NEW_ACTION_PHASE)
            {
                qint32 NumGTInner_Tgt = GetInnerTrackNum(m_InpMsgsBuf.MsgsAr[i].NumbGT_Tgt);
                m_CAirBallProc.SetActionPhase(NumGTInner_Tgt, m_InpMsgsBuf.MsgsAr[i].ActionPhase, l_CurTime);
            }
            else if (m_InpMsgsBuf.MsgsAr[i].MsgType == AIR_BALL::Str_InpMsg::SWAP_NUMBERS)
            {
                SwapNumbers(m_InpMsgsBuf.MsgsAr[i].NumbGT_Tgt, m_InpMsgsBuf.MsgsAr[i].NumbGT_IC);
            }
            else if (m_InpMsgsBuf.MsgsAr[i].MsgType == AIR_BALL::Str_InpMsg::BALL_COEFF)
            {
                qint32 NumGTInner_Tgt = GetInnerTrackNum(m_InpMsgsBuf.MsgsAr[i].NumbGT_Tgt);
                m_CAirBallProc.SetBallCoeffSrc(NumGTInner_Tgt, m_InpMsgsBuf.MsgsAr[i].Gamma);
            }
            else
            {
            }
        }
    }

    m_InpMsgsBuf.Reset();

    lockBuffer.unlock();
}


void CtrlAIR_BALLIST_M::NonBallGTProcessing(const CtrlAIR_BALL_Input_GT_Data* InData, qint32 ind_GT)
{
    qint32 ind_Ball = -1; //check whether ballistic index was provided
    if (ind_GT >= 0 && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGT);
        if (Ball_IDs_forGT[ind_GT] != nullptr) {
            ind_Ball = *Ball_IDs_forGT[ind_GT];
        }
        l_lock.unlock();

        //fill information about 1st update
        if (!m_p_Arr_BallProc->arData_GT[ind_GT].IsTaken)
        {
            CAirBall_Proc_InputData_GT l_InDataProcGT;
            FillBallProcInData(AIR_BALL::GT_DATA, const_cast<CtrlAIR_BALL_Input_GT_Data*>(InData), &l_InDataProcGT);
            m_CAirBallProc.NewTrackProcessing_GT(&l_InDataProcGT);
        }
    }
    if (ind_Ball > -1) //ballistic index was provided, i.e. current track before was classified as BT
    {        
        if (fabs(m_arDataCtrl[ind_GT].t_waiting_begin - AIR_BALL::Timer_INI) < con_eps)
        {
            m_arDataCtrl[ind_GT].t_waiting_begin = InData->tLoc;
        }
        else
        {
            if (InData->tLoc - m_arDataCtrl[ind_GT].t_waiting_begin > cDT_WAITING_FOR_RECLASSIFIED)
            {
                ClearBallisticData(InData->tLoc, ind_GT, ind_Ball); //clear ballistic data if BT was reclassified to non-BT a long time
                m_arDataCtrl[ind_GT].t_waiting_begin = AIR_BALL::Timer_INI;
            }
        }
    }
}


void CtrlAIR_BALLIST_M::SwapNumbers(qint32 Num1, qint32 Num2)
{
    qint32 Num1Inner = GetInnerTrackNum(Num1);
    qint32 Num2Inner = GetInnerTrackNum(Num2);
    if (0 < Num1Inner && 0 < Num2Inner /*&& Num1Inner < AIR_BALL::GT_FORM_AMOUNT
             && Num2Inner < AIR_BALL::GT_FORM_AMOUNT*/)
    {
        qint32 indGT1 = GetIdGT(0., Num1, Num1Inner);
        qint32 indGT2 = GetIdGT(0., Num2, Num2Inner);
        if (0 <= indGT1 && indGT1 < AIR_BALL::GT_FORM_AMOUNT
                && 0 <= indGT2 && indGT2 < AIR_BALL::GT_FORM_AMOUNT)
        {
            char OutString[200];
            sprintf(OutString, "----- SWAP Num1 %d Num2 %d OldInd1 %d OldInd2 %d",
                    Num1, Num2, indGT1, indGT2);

            QMutexLocker l_lock(&m_mutexGT);
            qint32 *p_tmp = GT_IDs[Num1Inner];
            GT_IDs[Num1Inner] = GT_IDs[Num2Inner];
            GT_IDs[Num2Inner] = p_tmp;
            l_lock.unlock();

            m_CAirBallProc.SetNumbGT(indGT1, Num2Inner);
            m_CAirBallProc.SetNumbGT(indGT2, Num1Inner);

            qint32 ind1cmp = GetIdGT(0., Num1, Num1Inner);
            qint32 ind2cmp = GetIdGT(0., Num2, Num2Inner);
            tLog(m_pLogIDs, "%s NewInd1 %d NewInd2 %d -----", OutString, ind1cmp, ind2cmp);
        }
    }
}


void CtrlAIR_BALLIST_M::FillBallProcInData(qint16 InDataMode, void* pInData, void* pInDataProc)
{
    CtrlAIR_BALL_Input_GT_Data* l_pInDataGT{nullptr};
    CtrlAIR_BALL_Input_ST_Data* l_pInDataST{nullptr};

    CAirBall_Proc_InputData_GT* l_pInDataProcGT{nullptr};
    CAirBall_Proc_InputData_ST* l_pInDataProcST{nullptr};

    if (InDataMode == AIR_BALL::TEST_DATA)
    {
        // TODO check and fix later
        /*

        //fill test input data for generalized track
        m_BallInpData.IF_GT   = m_TrajParam.ID;
        m_BallInpData.NumbGT  = NumGTInner;
        m_BallInpData.SetST.insert(m_TrajParam.ID);

        bool bBallist = IsBallistGT(m_BallInpData.IF_GT);
        m_BallInpData.IF_BALL = GetIdBall(m_TrajParam.ID, bBallist);

        m_BallInpData.tLoc    = m_TrajParam.tLoc;
        m_BallInpData.SignQuickReaction = !m_TrajParam.Sign_sufficient_time;
        m_BallInpData.Sign_large_load = (m_BO_quantity > AIR_BALL::BALL_LARGE_LOAD_AMOUNT ? true : false) ;

        m_BallInpData.Coord.x   = m_TrajParam.X;
        m_BallInpData.Coord.y   = m_TrajParam.Y;
        m_BallInpData.Coord.z   = m_TrajParam.Z;

        m_BallInpData.Vel.x     = m_TrajParam.VX;
        m_BallInpData.Vel.y     = m_TrajParam.VY;
        m_BallInpData.Vel.z     = m_TrajParam.VZ;

        m_BallInpData.Acc.x     = m_TrajParam.AX;
        m_BallInpData.Acc.y     = m_TrajParam.AY;
        m_BallInpData.Acc.z     = m_TrajParam.AZ;

        m_BallInpData.Class     = m_TrajParam.Class;
        m_BallInpData.BallType  = m_TrajParam.BallSubclass;

//        m_BallInpData.CovMatr.s_m   = 9;
//        m_BallInpData.CovMatr.s_n   = 9;
        m_BallInpData.CovMatr.Reset(9,9);

        m_BallInpData.CovMatr.M[0][0] = pow(m_TrajParam.SigX,2);
        m_BallInpData.CovMatr.M[1][1] = pow(m_TrajParam.SigY,2);
        m_BallInpData.CovMatr.M[2][2] = pow(m_TrajParam.SigZ,2);
        m_BallInpData.CovMatr.M[3][3] = pow(m_TrajParam.SigVX,2);
        m_BallInpData.CovMatr.M[4][4] = pow(m_TrajParam.SigVY,2);
        m_BallInpData.CovMatr.M[5][5] = pow(m_TrajParam.SigVZ,2);
        m_BallInpData.CovMatr.M[6][6] = pow(m_TrajParam.SigAX,2);
        m_BallInpData.CovMatr.M[7][7] = pow(m_TrajParam.SigAY,2);
        m_BallInpData.CovMatr.M[8][8] = pow(m_TrajParam.SigAZ,2);

        m_BallInpData.CovMatr.M[0][1] = m_TrajParam.KXY;
        m_BallInpData.CovMatr.M[1][0] = m_TrajParam.KXY;

        m_BallInpData.CovMatr.M[0][2] = m_TrajParam.KXZ;
        m_BallInpData.CovMatr.M[2][0] = m_TrajParam.KXZ;

        m_BallInpData.CovMatr.M[0][3] = m_TrajParam.KXVx;
        m_BallInpData.CovMatr.M[3][0] = m_TrajParam.KXVx;

        m_BallInpData.CovMatr.M[0][4] = m_TrajParam.KXVy;
        m_BallInpData.CovMatr.M[4][0] = m_TrajParam.KXVy;

        m_BallInpData.CovMatr.M[0][5] = m_TrajParam.KXVz;
        m_BallInpData.CovMatr.M[5][0] = m_TrajParam.KXVz;

        m_BallInpData.CovMatr.M[0][6] = m_TrajParam.KXAx;
        m_BallInpData.CovMatr.M[6][0] = m_TrajParam.KXAx;

        m_BallInpData.CovMatr.M[0][7] = m_TrajParam.KXAy;
        m_BallInpData.CovMatr.M[7][0] = m_TrajParam.KXAy;

        m_BallInpData.CovMatr.M[0][8] = m_TrajParam.KXAz;
        m_BallInpData.CovMatr.M[8][0] = m_TrajParam.KXAz;

        m_BallInpData.CovMatr.M[1][2] = m_TrajParam.KYZ;
        m_BallInpData.CovMatr.M[2][1] = m_TrajParam.KYZ;

        m_BallInpData.CovMatr.M[1][3] = m_TrajParam.KYVx;
        m_BallInpData.CovMatr.M[3][1] = m_TrajParam.KYVx;

        m_BallInpData.CovMatr.M[1][4] = m_TrajParam.KYVy;
        m_BallInpData.CovMatr.M[4][1] = m_TrajParam.KYVy;

        m_BallInpData.CovMatr.M[1][5] = m_TrajParam.KYVz;
        m_BallInpData.CovMatr.M[5][1] = m_TrajParam.KYVz;

        m_BallInpData.CovMatr.M[1][6] = m_TrajParam.KYAx;
        m_BallInpData.CovMatr.M[6][1] = m_TrajParam.KYAx;

        m_BallInpData.CovMatr.M[1][7] = m_TrajParam.KYAy;
        m_BallInpData.CovMatr.M[7][1] = m_TrajParam.KYAy;

        m_BallInpData.CovMatr.M[1][8] = m_TrajParam.KYAz;
        m_BallInpData.CovMatr.M[8][1] = m_TrajParam.KYAz;

        m_BallInpData.CovMatr.M[2][3] = m_TrajParam.KZVx;
        m_BallInpData.CovMatr.M[3][2] = m_TrajParam.KZVx;

        m_BallInpData.CovMatr.M[2][4] = m_TrajParam.KZVy;
        m_BallInpData.CovMatr.M[4][2] = m_TrajParam.KZVy;

        m_BallInpData.CovMatr.M[2][5] = m_TrajParam.KZVz;
        m_BallInpData.CovMatr.M[5][2] = m_TrajParam.KZVz;

        m_BallInpData.CovMatr.M[2][6] = m_TrajParam.KZAx;
        m_BallInpData.CovMatr.M[6][2] = m_TrajParam.KZAx;

        m_BallInpData.CovMatr.M[2][7] = m_TrajParam.KZAy;
        m_BallInpData.CovMatr.M[7][2] = m_TrajParam.KZAy;

        m_BallInpData.CovMatr.M[2][8] = m_TrajParam.KZAz;
        m_BallInpData.CovMatr.M[8][2] = m_TrajParam.KZAz;

        m_BallInpData.CovMatr.M[3][4] = m_TrajParam.KVxVy;
        m_BallInpData.CovMatr.M[4][3] = m_TrajParam.KVxVy;

        m_BallInpData.CovMatr.M[3][5] = m_TrajParam.KVxVz;
        m_BallInpData.CovMatr.M[5][3] = m_TrajParam.KVxVz;

        m_BallInpData.CovMatr.M[3][6] = m_TrajParam.KVxAx;
        m_BallInpData.CovMatr.M[6][3] = m_TrajParam.KVxAx;

        m_BallInpData.CovMatr.M[3][7] = m_TrajParam.KVxAy;
        m_BallInpData.CovMatr.M[7][3] = m_TrajParam.KVxAy;

        m_BallInpData.CovMatr.M[3][8] = m_TrajParam.KVxAz;
        m_BallInpData.CovMatr.M[8][3] = m_TrajParam.KVxAz;

        m_BallInpData.CovMatr.M[4][5] = m_TrajParam.KVyVz;
        m_BallInpData.CovMatr.M[5][4] = m_TrajParam.KVyVz;

        m_BallInpData.CovMatr.M[4][6] = m_TrajParam.KVyAx;
        m_BallInpData.CovMatr.M[6][4] = m_TrajParam.KVyAx;

        m_BallInpData.CovMatr.M[4][7] = m_TrajParam.KVyAy;
        m_BallInpData.CovMatr.M[7][4] = m_TrajParam.KVyAy;

        m_BallInpData.CovMatr.M[4][8] = m_TrajParam.KVyAz;
        m_BallInpData.CovMatr.M[8][4] = m_TrajParam.KVyAz;

        m_BallInpData.CovMatr.M[5][6] = m_TrajParam.KVzAx;
        m_BallInpData.CovMatr.M[6][5] = m_TrajParam.KVzAx;

        m_BallInpData.CovMatr.M[5][7] = m_TrajParam.KVzAy;
        m_BallInpData.CovMatr.M[7][5] = m_TrajParam.KVzAy;

        m_BallInpData.CovMatr.M[5][8] = m_TrajParam.KVzAz;
        m_BallInpData.CovMatr.M[8][5] = m_TrajParam.KVzAz;

        m_BallInpData.CovMatr.M[6][7] = m_TrajParam.KAxAy;
        m_BallInpData.CovMatr.M[7][6] = m_TrajParam.KAxAy;

        m_BallInpData.CovMatr.M[6][8] = m_TrajParam.KAxAz;
        m_BallInpData.CovMatr.M[8][6] = m_TrajParam.KAxAz;

        m_BallInpData.CovMatr.M[7][8] = m_TrajParam.KAyAz;
        m_BallInpData.CovMatr.M[8][7] = m_TrajParam.KAyAz;

//        tLog(m_pLogInp, "Input Data GT:  tLoc | Coord: %f %f %f | Vel: %f %f %f | Acc: %f %f %f",
//             m_BallInpData.tLoc, m_BallInpData.Coord.x, m_BallInpData.Coord.y, m_BallInpData.Coord.z,
//             m_BallInpData.Vel.x, m_BallInpData.Vel.y, m_BallInpData.Vel.z,
//             m_BallInpData.Acc.x, m_BallInpData.Acc.y, m_BallInpData.Acc.z);
//        LogMatrix(m_pLogInp, "Input Matrix GT", m_BallInpData.tLoc, m_BallInpData.CovMatr);
        m_BallInpData.CovMatr.FCovMatrAdaptation(DIM_CVA, cMinRMSE, cCovCorr);
//        LogMatrix(m_pLogInp, "Input Matrix GT after corr.", m_BallInpData.tLoc, m_BallInpData.CovMatr);

//        m_BallInpData.Prob_IMM_small_coeff = m_TrajParam.Prob_IMM_small_coeff;
//        m_BallInpData.Prob_IMM_large_coeff = m_TrajParam.Prob_IMM_large_coeff;
//        m_BallInpData.R_rdr_obj = m_TrajParam.R_rdr_obj;

//        qreal SigmaCoordMax = max(m_TrajParam.SigX, max(m_TrajParam.SigY, m_TrajParam.SigZ));
//        qreal SigmaVelMax = max(m_TrajParam.SigVX, max(m_TrajParam.SigVY, m_TrajParam.SigVZ));

//        //m_BallInpData.SignDetBMMark = true; //choise of table of cells for BM Mark determination, depending on RMSEs
//        if (SigmaCoordMax <= SIGMA_COORD_DIAP1 && SigmaVelMax <= SIGMA_VEL_DIAP1)
//            m_CellsBMMark = m_TrajParam.Cells_BMMark_diap1;
//        else if (SigmaCoordMax <= SIGMA_COORD_DIAP2 && SigmaVelMax <= SIGMA_VEL_DIAP2)
//            m_CellsBMMark = m_TrajParam.Cells_BMMark_diap2;
//        else if (SigmaCoordMax <= SIGMA_COORD_DIAP3 && SigmaVelMax <= SIGMA_VEL_DIAP3)
//            m_CellsBMMark = m_TrajParam.Cells_BMMark_diap3;
////        else
//            m_BallInpData.SignDetBMMark = false;
        //m_BallInpData.Cells_BMMark = &m_TrajParam.Cells_BMMark;
        m_BallInpData.Cells_BMMark = &m_CellsBMMark;

        m_CAirBallProc.FillInputData(m_BallInpData);

        //fill test input data for single track
        m_BallInpData_ST.IF_GT   = m_TrajParam.ID;
        m_BallInpData_ST.IF_ST   = m_TrajParam.ID;

        bBallist = IsBallistGT(m_BallInpData.IF_GT);
        m_BallInpData_ST.IF_BALL = GetIdBall(m_TrajParam.ID, bBallist);

        m_BallInpData_ST.tLoc    = m_TrajParam.tLoc;
        m_BallInpData_ST.SignQuickReaction = !m_TrajParam.Sign_sufficient_time;
        m_BallInpData_ST.Sign_large_load = (m_BO_quantity > AIR_BALL::BALL_LARGE_LOAD_AMOUNT ? true : false) ;

        m_BallInpData_ST.Coord.x   = m_TrajParam.X;
        m_BallInpData_ST.Coord.y   = m_TrajParam.Y;
        m_BallInpData_ST.Coord.z   = m_TrajParam.Z;

        m_BallInpData_ST.Vel.x     = m_TrajParam.VX;
        m_BallInpData_ST.Vel.y     = m_TrajParam.VY;
        m_BallInpData_ST.Vel.z     = m_TrajParam.VZ;

        m_BallInpData_ST.Acc.x     = m_TrajParam.AX;
        m_BallInpData_ST.Acc.y     = m_TrajParam.AY;
        m_BallInpData_ST.Acc.z     = m_TrajParam.AZ;

        m_BallInpData_ST.Class     = m_TrajParam.Class;
        m_BallInpData_ST.BallType  = m_TrajParam.BallSubclass;

//        m_BallInpData.CovMatr.s_m   = 9;
//        m_BallInpData.CovMatr.s_n   = 9;
        m_BallInpData_ST.CovMatr.Reset(9,9);
        m_BallInpData_ST.CovMatr = m_BallInpData.CovMatr;

        m_BallInpData_ST.Prob_IMM_small_coeff = m_TrajParam.Prob_IMM_small_coeff;
        m_BallInpData_ST.Prob_IMM_large_coeff = m_TrajParam.Prob_IMM_large_coeff;
        m_BallInpData_ST.R_rdr_obj = m_TrajParam.R_rdr_obj;

        m_CAirBallProc.FillInputData(m_BallInpData_ST);

        LogTestData(m_pLogTest, m_BallInpData_ST.IF_ST, m_ST_Data.tLoc, m_BallInpData_ST.Class, 0,
                    m_ST_Data.VectParam, m_BallInpData_ST.CovMatr);
        */
    }
    else if (InDataMode == AIR_BALL::GT_DATA)
    {
        l_pInDataGT = static_cast<CtrlAIR_BALL_Input_GT_Data*>(pInData);
        l_pInDataProcGT = static_cast<CAirBall_Proc_InputData_GT*>(pInDataProc);

        const qint32 NumGTInner = GetInnerTrackNum(l_pInDataGT->NumbGT);
        const qint32 ID_GT = GetIdGT(l_pInDataGT->tLoc, l_pInDataGT->NumbGT, NumGTInner);

        if (0 <= ID_GT && ID_GT < AIR_BALL::GT_FORM_AMOUNT)
        {
            CtrlAIR_BALL_Input_GT_Data& l_GT_Data = *l_pInDataGT;
            CAirBall_Proc_InputData_GT& l_BallInpDataGT = *l_pInDataProcGT;

            l_BallInpDataGT.Reset();

            l_BallInpDataGT.IF_GT   = ID_GT;
            l_BallInpDataGT.NumbGT  = NumGTInner;

            const bool bBallist = IsBallistGT(AIR_BALL::GT_DATA, ID_GT, l_pInDataGT->Class);
            l_BallInpDataGT.IF_BALL = GetIdBall(l_GT_Data.tLoc, ID_GT, bBallist);

            l_BallInpDataGT.tLoc    = l_GT_Data.tLoc;
            l_BallInpDataGT.SignQuickReaction = l_GT_Data.SignQuickReaction;
            l_BallInpDataGT.Sign_large_load = (m_BO_quantity > AIR_BALL::BALL_LARGE_LOAD_AMOUNT ? true : false);

            l_BallInpDataGT.Coord.x   = l_GT_Data.X;
            l_BallInpDataGT.Coord.y   = l_GT_Data.Y;
            l_BallInpDataGT.Coord.z   = l_GT_Data.Z;

            l_BallInpDataGT.Vel.x     = l_GT_Data.VX;
            l_BallInpDataGT.Vel.y     = l_GT_Data.VY;
            l_BallInpDataGT.Vel.z     = l_GT_Data.VZ;

            l_BallInpDataGT.Acc.x     = l_GT_Data.AX;
            l_BallInpDataGT.Acc.y     = l_GT_Data.AY;
            l_BallInpDataGT.Acc.z     = l_GT_Data.AZ;

            l_BallInpDataGT.Class     = l_GT_Data.Class;
            l_BallInpDataGT.BallType  = l_GT_Data.Type;
            l_BallInpDataGT.ClstNum   = l_GT_Data.ClstNum;
            l_BallInpDataGT.bAeroballistic = l_GT_Data.bAeroballistic;

            l_BallInpDataGT.CovMatr.Reset(9,9);

            l_BallInpDataGT.CovMatr.M[0][0] = pow(l_GT_Data.SigX,2);
            l_BallInpDataGT.CovMatr.M[1][1] = pow(l_GT_Data.SigY,2);
            l_BallInpDataGT.CovMatr.M[2][2] = pow(l_GT_Data.SigZ,2);
            l_BallInpDataGT.CovMatr.M[3][3] = pow(l_GT_Data.SigVX,2);
            l_BallInpDataGT.CovMatr.M[4][4] = pow(l_GT_Data.SigVY,2);
            l_BallInpDataGT.CovMatr.M[5][5] = pow(l_GT_Data.SigVZ,2);
            l_BallInpDataGT.CovMatr.M[6][6] = pow(l_GT_Data.SigAX,2);
            l_BallInpDataGT.CovMatr.M[7][7] = pow(l_GT_Data.SigAY,2);
            l_BallInpDataGT.CovMatr.M[8][8] = pow(l_GT_Data.SigAZ,2);

            l_BallInpDataGT.CovMatr.M[0][1] = l_GT_Data.KXY;
            l_BallInpDataGT.CovMatr.M[1][0] = l_GT_Data.KXY;

            l_BallInpDataGT.CovMatr.M[0][2] = l_GT_Data.KXZ;
            l_BallInpDataGT.CovMatr.M[2][0] = l_GT_Data.KXZ;

            l_BallInpDataGT.CovMatr.M[0][3] = l_GT_Data.KXVx;
            l_BallInpDataGT.CovMatr.M[3][0] = l_GT_Data.KXVx;

            l_BallInpDataGT.CovMatr.M[0][4] = l_GT_Data.KXVy;
            l_BallInpDataGT.CovMatr.M[4][0] = l_GT_Data.KXVy;

            l_BallInpDataGT.CovMatr.M[0][5] = l_GT_Data.KXVz;
            l_BallInpDataGT.CovMatr.M[5][0] = l_GT_Data.KXVz;

            l_BallInpDataGT.CovMatr.M[0][6] = l_GT_Data.KXAx;
            l_BallInpDataGT.CovMatr.M[6][0] = l_GT_Data.KXAx;

            l_BallInpDataGT.CovMatr.M[0][7] = l_GT_Data.KXAy;
            l_BallInpDataGT.CovMatr.M[7][0] = l_GT_Data.KXAy;

            l_BallInpDataGT.CovMatr.M[0][8] = l_GT_Data.KXAz;
            l_BallInpDataGT.CovMatr.M[8][0] = l_GT_Data.KXAz;

            l_BallInpDataGT.CovMatr.M[1][2] = l_GT_Data.KYZ;
            l_BallInpDataGT.CovMatr.M[2][1] = l_GT_Data.KYZ;

            l_BallInpDataGT.CovMatr.M[1][3] = l_GT_Data.KYVx;
            l_BallInpDataGT.CovMatr.M[3][1] = l_GT_Data.KYVx;

            l_BallInpDataGT.CovMatr.M[1][4] = l_GT_Data.KYVy;
            l_BallInpDataGT.CovMatr.M[4][1] = l_GT_Data.KYVy;

            l_BallInpDataGT.CovMatr.M[1][5] = l_GT_Data.KYVz;
            l_BallInpDataGT.CovMatr.M[5][1] = l_GT_Data.KYVz;

            l_BallInpDataGT.CovMatr.M[1][6] = l_GT_Data.KYAx;
            l_BallInpDataGT.CovMatr.M[6][1] = l_GT_Data.KYAx;

            l_BallInpDataGT.CovMatr.M[1][7] = l_GT_Data.KYAy;
            l_BallInpDataGT.CovMatr.M[7][1] = l_GT_Data.KYAy;

            l_BallInpDataGT.CovMatr.M[1][8] = l_GT_Data.KYAz;
            l_BallInpDataGT.CovMatr.M[8][1] = l_GT_Data.KYAz;

            l_BallInpDataGT.CovMatr.M[2][3] = l_GT_Data.KZVx;
            l_BallInpDataGT.CovMatr.M[3][2] = l_GT_Data.KZVx;

            l_BallInpDataGT.CovMatr.M[2][4] = l_GT_Data.KZVy;
            l_BallInpDataGT.CovMatr.M[4][2] = l_GT_Data.KZVy;

            l_BallInpDataGT.CovMatr.M[2][5] = l_GT_Data.KZVz;
            l_BallInpDataGT.CovMatr.M[5][2] = l_GT_Data.KZVz;

            l_BallInpDataGT.CovMatr.M[2][6] = l_GT_Data.KZAx;
            l_BallInpDataGT.CovMatr.M[6][2] = l_GT_Data.KZAx;

            l_BallInpDataGT.CovMatr.M[2][7] = l_GT_Data.KZAy;
            l_BallInpDataGT.CovMatr.M[7][2] = l_GT_Data.KZAy;

            l_BallInpDataGT.CovMatr.M[2][8] = l_GT_Data.KZAz;
            l_BallInpDataGT.CovMatr.M[8][2] = l_GT_Data.KZAz;

            l_BallInpDataGT.CovMatr.M[3][4] = l_GT_Data.KVxVy;
            l_BallInpDataGT.CovMatr.M[4][3] = l_GT_Data.KVxVy;

            l_BallInpDataGT.CovMatr.M[3][5] = l_GT_Data.KVxVz;
            l_BallInpDataGT.CovMatr.M[5][3] = l_GT_Data.KVxVz;

            l_BallInpDataGT.CovMatr.M[3][6] = l_GT_Data.KVxAx;
            l_BallInpDataGT.CovMatr.M[6][3] = l_GT_Data.KVxAx;

            l_BallInpDataGT.CovMatr.M[3][7] = l_GT_Data.KVxAy;
            l_BallInpDataGT.CovMatr.M[7][3] = l_GT_Data.KVxAy;

            l_BallInpDataGT.CovMatr.M[3][8] = l_GT_Data.KVxAz;
            l_BallInpDataGT.CovMatr.M[8][3] = l_GT_Data.KVxAz;

            l_BallInpDataGT.CovMatr.M[4][5] = l_GT_Data.KVyVz;
            l_BallInpDataGT.CovMatr.M[5][4] = l_GT_Data.KVyVz;

            l_BallInpDataGT.CovMatr.M[4][6] = l_GT_Data.KVyAx;
            l_BallInpDataGT.CovMatr.M[6][4] = l_GT_Data.KVyAx;

            l_BallInpDataGT.CovMatr.M[4][7] = l_GT_Data.KVyAy;
            l_BallInpDataGT.CovMatr.M[7][4] = l_GT_Data.KVyAy;

            l_BallInpDataGT.CovMatr.M[4][8] = l_GT_Data.KVyAz;
            l_BallInpDataGT.CovMatr.M[8][4] = l_GT_Data.KVyAz;

            l_BallInpDataGT.CovMatr.M[5][6] = l_GT_Data.KVzAx;
            l_BallInpDataGT.CovMatr.M[6][5] = l_GT_Data.KVzAx;

            l_BallInpDataGT.CovMatr.M[5][7] = l_GT_Data.KVzAy;
            l_BallInpDataGT.CovMatr.M[7][5] = l_GT_Data.KVzAy;

            l_BallInpDataGT.CovMatr.M[5][8] = l_GT_Data.KVzAz;
            l_BallInpDataGT.CovMatr.M[8][5] = l_GT_Data.KVzAz;

            l_BallInpDataGT.CovMatr.M[6][7] = l_GT_Data.KAxAy;
            l_BallInpDataGT.CovMatr.M[7][6] = l_GT_Data.KAxAy;

            l_BallInpDataGT.CovMatr.M[6][8] = l_GT_Data.KAxAz;
            l_BallInpDataGT.CovMatr.M[8][6] = l_GT_Data.KAxAz;

            l_BallInpDataGT.CovMatr.M[7][8] = l_GT_Data.KAyAz;
            l_BallInpDataGT.CovMatr.M[8][7] = l_GT_Data.KAyAz;

            l_BallInpDataGT.CovMatr.FCovMatrAdaptation(DIM_CVA, cMinRMSE, cCovCorr);

            l_BallInpDataGT.Cells_BMMark = &m_CellsBMMark;
            l_BallInpDataGT.p_arCovObj = &m_arCovObj;

            std::map<qint16, qint32>::iterator iter;
            for (iter = l_GT_Data.SetAssocST.begin(); iter != l_GT_Data.SetAssocST.end(); ++iter)
            {
                const qint32 ind_ST = GetIdST(l_GT_Data.tLoc, (*iter).first, (*iter).second);
                if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
                {
                    l_BallInpDataGT.SetST.insert(ind_ST);
                }
            }
        }
    }
    else if (InDataMode == AIR_BALL::ST_DATA)
    {
        l_pInDataST = static_cast<CtrlAIR_BALL_Input_ST_Data*>(pInData);
        l_pInDataProcST = static_cast<CAirBall_Proc_InputData_ST*>(pInDataProc);

        const qint32 IndST = GetIdST(l_pInDataST->tLoc, l_pInDataST->SrcInd, l_pInDataST->NtrInSrc);
        const qint32 NumGTInner = GetInnerTrackNum(l_pInDataST->NumGT);
        const qint32 IndGT = GetIdGT(l_pInDataST->tLoc, l_pInDataST->NumGT, NumGTInner);

        l_pInDataProcST->Reset();

        l_pInDataProcST->IF_ST   = IndST;
        l_pInDataProcST->IF_GT   = IndGT;

        const bool bBallist = IsBallistGT(AIR_BALL::ST_DATA, IndGT, l_pInDataST->Class);
        l_pInDataProcST->IF_BALL = GetIdBall(l_pInDataST->tLoc, IndGT, bBallist);

        l_pInDataProcST->SignQuickReaction = l_pInDataST->SignQuickReaction;
        l_pInDataProcST->Sign_large_load = (m_BO_quantity > AIR_BALL::BALL_LARGE_LOAD_AMOUNT ? true : false);
        l_pInDataProcST->bMeasured = l_pInDataST->bMeasured;
        l_pInDataProcST->bVDopplPresent = l_pInDataST->bVDopplPresent;

        l_pInDataProcST->tLoc    = l_pInDataST->tLoc;
        l_pInDataProcST->Coord.x = l_pInDataST->VectParam.Vec[0];
        l_pInDataProcST->Coord.y = l_pInDataST->VectParam.Vec[1];
        l_pInDataProcST->Coord.z = l_pInDataST->VectParam.Vec[2];
        l_pInDataProcST->Vel.x   = l_pInDataST->VectParam.Vec[3];
        l_pInDataProcST->Vel.y   = l_pInDataST->VectParam.Vec[4];
        l_pInDataProcST->Vel.z   = l_pInDataST->VectParam.Vec[5];
        l_pInDataProcST->Acc.x   = l_pInDataST->VectParam.Vec[6];
        l_pInDataProcST->Acc.y   = l_pInDataST->VectParam.Vec[7];
        l_pInDataProcST->Acc.z   = l_pInDataST->VectParam.Vec[8];
        l_pInDataProcST->CovMatr = l_pInDataST->CovMatr;

        l_pInDataProcST->Class   = l_pInDataST->Class;
        l_pInDataProcST->BallType= l_pInDataST->Type;

        l_pInDataProcST->Prob_IMM_small_coeff = l_pInDataST->Prob_KF_3D_222_small;
        l_pInDataProcST->Prob_IMM_large_coeff = l_pInDataST->Prob_KF_3D_222_large;
        l_pInDataProcST->R_rdr_obj = l_pInDataST->R;
        l_pInDataProcST->VDoppl  = l_pInDataST->VDoppl;
        l_pInDataProcST->N_StepSmooth = l_pInDataST->N_StepSmooth;
        l_pInDataProcST->Gamma   = l_pInDataST->GammaFlt;
        l_pInDataProcST->RMSE_R  = l_pInDataST->RMSE_R;
        l_pInDataProcST->RMSE_El = l_pInDataST->RMSE_El;
        l_pInDataProcST->RMSE_VDoppl = l_pInDataST->RMSE_VDoppl;

        if (0 <= IndST && IndST < AIR_BALL::ST_FORM_AMOUNT)
        {
            const qint16 path = m_p_Arr_BallProc->arData_ST[IndST].Path;
            LogTestData(m_pLogTest, IndST, l_pInDataST->tLoc, l_pInDataProcST->Class, path,
                        l_pInDataST->VectParam, l_pInDataST->CovMatr);
        }
    }
    else {
    }
}


void CtrlAIR_BALLIST_M::FormOutputData_GT(const qint32 ind_GT, const CAirBall_Proc_OutData& a_OutData_Ballist,
                                          CtrlAIR_BALL_Output_GT_Data& a_CAirBallOut)
{
    const CAirBall_Proc_OutData   *pBallOut = &a_OutData_Ballist;
    CAirBall_Proc_InnerData_GT    *pCurrEl  = &m_p_Arr_BallProc->arData_GT[ind_GT]; //pointer to the current element
                                                       //in the array of ballistic processing inner data

    if (pBallOut->bNewPrediction) {
        a_CAirBallOut.bNewPrediction = true;
    }

    if (pBallOut->bNewPred_InternalUse) {
        a_CAirBallOut.bNewPred_InternalUse = true;
    }

//    tLog(m_pLogProgn, "bNewPred: %d bNewPredInternal: %d", m_CAirBallOut.bNewPrediction, m_CAirBallOut.bNewPred_InternalUse);

    if (pBallOut->bNewPrediction || pBallOut->bNewPred_InternalUse)
    {
        bool bOutStartParent = false; //false - output start point for current track, true - output start point for parent track in the cluster

        if (pCurrEl->GTHasCluster()) //&& !m_p_Arr_BallProc->GTIsParentInCluster(ind_GT)) //current track is not parent in the cluster
        {
            qreal tStartClst = m_p_Arr_BallProc->GetStartTimeInCluster(ind_GT);
            GLPointDouble3D StartPointClst;
            EllipseInfo StartEllClst;
            m_p_Arr_BallProc->GetStartPointEllInCluster(ind_GT, StartPointClst, StartEllClst);

            if (fabs(tStartClst - AIR_BALL::Timer_INI) > con_eps
                    && !StartPointClst.IsZero()
                    && !StartEllClst.IsZero())
            {
                a_CAirBallOut.Predict.tStart        = tStartClst;
                a_CAirBallOut.Predict.StartPoint    = StartPointClst;
                a_CAirBallOut.Predict.StartEll      = StartEllClst;
                bOutStartParent = true;
            }
            else
            {
                qint32 ParentInd = m_p_Arr_BallProc->GetParentIndGTInCluster(ind_GT);
                if (0 <= ParentInd && ParentInd < AIR_BALL::GT_FORM_AMOUNT)
                {
                    CAirBall_Proc_InnerData_GT *pParentEl = &m_p_Arr_BallProc->arData_GT[ParentInd];
                    if (fabs(pParentEl->tStart - AIR_BALL::Timer_INI) > con_eps
                            && !pParentEl->StartPoint.IsZero())
                    {
                        a_CAirBallOut.Predict.tStart        = pParentEl->tStart;
                        a_CAirBallOut.Predict.StartPoint    = pParentEl->StartPoint;
                        a_CAirBallOut.Predict.StartEll      = pParentEl->StartEll;
                        bOutStartParent = true;
                    }
                }
            }
        }

        if (!bOutStartParent)
        {
            a_CAirBallOut.Predict.tStart        = pCurrEl->tStart;
            a_CAirBallOut.Predict.StartPoint    = pCurrEl->StartPoint;
            a_CAirBallOut.Predict.StartEll      = pCurrEl->StartEll;
        }

        a_CAirBallOut.Predict.tFall         = pCurrEl->tFall;
        a_CAirBallOut.Predict.FallPoint     = pCurrEl->FallPoint;
        a_CAirBallOut.Predict.FallEll       = pCurrEl->FallEll;
        a_CAirBallOut.Predict.Pol           = pBallOut->Polynoms;
        a_CAirBallOut.Predict.D             = pCurrEl->D;
        a_CAirBallOut.Predict.Hapogee       = pCurrEl->Hapogee;
        a_CAirBallOut.Predict.Gamma         = pCurrEl->Gamma;
        //m_CAirBallOut.Predict.Theta         = pCurrEl->ThrowAngle;
        a_CAirBallOut.Predict.arrPoints     = pBallOut->arrPoints;

        if (a_CAirBallOut.Predict.StartPoint.IsZero()
                && !a_CAirBallOut.Predict.FallPoint.IsZero())
        {
            a_CAirBallOut.Predict.tStart = pCurrEl->tFirst;
            a_CAirBallOut.Predict.StartPoint = pCurrEl->FirstPoint;
        }

        const Predict_Info *pPred = &a_CAirBallOut.Predict;

        tLog(m_pLogProgn, "%f | Gamma: %f | tStart %f Start point: %f %f %f | tFall %f Fall point: %f %f %f | Start ell.: %f %f %f | Fall ell.: %f %f %f | D %f Hapogee %f"
             " | 1st point: %f %f %f %f | 49th point: %f %f %f %f",
             pCurrEl->tLoc, pPred->Gamma, pPred->tStart, pPred->StartPoint.x, pPred->StartPoint.y, pPred->StartPoint.z,
             pPred->tFall, pPred->FallPoint.x, pPred->FallPoint.y, pPred->FallPoint.z,
             pPred->StartEll.aI, pPred->StartEll.bI, pPred->StartEll.BetaI,
             pPred->FallEll.aI, pPred->FallEll.bI, pPred->FallEll.BetaI,
             pPred->D, pPred->Hapogee,
             pPred->arrPoints[0].time_point, pPred->arrPoints[0].P.x, pPred->arrPoints[0].P.y, pPred->arrPoints[0].P.z,
             pPred->arrPoints[49].time_point, pPred->arrPoints[49].P.x, pPred->arrPoints[49].P.y, pPred->arrPoints[49].P.z);
    }

    if (pBallOut->bNewSP)
    {
        a_CAirBallOut.bNewSP = true;

        if (!pBallOut->bNewPrediction
                && (!pCurrEl->GTHasCluster())) //|| m_p_Arr_BallProc->GTIsParentInCluster(ind_GT)))
        {
            a_CAirBallOut.Predict.tStart        = pCurrEl->tStart;
            a_CAirBallOut.Predict.StartPoint    = pCurrEl->StartPoint;
            a_CAirBallOut.Predict.StartEll      = pCurrEl->StartEll;
//            a_CAirBallOut.Predict.Theta         = pCurrEl->ThrowAngle;
        }
    }

    if (pBallOut->bNewPathBranch)
    {
        a_CAirBallOut.bNewPathBranch = true;

        a_CAirBallOut.Path      = ConvertBallPath_AirBalltoGD(pCurrEl->Path);
        a_CAirBallOut.Branch    = ConvertBallBranch_AirBalltoGD(pCurrEl->Branch);
    }

    if (pBallOut->bNewActPred
            && pCurrEl->ActPredictInfo.New_ActPrognoz == AIR_BALL::NEW_ACT_PROGN)
    {
        a_CAirBallOut.bNewActPred = true;

        a_CAirBallOut.ActPred = pCurrEl->ActPredictInfo;

        char Name[100];
        sprintf(Name, "%f Prediction  on active path:", pCurrEl->tLoc);
        a_CAirBallOut.ActPred.Trapeze.LogData(m_pLogProgn, Name);
    }

    if (pBallOut->bNewTEAP)
    {
        a_CAirBallOut.bNewTEAP = true;

        a_CAirBallOut.tEAP = pCurrEl->tEAP;
        a_CAirBallOut.EAP_parameters = pCurrEl->EAP_parameters;
        a_CAirBallOut.EAP_Point = pCurrEl->EAP_Point;

        a_CAirBallOut.PointSepBoost1 = pCurrEl->PointSepBoost1;
        a_CAirBallOut.PointSepBoost2 = pCurrEl->PointSepBoost2;
    }

    if (pBallOut->bNewSubcl)
    {
        a_CAirBallOut.bNewSubcl = true;

        a_CAirBallOut.BallSubclass      = ConvertBallSubcl_AirBalltoGD(pCurrEl->BMMark.MarkValue);
        a_CAirBallOut.minBallSubclass   = ConvertBallSubcl_AirBalltoGD(pCurrEl->BMMark.minBMMark);
        a_CAirBallOut.maxBallSubclass   = ConvertBallSubcl_AirBalltoGD(pCurrEl->BMMark.maxBMMark);
        a_CAirBallOut.bSubclIsSingleValued = pCurrEl->BMMark.IsSingleValued;
        a_CAirBallOut.FlightTimeMax = GetMaxFlightTime(pCurrEl->BMMark.maxBMMark);
    }

    if (pBallOut->bNewTrajType)
    {
        a_CAirBallOut.bNewTrajType = true;

        a_CAirBallOut.TrajType = ConvertBallTType_AirBalltoGD(pCurrEl->TrajType);
    }

    if (pBallOut->bNewABTrapeze)
    {
        a_CAirBallOut.bNewABData = true;

        a_CAirBallOut.AB_Trapeze = pCurrEl->AB_Trapeze;
    }
}


qint16 CtrlAIR_BALLIST_M::ConvertBallSubcl_GDtoAirBall(const qint16 BallSubclGD)
{
    qint16 returnVariable;
    switch (BallSubclGD)
    {
//    case TYPE_BM60:
//        returnVariable = AIR_BALL::BM60;
//        break;
//    case SUBCLASS_BM100:
//        returnVariable = AIR_BALL::BM100;
//        break;
//    case SUBCLASS_BM200:
//        returnVariable = AIR_BALL::BM200;
//        break;
    case TYPE_BM300:
        returnVariable = AIR_BALL::BM300;
        break;
    case TYPE_BM600:
        returnVariable = AIR_BALL::BM600;
        break;
    case TYPE_BM750:
        returnVariable = AIR_BALL::BM750;
        break;
    case TYPE_BM1000:
        returnVariable = AIR_BALL::BM1000;
        break;
    case TYPE_BM1500:
        returnVariable = AIR_BALL::BM1500;
        break;
    case TYPE_BM1800:
        returnVariable = AIR_BALL::BM1800;
        break;
    case TYPE_BM2000:
        returnVariable = AIR_BALL::BM2000;
        break;
    case TYPE_BM2500:
        returnVariable = AIR_BALL::BM2500;
        break;
    case TYPE_BM2800:
        returnVariable = AIR_BALL::BM2800;
        break;
    case TYPE_BM3000:
        returnVariable = AIR_BALL::BM3000;
        break;
    case TYPE_BM3500:
        returnVariable = AIR_BALL::BM3500;
        break;
//    case SUBCLASS_BM5000:
//        returnVariable = AIR_BALL::BM5000;
//        break;
//    case SUBCLASS_BM6000:
//        returnVariable = AIR_BALL::BM6000;
//        break;
//    case SUBCLASS_BM7000:
//        returnVariable = AIR_BALL::BM7000;
//        break;
//    case SUBCLASS_BM8000:
//        returnVariable = AIR_BALL::BM8000;
//        break;
//    case SUBCLASS_BM9000:
//        returnVariable = AIR_BALL::BM9000;
//        break;
//    case SUBCLASS_BM10000:
//        returnVariable = AIR_BALL::BM10000;
//        break;
//    case SUBCLASS_BM11000:
//        returnVariable = AIR_BALL::BM11000;
//        break;
//    case SUBCLASS_BM12000:
//        returnVariable = AIR_BALL::BM12000;
//        break;
//    case SUBCLASS_BM_SHORT_RANGE:
//        returnVariable = AIR_BALL::UNKNOWN_OF_SHORT_RANGE;
//        break;
//    case SUBCLASS_BM_LONG_RANGE:
//        returnVariable = AIR_BALL::UNKNOWN_OF_LONG_RANGE;
//        break;
    default:
        returnVariable = AIR_BALL::UNDEFINED_BM_MARK;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallSubcl_AirBalltoGD(const qint16 BallMarkAirBall)
{
    qint16 returnVariable;
    switch (BallMarkAirBall)
    {
//    case AIR_BALL::BM60:
//        returnVariable = SUBCLASS_BM60;
//        break;
//    case AIR_BALL::BM100:
//        returnVariable = SUBCLASS_BM100;
//        break;
//    case AIR_BALL::BM200:
//        returnVariable = SUBCLASS_BM200;
//        break;
    case AIR_BALL::BM300:
        returnVariable = TYPE_BM300;
        break;
    case AIR_BALL::BM600:
        returnVariable = TYPE_BM600;
        break;
    case AIR_BALL::BM750:
        returnVariable = TYPE_BM750;
        break;
    case AIR_BALL::BM1000:
        returnVariable = TYPE_BM1000;
        break;
    case AIR_BALL::BM1500:
        returnVariable = TYPE_BM1500;
        break;
    case AIR_BALL::BM1800:
        returnVariable = TYPE_BM1800;
        break;
    case AIR_BALL::BM2000:
        returnVariable = TYPE_BM2000;
        break;
    case AIR_BALL::BM2500:
        returnVariable = TYPE_BM2500;
        break;
    case AIR_BALL::BM2800:
        returnVariable = TYPE_BM2800;
        break;
    case AIR_BALL::BM3000:
        returnVariable = TYPE_BM3000;
        break;
    case AIR_BALL::BM3500:
        returnVariable = TYPE_BM3500;
        break;
//    case AIR_BALL::BM5000:
//        returnVariable = SUBCLASS_BM5000;
//        break;
//    case AIR_BALL::BM6000:
//        returnVariable = SUBCLASS_BM6000;
//        break;
//    case AIR_BALL::BM7000:
//        returnVariable = SUBCLASS_BM7000;
//        break;
//    case AIR_BALL::BM8000:
//        returnVariable = SUBCLASS_BM8000;
//        break;
//    case AIR_BALL::BM9000:
//        returnVariable = SUBCLASS_BM9000;
//        break;
//    case AIR_BALL::BM10000:
//        returnVariable = SUBCLASS_BM10000;
//        break;
//    case AIR_BALL::BM11000:
//        returnVariable = SUBCLASS_BM11000;
//        break;
//    case AIR_BALL::BM12000:
//        returnVariable = SUBCLASS_BM12000;
//        break;
//    case AIR_BALL::UNKNOWN_OF_SHORT_RANGE:
//        returnVariable = SUBCLASS_BM_SHORT_RANGE;
//        break;
//    case AIR_BALL::UNKNOWN_OF_LONG_RANGE:
//        returnVariable = SUBCLASS_BM_LONG_RANGE;
//        break;
    default:
        returnVariable = SUBCLASS_UNKNOWN;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallPath_GDtoAirBall(const qint16 BallPathGD)
{
    qint16 returnVariable;
    switch(BallPathGD)
    {
    case NONE:
        returnVariable = AIR_BALL::UNDEFINED_PATH;
        break;
    case ACTIVE:
        returnVariable = AIR_BALL::ACTIVE_PATH;
        break;
    case PASSIVE:
        returnVariable = AIR_BALL::PASSIVE_PATH;
        break;
    case END_ACTIVE_PATH:
        returnVariable = AIR_BALL::END_OF_ACTIVE_PATH;
        break;
    default:
        returnVariable = AIR_BALL::UNDEFINED_PATH;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallPath_AirBalltoGD(const qint16 BallPathAirBall)
{
    qint16 returnVariable;
    switch(BallPathAirBall)
    {
    case AIR_BALL::UNDEFINED_PATH:
        returnVariable = NONE;
        break;
    case AIR_BALL::ACTIVE_PATH:
        returnVariable = ACTIVE;
        break;
    case AIR_BALL::PASSIVE_PATH:
        returnVariable = PASSIVE;
        break;
    case AIR_BALL::END_OF_ACTIVE_PATH:
        returnVariable = END_ACTIVE_PATH;
        break;
    default:
        returnVariable = NONE;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallBranch_GDtoAirBall(const qint16 BallBranchGD)
{
    qint16 returnVariable;
    switch (BallBranchGD)
    {
    case BRANCH_UNKNOWN:
        returnVariable = AIR_BALL::UNDEFINED_BRANCH;
        break;
    case ASCENDING:
        returnVariable = AIR_BALL::ASCENDING_BRANCH;
        break;
    case DESCENDING:
        returnVariable = AIR_BALL::DESCENDING_BRANCH;
        break;
    default:
        returnVariable = AIR_BALL::UNDEFINED_BRANCH;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallBranch_AirBalltoGD(const qint16 BallBranchAirBall)
{
    qint16 returnVariable;
    switch (BallBranchAirBall)
    {
    case AIR_BALL::UNDEFINED_BRANCH:
        returnVariable = BRANCH_UNKNOWN;
        break;
    case AIR_BALL::ASCENDING_BRANCH:
        returnVariable = ASCENDING;
        break;
    case AIR_BALL::DESCENDING_BRANCH:
        returnVariable = DESCENDING;
        break;
    case AIR_BALL::TRANSITION_BRANCHES:
        returnVariable = BRANCH_UNKNOWN;
        break;
    default:
        returnVariable = BRANCH_UNKNOWN;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallTType_GDtoAirBall(const qint16 BallTTypeGD)
{
    qint16 returnVariable;
    switch (BallTTypeGD)
    {
    case UNDEFINED_TRAJ:
        returnVariable = AIR_BALL::UNDEFINED_TRAJ_TYPE;
        break;
    case OPTIMAL_TRAJ:
        returnVariable = AIR_BALL::OPTIMAL;
        break;
    case LOFTED_TRAJ:
        returnVariable = AIR_BALL::LOFTED;
        break;
    case FLAT_TRAJ:
        returnVariable = AIR_BALL::FLAT;
        break;
    default:
        returnVariable = AIR_BALL::UNDEFINED_TRAJ_TYPE;
        break;
    }
    return returnVariable;
}


qint16 CtrlAIR_BALLIST_M::ConvertBallTType_AirBalltoGD(const qint16 BallTTypeAirBall)
{
    qint16 returnVariable;
    switch (BallTTypeAirBall)
    {
    case AIR_BALL::UNDEFINED_TRAJ_TYPE:
        returnVariable = UNDEFINED_TRAJ;
        break;
    case AIR_BALL::OPTIMAL:
        returnVariable = OPTIMAL_TRAJ;
        break;
    case AIR_BALL::LOFTED:
        returnVariable = LOFTED_TRAJ;
        break;
    case AIR_BALL::FLAT:
        returnVariable = FLAT_TRAJ;
        break;
    default:
        returnVariable = UNDEFINED_TRAJ;
        break;
    }
    return returnVariable;
}


qreal CtrlAIR_BALLIST_M::GetMaxFlightTime(const qint16 BallMarkAirBall)
{
    qreal returnVariable;
    switch (BallMarkAirBall)
    {
    case AIR_BALL::BM60:
        returnVariable = cTFlightMaxBM60;
        break;
    case AIR_BALL::BM100:
        returnVariable = cTFlightMaxBM100;
        break;
    case AIR_BALL::BM200:
        returnVariable = cTFlightMaxBM200;
        break;
    case AIR_BALL::BM300:
        returnVariable = cTFlightMaxBM300;
        break;
    case AIR_BALL::BM600:
        returnVariable = cTFlightMaxBM600;
        break;
    case AIR_BALL::BM750:
        returnVariable = cTFlightMaxBM750;
        break;
    case AIR_BALL::BM1000:
        returnVariable = cTFlightMaxBM1000;
        break;
    case AIR_BALL::BM1500:
        returnVariable = cTFlightMaxBM1500;
        break;
    case AIR_BALL::BM1800:
        returnVariable = cTFlightMaxBM1800;
        break;
    case AIR_BALL::BM2000:
        returnVariable = cTFlightMaxBM2000;
        break;
    case AIR_BALL::BM2500:
        returnVariable = cTFlightMaxBM2500;
        break;
    case AIR_BALL::BM2800:
        returnVariable = cTFlightMaxBM2800;
        break;
    case AIR_BALL::BM3000:
        returnVariable = cTFlightMaxBM3000;
        break;
    case AIR_BALL::BM3500:
        returnVariable = cTFlightMaxBM4000;
        break;
    case AIR_BALL::BM5000:
        returnVariable = cTFlightMaxBM5000;
        break;
    case AIR_BALL::BM6000:
        returnVariable = cTFlightMaxBM6000;
        break;
    case AIR_BALL::BM7000:
        returnVariable = cTFlightMaxBM7000;
        break;
    case AIR_BALL::BM8000:
        returnVariable = cTFlightMaxBM8000;
        break;
    case AIR_BALL::BM9000:
        returnVariable = cTFlightMaxBM9000;
        break;
    case AIR_BALL::BM10000:
        returnVariable = cTFlightMaxBM10000;
        break;
    case AIR_BALL::BM11000:
        returnVariable = cTFlightMaxBM11000;
        break;
    case AIR_BALL::BM12000:
        returnVariable = cTFlightMaxBM12000;
        break;
    default:
        returnVariable = 0;
        break;
    }
    return returnVariable;
}


void CtrlAIR_BALLIST_M::ReadCellsBMMark()
{
    qint32 meaning[AIR_BALL::NUMB_CELLS];
    memset(meaning, FP_NAN, sizeof(meaning));
    qint16 ind = 0;

    QString l_path(QCoreApplication::applicationDirPath());
//    QString l_name(QCoreApplication::applicationName());

//    l_path = QString("%1/../etc/%2").arg(l_path).arg(l_name);
//    l_path = QDir::cleanPath(l_path);
//    l_path = QString("%1/%2_%3").arg(l_path).arg(l_name).arg(AIR_BALL::hhe_ini_file);

    l_path = QString("%1/../data/input/%2").arg(l_path).arg(AIR_BALL::hhe_ini_file);

    AIR_BALL::cells_ini.open(qPrintable(l_path));
    if( (AIR_BALL::cells_ini.rdstate() & std::ifstream::failbit ) != 0 ) {
        AIR_BALL::cells_ini.close();
        return;
    }

    while (!AIR_BALL::cells_ini.eof())
    {
        AIR_BALL::cells_ini >> meaning[ind];
        ind ++;
    }

    for (ind=0; ind<AIR_BALL::NUMB_CELLS; ind++)
    {
        if (isfinite(meaning[ind]))
        {
            m_CellsBMMark.ArrCells[ind] = meaning[ind];
        }
    }

    AIR_BALL::cells_ini.close();
}


bool CtrlAIR_BALLIST_M::IsBallistGT(const qint16 InDataMode, const qint32 Ind_GT, const qint8 InClass)
{
    bool Result = false;
    if (Ind_GT >= 0 && Ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        if ( (InDataMode == AIR_BALL::GT_DATA) && (InClass == CLASS_BALLISTIC))
        {
            Result = true;
        }
//        else if ((InDataMode == AIR_BALL::TEST_DATA) && (m_TrajParam.Class == CLASS_BALLISTIC))
//        {
//            Result = true;
//        }
        else
        {
            if (m_arDataCtrl[Ind_GT].Class == CLASS_BALLISTIC)
            {
                Result = true;
            }
        }
    }

    return Result;
}


qint32 CtrlAIR_BALLIST_M::GetIdGT(const qreal tLoc, const qint32 NumGTExternal, const qint32 NumGTInner)
{
    qint32 id = -1;
    if (NumGTInner > 0 && NumGTInner < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGT);

        if (GT_IDs[NumGTInner] != nullptr) {
            id = *GT_IDs[NumGTInner];
        }
        else
        {
            for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++)
            {
                if (!ID_Values_GT[i].SignBusy)
                {
                    ID_Values_GT[i].SignBusy = true;
                    GT_IDs[NumGTInner] = &ID_Values_GT[i].Index;
                    id = *GT_IDs[NumGTInner];
                    m_CAirBallProc.SetNumbGT(id, NumGTInner); //set number of GT in the inner array of m_CAirBallProc
                    tLog(m_pLogIDs, " New ID GT %d NumGT %d NumGTInner %d", id, NumGTExternal, NumGTInner);
                    break;
                }
            }
        }
    }

    if (id < 0)
    {
        tLog(m_pLogIDs, "~~~ Wrong ID GT ~~~ NumGT %d NumGTInner %d tLoc %f", NumGTExternal, NumGTInner, tLoc);
        //assert(0);
    }
    else
    {
        //tLog(m_pLogIDs, "ID GT %d NumGT %d tLoc %f", id, NumGTInner, tLoc);
    }
    return id;
}


qint32 CtrlAIR_BALLIST_M::GetInnerTrackNum(qint32 NumExt)
{
    QMutexLocker l_lock(&m_mutexGT);
    qint32 NumInner = m_NumGTEnumeration.GetInnerTrackNumber(NumExt);
    return NumInner;
}


void CtrlAIR_BALLIST_M::ClearIdGT(const qreal tLoc, const qint32 NumGTExt, const qint32 NumGTInner)
{
    if (0 < NumGTInner && NumGTInner < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGT);
        if (GT_IDs[NumGTInner] != nullptr)
        {
            const qint32 id = *GT_IDs[NumGTInner];
            if (0 <= id && id < AIR_BALL::GT_FORM_AMOUNT)
            {
                tLog(m_pLogIDs, " Clear ID GT %d NumGT %d NumGTInner %d tLoc %f", id, NumGTExt, NumGTInner, tLoc);
                ID_Values_GT[id].SignBusy = false;
            }
            GT_IDs[NumGTInner] = nullptr;
        }
    }
}


qint32 CtrlAIR_BALLIST_M::GetIdST(const qreal tLoc, const qint16 SrcInd, const qint32 NtrInSrc)
{
    qint32 id = -1;

    if (SrcInd >= 0 && SrcInd < AIR_BALL::SRC_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexST);

        qint32 InnerNtrSrc = m_arSrcInnEnumeration[SrcInd].GetInnerTrackNumber(NtrInSrc);
        if (InnerNtrSrc >= 0 && InnerNtrSrc < AIR_BALL::ST_SRC_AMOUNT)
        {
            if (ST_IDs[SrcInd][InnerNtrSrc] != nullptr)
            {
                id = *ST_IDs[SrcInd][InnerNtrSrc];
            }
            else
            {
                for (qint32 i=0; i<AIR_BALL::ST_FORM_AMOUNT; i++)
                {
                    if (!ID_Values_ST[i].SignBusy)
                    {
                        ID_Values_ST[i].SignBusy = true;
                        ST_IDs[SrcInd][InnerNtrSrc] = &ID_Values_ST[i].Index;
                        id = ID_Values_ST[i].Index;
                        tLog(m_pLogIDs, " New ID ST %d SrcInd %d NtrInSrc %d InnerNtrSrc %d",
                             id, SrcInd, NtrInSrc, InnerNtrSrc);
                        break;
                    }
                }
            }
        }
    }
    if (id < 0)
    {
        tLog(m_pLogIDs, "~~~ Wrong ID ST ~~~ SrcInd %d NtrInSrc %d tLoc %f", SrcInd, NtrInSrc, tLoc);
        //assert(0);
    }
//    else
//    {
//        tLog(m_pLogIDs, "ID ST %d SrcInd %d NtrInSrc %d tLoc %f", id, SrcInd, InnerNtrSrc, tLoc);
//    }

    return id;
}


void CtrlAIR_BALLIST_M::ClearIdST(const qreal tLoc, const qint16 SrcInd, const qint32 NtrInSrc)
{
    if (0 <= SrcInd && SrcInd < AIR_BALL::SRC_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexST);

        qint32 InnerNtrSrc = m_arSrcInnEnumeration[SrcInd].GetInnerTrackNumber(NtrInSrc);
        if (0 <= InnerNtrSrc && InnerNtrSrc < AIR_BALL::ST_SRC_AMOUNT)
        {
            if (ST_IDs[SrcInd][InnerNtrSrc] != nullptr)
            {
                const qint32 id = *ST_IDs[SrcInd][InnerNtrSrc];
                if (0 <= id && id < AIR_BALL::ST_FORM_AMOUNT)
                {
                    tLog(m_pLogIDs, " Clear ID ST %d SrcInd %d NtrInSrc %d InnerNtrSrc %d tLoc %f",
                         id, SrcInd, NtrInSrc, InnerNtrSrc, tLoc);
                    ID_Values_ST[id].SignBusy = false;
                }
                ST_IDs[SrcInd][InnerNtrSrc] = nullptr;
            }
        }
        m_arSrcInnEnumeration[SrcInd].DeleteTrack(NtrInSrc);
    }
}


qint32 CtrlAIR_BALLIST_M::GetIdBall(const qreal tLoc, const qint32 Ind_GT, const bool bBallist)
{
    qint32 id = -1;
    if (Ind_GT >= 0 && Ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGT);
        if (Ball_IDs_forGT[Ind_GT] != nullptr)
        {
            id = *Ball_IDs_forGT[Ind_GT];
        }
        else if (bBallist)
        {
            for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
            {
                if (!ID_Values_Ball[i].SignBusy)
                {
                    ID_Values_Ball[i].SignBusy = true;
                    Ball_IDs_forGT[Ind_GT] = &ID_Values_Ball[i].Index;
                    id = *Ball_IDs_forGT[Ind_GT];
                    m_BO_quantity ++;
                    tLog(m_pLogIDs, " New ID Ball %d IndGT %d tLoc %f", id, Ind_GT, tLoc);
                    break;
                }
            }
        }
        else {
        }
    }
    if (id < 0)
    {
        tLog(m_pLogIDs, "~~~ Wrong ID Ball ~~~ IndGT %d tLoc %f", Ind_GT, tLoc);
        //assert(0);
    }
    else {
        //tLog(m_pLogIDs, "ID Ball %d IndGT %d tLoc %f", id, Ind_GT, tLoc);
    }

    return id;
}


void CtrlAIR_BALLIST_M::ClearIdBall(const qreal tLoc, const qint32 Ind_GT)
{
    if (0 <= Ind_GT && Ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        QMutexLocker l_lock(&m_mutexGT);
        if (Ball_IDs_forGT[Ind_GT] != nullptr)
        {
            const qint32 id = *Ball_IDs_forGT[Ind_GT];
            if (0 <= id && id < AIR_BALL::BALL_FORM_AMOUNT)
            {
                tLog(m_pLogIDs, " Clear ID Ball %d IndGT %d tLoc %f", id, Ind_GT, tLoc);
                ID_Values_Ball[id].SignBusy = false;
                m_BO_quantity --;
            }
            Ball_IDs_forGT[Ind_GT] = nullptr;
        }
    }
}


void CtrlAIR_BALLIST_M::Drop_GT(const CtrlAIR_BALL_Input_GT_Data* pInData)
{
    const qint32 NumGTInner = GetInnerTrackNum(pInData->NumbGT);
    const qint32 ind_GT = GetIdGT(pInData->tLoc, pInData->NumbGT, NumGTInner);
    const qint32 ind_Ball = GetIdBall(pInData->tLoc, ind_GT, false);

    if (ind_GT >= 0 && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        m_CAirBallProc.Drop_GT(ind_GT, ind_Ball);

        ClearInnerData_GT_1Track(ind_GT);
        ClearIdBall(pInData->tLoc, ind_GT);
        ClearIdGT(pInData->tLoc, pInData->NumbGT, NumGTInner);
    }
    QMutexLocker l_lock(&m_mutexGT);
    m_NumGTEnumeration.DeleteTrack(pInData->NumbGT);
}


void CtrlAIR_BALLIST_M::Drop_ST(const CtrlAIR_BALL_Input_ST_Data* pInData)
{
    const qint32 ind_ST = GetIdST(pInData->tLoc, pInData->SrcInd, pInData->NtrInSrc);
    if (0 <= ind_ST && ind_ST < AIR_BALL::ST_FORM_AMOUNT)
    {
        m_CAirBallProc.Drop_ST(ind_ST);
        ClearIdST(pInData->tLoc, pInData->SrcInd, pInData->NtrInSrc);
    }
}


void CtrlAIR_BALLIST_M::ClearBallisticData(const qreal tLoc, const qint32 ind_GT, const qint32 ind_Ball)
{
    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_CAirBallProc.ClearBallisticData(ind_Ball);
        ClearIdBall(tLoc, ind_GT);
    }
}


void CtrlAIR_BALLIST_M::ClearInnerData_GT()
{
    for (qint32 i=0; i<AIR_BALL::GT_FORM_AMOUNT; i++)
    {
        ClearInnerData_GT_1Track(i);
    }
}


void CtrlAIR_BALLIST_M::ClearInnerData_GT_1Track(const qint32 ind_GT)
{
    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT)
    {
        m_arDataCtrl[ind_GT].Reset();
    }
}

