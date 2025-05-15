#include "CAirBallist_Processing_structs.h"

CAirBall_Proc_InputData_GT::CAirBall_Proc_InputData_GT()
{
//    Reset();
    CovMatr.Reset();
}


void CAirBall_Proc_InputData_GT::Reset()
{
    tLoc = 0;
    IF_GT = -1;
    NumbGT = -1;
    IF_BALL = -1;
    SetST.clear();
    p_arCovObj = nullptr;
    Coord.clear();
    Vel.clear();
    Acc.clear();
    SignQuickReaction = false;
    Sign_large_load = false;
    bAeroballistic = false;
    CovMatr.Reset();
    Class = 0;
    BallType = 0;
    ClstNum = 0;
    Cells_BMMark = nullptr;
}


CAirBall_Proc_InputData_GT& CAirBall_Proc_InputData_GT::operator =(const CAirBall_Proc_InputData_GT &Q)
{
    this->tLoc      = Q.tLoc;
    this->IF_GT     = Q.IF_GT;
    this->NumbGT    = Q.NumbGT;
    this->IF_BALL   = Q.IF_BALL;

    this->SetST.clear();
    for (std::set<qint32>::iterator it = Q.SetST.begin(); it != Q.SetST.end(); it++)
    {
        this->SetST.insert((*it));
    }
//    this->SetST     = Q.SetST;
    this->p_arCovObj = Q.p_arCovObj;

    this->Coord     = Q.Coord;
    this->Vel       = Q.Vel;
    this->Acc       = Q.Acc;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->Sign_large_load   = Q.Sign_large_load;
    this->bAeroballistic    = Q.bAeroballistic;
    this->CovMatr   = Q.CovMatr;
    this->ClstNum   = Q.ClstNum;
    this->Class     = Q.Class;
    this->BallType  = Q.BallType;
    this->Cells_BMMark      = Q.Cells_BMMark;
    return *this;
}

CAirBall_Proc_InputData_GT::CAirBall_Proc_InputData_GT (const CAirBall_Proc_InputData_GT &Q)
{
    this->tLoc      = Q.tLoc;
    this->IF_GT     = Q.IF_GT;
    this->NumbGT    = Q.NumbGT;
    this->IF_BALL   = Q.IF_BALL;

    this->SetST.clear();
    for (std::set<qint32>::iterator it = Q.SetST.begin(); it != Q.SetST.end(); it++)
    {
        this->SetST.insert((*it));
    }
    this->p_arCovObj = Q.p_arCovObj;

    this->Coord     = Q.Coord;
    this->Vel       = Q.Vel;
    this->Acc       = Q.Acc;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->Sign_large_load   = Q.Sign_large_load;
    this->bAeroballistic    = Q.bAeroballistic;
    this->CovMatr   = Q.CovMatr;
    this->Class     = Q.Class;
    this->BallType  = Q.BallType;
    this->ClstNum   = Q.ClstNum;
    this->Cells_BMMark      = Q.Cells_BMMark;
}


CAirBall_Proc_InputData_ST::CAirBall_Proc_InputData_ST()
{
//    Reset();
    CovMatr.Reset();
}


void CAirBall_Proc_InputData_ST::Reset()
{
    tLoc = 0;
    IF_ST = -1;
    IF_GT = -1;
    IF_BALL = -1;
    bMeasured = false;
    bVDopplPresent = false;
    Coord.clear();
    Vel.clear();
    Acc.clear();
    SignQuickReaction = false;
    Sign_large_load = false;
    CovMatr.Reset();
    Class = 0;
    BallType = 0;
    N_StepSmooth = 0;
    Prob_IMM_small_coeff = 0;
    Prob_IMM_large_coeff = 0;
    R_rdr_obj = 0;
    VDoppl = 0;
    Gamma = 0;
    RMSE_R = 0;
    RMSE_VDoppl = 0;
    RMSE_El = 0;
}


CAirBall_Proc_InputData_ST& CAirBall_Proc_InputData_ST::operator=(const CAirBall_Proc_InputData_ST &Q)
{
    this->tLoc      = Q.tLoc;
    this->IF_ST     = Q.IF_ST;
    this->IF_GT     = Q.IF_GT;
    this->IF_BALL   = Q.IF_BALL;
    this->bMeasured = Q.bMeasured;
    this->bVDopplPresent    = Q.bVDopplPresent;
    this->Coord     = Q.Coord;
    this->Vel       = Q.Vel;
    this->Acc       = Q.Acc;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->Sign_large_load   = Q.Sign_large_load;
    this->CovMatr   = Q.CovMatr;
    this->Class     = Q.Class;
    this->BallType  = Q.BallType;
    this->Prob_IMM_small_coeff  = Q.Prob_IMM_small_coeff;
    this->Prob_IMM_large_coeff  = Q.Prob_IMM_large_coeff;
    this->R_rdr_obj = Q.R_rdr_obj;
    this->VDoppl    = Q.VDoppl;
    this->Gamma     = Q.Gamma;
    this->RMSE_R    = Q.RMSE_R;
    this->RMSE_VDoppl   = Q.RMSE_VDoppl;
    this->RMSE_El   = Q.RMSE_El;
    return *this;
}

CAirBall_Proc_InputData_ST::CAirBall_Proc_InputData_ST (const CAirBall_Proc_InputData_ST &Q)
{
    this->tLoc      = Q.tLoc;
    this->IF_ST     = Q.IF_ST;
    this->IF_GT     = Q.IF_GT;
    this->IF_BALL   = Q.IF_BALL;
    this->bMeasured = Q.bMeasured;
    this->bVDopplPresent    = Q.bVDopplPresent;
    this->Coord     = Q.Coord;
    this->Vel       = Q.Vel;
    this->Acc       = Q.Acc;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->Sign_large_load   = Q.Sign_large_load;
    this->CovMatr   = Q.CovMatr;
    this->Class     = Q.Class;
    this->BallType  = Q.BallType;
    this->Prob_IMM_small_coeff  = Q.Prob_IMM_small_coeff;
    this->Prob_IMM_large_coeff  = Q.Prob_IMM_large_coeff;
    this->R_rdr_obj = Q.R_rdr_obj;
    this->VDoppl    = Q.VDoppl;
    this->Gamma     = Q.Gamma;
    this->RMSE_R    = Q.RMSE_R;
    this->RMSE_VDoppl   = Q.RMSE_VDoppl;
    this->RMSE_El   = Q.RMSE_El;
}


CAirBall_Proc_OutData::CAirBall_Proc_OutData()
{    
//    Reset();
}


void CAirBall_Proc_OutData::Reset()
{
    bNewPrediction          = false;
    bNewPred_InternalUse    = false;
    bNewSP                  = false;
    bNewPathBranch          = false;
    bNewActPred             = false;
    bNewTEAP                = false;
    bNewSubcl               = false;
    bNewTrajType            = false;
    bNewABTrapeze           = false;

    arrPoints.Reset();
    Polynoms.Reset();
}


BMMark_AndDiapason::BMMark_AndDiapason()
{
//    Reset();
}


void BMMark_AndDiapason::Reset()
{
    memset(this, 0, sizeof(BMMark_AndDiapason));
}


BMMark_AndDiapason& BMMark_AndDiapason::operator=(const BMMark_AndDiapason &Q)
{
    this->MarkValue         = Q.MarkValue;      //value of BM mark, see enum BM_Marks
    this->minBMMark         = Q.minBMMark;      //minimal value of BM mark, possible for current BM
    this->maxBMMark         = Q.maxBMMark;      //maximal value of BM mark, possible for current BM
    this->IsSingleValued    = Q.IsSingleValued; //true if BM mark is defined uniquely

    return *this;
}


bool BMMark_AndDiapason::IsEmpty()
{
    bool bRes = false; //true if structure is empty
    if (MarkValue == 0 && minBMMark == 0 && maxBMMark == 0)
    {
        bRes = true;
    }
    return bRes;
}


T_U_SU::T_U_SU()
{
//    Reset();
}


T_U_SU::T_U_SU(qreal _Time, qreal _Par, qreal _SigPar)
{
    this->t     = _Time;
    this->U     = _Par;
    this->SigU  = _SigPar;
}


void T_U_SU::Reset()
{
    t       = AIR_BALL::Timer_INI;
    U       = 0;
    SigU    = 0;
}


T_U_SU& T_U_SU::operator=(const T_U_SU &Q)
{
    this->t     = Q.t;
    this->U     = Q.U;
    this->SigU  = Q.SigU;
    return *this;
}


bool T_U_SU::IsZero()
{
    bool bZero = false;
    if (fabs(t - AIR_BALL::Timer_INI) < con_par_eps
            || fabs(U) < con_par_eps
            || fabs(SigU) < con_par_eps)
    {
        bZero = true;
    }
    return bZero;
}


CAirBall_Proc_InnerData_GT::CAirBall_Proc_InnerData_GT()
{
//    Reset();
}


void CAirBall_Proc_InnerData_GT::Reset()
{
    IsTaken = false;
    Path = 0;
    Branch = 0;
    NumbGT = -1;
    ClstNum = 0;
    IndBall = -1;
    tLoc = 0;
    BallType = 0;
    tEAP = AIR_BALL::Timer_INI;
    T_EAP_finding_method = AIR_BALL::UNDEFINED_TEAP_METHOD;
    tPrevPathDet = AIR_BALL::Timer_INI;
    EAP_parameters.Reset();
    EAP_Point.clear();
    PointSepBoost1.clear();
    PointSepBoost2.clear();
    tLastWellSmoothed = AIR_BALL::Timer_INI;
    Vel.clear();
    VelPrev.clear();
    bAeroballistic = false;
    t0_Aeroballistic = AIR_BALL::Timer_INI;
    BMMark.Reset();
    TrajType = 0;
    StartPoint.clear();
    FallPoint.clear();
    StartEll.Reset();
    FallEll.Reset();
    tStart = AIR_BALL::Timer_INI;
    tFall = AIR_BALL::Timer_INI;
    D = 0;
//    ThrowAngle = 0;
    Hapogee = 0;
    tApogee = AIR_BALL::Timer_INI;
//    Veap = 0;
    Gamma = 0;
    tFirst = 0;
    FirstPoint.clear();
    RMSEs_FirstPoint.clear();
    ActPredictInfo.Reset();
    t_calc_act = 0;
    AB_Trapeze.Reset();
    CorrectSigAzPrev = 0;
    tPredictSatellite = AIR_BALL::Timer_INI;
    PrevTLoc = AIR_BALL::Timer_INI;
    DirPrev.Reset();
    DirPrev_1.Reset();
    DirPrev_2.Reset();
    ManTime = AIR_BALL::Timer_INI;
    ManPoint.clear();
}


bool CAirBall_Proc_InnerData_GT::GTHasCluster()
{
    bool bResult = false;
    if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
    {
        bResult = true;
    }
    return bResult;
}


CAirBall_Proc_InnerData_ST::CAirBall_Proc_InnerData_ST()
{
//    Reset();
}


void CAirBall_Proc_InnerData_ST::Reset()
{
    memset(this, 0, sizeof(CAirBall_Proc_InnerData_ST));
}


CAirBall_Proc_ClstGTData::CAirBall_Proc_ClstGTData()
{
//    Reset();
}


void CAirBall_Proc_ClstGTData::Reset()
{
    bBusy = false;
    ParentIndGT = -1;
    FstWHInd = -1;
    SetGTInd.clear();
    tStart = AIR_BALL::Timer_INI;
    StartPoint.clear();
    StartEll.Reset();
    bParentDropped = false;
    bArrPointsFilled = false;
    std::vector<TracksPoints>().swap(InitialArrPoints);
}


CAirBall_Proc_DataForGTNumber::CAirBall_Proc_DataForGTNumber()
{
//    Reset();
}


void CAirBall_Proc_DataForGTNumber::Reset()
{
    memset(this, 0, sizeof(CAirBall_Proc_DataForGTNumber));
}


CAirBall_Proc_Inner_Arr::CAirBall_Proc_Inner_Arr()
{
    //Reset();
    arData_GT.resize(AIR_BALL::GT_FORM_AMOUNT);
    arData_ST.resize(AIR_BALL::ST_FORM_AMOUNT);
    arData__NumbGT.resize(AIR_BALL::GT_FORM_AMOUNT);
}


void CAirBall_Proc_Inner_Arr::Reset()
{
    arData_GT.Reset();
    arData_ST.Reset();
    arData__Clst.Reset();
    arData__NumbGT.Reset();
}


bool CAirBall_Proc_Inner_Arr::GTIsParentInCluster(qint32 indGT)
{
    bool bParent = false;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        qint32 ClstNum = arData_GT[indGT].ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            if (indGT == arData__Clst[ClstNum].ParentIndGT)
            {
                bParent = true;
            }
        }
    }
    return bParent;
}


qint32 CAirBall_Proc_Inner_Arr::GetParentIndGTInCluster(qint32 indGT)
{
    qint32 ParentInd = -1;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        qint32 ClstNum = arData_GT[indGT].ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            ParentInd = arData__Clst[ClstNum].ParentIndGT;
        }
    }
    return ParentInd;
}


bool CAirBall_Proc_Inner_Arr::GTIsFirstWHInCluster(qint32 indGT)
{
    bool bFstWH = false;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        qint32 ClstNum = arData_GT[indGT].ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            if (indGT == arData__Clst[ClstNum].FstWHInd)
            {
                bFstWH = true;
            }
        }
    }
    return bFstWH;
}


qint32 CAirBall_Proc_Inner_Arr::GetFirstWHIndGTInCluster(qint32 indGT)
{
    qint32 FstWHInd = -1;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        qint32 ClstNum = arData_GT[indGT].ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            FstWHInd = arData__Clst[ClstNum].FstWHInd;
        }
    }
    return FstWHInd;
}


qreal CAirBall_Proc_Inner_Arr::GetStartTimeInCluster(qint32 indGT)
{
    qreal tStartClst = AIR_BALL::Timer_INI;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        qint32 ClstNum = arData_GT[indGT].ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            tStartClst = arData__Clst[ClstNum].tStart;
        }
    }
    return tStartClst;
}


void CAirBall_Proc_Inner_Arr::GetStartPointEllInCluster(qint32 indGT, GLPointDouble3D &ResStartPoint, EllipseInfo &ResStartEll)
{
    ResStartPoint.clear();
    ResStartEll.Reset();
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        qint32 ClstNum = arData_GT[indGT].ClstNum;
        if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
        {
            ResStartPoint   = arData__Clst[ClstNum].StartPoint;
            ResStartEll     = arData__Clst[ClstNum].StartEll;
        }
    }
}


void CAirBall_Proc_Inner_Arr::DetermineParentInClst(qint32 ClstNum)
{
    if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
    {
        CAirBall_Proc_ClstGTData *pClst = &arData__Clst[ClstNum]; //pointer to the cluster data

        qreal tMin = con_large_value;
        qint32 ParentInd = -1;
        std::set<qint32>::iterator it;
        for (it = pClst->SetGTInd.begin(); it != pClst->SetGTInd.end(); it++)
        {
            qint32 GTInd = (*it);
            if (0 <= GTInd && GTInd < AIR_BALL::GT_FORM_AMOUNT)
            {
                qreal tFirstCurr = arData_GT[GTInd].tFirst;
                if (tFirstCurr > AIR_BALL::Timer_INI + con_eps
                        && (tMin > con_large_value - con_eps
                            ||tMin > tFirstCurr))
                {
                    tMin = tFirstCurr;
                    ParentInd = GTInd;
                }
            }
        }

        pClst->ParentIndGT = ParentInd;
    }
}

void CAirBall_Proc_Inner_Arr::DetermineFstWHInClst(qint32 ClstNum)
{
    if (0 < ClstNum && ClstNum < AIR_BALL::CLST_FORM_AMOUNT)
    {
        CAirBall_Proc_ClstGTData *pClst = &arData__Clst[ClstNum];

        qreal tMin = con_large_value;
        qint32 IndFstWH = -1;
        std::set<qint32>::iterator it;
        for (it = pClst->SetGTInd.begin(); it != pClst->SetGTInd.end(); it++)
        {
            qint32 GTInd = (*it);
            if (0 <= GTInd && GTInd < AIR_BALL::GT_FORM_AMOUNT)
            {
                qint16 BallType = arData_GT[GTInd].BallType;
                qreal tFirstCurr = arData_GT[GTInd].tFirst;
                if (BallType == SUBCLASS_WARHEAD/* || BallType == TYPE_MaCRV || BallType == TYPE_QBM || BallType == TYPE_MaRV*/)
                {
                    if (tFirstCurr > AIR_BALL::Timer_INI + con_eps
                            && (tMin > con_large_value - con_eps
                                || tMin > tFirstCurr))
                    {
                        tMin = tFirstCurr;
                        IndFstWH = GTInd;
                    }
                }
            }
        }

        pClst->FstWHInd = IndFstWH;
    }
}


TrajParamBall::TrajParamBall()
{
//    Reset();
}


void TrajParamBall::Reset()
{
    memset(this, 0, sizeof(TrajParamBall));
}


TrajParamBall& TrajParamBall::operator=(const TrajParamBall &Q)
{
    this->H         = Q.H;
    this->V         = Q.V;
    this->Vh        = Q.Vh;
    this->He        = Q.He;
    this->ah        = Q.ah;
    this->a         = Q.a;
    this->ah_noC    = Q.ah_noC;
    this->SigH      = Q.SigH;
    this->SigV      = Q.SigV;
    this->SigVh     = Q.SigVh;
    this->SigHe     = Q.SigHe;
    this->SigAh     = Q.SigAh;
    this->SigA      = Q.SigA;
    return *this;
}
