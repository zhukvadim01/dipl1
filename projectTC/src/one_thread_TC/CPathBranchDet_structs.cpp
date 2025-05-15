#include "CPathBranchDet_structs.h"

CPathBranch_InputData_ST::CPathBranch_InputData_ST()
{
//    Reset();
}

void CPathBranch_InputData_ST::Reset()
{
    t_Loc = AIR_BALL::Timer_INI;
    IF_ST = -1;
    Type = 0;
    TypeGT = 0;
    bAeroballistic = false;
    TrParameters.Reset();
    pInDataST = nullptr;
    Prob_IMM_small_coeff = 0;
    Prob_IMM_large_coeff = 0;
    R_rdr_obj = 0;
}

CPathBranch_InputData_GT::CPathBranch_InputData_GT()
{
//    Reset();
}

void CPathBranch_InputData_GT::Reset()
{
    IF_GT = -1;
    tLoc = 0;
    SetST.clear();
    TypeGT = TYPE_UNDEFINED;
    tPrevPathDet = AIR_BALL::Timer_INI;
}

CPathBranch_OutputData_ST::CPathBranch_OutputData_ST()
{
//    Reset();
}

void CPathBranch_OutputData_ST::Reset()
{
    memset(this, 0, sizeof(CPathBranch_OutputData_ST));
}

CPathBranch_OutputData_GT::CPathBranch_OutputData_GT()
{
//    Reset();
}

void CPathBranch_OutputData_GT::Reset()
{
    memset(this, 0, sizeof(CPathBranch_OutputData_GT));
}

CPathBranch_InnerData_ST::CPathBranch_InnerData_ST()
{
//    Reset();
}

void CPathBranch_InnerData_ST::Reset()
{
    P_Path = 0;
    P_Branch = 0;
    T_def_eap = AIR_BALL::Timer_INI;
    T_act_prouved = AIR_BALL::Timer_INI;
    arr_TVh.Reset();
    arr_Vh.Reset();
    arr_SigVh.Reset();
    arr_Ah.Reset();
    arr_SigAh.Reset();
    arr_Prob_IMM_small_coeff.Reset();
    arr_Prob_IMM_large_coeff.Reset();
    currentVhIndex = 0;
    currentAhIndex = 0;
    Curr_filter_Ind = 0;
    currentVhArraySize = 0;
    currentAhArraySize = 0;
    currentFilterArraySize = 0;
    NumAhExpSm = 0;
    AhExpSm = 0;
    //SigAhExpSm = 0;
    t_begin_check_branch = AIR_BALL::Timer_INI;
    counter_check_branch = 0;
    Branch_saved_for_check = 0;
    t0_active_redetermination = AIR_BALL::Timer_INI;
    counter_active_redetermination = 0;
}

CPathBranch_SavedData_GT::CPathBranch_SavedData_GT()
{    
}

void CPathBranch_SavedData_GT::Reset()
{
    P_Path = 0;
    P_Branch = 0;
    T_def_eap = AIR_BALL::Timer_INI;
    T_eap = AIR_BALL::Timer_INI;
    bActObserved = false;
}
