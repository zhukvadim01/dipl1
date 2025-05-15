#include "CBallSubclassDet_structs.h"

BTMark_InputData::BTMark_InputData()
{
//    Reset();
}

void BTMark_InputData::Reset()
{
    memset(this, 0, sizeof(BTMark_InputData));
    pTrParameters = nullptr;
    Cells_BMMark = nullptr;
}

BTMark_OutputData::BTMark_OutputData()
{
//    Reset();
}

void BTMark_OutputData::Reset()
{
    memset(this, 0, sizeof(BTMark_OutputData));
}

BTMark_InnerData::BTMark_InnerData()
{
//    Reset();
}

void BTMark_InnerData::Reset()
{
    memset(this, 0, sizeof(BTMark_InnerData));
    tCheck = AIR_BALL::Timer_INI;
    tBeginConfirmation = AIR_BALL::Timer_INI;
    tBeginConfirm_min = AIR_BALL::Timer_INI;
    tBeginConfirm_max = AIR_BALL::Timer_INI;
    PrevTLoc = AIR_BALL::Timer_INI;
}

BM_Diapason::BM_Diapason()
{
//    Reset();
}

void BM_Diapason::Reset()
{
    HeMin = 0;
    HeMax = 0;
}

TrajType_InputData::TrajType_InputData()
{
//    Reset();
}

void TrajType_InputData::Reset()
{
    memset(this, 0, sizeof(TrajType_InputData));
}

TrajType_OutputData::TrajType_OutputData()
{
//    Reset();
}

void TrajType_OutputData::Reset()
{
    memset(this, 0, sizeof(TrajType_OutputData));
}

TrajType_InnerData::TrajType_InnerData()
{
//    Reset();
}

void TrajType_InnerData::Reset()
{
    memset(this, 0, sizeof(TrajType_InnerData));
}
