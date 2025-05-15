#include "CBallist_ActiveProgn_structs.h"

Pr_Act_Input::Pr_Act_Input()
{
//    Reset();
}

void Pr_Act_Input::Reset()
{
    memset(this, 0, sizeof(Pr_Act_Input));
    EstStartPoint0.clear();
}

Pr_Act_Const::Pr_Act_Const()
{
//    Reset();
}

void Pr_Act_Const::Reset()
{
    memset(this, 0, sizeof(Pr_Act_Const));
}

Pr_Act_Out::Pr_Act_Out()
{
//    Reset();
}

void Pr_Act_Out::Reset()
{
    P_Changed = false;
    ActPrInfo.Reset();
}
