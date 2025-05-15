//FILE			:   CBallist_ActiveProgn.h
//AUTHOR		:   TaniaP
#ifndef _rip_tc_progn_actm_h
#define _rip_tc_progn_actm_h

#include "CBallist_ActiveProgn_structs.h"
#include "conversion/Topocentric.h"


//CLASS			:CBallist_ActiveProgn
//DESCRIPTION	:Estimation of Start and Falling regions for Ballistic Target
//				 by data received on active leg
class CBallist_ActiveProgn_M
{
public:
    CBallist_ActiveProgn_M();
    ~CBallist_ActiveProgn_M();
	
    Pr_Act_Const	m_APrConst;

    //FUNCTION		:   CBallist_ActiveProgn::InitConst
    //DESCRIPTION	:   Initialization of constants
    //INPUTS		:   Reference to structure for constants space
    //RETURNS		:   void
    void InitConst(const Pr_Act_Const &cnst);

    //FUNCTION		:   CBallist_ActiveProgn::GetActiveProgn
    //DESCRIPTION	:   Estimation of Start and Fall regions on active leg
    //INPUT			:   reference to output structure
    //RETURNS		:   void
    void GetActiveProgn(const Pr_Act_Input *pInData, Pr_Act_Out	&rOutData);

    //FUNCTION		:   CBallist_ActiveProgn::GetActiveProgn
    //DESCRIPTION	:   Estimation of the start time
    //INPUT			:   Reference to to the current point in topocentric coordinate system;
    //              :   reference to the current velocity in topocentric coordinate system;
    //              :   reference to the resulting time;
    //              :   reference to the resulting sign of the first correctly calculated start point
    //RETURNS		:   True if result is OK
    bool StartTimeEstimation(const Pr_Act_Input *pInData,
                             const CTopocentric &CurPointT, const CTopocentric &CurVelT,
                             qreal &t_res, bool &P_first_SP_calc);

    //FUNCTION		:   CBallist_ActiveProgn::OutputParamClarification
    //DESCRIPTION	:   Clarification of the output parameters
    //INPUT			:   Sign "azimuth was increased",
    //              :   reference to the output parameters
    //RETURNS		:   True if result is OK
    void OutputParamClarification(const Pr_Act_Input *pInData, bool bAzIncreased,
                                  Pr_Act_Out &rOutData);
};

#endif
