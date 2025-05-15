//FILE			:rip_tc_progn_act_structs.h
//AUTHOR		:Tania

#ifndef _rip_tc_progn_act_structs_h
#define _rip_tc_progn_act_structs_h

#include <memory.h>
#include "ball_prognoz_structs.h"
#include "conversion/Geodesic.h"

struct Pr_Act_Input
{
    qreal			tL{0.};					//time of location, s
    qreal			tLastActProgn{0.};		//time of last estimation of prognosis on active leg
    qreal			X{0.};
    qreal           Y{0.};
    qreal           Z{0.};                  //coordinates, m
    qreal			VX{0.};
    qreal           VY{0.};
    qreal           VZ{0.};                 //velocities, m/s;
    qreal			SigVX{0.};
    qreal           SigVY{0.};
    qreal           SigVZ{0.};              //RMSE of velocities, m/s;
    qreal			H{0.};					//height, m
    qreal			V{0.};					//module of velocity, m/s
    qreal			Vh{0.};					//vertical velocity, m/s
    qreal           SigH{0.};               //RMSE of height, m
    qreal           SigV{0.};               //RMSE of absolute velocity, m/s
    qreal           SigVH{0.};              //RMSE of vertical velocity, m/s
    qreal           SigAzPrev{0.};          //Previous value of RMSE of azimuth
    CGeodesic       Center;                 //Center of TPSC, radians, radians, meters
    GLPointDouble3D EstStartPoint0;         //Previous estimated start point
    qreal           TimeStartEst0{0.};      //Previous estimated start time, s
    qint16 			Pbs{0};                 //sign of the end of active leg
    bool            ForcedActPr{false};     //sign of forced prognosis on active leg
    AIR_BALL::s_arCovObj *p_arCovObj{nullptr};  //pointer to the array of covered objects

    Pr_Act_Input();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Pr_Act_Input::Reset
    // DESCRIPTION	:   Reset of struct attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();
};

struct Pr_Act_Const 
{
    qreal	TPeriod{0.};            //period of launch of prognosis on active leg, s
    qreal	MinimumOfRange{0.};     //minimal possible flying range, m
    qreal	MaxumumOfRange{0.};     //maximal possible flying range, m
    qreal   MaximumOfRangeExt{0.};  //extended value of maximum range, m
    qreal	DeltaAzCorr{0.};        //correction for consideration of dynamic error by azimuth, degrees
    qreal	KSigmaAz{0.};           //confidence coefficient

    Pr_Act_Const();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Pr_Act_Const::Reset
    // DESCRIPTION	:   Reset of struct attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();
};

struct Pr_Act_Out
{
    bool				P_Changed{false};	//sign of estimated information change
    Active_Pred_Info	ActPrInfo;          //structure for calculated on active leg information

    Pr_Act_Out();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Pr_Act_Out::Reset
    // DESCRIPTION	:   Reset of struct attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();
};

#endif

