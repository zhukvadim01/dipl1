#ifndef CBALLSUBCLASSDET_STRUCTS_H
#define CBALLSUBCLASSDET_STRUCTS_H

#include <memory.h>
#include "air_ballist_constants.h"
#include "gl/GLArray.h"
#include "CtrlAir_Ballist_structs.h"
#include "CAirBallist_Processing_structs.h"

//PACKAGE       :   AirBallist
//STRUCTURE     :   BTMark_InputData
//DESCRIPTION	:   Input data for BT mark determination
struct BTMark_InputData
{
    TrajParamBall*      pTrParameters{nullptr}; //pointer to the trajectory parameters
    Cells_BMMark_Data*  Cells_BMMark{nullptr};  //pointer to the array defining the correspondence between the cells on the plane "H-He" and the BT marks

    qint32  IndBall{0};     //index of the ballistic track
    qreal   tLoc{0.};       //time of location, s
    qreal   tFall{0.};      //predicted time of fall, s
//    qreal  H;             //height, m
//    qreal  He;            //energetic height, m
    qreal   D{0.};          //distance from start to fall, m
    qreal   SigmaSP{0.};    //summary error for start and fall point, m
    qreal   Theta{0.};      //throw angle at the end of active path, degrees
    qreal   Veap{0.};       //velocity at the end of active path, m/s
    qint32  Branch{0};      //sign of branch of the trajectory, see enum BTBranches

    BTMark_InputData();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   BTMark_OutputData
//DESCRIPTION	:   Output data for BT mark determination
struct BTMark_OutputData
{
    bool                P_MarkIsChanged{false}; //true if value of BT mark is changed
    BMMark_AndDiapason  BMMark;                 //value of BT mark or diapason of values, see enum BM_Marks

    BTMark_OutputData();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   BTMark_InnerData
//DESCRIPTION	:   Inner data for BT mark determination
struct BTMark_InnerData
{
    BMMark_AndDiapason  BMMark;                 //value of BT mark or diapason of values, see enum BM_Marks
    qreal       tCheck{AIR_BALL::Timer_INI};    //s, time of last check of mark

    qreal       tBeginConfirmation{AIR_BALL::Timer_INI};    //s, time of begin of mark confirmation
    qint32      n_Confirmation{0};                          //counter of confirmations
    qreal       tBeginConfirm_min{AIR_BALL::Timer_INI};     //s, time of begin of minimal mark confirmation
    qint32      n_Confirm_min{0};                           //counter of confirmations of minimal mark
    qreal       tBeginConfirm_max{AIR_BALL::Timer_INI};     //s, time of begin of maximal mark confirmation
    qint32      n_Confirm_max{0};                           //counter of confirmation of maximal mark

    bool        SignConfrimation{false};        //true if confirmation is need
    bool        SignFinalDecision{false};       //true if final decision about mark is accepted
    bool        SignConfirmedMin{false};        //true if minimal mark is confirmed
    bool        SignConfirmedMax{false};        //true if maximal mark is confirmed
    qint32      MaxMark_Confirmed{0};           //confirmed value of maximal mark
    qint32      MinMark_Confirmed{0};           //confirmed value of mimimal mark
    qreal       PrevTLoc{AIR_BALL::Timer_INI};  //previous time of location

    BTMark_InnerData();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   BM_Diapason
//DESCRIPTION	:   Diapason of parameters for BM mark
struct BM_Diapason
{
    qreal   HeMin{0.};  //minimum value of the energetic height, m
    qreal   HeMax{0.};  //maximum value of the energetic height, m

    BM_Diapason();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   TrajType_InputData
//DESCRIPTION	:   Input data for trajectory type determination
struct TrajType_InputData
{
    qint32      IndBall{0};             //index of the ballistic track
    qreal       tLoc{0.};               //location time, s
    qreal       H{0.};                  //height, m
    qreal       VH{0.};                 //vertical velocity, m/s
    qreal       AH{0.};                 //vertical acceleration, m/s^2
    qreal       D{0.};                  //flight range of a missile, m
    qreal       throw_angle{0.};        //angle of departure, degrees
    qreal       bound_flat_tr{0.};      //maximal angle of departure for flat trajectory, degrees
    qreal       bound_lofted_tr{0.};    //minimal angle of departure for lofted trajectory, degrees

    TrajType_InputData();

    void Reset();
};

//PACKAGE       :   AirBallist
//STRUCTURE     :   TrajType_OutputData
//DESCRIPTION	:   Output data for trajectory type determination
struct TrajType_OutputData
{
    qint16  TrajType{0};            //type of trajectory, see enum BallTrajTypes
    bool	P_rewrite_TT{false};	//sign of overwriting of information about trajectory type

    TrajType_OutputData();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   TrajType_InnerData
//DESCRIPTION	:   Inner data for trajectory type determination
struct TrajType_InnerData
{
    qreal	Ac{0.};                 //vertical acceleration
    qint16	TType{0};               //type of trajectory (optimal, lofted, flat)
    bool	PrTTypeD{false};        //true if type of trajectory is known
    qreal	t_prev_TType_calc{0.};  //time of previous calculation of TType
    qreal	t_prev_TType_check{0.}; //time of previous check of TType
    qint16	prevTType{0};           //previous value of TType
    qint16	counterTType{0};        //counter for the equals TType

    TrajType_InnerData();

    void Reset();
};

#endif // CBALLSUBCLASSDET_STRUCTS_H
