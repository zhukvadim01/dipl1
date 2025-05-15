#ifndef CPATHBRANCHDET_STRUCTS_H
#define CPATHBRANCHDET_STRUCTS_H

#include <memory.h>
#include "gl/GLArray.h"
#include "air_ballist_constants.h"
#include "CAirBallist_Processing_structs.h"

//PACKAGE       :   AirBallist
//STRUCTURE     :   CPathBranch_InputData_ST
//DESCRIPTION   :   Input data for single track
struct CPathBranch_InputData_ST
{
    qreal  t_Loc{AIR_BALL::Timer_INI};  //time of location, s
    qreal  IF_ST{-1};                   //track index in array (Single Track)
    qreal  Type{0.};                    //type of ST, enum EType (GDEnum.h)
    qreal  TypeGT{0.};                  //type of GT, enum EType (GDEnum.h)
    bool   bAeroballistic{false};       //sign of aeroballistic object

    TrajParamBall TrParameters;
    CAirBall_Proc_InputData_ST* pInDataST{nullptr}; //pointer to the input data

    qreal  Prob_IMM_small_coeff{0.};    //probability of filter with small noise coefficient at the output of IMM filter
    qreal  Prob_IMM_large_coeff{0.};    //probability of filter with large noise coefficient at the output of IMM filter
    qreal  R_rdr_obj{0.};               //distance from source to the ballistic object, m

    CPathBranch_InputData_ST();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CPathBranch_InputData_GT
//DESCRIPTION   :   Input data for generalized track
struct CPathBranch_InputData_GT
{
    qint32              IF_GT{-1};  //index of generalized track
    qreal               tLoc{0.};   //location time, s
    std::set<qint32>    SetST;      //set of indexes of ST accociated with current GT, in the numeration of AirBallist
    qreal               TypeGT{TYPE_UNDEFINED};     //type of GT, enum EType (GDEnum.h)
    qreal               tPrevPathDet{AIR_BALL::Timer_INI}; //previous time of path determination

    CPathBranch_InputData_GT();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CPathBranch_OutputData_ST
//DESCRIPTION   :   Output data for single track
struct CPathBranch_OutputData_ST
{
    bool P_Save_prev_PathBranchData{false}; //true if data about path and branch has not changed

    qint16    P_Path{0};    //sign of path of trajectory (see enum BTPaths)
    qint16    P_Branch{0};  //sign of branch of trajectory (see enum BTBranches)
//    qreal  T_def_eap; //time when the trajectory path flag was set to «End of active path», s
//    qreal  T_eap; //end-of-boost time, s

    CPathBranch_OutputData_ST();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CPathBranch_OutputData_GT
//DESCRIPTION   :   Output data for generalized track
struct CPathBranch_OutputData_GT
{
    bool bPathBranchChanged{false}; //true if data about path and branch has changed

//    qint32 P_Path; //sign of path of trajectory (see enum BTPaths)
//    qint32 P_Branch; //sign of branch of trajectory (see enum BTBranches)
//    qreal  T_def_eap; //time when the trajectory path flag was set to «End of active path», s
//    qreal  T_eap; //end-of-boost time, s

    CPathBranch_OutputData_GT();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CPathBranch_InnerData_ST
//DESCRIPTION   :   Inner data about single tracks
struct CPathBranch_InnerData_ST
{
    qint16  P_Path{0};      //sign of path of trajectory (see enum BTPaths)
    qint16  P_Branch{0};    //sign of branch of trajectory (see enum BTBranches)
    qreal   T_def_eap{AIR_BALL::Timer_INI}; //time when the trajectory path flag was set to «End of active path», s
//    qreal  T_eap; //end-of-boost (= End of active path) time, s
    qreal   T_act_prouved{AIR_BALL::Timer_INI}; //time moment of last active path prouvement

    CGLArrayFix <qreal, DEPTH_VH_MAX> arr_TVh;      //array of time moments corresponding to the saved Vh values
    CGLArrayFix <qreal, DEPTH_VH_MAX> arr_Vh;       //array of recent values of Vh
    CGLArrayFix <qreal, DEPTH_VH_MAX> arr_SigVh;    //array of recent values of SigmaVh

    CGLArrayFix <qreal, DEPTH_AH_MAX> arr_Ah;       //array of recent values of Ah
    CGLArrayFix <qreal, DEPTH_AH_MAX> arr_SigAh;    //array of recent values of SigmaAh

    CGLArrayFix <qreal, MIN_OVERWEIGHT_POINT_NUMBER> arr_Prob_IMM_small_coeff; //array of recent values of Ah
    CGLArrayFix <qreal, MIN_OVERWEIGHT_POINT_NUMBER> arr_Prob_IMM_large_coeff; //array of recent values of SigmaAh

    qint32  currentVhIndex{0};   //index of current element in the arrays arr_Vh, arr_SigVh
    qint32  currentAhIndex{0};   //index of current element in the arrays arr_Ah, arr_SigAh
    qint32  Curr_filter_Ind{0};  //index of current element in the arrays arr_Prob_IMM_small_coeff, arr_Prob_IMM_large_coeff

    qint32  currentVhArraySize{0};       //number of filled elements in the arrays arr_Vh, arr_SigVh
    qint32  currentAhArraySize{0};       //number of filled elements in the arrays arr_Ah, arr_SigAh
    qint32  currentFilterArraySize{0};   //number of filled elements in the arrays arr_Prob_IMM_small_coeff, arr_Prob_IMM_large_coeff

    qint32  NumAhExpSm{0};  //sequential number of exponentially smoothed acceleration
    qreal   AhExpSm{0};     //exponentially smoothed acceleration
    //qreal SigAhExpSm; //RMSE of the exponentially smoothed acceleration

    qreal   t_begin_check_branch{AIR_BALL::Timer_INI}; //time of begin of check for branch, s
    qint32  counter_check_branch{0};                  //counter for branch check
    qint32  Branch_saved_for_check{0};                //additional saved value of branch for check

    qreal   t0_active_redetermination{AIR_BALL::Timer_INI};   //time of first active path redetermination after passive path
    qreal   counter_active_redetermination{0};                //counter of active path redeterminations after passive path

    CPathBranch_InnerData_ST();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CPathBranch_SavedData_GT
//DESCRIPTION   :   Inner data for generalized track
struct CPathBranch_SavedData_GT
{
    qint16   P_Path{0};             //sign of path of ballistic trajectory (see enum BTPaths)
    qint16   P_Branch{0};           //sign of branch of ballistic trajectory (enum BTBranches)
    qreal    T_def_eap{AIR_BALL::Timer_INI}; //time when the trajectory path flag was set to «End of active path», s
    qreal    T_eap{AIR_BALL::Timer_INI}; //end-of-boost (= End of active) path time, s
    bool     bActObserved{false};   //true if the active path was observed

    CPathBranch_SavedData_GT();

    void Reset();
};

#endif // CPATHBRANCHDET_STRUCTS_H
