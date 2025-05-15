#ifndef CPATHBRANCHDETERMINATIONM_H
#define CPATHBRANCHDETERMINATIONM_H

#include <stdio.h>
#include "CPathBranchDet_structs.h"
#include "ball_amount_const.h"
#include "gl/GLArray.h"

class CPathBranchDetermination_M
{
    friend class CAirBallist_Processing_M;

public:
    CPathBranchDetermination_M();
    ~CPathBranchDetermination_M();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Reset
    // DESCRIPTION	:   Reset of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CPathBranchDetermination::InitPointerToBallProcArray
    //DESCRIPTION	:   Copies the pointer to array from CAirBallist_Processing to m_p_Arr_BallProc
    //INPUTS		:   Reference to the array
    //RETURNS		:   None
    void InitPointerToBallProcArray(CAirBall_Proc_Inner_Arr *Arr);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::PathBranchDet_Realization_ST()
    // DESCRIPTION	:   Main part of path and branch determination algorithm, for single track
    // INPUTS		:	None
    // RETURNS		:	None
    void PathBranchDet_Realization_ST(const CPathBranch_InputData_ST *pInData,
                                      CPathBranch_OutputData_ST &rOutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::PathBranchDet_Realization_GT()
    // DESCRIPTION	:   Main part of path and branch determination algorithm, for generalized track
    // INPUTS		:	None
    // RETURNS		:	None
    void PathBranchDet_Realization_GT(const CPathBranch_InputData_GT *pInData,
                                      CPathBranch_OutputData_GT &rOutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Path_Fusion_GT()
    // DESCRIPTION	:   Determination of trajectory path value of generalized track in accordance to those values of singles tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void Path_Fusion_GT(const CPathBranch_InputData_GT *pInData,
                        CPathBranch_OutputData_GT &rOutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::ClarifyEAP_Time()
    // DESCRIPTION	:   Clarify EAP time using information from tables of EAP etc
    // INPUTS		:	None
    // RETURNS		:	None
    void ClarifyEAP_Time();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Drop_GT
    // DESCRIPTION	:   Drop generalized track
    // INPUTS		:	Index of generalized track
    // RETURNS		:	None
    void Drop_GT(const qint32 ind_GT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Drop_GT
    // DESCRIPTION	:   Drop single track
    // INPUTS		:	Index of generalized track
    // RETURNS		:	None
    void Drop_ST(const qint32 ind_ST);

private:
    CGLVector <CPathBranch_SavedData_GT>     m_arPathBrData_GT;     //array of structures containing saved data about generalized tracks
    CGLVector <CPathBranch_InnerData_ST>     m_arPathBrData_ST;     //array of structures containing inner data about single tracks

    CAirBall_Proc_Inner_Arr*    m_p_Arr_BallProc{nullptr};      //pointer to the working array in CAirBallist_Processing

    FILE*   m_pLogPBr{nullptr};         //pointer to the log file

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Branch_Determination_ST()
    // DESCRIPTION	:   Determination of branch for single track (see enum BTBranches)
    // INPUTS		:	None
    // RETURNS		:	None
    void Branch_Determination_ST(const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Branch_Check_ST()
    // DESCRIPTION	:   Check of branch for single track (see enum BTBranches)
    // INPUTS		:	None
    // RETURNS		:	None
    void Branch_Check_ST(const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Path_Determination_ST()
    // DESCRIPTION	:   Determination of path for single track (see enum BTPaths)
    // INPUTS		:	None
    // RETURNS		:	None
    void Path_Determination_ST(const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Branch_Det_forCurrentUpd_ST()
    // DESCRIPTION	:   Determination of branch (see enum BTBranches) for current update, for single track
    // INPUTS		:	None
    // RETURNS		:	CurrBranch - value of branch determinated for current update
    void Branch_Det_forCurrentUpd_ST(qint32 &CurrBranch,
                                     const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Path_Det_forCurrentUpd_ST()
    // DESCRIPTION	:   Determination of path (see enum BTPaths) for current update, for single track
    // INPUTS		:	None
    // RETURNS		:	CurrPath - value of path determinated for current update
    void Path_Det_forCurrentUpd_ST(qint32 &CurrPath,
                                   const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::EstimatePathByVertVel()
    // DESCRIPTION	:   Estimates path using vertical velocity
    // INPUTS		:	None
    // RETURNS		:	Path value estimated using vertical velocity
    qint16 EstimatePathByVertVel(const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::VhArrayFilling()
    // DESCRIPTION	:   Filling array ArrVH of vertical velocities to remember several latest values
    // INPUTS		:	None
    // RETURNS		:	None
    void VhArrayFilling(const CPathBranch_InputData_ST *pInData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::DepthFindingInVhCalculating()
    // DESCRIPTION	:   Finding of depth value to calculate a mean vertical acceleration
    // INPUTS		:	None
    // RETURNS		:	None
    void DepthFindingInVhCalculating();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::ClearInnerData_GT
    // DESCRIPTION	:   Cleaning inner information about general tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData_GT();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::ClearInnerData_GT_1Track
    // DESCRIPTION	:   Cleaning inner data containing information about given general track
    // INPUTS		:	Index of general track
    // RETURNS		:	None
    void ClearInnerData_GT_1Track(const qint32 index);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::ClearInnerData_ST
    // DESCRIPTION	:   Cleaning inner information about single tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::ClearInnerData_ST_1Track
    // DESCRIPTION	:   Cleaning inner data containing information about given single track
    // INPUTS		:	Index of general track
    // RETURNS		:	None
    void ClearInnerData_ST_1Track(const qint32 index);
};

#endif // CPATHBRANCHDETERMINATION_H
