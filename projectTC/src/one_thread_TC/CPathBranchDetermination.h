#ifndef CPATHBRANCHDETERMINATION_H
#define CPATHBRANCHDETERMINATION_H

#include <stdio.h>
#include "CPathBranchDet_structs.h"
#include "ball_amount_const.h"
#include "gl/GLArray.h"

const qreal SigmaA0 = 2. * cK_VT
        * sqrt((pow(cEAP_Ksi, 2.*static_cast<qreal>(cEAP_m0) - 2.)*(3.*cEAP_Ksi-1.)
                + sqr(cEAP_Ksi) - 2.*cEAP_Ksi + 1) / (cEAP_Ksi + 1.));
const qreal SigmaA1 = 2. * cK_VT
        * sqrt((pow(cEAP_Ksi, 2.*(static_cast<qreal>(cEAP_m0)+8.) - 2.)*(3.*cEAP_Ksi-1.)
                + sqr(cEAP_Ksi) - 2.*cEAP_Ksi + 1) / (cEAP_Ksi + 1.));

class CPathBranchDetermination
{
public:
    CPathBranchDetermination();

    ~CPathBranchDetermination();

    CPathBranch_OutputData_ST   m_OutData_PathBranch_ST; //structure containing output data, for single track
    CPathBranch_OutputData_GT   m_OutData_PathBranch_GT; //structure containing output data, for generalized track
    CGLVector <CPathBranch_SavedData_GT>     m_arPathBrData_GT; //array of structures containing saved data about generalized tracks
    CGLVector <CPathBranch_InnerData_ST>     m_arPathBrData_ST; //array of structures containing inner data about single tracks

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Reset
    // DESCRIPTION	:   Reset of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::FillInputData_ST
    // DESCRIPTION	:   Filling of input data for single track
    // INPUTS		:	Structure containing the input data
    // RETURNS		:	None
    void FillInputData_ST(CPathBranch_InputData_ST &InData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::FillInputData_GT
    // DESCRIPTION	:   Filling of input data for generalized track
    // INPUTS		:	Structure containing the input data
    // RETURNS		:	None
    void FillInputData_GT(CPathBranch_InputData_GT &InData);

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
    void PathBranchDet_Realization_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::PathBranchDet_Realization_GT()
    // DESCRIPTION	:   Main part of path and branch determination algorithm, for generalized track
    // INPUTS		:	None
    // RETURNS		:	None
    void PathBranchDet_Realization_GT();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Path_Fusion_GT()
    // DESCRIPTION	:   Determination of trajectory path value of generalized track in accordance to those values of singles tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void Path_Fusion_GT();

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
    CPathBranch_InputData_ST*   m_pInData_ST{nullptr};      //pointer to the structure containing input data for single track
    CPathBranch_InputData_GT*   m_pInData_GT{nullptr};      //pointer to the structure containing input data for generalized track

    CAirBall_Proc_Inner_Arr*    m_p_Arr_BallProc{nullptr};  //pointer to the working array in CAirBallist_Processing

    FILE*   m_pLogPBr{nullptr}; //pointer to the log file

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Branch_Determination_ST()
    // DESCRIPTION	:   Determination of branch for single track (see enum BTBranches)
    // INPUTS		:	None
    // RETURNS		:	None
    void Branch_Determination_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Branch_Check_ST()
    // DESCRIPTION	:   Check of branch for single track (see enum BTBranches)
    // INPUTS		:	None
    // RETURNS		:	None
    void Branch_Check_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Path_Determination_ST()
    // DESCRIPTION	:   Determination of path for single track (see enum BTPaths)
    // INPUTS		:	None
    // RETURNS		:	None
    void Path_Determination_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Branch_Det_forCurrentUpd_ST()
    // DESCRIPTION	:   Determination of branch (see enum BTBranches) for current update, for single track
    // INPUTS		:	None
    // RETURNS		:	CurrBranch - value of branch determinated for current update
    void Branch_Det_forCurrentUpd_ST(qint32 &CurrBranch);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::Path_Det_forCurrentUpd_ST()
    // DESCRIPTION	:   Determination of path (see enum BTPaths) for current update, for single track
    // INPUTS		:	None
    // RETURNS		:	CurrPath - value of path determinated for current update
    void Path_Det_forCurrentUpd_ST(qint32 &CurrPath);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::EstimatePathByVertVel()
    // DESCRIPTION	:   Estimates path using vertical velocity
    // INPUTS		:	None
    // RETURNS		:	Path value estimated using vertical velocity
    qint16 EstimatePathByVertVel();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::EAP_Det_VDoppl_ST()
    // DESCRIPTION	:   Determination of End of Active Path using Doppler velocity for single track
    // INPUTS		:	None
    // RETURNS		:	None
//    void EAP_Det_VDoppl_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CPathBranchDetermination::VhArrayFilling()
    // DESCRIPTION	:   Filling array ArrVH of vertical velocities to remember several latest values
    // INPUTS		:	None
    // RETURNS		:	None
    void VhArrayFilling();

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
