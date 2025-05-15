#ifndef CAIRBALLIST_PROCESSING_H
#define CAIRBALLIST_PROCESSING_H

#include <stdio.h>
#include "CAirBallist_Processing_structs.h"
#include "ball_amount_const.h"
#include "CPathBranchDetermination.h"
#include "CBallSubclassDetermination.h"
#include "CBallisticProlongation.h"
#include "CBallist_ActiveProgn.h"
#include "gl/GLArray.h"

class CAirBallist_Processing
{
public:
    CAirBallist_Processing();

    ~CAirBallist_Processing();

    CAirBall_Proc_OutData m_OutData_Ballist; //output data

    CAirBall_Proc_Inner_Arr m_arProcData;   //array of data

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Reset
    // DESCRIPTION	:   Reset of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::FillInputData
    // DESCRIPTION	:   Filling of input data, for GT
    // INPUTS		:	Structure containing the input data
    // RETURNS		:	None
    void FillInputData(CAirBall_Proc_InputData_GT &InData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::FillInputData
    // DESCRIPTION	:   Filling of input data, for ST
    // INPUTS		:	Structure containing the input data
    // RETURNS		:	None
    void FillInputData(CAirBall_Proc_InputData_ST &InData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::AirBallist_Realization
    // DESCRIPTION	:   Realization of trajectory data processing to perform ballistic processing tasks, for GT
    // INPUTS		:	None
    // RETURNS		:	None
    void AirBallist_Realization();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::AirBallist_Realization_ST
    // DESCRIPTION	:   Realization of trajectory data processing to perform ballistic processing tasks, for ST
    // INPUTS		:	None
    // RETURNS		:	None
    void AirBallist_Realization_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::AirBallist_Realization_Satellite
    // DESCRIPTION	:   Realization of trajectory data processing to perform ballistic processing tasks, for GT of the satellite
    // INPUTS		:	Reference to the structure containing output data
    // RETURNS		:	None
    void AirBallist_Realization_Satellite(CAirBall_Proc_OutData &OutData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::SetPersistentData
    // DESCRIPTION	:   Set persistent data
    // INPUTS		:	Pointer to the structure containing array of tables of EAP;
    //              :   pointer to the structure containing array of tables of ballistic coefficients
    // RETURNS		:	None
    void SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables,
                           Str_Ar_BallCoef_Param_AllTables* p_arBallCoef_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::SetPersistentData
    // DESCRIPTION	:   Set persistent data
    // INPUTS		:	Pointer to the structure containing array of tables of EAP
    // RETURNS		:	None
    void SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::FillHeBoundaryValues
    // DESCRIPTION	:   Fills boundary values of energetic height
    // INPUTS		:	Pointer to the array of cells with data about BM Marks
    // RETURNS		:	None
    void FillHeBoundaryValues(Cells_BMMark_Data *pArrCells);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::SetActionPhase
    // DESCRIPTION	:   Set action phase
    // INPUTS		:	Number of generalized track; action phase (enum EActionPhaseAO in GDEnum.h);
    //              :   current time
    // RETURNS		:	None
    void SetActionPhase(const qint32 NumbGT, const qint16 ActionPhase, qreal CurTime);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::SetBallCoeffSrc
    // DESCRIPTION	:   Set ballistic coefficient obtained from source
    // INPUTS		:	Number of generalized track; ballistic coefficient obtained from source (m^2/kg)
    // RETURNS		:	None
    void SetBallCoeffSrc(const qint32 NumbGT, const qreal Gamma);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::SetNumbGT
    // DESCRIPTION	:   Set number of generalized track to the array of GT
    // INPUTS		:	Index of generalized track, number of generalized track
    // RETURNS		:	None
    void SetNumbGT(const qint32 IndGT, const qint32 NumbGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Drop_GT
    // DESCRIPTION	:   Drop generalized track (GT)
    // INPUTS		:	Index of GT, index of corresponding ballistic track
    // RETURNS		:	None
    void Drop_GT(const qint32 ind_GT, const qint32 ind_Ball);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Drop_ST
    // DESCRIPTION	:   Drop single track (ST)
    // INPUTS		:	Index of ST
    // RETURNS		:	None
    void Drop_ST(const qint32 ind_ST);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::ClearBallisticData
    // DESCRIPTION	:   Clears ballistic data
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void ClearBallisticData(const qint32 ind_Ball);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::NewTrackProcessing_GT()
    // DESCRIPTION	:   Processing of new generalized track
    // INPUTS		:	None
    // RETURNS		:	None
    void NewTrackProcessing_GT();

private:
    CAirBall_Proc_InputData_GT* m_pInData{nullptr};     //pointer to the input data, for GT
    CAirBall_Proc_InputData_ST* m_pInData_ST{nullptr};  //pointer to the input data, for ST
    Str_Ar_EAP_Param_AllTables* m_pEAP_Tables{nullptr}; //pointer to the structure containing EAP Tables

    CPathBranchDetermination    m_PathBranchDet;     //instance of class that implements the path and branch determination
    CBallSubclassDetermination  m_BallSubclDet;      //instance of class that implements the ballistic subclass and trajectory type determination
    CBallisticProlongation      m_BallProlong;       //instance of class that implements prolongation tasks
    CBallist_ActiveProgn        m_BallActPred;       //instance of class that implements prediction on active path

    TrajParamBall m_TrPar;          //trajectory parameters, calculated for current update

//    FILE *m_pLogCtrl; //pointer to the log file
    FILE* m_pLogProlTrack{nullptr}; //pointer to the log file for prolonged track

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::NewTrackProcessing_ST()
    // DESCRIPTION	:   Processing of new single track
    // INPUTS		:	None
    // RETURNS		:	None
    void NewTrackProcessing_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::InitConst()
    // DESCRIPTION	:   Fill constants
    // INPUTS		:	None
    // RETURNS		:	None
    void InitConst();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::UpdateCluster()
    // DESCRIPTION	:   Updates information about cluster
    // INPUTS		:	None
    // RETURNS		:	None
    void UpdateCluster();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::ClstClearForGT()
    // DESCRIPTION	:   Clears cluster data for given GT
    // INPUTS		:	Index of the generalized track; sign "GT is dropped"
    // RETURNS		:	None
    void ClstClearForGT(qint32 indGT, bool bGTDroped);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::UpdateArrPointsForClst()
    // DESCRIPTION	:   Updates initial array of points for given cluster
    // INPUTS		:	Cluster number
    // RETURNS		:	None
    void UpdateArrPointsForClst(qint32 ClstNum);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::ClearInnerData()
    // DESCRIPTION	:   Cleaning inner data containing information about all tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::ClearInnerData()
    // DESCRIPTION	:   Cleaning inner data containing information about given track
    // INPUTS		:	index of track
    // RETURNS		:	None
    void ClearInnerData_1Track(const qint32 index);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::CalcTrajParam
    // DESCRIPTION	:   Calculation of trajectory parameters (heigh, velocity, energetic height etc; and RMSEs of this parameters)
    // INPUTS		:	Sign of generalized track (true - generalized track, false - signle track)
    // RETURNS		:	None
    void CalcTrajParam(const bool IsGT = true);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::CurrSTIsWellSmoothed
    // DESCRIPTION	:   Checks whether current ST is well smoothed
    // INPUTS		:	None
    // RETURNS		:	True if current ST is "well smoothed"
    bool CurrSTIsWellSmoothed();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::CurrGTIsWellSmoothed
    // DESCRIPTION	:   Checks whether current GT is well smoothed
    // INPUTS		:	None
    // RETURNS		:	True if current GT is "well smoothed"
    bool CurrGTIsWellSmoothed();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Path_Branch_Processing_GT
    // DESCRIPTION	:   Data processing for path and branch determination;
    //              :   implementation the path and branch determination, for generalized track;
    // INPUTS		:	None
    // RETURNS		:	None
    void Path_Branch_Processing_GT();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Path_Branch_Processing_ST
    // DESCRIPTION	:   Data processing for path and branch determination;
    //              :   implementation the path and branch determination, for single track
    // INPUTS		:	None
    // RETURNS		:	None
    void Path_Branch_Processing_ST();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Active_Prediction_Processing
    // DESCRIPTION	:   Data processing for impact area prediction on active path;;
    //              :   implementation the impact area prediction on active path
    // INPUTS		:	None
    // RETURNS		:	None
    void Active_Prediction_Processing();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::DeterminationBallCoeff
    // DESCRIPTION	:   Ballistic coefficient determination
    // INPUTS		:	None
    // RETURNS		:	None
    void DeterminationBallCoeff();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Prediction_Processing
    // DESCRIPTION	:   Data processing for prediction;
    //              :   implementation the prediction
    // INPUTS		:	None
    // RETURNS		:	None
    void Prediction_Processing();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::TrajType_Processing
    // DESCRIPTION	:   Data processing for determination of the trajectory type (see enum BallTrajTypes);
    //              :   determination of the trajectory type
    // INPUTS		:	None
    // RETURNS		:	None
    void TrajType_Processing();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::BT_Mark_Processing
    // DESCRIPTION	:   Data processing for determination of the trajectory mark;
    //              :   determination of the trajectory mark
    // INPUTS		:	None
    // RETURNS		:	None
    void BT_Mark_Processing();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::HorizontalManeuverDetection
    // DESCRIPTION	:   Maneuver detection in the horizontal plane
    // INPUTS		:	None
    // RETURNS		:	None
    void HorizontalManeuverDetection();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::TimeEAP_Updating
    // DESCRIPTION	:   Updates time of the end of active path
    // INPUTS		:	None
    // RETURNS		:	None
    void TimeEAP_Updating();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::TimeEAP_Updating
    // DESCRIPTION	:   Updates time of the end of active path
    // INPUTS		:	None
    // RETURNS		:	None
    void ParametersEAP_Updating();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::GetMinActPathTimeByMark
    // DESCRIPTION	:   Returns minimum value of active path duration time for given GT
    // INPUTS		:	GT index
    // RETURNS		:	Minimum value of active path duration time
    qreal GetMinActPathTimeByMark(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::CheckConditionsForPredict
    // DESCRIPTION	:   Checks conditions for prediction performing
    // INPUTS		:	None
    // RETURNS		:	True if prediction conditions are performed
    bool CheckConditionsForPredict();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::CheckPredictionDeviation
    // DESCRIPTION	:   Checks deviation of prediction from the input data
    // INPUTS		:	None
    // RETURNS		:	True if deviation of prediction is large
    bool CheckPredictionDeviation();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::CalcDistance
    // DESCRIPTION	:   Calculate distance between start and fall
    // INPUTS		:	None
    // RETURNS		:	true if result is ok
//    bool CalcDistance();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Estimate_Threat
    // DESCRIPTION	:   Estimates threat for given ballistic object
    // INPUTS		:	Index of ballistic object
    // RETURNS		:	Sign of threat
    bool Estimate_Threat(qint32 indBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::Estimate_AB_Distance
    // DESCRIPTION	:   Estimates maximum distance for possible aeroballistic object
    // INPUTS		:	Index of generalized track
    // RETURNS		:	Distance for possible aeroballistic object
    qreal Estimate_AB_Distance(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::GetFlatLoftBound_Mark
    // DESCRIPTION	:   Returns boundary angles for lofted and flat trajectories depending on mark
    // INPUTS		:	None
    // RETURNS		:	Boundary angles for lofted and flat trajectories
    void GetFlatLoftBound_Mark(qreal &bound_flat_tr, qreal &bound_lofted_tr);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::ReturnFromPassToAct_Processing
    // DESCRIPTION	:   Data processing in the case of transition from passive path to active path
    // INPUTS		:	None
    // RETURNS		:	None
    void ReturnFromPassToAct_Processing();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBallist_Processing::LogCtrlPoint
    // DESCRIPTION	:   Log data in the control point
    // INPUTS		:	Number of control point; sign of Generalized Track (true - GT, false - ST)
    // RETURNS		:	None
//    void LogCtrlPoint(const qint16 NumCtrlPoint, const bool bIsGT);

};

#endif // CAIRBALLIST_PROCESSING_H
