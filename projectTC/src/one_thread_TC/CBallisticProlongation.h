#ifndef CBALLISTICPROLONGATION_H
#define CBALLISTICPROLONGATION_H

#include "gl/GLGeometry.h"
#include "gl/GLMatrix.h"
#include "gl/GLArray.h"
#include "gl/GLSolve_eq_2_3_4_deg.h"

#include "conversion/Topocentric.h"
#include "conversion/Geocentric_Topo.h"

#include "ball_prognoz_structs.h"
#include "air_ballist_constants.h"
#include "ball_amount_const.h"
#include "CAirBallist_Processing_structs.h"
#include "CBallisticProlong_structs.h"
#include "ball_one_prognoz.h"
#include "cellipsepoint.h"

class CBallisticProlongation
{
public:
    CBallisticProlongation();
    ~CBallisticProlongation();

    Pr_Const            m_PrognConst;   //constants and adjustable parameters for COnePrognosis call
    CBallProl_Output    m_OutData;      //output data
    CGLVector <CBallistProlong_InnerData> m_arProlData; //array of inner data

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::Reset
    //DESCRIPTION	:   Reset class attributes
    //INPUTS		:   None
    //RETURNS		:   None
    void Reset();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::FillInputData
    //DESCRIPTION	:   Fill input data
    //INPUTS		:   Reference to the structure containing input data
    //RETURNS		:   None
    void FillInputData(CBallProl_Input &InData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallisticProlongation::SetPersistentData
    // DESCRIPTION	:   Set persistent data
    // INPUTS		:	Pointer to the structure containing array of tables of EAP;
    //              :   pointer to the structure containing array of tables of ballistic coefficients
    // RETURNS		:	None
    void SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables,
                           Str_Ar_BallCoef_Param_AllTables* p_arBallCoef_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallisticProlongation::SetPersistentData
    // DESCRIPTION	:   Set persistent data
    // INPUTS		:	Pointer to the structure containing array of tables of EAP
    // RETURNS		:	None
    void SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::InitConst
    //DESCRIPTION	:   Initialization of constants
    //INPUTS		:   None
    //RETURNS		:   None
    void InitConst(const Pr_Const &Const);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::InitPointerToBallProcArray
    //DESCRIPTION	:   Copies the pointer to array from CAirBallist_Processing to m_p_Arr_BallProc
    //INPUTS		:   Reference to the array
    //RETURNS		:   None
    void InitPointerToBallProcArray(CAirBall_Proc_Inner_Arr *Arr);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CovMatrixProlongation_FinDiff (prolongation of point is known)
    //DESCRIPTION	:   prolongation of covariance matrix using method of finite differences
    //INPUTS		:   time of initial point, time of last point,
    //              :   DeltaC - increment of initial data for coordinates, DeltaV is the same for velocities
    //              :   position of initial point, velocity components of initial point,
    //              :   position of end point, velocity of end point
    //              :   covariance matrix in initial point (3 coordinates and 3 velocities), ballistic coefficient,
    //              :   sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS		:   covariance matrix in the last poing;
    //              :   returns true if result is ok
    bool CovMatrixProlongation_FinDiff(const qreal t0, const qreal t_end, const qreal DeltaC, const qreal DeltaV,
                                       const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0,
                                       const GLPointDouble3D &Pos_end, const GLPointDouble3D &Vel_end,
                                       const TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse,
                                       TMatrix<DIM_CV> &CovCVRes);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CovMatrixProlongation_FinDiff
    //DESCRIPTION	:   prolongation of covariance matrix using method of finite differences (prolongation of point is unknown)
    //INPUTS		:   time of initial point, time of last point,
    //              :   DeltaC - increment of initial data for coordinates, DeltaV is the same for velocities
    //              :   position of initial point, velocity of initial point,
    //              :   covariance matrix in initial point (3 coordinates and 3 velocities), ballistic coefficient,
    //              :   sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS		:   covariance matrix in the last poing;
    //              :   returns true if result is ok
    bool CovMatrixProlongation_FinDiff(const qreal t0, const qreal t_end, const qreal DeltaC, const qreal DeltaV,
                                       const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0,
                                       const TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse,
                                       TMatrix<DIM_CV> &CovCVRes);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CovMatrixProlongation_Unscented
    //DESCRIPTION	:   prolongation of covariance matrix using method of unscented transform (prolongation of point is known)
    //INPUTS		:   time of initial point, time of last point, position of initial point,
    //              :   velocity components of initial point,
    //              :   position of end point, velocity components of end point,
    //              :   covariance matrix in initial point (3 coordinates and 3 velocities)
    //              :   ballistic coefficient,
    //              :   sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS		:   covariance matrix in the last poing;
    //              :   returns true if result is ok
    bool CovMatrixProlongation_Unscented(const qreal t0, const qreal t_end, const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0,
                                       const GLPointDouble3D &Pos_end, const GLPointDouble3D &Vel_end,
                                       TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse, TMatrix<DIM_CV> &CovCVRes);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CovMatrixProlongation_Unscented
    //DESCRIPTION	:   prolongation of covariance matrix using method of unscented transform (prolongation of point is unknown)
    //INPUTS		:   time of initial point, time of last point, position of initial point,
    //              :   velocity components of initial point,
    //              :   covariance matrix in initial point (3 coordinates and 3 velocities)
    //              :   ballistic coefficient,
    //              :   sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS		:   covariance matrix in the last poing;
    //              :   returns true if result is ok
    bool CovMatrixProlongation_Unscented(const qreal t0, const qreal t_end, const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0,
                                       TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse, TMatrix<DIM_CV> &CovCVRes);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CovMatrixExtrapolation
    //DESCRIPTION	:   Extrapolation of the covariance matrix
    //INPUTS		:   Time of initial point; time of last point;
    //              :   covariance matrix in initial point (3 coordinates and 3 velocities);
    //              :   reference to the resulting covariance matrix in the last point
    //RETURNS		:   True if result is OK
    bool CovMatrixExtrapolation(const qreal t0, const qreal t_end,
                                TMatrix<DIM_CV> &CovCV0, TMatrix<DIM_CV> &CovCVRes);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcStartPoint
    //DESCRIPTION	:   Calculation or recalculation or check and giving of start point
    //INPUTS		:   None
    //RETURNS		:   None
    void CalcStartPoint(CBallProl_Output &ProlOut);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcStartPointByInvProl
    //DESCRIPTION	:   Calculation of start point using inverse prolongation
    //INPUTS		:   None
    //RETURNS		:   Returns true if result is OK
    bool CalcStartPointByInvProl();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcStartPointByTable
    //DESCRIPTION	:   Calculation of start point using table of EAP
    //INPUTS		:   Pointer to the EAP table; sign of single valued BM mark
    //RETURNS		:   None
    void CalcStartPointByTable(Str_Ar_EAP_Param_1Table* pEAP_table, bool bSingleValued);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::EstimationStartPointByDownExtr
    //DESCRIPTION	:   Estimation of start point using reverse extrapolation
    //INPUTS		:   None
    //RETURNS		:   None
    void EstimationStartPointByReverseExtr();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CompareTime
    //DESCRIPTION	:   Comparison of estimated delta start time (i.e. start time minus current time)
    //              :   represented as a complex number, with previously estimated delta start time
    //              :   represented as a real number
    //INPUTS		:   Reference to the delta time represented as a real number;
    //              :   reference to the delta time represented as a complex number
    //RETURNS		:   None
    void CompareTime(GLComplexNumb &dTcompl, qreal &dTreal);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::UpdateApogeeTime
    //DESCRIPTION	:   Update apogee time
    //INPUTS		:   Index of the ballistic object; new value of the apogee time
    //RETURNS		:   None
    void UpdateApogeeTime(qint32 ball_ind, qreal tApogeeNew);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcFallPoint
    //DESCRIPTION	:   Calculation of fall point
    //INPUTS		:   Sign of forced prolongation
    //RETURNS		:   None
    void CalcFallPoint(CBallProl_Output &ProlOut, const bool ForcedProlong=false);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcSepBoostPoint
    //DESCRIPTION	:   Calculation of booster separation point
    //INPUTS		:   Start time, s; time interval from start to booster separation, s;
    //              :   start point; point of the EAP;
    //              :   height of booster separation point, m;
    //              :   distance from the start point to the booster separation point, m;
    //              :   velocity of the ballistic missile at the booster separation point, m/s;
    //              :   reference to the resulting point of booster separation
    //RETURNS		:   True if result is OK
    bool CalcSepBoostPoint(qreal t_start, qreal dt_sep, GLPointDouble3D SP, GLPointDouble3D P_EAP,
                           qreal HBoost, qreal LBoost, qreal VBoost,
                           GLPoint_tXYZVTh &ResBoostSepP);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcPolynomSatellite
    //DESCRIPTION	:   Calculation of the polynom for part of satellite trajectory
    //INPUTS		:   Reference to the structure containing output data
    //RETURNS		:   None
    void CalcPolynomSatellite(CBallProl_Output &OutData);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CheckConditionsForStartPointDownPr
    //DESCRIPTION	:   Check conditions for estimate start point using down prolongation
    //INPUTS		:   None; reference to the resulting sign "counter is exceeded"
    //RETURNS		:   True if it is needed to calculate start point using down prolongation
    bool CheckConditionsForStartPointDownPr(bool &bCounterExceeded);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GetStartPoint
    //DESCRIPTION	:   Calculation or recalculation or check and giving of start point
    //INPUTS		:   Index of ballistic track
    //RETURNS		:   Coordinates of Start point in ECEF; start time
    void GetStartPoint(const qint32 ind_ball, GLPointDouble3D &StartPoint, qreal &StartTime);
    void GetStartPoint(GLPointDouble3D &StartPoint, qreal &StartTime);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GetFallPoint
    //DESCRIPTION	:   Calculation of fall point
    //INPUTS		:   Index of ballistic track
    //RETURNS		:   Coordinates of Fall point in ECEF
    void GetFallPoint(const qint32 ind_ball, GLPointDouble3D &FallPoint, qreal &FallTime);
    void GetFallPoint(GLPointDouble3D &FallPoint, qreal &FallTime);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GetEllipse
    //DESCRIPTION	:   Calculation of ellipse using prolongation data
    //INPUTS		:   Mark of ballistic missile; type of balistic trajectory; current time, s; time of end of prolongation, s;
    //              :   covariance matrix; current coordinates; current velocity components;
    //              :   coordinates of end point; velocity components of end point;
    //              :   sign of large load; ballistic coefficient; sign of inverse prolongaiton;
    //              :   reference to the resulting parameters of dispersion ellipse (includes multiplicator 3 for semiaxes);
    //              :   reference to the resulting covariance matrix
    //RETURNS		:   True if result is OK
    bool CalculateEllipse(const qint16 BMMarkValue, const qint16 TrajType, const qreal CurrTime, const qreal EndTime, GLMatrix &CovMatr,
                    const GLPointDouble3D &CurrPoint, const GLPointDouble3D &CurrVel,
                    const GLPointDouble3D &EndPoint, const GLPointDouble3D &EndVel, const bool SignLargeLoad,
                    const qreal Gamma, const bool SignInvProgn, EllipseInfo &EllInfo, TMatrix<DIM_CV> &CovCVRes);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GetEllipse
    //DESCRIPTION	:   Calculation of ellipse using prolongation data
    //INPUTS		:   Mark of ballistic missile, type of balistic trajectory, current time, time of end of prolongation,
    //              :   covariance matrix, current coordinates, current velocity components,
    //              :   coordinates of end point, velocity components of end point,
    //              :   sign of large load,
    //              :   ballistic coefficient, sign of inverse prolongaiton
    //RETURNS		:   Parameters of dispersion ellipse (includes multiplicator 3 for semiaxes);
    //              :   returns true if result is OK
    bool CalculateEllipse(const qint16 BMMarkValue, const qint16 TrajType, const qreal CurrTime, const qreal EndTime, GLMatrix &CovMatr,
                    const GLPointDouble3D &CurrPoint, const GLPointDouble3D &CurrVel,
                    const GLPointDouble3D &EndPoint, const GLPointDouble3D &EndVel, const bool SignLargeLoad,
                    const qreal Gamma, const bool SignInvProgn, EllipseInfo &EllInfo);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalculateTrapezeAB
    //DESCRIPTION	:   Calculation of "trapeze" parameters for possible aeroballistic missile
    //INPUTS		:   None
    //RETURNS		:   True if result is OK
    bool CalculateTrapezeAB();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CorrectionStartPoint
    //DESCRIPTION	:   Correction of start point
    //INPUTS		:   None
    //RETURNS		:   None
    void CorrectionStartPoint();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CheckStartPoint
    //DESCRIPTION	:   Check of start point
    //INPUTS		:   Start time, s; start point in ECEF, m; covariance matrix in the start point
    //RETURNS		:   True if start point is correct
    bool CheckStartPoint(const qreal &tStart, GLPointDouble3D &SP, TMatrix<DIM_CV> &CovSP);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallisticProlongation::Drop_BallTrack
    // DESCRIPTION	:   Drop ballistic track
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void Drop_BallTrack(const qint32 ind_Ball);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallisticProlongation::SetSignBMMarkChanged
    // DESCRIPTION	:   Sets sign of changed mark of ballistic missile
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void SetSignBMMarkChanged(const qint32 ind_Ball);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::Get50Points
    //DESCRIPTION	:   Returns 50 points of the prolongated trajectory
    //INPUTS		:   Index of the generalized track; index of the track in the array of ballistic tracks;
    //              :   reference to the output data
    //RETURNS		:   None
    void GetOutPoints(qint32 GTInd, qint32 ind_ball, CBallProl_Output &pOutput);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallisticProlongation::ClearInnerData()
    // DESCRIPTION	:   Cleaning inner data containing information about given track
    // INPUTS		:	Index of ballistic track
    // RETURNS		:	None
    void ClearInnerData_1Track(const qint32 ind_Ball);

private:    
    CAirBall_Proc_Inner_Arr*    m_p_Arr_BallProc{nullptr};  //pointer to the working array in CAirBallist_Processing
    CBallProl_Input*            m_pInData{nullptr};         //pointer to the structure containing input data
    COnePrognosis               m_OneProgn;                 //functions of prognostication

    Str_Ar_EAP_Param_AllTables*         m_pEAP_Tables{nullptr};      //pointer to the structure containing EAP Tables
    Str_Ar_BallCoef_Param_AllTables*    m_pBallCoef_Tables{nullptr}; //pointer to the structure containing tables of ballistic coefficients

    FILE* m_pLogEll{nullptr};       //pointer to the log file for ellipse parameters
    FILE* m_pLogCorrSP{nullptr};    //pointer to the log for start point correction

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GlueProlongedTrack
    //DESCRIPTION	:   Glue the new information about prolonged track with the old information
    //INPUTS		:   std::vector containing new prolonged track
    //RETURNS		:   None
    void GlueProlongedTrack(const std::vector<M10> &NewProlTrack);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GetDeltaFinDiff
    //DESCRIPTION	:   Get values of DeltaCoord, DeltaVel used in method of finite differences,
    //              :   depending on BM-Mark
    //INPUTS		:   Mark of ballistic missile (enum BM_Marks), type of ballistic trajectory (enum BallTrajTypes)
    //RETURNS		:   Values of DeltaCoord, DeltaVel used in method of finite differences
    void GetDeltaFinDiff(const qint16 BMMark, const qint16 TrajType, qreal &DeltaCoord, qreal &DeltaVel);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::GetProlPeriod
    //DESCRIPTION	:   Returns period of prolongation, depending on load, on BO type and on margin of time
    //INPUTS		:   None
    //RETURNS		:   Period of prolongation
    qreal GetProlPeriod();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcKGamma
    //DESCRIPTION	:   Returns KGamma coefficient for start point estimation
    //INPUTS		:   Height, m
    //RETURNS		:   Coefficient KGamma for start point estimation
    qreal CalcKGamma(qreal H);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CorrInverseProlTrack
    //DESCRIPTION	:   Correction of inverse prolongated track
    //INPUTS		:   None
    //RETURNS		:   None
    void CorrInverseProlTrack(bool bNewEAP=false, qreal tNewEAP=0, GLPointDouble3D PointNewEAP=GLPointDouble3D(0,0,0));
    void CorrInverseProlTrack_old1(const M10 &SP, const std::vector<M10> &TrackFragment);
    void CorrInverseProlTrack_old(const M10 &SP, const std::vector<M10> &TrackFragment);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CorrInverseProlTrackTStartLessTOld
    //DESCRIPTION	:   Correction of inverse prolongated track when the new start time is less then the old
    //INPUTS		:   Reference to the new start point;
    //              :   reference to the old prolonged track;
    //              :   reference to the fragment of inverse prolongated track;
    //RETURNS		:   None
    bool CorrInverseProlTrackTStartLessTOld(const M10 &SP, const std::vector<M10> &OldTrack, const std::vector<M10> &TrackFragment);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CorrInverseProlTrack_MoveToPoint
    //DESCRIPTION	:   Correction of inverse prolongated track, stitching the prolongated track with base point
    //INPUTS		:   Base point; time to stiching; end time of correction; time for base point; sign of time correction
    //RETURNS		:   None
    void CorrInverseProlTrack_MoveToPoint(GLPointDouble3D &BasePoint, qreal t_0, qreal t_end, qreal t_0_base, bool bTimeCorr);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalculateNforT
    //DESCRIPTION	:   Returns sequence number of element in the array corresponding to the given time moment
    //INPUTS		:   time, s; array of elements of prolonged track
    //RETURNS		:   Sequence number of element in the array corresponding to the given time moment
    qint16 CalculateNforT(qreal tN, const std::vector<M10> &OldTrack);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::Get50Points
    //DESCRIPTION	:   Calculates 50 points of the prolongated trajectory
    //INPUTS		:   Index of the generalized track; index of the ballistic track in the array of ballistic tracks
    //RETURNS		:   None
    void CalcOutPoints(qint32 GTInd, qint32 ind_ball);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::SubstituteOutPoint
    //DESCRIPTION	:   Substitute point in the array of output points by given point
    //INPUTS		:   Time, s; point in ECEF
    //RETURNS		:   None
    void SubstituteOutPoint(const qreal &t0, GLPointDouble3D &Point0);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CBallisticProlongation::CalcDistance
    //DESCRIPTION	:   Calculates distance between start point and fall point
    //INPUTS		:   None
    //RETURNS		:   Distance between start point and fall point
    qreal CalcDistance();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallisticProlongation::ClearInnerData()
    // DESCRIPTION	:   Cleaning inner data containing information about all tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData();
};

#endif // CBALLISTICPROLONGATION_H
