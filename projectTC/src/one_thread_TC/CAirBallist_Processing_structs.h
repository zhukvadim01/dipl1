#ifndef CAIRBALLIST_PROCESSING_STRUCTS_H
#define CAIRBALLIST_PROCESSING_STRUCTS_H

#include <memory.h>
#include <set>
#include "gl/GLMatrix.h"
#include "gl/GLGeometry.h"
#include "gl/GLArray.h"
#include "common/GDEnum.h"
#include "ball_amount_const.h"
#include "air_ballist_constants.h"
#include "CtrlAir_Ballist_structs.h"

// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_InputData_GT
// DESCRIPTION	:   Input data for GT
struct CAirBall_Proc_InputData_GT
{
    qreal               tLoc{0.};       //location time, s
    qint32              IF_GT{-1};      //track index in array (Generalized Track)
    qint32              NumbGT{-1};     //number of generalized track
    qint32              IF_BALL{-1};    //track index in arrays of ballistic tracks
    std::set<qint32>    SetST;          //set of indexes of ST accociated with current GT, in the numeration of AirBallist
    AIR_BALL::s_arCovObj* p_arCovObj{nullptr};   //pointer to the array of covered objects

    GLPointDouble3D     Coord;          //3 coordinates
    GLPointDouble3D     Vel;            //3 velocities
    GLPointDouble3D     Acc;            //3 accelerations
    bool                SignQuickReaction{false}; //sign of quick reaction mode, 0 - normal reaction mode, 1 - quick reaction mode
    bool                Sign_large_load{false}; //true if the load is large (large quantity of ballistic objects)
    bool                bAeroballistic{false}; //sign of aeroballistic object

    GLMatrix            CovMatr;        //Covariance matrix of input data

    qint32              Class{0};       //Class of air object, see enum EClassAO (GDEnum.h)
    qint16              BallType{0};    //type of ballistic object (enum EType in GDEnum.h)
    qint32              ClstNum{0};     //number of the cluster
    Cells_BMMark_Data   *Cells_BMMark{nullptr}; //pointer to the array defining the correspondence between the cells on the plane "H-He" and the BT marks

    CAirBall_Proc_InputData_GT();

    void Reset();

    CAirBall_Proc_InputData_GT& operator=(const CAirBall_Proc_InputData_GT &Q);

    explicit CAirBall_Proc_InputData_GT (const CAirBall_Proc_InputData_GT &Q);
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_InputData_ST
// DESCRIPTION	:   Input data for ST
struct CAirBall_Proc_InputData_ST
{
    qreal           tLoc{0};                //location time, s
    qint32          IF_ST{-1};              //track index in array (Single Track)
    qint32          IF_GT{-1};              //track index in array (General Track) (-1 in the case of absence)
    qint32          IF_BALL{-1};            //track index in arrays of ballistic tracks (-1 in the case of GT absence)
    bool            bMeasured{false};       //true - for measured data processing, false - for smoothed on radar data processing
    bool            bVDopplPresent{false};  //true - Doppler Velocity is present, false - Doppler Velocity is absent

    GLPointDouble3D Coord;                  //3 coordinates
    GLPointDouble3D Vel;                    //3 velocities
    GLPointDouble3D Acc;                    //3 accelerations
    bool            SignQuickReaction{false};   //sign of quick reaction mode, 0 - normal reaction mode, 1 - quick reaction mode
    bool            Sign_large_load{false}; //true if the load is large (large quantity of ballistic objects)

    GLMatrix    CovMatr;                    //Covariance matrix of input data

    qint32      Class{0};                   //Class of air object, see enum EClassAO (GDEnum.h)
    qint16      BallType{0};                //type of ballistic object (enum EType in GDEnum.h)

    qint32      N_StepSmooth{0};            //number of step of smoothing, obtained from filters
    qreal       Prob_IMM_small_coeff{0.};   //probability of filter with small noise coefficient at the output of IMM filter
    qreal       Prob_IMM_large_coeff{0.};   //probability of filter with large noise coefficient at the output of IMM filter
    qreal       R_rdr_obj{0.};              //distance from source to the ballistic object, m
    qreal       VDoppl{0.};                 //doppler velocity, m/s
    qreal       Gamma{0.};                  //ballistic coefficient, m^2/kg
    qreal       RMSE_R{0.};                 //RMSE of range, m
    qreal       RMSE_VDoppl{0.};            //RMSE of Doppler velocity, m/s
    qreal       RMSE_El{0.};                //RMSE of elevation, radians

    CAirBall_Proc_InputData_ST();

    void Reset();

    CAirBall_Proc_InputData_ST& operator=(const CAirBall_Proc_InputData_ST &Q);

    explicit CAirBall_Proc_InputData_ST (const CAirBall_Proc_InputData_ST &Q);
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_OutData
// DESCRIPTION	:   Output data
struct CAirBall_Proc_OutData
{
    bool    bNewPrediction{false};  //sign of new prediction, true - new, false - previous
    bool    bNewPred_InternalUse{false}; //sign of new prediction for internal use in AIR, true - new, false - previous
    bool    bNewSP{false};          //sign of new start point, true - new, false - previous
    bool    bNewPathBranch{false};  //sign of new path or branch, true - new, false - previous
    bool    bNewActPred{false};     //sign of new prediction on active path, true - new, false - previous
    bool    bNewTEAP{false};        //sign of new time of EAP, true - new, false - previous
    bool    bNewSubcl{false};       //sign of new subclass of ballistic, true - new, false - previous
    bool    bNewTrajType{false};    //sign of new trajectory type, true - new, false - previous
    bool    bNewABTrapeze{false};   //sign of new parameters of AB_Trapeze, true - new, false - previous

    CGLArrayFix <TracksPoints, AIR_BALL::N_OUT_POINTS> arrPoints; //array of read-out points (50 points of trajectory)
    Polynoms_data Polynoms;         //data about polynoms of 2nd and 4th and 8th deg
//    Trapeze_Parameters AB_Trapeze; //trapeze parameters for possible aeroballistic missile

//    bool    P_Save_prev_PathBranchData; //true if data about path and branch has not changed
//    qint32     Path; //ballistic trajectory path; see enum BTPaths
//    qint32     Branch; //ballistic trajectory branch; see enum BTBranches

//    qint32     TrajType; //type of trajectory, see enum BallTrajTypes
//    BMMark_AndDiapason  BMMark; //value of BT mark or diapason of values, see enum BM_Marks

    CAirBall_Proc_OutData();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   BMMark_AndDiapason
//DESCRIPTION	:   Structure containing information about BT mark and possible diapasons of BT mark
struct BMMark_AndDiapason
{
    qint16   MarkValue{0};          //value of BM mark, see enum BM_Marks
    qint16   minBMMark{0};          //minimal value of BM mark, possible for current BM
    qint16   maxBMMark{0};          //maximal value of BM mark, possible for current BM
    bool     IsSingleValued{false}; //true if BM mark is defined uniquely

    BMMark_AndDiapason();

    void Reset();

    BMMark_AndDiapason& operator=(const BMMark_AndDiapason &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   BMMark_AndDiapason::IsEmpty()
    // DESCRIPTION	:   Checks whether data is empty
    // INPUTS		:	None
    // RETURNS		:	None
    bool IsEmpty();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   T_U_SU
//DESCRIPTION	:   Structure containing 1-D parameter, time and RMSE of parameter
struct T_U_SU
{
    qreal   t{AIR_BALL::Timer_INI}; //time
    qreal   U{0.};                  //parameter
    qreal   SigU{0.};               //RMSE of parameter

    T_U_SU();

    T_U_SU(qreal _Time, qreal _Par, qreal _SigPar);

    void Reset();

    // PACKAGE		: AirBallist
    // FUNCTION		: operator=()
    // DESCRIPTION	: Assignment operator
    // INPUTS		: Assignming data
    // RETURNS		: Result of assignment
    T_U_SU& operator=(const T_U_SU &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   T_U_SU::IsZero()
    // DESCRIPTION	:   Checks whether current parameters are zeros
    // INPUTS		:	None
    // RETURNS		:	True if current parameters are zeros
    bool IsZero();
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_InnerData_GT
// DESCRIPTION	:   Inner data, for generalized track
struct CAirBall_Proc_InnerData_GT
{
    bool                IsTaken{false};     //true if form is occupied
    qint16              Path{0};            //path of ballistic trajectory; see enum BTPaths
    qint16              Branch{0};          //branch of ballistic trajectory; see enum BTBranches
    qint32              NumbGT{0};          //number of generalizet track in the AIR numeration
    qint32              ClstNum{0};         //number of the cluster
    qint32              IndBall{-1};        //index of the ballistic track in the array of ballistic tracks
    qreal               tLoc{0.};           //location time, s
    qint16              BallType{0};        //type of ballistic object (enum EType in GDEnum.h)
    qreal               tEAP{AIR_BALL::Timer_INI};   //time of the end of active path, s
    qint16              T_EAP_finding_method{AIR_BALL::UNDEFINED_TEAP_METHOD};   //method of EAP time finding, enum AIR_BALL::TimeEAPFinding
    qreal               tPrevPathDet{AIR_BALL::Timer_INI};  //previous time of path determination
    EAP_Param           EAP_parameters;     //parameters of the End of Active path
    GLTrackPointDouble  EAP_Point;          //point of the End of Active path
    GLPoint_tXYZVTh     PointSepBoost1;     //point of the 1st booster separation
    GLPoint_tXYZVTh     PointSepBoost2;     //point of the 2nd booster separation
    qreal               tLastWellSmoothed{AIR_BALL::Timer_INI};  //time of the last well smoothed update
    GLPointDouble3D     Vel;                //vector of velocity, m/s
    GLPointDouble3D     VelPrev;            //previous vector of velocity, m/s
    bool                bAeroballistic{false}; //sign of aeroballistic object
    qreal               t0_Aeroballistic{AIR_BALL::Timer_INI}; //initial time of the aeroballistic maneuver

    BMMark_AndDiapason  BMMark;             //value of BT mark or diapason of values, see enum BM_Marks
    qint16              TrajType{0};        //trajectory type; see enum BallTrajTypes

    GLPointDouble3D     StartPoint;         //start point, in ECEF, m
    GLPointDouble3D     FallPoint;          //fall point, in ECEF, m
    EllipseInfo         StartEll;           //dispersion ellipse in the start point
    EllipseInfo         FallEll;            //dispersion ellipse in the fall point
    qreal               tStart{AIR_BALL::Timer_INI};         //time of start, s
    qreal               tFall{AIR_BALL::Timer_INI};          //predicted time of fall, s

    qreal               D{0.};              //flying distance, m
    qreal               Hapogee{0.};        //height of apogee, m
    qreal               tApogee{AIR_BALL::Timer_INI};         //time of the apogee, s
    qreal               Gamma{0.};          //ballistic coefficient, m^2/kg

    qreal               tFirst{0.};         //time of first point, s
    GLPointDouble3D     FirstPoint;         //Coordinates of the first point of track
    GLPointDouble3D     RMSEs_FirstPoint;   //RMSEs of 1st point by x, y, z

    Active_Pred_Info    ActPredictInfo;     //information about prediction on active phase
    qreal               t_calc_act{0.};     //time of last calculation of Active_Pred_Info for current target (s)

    GLTrapeze           AB_Trapeze;             //"trapeze" for possible aeroballistic missiles (MaRV or QBM)
    qreal               CorrectSigAzPrev{0.};   //previous correct value of azimuth RMSE for "trapeze"
    qreal               tPredictSatellite{AIR_BALL::Timer_INI}; //time of previous prediction for satellite, s
    qreal               PrevTLoc{AIR_BALL::Timer_INI}; //previous location time, s

    T_U_SU              DirPrev;            //time, azimuth of velocity vector (radians) and RMSE of azimuth of velocity vector (radians) for previous update
    T_U_SU              DirPrev_1;          //time, azimuth of velocity vector (radians) and RMSE of azimuth of velocity vector (radians) for (previous-1) update
    T_U_SU              DirPrev_2;          //time, azimuth of velocity vector (radians) and RMSE of azimuth of velocity vector (radians) for (previous-2) update
    qreal               ManTime{AIR_BALL::Timer_INI};   //time of maneuver, s
    GLPointDouble3D     ManPoint;           //point of maneuver

    CAirBall_Proc_InnerData_GT();

    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_InnerData_GT::GTHasCluster()
    // DESCRIPTION	:   Checks whether the generalized track has a cluster
    // INPUTS		:	None
    // RETURNS		:	True if GT has a cluster
    bool GTHasCluster();
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_InnerData_GT
// DESCRIPTION	:   Inner data, for generalized track
struct CAirBall_Proc_InnerData_ST
{
    bool        IsTaken{false};     //true if form is occupied
    qint16      Path{0};            //path of ballistic trajectory; see enum BTPaths
    qint16      Branch{0};          //branche of ballistic trajectory; see enum BTBranches
    qreal       RMSE_mean{0.};      //mean value of RMSE
    qint32      NUpdates{0};        //number of updates of current track
    qint32      N_StepSmooth{0};    //number of step of smoothing, obtained from filters
    bool        bWellSmoothed{false}; //true if current update is well smoothed
    qreal       Gamma{0.};          //ballistic coefficient, m^2/kg
    qreal       tLoc{0.};           //location time, s
    qreal       MeanPeriod{0.};     //mean survey period, s

    CAirBall_Proc_InnerData_ST();

    void Reset();
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_ClstGTData
// DESCRIPTION	:   Inner data for clusters of generalized track
struct CAirBall_Proc_ClstGTData
{
    bool                bBusy{false};               //true if the cluster is busy
    qint32              ParentIndGT{-1};            //index of the parent generalized track in the cluster
    qint32              FstWHInd{-1};               //index of the first warhead
    std::set<qint32>    SetGTInd;                   //set of the indexes of generalized tracks in the cluster
    qreal               tStart{AIR_BALL::Timer_INI}; //start time, s
    GLPointDouble3D     StartPoint;                 //start point
    EllipseInfo         StartEll;                   //ellipse of the start point
    bool                bParentDropped{false};      //true if index of parent GT was dropped
    bool                bArrPointsFilled{false};    //true if initial array of points is filled
    std::vector<TracksPoints>   InitialArrPoints;   //initial array of points, for array of 50 points forming

    CAirBall_Proc_ClstGTData();

    void Reset();
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBallProc_DataForGTNumber
// DESCRIPTION	:   Data assigned for number of GT
struct CAirBall_Proc_DataForGTNumber
{
    qint16  ActionPhase{0};      //action phase, see enum EActionPhaseAO in GDEnum.h
    qreal   GammaSrc{0.};        //ballistic coefficient obtained from source, m^2/kg
    qreal   tObtainActPhase{0.}; //time of obtainement of the action phase

    CAirBall_Proc_DataForGTNumber();

    void Reset();
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   CAirBall_Proc_Inner_Arr
// DESCRIPTION	:   Structure for array of inner data
struct CAirBall_Proc_Inner_Arr
{
    CGLVector <CAirBall_Proc_InnerData_GT> arData_GT; //inner data for GT, by inner index of AIR BALLIST
    CGLVector <CAirBall_Proc_InnerData_ST> arData_ST; //inner data for ST, by inner index of AIR BALLIST

    CGLArrayFix <CAirBall_Proc_ClstGTData, AIR_BALL::CLST_FORM_AMOUNT> arData__Clst; //inner data for the cluster of GT
    CGLVector <CAirBall_Proc_DataForGTNumber> arData__NumbGT; //inner data for GT, by number in the AIR numeration

    CAirBall_Proc_Inner_Arr();

    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::GTIsParentInCluster()
    // DESCRIPTION	:   Checks whether the generalized track is the parent in the cluster
    // INPUTS		:	Index of the generalized track
    // RETURNS		:	True if GT is the parent in the cluster
    bool GTIsParentInCluster(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::GetParentIndGTInCluster()
    // DESCRIPTION	:   Returns index of the parent GT in the cluster (-1 if absents)
    // INPUTS		:	Index of the generalized track
    // RETURNS		:	Index of the parent GT in the cluster (-1 if absents)
    qint32 GetParentIndGTInCluster(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::GTIsFirstWHInCluster()
    // DESCRIPTION	:   Checks whether the generalized track is the first warhead in the cluster
    // INPUTS		:	Index of the generalized track
    // RETURNS		:	True if GT is the first warhead in the cluster
    bool GTIsFirstWHInCluster(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::GetFirstWHIndGTInCluster()
    // DESCRIPTION	:   Returns index of GT of the first warhead in the cluster (-1 if absents)
    // INPUTS		:	Index of the generalized track
    // RETURNS		:	Index of the GT of first warhead in the cluster (-1 if absents)
    qint32 GetFirstWHIndGTInCluster(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::GetStartTimeInCluster()
    // DESCRIPTION	:   Returns start time in the cluster for given index of GT
    // INPUTS		:	Index of the generalized track
    // RETURNS		:	Start time in the cluster
    qreal GetStartTimeInCluster(qint32 indGT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::GetStartPointEllInCluster()
    // DESCRIPTION	:   Returns start point and start point disersion ellipse in the cluster for given index of GT
    // INPUTS		:	Index of the generalized track; reference to the resulting value of start point;
    //              :   reference to the reslting value of dispersion ellipse in the start point
    // RETURNS		:	None
    void GetStartPointEllInCluster(qint32 indGT, GLPointDouble3D &ResStartPoint, EllipseInfo &ResStartEll);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::DetermineParentInClst()
    // DESCRIPTION	:   Determines index of parent GT in the cluster
    // INPUTS		:	Number of the cluster
    // RETURNS		:	None
    void DetermineParentInClst(qint32 ClstNum);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CAirBall_Proc_Inner_Arr::DetermineFstWHInClst()
    // DESCRIPTION	:   Determines index of GT of the first warhead in the cluster
    // INPUTS		:	Number of the cluster
    // RETURNS		:	None
    void DetermineFstWHInClst(qint32 ClstNum);
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   TrajParamBall
// DESCRIPTION	:   Trajectory parameters
struct TrajParamBall
{
    qreal  H{0.};      //height, m
    qreal  V{0.};      //velocity, m/s
    qreal  Vh{0.};     //vertical velocity, m/s
    qreal  He{0.};     //energetic height, m
    qreal  ah{0.};     //vertical acceleration, m/s^2
    qreal  a{0.};      //absolute acceleration, m/s^2
    qreal  ah_noC{0.}; //vertical acceleration calculated with subtraction of Coriolis acceleration, m/s^2

    qreal  SigH{0.};   //RMSE of height, m
    qreal  SigV{0.};   //RMSE of velocity, m/s
    qreal  SigVh{0.};  //RMSE of vertical velocity, m/s
    qreal  SigHe{0.};  //RMSE of energetic height, m
    qreal  SigAh{0.};  //RMSE of vertical acceleration, m/s^2
    qreal  SigA{0.};   //RMSE of absolute acceleration, m/s^2

    TrajParamBall();

    void Reset();

    TrajParamBall& operator=(const TrajParamBall &Q);
};

#endif // CAIRBALLIST_PROCESSING_STRUCTS_H
