#ifndef CTRLAIR_BALLIST_STRUCTS_H
#define CTRLAIR_BALLIST_STRUCTS_H

#include <memory.h>
#include <map>
#include <vector>
#include <set>

#include "air_ballist_constants.h"
#include "ball_amount_const.h"

#include "gl/GLArray.h"
#include "gl/GLGeometry.h"
#include "gl/GLMatrix.h"
#include "gl/GLTrapeze.h"
#include "gl/GLTrackPoint.h"

#include <QRecursiveMutex>
#include "conversion/Geodesic.h"

#include "common/GDEnum.h"

//PACKAGE       :   AirBallist
//STRUCTURE     :   Cells_BMMark_Data
//DESCRIPTION   :   structure for array of cells with data about BM Marks
struct Cells_BMMark_Data
{
    CGLArrayFix<qint32, AIR_BALL::NUMB_CELLS> ArrCells; //array defining the correspondence between the cells on the plane "H-He" and the BT marks
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CtrlAIR_BALL_Input_GT_Data
//DESCRIPTION   :   Input data of GT for ballistic processing task
struct CtrlAIR_BALL_Input_GT_Data
{
    qint16      TrackingSign{0};    //state of track data, see enum ETrackingSign (in GD)
//    qint32     ID;                //index of GT
    qint32      NumbGT{0};          //number of GT
    qreal       tLoc{0.};           //location time, s
    std::map<qint16, qint32> SetAssocST; //set of ST associated with current GT;
                        //1st field is index of source, 2nd field is ST number in the numeration of source

    qint16      Class{0};      //class of air object, enum EClassAO (in GD)
    qint16      Type{0};       //type of object, see enum EType (in GD)
    qint32      ClstNum{0};    //number of the cluster
    bool        SignQuickReaction{false}; //sign of quick reaction mode, 0 - normal reaction mode, 1 - quick reaction mode
    bool        bAeroballistic{false}; //sign of aeroballistic object

    qreal       X{0.};          //X coordinate, m
    qreal       Y{0.};          //Y coordinate, m
    qreal       Z{0.};          //Z coordinate, m

    qreal       VX{0.};         //VX velocity component, m/s
    qreal       VY{0.};         //VY velocity component, m/s
    qreal       VZ{0.};         //VZ velocity component, m/s

    qreal       AX{0.};         //AX acceleration component, m/s^2
    qreal       AY{0.};         //AY acceleration component, m/s^2
    qreal       AZ{0.};         //AZ acceleration component, m/s^2

    qreal       SigX{0.};       //RMSE of X, m
    qreal       SigY{0.};       //RMSE of Y, m
    qreal       SigZ{0.};       //RMSE of Z, m
    qreal       SigVX{0.};      //RMSE of VX, m/s
    qreal       SigVY{0.};      //RMSE of VY, m/s
    qreal       SigVZ{0.};      //RMSE of VZ, m/s
    qreal       SigAX{0.};      //RMSE of AX, m/s^2
    qreal       SigAY{0.};      //RMSE of AY, m/s^2
    qreal       SigAZ{0.};      //RMSE of AZ, m/s^2

    qreal       KXY{0.};        //covariance coefficient XY, m^2
    qreal       KXZ{0.};        //covariance coefficient XZ, m^2
    qreal       KYZ{0.};        //covariance coefficient YZ, m^2
    qreal       KVxVy{0.};      //covariance coefficient VxVy, m^2/s^2
    qreal       KVxVz{0.};      //covariance coefficient VxVz, m^2/s^2
    qreal       KVyVz{0.};      //covariance coefficient VyVz, m^2/s^2
    qreal       KAxAy{0.};      //covariance coefficient AxAy, m^2/s^4
    qreal       KAxAz{0.};      //covariance coefficient AxAz, m^2/s^4
    qreal       KAyAz{0.};      //corariance coefficient AyAz, m^2/s^4
    qreal       KXVx{0.};       //covariance coefficient XVx, m^2/s
    qreal       KXVy{0.};       //covariance coefficient XVy, m^2/s
    qreal       KXVz{0.};       //covariance coefficient XVz, m^2/s
    qreal       KXAx{0.};       //covariance coefficient XAx, m^2/s^2
    qreal       KXAy{0.};       //covariance coefficient XAy, m^2/s^2
    qreal       KXAz{0.};       //covariance coefficient XAz, m^2/s^2
    qreal       KYVx{0.};       //covariance coefficient YVx, m^2/s
    qreal       KYVy{0.};       //covariance coefficient YVy, m^2/s
    qreal       KYVz{0.};       //covariance coefficient YVz, m^2/s
    qreal       KYAx{0.};       //covariance coefficient YAx, m^2/s^2
    qreal       KYAy{0.};       //covariance coefficient YAy, m^2/s^2
    qreal       KYAz{0.};       //covariance coefficient YAz, m^2/s^2
    qreal       KZVx{0.};       //covariance coefficient ZVx, m^2/s
    qreal       KZVy{0.};       //covariance coefficient ZVy, m^2/s
    qreal       KZVz{0.};       //covariance coefficient ZVz, m^2/s
    qreal       KZAx{0.};       //covariance coefficient ZAx, m^2/s^2
    qreal       KZAy{0.};       //covariance coefficient ZAy, m^2/s^2
    qreal       KZAz{0.};       //covariance coefficient ZAz, m^2/s^2
    qreal       KVxAx{0.};      //covariance coefficient VxAx, m^2/s^3
    qreal       KVxAy{0.};      //covariance coefficient VxAy, m^2/s^3
    qreal       KVxAz{0.};      //covariance coefficient VxAz, m^2/s^3
    qreal       KVyAx{0.};      //covariance coefficient VyAx, m^2/s^3
    qreal       KVyAy{0.};      //covariance coefficient VyAy, m^2/s^3
    qreal       KVyAz{0.};      //covariance coefficient VyAz, m^2/s^3
    qreal       KVzAx{0.};      //covariance coefficient VzAx, m^2/s^3
    qreal       KVzAy{0.};      //covariance coefficient VzAy, m^2/s^3
    qreal       KVzAz{0.};      //covariance coefficient VzAz, m^2/s^3

    qint32      AssignTgtNum{0};//number of assigned target, data for interceptor

    CtrlAIR_BALL_Input_GT_Data() = default;

    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALL_Input_GT_Data::operator=
    // DESCRIPTION	:   Assignment operator
    // INPUTS		:	CtrlAIR_BALL_Input_GT_Data
    // RETURNS		:	CtrlAIR_BALL_Input_GT_Data
    CtrlAIR_BALL_Input_GT_Data& operator=(const CtrlAIR_BALL_Input_GT_Data &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALL_Input_GT_Data::CtrlAIR_BALL_Input_GT_Data
    // DESCRIPTION	:   Copy constructor
    // INPUTS		:	CtrlAIR_BALL_Input_GT_Data
    // RETURNS		:	CtrlAIR_BALL_Input_GT_Data
    explicit CtrlAIR_BALL_Input_GT_Data (const CtrlAIR_BALL_Input_GT_Data &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALL_Input_GT_Data::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file;
    //              :   sign of simplified output
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, bool bSimplifiedOut=false);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CtrlAIR_BALL_Input_ST_Data
//DESCRIPTION   :   Input data of ST for ballistic processing task
struct CtrlAIR_BALL_Input_ST_Data
{
    qint16      TrackingSign{0};        //state of track data, see enum ETrackingSign (in GD)
    qint16      SrcInd{0};              //index of source
    qint32      NtrInSrc{0};            //number of trace in the numeration of source
    qint32      NumGT{0};               //SC4I-number of corresponding GT (-1 if ST is not associated with GT)
    qreal       tLoc{0.};               //location time, s
    bool        bMeasured{false};       //true - for measured data processing, false - for smoothed on radar data processing
    bool        bVDopplPresent{false};  //true - Doppler Velocity is present, false - Doppler Velocity is absent
    bool        SignQuickReaction{false}; //sign of quick reaction mode, 0 - normal reaction mode, 1 - quick reaction mode

    GLVector    VectParam;              //vector of (smoothed) coordinate parameters (0-x, 1-y, 2-z, 3-vx, 4-vy, 5-vz, 6-ax, 7-ay, 8-az)
    GLMatrix    CovMatr;                //covariance matrix of VectParam

    qint32      N_StepSmooth{0};        //number of step of smoothing, obtained from filters
    qreal       GammaFlt{0.};           //ballistic coefficient obtained from Air Filter, m^2/kg
    qreal       R{0.};                  //range, m
    qreal       VDoppl{0.};             //doppler velocity, m/s
    qreal       RMSE_R{0.};             //RMSE of range, m
    qreal       RMSE_VDoppl{0.};        //RMSE of Doppler velocity, m/s
    qreal       RMSE_El{0.};            //RMSE of elevation, radians

    qint16      Class{0};               //class of air object, enum EClassAO (in GD)
    qint16      Type{0};                //type of object, see enum EType (GDEnum.h)

    qreal       Prob_KF_3D_222_small{0.}; // | - probabilities from IMM-filter output,
    qreal       Prob_KF_3D_222_large{0.}; // |   performed for measured data

    CtrlAIR_BALL_Input_ST_Data();

    void Reset();

    CtrlAIR_BALL_Input_ST_Data& operator=(const CtrlAIR_BALL_Input_ST_Data &Q);

    explicit CtrlAIR_BALL_Input_ST_Data (const CtrlAIR_BALL_Input_ST_Data &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALL_Input_ST_Data::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; sign of simplified output
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, bool bSimplifiedOut=false);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CtrlAIR_BALL_Input_GT_Data
//DESCRIPTION   :   Input test data for ballistic processing task
struct  CtrlAIR_BALL_traj_param
{
    qint32  ID{0};          //index of ST in numeration of AIR
    qreal   tLoc{0.};       //location time, s

    bool    Sign_sufficient_time{false}; //sign of availability of sufficient time for information processing

    qreal   X{0.};          //X coordinate, m
    qreal   Y{0.};          //Y coordinate, m
    qreal   Z{0.};          //Z coordinate, m

    qreal   VX{0.};         //VX velocity component, m/s
    qreal   VY{0.};         //VY velocity component, m/s
    qreal   VZ{0.};         //VZ velocity component, m/s

    qreal   AX{0.};         //AX acceleration component, m/s^2
    qreal   AY{0.};         //AY acceleration component, m/s^2
    qreal   AZ{0.};         //AZ acceleration component, m/s^2

    qint16  Class{0};       //class of air object, see enum AOClasses
    qint16  Path{0};        //path of ballistic trajectory, see enum BT_Paths
    qint16  BallSubclass{0}; //subclass of ballistic object (enum EType in GDEnum.h)

    qreal   SigX{0.};       //RMSE of X, m
    qreal   SigY{0.};       //RMSE of Y, m
    qreal   SigZ{0.};       //RMSE of Z, m
    qreal   SigVX{0.};      //RMSE of VX, m/s
    qreal   SigVY{0.};      //RMSE of VY, m/s
    qreal   SigVZ{0.};      //RMSE of VZ, m/s
    qreal   SigAX{0.};      //RMSE of AX, m/s^2
    qreal   SigAY{0.};      //RMSE of AY, m/s^2
    qreal   SigAZ{0.};      //RMSE of AZ, m/s^2

    qreal   KXY{0.};        //covariance coefficient XY, m^2
    qreal   KXZ{0.};        //covariance coefficient XZ, m^2
    qreal   KYZ{0.};        //covariance coefficient YZ, m^2
    qreal   KVxVy{0.};      //covariance coefficient VxVy, m^2/s^2
    qreal   KVxVz{0.};      //covariance coefficient VxVz, m^2/s^2
    qreal   KVyVz{0.};      //covariance coefficient VyVz, m^2/s^2
    qreal   KAxAy{0.};      //covariance coefficient AxAy, m^2/s^4
    qreal   KAxAz{0.};      //covariance coefficient AxAz, m^2/s^4
    qreal   KAyAz{0.};      //corariance coefficient AyAz, m^2/s^4
    qreal   KXVx{0.};       //covariance coefficient XVx, m^2/s
    qreal   KXVy{0.};       //covariance coefficient XVy, m^2/s
    qreal   KXVz{0.};       //covariance coefficient XVz, m^2/s
    qreal   KXAx{0.};       //covariance coefficient XAx, m^2/s^2
    qreal   KXAy{0.};       //covariance coefficient XAy, m^2/s^2
    qreal   KXAz{0.};       //covariance coefficient XAz, m^2/s^2
    qreal   KYVx{0.};       //covariance coefficient YVx, m^2/s
    qreal   KYVy{0.};       //covariance coefficient YVy, m^2/s
    qreal   KYVz{0.};       //covariance coefficient YVz, m^2/s
    qreal   KYAx{0.};       //covariance coefficient YAx, m^2/s^2
    qreal   KYAy{0.};       //covariance coefficient YAy, m^2/s^2
    qreal   KYAz{0.};       //covariance coefficient YAz, m^2/s^2
    qreal   KZVx{0.};       //covariance coefficient ZVx, m^2/s
    qreal   KZVy{0.};       //covariance coefficient ZVy, m^2/s
    qreal   KZVz{0.};       //covariance coefficient ZVz, m^2/s
    qreal   KZAx{0.};       //covariance coefficient ZAx, m^2/s^2
    qreal   KZAy{0.};       //covariance coefficient ZAy, m^2/s^2
    qreal   KZAz{0.};       //covariance coefficient ZAz, m^2/s^2
    qreal   KVxAx{0.};      //covariance coefficient VxAx, m^2/s^3
    qreal   KVxAy{0.};      //covariance coefficient VxAy, m^2/s^3
    qreal   KVxAz{0.};      //covariance coefficient VxAz, m^2/s^3
    qreal   KVyAx{0.};      //covariance coefficient VyAx, m^2/s^3
    qreal   KVyAy{0.};      //covariance coefficient VyAy, m^2/s^3
    qreal   KVyAz{0.};      //covariance coefficient VyAz, m^2/s^3
    qreal   KVzAx{0.};      //covariance coefficient VzAx, m^2/s^3
    qreal   KVzAy{0.};      //covariance coefficient VzAy, m^2/s^3
    qreal   KVzAz{0.};      //covariance coefficient VzAz, m^2/s^3

    qreal   Center_Lat{0.}; //Latitude of TPCS center, radians
    qreal   Center_Long{0.};//Longitude of TPCS center, radians
    qreal   Center_H{0.};   //Altitude of TPCS center, meters

    qreal   Prob_IMM_small_coeff{0.};       //probability of filter with small noise coefficient at the output of IMM filter
    qreal   Prob_IMM_large_coeff{0.};       //probability of filter with large noise coefficient at the output of IMM filter
    qreal   R_rdr_obj{0.};                  //distance from source to the ballistic object, m
    Cells_BMMark_Data Cells_BMMark_diap1;   //array defining the correspondence between the cells on the plane "H-He" and the BT marks in diapason 1 of RMSEs
    Cells_BMMark_Data Cells_BMMark_diap2;   //array defining the correspondence between the cells on the plane "H-He" and the BT marks in diapason 2 of RMSEs
    Cells_BMMark_Data Cells_BMMark_diap3;   //array defining the correspondence between the cells on the plane "H-He" and the BT marks in diapason 3 of RMSEs

    CtrlAIR_BALL_traj_param();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   TracksPoints
//DESCRIPTION   :   Structure for points of prolongated trajectory
struct TracksPoints
{
    GLPointDouble3D     P;              //coordinates
    GLPointDouble3D     V;              //velocity components
    GLPointDouble3D     A;              //acceleration components
    qreal               time_point{0.};	//time

    TracksPoints();

    void Reset();

    TracksPoints& operator=(const TracksPoints &Q);

    TracksPoints (const TracksPoints &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   TracksPoints::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Polynoms_data
//DESCRIPTION   :   Structure for polynoms data
struct Polynoms_data
{
    GLPointDouble3D     Part0_4deg;         //coefficients to 0 degree in the polynom of 4 degree
    GLPointDouble3D     Part1_4deg;         //coefficients to 1 degree in the polynom of 4 degree
    GLPointDouble3D     Part2_4deg;         //coefficients to 2 degree in the polynom of 4 degree
    GLPointDouble3D     Part3_4deg;         //coefficients to 3 degree in the polynom of 4 degree
    GLPointDouble3D     Part4_4deg;         //coefficients to 4 degree in the polynom of 4 degree

    GLPointDouble3D     Part0_8deg;         //coefficients to 0 degree in the polynom of 8 degree
    GLPointDouble3D     Part1_8deg;         //coefficients to 1 degree in the polynom of 8 degree
    GLPointDouble3D     Part2_8deg;         //coefficients to 2 degree in the polynom of 8 degree
    GLPointDouble3D     Part3_8deg;         //coefficients to 3 degree in the polynom of 8 degree
    GLPointDouble3D     Part4_8deg;         //coefficients to 4 degree in the polynom of 8 degree
    GLPointDouble3D     Part5_8deg;         //coefficients to 5 degree in the polynom of 8 degree
    GLPointDouble3D     Part6_8deg;         //coefficients to 6 degree in the polynom of 8 degree
    GLPointDouble3D     Part7_8deg;         //coefficients to 7 degree in the polynom of 8 degree
    GLPointDouble3D     Part8_8deg;         //coefficients to 8 degree in the polynom of 8 degree

    GLPointDouble3D     Part0_2deg;         //coefficients to 0 degree in the polynom of 2 degree
    GLPointDouble3D     Part1_2deg;         //coefficients to 1 degree in the polynom of 2 degree
    GLPointDouble3D     Part2_2deg;         //coefficients to 2 degree in the polynom of 2 degree

    qreal               AnchorTime_4deg{0.}; //s, anchor time for polynom of 4 degree; Polynom(t) = (sum)Part(n)*(t-AnchorTime)^n
    qreal               AnchorTime_8deg{0.}; //s, anchor time for polynom of 8 degree
    qreal               AnchorTime_2deg{0.}; //s, anchor time for polynom of 2 degree
    qreal               TimeChange{0.};      //s, time of model change from 4 to 8 degree

    Polynoms_data();

    void Reset();

    Polynoms_data& operator=(const Polynoms_data &Q);

    Polynoms_data (const Polynoms_data &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Polynoms_data::LogPolynom()
    // DESCRIPTION	:   Output polynom coefficient to the log file
    // INPUTS		:	None
    // RETURNS		:	None
    bool LogPolynom();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Polynoms_data::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; name of string
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, const char* _Name);

};


//PACKAGE       :   AirBallist
//STRUCTURE     :   EllipseInfo
//DESCRIPTION   :   Structure for parameters of dispersion ellipse
struct EllipseInfo
{
    qreal	aI{0.};		//half of large axis (already multiplied by 3)
    qreal	bI{0.};		//half of small axis (already multiplied by 3)
    qreal	BetaI{0.};	//azimuth of large axis, in radians

    EllipseInfo();

    EllipseInfo (const EllipseInfo &Q);

    void Reset();

    EllipseInfo& operator=(const EllipseInfo &Q);

    //PACKAGE		: AirBallist
    //FUNCTION		: EllipseInfo::IsZero
    //DESCRIPTION	: Checks whether ellipse is zero
    //INPUTS		: None
    //RETURNS		: True if ellipse is zero
    bool IsZero();
};

struct M10
{
    qreal   M_info[10]; //array contains t, X, Y, Z, VX, VY, VZ, AX, AY, AZ

    M10();

    M10(const M10 &Q);

    void Reset();

    M10& operator=(const M10 &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   M10::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   BT_EAP_Param
//DESCRIPTION   :   Parameters from table of characteristics of end active leg
struct EAP_Param
{
    enum Param_Ownership  //ownership of table parameters
    {
        EAP_POINT         = 1, //parameters of the end of active path
        BOOST1_SEP_POINT  = 2, //parameters of the point of 1st booster separation
        BOOST2_SEP_POINT  = 3  //parameters of the point of 2nd booster separation
    };

    qreal   theta{0.};    //angle of departure (throw angle), degree
    qreal   H{0.};        //height, m
    qreal   L{0.};        //distance along Earth surface from start point to the EAP point, m
    qreal   V{0.};        //velocity, m/s

    EAP_Param();

    void Reset();

    EAP_Param& operator=(const EAP_Param &Q);

    explicit EAP_Param (const EAP_Param &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   EAP_Param::IsEmpty()
    // DESCRIPTION	:   Checks wheather current data is empty
    // INPUTS		:	None
    // RETURNS		:	True if current data is empty
    bool IsEmpty();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   EAP_Param::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; mark of ballistic missile (enum AIR_BALL::BM_Marks);
    //              :   sign of the ownership of table parameters (enum Param_Ownership)
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, qint16 BallMark, qint16 Ownership);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Predict_Info
//DESCRIPTION   :   Structure for parameters of prolongation
struct Predict_Info
{
    qreal           tStart{0.};     //start time, s
    qreal           tFall{0.};      //fall time, s
    GLPointDouble3D StartPoint;     //start point
    GLPointDouble3D FallPoint;      //fall point
    EllipseInfo     StartEll;       //dispersion ellipse in start point
    EllipseInfo     FallEll;        //dispersion ellipse in fall point
    Polynoms_data   Pol;            //information about polynoms
    qreal           D{0.};          //flying distance, m
    qreal           Hapogee{0.};    //height of apogee, m
    qreal           Gamma{0.};      //ballistic coefficient, m^2/kg
//    qreal  Theta; //angle of departure, degrees
    CGLArrayFix <TracksPoints, AIR_BALL::N_OUT_POINTS> arrPoints; //array of read-out points (50 points of trajectory)
//    std::vector<GLTrackPointDouble> ProlTrackFragm; //fragment of prolonged track

    Predict_Info();

    void Reset();

    Predict_Info& operator=(const Predict_Info &Q);

    explicit Predict_Info (const Predict_Info &Q);


    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Predict_Info::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file;
    //              :   sign of 50 points output; sign of polynom output
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, bool bOut50points, bool bOutPolynom);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Predict_Info::LogStartPoint()
    // DESCRIPTION	:   Output start point to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogStartPoint(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Active_Pred_Info
//DESCRIPTION   :   Structure for calculated information on active leg
struct Active_Pred_Info
{
    qint16              New_ActPrognoz{0};	//sign of prognostication on Active Leg (0 - old, 1 - new, 2 - unreliable)
    GLTrapeze           Trapeze;            //parameters of "trapeze"
    GLPointDouble3D     EstStartPoint;      //estimated start point (m, GSC)
    GLPointDouble3D     EstFallPoint;       //estimated fall point (m, GSC)
    qreal               TimeStartEst{AIR_BALL::Timer_INI};   //estimated start time, seconds from the day's beginning
    std::set<qint32>    NumsThreat;         //numbers of threatened covered objects

//    qreal			ThetaCalc;		//Calculated Throw Angle

    Active_Pred_Info();

    void Reset();

    Active_Pred_Info& operator=(const Active_Pred_Info &Q);

    explicit Active_Pred_Info (const Active_Pred_Info &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Active_Pred_Info::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CtrlAIR_BALL_Input_GT_Data
//DESCRIPTION   :   Output data of GT for ballistic processing task
struct CtrlAIR_BALL_Output_GT_Data
{
    bool        bNewPrediction{false};          //sign of new prediction, true - new, false - previous
    bool        bNewPred_InternalUse{false};    //sign of new prediction for internal use in AIR, true - new, false - previous
    bool        bNewSP{false};                  //sign of new start point, true - new, false - previous
    bool        bNewPathBranch{false};          //sign of new path or branch, true - new, false - previous
    bool        bNewActPred{false};             //sign of new prediction on active path, true - new, false - previous
    bool        bNewTEAP{false};                //sign of new time of EAP, true - new, false - previous
    bool        bNewSubcl{false};               //sign of new subclass of ballistic, true - new, false - previous
    bool        bNewTrajType{false};            //sign of new trajectory type, true - new, false - previous
    bool        bNewABData{false};              //sign of new data for possible aeroballistic object, true - new, false - previous
    bool        bNewPolynomSat{false};          //sign of new polynom for satellite

    qint16      Path{0};                        //path of trajectory (enum EBallisticPathSign in GDEnum.h)
    qint16      Branch{0};                      //branch of trajectory (enum EBallisticBranchSign in GDEnum.h, or NONE)
    qreal       tEAP{0.};                       //time of the end of active path, s
    qint16      TrajType{0};                    //type of ballistic trajectory (enum BallTrajTypes in air_ballist_literal.h)

    qint16      BallSubclass{0};                //subclass of ballistic missile (enum ESubClassBT in GDEnum.h)
    qint16      minBallSubclass{0};             //minimum value of subclass of ballistic missile (enum ESubClassBT in GDEnum.h)
    qint16      maxBallSubclass{0};             //maximum value of subclass of ballistic missile (enum ESubClassBT in GDEnum.h)
    bool        bSubclIsSingleValued{false};    //true if minBallSubclass == maxBallSubclass == BallSubclass
    qreal       FlightTimeMax{0.};              //maximum possible flight time from launch to fall, s

    Predict_Info        Predict;                //information about prediction
    Active_Pred_Info    ActPred;                //information about prediction on active path
    EAP_Param           EAP_parameters;         //parameters of the End of Active path
    GLTrackPointDouble  EAP_Point;              //point of the End of Active path
    GLPoint_tXYZVTh     PointSepBoost1;         //point of the 1st booster separation
    GLPoint_tXYZVTh     PointSepBoost2;         //point of the 2nd booster separation

    bool        bPossibleMaRV{false};           //true if ballistic warhead is possible MaRV
    bool        bPossibleQBM{false};            //true if ballistic warhead is possible QBM
    GLTrapeze   AB_Trapeze;                     //"trapeze" for possible aeroballistic missiles (MaRV or QBM)

    QRecursiveMutex	CritSect;                   //Critical section

    CtrlAIR_BALL_Output_GT_Data();

    void Reset();

    CtrlAIR_BALL_Output_GT_Data& operator=(const CtrlAIR_BALL_Output_GT_Data &Q);

    explicit CtrlAIR_BALL_Output_GT_Data (const CtrlAIR_BALL_Output_GT_Data &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALL_Output_GT_Data::AnyChanges()
    // DESCRIPTION	:   Checks whether output data contains the changes
    // INPUTS		:	None
    // RETURNS		:	True in the case of any change
    bool AnyChanges();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALL_Output_GT_Data::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; GT number in the SC4I enumeration;
    //              :   GT index in the AirBallist enumeration; location time;
    //              :   sign of 50 points output, sign of polynom output
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, qint32 NumGT, qint32 ID_GT, qreal tLoc, bool bOut50points, bool bOutPolynom);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CtrlAIR_BALL_Inner_Data
//DESCRIPTION   :   Inner data of GT for CtrlAIR_Ballist task
struct CtrlAIR_BALL_Inner_Data_GT
{
    qreal    t_waiting_begin{AIR_BALL::Timer_INI};  //time of begin of waiting before drop reclassified (to non-BT) track
    qint16   Class{0};                              //class of object, enum EClassAO in GDEnum.h

    CtrlAIR_BALL_Inner_Data_GT();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Str_Ar_BT_EAP_Param_1Table
//DESCRIPTION   :   Structure for array of characteristics of the end of active path for one BT-subclass
struct Str_Ar_EAP_Param_1Table
{
    CGLArrayFix <EAP_Param, AIR_BALL::N_ELEM_KAU_PARAM> Items;
    CGLArrayFix <EAP_Param, AIR_BALL::N_ELEM_KAU_PARAM> ItemsBoost1;
    CGLArrayFix <EAP_Param, AIR_BALL::N_ELEM_KAU_PARAM> ItemsBoost2;

    qint16      BallSubclass{0};        //subclass of ballistic target (ESubClassBT from GDEnum.h)
    qint16      BallMark_inner{0};      //mark of ballistic target (BM_Marks from air_ballist_literal.h); do not fill in the external tacks
    qreal       t_EAP{0.};              //time of the end active leg, s
    qreal       t_SepBoost1{0.};        //time ot the 1st booster separation, s
    qreal       t_SepBoost2{0.};        //time of the 2nd booster separation, s
    qint16      QuantityBoost{0};       //quanity of boosters
    qreal       Gamma{0.};              //average value of ballistic coefficient
    qint16      N_el{0};                //quantity of filled elements in array Items; do not fill if AddRow() function is used
    qint16      N_el_Boost1{0};         //quantity of filled elements in array Items; do not fill if AddRow() function is used
    qint16      N_el_Boost2{0};         //quantity of filled elements in array Items; do not fill if AddRow() function is used

    Str_Ar_EAP_Param_1Table();

    void Reset();

    Str_Ar_EAP_Param_1Table& operator=(const Str_Ar_EAP_Param_1Table &Q);

    explicit Str_Ar_EAP_Param_1Table (const Str_Ar_EAP_Param_1Table &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_1Table::AddRow()
    // DESCRIPTION	:   Inserts new row to the table of EAP parameters
    // INPUTS		:	Elements of new row
    // RETURNS		:	None
    void AddRow(const EAP_Param &NewRow);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_1Table::AddRowBoost1()
    // DESCRIPTION	:   Inserts new row to the table of 1st booster separation point parameters
    // INPUTS		:	Elements of new row
    // RETURNS		:	None
    void AddRowBoost1(const EAP_Param &NewRow);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_1Table::AddRowBoost2()
    // DESCRIPTION	:   Inserts new row to the table of 2nd booster separation point parameters
    // INPUTS		:	Elements of new row
    // RETURNS		:	None
    void AddRowBoost2(const EAP_Param &NewRow);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_1Table::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Str_Ar_BT_EAP_Param_AllTables
//DESCRIPTION   :   Structure for array of characteristics of the end active leg for all BT-subclasses
struct Str_Ar_EAP_Param_AllTables
{
    CGLArrayFix <Str_Ar_EAP_Param_1Table, AIR_BALL::N_BALL_SUBCL> arData; //array of tables of the end active paths for all BT-types

    qint16 N_el{0}; //Number of filled elements in the array; do not fill if AddTable() function is used

    Str_Ar_EAP_Param_AllTables();

    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_AllTables::getCell
    // DESCRIPTION	:   Returns pointer to the cell containing table of EAP for given ballistic subclass
    // INPUTS		:	Ballistic subclass, enum ESubClassBT from GDEnum.h
    // RETURNS		:	Pointer to the cell containing table of EAP
    Str_Ar_EAP_Param_1Table* getCell(const qint16 BallSubcl);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_AllTables::getCellMark
    // DESCRIPTION	:   Returns pointer to the cell containing table of EAP for given mark of ballistic missile
    // INPUTS		:	Ballistic subclass, enum BM_Marks from air_ballist_literal.h
    // RETURNS		:	Pointer to the cell containing table of EAP
    Str_Ar_EAP_Param_1Table* getCellMark(const qint16 BallMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_AllTables::AddTable()
    // DESCRIPTION	:   Add a new table
    // INPUTS		:	Structure containing new table
    // RETURNS		:	None
    void AddTable(const Str_Ar_EAP_Param_1Table &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_EAP_Param_AllTables::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Str_Ar_Gamma_Param_1Table
//DESCRIPTION   :   Structure for arrays of table characteristics to determine ballistic coefficient for one BT-subclass
struct Str_Ar_BallCoef_Param_1Table
{
    CGLArrayFix<qreal, AIR_BALL::N_H_DIAP_GAMMA>    H_Diapasons;    //array of boundary values of diapasons by height
    CGLArrayFix<qreal, AIR_BALL::N_MACH_DIAP_GAMMA> Mach_Diapasons; //array of boundary values of diapasons by Mach number
    CGLArrayFix<CGLArrayFix<qreal, AIR_BALL::N_H_DIAP_GAMMA>, AIR_BALL::N_MACH_DIAP_GAMMA> GammaValues; //array of Ballistic Coefficient values

    qint16      BallSubclass{0};     //subclass of ballistic target (ESubClassBT from GDEnum.h)
    qint16      BallMark_inner{0};   //mark of ballistic target (BM_Marks from air_ballist_literal.h)
    qint16      N_H_diap{0};         //quantity of diapasones on height
    qint16      N_Mach_diap{0};      //quantity of diapasones on Mach number

    Str_Ar_BallCoef_Param_1Table();

    void Reset();

    Str_Ar_BallCoef_Param_1Table& operator=(const Str_Ar_BallCoef_Param_1Table &Q);

    explicit Str_Ar_BallCoef_Param_1Table (const Str_Ar_BallCoef_Param_1Table &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_BallCoef_Param_1Table::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   Str_Arr_Gamma_Param_AllTables
//DESCRIPTION   :   Structure for arrays of table characteristics to determine ballistic coefficient for all BT-subclasses
struct Str_Ar_BallCoef_Param_AllTables
{
    CGLArrayFix <Str_Ar_BallCoef_Param_1Table, AIR_BALL::N_BALL_SUBCL> arData; //array of tables of characteristics to determine ballistic coefficient for all BT-subclasses

    qint16  N_el{0};    //number of filled elements in the array

    Str_Ar_BallCoef_Param_AllTables();

    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_BallCoef_Param_AllTables::getCell
    // DESCRIPTION	:   Returns pointer to the cell containing table of ballistic coefficients for given ballistic subclass
    // INPUTS		:	Ballistic subclass, enum ESubClassBT from GDEnum.h
    // RETURNS		:	Pointer to the cell containing table of ballistic coefficients
    Str_Ar_BallCoef_Param_1Table* getCell(const qint16 BallSubcl);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_BallCoef_Param_AllTables::getCellMark
    // DESCRIPTION	:   Returns pointer to the cell containing table of ballistic coefficients for given mark of ballistic missile
    // INPUTS		:	Ballistic subclass, enum BM_Marks from air_ballist_literal.h
    // RETURNS		:	Pointer to the cell containing table of ballistic coefficients
    Str_Ar_BallCoef_Param_1Table* getCellMark(const qint16 BallMark);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_BallCoef_Param_AllTables::AddTable()
    // DESCRIPTION	:   Add a new table
    // INPUTS		:	Structure containing new table
    // RETURNS		:	None
    void AddTable(const Str_Ar_BallCoef_Param_1Table &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Str_Ar_BallCoef_Param_AllTables::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file
    // RETURNS		:	None
    void LogData(FILE* _pLogFile);
};


namespace AIR_BALL
{
    //PACKAGE       :   AirBallist
    //STRUCTURE     :   IndCell
    //DESCRIPTION   :   Auxiliary structure for forming array of indexed cells
    struct IndCell
    {
        qint32      Index{0};           //index
        bool        SignBusy{false};    //true if cell is busy
    };


    //PACKAGE		:   AirBallist
    //STRUCTURE 	:   sCovObj
    //DESCRIPTION	:   Information about covered object
    struct sCovObj
    {
        qint32      Num{0};         //number of covered object
        CGeodesic   CoordGeodez;    //geodesic coordinates of covered object (radians, radians, meters)
        qreal       Radius{0.};     //radius, m

        sCovObj();

        void Reset();

        sCovObj& operator=(const sCovObj &Q);

        explicit sCovObj (const sCovObj &Q);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   sCovObj::LogData()
        // DESCRIPTION	:   Output data to the log file
        // INPUTS		:	Specified file
        // RETURNS		:	None
        void LogData(FILE* _pLogFile);
    };


    //PACKAGE		:   AirBallist
    //STRUCTURE 	:   s_arCovObj
    //DESCRIPTION	:   Structure containing array of covered objects
    struct s_arCovObj
    {
        qint32      N_obj{0};       //actual number of covered objects
        CGLArrayFix <sCovObj, AIR_BALL::COVERED_OBJ_AMOUNT> arCovObj; //array of covered objects

        s_arCovObj();

        void Reset();

//        s_arCovObj operator=(const s_arCovObj &Q);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   s_arCovObj::AddCovObj()
        // DESCRIPTION	:   Add new covered object
        // INPUTS		:	Reference to the new covered object
        // RETURNS		:	None
        void AddCovObj(const sCovObj &CovObj);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   s_arCovObj::LogData()
        // DESCRIPTION	:   Output data to the log file
        // INPUTS		:	Specified file
        // RETURNS		:	None
        void LogData(FILE* _pLogFile);
    };


    //PACKAGE		:   AirBallist
    //STRUCTURE 	:   Str_InpMsg
    //DESCRIPTION	:   Structure for input messages to AirBallist tack
    struct Str_InpMsg
    {
        enum InpMsgTypes //types of input messages
        {
            UNDEFINED_MSG = 0,
            NEW_ACTION_PHASE = 1, //new combat action phase
            SWAP_NUMBERS = 2, //swap SC4I numbers
            BALL_COEFF = 3 //value of ballistic coefficient, obtained from source
        };

        qint16      MsgType{0};     //type of message, enum InpMsgTypes
        qint32      NumbGT_Tgt{0};  //number of generalized track of target
        qint32      NumbGT_IC{0};   //number of generalized track of interceptor
        qint16      ActionPhase{0}; //combat action phase, enum EActionPhaseAO
        qreal       Time_APh{AIR_BALL::Timer_INI};   //time of obtainment for current Action Phase
        qreal       Gamma{0.};      //ballistic coefficient obtained from source, m^2/kg

        Str_InpMsg();

        void Reset();

        Str_InpMsg& operator=(const Str_InpMsg &Q);

        explicit Str_InpMsg (const Str_InpMsg &Q);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   Str_InpMsg::LogData()
        // DESCRIPTION	:   Output data to the log file
        // INPUTS		:	Specified file
        // RETURNS		:	None
        void LogData(FILE* _pLogFile);
    };


    //PACKAGE		:   AirBallist
    //STRUCTURE 	:   Str_InpMsgsBuffer
    //DESCRIPTION	:   Structure for buffer of input messages to AirBallist tack
    struct Str_InpMsgsBuffer : QRecursiveMutex //OSCriticalSection
    {
        qint16   MsgsAmount{0};     //amount of input messages
        CGLArrayFix <Str_InpMsg, AIR_BALL::INP_MSGS_AMOUNT> MsgsAr; //array of messages

        Str_InpMsgsBuffer();

        void Reset();

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   Str_InpMsgsBuffer::AddMsg
        // DESCRIPTION	:   Add message to the buffer of input messages to Air Ballist tack
        // INPUTS		:	Type of message (enum Str_InpMsg::InpMsgTypes), SC4I number of GT of target,
        //              :   SC4I number of GT of interceptor, action phase (enum EActionPhaseAO)
        //              :   time of obtainment for current Action Phase
        // RETURNS		:	None
        void AddMsg(const qint16 MsgType, const qint32 NumGT_Tgt, const qint32 NumGT_IC, const qint16 ActionPhase, const qreal Time_APh);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   Str_InpMsgsBuffer::AddMsg
        // DESCRIPTION	:   Add message to the buffer of input messages to Air Ballist tack
        // INPUTS		:	Type of message (enum Str_InpMsg::InpMsgTypes), SC4I number of 1st object,
        //              :   SC4I number of 2nd object
        // RETURNS		:	None
        void AddMsg(const qint16 MsgType, const qint32 Num1, const qint32 Num2);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   Str_InpMsgsBuffer::AddMsg
        // DESCRIPTION	:   Add message to the buffer of input messages to Air Ballist tack
        // INPUTS		:	Filled form of message
        // RETURNS		:	None
        void AddMsg(const Str_InpMsg &Msg);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   Str_InpMsgsBuffer::AddMsgBallCoef
        // DESCRIPTION	:   Add message containing ballistic coefficient obtained from the source
        //              :   to the buffer of input messages to Air Ballist tack
        // INPUTS		:	SC4I number of generalized track; ballistic coefficient (m^2/kg)
        // RETURNS		:	None
        void AddMsgBallCoef(const qint32 NumGT, const qreal Gamma);

        // PACKAGE		:   AirBallist
        // FUNCTION 	:   Str_InpMsgsBuffer::LogData()
        // DESCRIPTION	:   Output data to the log file
        // INPUTS		:	Specified file
        // RETURNS		:	None
        void LogData(FILE* _pLogFile);
    };
}


#endif // CTRLAIR_BALLIST_STRUCTS_H
