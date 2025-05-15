#ifndef CBALLISTICPROLONG_STRUCTS_H
#define CBALLISTICPROLONG_STRUCTS_H

#include "ball_prognoz_structs.h"
#include "gl/GLGeometry.h"


//PACKAGE       :   AirBallist
//STRUCTURE     :   CBallProl_Input
//DESCRIPTION	:   Input data for ballistic prolongation tacks
struct CBallProl_Input
{
    qint32          IF_GT{-1};                  //index of GT
    qint32          IF_BALL{-1};                //index of track in the array of ballistic tracks
    qreal           tLoc{0.};                   //time of location, s
    GLPointDouble3D Coord;                      //coordinates in ECEF, m
    GLPointDouble3D Vel;                        //velocity components in ECEF, m/s
    GLPointDouble3D Acc;                        //acceleration components in ECEF, m/s^2
    qreal           H{0.};                      //height in the current point, m
    GLMatrix        CovMatr;                    //Covariance matrix of input data
    bool            SignQuickReaction{false};   //sign of quick reaction mode, 0 - normal reaction mode, 1 - quick reaction mode
    bool            Sign_large_load{false};     //true if the load is large (large quantity of ballistic objects)
    qint16          BallType{0};                //type of ballistic object (enum EType in GDEnum.h)
    qreal           AB_MaxDistance{0.};         //maximum distance for possible aeroballistic missiles (MaRV or QBM)
    qint16          ActionPhase{0};             //action phase, enum EActionPhaseAO in GDEnum.h
    qreal           tObtainActionPhase{0.};     //time of obtainement of the action phase
    bool            bAeroballistic{false};      //sign of aeroballistic object

    CBallProl_Input();

    void Reset();
};


//PACKAGE       :   AirBallist
//STRUCTURE     :   CBallProl_Output
//DESCRIPTION	:   Output data for ballistic prolongation tacks
struct CBallProl_Output
{
    bool            p_StartIsChanged{false};    //true if information about start point is changed

    bool            p_FallIsChanged{false};     //true if information about prolongation is changed
    bool            p_FallEllIsChanged{false};  //true if information about ellipse in fall point is changed
    qint16          SignProgn{0};               //sign of prognosis, see enum Prognosis_Signes

    CGLArrayFix <TracksPoints, AIR_BALL::N_OUT_POINTS> arrPoints; //array of read-out points (50 points of trajectory)
    bool            bArrPointsFilled{false};    //true if arrPoints is filled
    Polynoms_data   Polynoms;                   //data about polynoms of 4th and 8th deg
    GLTrapeze       AB_Trapeze;                 //trapeze parameters for possible aeroballistic missile

    bool            bBoostSepPointsChanged{false}; //true if information about points of boosters separation was changed
    GLPoint_tXYZVTh PointSepBoost1;             //point of the 1st booster separation
    GLPoint_tXYZVTh PointSepBoost2;             //point of the 2nd booster separation

    CBallProl_Output();

    void Reset();

    CBallProl_Output& operator=(const CBallProl_Output &Q);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallProl_Output::GetNearIndex()
    // DESCRIPTION	:   Return index of element in the arrPoints array nearest to the given time moment
    // INPUTS		:	Time, s
    // RETURNS		:	Index of element in the arrPoints array nearest to the time tCurr
    qint32 GetNearIndex(qreal tCurr);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallProl_Output::LogProlongedTrack()
    // DESCRIPTION	:   Output prolonged track to the log file
    // INPUTS		:	Specified file; location time; number of GT
    // RETURNS		:	None
    void LogTrackPoints(FILE *_pLogFile, qreal tLoc, qint32 NumGT);
};


// PACKAGE		:  AirBallist
// STRUCT		:  CBallistProlong_InnerData
// DESCRIPTION	:  Inner data for ballistic prolongation tack
struct CBallistProlong_InnerData
{
    GLPointDouble3D StartPoint;                             //start point, in ECEF, m
    GLPointDouble3D FallPoint;                              //fall point, in ECEF, m
    qint16          StartPointFindMethod{0};                //method of start point finding; see enum StartPointFinding
    bool            bStartByTablesPerformed{false};         //true if start point determination by EAP Tables was performed
    bool            bStartByTablesPerformed_multival{false}; //true if start point determination by EAP Tables was performed, in the case of multivalue mark
    qreal           tStart{AIR_BALL::Timer_INI};            //time of start, s
    qreal           tFall{AIR_BALL::Timer_INI};             //predicted time of fall, s
    EllipseInfo     StartEll;                               //dispersion ellipse of start point (includes multiplicator 3 for semiaxes)
    EllipseInfo     FallEll;                                //dispersion ellipse of fall point (includes multiplicator 3 for semiaxes)
//    Trapeze_Parameters AB_Trapeze; //trapeze parameters for possible aeroballistic missile
    qreal           tSavedProl{AIR_BALL::Timer_INI};        //time of saved prolongation, s
    qreal           tSavedEll{AIR_BALL::Timer_INI};         //time of saved ellipse in the fall point, s
    qint32          N_Prol{0};                              //number of current prolongation
    qreal           tDetStartDownPr{AIR_BALL::Timer_INI};   //time of start point determination by down prognosis, s
    qreal           tPrevTryDetStartDownPr{AIR_BALL::Timer_INI}; //time of the previous try to determine start point using down prognosis, s
    qint16          CounterTryDetStartPointDownPr{0};       //counter for attempts of start point calculation using down prognosis
    qreal           tPrevTryDetStartByTables{AIR_BALL::Timer_INI}; //time of the previous try to determine start point using tables, s
    std::vector<M10>    ProlongedTrack;             //whole prolonged track
    Polynoms_data   PolData;                        //polynom data (4 deg glued with 8 deg)
    qreal           Hapogee{0.};                    //height of the apogee, m
    qreal           tApogee{AIR_BALL::Timer_INI};   //time of the apogee, s
    qreal           min_tApogee{AIR_BALL::Timer_INI}; //minimum value of the apogee time, s
    qint16          SignLastProgn{0};               //sign of last prognosis, enum Prognosis_Signes
    qreal           D{0.};                          //flying distance, m
    qreal           ThrowAngle{0.};                 //throw angle (at the end of active path), degrees
    qreal           Veap{0.};                       //absolute velocity at the end of active path, m/s
    qreal           Teap_byTable{AIR_BALL::Timer_INI}; //time of end of active path, determined by table
    bool            bBMMarkChanged{false};          //true if the mark of ballistic missile was changed
    qreal           DeltaVmin{0.};                  //minimum value of DeltaV, m/s
    qint16          MarkEAPDet{0};                  //mark using which EAP is determined, enum BM_Marks
    qint16          CounterBadInvProl{0};           //counter of bad inverse prolongations

    CBallistProlong_InnerData();

    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallistProlong_InnerData::InterpolationProlongAtTime()
    // DESCRIPTION	:   Interpolates prolonged track at the given time moment
    // INPUTS		:	Time moment; resulting coordinates
    // RETURNS		:	True if result is OK
    bool InterpolationProlongAtTime(qreal CurTime, GLPointDouble3D &CoordRes);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallistProlong_InnerData::GetNearIndex()
    // DESCRIPTION	:   Returns index of nearest element to the current time
    // INPUTS		:	Time moment
    // RETURNS		:	Index of the nearest element (-1 if absents)
    qint32 GetNearIndex(qreal CurTime);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CBallistProlong_InnerData::LogProlongedTrack()
    // DESCRIPTION	:   Output prolonged track to the log file
    // INPUTS		:	Specified file; location time; number of GT
    // RETURNS		:	None
    void LogProlongedTrack(FILE *_pLogFile, qreal tLoc, qint32 NumGT);
};


#endif // CBALLISTICPROLONG_STRUCTS_H
