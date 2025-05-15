//PACKAGE       AirBallist
//FILE          CtrlAir_Ballist.h
//AUTHOR        TaniaP
//DESCRIPTION   Header file for CtrlAIR_BALLIST class

#ifndef CTRLAIR_BALLISTM_H
#define CTRLAIR_BALLISTM_H

#include "CtrlAir_Ballist_structs.h"
#include "CAirBallist_Processing_M.h"
#include "ball_prognoz_structs.h"

#include "gl/GLSrcEnumeration.h"

// PACKAGE		:   AirBallist
// CLASS     	:   CtrlAIR_BALLIST
// DESCRIPTION	:   Control of ballistic data processing tacks
class CtrlAIR_BALLIST_GD
{
    friend class CtrlAIR_BALLIST_M;

public:
    CtrlAIR_BALLIST_GD() = delete;
    explicit CtrlAIR_BALLIST_GD(std::shared_ptr<CAirBallist_Processing_GD> pProcGD);

    ~CtrlAIR_BALLIST_GD();

private:

    QRecursiveMutex                 m_mutexGD;     //critical section

    AIR_BALL::Str_InpMsgsBuffer     m_InpMsgsBuf;   //buffer of input messages to Air Ballist tack

    std::shared_ptr<CAirBallist_Processing_GD>      m_pGlobalDataProc;

    AIR_BALL::s_arCovObj            m_arCovObj;         //array of covered objects
    Str_Ar_EAP_Param_AllTables      m_EAP_Tables;       //array of EAP tables
    Str_Ar_BallCoef_Param_AllTables m_BallCoef_Tables;  //array of ballistic coefficients tables
    Cells_BMMark_Data               m_CellsBMMark;      //array defining the correspondence between the cells on the plane "H-He" and the BT marks

    qint32      m_BO_quantity{0};                   //quantity of ballistic objects

    CGLVector <CtrlAIR_BALL_Inner_Data_GT> m_arDataCtrl;    //array of inner data for the CtlrAir_Ballist tack
    CGLVector <qint32*>          GT_IDs;                    //array of correspondence of number of generalized tracks and indexes of generalized tracks
                                                                //array indexes are numbers of GT,
                                                                //values are pointers to the indexes of GT

    GLInnerEnumeration          m_NumGTEnumeration;      //inner enumeration for GT
    CGLVector <qint32*>          Ball_IDs_forGT;         //array of correspondence of identificators of ballistic objects:
            //array indexes are indexes of GT,
            //values are pointers to the indexes of ballistic objects in air ballist numeration

    CGLVector <GLSrcEnumeration>     m_arSrcInnEnumeration;  //array of inner enumerators for sources
    CGLVector <CGLVector<qint32*>>    ST_IDs;                 //array of correspondence of identificators of ST:
            //rows - indexes of sources, columns - indexes of ST in source numeration,
            //values - pointers to the indexes of ST in air ballist numeration

    CGLVector <AIR_BALL::IndCell>    ID_Values_GT;         //array of values of GT indexes for air ballist numeration
    CGLVector <AIR_BALL::IndCell>    ID_Values_ST;         //array of values of ST indexes for air ballist numeration
    CGLVector <AIR_BALL::IndCell>    ID_Values_Ball;       //array of values of ballistic object indexes for air ballist numeration

    FILE*   m_pLogIDs{nullptr};         //pointer to the log file for indexes
    FILE*   m_pLogProgn{nullptr};       //pointer to the log file for prognosis parameters
    FILE*   m_pLogInp{nullptr};         //pointer to the log file for the input data
    FILE*   m_pLogTest{nullptr};        //pointer to the log file for output the test data
    FILE*   m_pLogRDID{nullptr};        //pointer to the log file for reference data and operative and tactical information
    FILE*   m_pLogOut{nullptr};         //pointer to the log file for output data


    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::SetPersistentData
    // DESCRIPTION	:   Sets persistent data
    // INPUTS		:   Reference to the structure containing array of tables of EAP;
    //              :   reference to the structure containing array of tables of ballistic coefficients
    // RETURNS		:	None
    void SetPersistentData(const Str_Ar_EAP_Param_AllTables &arrEAP_Tables,
                           const Str_Ar_BallCoef_Param_AllTables &arrBallCoef_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::SetPersistentData
    // DESCRIPTION	:   Sets persistent data
    // INPUTS		:   Reference to the structure containing array of tables of EAP
    // RETURNS		:	None
    void SetPersistentData(const Str_Ar_EAP_Param_AllTables &arrEAP_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FillCovObj
    // DESCRIPTION	:   Filling of array of covered objects
    // INPUTS		:	Reference to array of covered objects
    // RETURNS		:	None
    void FillCovObj(const AIR_BALL::s_arCovObj &arr_CovObj);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::Reset
    // DESCRIPTION	:   Reset of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

private:

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::InpMsgsProcessing
    // DESCRIPTION	:   Processing of input messages
    // INPUTS		:	None
    // RETURNS		:	None
    void InpMsgsProcessing(qreal currTime);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::NonBallGTProcessing
    // DESCRIPTION	:   Processing of non ballistic generalized track
    // INPUTS		:	Index of the generalized track; inner number of GT
    // RETURNS		:	None
    void NonBallGTProcessing(qint32 ind_GT, qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::SwapNumbers
    // DESCRIPTION	:   Swap SC4I numbers of GT
    // INPUTS		:	SC4I number of 1st GT, SC4I number of 2nd GT
    // RETURNS		:	None
    void SwapNumbers(qint32 Num1, qint32 Num2);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallSubcl_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic subclass from GD to AIR_BALL
    // INPUTS		:	Ballistic subclass in GD (enum ESubClassBT)
    // RETURNS		:	Ballistic mark in AIR_BALL (enum BM_Marks)
    static qint16 ConvertBallSubcl_GDtoAirBall(const qint16 BallSubclGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallSubcl_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic subclass from GD to AIR_BALL
    // INPUTS		:	Ballistic mark in AIR_BALL (enum BM_Marks)
    // RETURNS		:	Ballistic subclass in GD (enum ESubClassBT)
    static qint16 ConvertBallSubcl_AirBalltoGD(const qint16 BallMarkAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallPath_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic path from GD to AIR_BALL
    // INPUTS		:	Ballistic path in GD (enum EBallisticPathSign)
    // RETURNS		:	Ballistic path in AIR_BALL (enum BTPaths)
    static qint16 ConvertBallPath_GDtoAirBall(const qint16 BallPathGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallPath_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic path from GD to AIR_BALL
    // INPUTS		:	Ballistic path in AIR_BALL (enum BTPaths)
    // RETURNS		:	Ballistic path in GD (enum EBallisticPathSign)
    static qint16 ConvertBallPath_AirBalltoGD(const qint16 BallPathAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallBranch_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic branch from GD to AIR_BALL
    // INPUTS		:	Ballistic branch in GD (enum EBallisticBranchSign)
    // RETURNS		:	Ballistic branch in AIR_BALL (enum BTBranches)
    static qint16 ConvertBallBranch_GDtoAirBall(const qint16 BallBranchGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallBranch_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic branch from GD to AIR_BALL
    // INPUTS		:	Ballistic branch in AIR_BALL (enum BTBranches)
    // RETURNS		:	Ballistic branch in GD (enum EBallisticBranchSign)
    static qint16 ConvertBallBranch_AirBalltoGD(const qint16 BallBranchAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallTType_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic trajectory type from GD to AIR_BALL
    // INPUTS		:	Ballistic trajectory type in GD (enum EBallisticTrajType)
    // RETURNS		:	Ballistic trajectory type in AIR_BALL (enum BallTrajTypes)
    static qint16 ConvertBallTType_GDtoAirBall(const qint16 BallTTypeGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallTType_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic trajectory type from GD to AIR_BALL
    // INPUTS		:	Ballistic trajectory type in AIR_BALL (enum BallTrajTypes)
    // RETURNS		:	Ballistic trajectory type in GD (enum EBallisticTrajType)
    static qint16 ConvertBallTType_AirBalltoGD(const qint16 BallTTypeAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetMaxFlyingTime
    // DESCRIPTION	:   Returns maximum possible flight time from launch to fall
    // INPUTS		:	Ballistic mark in AIR_BALL (enum BM_Marks)
    // RETURNS		:	Maximum possible flight time from launch to fall, s
    static qreal GetMaxFlightTime(const qint16 BallMarkAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ReadCellsBMMark
    // DESCRIPTION	:   Reads array with information about BM Mark
    // INPUTS		:	None
    // RETURNS		:	None
    void ReadCellsBMMark();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetIdGT
    // DESCRIPTION	:   Returns ID of GT in the inner numeration of AirBallist
    // INPUTS		:	Number of GT in the numeration of AIR
    // RETURNS		:	ID of GT in the inner numeration of AirBallist
    qint32 GetIdGT(const qint32 NumGTExternal, const qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetInnerTrackNum()
    // DESCRIPTION	:   Returns the inner track number for the given external track number
    // INPUTS		:	Given external track number
    // RETURNS		:	Inner track number
    qint32 GetInnerTrackNum(qint32 NumExt);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearIdGT
    // DESCRIPTION	:   Clears ID of GT in the inner numeration of AirBallist
    // INPUTS		:	Number of GT in the numeration of AIR
    // RETURNS		:	None
    void ClearIdGT(qreal inputTime, const qint32 NumGTExt, const qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetIdST
    // DESCRIPTION	:   Returns ID of ST in the inner numeration of AirBallist
    // INPUTS		:	Index of source, number of trace in the numeration of source
    // RETURNS		:	ID of ST in the inner numeration of AirBallist
    qint32 GetIdST(qreal inputTime, const qint16 SrcInd, const qint32 NtrInSrc);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearIdST
    // DESCRIPTION	:   Clears ID of ST in the inner numeration of AirBallist
    // INPUTS		:	Index of source, number of trace in the numeration of source
    // RETURNS		:	None
    void ClearIdST(qreal inputTime, const qint16 SrcInd, const qint32 NtrInSrc);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetIdBall
    // DESCRIPTION	:   Returns ID of ballistic object in the inner numeration of AirBallist
    //              :   (-1 in the case of GT absence)
    // INPUTS		:	Index of GT, sign of ballistic trajectory
    // RETURNS		:	ID of ballistic object in the inner numeration of AirBallist
    qint32 GetIdBall(qreal inputTime, const qint32 Ind_GT, const bool bBallist);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearIdBall
    // DESCRIPTION	:   Clears ID of ballistic object in the inner numeration of AirBallist
    // INPUTS		:	Index of GT
    // RETURNS		:	None
    void ClearIdBall(qreal inputTime, const qint32 Ind_GT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearInnerData_GT()
    // DESCRIPTION	:   Cleaning inner data containing information about all generalized tracks
    // INPUTS		:	None
    // RETURNS		:	None
    void ClearInnerData_GT();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearInnerData_GT_1Track()
    // DESCRIPTION	:   Cleaning inner data containing information about given generalized track
    // INPUTS		:	index of track
    // RETURNS		:	None
    void ClearInnerData_GT_1Track(const qint32 ind_GT);
};


class CtrlAIR_BALLIST_M : public QObject
{
public:
    CtrlAIR_BALLIST_M() = delete;
    explicit CtrlAIR_BALLIST_M(std::shared_ptr<CtrlAIR_BALLIST_GD> pGD,
                               std::shared_ptr<CAirBallist_Processing_GD> pProcGD);
    ~CtrlAIR_BALLIST_M();

    static void initConsts(quint16, quint16, quint16);

    std::shared_ptr<CtrlAIR_BALLIST_GD> m_pGlobalData;

    QRecursiveMutex              m_mutexProc;            // mutex for processing task
    CAirBallist_Processing_M     m_CAirBallProc;         // Instance of the class CAirBallist_Processing

    qreal       m_CurTime{-1};                      //current time, seconds from day's beginning (for correct work of autonom)
    qint16      m_InDataMode{AIR_BALL::TEST_DATA};  //information about used structure of input data, see enum InDataModes

    CAirBall_Proc_InputData_GT      m_BallInpData;      //The input data to CAirBallist_Processing, for GT
    CAirBall_Proc_InputData_ST      m_BallInpData_ST;   //The input data to CAirBallist_Processing, for ST
    CtrlAIR_BALL_Output_GT_Data     m_CAirBallOut;

    CtrlAIR_BALL_traj_param     m_TrajParam;        //Received trajectory parameters
    CtrlAIR_BALL_Input_ST_Data  m_ST_Data;          //input data about single track
    CtrlAIR_BALL_Input_GT_Data  m_GT_Data;          //input data about generalized track

    FILE*   m_pLogIDs{nullptr};         //pointer to the log file for indexes
    FILE*   m_pLogProgn{nullptr};       //pointer to the log file for prognosis parameters
    FILE*   m_pLogInp{nullptr};         //pointer to the log file for the input data
    FILE*   m_pLogTest{nullptr};        //pointer to the log file for output the test data
    FILE*   m_pLogRDID{nullptr};        //pointer to the log file for reference data and operative and tactical information
    FILE*   m_pLogOut{nullptr};         //pointer to the log file for output data

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::SetPersistentData
    // DESCRIPTION	:   Sets persistent data
    // INPUTS		:   Reference to the structure containing array of tables of EAP;
    //              :   reference to the structure containing array of tables of ballistic coefficients
    // RETURNS		:	None
    void SetPersistentData(const Str_Ar_EAP_Param_AllTables &arrEAP_Tables,
                           const Str_Ar_BallCoef_Param_AllTables &arrBallCoef_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::SetPersistentData
    // DESCRIPTION	:   Sets persistent data
    // INPUTS		:   Reference to the structure containing array of tables of EAP
    // RETURNS		:	None
    void SetPersistentData(const Str_Ar_EAP_Param_AllTables &arrEAP_Tables);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FillCovObj
    // DESCRIPTION	:   Filling of array of covered objects
    // INPUTS		:	Reference to array of covered objects
    // RETURNS		:	None
    void FillCovObj(const AIR_BALL::s_arCovObj &arr_CovObj);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FillTrajParam
    // DESCRIPTION	:   Filling of trajectory parameters for generalized track
    // INPUTS		:	Structure containing the trajectory parameters
    // RETURNS		:	None
    void FillTrajParam(const CtrlAIR_BALL_Input_GT_Data &inp_data);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FillTrajParam
    // DESCRIPTION	:   Filling of trajectory parameters for singe track
    // INPUTS		:	Structure containing the trajectory parameters
    // RETURNS		:	None
    void FillTrajParam(const CtrlAIR_BALL_Input_ST_Data &inp_data);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FillTrajParam
    // DESCRIPTION	:   Filling of trajectory parameters for test data
    // INPUTS		:	Structure containing the trajectory parameters
    // RETURNS		:	None
    void FillTrajParam(const CtrlAIR_BALL_traj_param &tr_param);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ProcessingTrajParam
    // DESCRIPTION	:   Processing of trajectory parameters
    // INPUTS		:	Mode of input data, enum AIR_BALL::InDataModes;
    //              :   pointer to the output data for GT
    // RETURNS		:	None
    void ProcessingTrajParam(qint16 InDataMode=-1, CtrlAIR_BALL_Output_GT_Data *pOutGT=nullptr);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ProcessingAirBallist
    // DESCRIPTION	:   Processing of trajectory parameters
    // INPUTS		:	Reference to the input ST data
    // RETURNS		:	None
    void ProcessingAirBallist(const CtrlAIR_BALL_Input_ST_Data &InData);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ProcessingAirBallist
    // DESCRIPTION	:   Processing of trajectory parameters
    // INPUTS		:	Reference to the input GT data; reference to the output GT data
    // RETURNS		:	None
    void ProcessingAirBallist(const CtrlAIR_BALL_Input_GT_Data &InData, CtrlAIR_BALL_Output_GT_Data &OutData);

    // PACKAGE		:   AirClassif
    // FUNCTION 	:   CtrlAIR_BALLIST::SetCurTime
    // DESCRIPTION	:   Sets current time, for correct work of autonom
    // INPUTS		:	Current time, seconds from 01.01.1970
    // RETURNS		:	None
    void SetCurTime(qreal _CurTime);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::Reset
    // DESCRIPTION	:   Reset of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

private:
    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FillBallProcInData
    // DESCRIPTION	:   Filling of input data to ballistic data processing
    // INPUTS		:	Inner number of GT
    // RETURNS		:	None
    void FillBallProcInData(qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::FormOutputData_GT
    // DESCRIPTION	:   Forming output data for GT
    // INPUTS		:	Index of generalized track
    // RETURNS		:	None
    void FormOutputData_GT(const qint32 ind_GT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::InpMsgsProcessing
    // DESCRIPTION	:   Processing of input messages
    // INPUTS		:	None
    // RETURNS		:	None
    void InpMsgsProcessing();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::NonBallGTProcessing
    // DESCRIPTION	:   Processing of non ballistic generalized track
    // INPUTS		:	Index of the generalized track; inner number of GT
    // RETURNS		:	None
    void NonBallGTProcessing(qint32 ind_GT, qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::SwapNumbers
    // DESCRIPTION	:   Swap SC4I numbers of GT
    // INPUTS		:	SC4I number of 1st GT, SC4I number of 2nd GT
    // RETURNS		:	None
    void SwapNumbers(qint32 Num1, qint32 Num2);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallSubcl_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic subclass from GD to AIR_BALL
    // INPUTS		:	Ballistic subclass in GD (enum ESubClassBT)
    // RETURNS		:	Ballistic mark in AIR_BALL (enum BM_Marks)
    qint16 ConvertBallSubcl_GDtoAirBall(const qint16 BallSubclGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallSubcl_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic subclass from GD to AIR_BALL
    // INPUTS		:	Ballistic mark in AIR_BALL (enum BM_Marks)
    // RETURNS		:	Ballistic subclass in GD (enum ESubClassBT)
    qint16 ConvertBallSubcl_AirBalltoGD(const qint16 BallMarkAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallPath_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic path from GD to AIR_BALL
    // INPUTS		:	Ballistic path in GD (enum EBallisticPathSign)
    // RETURNS		:	Ballistic path in AIR_BALL (enum BTPaths)
    qint16 ConvertBallPath_GDtoAirBall(const qint16 BallPathGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallPath_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic path from GD to AIR_BALL
    // INPUTS		:	Ballistic path in AIR_BALL (enum BTPaths)
    // RETURNS		:	Ballistic path in GD (enum EBallisticPathSign)
    qint16 ConvertBallPath_AirBalltoGD(const qint16 BallPathAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallBranch_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic branch from GD to AIR_BALL
    // INPUTS		:	Ballistic branch in GD (enum EBallisticBranchSign)
    // RETURNS		:	Ballistic branch in AIR_BALL (enum BTBranches)
    qint16 ConvertBallBranch_GDtoAirBall(const qint16 BallBranchGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallBranch_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic branch from GD to AIR_BALL
    // INPUTS		:	Ballistic branch in AIR_BALL (enum BTBranches)
    // RETURNS		:	Ballistic branch in GD (enum EBallisticBranchSign)
    qint16 ConvertBallBranch_AirBalltoGD(const qint16 BallBranchAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallTType_GDtoAirBall
    // DESCRIPTION	:   Convert ballistic trajectory type from GD to AIR_BALL
    // INPUTS		:	Ballistic trajectory type in GD (enum EBallisticTrajType)
    // RETURNS		:	Ballistic trajectory type in AIR_BALL (enum BallTrajTypes)
    qint16 ConvertBallTType_GDtoAirBall(const qint16 BallTTypeGD);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ConvertBallTType_AirBalltoGD
    // DESCRIPTION	:   Convert ballistic trajectory type from GD to AIR_BALL
    // INPUTS		:	Ballistic trajectory type in AIR_BALL (enum BallTrajTypes)
    // RETURNS		:	Ballistic trajectory type in GD (enum EBallisticTrajType)
    qint16 ConvertBallTType_AirBalltoGD(const qint16 BallTTypeAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetMaxFlyingTime
    // DESCRIPTION	:   Returns maximum possible flight time from launch to fall
    // INPUTS		:	Ballistic mark in AIR_BALL (enum BM_Marks)
    // RETURNS		:	Maximum possible flight time from launch to fall, s
    qreal GetMaxFlightTime(const qint16 BallMarkAirBall);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetInputTime
    // DESCRIPTION	:   Returns location time in the input data
    // INPUTS		:	None
    // RETURNS		:	Location time in the input data
    qreal GetInputTime();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ReadCellsBMMark
    // DESCRIPTION	:   Reads array with information about BM Mark
    // INPUTS		:	None
    // RETURNS		:	None
    void ReadCellsBMMark();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::IsBallistGT
    // DESCRIPTION	:   Returns true if GT is ballistic
    // INPUTS		:	Index of GT
    // RETURNS		:	True if GT is ballistic, false - otherwise
    bool IsBallistGT(const qint32 Ind_GT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetIdGT
    // DESCRIPTION	:   Returns ID of GT in the inner numeration of AirBallist
    // INPUTS		:	Number of GT in the numeration of AIR
    // RETURNS		:	ID of GT in the inner numeration of AirBallist
    qint32 GetIdGT(const qint32 NumGTExternal, const qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetInnerTrackNum()
    // DESCRIPTION	:   Returns the inner track number for the given external track number
    // INPUTS		:	Given external track number
    // RETURNS		:	Inner track number
    qint32 GetInnerTrackNum(qint32 NumExt);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearIdGT
    // DESCRIPTION	:   Clears ID of GT in the inner numeration of AirBallist
    // INPUTS		:	Number of GT in the numeration of AIR
    // RETURNS		:	None
    void ClearIdGT(const qint32 NumGTExt, const qint32 NumGTInner);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetIdST
    // DESCRIPTION	:   Returns ID of ST in the inner numeration of AirBallist
    // INPUTS		:	Index of source, number of trace in the numeration of source
    // RETURNS		:	ID of ST in the inner numeration of AirBallist
    qint32 GetIdST(const qint16 SrcInd, const qint32 NtrInSrc);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearIdST
    // DESCRIPTION	:   Clears ID of ST in the inner numeration of AirBallist
    // INPUTS		:	Index of source, number of trace in the numeration of source
    // RETURNS		:	None
    void ClearIdST(const qint16 SrcInd, const qint32 NtrInSrc);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::GetIdBall
    // DESCRIPTION	:   Returns ID of ballistic object in the inner numeration of AirBallist
    //              :   (-1 in the case of GT absence)
    // INPUTS		:	Index of GT, sign of ballistic trajectory
    // RETURNS		:	ID of ballistic object in the inner numeration of AirBallist
    qint32 GetIdBall(const qint32 Ind_GT, const bool bBallist);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearIdBall
    // DESCRIPTION	:   Clears ID of ballistic object in the inner numeration of AirBallist
    // INPUTS		:	Index of GT
    // RETURNS		:	None
    void ClearIdBall(const qint32 Ind_GT);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::Drop_GT
    // DESCRIPTION	:   Drop generalized track (GT)
    // INPUTS		:	Number of GT in the enumeration of AIR; internal number of GT; index of corresponding ballistic track
    // RETURNS		:	None
    void Drop_GT(const qint32 Numb_GT, qint32 NumGTInner, const qint32 ind_Ball);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::Drop_ST
    // DESCRIPTION	:   Drop single track (ST)
    // INPUTS		:	Index of ST
    // RETURNS		:	None
    void Drop_ST(const qint32 ind_ST);

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   CtrlAIR_BALLIST::ClearBallisticData
    // DESCRIPTION	:   Clears ballistic data
    // INPUTS		:	Index of GT, index of corresponding ballistic track
    // RETURNS		:	None
    void ClearBallisticData(const qint32 ind_GT, const qint32 ind_Ball);

};


namespace AIR_BALL
{
    // PACKAGE		: AirBallist
    // FUNCTION		: AIR_BALL::NumGen2_IsGreater
    // DESCRIPTION	: Defines which of two given GT indexes is the latest (not simply the greatest because of numbers recurrence)
    // INPUTS		: _IndGT1 - the first index
    //				: _IndGT2 - the second index
    // RETURN		: true if the second index is later than the first one
    bool IndGT2_IsLater(qint32 _IndGT1, qint32 _IndGT2);
}

#endif // CTRLAIR_BALLISTM_H
