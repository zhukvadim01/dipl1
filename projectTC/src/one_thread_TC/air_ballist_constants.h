#ifndef AIR_BALLIST_CONSTANTS_H
#define AIR_BALLIST_CONSTANTS_H

#include "air_ballist_literal.h"
#include "ball_amount_const.h"

const qint32        DEPTH_VH = 3;                   //a parameter which determines the initial number of recent used results to estimate the average value of the vertical velocity and its RMSE
const qint32        DEPTH_VH_MAX = 40;              //a parameter which determines the maximum number of recent used results to estimate the average value of the vertical velocity and its RMSE
const qreal         MAX_ALLOWABLE_VH_RMSE = 20.;    //m/s, a parameter used to determine the number of recent used results to estimate the average value of the vertical velocity and its RMSE
const qreal         BOUND_VH_ACCEPT = 4.;           //a parameter used to set value boundary at which the hypothesis about the vertical velocity value shall be accepted

const qint32       SIZE_ARR_POLYNOM = 10;           //size of array contains parameters of polynom

const qreal     H_CHANGE_POLYNOM = 100000;          //m, the height at whitch the polynom changes at descending branch

const qreal     cDT_WAITING_FOR_RECLASSIFIED = 30;  //s, time interval of waiting before drop reclassified (to non-BT) track

const qreal     c360 = 360;
const qreal     c1000 = 1000;
const qreal     H_ATMOSPHERE = 100000;          //m, height of atmosphere
const qreal     cAngleDirMan = 5.;              //degrees, boundary value of angle between azimuths of velocity vectors used for maneuver detection
const qreal     cCoefDirMan = 1.5;              //coefficient used for maneuver detection

const qint32       DIM_CVA = 9 ;                //dimension of vectors/matrices composed of coordinates, velocity and acceleration components; do not change
const qint32       DIM_CV = 6;                  //dimension of vectors/matrices composed of coordinates and velocity components; do not change
const qint32       DIM_C = 3;                   //dimension of vectors/matrices composed of coordinates; do not change

const qreal     cMinRMSE = 0.1;             //minimum admissible value of RMSE
const qreal     cCovCorr = 0.001;           //correction value for covariance moments

const qreal     LAMBDA_UNSC_COV = 1.;       //parameter Lambda used in prolongation of covariance matrix by means of unscented transform

//constants for path and branch determination
const qint32    DEPTH_AH = 5;                       //a parameter which determines the initial number of recent used results to estimate the average value of the vertical acceleration and its RMSE
const qint32    DEPTH_AH_MAX = 60;                  //a parameter which determines the maximum number of recent used results to estimate the average value of the vertical acceleration and its RMSE
const qreal     MAX_ALLOWABLE_AH_RMSE = 3.;         //a parameter used to determine the number of recent used results to estimate the average value of the vertical acceleration and its RMSE
const qreal     BOUND_AH_EAP_ACCEPT = 1;            //a parameter used to set value boundary at which the hypothesis about the end of active phase using vertical acceleration values shall be accepted
const qreal     BOUND_Ah_ACT_PATH_ACCEPT = 7;       //a parameter used to set value boundary at which the hypothesis about the motion phase being an active phase based on vertical acceleration values shall be accepted
const qreal     Ah_FOR_PASSIVE_PATH_ACCEPT = -4;    //a value which is used for passive path determination (if Ah less than this value than this is a passive path)
const qreal     CORR_RMSE_AH = 0.04;                //correction for RMSE of vertical acceleration
const qreal     cCoef_a_g_sigA = 2.;                //coefficient for a, g and RMSE of a comparison

const qreal     cK_VT = 10;             //maximum admissible SigmaVhSum/dt ratio for path determination using vertical velocity
const qreal     cEAP_m0 = 4;            //first sequential number for EAP determination
const qreal     cEAP_Ksi = 0.6436341;   //coefficient for exponential smoothing
const qreal     cEAP_P0 = -15;          //threshold value for m0 sequential number
const qreal     cEAP_P1 = -4;           //threshold value for m0+8, m0+9, ... sequential numbers

const qreal     SIGMA_EPSILON = 0.01;                   //radar characteristics, elevation RMSE
const qreal     SIGMA_RANGE = 0.01;                     //radar characteristics, slant range RMSE
const qint32    MIN_OVERWEIGHT_POINT_NUMBER = 4;        // a parameter to set minimal number of iterations at which the probability of the second motion model is bigger than the first one, and which is used to determine the end of active phase
const qreal     MAX_ALLOWABLE_H_RMSE_FOR_FILTER = 10;   // a parameter setting the maximum RMSE value of measurements when the filter method is used
const qreal     DT_PATH_REDETERMINATION_CONTROL = 40;   //s, time after the EAP for perform the control of path redetermination
const qreal     DT_CTRL_PATH_REDETERM = 10;             //s, control time for confirmation of redetermination
const qint16    N_CTRL_PATH_REDETERM = 5;               //counter value for confirmatin of redetermination

const qreal     PERIOD_BRANCH_CHECK_CONFIRM = 5;        //s, period of branch confirmation diring check
const qint32    NUMB_BRANCH_CHECK_CONFIRM = 3;          //number of branch confirmations during check
const qreal     cDT_EAP_margin = 40;                    //s, margin for EAP time comparison
const qreal     EAP_FUSION_COEFFICIENT = 2;             //coefficient used in EAP fusion

//constants for prolongation
const qreal     c_PrognPeriod_WH_Usual = 10;                //prognostication period for WH, usual mode, s
const qreal     c_PrognPeriod_WH_Usual_HI_LOAD = 20;        //prognostication period for WH, usual mode, high load, s
const qreal     c_PrognPeriod_WH_Urgent = 5;                //prognostication period for WH, urgent mode, s
const qreal     c_PrognPeriod_WH_Urgent_HI_LOAD = 10;       //prognostication  period for WH, urgent mode, high load, s
const qreal     c_PrognPeriod_Boost = 10;//30;              //prognostication period for boosters, s
const qreal     c_PrognPeriod_Boost_HI_LOAD = 40;           //prognostication period for boosters, high load, s
const qreal     c_PrognPeriod_Aeroball = 0.49;                 //s, prognostication period for aeroballistic missiles (QBM and MaRV)
const qreal     c_PrognPeriod_Aeroball_HI_LOAD = 2;         //s, prognostication period for aeroballistic missiles (QBM and MaRV), high load
const qreal     c_PrognPeriod_NearMeet = 0.9;               //s, prognostication period near the meet point
const qreal     c_PrognPeriod_Satellite = 30;               //s, prognostication period for Satellite
const qreal     c_ControlTimeNearMeet = 20;                 //s, control time with prognostication near the meet point
const qreal     c_Tcontr = 5000;                            //control time, s
const qreal     c_Gamma_st = 0.0001;                        //average ballistic coefficient, m^2/kg
const qreal     c_Gamma_max = 0.013;                        //maximum value of ballistic coefficient, m^2/kg
const qreal     c_Gamma_max_WH = 0.0005;                    //maximum value of ballistic coefficient for WH
const qint32    c_NumSmoothUseGamma = 25;                   //number of smoothing step from which it is possible to use filtered Gamma
const qreal     c_h0 = 0;                                   //accuracy of reaching the surface of the earth
const qreal     c_Delta_Eps = 10;                           //accuracy for solving the system of differential equations
const qreal     c_step = 1;                                 //integration step, s
const qreal     c_min_step = 0.01;                          //minimum step of prolongation //changed for comparison of Step, s
const qreal     c_min_step_CloseToEarth = 0.0000001;        //minimum step of prolongation close to the Earth surface, s
const qreal     c_K_Gamma = 5;                              //coefficient for Gamma in 1st method of start point computation
const qreal     c_H_Eps = 0.1;                              //precision of H calculation
const qreal     c_H_tolerance = 100;                        //tolerance of H for finding of start point
const qreal     c_VH_tolerance = 0;                         //tolerance of VH for finding of start point
const qreal     c_T_act_def = 60;                           //default time of end of active leg, s
const qreal     c_dT_def_start_2method = 200;               //interval of try for 2st method of start point computation when 1st method is executed, s
const qreal     c_dTheta1 = 0.1;                            //1st boundary for comparison of Theta, Degrees
const qreal     c_dTheta2 = 8;                              //2st boundary for comparison of Theta, Degrees
const qreal     c_dModVel = 15;                             //correction for velocity module comparison, m/s
const qreal     c_coefDeltaL = 0.5;                         //coefficient at the difference of the distance between two cells in the table of the ends of active path
const qreal     c_dT_corr_progn = 60;                       //time interval for correction of previous prolonged track, s
const qreal     c_dT_storage = 1;                           //s, time interval for stored prolongated data
const qreal     c_Epsilon_dTst = 0.05;                      //s, admissible deviation for c_dT_storage
const qreal     c_MultEllSemiaxes = 3.;                     //coefficient at the ellipse semiaxes
const qreal     c_MinSemiaxesRatio = 0.1;                   //minimum ratio "small semiaxis divided by large semiaxis"
const qreal     c_CoefDT_Apogee = 0.2;                      //coefficient at the delta_t_apogee for calculation of polynom of 2 degree
const qreal     c_TimeProlSatellite = 300;                  //s, prolongation time for satellite
const qreal     c_dTStart_CheckSP = 120;                    //s, delta start time for check start point
const qreal     c_MaxStrobeSize = 10000;                    //m, maximum size of the strobe
const qreal     c_OutPointsDelay = 5;                      //s, delay for output of 50 points
const qreal     c_ProlDelay = 5;                            //s, delay for prolongation
const qreal     c_MinDistOutPnts = 80000;                   //m, minimum flight distance for output of 50 points
const qreal     c_AngleVcurrVprev = 0.1745320;              //radians, angle between current and previous velocity vectors

const qreal     cHeight4Start		= 3000;                 //m, necessary height for detection of start point by 1st point of trace
const qreal     cHeight4StartCheck	= 10000;                //m, necessary height for checking of start point by 1st point of trace
const qreal     cCoefPeriodEll_Usual  = 2.;                 //coefficient of ellipse calculation period, usual mode
const qreal     cCoefPeriodEll_Urgent = 1.;                 //coefficient of ellipse calculation period, urgent mode

const qreal     cHeightLowApogee = 50000;                   //m, heiht of low apogee
const qreal     cHeightStopProlong_LowApogee = 5000;        //m, height of prolongation stop for trajectories with low apogee
const qreal     cHeightStopProlong_HiApogee = 5000;         //m, height of prolongation stop for trajectories with hight apogee
const qreal     cCoefPredictionDeviation = 3.;              //coefficient for check prediction deviation

const qreal     cHeightKGamma1 = 20000;                     //1st boundary height for KGamma calculation
const qreal     cHeightKGamma2 = 100000;                    //2nd boundary height for KGamma calculation
const qreal     cKGammaMin = 1.;                            //minimum value of KGamma

const qreal     cHeightCtrlStartCalc = 30000;               //m, height for begin of control for start point calculation attempts
const qreal     cCoefHApogeeCtrlStartCalc = 0.3;            //coefficient at HApogee for height comparison during control for start point calculation attempts
const qreal     cCounterCtrlStartCalc = 5;                  //boundary value of counter for start point calculation attempts
const qint16    cCounterBadInvProl = 3;                     //boundary value of counter for bad inverse prolongations

const qreal     cDTMaxPrognAeroball = 120.;                 //s, maximum prolongation time for aeroballistic maneuver
const qreal     cLiftCoeff_default = 0.001;                 //default value of the lift coefficient for aeroballistic missiles, m^2/kg

//The constants for definition of BT trajectory type
const qreal     KsiAh = 0.5;            //parameter of exponential smoothing of vertical accelerations
const qreal     kAh = 0.02;             //limitative constant
const qreal     epsAh = 0.01;           //limitative constant
const qreal     sAh = 0.7;              //parameter for exponentiation
const qreal     lam1Ah = 10;            //1st boundary value of Lambda
const qreal     lam2Ah = 30;            //2st boundary value of Lambda
const qint16	NminTType = 2;          //quantity of the equals values of TType, necessary to make decision about TType
const qreal     dT_TType = 1;           //time interval for phase of determination of TType
const qreal     cTTypeRecount = 60;     //s, time for periodic control of Trajectory Type

//The constants for definition of BT Mark
const qreal     Cells_H_He_Bounds[4]        = {200000, 800000, 1600000, 7600000};   //boundaries of the diapasons on the plane "H-He"
const qreal     Cells_sizes[4]              = {10000, 20000, 40000, 80000};         //sizes of the cell depending on the diapason
const qint32    Cells_N0_diapason[4]        = {0, 300, 1500, 2700};                 //shift of the numeration depending on the diapason

const qreal     PERIOD_MARK_CHECK = 150;            //s, period of mark check
const qreal     DT_REFUSE_MARK_RECOUNT = 200;       //s, refuse mark recount if time before the fall is less than this value
const qreal     PERIOD_MARK_CONFIRM = 5;            //s, period of mark confirmation
const qint32    NUMB_MARK_CONFIRM = 3;              //number of mark confirmations

const qint32    N_LIMITS_H_MARK = 6;                                                            //number of elements in the array H_ATM_MARK
const qreal     H_LIM_MARK[N_LIMITS_H_MARK][2] = {
                                                    {AIR_BALL::UNDEFINED_BM_MARK, 80000},
                                                    {AIR_BALL::BM60, 20000},                    //array of limiting values of height
                                                    {AIR_BALL::BM100, 40000},                   //to determine mark on descending branch
                                                    {AIR_BALL::BM1000, 30000},
                                                    {AIR_BALL::BM3000, 40000},
                                                    {AIR_BALL::UNKNOWN_OF_LONG_RANGE, 80000}};

//Values of "Delta" for method of finite differences
//DeltaCoord optimal, DeltaVel optimal, DeltaCoord lofted, DeltaVel lofted, DeltaCoord flat, DeltaVel flat
const qreal     Delta_FinDiff[AIR_BALL::NUMB_BM_MARKS][6] = {{1000, 10, 1000, 10, 1000, 10}, //BM60
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM100
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM200
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM300
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM600
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM750
                                                             {1000, 13, 1000, 10, 1000, 16}, //BM1000
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM1500
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM1800
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM2000
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM2500
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM2800
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM3000
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM4000
                                                             {1000, 30, 1000, 10, 1000,  7}, //BM5000
                                                             {1000, 10, 1000, 10, 1000, 10}, //BM6000
                                                             {1000, 10, 1000, 10, 1000,  5}, //BM7000
                                                             {1000, 10, 1000, 10, 1000,  5}, //BM8000
                                                             {1000, 10, 1000, 10, 1000,  5}, //BM9000
                                                             {1000, 10, 1000, 10, 1000,  5}, //BM10000
                                                             {1000, 10, 1000, 10, 1000,  5}, //BM11000
                                                             {1000, 10, 1000, 10, 1000,  5}, //BM12000
                                                            };

const qreal DeltaCoord_defalut  = 1000;     //m, default value of DeltaCoord for method of finite differences
const qreal DeltaVel_default    = 10;       //m/s, default value of DeltaVel for method of finite differences

const qreal    K_NomDistance    = 1.1;      //coefficient at distance in the situation of comparison with nominal distance
const qreal    K_NomMinDistance = 0.2;      //coefficient at minimum distance for check

//Array of constants for calculation density function
const qreal	DensCoef[8][4] = {	{0,		 1.225,		-0.2639e-8, 0.7825e-4},
                                {20000,  0.891e-1,	0.4407e-9,	0.16375e-3},
                                {60000,  2.578e-4,	-0.2560e-8, 0.5905e-4},
                                {100000, 4.061e-7,	0.1469e-8,	0.1787e-3},
                                {150000, 2.130e-9,	0.8004e-10, 0.3734e-4},
                                {300000, 4.764e-11, 0.7111e-11, 0.1547e-4},
                                {600000, 8.726e-12, 0.1831e-11, 0.928e-5},
                                {900000, 6.367e-13, 0,			0.954e-5}};

//The constants for Estimation of Prognosis on Active Leg
//Array of bounding values of height and velocity
const qreal	a_LHV[22][3] =	{	{60,    20000,  1000},
                                {100,   25000,  1200},
                                {200,   30000,  1400},
                                {300,	40000,	1800},
                                {600,	60000,	2400},
                                {750,   65000,  2600},
                                {1000,	130000,	2900},
                                {1500,  140000, 3600},
                                {1800,  160000, 4000},
                                {2000,	180000,	4100},
                                {2500,  200000, 4300},
                                {2800,  140000, 4700},
                                {3000,  170000, 5000},
                                {4000,  200000, 5500},
                                {5000,  400000, 5800},
                                {6000,  460000, 6100},
                                {8000,  430000, 6500},
                                {7000,  480000, 6500},
                                {9000,  520000, 7200},
                                {10000, 800000, 7300},
                                {11000, 810000, 7400},
                                {12000, 820000, 7500}
                            };
const qreal cHboundBraking[2] = {30000., 40000.};       //boundary height to calculate coefficient of braking
const qreal cCoeffBraking[3]  = {0.5, 0.7, 0.8};        //values of coefficient of braking

const qreal	cConf_Angle		 = 7;           //deg, confidence value for RMSE of angles
const qreal cCoefDMinMax     = 1.1;         //coefficient for calculation max distance in the case when max distance < min distance
const qreal cCoefDMax        = 1.2;         //coefficient for estimation maximum distance using comparison with distance to the covered objects

const qreal cTPeriod_ActPr              = 2.;           //s, period of active prediction
const qreal cMinRange_ActPr             = 1000;         //m, minimum of range
const qreal cMaxRange_ActPr_default     = 2000000;      //m, default value for maximum of range
const qreal cMaxRange_ActPr_extended    = 12000000;     //m, extended value for maximum of range
const qreal cDeltaAzCorr                = 2.9;          //degrees, correction for consideration of dynamic error by azimuth, degrees
const qreal cKSigmaAz                   = 3.;           //confidence coefficient

//maximum flight time depending on the BM-mark
const qreal cTFlightMaxBM60         = 220;
const qreal cTFlightMaxBM100        = 270;
const qreal cTFlightMaxBM200        = 340;
const qreal cTFlightMaxBM300        = 420;
const qreal cTFlightMaxBM600        = 550;
const qreal cTFlightMaxBM750        = 600;
const qreal cTFlightMaxBM1000       = 760;
const qreal cTFlightMaxBM1500       = 910;
const qreal cTFlightMaxBM1800       = 1030;
const qreal cTFlightMaxBM2000       = 1140;
const qreal cTFlightMaxBM2500       = 1210;
const qreal cTFlightMaxBM2800       = 1340;
const qreal cTFlightMaxBM3000       = 1400;
const qreal cTFlightMaxBM4000       = 1620;
const qreal cTFlightMaxBM5000       = 1950;
const qreal cTFlightMaxBM6000       = 2230;
const qreal cTFlightMaxBM7000       = 2450;
const qreal cTFlightMaxBM8000       = 2640;
const qreal cTFlightMaxBM9000       = 2960;
const qreal cTFlightMaxBM10000      = 3570;
const qreal cTFlightMaxBM11000      = 3720;
const qreal cTFlightMaxBM12000      = 3800;

namespace AIR_BALL
{
    const qreal     Timer_INI           = 0.;           //initial value for timers
    const qreal     K_Sigma             = 3.;           //coefficient at RMSE
    const qint32    MaxCycleIter        = 10000;     //maximum iterations quantity for the "while" cycle
    const qreal     cLambdaCalcPeriod   = 0.5;          //coefficient for mean period estimation
    const qreal     cCoefCmpPeriod      = 2.;           //coefficient for periods comparison

    const qreal c2 = 2.;
    const qreal c3 = 3.;
    const qreal c4 = 4.;
    const qreal c5 = 5.;
    const qreal c6 = 6.;
    const qreal c7 = 7.;
    const qreal c8 = 8.;
    const qreal c100 = 100.;

        //MaRV
    const qreal    MAX_D_PERCENTAGE_MAN_MARV = 10;      //percents, Maximum of the percentage share of the maneuver area along the Earth surface to the total flight distance for  MaRV
        //QBM
    const qreal    MAX_D_PERCENTAGE_MAN_QBM = 40;       //percents, Maximum of the percentage share of the maneuver area along the Earth surface to the total flight distance for QBM
    const qreal    MAX_H_APOGEE_QBM = 200000;           //m, Maximum possible apogee altitude value  for QBM
    const qreal    MAX_DIST_QBM = 2000;                 //km, Maximum possible flight range from a launch point  up to impact point for QBM
    const qreal    MIN_H_AB_MANEUVER = 5000;            //m, minimum height of an aeroballistic maneuver

    const qint16   cN_Well_Sm = 5;                      //step of smoothing needed to considerer update as "well smoothed"
    const qreal    cRMSE_Coord_Well_Sm = 100;           //m, RMSE of coordinates needed to considerer update as "well smoothed"
    const qreal    cRMSE_Vel_Well_Sm = 15;              //m/s, RMSE of velocities needed to considerer update as "well smoothed"
    const qreal    cRMSE_Acc_Well_Sm = 5;               //m/s^2, RMSE of accelerations needed to considerer update as "well smoothed"
    const qreal    cDTContrCheckWellSm = 20;            //s, control time for check well smoothed update

    //The constants for prognostication
    const qreal RZ			=   6371000;                //Earth radius
    const qreal cd_b0		=	3.98605e14;             //m^3 / s^2
    const qreal cd_b2		=	1.756e25;               //m^5 / s
    const qreal cd_a		=	6378137.0;              //m
    const qreal cd_Alpha	=	1./298.257223563;       //b / r
    const qreal OmegaZ		=	7.2921158e-5;           //s^-1
}

#endif // AIR_BALLIST_CONSTANTS_H
