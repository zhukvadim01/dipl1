//
//FILE			:rip_tc_prognoz_structs.h
//
//AUTHOR		:tatka,	13 May 2006
//

#ifndef _rip_tc_prognoz_structs_h
#define _rip_tc_prognoz_structs_h

#include "ball_amount_const.h"
//#include "ball_constants.h"
#include "CtrlAir_Ballist_structs.h"
#include "gl/GLArray.h"
#include "gl/GLGeometry.h"
#include<vector>


//struct Geodesic
//{
//    qreal B;//Latitude
//    qreal L;//Longtitude
//    qreal H;//Altitude
//};

// PACKAGE		:   AirBallist
// STRUCTURE   	:   Pred_Polinom
// DESCRIPTION	:   Parameters of polynom of 4 degree
struct Pred_Polinom //old polynom
{
    GLPointDouble3D Part0;	//coefficient to 0 degree
    GLPointDouble3D Part1;	//coefficient to 1 degree
    GLPointDouble3D Part2;	//coefficient to 2 degree
    GLPointDouble3D Part3;	//coefficient to 3 degree
    GLPointDouble3D Part4;	//coefficient to 4 degree
    qreal	AnchorTime{0.};	//Polinom(t) => t:=t-AnchorTime
    //when midnight is and AnchorTime > 86400 then	Polinom(t + 86400), t is a.m.
    //												Polinom(t), t is p.m.
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   One_Pr_Const
// DESCRIPTION	:   Constants and adjustable parameters for ball_one_prognoz task
struct One_Pr_Const
{
    qreal	Tcontr{0.};         //control time, s
    qreal	h0{0.};             //accuracy of reaching the surface of the earth
    qreal	Delta_Eps{0.};      //accuracy for solving the system of differential equations
    qreal	step{0.};           //integration step, s
    qreal	min_step{0.};       //minimum step of prolongation //changed for comparison of Step, s
    qreal   min_step_CloseToEarth{0.}; //minimum step of prolongation close to the Earth surface, s
    qreal	H_Eps{0.};          //precision of H calculation
    qreal	H_tolerance{0.};	//tolerance of H for finding of start point
    qreal	VH_tolerance{0.};	//tolerance of VH for finding of start point
    qreal	T_act_def{0.};      //default time of end of active leg, s
    qreal	dTheta1{0.};        //1st boundary for comparison of Theta, Degrees
    qreal	dTheta2{0.};        //2st boundary for comparison of Theta, Degrees

    One_Pr_Const();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   One_Pr_Const::Reset
    // DESCRIPTION	:   Reset of struct attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   One_Pr_Const::operator=
    // DESCRIPTION	:   Assignment operator
    // INPUTS		:	One_Pr_Const
    // RETURNS		:	One_Pr_Const
    One_Pr_Const& operator=(const One_Pr_Const &Q);

    explicit One_Pr_Const (const One_Pr_Const &Q);
};


// PACKAGE		:   AirBallist
// STRUCTURE   	:   Pr_Const
// DESCRIPTION	:   Constants and adjustable parameters for CAirBallisticProcessing task
struct Pr_Const
{
    qreal       PrognPeriod_WH_Usual{0};            //prognostication period for WH, usual mode, s
    qreal       PrognPeriod_WH_Usual_HI_LOAD{0};    //prognostication period for WH, usual mode, high load, s
    qreal       PrognPeriod_WH_Urgent{0};           //prognostication period for WH, urgent mode, s
    qreal       PrognPeriod_WH_Urgent_HI_LOAD{0};   //prognostication period for WH, urgent mode, high load, s
    qreal       PrognPeriod_Boost{0};               //prognostication period for boosters, s
    qreal       PrognPeriod_Boost_HI_LOAD{0};       //prognostication period for boosters, high load,s

    qreal       Gamma_st{0};                //average ballistic coefficient
    qreal       K_Gamma{0};                 //coefficient for Gamma in 1st method of start point computation
    qreal       dT_def_start_2method{0};	//interval of try for 2st method of start point computation when 1st method is executed, s
    qreal       dT_corr_progn{0};           //time interval for correction of previous prolonged track, s
    One_Pr_Const    OnePrConsts;            //constants for ball_one_prognoz task

    Pr_Const();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Pr_Const::Reset
    // DESCRIPTION	:   Reset of struct attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   AirBallist
    // FUNCTION 	:   Pr_Const::operator=
    // DESCRIPTION	:   Assignment operator
    // INPUTS		:	Pr_Const
    // RETURNS		:	Pr_Const
    Pr_Const& operator=(const Pr_Const &Q);

    explicit Pr_Const (const Pr_Const &Q);
};


#endif

