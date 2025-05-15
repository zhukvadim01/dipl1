//
//FILE			:rip_tc_prognoz.h
//
//AUTHOR		:tatka,	13 May 2006
//
#ifndef _rip_tc_prognozm_h
#define _rip_tc_prognozm_h

#include<math.h>
#include<vector>

#include "air_ballist_constants.h"
#include "ball_prognoz_structs.h"

#include "gl/GLMatrix.h"
#include "gl/GLArray.h"

#include "conversion/Topocentric.h"


//PACKAGE		:   AirBallist
//CLASS			:   COnePrognosis
//DESCRIPTION	:   Prognostication of BT (Ballistic Target)
class COnePrognosis_M
{
protected:	
    std::vector<M10>	Param;  //vector of coordinate parameters
    std::vector<qreal> VectH;   //vector of heights
    qint32      dim{0};         //point to write in the array Param //M
    qreal		RK[4][6];       //|
    qreal		F[6];           //|
    qreal		UK[6];          //|->arrays for solving differential equation
    qreal		AK{0.}; qreal BK{0.}; qreal CK{0.};	//|  by Runge-Kutta method
    qreal		H0{0.}; qreal VK{0.}; qreal RoK{0.};	//|
    qreal       Hmax{0.};       //maximum value of height, m
    qreal       tApogee{AIR_BALL::Timer_INI}; //time of the apogee, s
    qreal		T{0.};			//|
    qreal		Step{0.};		//|
    GLPointDouble3D	CalcStartPoint;
    qreal           Tpass{0.};              //time interval of motion on passive path
    bool            SignCorrectProl{true};  //sign of correct prolongation, true if prolongation is correct
    Polynoms_data   PolynomComb;            //combined polynom (4th and 8th deg) in ECEF coordinate system
    Polynoms_data   PolynomCombNUE;         //combined polynom (4th and 8th deg) in NUE coordinate system

    FILE* m_pLogEll{nullptr}; //pointer to the log file for ellipse parameters

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::Solution
    //DESCRIPTION	:   Function for solving system of differential equations
    //INPUTS		:   Sign of inverse prediction; ballisitc coefficient, m^2/kg
    //RETURNS		:   Accuracy detector: 0 - sufficient accuracy, 1 - insufficient accuracy
    qint16 Solution(const bool P_Inverse, const qreal gamma);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::SolutionAeroball
    //DESCRIPTION	:   Function for solving system of differential equations for aeroballistic missile
    //INPUTS		:   Ballisitc coefficient, m^2/kg; lift coefficient, m^2/kg
    //RETURNS		:   Accuracy detector: 0 - sufficient accuracy, 1 - insufficient accuracy
    qint16 SolutionAeroball(const qreal gamma, const qreal delta, M10 &M);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::Check_EAP
    //DESCRIPTION	:   Function for detecting of end of active path
    //INPUTS		:   Reference to the table of the end of active path parameters; current height, m; current angle or departure (throw angle), degrees;
    //              :   current velocity, m/s; resulting value of delta distance between neighbour table values, m;
    //              :   sign of the final decision; resulting value of delta velocity betoeen current and neighbour table values, m/s;
    //              :   resulting values of height (m), distance from start point (m), velocity (m/s) in the point of 1st booster separation;
    //              :   resulting values of height (m), distance from start point (m), velocity (m/s) in the point of 2nd booster separation
    //RETURNS		:   Void
    qreal Check_EAP(const Str_Ar_EAP_Param_1Table *pEAP_table, const qreal currH, const qreal currTheta,
                        const qreal currV, qreal &DeltaL, bool &p_final, qreal &ResDeltaV,
                        qreal &ResHBoost1, qreal &ResLBoost1, qreal &ResVBoost1,
                        qreal &ResHBoost2, qreal &ResLBoost2, qreal &ResVBoost2);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalculateSepBoostParam
    //DESCRIPTION	:   Calculates parameters of boosters separation points using table of EAP and tables of boosters separation points parameters
    //INPUTS		:   Reference to the table of the end of active path parameters and tables of boosters separation points parameters;
    //              :   number of the row in the tables of EAP and separation points parameters;
    //              :   current value of the angle of departure (throw angle), degrees;
    //              :   resulting values of height (m), distance from start point (m), velocity (m/s) in the point of 1st booster separation;
    //              :   resulting values of height (m), distance from start point (m), velocity (m/s) in the point of 2nd booster separation
    //RETURNS		:   Void
    void CalculateSepBoostParam(const Str_Ar_EAP_Param_1Table *pEAP_table, const qint16 i_row, const qreal CurrTheta,
                                qreal &ResHBoost1, qreal &ResLBoost1, qreal &ResVBoost1,
                                qreal &ResHBoost2, qreal &ResLBoost2, qreal &ResVBoost2);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetDensity
    //DESCRIPTION	:   Function for calculation of density
    //INPUTS		:   Coordinates x, y, z in ECEF, m
    //RETURNS		:   Air density, kg/m^3
    qreal GetDensity(const qreal x, const qreal y, const qreal z);
	
    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetAbsVector
    //DESCRIPTION	:   Function for calculation of absolute meaning of 3D-vector
    //INPUTS		:   Coordinates x, y, z of 3D-vector
    //RETURNS		:   Absoulute value of vector
    qreal GetAbsVector(const qreal x, const qreal y, const qreal z);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetHeight
    //DESCRIPTION	:   Function for calculation of the height above ellipsoid surface
    //INPUTS		:   Coordinates x, y, z in ECEF, m
    //RETURNS		:   Height, m
    qreal GetHeight(const qreal x, const qreal y, const qreal z);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::RecalcSystemCoefs
    //DESCRIPTION	:   Function for calculation of system coefficients
    //INPUTS		:   Coordinates x, y, z in ECEF, m; resulting values of coefficients A, B, C in differential equation of ballistic motion
    //RETURNS		:   Void
    void RecalcSystemCoefs(const qreal x, const qreal y, const qreal z,
        qreal &C, qreal &B, qreal &A);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::RungeKutta
    //DESCRIPTION	:   Function for calculation Runge-Kutta coefficients
    //INPUTS		:   Vector uk of parameters (3 coordinates + 3 velocity components) in initial point;
    //              :   coefficients A, B, C in differential equation of ballistic motion; air dencity, kg/m^3; absolute velocity, m/s;
    //              :   ballistic coefficient, m^2/kg; step, s; sign of inverse prolongation
    //RETURNS		:   Void
    void RungeKutta(const qreal uk[],  const qreal Ak,  const qreal Bk,  const qreal Ck,  const qreal Rok,  const qreal Vk, const qreal gam,  const qreal h, const bool P_Inverse);
	
    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::RungeKutta_Aeroball
    //DESCRIPTION	:   Function for calculation Runge-Kutta coefficients for aeroballistic missile
    //INPUTS		:   Vector uk of parameters (3 coordinates + 3 velocity components) in initial point;
    //              :   coefficients A, B, C in differential equation of ballistic motion; air dencity, kg/m^3; absolute velocity, m/s;
    //              :   ballistic coefficient, m^2/kg; lift coefficient, m^2/kg; step, s
    //RETURNS		:   Void
    void RungeKutta_Aeroball(const qreal uk[],  const qreal Ak,  const qreal Bk,  const qreal Ck,  const qreal Rok,  const qreal Vk, const qreal gam, const qreal delta,  const qreal h);


    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::F_Diff_Eq
    //DESCRIPTION	:   Function defining the system of differential equations
    //INPUTS		:   Vector uk of parameters (3 coordinates + 3 velocity components) in initial point;
    //              :   coefficients A, B, C in differential equation of ballistic motion; air dencity, kg/m^3; absolute velocity, m/s;
    //              :   ballistic coefficient, m^2/kg
    //RETURNS		:   Void
    void F_Diff_Eq(const qreal U1[], const qreal A1, const qreal B1, const qreal C1, const qreal Ro1, const qreal V1, const qreal Gam1);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::F_Diff_Eq_Inverse
    //DESCRIPTION	:   Function defining the system of differential equation of ballistic target movement in inverse prognosis
    //INPUTS		:   Vector uk of parameters (3 coordinates + 3 velocity components) in initial point;
    //              :   coefficients A, B, C in differential equation of ballistic motion; air dencity, kg/m^3; absolute velocity, m/s;
    //              :   ballistic coefficient, m^2/kg
    //RETURNS		:   Void
    void F_Diff_Eq_Inverse(const qreal U1[], const qreal A1, const qreal B1, const qreal C1, const qreal Ro1, const qreal V1, const qreal Gam1);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::F_Diff_Eq_Aeroball
    //DESCRIPTION	:   Function defining the system of differential equations during aeroballistic maneuver
    //INPUTS		:   Vector uk of parameters (3 coordinates + 3 velocity components) in initial point;
    //              :   coefficients A, B, C in differential equation of ballistic motion; air dencity, kg/m^3; absolute velocity, m/s;
    //              :   ballistic coefficient, m^2/kg; lift coefficient, m^2/kg
    //RETURNS		:   Void
    void F_Diff_Eq_Aeroball(const qreal U1[], const qreal A1, const qreal B1, const qreal C1, const qreal Ro1, const qreal V1, const qreal Gam1, const qreal Delta1);

    //FUNCTION		:   COnePrognosis::GetPolinom_Ndeg
    //DESCRIPTION	:   Calculates coefficients of polinom of degree N; where N is even number
    //INPUTS		:   N - degree of polynom, ind_begin - index of 1st element under consideration;
    //              :   &ProlTrack - reference on whole prolonged track;
    //              :   ind_end - index of last element under consideration
    //RETURNS		:   array of polynomial coefficients; anchor time;
    //              :   time moment to fix one of output points
    //              :   result is true if the function is successful
    bool GetPolinom_Ndeg(const qint16 N, std::vector<M10> &ProlTrack, const qint32 ind_begin, const qint32 ind_end,
                         CGLArrayFix<GLPointDouble3D, SIZE_ARR_POLYNOM> &PolData, qreal &AnchorTime, qreal &FixTime);

public:
    One_Pr_Const	m_PrConst;	//Constants

    COnePrognosis_M() = delete;
    explicit COnePrognosis_M(FILE* &out_param /*= nullptr*/);
    ~COnePrognosis_M();


    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::Reset
    //DESCRIPTION	:   Reset class attributes
    //INPUTS		:   void
    //RETURNS		:   void
    void Reset();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::InitConst
    //DESCRIPTION	:   Initialization of constants
    //INPUTS		:   Reference to structure for constants space
    //RETURNS		:   void
    void InitConst(const One_Pr_Const &cnst);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalcTrack
    //DESCRIPTION	:   Prolonged track calculation
    //INPUTS		:   initial time (s), coordinates (m), velocity components (m/s) in geocentric coordinate system (ECEF);
    //              :   ballistic coefficient, m^2/kg; sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS		:   void
    void CalcTrack(const qreal t0, const qreal x0, const qreal y0, const qreal z0,
                    const qreal v0x, const qreal v0y, const qreal v0z, const qreal gamma, const bool P_Inverse);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalcTrack
    //DESCRIPTION	:   Prolonged track calculation
    //INPUTS		:   initial and final time (s), coordinates(m), velocity components (m/s) in geocentric coordinate system (ECEF);
    //              :   ballistic coefficient, m^2/kg; sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS		:   true if result is ok
    bool CalcTrack_OnTime(const qreal t0, const qreal t_end, const qreal x0, const qreal y0, const qreal z0,
                    const qreal v0x, const qreal v0y, const qreal v0z, const qreal gamma, const bool P_Inverse);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalcTrack_Aeroball
    //DESCRIPTION	:   Prolonged track calculation for aeroballistic missile
    //INPUTS		:   initial time (s); initial time of an aeroballistic maneuver (s);
    //              :   coordinates (m), velocity components (m/s) in geocentric coordinate system (ECEF);
    //              :   ballistic coefficient, m^2/kg; lift coefficient, m^2/kg
    //RETURNS		:   true if result is OK
    bool CalcTrack_Aeroball(const qreal t0, const qreal t0_aeroball, const qreal x0, const qreal y0, const qreal z0,
                    const qreal v0x, const qreal v0y, const qreal v0z, const qreal gamma, const qreal delta);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::OutputTrack
    //DESCRIPTION	:   composition of a log-file for review CalcTrack function results
    //INPUTS		:   void
    //RETURNS		:   void
    void OutputTrack();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::ClearTrack
    //DESCRIPTION	:   clear attribute Param
    //INPUTS		:   void
    //RETURNS		:   void
    void ClearTrack();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::OutputLastPoint
    //DESCRIPTION	:   output last point in Param
    //INPUTS		:   void
    //RETURNS		:   time (s), size of Param,
    //              :   coordinates (m), velocities (m/s), accelerations (m/s^2) of last point in Param
    //              :   result is true if Param is not empty
    bool OutputLastPoint( qreal &Time,  qint32 &Size, GLPointDouble3D &Coord, GLPointDouble3D &Vel, GLPointDouble3D &Acc);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::OutputLastPoint
    //DESCRIPTION	:   output last point in Param
    //INPUTS		:   void
    //RETURNS		:   time (s), size of Param,
    //              :   coordinates (m), velocities (m/s) of last point in Param
    //              :   result is true if Param is not empty
    bool OutputLastPoint(qreal &Time, qint32 &Size, GLPointDouble3D &Coord, GLPointDouble3D &Vel);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetProlongedTrack
    //DESCRIPTION	:   Returns prolonget track from the current point to the last point
    //INPUTS		:   void
    //RETURNS		:   Prolonget track from the current point to the last point;
    //              :   result is true if Param is not empty
    void GetProlongedTrack(std::vector<M10> &ProlongedTrack, qint32 &dimension);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::OutputStartPointByTables
    //DESCRIPTION	:   output start point calculated using tables
    //INPUTS		:   Current time; pointer to table containing EAP parameters
    //RETURNS		:   Start time;
    //              :   coordinates (m, in Geocentric System) of start point calculated by table
    bool OutputStartPointByTables(const qreal &CurrTime, const Str_Ar_EAP_Param_1Table *pEAP_table,
                                  qreal &StartTime, GLPointDouble3D &Coord);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::OutputStartPointByTables
    //DESCRIPTION	:   output start point calculated using tables
    //INPUTS		:   Current time; pointer to table containing EAP parameters
    //RETURNS		:   Start time;
    //              :   coordinates (m, in Geocentric System) of start point calculated by table;
    //              :   time interval from begin of passive path to the current time;
    //              :   coordinates at the end of active path determined by EAP table;
    //              :   velocity components at the end of active path determined by EAP table
    bool OutputStartPointByTables(const qreal &CurrTime, const Str_Ar_EAP_Param_1Table *pEAP_table,
                                  qreal &StartTime, GLPointDouble3D &CoordSP, qreal &T_passive,
                                  GLPointDouble3D &CoordEAP, GLPointDouble3D &VelEAP);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::OutputLastPoint
    //DESCRIPTION	:   output last point in Param
    //INPUTS		:   void
    //RETURNS		:   time (s), size of Param,
    //              :   coordinates (m), velocities (m/s) of last point in Param in the form of column of matrix
    //              :   result is true if Param is not empty
    bool OutputLastPoint(qreal &Time, qint32 &Size, TMatrix<6> &ColumnCV);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::FindStartByKau
    //DESCRIPTION	:   finding start point by end of active leg characteristics
    //INPUTS		:   Pointer to the EAP table;
    //              :   initial time (s), coordinates (m), velocity components (m/s) in geocentric coordinate system (ECEF);
    //              :   ballistic coefficient, m^2/kg; sign of inverse prognosis (true - inverse, false - direct prognosis)
    //RETURNS       :   true if result is ok;
    //              :   throw angle at the end of active path Theta; absolute velocity at the end of active path;
    //              :   delta of distance between two cells in the table of the ends of active path;
    //              :   resulting values of height (m), distance from start point (m), velocity (m/s) in the point of 1st booster separation;
    //              :   resulting values of height (m), distance from start point (m), velocity (m/s) in the point of 2nd booster separation
    bool FindStartByEAP(Str_Ar_EAP_Param_1Table* pEAP_table, const qreal t0, const qreal x0, const qreal y0, const qreal z0,
                        const qreal v0x, const qreal v0y, const qreal v0z, const qreal gamma, qreal &Theta,
                        qreal &modV, qreal &DeltaL, qreal &ResDeltaV,
                        qreal &ResHBoost1, qreal &ResLBoost1, qreal &ResVBoost1,
                        qreal &ResHBoost2, qreal &ResLBoost2, qreal &ResVBoost2);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalcEllipse
    //DESCRIPTION	:   Dispersion ellipse calculation (with matrix extrapolation)
    //INPUTS		:   Structure of covariance matrix, qreal time of location and fall,
    //              :   structures for Fall point and velocities in Fall point,
    //RETURNS		:   References for qreal ellipses characteristics aI, bI (without multiplicator 3), betaI (radians)
    void CalcEllipse (const TMatrix<DIM_CV> &K, const qreal t, const qreal tp,
                      const GLPointDouble3D &SP, const GLPointDouble3D &SV,
                      qreal &aI, qreal &bI, qreal &betaI);

    void CalcEllipse (TMatrix<SIZE_M> &K, const qreal t, const qreal tp,
                      const GLPointDouble3D &SP, const GLPointDouble3D &SV,
                      qreal &aI, qreal &bI, qreal &betaI);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalcEllipse
    //DESCRIPTION	:   Dispersion ellipse calculation (without matrix extrapolation)
    //INPUTS		:   Structure of covariance matrix,
    //              :   structures for Fall point and velocities in Fall point,
    //RETURNS		:   References for qreal ellipses characteristics aI, bI (without multiplicator 3), betaI
    void CalcEllipse (const TMatrix<DIM_CV> &K, const GLPointDouble3D &SP, const GLPointDouble3D &SV,
                      qreal &aI, qreal &bI, qreal &betaI);

    // PACKAGE		:   AirBallist
    // FUNCTION		:   COnePrognosis::CalcBetaEpsilon
    // DESCRIPTION	:   Calculate azimuth and elevation using vector in Topocentric coordinate system (NUE)
    // INPUTS		:   Reference to vector in Topocentric
    // RETURNS		:   References to azimuth and elevation
    void CalcBetaEpsilon(const CTopocentric &CTC_V, qreal &beta2, qreal &eps2);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetM10_ForPoint
    //DESCRIPTION	:   Forms structure M10 for given point
    //INPUTS		:   Time, coordinates (in ECEF) and velocities (in ECEF) of point, ballistic coefficient
    //RETURNS		:   Point as the structure M10
    void GetM10_ForPoint(const qreal T, const GLPointDouble3D &Coord, const GLPointDouble3D &Vel, const qreal Gamma, M10 &Result);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetPolinom_4deg
    //DESCRIPTION	:   Calculates coefficients of polinom
    //INPUTS		:   void
    //RETURNS		:   void
    void GetPolinom_4deg(Pred_Polinom &Pol);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetSignCorrectProgn
    //DESCRIPTION	:   Returns sign of correct prognosis
    //INPUTS		:   void
    //RETURNS		:   Sign of correct prognosis (true if prognosis is correct)
    bool GetSignCorrectProgn();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetPolinom
    //DESCRIPTION	:   Calculates coefficients of polynoms (4deg and 8deg), in ECEF coordinate system
    //INPUTS		:   Reference on whole prolongated track
    //RETURNS		:   structure Polynom_data of polynomial coefficients and parameters
    //              :   result is true if the function is performed successful
    bool GetPolinom(std::vector<M10> &ProlTrack, Polynoms_data &Pol);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetPolinom
    //DESCRIPTION	:   Calculates coefficients of polynoms (4deg and 8deg), in ECEF coordinate system
    //INPUTS		:   Pointer to the structure Polynom_data of polynomial coefficients and parameters
    //RETURNS		:   Result is true if the function is performed successful
    bool GetPolinom(Polynoms_data &Pol);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetPolynomECEF
    //DESCRIPTION	:   Returns combined polynom in ECEF coordinate system
    //INPUTS		:   None
    //RETURNS		:   Polynomial coefficients and parameters
    Polynoms_data GetPolynomECEF();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetPolynomNUE
    //DESCRIPTION	:   Returns combined polynom in NUE coordinate system
    //INPUTS		:   None
    //RETURNS		:   Polynomial coefficients and parameters
    Polynoms_data GetPolynomNUE();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::CalcPolinom
    //DESCRIPTION	:   Calculates coefficients of polynom, in ECEF coordinate system
    //INPUTS		:   None
    //RETURNS		:   Result is true if the function is performed successful
    bool CalcPolynom();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::RecalcPolynomToNUE
    //DESCRIPTION	:   Realculates coefficients of polynom from ECEF to NUE coordinate system
    //INPUTS		:   Center of NUE; polynom in ECEF; reference to the resulting polynom in NUE
    //RETURNS		:   Result is true if the function is performed successful
    bool RecalcPolynomToNUE(CGeodesic &Center, const Polynoms_data &PolECEF, Polynoms_data &PolNUE);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::RecalcPolynomToNUE
    //DESCRIPTION	:   Realculates coefficients of polynom from ECEF to NUE coordinate system
    //INPUTS		:   Center of NUE; polynom in ECEF; reference to the resulting polynom in NUE
    //RETURNS		:   Result is true if the function is performed successful
    bool RecalcPolynomToNUE(CGeodesic &Center);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetHmax
    //DESCRIPTION	:   Returns maximum height of prolonged track
    //INPUTS		:   None
    //RETURNS		:   Maximum height of prolonged track (m)
    qreal GetHmax();

    //PACKAGE		:   AirBallist
    //FUNCTION		:   COnePrognosis::GetTApogee
    //DESCRIPTION	:   Returns predicted time of the apogee
    //INPUTS		:   None
    //RETURNS		:   Predicted time of the apogee (s)
    qreal GetTApogee();

    // PACKAGE		:	AirBallist
    // FUNCTION		:	COnePrognosis::extrCoordByCombPolynom
    // DESCRIPTION	:	Extrapolation of coordinates for input time using combination of 2 polynoms
    // INPUTS 		:	Given time (seconds elapsed since day's beginning); reference to the polynoms data;
    //              :   reference to the output extrapolated point
    // RETURNS		:   Result is true if the function is performed successful
    bool extrCoordByCombPolynom(const qreal _time, const Polynoms_data &Pol, GLPointDouble3D &ResPoint);

    // PACKAGE		:	AirBallist
    // FUNCTION		:	COnePrognosis::extrCoordByCombPolynom
    // DESCRIPTION	:	Extrapolation of coordinates for input time using combination of 2 polynoms,
    //              :   in ECEF coordinate system
    // INPUTS 		:	Given time (seconds elapsed since day's beginning);
    //              :   reference to the output extrapolated point
    // RETURNS		:   Result is true if the function is performed successful
    bool extrCoordByCombPolynom(const qreal _time, GLPointDouble3D &ResPoint);

    // PACKAGE		:	AirBallist
    // FUNCTION		:	COnePrognosis::extrCoordByCombPolynomNUE
    // DESCRIPTION	:	Extrapolation of coordinates for input time using combination of 2 polynoms,
    //              :   in NUE coordinate system
    // INPUTS 		:	Given time (seconds elapsed since day's beginning);
    //              :   reference to the output extrapolated point
    // RETURNS		:   Result is true if the function is performed successful
    bool extrCoordByCombPolynomNUE(const qreal _time, GLPointDouble3D &ResPoint);

    // PACKAGE		:	AirBallist
    // FUNCTION		:	COnePrognosis::extrCoordVelByCombPolynom
    // DESCRIPTION	:	Extrapolation of coordinates and velocities for input time using combination of 2 polynoms
    // INPUTS 		:	Given time (seconds elapsed since day's beginning); reference to the polynoms data;
    //              :   reference to the output extrapolated point; reference to the output extrapolated velocity
    // RETURNS		:   Result is true if the function is performed successful
    bool extrCoordVelByCombPolynom(const qreal _time, const Polynoms_data &Pol, GLPointDouble3D &ResPoint, GLPointDouble3D &ResVel);

    // PACKAGE		:	AirBallist
    // FUNCTION		:	COnePrognosis::extrCoordVelByCombPolynom
    // DESCRIPTION	:	Extrapolation of coordinates and velocities for input time using combination of 2 polynoms,
    //              :   in ECEF coordinate system
    // INPUTS 		:	Given time (seconds elapsed since day's beginning);
    //              :   reference to the output extrapolated point; reference to the output extrapolated velocity
    // RETURNS		:   Result is true if the function is performed successful
    bool extrCoordVelByCombPolynom(const qreal _time, GLPointDouble3D &ResPoint, GLPointDouble3D &ResVel);

    // PACKAGE		:	AirBallist
    // FUNCTION		:	COnePrognosis::extrCoordVelByCombPolynomNUE
    // DESCRIPTION	:	Extrapolation of coordinates and velocities for input time using combination of 2 polynoms,
    //              :   in NUE coordinate system
    // INPUTS 		:	Given time (seconds elapsed since day's beginning);
    //              :   reference to the output extrapolated point; reference to the output extrapolated velocity
    // RETURNS		:   Result is true if the function is performed successful
    bool extrCoordVelByCombPolynomNUE(const qreal _time, GLPointDouble3D &ResPoint, GLPointDouble3D &ResVel);

    void GetTPSCPolinom(const std::vector<M10>& P, const CGeodesic &Ctpsc, Pred_Polinom &TPol);

    void TransmitPointerToTheLogFiles(FILE *_pLogEll);
};

#endif
