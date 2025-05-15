#include "ball_prognoz_structs.h"

One_Pr_Const::One_Pr_Const()
{
//    Reset();
}

void One_Pr_Const::Reset()
{
    memset(this, 0, sizeof(One_Pr_Const));
}

One_Pr_Const& One_Pr_Const::operator=(const One_Pr_Const &Q)
{
    this->Tcontr      = Q.Tcontr;
    this->h0          = Q.h0;
    this->Delta_Eps   = Q.Delta_Eps;
    this->step        = Q.step;
    this->min_step    = Q.min_step;
    this->min_step_CloseToEarth = Q.min_step_CloseToEarth;
    this->H_Eps       = Q.H_Eps;
    this->H_tolerance = Q.H_tolerance;
    this->VH_tolerance= Q.VH_tolerance;
    this->T_act_def   = Q.T_act_def;
    this->dTheta1     = Q.dTheta1;
    this->dTheta2     = Q.dTheta2;
    return *this;
}

One_Pr_Const::One_Pr_Const (const One_Pr_Const &Q)
{
    this->Tcontr      = Q.Tcontr;
    this->h0          = Q.h0;
    this->Delta_Eps   = Q.Delta_Eps;
    this->step        = Q.step;
    this->min_step    = Q.min_step;
    this->min_step_CloseToEarth = Q.min_step_CloseToEarth;
    this->H_Eps       = Q.H_Eps;
    this->H_tolerance = Q.H_tolerance;
    this->VH_tolerance= Q.VH_tolerance;
    this->T_act_def   = Q.T_act_def;
    this->dTheta1     = Q.dTheta1;
    this->dTheta2     = Q.dTheta2;
}

Pr_Const::Pr_Const()
{
//    Reset();
}

void Pr_Const::Reset()
{
    memset(this, 0, sizeof(Pr_Const));
    OnePrConsts.Reset();
}
Pr_Const& Pr_Const::operator=(const Pr_Const &Q)
{
    this->PrognPeriod_WH_Usual            = Q.PrognPeriod_WH_Usual;
    this->PrognPeriod_WH_Usual_HI_LOAD    = Q.PrognPeriod_WH_Usual_HI_LOAD;
    this->PrognPeriod_WH_Urgent           = Q.PrognPeriod_WH_Urgent;
    this->PrognPeriod_WH_Urgent_HI_LOAD   = Q.PrognPeriod_WH_Urgent_HI_LOAD;
    this->PrognPeriod_Boost               = Q.PrognPeriod_Boost;
    this->PrognPeriod_Boost_HI_LOAD       = Q.PrognPeriod_Boost_HI_LOAD;
    this->Gamma_st                        = Q.Gamma_st;
    this->K_Gamma                         = Q.K_Gamma;
    this->dT_def_start_2method            = Q.dT_def_start_2method;
    this->dT_corr_progn                   = Q.dT_corr_progn;
    this->OnePrConsts                     = Q.OnePrConsts;
    return *this;
}

Pr_Const::Pr_Const (const Pr_Const &Q)
{
    this->PrognPeriod_WH_Usual            = Q.PrognPeriod_WH_Usual;
    this->PrognPeriod_WH_Usual_HI_LOAD    = Q.PrognPeriod_WH_Usual_HI_LOAD;
    this->PrognPeriod_WH_Urgent           = Q.PrognPeriod_WH_Urgent;
    this->PrognPeriod_WH_Urgent_HI_LOAD   = Q.PrognPeriod_WH_Urgent_HI_LOAD;
    this->PrognPeriod_Boost               = Q.PrognPeriod_Boost;
    this->PrognPeriod_Boost_HI_LOAD       = Q.PrognPeriod_Boost_HI_LOAD;
    this->Gamma_st                        = Q.Gamma_st;
    this->K_Gamma                         = Q.K_Gamma;
    this->dT_def_start_2method            = Q.dT_def_start_2method;
    this->dT_corr_progn                   = Q.dT_corr_progn;
    this->OnePrConsts                     = Q.OnePrConsts;
}
