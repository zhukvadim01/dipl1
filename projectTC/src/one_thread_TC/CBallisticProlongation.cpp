#include "CBallisticProlongation.h"
#include "ball_one_prognoz.h"
#include "air_ballist_literal.h"

#include "common/GDEnum.h"

#include "gl/GLMatrix.h"
#include "gl/GLFileLog.h"
#include "gl/GLCalcTrajParam.h"
#include "gl/GLEllipse2D.h"
#include "gl/GLEllipse.h"

#include "conversion/Geocentric_Geo.h"
#include "conversion/Topo_Topographic.h"
#include "conversion/Spherical_Topo.h"

using namespace GLFileLog;

CBallisticProlongation::CBallisticProlongation()
{
    m_arProlData.resize(AIR_BALL::BALL_FORM_AMOUNT);
    try
    {
        m_pLogEll = OpenLog("Ball_EllipseParam", AIR_BALL::log_path);
        m_pLogCorrSP = OpenLog("StartPointCorrection", AIR_BALL::log_path);
    }
    catch(...)
    {
        DPS_ASSERT(false);
    }
//    Reset();
}


CBallisticProlongation::~CBallisticProlongation()
{
    if (m_pLogEll)
    {
        fclose(m_pLogEll);
    }
    m_pLogEll = nullptr;

    if (m_pLogCorrSP)
    {
        fclose(m_pLogCorrSP);
    }
    m_pLogCorrSP = nullptr;
}


void CBallisticProlongation::Reset()
{
    m_pInData = nullptr;
    m_pBallCoef_Tables = nullptr;
    m_pEAP_Tables = nullptr;
    m_p_Arr_BallProc = nullptr;

    m_OneProgn.Reset();
    m_arProlData.Reset();      
    ClearInnerData();

    try
    {
        m_pLogEll = OpenLog("Ball_EllipseParam", AIR_BALL::log_path);
        m_pLogCorrSP = OpenLog("StartPointCorrection", AIR_BALL::log_path);
    }
    catch(...)
    {
        DPS_ASSERT(false);
    }
    m_OneProgn.TransmitPointerToTheLogFiles(m_pLogEll);
}


bool CBallisticProlongation::CovMatrixProlongation_FinDiff(const qreal t0, const qreal t_end, const qreal DeltaC, const qreal DeltaV, const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0, const GLPointDouble3D &Pos_end, const GLPointDouble3D &Vel_end, const TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse, TMatrix<DIM_CV> &CovCVRes)
{
    bool ItsOK = true;
    COnePrognosis currOneProgn; //for appeal to class performing of prolongation
    currOneProgn.InitConst(m_PrognConst.OnePrConsts);
    currOneProgn.TransmitPointerToTheLogFiles(m_pLogEll);

    TCMatrixFunc<DIM_CV> fMatr; //for appeal to class performing of matrix functions
    qint32 j;
    qint32 k;

    GLPointDouble3D Coord0curr;
    GLPointDouble3D Vel0curr; //initial coordinates and velocities with increment

    qreal tProlCurr;  //|
    qint32 Size;          //| - auxiliary parameters for output

    TMatrix<DIM_CV> colProlCenter(DIM_CV,1);
    TMatrix<DIM_CV> colProlCurr(DIM_CV,1);
    TMatrix<DIM_CV> J_fin_diff(DIM_CV,DIM_CV);
    TMatrix<DIM_CV> J_fin_diffT(DIM_CV,DIM_CV);
    TMatrix<DIM_CV> Matr1(DIM_CV,1);
    TMatrix<DIM_CV> MatrDelta(DIM_CV,1);

    colProlCenter.M[0][0] = Pos_end.x;
    colProlCenter.M[1][0] = Pos_end.y;
    colProlCenter.M[2][0] = Pos_end.z;
    colProlCenter.M[3][0] = Vel_end.x;
    colProlCenter.M[4][0] = Vel_end.y;
    colProlCenter.M[5][0] = Vel_end.z;

    for (j=0; j<DIM_CV; j++) //6 initial points with increments
    {
        Coord0curr = Pos0;
        Vel0curr = Vel0;

        switch (j) //addition of increment
        {
        case 0: Coord0curr.x += DeltaC; break;
        case 1: Coord0curr.y += DeltaC; break;
        case 2: Coord0curr.z += DeltaC; break;
        case 3: Vel0curr.x += DeltaV; break;
        case 4: Vel0curr.y += DeltaV; break;
        case 5: Vel0curr.z += DeltaV; break;
        default: break;
        }

        currOneProgn.ClearTrack();
        ItsOK = currOneProgn.CalcTrack_OnTime(t0, t_end, Coord0curr.x, Coord0curr.y, Coord0curr.z, //prolongation for current point with increment
                                      Vel0curr.x, Vel0curr.y, Vel0curr.z, Gamma, P_Inverse);
        if (!ItsOK)
        {
            return false;
        }
        ItsOK = currOneProgn.OutputLastPoint(tProlCurr, Size, colProlCurr);

        Matr1.Reset(6,1);
        MatrDelta.Reset(6,1);
        fMatr.Negative(colProlCenter, Matr1);
        ItsOK = ItsOK && fMatr.Add(colProlCurr, Matr1, MatrDelta);

        for (k=0; k<DIM_CV; k++)
        {
            if (j<3)
            {
                J_fin_diff.M[k][j] = MatrDelta.M[k][0] / DeltaC;
            }
            else
            {
                J_fin_diff.M[k][j] = MatrDelta.M[k][0] / DeltaV;
            }
        }
    }

    ItsOK = ItsOK && fMatr.Transpon(J_fin_diff, J_fin_diffT); //J_fin_diffT is J_fin_diff transposed
    ItsOK = ItsOK && fMatr.MatrXMatrXMatr(J_fin_diff, CovCV0, J_fin_diffT, CovCVRes);

    return ItsOK;
}


bool CBallisticProlongation::CovMatrixProlongation_FinDiff(const qreal t0, const qreal t_end, const qreal DeltaC, const qreal DeltaV, const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0, const TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse, TMatrix<DIM_CV> &CovCVRes)
{
    COnePrognosis currOneProgn;
    currOneProgn.InitConst(m_PrognConst.OnePrConsts);
    currOneProgn.TransmitPointerToTheLogFiles(m_pLogEll);

    bool ItsOK = true;
    GLPointDouble3D Pos_end;
    GLPointDouble3D Vel_end;

    qreal tProlCenter; //|
    qint32 Size;           //| - auxiliary parameters for output

    ItsOK = currOneProgn.CalcTrack_OnTime(t0, t_end, Pos0.x, Pos0.y, Pos0.z, Vel0.x, Vel0.y, Vel0.z, Gamma, P_Inverse); //prolongation for given point
    if (!ItsOK)
    {
        return false;
    }
    ItsOK = ItsOK && currOneProgn.OutputLastPoint(tProlCenter, Size, Pos_end, Vel_end);

    ItsOK = ItsOK && CovMatrixProlongation_FinDiff(t0, t_end, DeltaC, DeltaV, Pos0, Vel0, Pos_end, Vel_end, CovCV0, Gamma, P_Inverse, CovCVRes);

    return ItsOK;
}


bool CBallisticProlongation::CovMatrixProlongation_Unscented(const qreal t0, const qreal t_end, const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0, const GLPointDouble3D &Pos_end, const GLPointDouble3D &Vel_end, TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse, TMatrix<DIM_CV> &CovCVRes)
{
    qint32 jU;

    COnePrognosis currOneProgn; //for appeal to class performing of prolongation
    currOneProgn.InitConst(m_PrognConst.OnePrConsts);
    currOneProgn.TransmitPointerToTheLogFiles(m_pLogEll);

    TCMatrixFunc<DIM_CV> fMatr; //for appeal to class performing of matrix functions
    bool ItsOK = true;

    //GLPointDouble3D Coord0curr, Vel0curr; //initial coordinates and velocities in sigma-points

    TMatrix<DIM_CV> sqrt_Chol(DIM_CV, DIM_CV);
    TMatrix<DIM_CV> Matr1;
    TMatrix<DIM_CV> Matr2;
    TMatrix<DIM_CV> currDelta;
    TMatrix<DIM_CV> currDeltaT;
    TMatrix<DIM_CV> MPoint0(DIM_CV,1);
    TMatrix<DIM_CV> curr_col_Chol(DIM_CV, 1);

    const qint32 numb_SigP = 2*DIM_CV + 1; //number of sigma-points
    TMatrix<DIM_CV> curr_SigPoint_0; //current sigma-point in the initial point
    TMatrix<DIM_CV> Arr_SigPoints_Prol[numb_SigP];      //array of sigma-points in the end point
    qreal Weights[numb_SigP]; //array of weights of sigma-points

    qreal tProlCurr;   //|
    qint32 Size;           //| - auxiliary parameters for output

    Matr1.Reset(6,6);
    qreal Multiplicator = static_cast<qreal>(DIM_CV) + LAMBDA_UNSC_COV; //dimension of matrix + Lambda
    fMatr.Mult(Multiplicator, CovCV0, Matr1); //Matr1 = Multiplicator * CovCV0
    ItsOK = ItsOK && fMatr.CholeskyDecomposition(Matr1, sqrt_Chol); //sqrt_Chol is the "matrix square root" of Matr1

    MPoint0.M[0][0] = Pos0.x; //initial point in the matrix form
    MPoint0.M[1][0] = Pos0.y;
    MPoint0.M[2][0] = Pos0.z;
    MPoint0.M[3][0] = Vel0.x;
    MPoint0.M[4][0] = Vel0.y;
    MPoint0.M[5][0] = Vel0.z;

    for (jU=0; jU<2*DIM_CV; jU++)
    {
        //curr_col_Chol = sqrt_Chol;
        //curr_col_Chol.PickOut(0, DIM_CV, jU, jU, ItsOK); //curr_col_Chol is jU-th column of sqrt_Chol matrix
        ItsOK = ItsOK && fMatr.Submatrix(sqrt_Chol, 0, DIM_CV-1, jU%DIM_CV, jU%DIM_CV, curr_col_Chol); //curr_col_Chol is jU-th column of sqrt_Chol matrix
        if (!ItsOK)
        {
            return false;
        }

        if (jU < DIM_CV)
        {
            ItsOK = ItsOK && fMatr.Add(MPoint0, curr_col_Chol, curr_SigPoint_0); //curr_SigPoint0 = MPoint0 + curr_col_Chol
        }
        else
        {
            ItsOK = ItsOK && fMatr.Subtr(MPoint0, curr_col_Chol, curr_SigPoint_0); //curr_SigPoint0 = MPoint0 - curr_col_Chol;
        }

        currOneProgn.ClearTrack();
        ItsOK = ItsOK && currOneProgn.CalcTrack_OnTime(t0, t_end, curr_SigPoint_0.M[0][0], curr_SigPoint_0.M[1][0],
                            curr_SigPoint_0.M[2][0], curr_SigPoint_0.M[3][0], curr_SigPoint_0.M[4][0],
                            curr_SigPoint_0.M[5][0], Gamma, P_Inverse); //prolongation of current sigma-point
        if (!ItsOK)
        {
            return false;
        }
        ItsOK = ItsOK && currOneProgn.OutputLastPoint(tProlCurr, Size, Arr_SigPoints_Prol[jU]); //Arr_SigPoints_Prol[jU] is the prolongated current sigma-point

        Weights[jU] = 1. / (2. * Multiplicator);
    }

    Arr_SigPoints_Prol[2*DIM_CV].Reset(6,1);
    Arr_SigPoints_Prol[2*DIM_CV].M[0][0] = Pos_end.x; //end point in the matrix form; it is the last element in Arr_SigPoints_Prol
    Arr_SigPoints_Prol[2*DIM_CV].M[1][0] = Pos_end.y;
    Arr_SigPoints_Prol[2*DIM_CV].M[2][0] = Pos_end.z;
    Arr_SigPoints_Prol[2*DIM_CV].M[3][0] = Vel_end.x;
    Arr_SigPoints_Prol[2*DIM_CV].M[4][0] = Vel_end.y;
    Arr_SigPoints_Prol[2*DIM_CV].M[5][0] = Vel_end.z;

    Weights[2*DIM_CV] = LAMBDA_UNSC_COV / Multiplicator; //weight of last element in Arr_SigPoints_Prol

    TMatrix<DIM_CV> MAlpha0_prol(6,1); //Weighted center of prolongated sigma-points

    for (jU=0; jU<=2*DIM_CV; jU++) //calculation of MAlpha0_prol
    {
        Matr1.Reset(6,1);
        fMatr.Mult(Weights[jU], Arr_SigPoints_Prol[jU], Matr1); //Matr1 = Weights[jU] * Arr_SigPoints_Prol[jU]
        Matr2.Reset(6,1);
        Matr2 = MAlpha0_prol;
        ItsOK = ItsOK && fMatr.Add(Matr2, Matr1, MAlpha0_prol); //MAlpha0_prol += Matr1
    }

    for (jU=0; jU<=2*DIM_CV; jU++)
    {
        ItsOK = ItsOK && fMatr.Subtr(Arr_SigPoints_Prol[jU], MAlpha0_prol, currDelta); //currDelta = Arr_SigPoints_Prol[jU] - MAlpha0_prol
        ItsOK = ItsOK && fMatr.Transpon(currDelta, currDeltaT); //currDeltaT is currDelta transposed

        ItsOK = ItsOK && fMatr.MatrXMatr(currDelta, currDeltaT, Matr1); //Matr1 = currDelta * currDeltaT
        fMatr.Mult(Weights[jU], Matr1, Matr2); //Matr2 = Weights[jU] * Matr1

        Matr1.Reset(6,6);
        Matr1 = CovCVRes;
        ItsOK = ItsOK && fMatr.Add(Matr1, Matr2, CovCVRes); //CovCVRes += Matr2
    }

    return ItsOK;
}


bool CBallisticProlongation::CovMatrixProlongation_Unscented(const qreal t0, const qreal t_end, const GLPointDouble3D &Pos0, const GLPointDouble3D &Vel0, TMatrix<DIM_CV> &CovCV0, const qreal Gamma, const bool P_Inverse, TMatrix<DIM_CV> &CovCVRes)
{
    COnePrognosis currOneProgn;
    currOneProgn.InitConst(m_PrognConst.OnePrConsts);
    currOneProgn.TransmitPointerToTheLogFiles(m_pLogEll);

    bool ItsOK = true;
    GLPointDouble3D Pos_end;
    GLPointDouble3D Vel_end;

    qreal tProlCenter; //|
    qint32 Size;           //| - auxiliary parameters for output

    ItsOK = currOneProgn.CalcTrack_OnTime(t0, t_end, Pos0.x, Pos0.y, Pos0.z, Vel0.x, Vel0.y, Vel0.z, Gamma, P_Inverse);
    if (!ItsOK)
    {
        return false;
    }
    ItsOK = ItsOK && currOneProgn.OutputLastPoint(tProlCenter, Size, Pos_end, Vel_end);

    ItsOK = ItsOK && CovMatrixProlongation_Unscented(t0, t_end, Pos0, Vel0, Pos_end, Vel_end, CovCV0, Gamma, P_Inverse, CovCVRes);

    return ItsOK;
}


bool CBallisticProlongation::CovMatrixExtrapolation(const qreal t0, const qreal t_end, TMatrix<DIM_CV> &CovCV0, TMatrix<DIM_CV> &CovCVRes)
{
    bool bRes = true;
    qreal dt = t_end - t0;
    qint16 i;
    TMatrix<DIM_CV> Mextr(6,6), MextrT(6,6); //extrapolation matrix and tranposed extrapolation matrix
    for (i=0; i<3; i++)
    {
        Mextr.M[i][i] = 1.;
        Mextr.M[i+3][i+3] = 1.;
        Mextr.M[i][i+3] = dt;
    }

    TCMatrixFunc<DIM_CV> fMatr;
    MextrT = fMatr.Transpon(Mextr);

    bRes = fMatr.MatrXMatrXMatr(Mextr, CovCV0, MextrT, CovCVRes);
    return bRes;
}


void CBallisticProlongation::CalcStartPoint(CBallProl_Output &ProlOut)
{
    ProlOut.Reset();
    if (m_p_Arr_BallProc == nullptr)
    {
        return;
    }

    m_OutData.Reset();
    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;
//    qint32 i;

    m_OutData.p_StartIsChanged = false;

    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        bool bCounterExeeded = false; //true if counter is exceeded
        bool bCalcByDownPr = CheckConditionsForStartPointDownPr(bCounterExeeded);

        CAirBall_Proc_InnerData_GT *pGTData = &m_p_Arr_BallProc->arData_GT[ind_GT];

        if (bCalcByDownPr)
        {
            bool bOK = CalcStartPointByInvProl();
            m_arProlData[ind_ball].tPrevTryDetStartDownPr = m_pInData->tLoc;
            if (!bOK)
            {
                return;
            }            
        }
        else
        {
            if (bCounterExeeded
                    && pGTData->Branch == AIR_BALL::ASCENDING_BRANCH
                    && (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::UNDEFINED_START_POINT
                        || (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_DOWN_PROGNOSIS
                            && m_arProlData[ind_ball].StartPoint.IsZero())))
            {
                EstimationStartPointByReverseExtr();
            }

            if (fabs(m_pInData->tLoc - m_arProlData[ind_ball].tPrevTryDetStartByTables) > con_eps)
            {
                m_arProlData[ind_ball].tPrevTryDetStartByTables = m_pInData->tLoc;

                if ( m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_DOWN_PROGNOSIS
                     || m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_EAP_MULTIVAL
                     || ( (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_1ST_POINT)
                          && !m_arProlData[ind_ball].bStartByTablesPerformed ) )
                {
                    //perform start point calculation by EAP Tables if start point was calculated by down prognosis
                    //or if start point was calculated for multiple valued mark
                    //or if start point was corrected by first point after start point calculation by down prognosis
                    BMMark_AndDiapason *pCurMark = &pGTData->BMMark;
                    const qint16 BMMarkVal = pCurMark->MarkValue;
                    if ( (BMMarkVal > AIR_BALL::UNKNOWN_OF_SHORT_RANGE)
                         && (BMMarkVal < AIR_BALL::UNKNOWN_OF_LONG_RANGE)
                         && (pCurMark->IsSingleValued)
                         && (pCurMark->MarkValue != m_arProlData[ind_ball].MarkEAPDet)
                         && (fabs(m_pInData->tLoc - m_arProlData[ind_ball].tDetStartDownPr) <= c_dT_def_start_2method
                             || (fabs(m_arProlData[ind_ball].tDetStartDownPr - AIR_BALL::Timer_INI) < con_par_eps
                                 && (fabs(m_pInData->tLoc - pGTData->tEAP) <= c_dT_def_start_2method)))
                         && (m_pEAP_Tables != nullptr))
                    {
                        //            qreal Gamma = m_p_Arr_BallProc->Form[ind_GT].Gamma;
                        m_OneProgn.Reset();
                        Str_Ar_EAP_Param_1Table* pEAP_table = m_pEAP_Tables->getCellMark(BMMarkVal);
                        if (pEAP_table != nullptr)
                        {
                            CalcStartPointByTable(pEAP_table, true);
                        }
                    }
                }

                if ( m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_DOWN_PROGNOSIS
                     || ( m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_EAP_MULTIVAL && m_arProlData[ind_ball].bBMMarkChanged)
                     || ( m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_1ST_POINT
                          && !m_arProlData[ind_ball].bStartByTablesPerformed
                          && !m_arProlData[ind_ball].bStartByTablesPerformed_multival) )
                {
                    //perform start point calculation by EAP Tables for multiple valued BM mark
                    //if start point was calculated by down prognosis
                    //or if start point was corrected by first point after start point calculation by down prognosis
                    BMMark_AndDiapason *pCurMark = &pGTData->BMMark;
                    if ( (pCurMark->maxBMMark > AIR_BALL::UNKNOWN_OF_SHORT_RANGE)
                         && (pCurMark->minBMMark < AIR_BALL::UNKNOWN_OF_LONG_RANGE)
                         && (!pCurMark->IsSingleValued)
                         && (fabs(m_pInData->tLoc - m_arProlData[ind_ball].tDetStartDownPr) <= c_dT_def_start_2method
                             || (fabs(m_arProlData[ind_ball].tDetStartDownPr - AIR_BALL::Timer_INI) < con_par_eps
                                 && (fabs(m_pInData->tLoc - pGTData->tEAP) <= c_dT_def_start_2method)))
                         && (m_pEAP_Tables != nullptr))
                    {
                        m_arProlData[ind_ball].DeltaVmin = 0;
                        for (qint16 i = pCurMark->minBMMark; i<= pCurMark->maxBMMark; i++)
                        {
                            m_OneProgn.Reset();
                            Str_Ar_EAP_Param_1Table* pEAP_table = m_pEAP_Tables->getCellMark(i);
                            if (pEAP_table != nullptr)
                            {
                                CalcStartPointByTable(pEAP_table, false);
                            }
                            //                        if (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_EAP_MULTIVAL) //if start point is estimated
                            //                        {
                            //                            break;
                            //                        }
                        }
                    }
                }

                m_arProlData[ind_ball].bBMMarkChanged = false;
            }
        }

        if (m_OutData.p_StartIsChanged)
        {
            //correction of start point
            CorrectionStartPoint();

            bool bHorManeuver = fabs(pGTData->ManTime - AIR_BALL::Timer_INI) > con_eps      //sign of horizontal maneuver
                                    && pGTData->ManTime > pGTData->tFirst
                                    && m_pInData->tLoc > pGTData->ManTime;

            if (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_1ST_POINT)
            {
                //correction of prolonged track
                CorrInverseProlTrack();
//                M10 SP;
//                SP.M_info[0] = m_arProlData[ind_ball].tStart;
//                SP.M_info[1] = m_arProlData[ind_ball].StartPoint.x;
//                SP.M_info[2] = m_arProlData[ind_ball].StartPoint.y;
//                SP.M_info[3] = m_arProlData[ind_ball].StartPoint.z;
//                for (i=4; i<10; i++)
//                {
//                    SP.M_info[i] = 0.;
//                }
//                M10 CurrPoint;
//                std::vector<M10> TrackFragment;
//                bool bTrackFragmFilled = false;
//                qreal TimeGlue = m_pInData->tLoc;
//                if (bHorManeuver)
//                {
//                    TimeGlue = pGTData->ManTime;
//                }
//                std::vector<M10>::iterator it;
//                for (it = m_arProlData[ind_ball].ProlongedTrack.begin(); it != m_arProlData[ind_ball].ProlongedTrack.end(); it++)
//                {
//                    M10 CurEl = (*it);
//                    if (CurEl.M_info[0] < TimeGlue)
//                    {
//                        TrackFragment.push_back(CurEl);
//                        bTrackFragmFilled = true;
//                    }
//                }
//                if (!bTrackFragmFilled)
//                {
//                    m_OneProgn.GetM10_ForPoint(m_pInData->tLoc, m_pInData->Coord, m_pInData->Vel,
//                                               pGTData->Gamma, CurrPoint);
//                    TrackFragment.push_back(CurrPoint);
//                }
//                CorrInverseProlTrack(SP, TrackFragment);
            }
            else
            {
                if (bHorManeuver
                        && (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_DOWN_PROGNOSIS
                            || m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_EAP
                            || m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_BY_EAP_MULTIVAL))
                {
                    CorrInverseProlTrack();
                }
            }
        }
    }

    ProlOut = m_OutData;
}


bool CBallisticProlongation::CalcStartPointByInvProl()
{
    bool bRes = false;
    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;
    qint32 i;
    qint32 j;

    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT
            && (m_p_Arr_BallProc->arData_GT[ind_GT].Path == AIR_BALL::END_OF_ACTIVE_PATH
                   || m_p_Arr_BallProc->arData_GT[ind_GT].Path == AIR_BALL::PASSIVE_PATH))
    {
        const qreal Gamma = m_p_Arr_BallProc->arData_GT[ind_GT].Gamma;
        const qreal KGamma = CalcKGamma(m_pInData->H);

        m_OneProgn.Reset();
        //reverse prolongation
        m_OneProgn.CalcTrack(m_pInData->tLoc, m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z,
                               - m_pInData->Vel.x, - m_pInData->Vel.y, - m_pInData->Vel.z,
                               Gamma * KGamma, true);
        std::vector<M10> ProlTrack;
        qint32 dimension = 0;
        m_OneProgn.GetProlongedTrack(ProlTrack, dimension);
        if (!ProlTrack.empty() && m_OneProgn.GetSignCorrectProgn())
        {
            if (m_p_Arr_BallProc->arData_GT[ind_GT].Branch == AIR_BALL::DESCENDING_BRANCH) //m_OneProgn.GetHmax() > m_arProlData[ind_ball].Hapogee)
            {
                m_arProlData[ind_ball].Hapogee = m_OneProgn.GetHmax();
                qreal tApogeeNew = AIR_BALL::c2 * m_pInData->tLoc - m_OneProgn.GetTApogee();
                UpdateApogeeTime(ind_ball, tApogeeNew);
            }

            //output last prolongated point
            GLPointDouble3D CoordSP(0.,0.,0.);
            GLPointDouble3D VelSP(0.,0.,0.);
            qint32 size = 0;
            qreal Tprol = 0.;
            bool ItsOK = m_OneProgn.OutputLastPoint(Tprol, size, CoordSP, VelSP);
            if (!ItsOK || CoordSP.IsZero())
            {
                return false;
            }

            GLCalcTrajParam FCalc;
            qreal H_SP = FCalc.CalcHeight(CoordSP); //height of the start point
            if (H_SP >  c_H_tolerance + c_H_Eps)
            {
                return false;
            }

            //Calculation of ellipsoid in start point
            EllipseInfo EllInfo;
            TMatrix<DIM_CV> CovSP;
            ItsOK = CalculateEllipse(m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.MarkValue,
                             m_p_Arr_BallProc->arData_GT[ind_GT].TrajType,
                             m_pInData->tLoc, Tprol, m_pInData->CovMatr,
                             m_pInData->Coord, m_pInData->Vel, CoordSP, VelSP,
                             m_pInData->Sign_large_load, Gamma * KGamma, true, EllInfo, CovSP);
            if (!ItsOK)
            {
                return false;
            }

            qreal tStart = 2.* m_pInData->tLoc - Tprol;

            bool bCheckOK = CheckStartPoint(tStart, CoordSP, CovSP);
            if (!bCheckOK)
            {
                m_arProlData[ind_ball].CounterBadInvProl++;
                if (m_arProlData[ind_ball].CounterBadInvProl > cCounterBadInvProl)
                {
                    m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_ABSENCE;
                }
            }

            ItsOK = ItsOK && bCheckOK;
            if (ItsOK)
            {
                m_arProlData[ind_ball].StartEll = EllInfo;

                //down-prolonged track storage
                qint32 Size = ProlTrack.size();
                for (j=1; j<Size; j++)
                {
                    ProlTrack[j].M_info[0] = 2. * m_pInData->tLoc - ProlTrack[j].M_info[0];
                    ProlTrack[j].M_info[4] = - ProlTrack[j].M_info[4]; //because inverse prolongation
                    ProlTrack[j].M_info[5] = - ProlTrack[j].M_info[5];
                    ProlTrack[j].M_info[6] = - ProlTrack[j].M_info[6];

                }
                i=0;
                std::vector<M10>().swap(m_arProlData[ind_ball].ProlongedTrack); //cleaning prolonged track in the array m_arProlData
                m_arProlData[ind_ball].ProlongedTrack.push_back(ProlTrack[Size-1]);
                for (j=1; j<Size; j++)
                {
                    if (ProlTrack[Size-1-j].M_info[0] - m_arProlData[ind_ball].ProlongedTrack[i].M_info[0]
                            > c_dT_storage - c_Epsilon_dTst
                            || j == Size-1)
                    {
                        i++;
                        m_arProlData[ind_ball].ProlongedTrack.push_back(ProlTrack[Size-1-j]);
                    }
                }

                //start point storage
                m_arProlData[ind_ball].StartPoint = CoordSP;
                m_arProlData[ind_ball].tStart = tStart; //2.* m_pInData->tLoc - Tprol;
                m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_BY_DOWN_PROGNOSIS;
                m_arProlData[ind_ball].tDetStartDownPr = m_pInData->tLoc;
                m_arProlData[ind_ball].bStartByTablesPerformed = false; //mean that start point calculation by EAP tables not performed
                m_arProlData[ind_ball].bStartByTablesPerformed_multival = false;
                m_OutData.p_StartIsChanged = true;
                bRes = true;
            }
        }
    }
    return bRes;
}


void CBallisticProlongation::CalcStartPointByTable(Str_Ar_EAP_Param_1Table *pEAP_table, bool bSingleValued)
{
    if (pEAP_table == nullptr)
    {
        return;
    }

    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;

    CAirBall_Proc_InnerData_GT *pGTData = &m_p_Arr_BallProc->arData_GT[ind_GT];

    const qreal Gamma = pGTData->Gamma;
    qreal Theta = 0.;
    qreal modVeap = 0.;
    bool ItsOK;
//    qint32 i;
//    qint32 j;
    qreal DeltaL = 0;
    qreal ResDeltaV = 0;
    qreal ResHBoost1=0, ResLBoost1=0, ResVBoost1=0, ResHBoost2=0, ResLBoost2=0, ResVBoost2=0;

    ItsOK = m_OneProgn.FindStartByEAP(pEAP_table, m_pInData->tLoc,
                                      m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z,
                                      -m_pInData->Vel.x, -m_pInData->Vel.y, -m_pInData->Vel.z,
                                      Gamma, Theta, modVeap, DeltaL, ResDeltaV,
                                      ResHBoost1, ResLBoost1, ResVBoost1, ResHBoost2, ResLBoost2, ResVBoost2);
    bool bKeepData_multi = false;
    if (!bSingleValued
            && (fabs(m_arProlData[ind_ball].DeltaVmin) < con_eps
                || (fabs(ResDeltaV) > con_eps && fabs(ResDeltaV) < fabs(m_arProlData[ind_ball].DeltaVmin))))
    {
        m_arProlData[ind_ball].DeltaVmin = fabs(ResDeltaV);
        bKeepData_multi = true;
    }

    if (ItsOK && (bSingleValued || bKeepData_multi)) //EAP is found
    {
        qreal StartTime = 0.;
        qreal T_passive = 0.;
        GLPointDouble3D CoordSP(0.,0.,0.);
        GLPointDouble3D CoordEAP(0.,0.,0.);
        GLPointDouble3D VelEAP(0.,0.,0);
        ItsOK = m_OneProgn.OutputStartPointByTables(m_pInData->tLoc, pEAP_table,
                           StartTime, CoordSP, T_passive, CoordEAP, VelEAP);

        qreal LCurrFst = m_pInData->Coord.getDistanceOnSurface(pGTData->FirstPoint);
        qreal LCurrStart = m_pInData->Coord.getDistanceOnSurface(CoordSP);

        if (!ItsOK || CoordSP.IsZero() || LCurrFst > LCurrStart)
        {
            return;
        }

        if (pGTData->Branch == AIR_BALL::DESCENDING_BRANCH) //m_OneProgn.GetHmax() > m_arProlData[ind_ball].Hapogee)
        {
            m_arProlData[ind_ball].Hapogee = m_OneProgn.GetHmax();
            qreal tApogeeNew = AIR_BALL::c2 * m_pInData->tLoc - m_OneProgn.GetTApogee();
            UpdateApogeeTime(ind_ball, tApogeeNew);
        }       

        //Calculation of ellipsoid in start point
        EllipseInfo EllInfo;
        ItsOK = CalculateEllipse(pGTData->BMMark.MarkValue,
                                 pGTData->TrajType,
                                 m_pInData->tLoc, m_pInData->tLoc+T_passive, m_pInData->CovMatr,
                                 m_pInData->Coord, m_pInData->Vel*(-1.),
                                 CoordEAP, VelEAP, m_pInData->Sign_large_load,
                                 Gamma, true, EllInfo);
        if (!ItsOK || fabs(EllInfo.aI) < con_eps)
        {
            return;
        }
        qreal CoefEll = 1. + c_coefDeltaL*fabs(DeltaL)/EllInfo.aI;
        EllInfo.aI *= CoefEll;
        EllInfo.bI *= CoefEll;
        m_arProlData[ind_ball].StartEll = EllInfo;

        //storage of start point
        m_arProlData[ind_ball].tStart = StartTime;
        m_arProlData[ind_ball].StartPoint = CoordSP;

        qreal tEAP_by_table = m_pInData->tLoc - T_passive;
        CorrInverseProlTrack(true, tEAP_by_table, CoordEAP);

        if (bSingleValued)
        {
            m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_BY_EAP;
            m_arProlData[ind_ball].bStartByTablesPerformed = true;
        }
        else
        {
            m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_BY_EAP_MULTIVAL;
            m_arProlData[ind_ball].bStartByTablesPerformed_multival = true;
        }

        //points of boosters separation
        if (ResHBoost1 > con_eps && fabs(ResLBoost1) > con_eps && ResVBoost1 > con_eps)
        {
//            GLPointDouble3D P_EAP(TrackFragment[0].M_info[1], TrackFragment[0].M_info[2], TrackFragment[0].M_info[3]);
            bool bOK = CalcSepBoostPoint(StartTime, pEAP_table->t_SepBoost1, CoordSP, CoordEAP,
                                         ResHBoost1, ResLBoost1, ResVBoost1, m_OutData.PointSepBoost1);
            if (bOK)
            {
                m_OutData.bBoostSepPointsChanged = true;

                if (ResHBoost2 > con_eps && fabs(ResLBoost2) > con_eps && ResVBoost2 > con_eps)
                {
                    bOK = CalcSepBoostPoint(StartTime, pEAP_table->t_SepBoost2, CoordSP, CoordEAP,
                                            ResHBoost2, ResLBoost2, ResVBoost2, m_OutData.PointSepBoost2);
                }
            }
        }

        m_arProlData[ind_ball].ThrowAngle = Theta;
        m_arProlData[ind_ball].Veap = modVeap;
        m_arProlData[ind_ball].Teap_byTable = tEAP_by_table;
        m_arProlData[ind_ball].MarkEAPDet = pEAP_table->BallMark_inner;
        m_OutData.p_StartIsChanged = true;
    }
}


void CBallisticProlongation::EstimationStartPointByReverseExtr()
{
    const qint32 indGT = m_pInData->IF_GT;
    const qint32 indBall = m_pInData->IF_BALL;
    if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= indBall && indBall < AIR_BALL::BALL_FORM_AMOUNT)
    {
        CAirBall_Proc_InnerData_GT *pDataGT = &m_p_Arr_BallProc->arData_GT[indGT];
        GLPointDouble3D VelSP = m_pInData->Vel;

        if (!pDataGT->ActPredictInfo.EstStartPoint.IsZero()
                && pDataGT->ActPredictInfo.TimeStartEst > con_eps) //if start point is estimated on active path
        {
            m_OutData.p_StartIsChanged = true;
            m_arProlData[indBall].tStart = pDataGT->ActPredictInfo.TimeStartEst;
            m_arProlData[indBall].StartPoint = pDataGT->ActPredictInfo.EstStartPoint;
        }
        else
        {
            //start point estimation using backward extrapolation to the Earth plane
            qreal dtRes=0; //resulting delta time "Estimated start time - current time"
            GLCalcTrajParam fCalc;
            GLComplexNumb T1c, T2c, T3c, T4c;
            //estrapolation using coordinates, velocities and accelerations
            fCalc.EstimateTimeEarthSurfaceIntersection(m_pInData->Coord, m_pInData->Vel, m_pInData->Acc, T1c, T2c, T3c, T4c);

            CompareTime(T1c, dtRes);
            CompareTime(T2c, dtRes);
            CompareTime(T3c, dtRes);
            CompareTime(T4c, dtRes);

            if (dtRes < - con_eps)
            {
                m_OutData.p_StartIsChanged = true;
                m_arProlData[indBall].tStart = m_pInData->tLoc + dtRes;
                m_arProlData[indBall].StartPoint = m_pInData->Coord + (m_pInData->Vel*dtRes) + (m_pInData->Acc*sqr(dtRes)/2.);
                VelSP = m_pInData->Vel + (m_pInData->Acc*dtRes);
            }
            else //intersection is not found
            {
                //extrapolation using coordinates and velocities
                fCalc.EstimateTimeEarthSurfaceIntersection(m_pInData->Coord, m_pInData->Vel, T1c, T2c);
                CompareTime(T1c, dtRes);
                CompareTime(T2c, dtRes);
                if (dtRes < - con_eps)
                {
                    m_OutData.p_StartIsChanged = true;
                    m_arProlData[indBall].tStart = m_pInData->tLoc + dtRes;
                    m_arProlData[indBall].StartPoint = m_pInData->Coord + (m_pInData->Vel*dtRes);
                    VelSP = m_pInData->Vel;
                }
            }
        }

        if (m_OutData.p_StartIsChanged)
        {
            TMatrix<DIM_CV> CovCV0, CovCVRes;
            TCMatrixFunc<DIM_CV> fMatr;
            fMatr.Copy(CovCV0, m_pInData->CovMatr, 0, DIM_CV-1, 0, DIM_CV-1);
            bool bOK = CovMatrixExtrapolation(m_pInData->tLoc, m_arProlData[indBall].tStart, CovCV0, CovCVRes);
            if (bOK)
            {
                qreal aI=0, bI=0, BetaI=0;
                GLEllipse FuncEllipse;
                FuncEllipse.SetMinSemiaxesRatio(c_MinSemiaxesRatio);
                bOK &= FuncEllipse.CalcEllipse(CovCVRes, m_arProlData[indBall].StartPoint, VelSP, aI, bI, BetaI, true);
                m_arProlData[indBall].StartEll.aI = c_MultEllSemiaxes * aI;
                m_arProlData[indBall].StartEll.bI = c_MultEllSemiaxes * bI;
                m_arProlData[indBall].StartEll.BetaI = BetaI;
                m_arProlData[indBall].StartPointFindMethod = AIR_BALL::START_POINT_BY_DOWN_PROGNOSIS;
                m_arProlData[indBall].tDetStartDownPr = m_pInData->tLoc;
            }

            if (!bOK)
            {
                m_OutData.p_StartIsChanged = false;
            }
        }
    }
}


void CBallisticProlongation::CompareTime(GLComplexNumb &dTcompl, qreal &dTreal)
{
    if (dTcompl.IsReal() && dTcompl.GetReal() < - con_eps)
    {
        if (dTcompl.GetReal() > dTreal || dTreal > - con_eps)
        {
            dTreal = dTcompl.GetReal();
        }
    }
}


void CBallisticProlongation::UpdateApogeeTime(qint32 ball_ind, qreal tApogeeNew)
{
    if (0 <= ball_ind && ball_ind < AIR_BALL::BALL_FORM_AMOUNT
            && fabs(tApogeeNew - AIR_BALL::Timer_INI) > con_eps)
    {
        qreal PrevTApogee = m_arProlData[ball_ind].tApogee;
        m_arProlData[ball_ind].tApogee = tApogeeNew;

        if (fabs(m_arProlData[ball_ind].min_tApogee - AIR_BALL::Timer_INI) < con_eps)
        {
            m_arProlData[ball_ind].min_tApogee = PrevTApogee;
        }
        else
        {
            if (m_arProlData[ball_ind].min_tApogee > PrevTApogee)
            {
                m_arProlData[ball_ind].min_tApogee = PrevTApogee;
            }
        }

        if (fabs(m_arProlData[ball_ind].min_tApogee - AIR_BALL::Timer_INI) < con_eps)
        {
            m_arProlData[ball_ind].min_tApogee = tApogeeNew;
        }
        else
        {
            if (m_arProlData[ball_ind].min_tApogee > tApogeeNew)
            {
                m_arProlData[ball_ind].min_tApogee = tApogeeNew;
            }
        }
    }
}


void CBallisticProlongation::CalcFallPoint(CBallProl_Output &ProlOut, const bool ForcedProlong)
{
    if (m_p_Arr_BallProc == nullptr)
    {
        return;
    }

    m_OutData.p_FallIsChanged = false;
    m_OutData.SignProgn = AIR_BALL::OLD_PROGNOSIS;
    const qint32 ind_ball = m_pInData->IF_BALL;
    const qint32 ind_GT = m_pInData->IF_GT;
    const qreal PrPeriod = GetProlPeriod();
    bool ItsOK = true;

    if (m_pInData->tLoc - m_arProlData[ind_ball].tSavedProl > PrPeriod
            || ForcedProlong)
    {
        m_OneProgn.Reset();
        const qreal Gamma = m_p_Arr_BallProc->arData_GT[ind_GT].Gamma;

        if (!m_pInData->bAeroballistic)
        {
            m_OneProgn.CalcTrack(m_pInData->tLoc, m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z,
                                 m_pInData->Vel.x, m_pInData->Vel.y, m_pInData->Vel.z, Gamma, false);
        }
        else //aeroballistic missile
        {
            const qreal t0_Aeroball = m_p_Arr_BallProc->arData_GT.at(ind_GT).t0_Aeroballistic;
            const qreal Delta = cLiftCoeff_default;
            bool bOK_Ab = m_OneProgn.CalcTrack_Aeroball(m_pInData->tLoc, t0_Aeroball,
                                                       m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z,
                                                       m_pInData->Vel.x, m_pInData->Vel.y, m_pInData->Vel.z, Gamma, Delta);
            if (!bOK_Ab)
            {
                m_OneProgn.CalcTrack(m_pInData->tLoc, m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z,
                                     m_pInData->Vel.x, m_pInData->Vel.y, m_pInData->Vel.z, Gamma, false);
            }
        }

        if (m_p_Arr_BallProc->arData_GT[ind_GT].Branch == AIR_BALL::ASCENDING_BRANCH
                && !m_pInData->bAeroballistic)
//                || m_OneProgn.GetHmax() > m_arProlData[ind_ball].Hapogee)
        {
            m_arProlData[ind_ball].Hapogee = m_OneProgn.GetHmax();
            qreal tApogeeNew = m_OneProgn.GetTApogee();
            UpdateApogeeTime(ind_ball, tApogeeNew);
        }        

        if (!m_OneProgn.GetSignCorrectProgn())
        {
            m_OutData.SignProgn = AIR_BALL::INCORRECT_PROGNOSIS;
            return;
        }

        //output last prolongated point
        GLPointDouble3D CoordFP(0.,0.,0.);
        GLPointDouble3D VelFP(0.,0.,0.);
        qint32 size = 0;
        qreal Tprol = 0.;
        ItsOK = m_OneProgn.OutputLastPoint(Tprol, size, CoordFP, VelFP);
        if (!ItsOK)
        {
            return;
        }

        std::vector<M10> ProlTrack;
        qint32 dimension = 0;
        m_OneProgn.GetProlongedTrack(ProlTrack, dimension);
        GlueProlongedTrack(ProlTrack); //gluing prolonged track

        ItsOK = m_OneProgn.GetPolinom(ProlTrack, m_arProlData[ind_ball].PolData); //calculate polinom
        if (ItsOK)
        {
            m_OutData.Polynoms = m_arProlData[ind_ball].PolData;
        }

        CalcOutPoints(m_pInData->IF_GT, m_pInData->IF_BALL); //calculate 50 points of trajectory

        //Calculation of dispersion ellipse in fall point
        qreal EllPrognPeriod; //period of ellipse recalculation
        if (!m_pInData->SignQuickReaction)
        {
            EllPrognPeriod = cCoefPeriodEll_Usual * PrPeriod;
        }
        else
        {
            EllPrognPeriod = cCoefPeriodEll_Urgent * PrPeriod;
        }

        if (m_pInData->tLoc - m_arProlData[ind_ball].tSavedEll > EllPrognPeriod)
        {
            EllipseInfo EllInfo;
            ItsOK = CalculateEllipse(m_p_Arr_BallProc->arData_GT[ind_GT].BMMark.MarkValue,
                                     m_p_Arr_BallProc->arData_GT[ind_GT].TrajType,
                                     m_pInData->tLoc, Tprol, m_pInData->CovMatr,
                                     m_pInData->Coord, m_pInData->Vel,
                                     CoordFP, VelFP, m_pInData->Sign_large_load,
                                     Gamma, false, EllInfo);
            if (ItsOK)
            {
                m_OutData.p_FallEllIsChanged = true;
                m_arProlData[ind_ball].FallEll = EllInfo;
                m_arProlData[ind_ball].tSavedEll = m_pInData->tLoc;                
            }
        }

        //fall point storage
        m_arProlData[ind_ball].tSavedProl = m_pInData->tLoc;
        m_arProlData[ind_ball].N_Prol ++;
        m_arProlData[ind_ball].FallPoint = CoordFP;
        m_arProlData[ind_ball].tFall = Tprol;
        m_arProlData[ind_ball].D = CalcDistance();
        m_OutData.p_FallIsChanged = true;
        if (m_OutData.bArrPointsFilled && m_OutData.SignProgn != AIR_BALL::INCORRECT_PROGNOSIS)
        {
            m_OutData.SignProgn = AIR_BALL::NEW_PROGNOSIS;
        }
        else    //incorrect 50 points
        {
            m_OutData.SignProgn = AIR_BALL::INCORRECT_PROGNOSIS;
        }

        m_arProlData[ind_ball].SignLastProgn = m_OutData.SignProgn;

        if((m_p_Arr_BallProc->arData_GT[ind_GT].Branch == AIR_BALL::DESCENDING_BRANCH))  //this conditions are written in order to have a not very big log file

        {
            const bool isFirst = m_OutData.Polynoms.LogPolynom();
            if (isFirst)
            {
                m_OneProgn.OutputTrack();
            }
        }
    }

    ProlOut = m_OutData;
}


bool CBallisticProlongation::CalcSepBoostPoint(qreal t_start, qreal dt_sep, GLPointDouble3D SP, GLPointDouble3D P_EAP,
                                               qreal HBoost, qreal LBoost, qreal VBoost, GLPoint_tXYZVTh &ResBoostSepP)
{
    bool bOK = true;
    ResBoostSepP.clear();

    if (fabs(t_start) > con_eps && dt_sep > con_eps
            && HBoost > con_eps && fabs(LBoost) > con_eps && VBoost > con_eps)
    {
        CGeocentric SP_gc(SP.x, SP.y, SP.z), P_EAP_gc(P_EAP.x, P_EAP.y, P_EAP.z), P_Sep_gc;
        CGeodesic SP_gd;
        CTopocentric P_EAP_tp, P_sep_tp;
        CTopographic P_EAP_topogr, P_Sep_topogr;

        bOK = GEOCENTRIC_GEO(&SP_gc, &SP_gd);
        bOK = bOK && GEOCENTRIC_TOPO(&SP_gd, &P_EAP_gc, &P_EAP_tp);
        bOK = bOK && RecountTOPOCENTRICtoTOPOGRAPHIC_coord(&P_EAP_tp, &P_EAP_topogr);

        const qreal L0 = sqrt(sqr(P_EAP_topogr.m_dXt) + sqr(P_EAP_topogr.m_dZt));

        if (bOK)
        {
            if (L0 > con_eps)
            {
                P_Sep_topogr.m_dXt = P_EAP_topogr.m_dXt * LBoost/L0;
                P_Sep_topogr.m_dZt = P_EAP_topogr.m_dZt * LBoost/L0;
                P_Sep_topogr.m_dHt = HBoost;

                bOK = RecountTOPOGRAPHICtoTOPOCENTRIC_coord(&P_Sep_topogr, &P_sep_tp);
                bOK = bOK && TOPO_GEOCENTRIC(&SP_gd, &P_sep_tp, &P_Sep_gc);

                qreal t_sep = t_start + dt_sep;

                ResBoostSepP.time = SEC70_to_MSEC70(t_sep);
                ResBoostSepP.coord.init(P_Sep_gc.m_dX, P_Sep_gc.m_dY, P_Sep_gc.m_dZ);
                ResBoostSepP.V = VBoost;
            }
            else
            {
                bOK = false;
            }
        }
    }
    else
    {
        bOK = false;
    }

    return bOK;
}


void CBallisticProlongation::CalcPolynomSatellite(CBallProl_Output &OutData)
{
    OutData.Reset();
    m_OneProgn.Reset();
    qreal tProgn = m_pInData->tLoc + c_TimeProlSatellite;
    bool bOK = m_OneProgn.CalcTrack_OnTime(m_pInData->tLoc, tProgn, m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z,
                                m_pInData->Vel.x, m_pInData->Vel.y, m_pInData->Vel.z, c_Gamma_st, false);
    if (bOK && m_OneProgn.GetSignCorrectProgn())
    {
        std::vector<M10> ProlTrack;
        qint32 dimension = 0;
        m_OneProgn.GetProlongedTrack(ProlTrack, dimension);
        if (dimension > 1)
        {
            bOK = m_OneProgn.GetPolinom(ProlTrack, OutData.Polynoms);
            if (bOK)
            {
                OutData.SignProgn = AIR_BALL::NEW_PROGNOSIS;
            }
        }
    }
}


bool CBallisticProlongation::CheckConditionsForStartPointDownPr(bool &bCounterExceeded)
{
    bool bRes = false;
    bCounterExceeded = false;

    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;

    if (0 <= ind_GT && ind_GT < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        if (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::UNDEFINED_START_POINT)
        {
            if (fabs(m_pInData->tLoc - m_arProlData[ind_ball].tPrevTryDetStartDownPr) > con_eps)
            {
                if (m_p_Arr_BallProc->arData_GT[ind_GT].Branch == AIR_BALL::ASCENDING_BRANCH
                        && m_arProlData[ind_ball].Hapogee > con_eps)
                {
                    qreal HCmp = std::min(cHeightCtrlStartCalc,
                                          cCoefHApogeeCtrlStartCalc * m_arProlData[ind_ball].Hapogee);
                    if (m_pInData->H < HCmp)
                    {
                        bRes = true;
                        m_arProlData[ind_ball].CounterTryDetStartPointDownPr = 0;
                    }
                }

                if (!bRes)
                {
                    if (m_arProlData[ind_ball].CounterTryDetStartPointDownPr <= cCounterCtrlStartCalc)
                    {
                        bRes = true;
                    }
                    else
                    {
                        bCounterExceeded = true;
                    }
                    m_arProlData[ind_ball].CounterTryDetStartPointDownPr ++;
                }
            }
        }
    }

    return bRes;
}


void CBallisticProlongation::GetStartPoint(const qint32 ind_ball, GLPointDouble3D &StartPoint, qreal &StartTime)
{    
    StartPoint.clear();
    StartTime = 0.;
    if (ind_ball >= 0)
    {
        StartPoint = m_arProlData[ind_ball].StartPoint;
        StartTime = m_arProlData[ind_ball].tStart;
    }
}


void CBallisticProlongation::GetStartPoint(GLPointDouble3D &StartPoint, qreal &StartTime)
{
    GetStartPoint(m_pInData->IF_BALL, StartPoint, StartTime);
}


void CBallisticProlongation::GetFallPoint(const qint32 ind_ball, GLPointDouble3D &FallPoint, qreal &FallTime)
{

    if (ind_ball >= 0)
    {
        FallPoint = m_arProlData[ind_ball].FallPoint;
        FallTime = m_arProlData[ind_ball].tFall;
    }
}


void CBallisticProlongation::GetFallPoint(GLPointDouble3D &FallPoint, qreal &FallTime)
{
    GetFallPoint(m_pInData->IF_BALL, FallPoint, FallTime);
}


bool CBallisticProlongation::CalculateEllipse(const qint16 BMMarkValue, const qint16 TrajType, const qreal CurrTime, const qreal EndTime, GLMatrix &CovMatr, const GLPointDouble3D &CurrPoint, const GLPointDouble3D &CurrVel, const GLPointDouble3D &EndPoint, const GLPointDouble3D &EndVel, const bool SignLargeLoad, const qreal Gamma, const bool SignInvProgn, EllipseInfo &EllInfo, TMatrix<DIM_CV> &CovCVRes)
{
    TMatrix<DIM_CV> CovCV0;
    CovCVRes.Reset();
    TCMatrixFunc<DIM_CV> fMatr;
    fMatr.Copy(CovCV0, CovMatr, 0, DIM_CV-1, 0, DIM_CV-1);
    bool bOK = true;

    if (SignLargeLoad) //large load
    {
        qreal DeltaCoord = 0., DeltaVel = 0.;
        GetDeltaFinDiff(BMMarkValue, TrajType, DeltaCoord, DeltaVel); //Calculation of DeltaCoord, DeltaVel for method of finite differences

        //calculate covariance matrix in end point
        bOK = CovMatrixProlongation_FinDiff(CurrTime, EndTime, DeltaCoord, DeltaVel,
                                      CurrPoint, CurrVel, EndPoint, EndVel,
                                      CovCV0, Gamma, SignInvProgn, CovCVRes);
        if (!bOK)
        {
            bOK = CovMatrixExtrapolation(CurrTime, EndTime, CovCV0, CovCVRes);
            if (!bOK)
            {
                return false;
            }
        }
    }
    else //small load
    {
        bOK = CovMatrixProlongation_Unscented(CurrTime, EndTime,
                                      CurrPoint, CurrVel, EndPoint, EndVel,
                                      CovCV0, Gamma, SignInvProgn, CovCVRes);
        if (!bOK)
        {
            qreal DeltaCoord = 0., DeltaVel = 0.;
            GetDeltaFinDiff(BMMarkValue, TrajType, DeltaCoord, DeltaVel); //Calculation of DeltaCoord, DeltaVel for method of finite differences

            //calculate covariance matrix in end point
            bOK = CovMatrixProlongation_FinDiff(CurrTime, EndTime, DeltaCoord, DeltaVel,
                                          CurrPoint, CurrVel, EndPoint, EndVel,
                                          CovCV0, Gamma, SignInvProgn, CovCVRes);
            if (!bOK)
            {
                bOK = CovMatrixExtrapolation(CurrTime, EndTime, CovCV0, CovCVRes);
                if (!bOK)
                {
                    return false;
                }
            }
        }
    }

    TMatrix<DIM_CV> CovCVCmp; //covariance matrix for comparison
    bOK = CovMatrixExtrapolation(CurrTime, EndTime, CovCV0, CovCVCmp);
    if (!bOK)
    {
        return false;
    }

    qreal MaxDispRes = CovCVRes.GetMaxDiag3(); //maximum coordinate dispersion for CovCVRes
    qreal MaxDispCmp = CovCVCmp.GetMaxDiag3(); //maximum coordinate dispersion for CovCVCmp
    if (MaxDispRes > MaxDispCmp + con_eps && MaxDispCmp > con_eps)
    {
        CovCVRes = CovCVCmp;
    }

    //calculation dispersion ellipse
    qreal aI = 0.;
    qreal bI = 0.;
    qreal BetaI = 0.;
    GLEllipse FuncEllipse;
    FuncEllipse.SetMinSemiaxesRatio(c_MinSemiaxesRatio);
    bOK &= FuncEllipse.CalcEllipse(CovCVRes, EndPoint, EndVel, aI, bI, BetaI, true);
    EllInfo.aI = c_MultEllSemiaxes * aI;
    EllInfo.bI = c_MultEllSemiaxes * bI;
    EllInfo.BetaI = BetaI;

    return bOK;
}


bool CBallisticProlongation::CalculateEllipse(const qint16 BMMarkValue, const qint16 TrajType, const qreal CurrTime, const qreal EndTime, TMatrix<SIZE_M> &CovMatr, const GLPointDouble3D &CurrPoint, const GLPointDouble3D &CurrVel, const GLPointDouble3D &EndPoint, const GLPointDouble3D &EndVel, const bool SignLargeLoad, const qreal Gamma, const bool SignInvProgn, EllipseInfo &EllInfo)
{
    TMatrix<DIM_CV> CovCVRes;
    bool bRes = CalculateEllipse(BMMarkValue, TrajType, CurrTime, EndTime, CovMatr, CurrPoint, CurrVel, EndPoint, EndVel, SignLargeLoad, Gamma, SignInvProgn, EllInfo, CovCVRes);
    return bRes;
}


bool CBallisticProlongation::CalculateTrapezeAB()
{
    const qint32 ind_Ball = m_pInData->IF_BALL;

//    m_pInData->Coord; //current point in ECEF
//    m_pInData->AB_MaxDistance;
//    m_arProlData[ind_Ball].FallPoint; //fall point in ECEF
//    m_arProlData[ind_Ball].FallEll; //ellipse parameters, TPCS in fall point

//    m_OutData.AB_Trapeze;

    //add Evgenia
    bool ok;
    CGeocentric currpointECEF(m_pInData->Coord.x,
                              m_pInData->Coord.y,
                              m_pInData->Coord.z); //current point in ECEF
    CGeocentric fallpointECEF(m_arProlData[ind_Ball].FallPoint.x,
                              m_arProlData[ind_Ball].FallPoint.y,
                              m_arProlData[ind_Ball].FallPoint.z);//fall point in ECEF
    CGeodesic fallpointGDCS; //fall point in GDCS
    ok = GEOCENTRIC_GEO(&fallpointECEF, &fallpointGDCS);

    CTopocentric currpointTPCS; //current point in fall point TPCS
    ok = GEOCENTRIC_TOPO(&fallpointGDCS, &currpointECEF, &currpointTPCS);
    currpointTPCS.m_dYt = 0; //project current point on XZ plane

    CGeocentric currpointprojECEF;//current point projection back to ECEF coordinates
    ok = TOPO_GEOCENTRIC(&fallpointGDCS, &currpointTPCS, &currpointprojECEF);

    const qreal &a = m_arProlData[ind_Ball].FallEll.aI;
    const qreal &b = m_arProlData[ind_Ball].FallEll.bI;
    const qreal &beta = m_arProlData[ind_Ball].FallEll.BetaI;
    const qreal &Pz = currpointTPCS.m_dZt;
    const qreal &Px = currpointTPCS.m_dXt;
    CEllipsePoint el(b,a,beta,Pz,Px);

    qreal r1 = 0;
    qreal r2 = 0;
    qreal alpha = 0;
    qreal azflu = 0;
    el.calcR(r1,r2);
    el.calcAngles(alpha,azflu);

    m_OutData.AB_Trapeze.MinRange = r1;
    m_OutData.AB_Trapeze.MaxRange = std::max(r2,m_pInData->AB_MaxDistance);
    m_OutData.AB_Trapeze.P_base.x = currpointprojECEF.m_dX;
    m_OutData.AB_Trapeze.P_base.y = currpointprojECEF.m_dY;
    m_OutData.AB_Trapeze.P_base.z = currpointprojECEF.m_dZ;
    m_OutData.AB_Trapeze.AzFly = azflu * con_180_div_PI; //radians --> degrees
    m_OutData.AB_Trapeze.SStrAzFly = alpha * con_180_div_PI; //radians --> degrees
    m_OutData.AB_Trapeze.SigmaAzFly = (alpha/3.) * con_180_div_PI; //radians --> degrees

    return ok;
}


void CBallisticProlongation::CorrectionStartPoint()
{
    const qint32 ind_GT = m_pInData->IF_GT;
    const qint32 ind_ball = m_pInData->IF_BALL;

    CAirBall_Proc_InnerData_GT *pGTData = &m_p_Arr_BallProc->arData_GT[ind_GT];

    GLCalcTrajParam fCalcP;
    GLPointDouble3D FirstPoint = pGTData->FirstPoint;
    GLPointDouble3D FirstP_H0; //Projection of 1st point to the Earth surface
    const GLPointDouble3D Sigma1stP = pGTData->RMSEs_FirstPoint;

    GLPointDouble3D CalculatedSP = m_arProlData[ind_ball].StartPoint;
    GLPointDouble3D *pResSP = &m_arProlData[ind_ball].StartPoint;

    const bool SignFstP = !FirstPoint.IsZero();  //sign of presence of 1st point
    bool bOK = true;

    // 1. Criterion on height of 1st point
    const qreal H1st = fCalcP.CalcHeight(FirstPoint.x, FirstPoint.y, FirstPoint.z); //height of 1st point

    if (H1st <= cHeight4StartCheck && SignFstP) //BM was detected close to the launch
    {
        //put 1st point to the Earth surface
        CGeocentric FstP_ECEF(FirstPoint.x, FirstPoint.y, FirstPoint.z); //first point in ECEF
        CGeodesic FstP_GEOD; //First point in Geodesic
        bOK = GEOCENTRIC_GEO(&FstP_ECEF, &FstP_GEOD);
        FstP_GEOD.m_dAltitude = 0;
        bOK = bOK && GEO_GEOCENTRIC(&FstP_GEOD, &FstP_ECEF);
        FirstP_H0.x = FstP_ECEF.m_dX;
        FirstP_H0.y = FstP_ECEF.m_dY;
        FirstP_H0.z = FstP_ECEF.m_dZ;

        if (H1st <= cHeight4Start && bOK) //BM was detected very close to the launch
        {
            bool bNeedCorrection = false;

            const GLPointDouble3D DeltaSP_1P = m_arProlData[ind_ball].StartPoint - FirstP_H0;
            if (   fabs(DeltaSP_1P.x) > AIR_BALL::K_Sigma * Sigma1stP.x
                || fabs(DeltaSP_1P.y) > AIR_BALL::K_Sigma * Sigma1stP.y
                || fabs(DeltaSP_1P.z) > AIR_BALL::K_Sigma * Sigma1stP.z   )
            {
                bNeedCorrection = true;
            }
            else
            {   //check whether the first point belongs to the ellipse in the start point
                CGeocentric StartP_gc(m_arProlData[ind_ball].StartPoint.x, m_arProlData[ind_ball].StartPoint.y, m_arProlData[ind_ball].StartPoint.z);
                CGeodesic StartP_gd;
                CTopocentric FstP_tp;

                bOK = bOK && GEOCENTRIC_GEO(&StartP_gc, &StartP_gd);
                bOK = bOK && GEOCENTRIC_TOPO(&StartP_gd, &FstP_ECEF, &FstP_tp);

                GLEllipse2D EllStart(m_arProlData[ind_ball].StartEll.bI,
                                     m_arProlData[ind_ball].StartEll.aI,
                                     m_arProlData[ind_ball].StartEll.BetaI, 0, 0);
                if (bOK && !EllStart.IsPointInside(FstP_tp.m_dZt, FstP_tp.m_dXt))
                {
                    bNeedCorrection = true;
                }
            }

            if (bNeedCorrection)
            {
                //i.e. prolongated Start point is far from the 1st point (not in the error ellipsoid of 1st point)
                m_arProlData[ind_ball].StartPoint = FirstP_H0;
                m_arProlData[ind_ball].tStart = pGTData->tFirst;

                qreal D_SPcalcSPcorr = pResSP->getDistanceOnSurface(CalculatedSP);
                tLog(m_pLogCorrSP, "===== Start point correction (H < 3000 m) =====");
                tLog(m_pLogCorrSP, "NumGT: %d IndGT: %d tLoc: %f H0: %f",
                     pGTData->NumbGT, ind_GT, m_pInData->tLoc, H1st);
                tLog(m_pLogCorrSP, "Calculated SP: %f %f %f Corrected SP: %f %f %f Distance: %f",
                     CalculatedSP.x, CalculatedSP.y, CalculatedSP.z,
                     pResSP->x, pResSP->y, pResSP->z, D_SPcalcSPcorr);
            }
            m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_BY_1ST_POINT;
        }
        else //i.e. cHeight4Start < H1st <= cHeight4StartCheck,,
        {
            CGeocentric CurrP_ECEF(m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z),
                        SP_ECEF(m_arProlData[ind_ball].StartPoint.x,
                                m_arProlData[ind_ball].StartPoint.y,
                                m_arProlData[ind_ball].StartPoint.z) ;
            CGeodesic CurrP_GEOD;
            CGeodesic SP_GEOD;
            bOK = bOK && GEOCENTRIC_GEO(&CurrP_ECEF, &CurrP_GEOD);
            bOK = bOK && GEOCENTRIC_GEO(&SP_ECEF, &SP_GEOD);
            const qreal L_C_1 = CurrP_GEOD.GetGeodesicLine(CurrP_GEOD, FstP_GEOD); //geodesic line from current point to the 1st point
            const qreal L_C_S = CurrP_GEOD.GetGeodesicLine(CurrP_GEOD, SP_GEOD); //geodesic line from current point to the prolongated start point

            if (L_C_S < L_C_1)
            {
                m_arProlData[ind_ball].StartPoint = FirstP_H0;
                m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_BY_1ST_POINT;

                qreal D_SPcalcSPcorr = pResSP->getDistanceOnSurface(CalculatedSP);
                tLog(m_pLogCorrSP, "===== Start point correction (H < 10000 m) =====");
                tLog(m_pLogCorrSP, "NumGT: %d IndGT: %d tLoc: %f H0: %f",
                     pGTData->NumbGT, ind_GT, m_pInData->tLoc, H1st);
                tLog(m_pLogCorrSP, "Calculated SP: %f %f %f Corrected SP: %f %f %f Distance: %f",
                     CalculatedSP.x, CalculatedSP.y, CalculatedSP.z,
                     pResSP->x, pResSP->y, pResSP->z, D_SPcalcSPcorr);
            }
        }

        if (m_arProlData[ind_ball].tStart > pGTData->tFirst)
        {
            m_arProlData[ind_ball].tStart = pGTData->tFirst;
        }
    }

    if (H1st > cHeight4Start || !SignFstP)
    {
        GLPointDouble3D SP_Act = pGTData->ActPredictInfo.EstStartPoint; //m_p_Arr_BallProc->Form[ind_GT].SP_Active;
        const qreal tStart_act = pGTData->ActPredictInfo.TimeStartEst;

        if (tStart_act > con_eps && !SP_Act.IsZero())
        {
            CGeocentric CurrP_ECEF(m_pInData->Coord.x, m_pInData->Coord.y, m_pInData->Coord.z),
                        SP_ECEF(m_arProlData[ind_ball].StartPoint.x,
                                m_arProlData[ind_ball].StartPoint.y,
                                m_arProlData[ind_ball].StartPoint.z),
                        SP_Act_ECEF(SP_Act.x, SP_Act.y, SP_Act.z);
            CGeodesic CurrP_GEOD;
            CGeodesic SP_GEOD;
            CGeodesic SP_Act_GEOD;
            bOK = bOK && GEOCENTRIC_GEO(&CurrP_ECEF, &CurrP_GEOD);
            bOK = bOK && GEOCENTRIC_GEO(&SP_ECEF, &SP_GEOD);
            bOK = bOK && GEOCENTRIC_GEO(&SP_Act_ECEF, &SP_Act_GEOD);

            const qreal L_C_S = CurrP_GEOD.GetGeodesicLine(CurrP_GEOD, SP_GEOD); //geodesic line from current point to the prolongated start point
            const qreal L_C_SAct = CurrP_GEOD.GetGeodesicLine(CurrP_GEOD, SP_Act_GEOD); //geodesic line from current point to the estimated on active path start point

            if (L_C_S > L_C_SAct && bOK)
            {
                m_arProlData[ind_ball].StartPoint = SP_Act;
                m_arProlData[ind_ball].tStart = tStart_act;
                m_arProlData[ind_ball].StartPointFindMethod = AIR_BALL::START_POINT_BY_1ST_POINT;

                qreal D_SPcalcSPcorr = pResSP->getDistanceOnSurface(CalculatedSP);
                tLog(m_pLogCorrSP, "===== Start point correction (using start point estimated on active path) =====");
                tLog(m_pLogCorrSP, "NumGT: %d IndGT: %d tLoc: %f H0: %f",
                     pGTData->NumbGT, ind_GT, m_pInData->tLoc, H1st);
                tLog(m_pLogCorrSP, "Calculated SP: %f %f %f Corrected SP: %f %f %f Distance: %f",
                     CalculatedSP.x, CalculatedSP.y, CalculatedSP.z,
                     pResSP->x, pResSP->y, pResSP->z, D_SPcalcSPcorr);
            }
        }
    }

    //start point correction in the case of horizontal maneuver
    if (m_arProlData[ind_ball].StartPointFindMethod != AIR_BALL::START_POINT_BY_1ST_POINT
            && SignFstP
            && fabs(pGTData->tFirst - AIR_BALL::Timer_INI) > con_eps
            && fabs(pGTData->ManTime - AIR_BALL::Timer_INI) > con_eps
            && !pGTData->ManPoint.IsZero()
            && m_pInData->tLoc > pGTData->ManTime
            && pGTData->ManTime > pGTData->tFirst)
    {
        GLPointDouble3D StartPoint0 = m_arProlData[ind_ball].StartPoint;
        qreal LStartMan = StartPoint0.getDistanceOnSurface(pGTData->ManPoint); //distance from the start point to the maneuver point

        CGeocentric ManPoint_gc(pGTData->ManPoint.x, pGTData->ManPoint.y, pGTData->ManPoint.z);
        CGeocentric StartPoint_gc_new;
        CGeocentric FirstPoint_gc(pGTData->FirstPoint.x, pGTData->FirstPoint.y, pGTData->FirstPoint.z);
        CGeodesic ManPoint_gd;
        CTopocentric StartPoint_tp_new; //in the TPCS with the center in the maneuver point
        CTopocentric FirstPoint_tp; //in the TPCS with the center in the manauver point
        CTopographic StartPoint_topogr_new;
        CTopographic FirstPoint_topogr;

        bOK = GEOCENTRIC_GEO(&ManPoint_gc, &ManPoint_gd);
        ManPoint_gd.m_dAltitude = 0;

        bOK = bOK && GEOCENTRIC_TOPO(&ManPoint_gd, &FirstPoint_gc, &FirstPoint_tp);

        bOK = bOK && RecountTOPOCENTRICtoTOPOGRAPHIC_coord(&FirstPoint_tp, &FirstPoint_topogr);

        qreal D0 = sqrt(sqr(FirstPoint_topogr.m_dZt) + sqr(FirstPoint_topogr.m_dXt));
        if (bOK && LStartMan > con_eps && D0 > con_eps)
        {
            StartPoint_topogr_new.m_dXt = FirstPoint_topogr.m_dXt * LStartMan / D0;
            StartPoint_topogr_new.m_dZt = FirstPoint_topogr.m_dZt * LStartMan / D0;
            StartPoint_topogr_new.m_dHt = 0;

            bOK = RecountTOPOGRAPHICtoTOPOCENTRIC_coord(&StartPoint_topogr_new, &StartPoint_tp_new);

            bOK = bOK && TOPO_GEOCENTRIC(&ManPoint_gd, &StartPoint_tp_new, &StartPoint_gc_new);

            if (bOK)
            {
                m_arProlData[ind_ball].StartPoint.init(StartPoint_gc_new.m_dX, StartPoint_gc_new.m_dY, StartPoint_gc_new.m_dZ);
            }
        }
    }
}


bool CBallisticProlongation::CheckStartPoint(const qreal &tStart, GLPointDouble3D &SP, TMatrix<DIM_CV> &CovSP)
{
    bool bCorrectSP = true;
    if (SP.IsZero())
    {
        bCorrectSP = false;
    }
    else
    {
        qint32 indGT = m_pInData->IF_GT;
        qint32 indBall = m_pInData->IF_BALL;
        if (0 <= indGT && indGT < AIR_BALL::GT_FORM_AMOUNT
                && 0 <= indBall && indBall < AIR_BALL::BALL_FORM_AMOUNT)
        {
            CAirBall_Proc_InnerData_GT *pGTData = &m_p_Arr_BallProc->arData_GT[indGT];
            GLPointDouble3D FstP = pGTData->FirstPoint;
            GLPointDouble3D CurrP = m_pInData->Coord;
            if (!FstP.IsZero() && !CurrP.IsZero())
            {
                qreal LCurrFstP = CurrP.getDistanceOnSurface(FstP);
                qreal LCurrSP = CurrP.getDistanceOnSurface(SP);
                if (LCurrFstP > LCurrSP || tStart - pGTData->tFirst > c_dTStart_CheckSP)
                {
                    qreal DFstPSP = FstP.getDistance(SP);
                    qreal SigMaxSP = sqrt(CovSP.GetMaxDiag3());
                    qreal SigMaxFstP = pGTData->RMSEs_FirstPoint.getMax();
                    if (DFstPSP > AIR_BALL::K_Sigma * (SigMaxSP + SigMaxFstP))
                    {
                        bCorrectSP = false;
                    }
                }
            }
        }
    }
    return bCorrectSP;
}


void CBallisticProlongation::SetSignBMMarkChanged(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_arProlData[ind_Ball].bBMMarkChanged = true;
        m_arProlData[ind_Ball].bStartByTablesPerformed_multival = false;
    }
}


void CBallisticProlongation::GetOutPoints(qint32 GTInd, qint32 ind_ball, CBallProl_Output &pOutput)
{
    CalcOutPoints(GTInd, ind_ball);
    pOutput = m_OutData;
}


void CBallisticProlongation::Drop_BallTrack(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        ClearInnerData_1Track(ind_Ball);
    }
}


void CBallisticProlongation::FillInputData(CBallProl_Input &InData)
{
    m_pInData = &InData;
}


void CBallisticProlongation::SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables, Str_Ar_BallCoef_Param_AllTables *p_arBallCoef_Tables)
{
    m_pEAP_Tables = p_arEAP_Tables;
    m_pBallCoef_Tables = p_arBallCoef_Tables;
}


void CBallisticProlongation::SetPersistentData(Str_Ar_EAP_Param_AllTables *p_arEAP_Tables)
{
    m_pEAP_Tables = p_arEAP_Tables;
}


void CBallisticProlongation::InitConst(const Pr_Const &Const)
{
    m_PrognConst = Const;
    m_OneProgn.InitConst(Const.OnePrConsts);
}


void CBallisticProlongation::InitPointerToBallProcArray(CAirBall_Proc_Inner_Arr *Arr)
{
    m_p_Arr_BallProc = Arr;
}


void CBallisticProlongation::GlueProlongedTrack(const  std::vector<M10> &NewProlTrack)
{
    const qint32 ind_ball = m_pInData->IF_BALL;
    if (ind_ball < 0)
    {
        m_OutData.SignProgn = AIR_BALL::INCORRECT_PROGNOSIS;
        return;
    }

    const qint32 SizeOldProl = m_arProlData[ind_ball].ProlongedTrack.size();
    if (SizeOldProl > 1)
    {        
        //find an index at which old prognosis and new one should be sticked together
        qint32 Ai = -1, j;
        qint32 j_corr = -1; //index of 1st element for correction
        qreal t_corr = m_pInData->tLoc - m_PrognConst.dT_corr_progn; //time at which correction begins

        for (j=0; j<SizeOldProl-1; j++)
        {
            if (    m_arProlData[ind_ball].ProlongedTrack[j].M_info[0] - t_corr < con_par_eps
                 && m_arProlData[ind_ball].ProlongedTrack[j+1].M_info[0] - t_corr > -con_par_eps)
            {       //t[j]-epsilon < t_corr < t[j+1]+epsilon
                j_corr = j;
            }

            if (    m_arProlData[ind_ball].ProlongedTrack[j].M_info[0] - m_pInData->tLoc < con_par_eps
                 && m_arProlData[ind_ball].ProlongedTrack[j+1].M_info[0] - m_pInData->tLoc > -con_par_eps)
            {       //t[j]-epsilon < tLoc < t[j+1]+epsilon
                Ai = j;
                break;
            }
        }
        if (Ai == -1)
        {
            if (m_pInData->tLoc > m_arProlData[ind_ball].ProlongedTrack[SizeOldProl-1].M_info[0])
            {
                Ai = SizeOldProl - 1;
            }
            else
            {
                Ai = 0;
            }
        }

        m_arProlData[ind_ball].ProlongedTrack.resize(Ai+1); //clear part of vector with time > tLoc

        qint32 i=0;
        m_arProlData[ind_ball].ProlongedTrack.push_back(NewProlTrack[0]);
        qint32 SizeNew = static_cast<qint32>(NewProlTrack.size());
        for (j=1; j < SizeNew; j++)
        {
            if (NewProlTrack[j].M_info[0] - m_arProlData[ind_ball].ProlongedTrack[Ai+i].M_info[0]
                    > c_dT_storage - c_Epsilon_dTst)
            {
                i++;
                m_arProlData[ind_ball].ProlongedTrack.push_back(NewProlTrack[j]);
            }
        }

        //correction of prognosis
        if (Ai > 0)
        {
            if (j_corr < 0)
            {
                j_corr = 0;
            }
            t_corr = m_arProlData[ind_ball].ProlongedTrack[j_corr].M_info[0];

            qint32 j_dt0 = Ai;
            if (m_pInData->tLoc - m_arProlData[ind_ball].ProlongedTrack[Ai].M_info[0] < con_par_eps)
            {
                j_dt0 = Ai-1;
            }
            const qreal dT0 = m_pInData->tLoc - m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[0];
            const qreal dT = m_pInData->tLoc - t_corr;

            const qreal Xextr = m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[1]
                        + dT0 * m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[4]
                        + dT0 * dT0 * m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[7] / 2;
            const qreal Yextr = m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[2]
                        + dT0 * m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[5]
                        + dT0 * dT0 * m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[8] / 2;
            const qreal Zextr = m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[3]
                        + dT0 * m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[6]
                        + dT0 * dT0 * m_arProlData[ind_ball].ProlongedTrack[j_dt0].M_info[9] / 2;

            const qreal dX_t = (m_pInData->Coord.x - Xextr) / dT;
            const qreal dY_t = (m_pInData->Coord.y - Yextr) / dT;
            const qreal dZ_t = (m_pInData->Coord.z - Zextr) / dT;

            for (j=j_corr+1; j<=Ai; j++)
            {
                const qreal dTj = m_arProlData[ind_ball].ProlongedTrack[j].M_info[0] - t_corr;
                m_arProlData[ind_ball].ProlongedTrack[j].M_info[1] += dTj * dX_t;
                m_arProlData[ind_ball].ProlongedTrack[j].M_info[2] += dTj * dY_t;
                m_arProlData[ind_ball].ProlongedTrack[j].M_info[3] += dTj * dZ_t;
            }
        }
    }
    else
    {
//        m_OutData.SignProgn = AIR_BALL::INCORRECT_PROGNOSIS;
//        return;
        if (m_arProlData[ind_ball].StartPointFindMethod == AIR_BALL::START_POINT_ABSENCE)
        {
            qint32 i=0, j=0;
            m_arProlData[ind_ball].ProlongedTrack.push_back(NewProlTrack[0]);
            qint32 SizeNew = static_cast<qint32>(NewProlTrack.size());
            for (j=1; j < SizeNew; j++)
            {
                if (NewProlTrack[j].M_info[0] - m_arProlData[ind_ball].ProlongedTrack[i].M_info[0]
                        > c_dT_storage - c_Epsilon_dTst)
                {
                    i++;
                    m_arProlData[ind_ball].ProlongedTrack.push_back(NewProlTrack[j]);
                }
            }
        }
    }
}


void CBallisticProlongation::GetDeltaFinDiff(const qint16 BMMark, const qint16 TrajType, qreal &DeltaCoord, qreal &DeltaVel)
{
    if ( (BMMark >= AIR_BALL::BM60) && (BMMark <= AIR_BALL::BM12000))
    {
        const qint16 row_num = BMMark - AIR_BALL::BM60;
        qint16 col_num = 0;
        if (TrajType == AIR_BALL::LOFTED)
        {
            col_num = 2;
        }
        else if (TrajType == AIR_BALL::FLAT)
        {
            col_num = 4;
        }
        else
        {
        }

        DeltaCoord = Delta_FinDiff[row_num][col_num];
        DeltaVel = Delta_FinDiff[row_num][col_num+1];
    }
    else
    {
        DeltaCoord = DeltaCoord_defalut;
        DeltaVel = DeltaVel_default;
    }

    if (fabs(DeltaCoord < con_eps))
    {
        DeltaCoord = DeltaCoord_defalut;
    }

    if (fabs(DeltaVel) < con_eps)
    {
        DeltaVel = DeltaVel_default;
    }
}


qreal CBallisticProlongation::GetProlPeriod()
{
    qreal PrPeriod = 0.;

    if (( m_pInData->ActionPhase == KILL_ASSESSMENT
          || m_pInData->ActionPhase == ASSESSMENT_RESULT
          || m_pInData->ActionPhase == MEET_POINT
          || m_pInData->ActionPhase == DESTROYED
          || m_pInData->ActionPhase == MISSED)
            && fabs(m_pInData->tLoc - m_pInData->tObtainActionPhase) < c_ControlTimeNearMeet)
    {
        PrPeriod = c_PrognPeriod_NearMeet;
    }
    else
    {
        if (m_pInData->BallType == SUBCLASS_WARHEAD /*|| m_pInData->BallType == TYPE_MaCRV*/) //BO is warhead
        {
            if (m_pInData->bAeroballistic) //aeroballistic missile
            {
                if (m_pInData->Sign_large_load) //large load
                {
                    PrPeriod = c_PrognPeriod_Aeroball_HI_LOAD;
                }
                else //small load
                {
                    PrPeriod = c_PrognPeriod_Aeroball;
                }
            }
            else if (!m_pInData->SignQuickReaction) //sufficient margin of time
            {
                if (m_pInData->Sign_large_load) //large load
                {
                    PrPeriod = c_PrognPeriod_WH_Usual_HI_LOAD;
                }
                else //small load
                {
                    PrPeriod = c_PrognPeriod_WH_Usual;
                }
            }
            else //small margin of time
            {
                if (m_pInData->Sign_large_load) //large load
                {
                    PrPeriod = c_PrognPeriod_WH_Urgent_HI_LOAD;
                }
                else //small load
                {
                    PrPeriod = c_PrognPeriod_WH_Urgent;
                }
            }
        }
        else //not warhead
        {
            if (m_pInData->Sign_large_load) //large load
            {
                PrPeriod = c_PrognPeriod_Boost_HI_LOAD;
            }
            else //small load
            {
                PrPeriod = c_PrognPeriod_Boost;
            }
        }
    }

    return PrPeriod;
}


qreal CBallisticProlongation::CalcKGamma(qreal H)
{
    qreal KGamma = cKGammaMin;
    if (H > cHeightKGamma1)
    {
        if (H > cHeightKGamma2)
        {
            KGamma = m_PrognConst.K_Gamma;
        }
        else // cHeightKGamma1 < H <= cHeightKGamma2
        {
            KGamma = cKGammaMin +
                      (m_PrognConst.K_Gamma - cKGammaMin) *
                         ((H-cHeightKGamma1) / (cHeightKGamma2 - cHeightKGamma1));
        }
    }
    return KGamma;
}

void CBallisticProlongation::CorrInverseProlTrack_old(const M10 &SP, const std::vector<M10> &TrackFragment)
{
    const qint32 ind_ball = m_pInData->IF_BALL; //index of ballistic trajectory
    qint32 i;

    const std::vector<M10> OldTrack = m_arProlData[ind_ball].ProlongedTrack;    //old prolonged track
    const qint32 SizeOldTrack = OldTrack.size();
    std::vector<M10>().swap(m_arProlData[ind_ball].ProlongedTrack); //cleaning of m_arProlData[ind_ball].ProlongedTrack

    m_arProlData[ind_ball].ProlongedTrack.push_back(SP);
    qreal tGlue = SP.M_info[0];

    qint32 SizeFragm = TrackFragment.size();
    if (SizeFragm > 0)
    {
        for (i=0; i<SizeFragm; i++)
        {
            const M10 *pCurEl = &TrackFragment[i];
            if (pCurEl->M_info[0] > tGlue)
            {
                m_arProlData[ind_ball].ProlongedTrack.push_back(*pCurEl);
                tGlue = pCurEl->M_info[0];
            }
        }
    }

    if (SizeOldTrack > 0)
    {
        for (i=0; i<SizeOldTrack; i++)
        {
            const M10 *pCurEl = &OldTrack[i];
            if (pCurEl->M_info[0] > tGlue)
            {
                m_arProlData[ind_ball].ProlongedTrack.push_back(*pCurEl);
            }
        }
    }

}


void CBallisticProlongation::CorrInverseProlTrack(bool bNewEAP, qreal tNewEAP, GLPointDouble3D PointNewEAP)
{
    qint32 GTInd = m_pInData->IF_GT;
    qint32 BallInd = m_pInData->IF_BALL;
    bool bOK = true;
    if (0 <= GTInd && GTInd < AIR_BALL::GT_FORM_AMOUNT
            && 0 <= BallInd && BallInd < AIR_BALL::BALL_FORM_AMOUNT)
    {
        CAirBall_Proc_InnerData_GT *pGTData = &m_p_Arr_BallProc->arData_GT[GTInd];
        CBallistProlong_InnerData *pBallData = &m_arProlData[BallInd];

        qint16 Size = static_cast<qint16>(pBallData->ProlongedTrack.size());

        qreal tEAP = pGTData->tEAP;
        GLPointDouble3D EAP_Point = pGTData->EAP_Point.coord;
        if (bNewEAP
                && fabs(tEAP - AIR_BALL::Timer_INI) > con_eps
                && !EAP_Point.IsZero())
        {
            tEAP = tNewEAP;
            EAP_Point = PointNewEAP;
        }

        qreal t_attachment = m_pInData->tLoc;
        GLPointDouble3D PointAttachment = m_pInData->Coord;
        if (fabs(pBallData->tApogee - AIR_BALL::Timer_INI) > con_eps)
        {
            if (pGTData->tFirst > pBallData->tApogee
                    && fabs(tEAP - AIR_BALL::Timer_INI) > con_eps)
            {
                t_attachment = tEAP;
                PointAttachment = EAP_Point;
            }
            else
            {
                if (t_attachment > pBallData->tApogee)
                {
                    t_attachment = pBallData->tApogee;

                    if (fabs(pBallData->min_tApogee - AIR_BALL::Timer_INI) > con_eps
                            && t_attachment > pBallData->min_tApogee)
                    {
                        t_attachment = pBallData->min_tApogee;
                    }
                    bOK = pBallData->InterpolationProlongAtTime(t_attachment, PointAttachment);
                }
            }
        }

        if (fabs(pBallData->tStart - AIR_BALL::Timer_INI) > con_eps
                && Size > 0)
        {
            if (pBallData->tStart > pGTData->tFirst)
            {
                pBallData->tStart = pGTData->tFirst;
            }

            qreal tFix = tEAP;
            GLPointDouble3D PointFix = EAP_Point;
            if (fabs(pGTData->ManTime - AIR_BALL::Timer_INI) > con_eps
                    && (pGTData->ManTime > tEAP
                        || fabs(tFix - AIR_BALL::Timer_INI) < con_eps))
            {
                    tFix = pGTData->ManTime;
                    PointFix = pGTData->ManPoint;
            }
            if (fabs(tFix - AIR_BALL::Timer_INI) < con_eps)
            {
                tFix = t_attachment;
                PointFix = PointAttachment;
            }

            qreal t0 = pBallData->ProlongedTrack[0].M_info[0];

            //time attachment: first time of prolonged track = start time
            CorrInverseProlTrack_MoveToPoint(pBallData->StartPoint, t0, tFix, pBallData->tStart, true);

            if (fabs(tFix - t_attachment) > con_eps
                    && !PointFix.IsZero()
                    && bOK)
            {
                //attachment track to the max(tEAP, tManeuver)
                CorrInverseProlTrack_MoveToPoint(PointFix, tFix, t_attachment, tFix, false);

                if (fabs(tEAP - AIR_BALL::Timer_INI) > con_eps
                        && fabs(pGTData->ManTime - AIR_BALL::Timer_INI) > con_eps
                        && fabs(tEAP - pGTData->ManTime) > con_eps
                        && t_attachment - tEAP > con_eps
                        && t_attachment - pGTData->ManTime > con_eps
                        && !EAP_Point.IsZero()
                        && !pGTData->ManPoint.IsZero())
                {
                    qreal tFix2 = tEAP;
                    PointFix = EAP_Point;
                    if (tFix2 > pGTData->ManTime)
                    {
                        tFix2 = pGTData->ManTime;
                        PointFix = pGTData->ManPoint;
                    }

                    //attachment track to the min(tEAP, tManeuver)
                    CorrInverseProlTrack_MoveToPoint(PointFix, tFix2, tFix, tFix2, false);
                    tFix = tFix2;
                }

                //attachment track to the start point
                CorrInverseProlTrack_MoveToPoint(pBallData->StartPoint, pBallData->tStart, tFix, pBallData->tStart, false);
            }
        }
    }
}


void CBallisticProlongation::CorrInverseProlTrack_old1(const M10 &SP, const std::vector<M10> &TrackFragment)
{
    const qint32 ind_ball = m_pInData->IF_BALL; //index of ballistic trajectory

    std::vector<M10> OldTrack = m_arProlData[ind_ball].ProlongedTrack;    //old prolonged track
    const qint32 SizeOldTrack = OldTrack.size();
    std::vector<M10>().swap(m_arProlData[ind_ball].ProlongedTrack); //cleaning of m_arProlData[ind_ball].ProlongedTrack
    //TrackFragment[0].M_info[0] - time t0
    //TrackFragment[0].M_info[1] - coordinate X0
    //TrackFragment[0].M_info[2] - coordinate Y0
    //TrackFragment[0].M_info[3] - coordinate Z0
    const qint32 Size0 = TrackFragment.size();
    //TrackFragment[Size0-1].M_info[0] - time tLoc
    //TrackFragment[Size0-1].M_info[1] - coordinate X_Loc ...

    //add Yulia

    bool bResOK = false; //true if result is ok
    const qreal t_sp = SP.M_info[0];
    const qreal t0 = TrackFragment[0].M_info[0];

    M10 NewPoint;
    std::vector<M10> NewTrack;

    qint32 Loc_SP=0;
    qint32 Loc0=0;

    qreal dx_sp;
    qreal dy_sp;
    qreal dz_sp;
    qreal dx_0;
    qreal dy_0;
    qreal dz_0;

    qreal dx=0;
    qreal dy=0;
    qreal dz=0;
    qreal dt=0;

    if(t_sp > OldTrack[0].M_info[0] || fabs(t_sp - OldTrack[0].M_info[0]) < con_eps)
    {
        bResOK = true;
        for(qint32 i=0; i<SizeOldTrack-1; i++)
        {
            if(OldTrack[i].M_info[0] - con_eps < t_sp && t_sp < OldTrack[i+1].M_info[0])
            {
                Loc_SP = i;
            }
            if(OldTrack[i].M_info[0] - con_eps < t0 && t0 < OldTrack[i+1].M_info[0])
            {
                Loc0 = i;
            }
        }

        dx_sp = SP.M_info[1] - OldTrack[Loc_SP].M_info[1];
        dy_sp = SP.M_info[2] - OldTrack[Loc_SP].M_info[2];
        dz_sp = SP.M_info[3] - OldTrack[Loc_SP].M_info[3];

        dx_0 = TrackFragment[0].M_info[1] - OldTrack[Loc0].M_info[1];
        dy_0 = TrackFragment[0].M_info[2] - OldTrack[Loc0].M_info[2];
        dz_0 = TrackFragment[0].M_info[3] - OldTrack[Loc0].M_info[3];

        NewTrack.push_back(SP);
        qint32 SizeNew = 1;
        for(qint32 i=Loc_SP+1; i<Loc0; i++)
        {
            if (fabs(t_sp - t0) > con_eps)
            {
                dx = (dx_sp - dx_0)*(OldTrack[i].M_info[0] - t0)/(t_sp - t0) + dx_0;
                dy = (dy_sp - dy_0)*(OldTrack[i].M_info[0] - t0)/(t_sp - t0) + dy_0;
                dz = (dz_sp - dz_0)*(OldTrack[i].M_info[0] - t0)/(t_sp - t0) + dz_0;
            }

            NewPoint.M_info[0] = OldTrack[i].M_info[0];
            NewPoint.M_info[1] = dx + OldTrack[i].M_info[1];
            NewPoint.M_info[2] = dy + OldTrack[i].M_info[2];
            NewPoint.M_info[3] = dz + OldTrack[i].M_info[3];

            dt = NewPoint.M_info[0] - NewTrack[SizeNew-1].M_info[0];

            if (dt > con_eps)
            {
                NewPoint.M_info[4] = (NewPoint.M_info[1] - NewTrack[SizeNew-1].M_info[1])/dt;
                NewPoint.M_info[5] = (NewPoint.M_info[2] - NewTrack[SizeNew-1].M_info[2])/dt;
                NewPoint.M_info[6] = (NewPoint.M_info[3] - NewTrack[SizeNew-1].M_info[3])/dt;

                NewPoint.M_info[7] = (NewPoint.M_info[4] - NewTrack[SizeNew-1].M_info[4])/dt;
                NewPoint.M_info[8] = (NewPoint.M_info[5] - NewTrack[SizeNew-1].M_info[5])/dt;
                NewPoint.M_info[9] = (NewPoint.M_info[6] - NewTrack[SizeNew-1].M_info[6])/dt;

                NewTrack.push_back(NewPoint);
                SizeNew ++;
                NewPoint.Reset();
            }
//            else
//            {
//                bResOK = false;
//            }
        }

        if (bResOK)
        {
            for(qint32 i=0; i<Size0; i++)
            {
                NewTrack.push_back(TrackFragment[i]);
            }

            m_arProlData[ind_ball].ProlongedTrack = NewTrack;
        }
    }
    else if(t_sp < OldTrack[0].M_info[0])
    {
        bResOK = CorrInverseProlTrackTStartLessTOld(SP, OldTrack, TrackFragment);
    }
    else
    {
    }

    if (!bResOK)
    {
        m_arProlData[ind_ball].ProlongedTrack.push_back(SP);
        qreal tGlue = SP.M_info[0];
        qint16 i;

        qint32 SizeFragm = TrackFragment.size();
        if (SizeFragm > 0)
        {
            for (i=0; i<SizeFragm; i++)
            {
                const M10 *pCurEl = &TrackFragment[i];
                if (pCurEl->M_info[0] > tGlue)
                {
                    m_arProlData[ind_ball].ProlongedTrack.push_back(*pCurEl);
                    tGlue = pCurEl->M_info[0];
                }
            }
        }

        if (SizeOldTrack > 0)
        {
            for (i=0; i<SizeOldTrack; i++)
            {
                const M10 *pCurEl = &OldTrack[i];
                if (pCurEl->M_info[0] > tGlue)
                {
                    m_arProlData[ind_ball].ProlongedTrack.push_back(*pCurEl);
                }
            }
        }
    }
}


bool CBallisticProlongation::CorrInverseProlTrackTStartLessTOld(const M10 &SP, const std::vector<M10> &OldTrack, const std::vector<M10> &TrackFragment)
{
    bool bResOK = false;
    const M10 *pOld0 = &OldTrack[0];
    const M10 *pN0 = &TrackFragment[0];
    qreal t0 = pOld0->M_info[0];
    qreal tN = pN0->M_info[0];
    const qint32 SizeOldTrack = OldTrack.size();
    const qint32 ind_ball = m_pInData->IF_BALL; //index of ballistic trajectory

    if ((tN - t0) > con_eps)
    {
        qint16 i, j;
        qint16 iN = CalculateNforT(tN, OldTrack);

        if (iN > 0)
        {
            M10 Mult;
            const M10 *pNi = &OldTrack[iN];
            qreal dtN = pN0->M_info[0] - pOld0->M_info[0];
            for (j=1; j<10; j++)
            {
                Mult.M_info[j] = (pN0->M_info[j] - pNi->M_info[j])/dtN;
            }

            std::vector<M10> CmpTrack;
            for (i=0; i<SizeOldTrack; i++)
            {
                const M10 *pOldCur = &OldTrack[i];
                if (tN > pOldCur->M_info[0])
                {
                    M10 NewEl;
                    NewEl.M_info[0] = pOldCur->M_info[0];
                    qreal dtCur = pOldCur->M_info[0] - pOld0->M_info[0];
                    for (j=1; j<10; j++)
                    {
                        NewEl.M_info[j] = pOldCur->M_info[j] + Mult.M_info[j]*dtCur;
                    }
                    CmpTrack.push_back(NewEl);
                }
            }

            qint16 SizeCmp = CmpTrack.size();
            if (SizeCmp > 0)
            {
                bResOK = true;
                std::vector<M10> NewTrack;
                NewTrack.push_back(SP);
                for (i=1; i<SizeCmp; i++)
                {
                    const M10 *pPrevEl = &NewTrack[i-1];
                    const M10 *pPrevCmp = &CmpTrack[i-1];
                    const M10 *pCurCmp = &CmpTrack[i];
                    M10 NewEl;
                    for (j=0; j<10; j++)
                    {
                        if (fabs(pN0->M_info[j]-pPrevCmp->M_info[j]) > con_eps)
                        {
                            NewEl.M_info[j] = pPrevEl->M_info[j] + (pN0->M_info[j] - pPrevEl->M_info[j]) * (pCurCmp->M_info[j]-pPrevCmp->M_info[j])/(pN0->M_info[j]-pPrevCmp->M_info[j]);
                        }
                        else
                        {
                            bResOK = false;
                            break;
                        }
                    }
                    if (bResOK)
                    {
                        NewTrack.push_back(NewEl);
                    }
                    else
                    {
                        break;
                    }
                }

                if (bResOK)
                {
                    qreal tGlue = SP.M_info[0];
                    qint32 SizeNew = NewTrack.size();
                    if (SizeNew > 0)
                    {
                        for (i=0; i<SizeNew; i++)
                        {
                            const M10 *pCurEl = &NewTrack[i];
                            if (pCurEl->M_info[0] > tGlue)
                            {
                                m_arProlData[ind_ball].ProlongedTrack.push_back(*pCurEl);
                                tGlue = pCurEl->M_info[0];
                            }
                        }
                    }

                    qint32 SizeFragm = TrackFragment.size();
                    if (SizeFragm > 0)
                    {
                        for (i=0; i<SizeFragm; i++)
                        {
                            const M10 *pCurEl = &TrackFragment[i];
                            if (pCurEl->M_info[0] > tGlue)
                            {
                                m_arProlData[ind_ball].ProlongedTrack.push_back(*pCurEl);
                                tGlue = pCurEl->M_info[0];
                            }
                        }
                    }
                }
            }
        }
    }
    return bResOK;
}


void CBallisticProlongation::CorrInverseProlTrack_MoveToPoint(GLPointDouble3D &BasePoint, qreal t_0, qreal t_end, qreal t_0_base, bool bTimeCorr)
{
    qint32 BallInd = m_pInData->IF_BALL;
    if (0 <= BallInd && BallInd < AIR_BALL::BALL_FORM_AMOUNT
            && t_end - t_0 > con_eps
            && t_end - t_0_base > con_eps
            && t_end > t_0_base
            && t_end > t_0)
    {
        CBallistProlong_InnerData *pBallData = &m_arProlData[BallInd];

        GLPointDouble3D Point_0; //initial prolongation at time moment t0
        GLPointDouble3D Point_end; //initial prolongation at time moment t_end
        pBallData->InterpolationProlongAtTime(t_0, Point_0);
        pBallData->InterpolationProlongAtTime(t_end, Point_end);

        std::vector<M10>::iterator it;
        if (bTimeCorr && fabs(t_0_base - t_0) > con_eps
                && fabs(t_0 - t_end) > con_eps) //time correction
        {
            qreal CoefT = (t_0_base - t_end) / (t_0 - t_end);

            for (it = pBallData->ProlongedTrack.begin(); it != pBallData->ProlongedTrack.end(); it++)
            {
                M10 *pCurEl = &(*it);
                if (t_end > pCurEl->M_info[0])
                {
                    pCurEl->M_info[0] = t_end + (pCurEl->M_info[0] - t_end) * CoefT;
                }
                else
                {
                    break;
                }
            }
        }

//        qreal CoefX = (BasePoint.x - Point_0.x) / m_Tm.secST(t_0_base, t_end);
//        qreal CoefY = (BasePoint.y - Point_0.y) / m_Tm.secST(t_0_base, t_end);
//        qreal CoefZ = (BasePoint.z - Point_0.z) / m_Tm.secST(t_0_base, t_end);

//        qreal Coef_X_t = (BasePoint.x - Point_end.x) / m_Tm.secST(t_0_base, t_end);
//        qreal Coef_Y_t = (BasePoint.y - Point_end.y) / m_Tm.secST(t_0_base, t_end);
//        qreal Coef_Z_t = (BasePoint.z - Point_end.z) / m_Tm.secST(t_0_base, t_end);

//        qreal CoefX = 0;
//        qreal CoefY = 0;
//        qreal CoefZ = 0;
//        if (fabs(BasePoint.x - Point_end.x) > con_eps)
//        {
//            CoefX = (BasePoint.x - Point_end.x) / (Point_0.x - Point_end.x);
//        }
//        if (fabs(BasePoint.y - Point_end.y) > con_eps)
//        {
//            CoefY = (BasePoint.y - Point_end.y) / (Point_0.y - Point_end.y);
//        }
//        if (fabs(BasePoint.z - Point_end.z) > con_eps)
//        {
//            CoefZ = (BasePoint.z - Point_end.z) / (Point_0.z - Point_end.z);
//        }

//        for (it = pBallData->ProlongedTrack.begin(); it != pBallData->ProlongedTrack.end(); it++)
//        {
//            M10 *pCurEl = &(*it);
//            if (m_Tm.secIsGreater(t_end, pCurEl->M_info[0]))
//            {
////                pCurEl->M_info[1] += CoefX * m_Tm.secST(pCurEl->M_info[0], t_end);
////                pCurEl->M_info[2] += CoefY * m_Tm.secST(pCurEl->M_info[0], t_end);
////                pCurEl->M_info[3] += CoefZ * m_Tm.secST(pCurEl->M_info[0], t_end);
//                pCurEl->M_info[1] = Point_end.x + CoefX*(pCurEl->M_info[1] - Point_end.x);
//                pCurEl->M_info[2] = Point_end.y + CoefY*(pCurEl->M_info[2] - Point_end.y);
//                pCurEl->M_info[3] = Point_end.z + CoefZ*(pCurEl->M_info[3] - Point_end.z);

//                if (CoefX < 0) //corrections for convexity
//                {
//                    qreal Xmean = Point_end.x + Coef_X_t * m_Tm.secST(pCurEl->M_info[0], t_end);
//                    pCurEl->M_info[1] = 2.*Xmean - pCurEl->M_info[1];
//                }
//                if (CoefY < 0)
//                {
//                    qreal Ymean = Point_end.y + Coef_Y_t * m_Tm.secST(pCurEl->M_info[0], t_end);
//                    pCurEl->M_info[2] = 2.*Ymean - pCurEl->M_info[2];
//                }
//                if (CoefZ < 0)
//                {
//                    qreal Zmean = Point_end.z + Coef_Z_t * m_Tm.secST(pCurEl->M_info[0], t_end);
//                    pCurEl->M_info[3] = 2.*Zmean - pCurEl->M_info[3];
//                }
//            }
//            else
//            {
//                break;
//            }
//        }


        qreal dPoints = BasePoint.getDistance(Point_0);
        if (dPoints > con_eps)
        {
            //rotation of vector (Point_end->Point_0) to the vector (Point_end->BasePoint):
            //      1) in the horizontal plane;
            //      2) in the vertical plane
            //and multiplication by (distance(Point_end,BasePoint)/distance(Point_end,Point_0))
            CGeocentric PointEnd_gc(Point_end.x, Point_end.y, Point_end.z);
            CGeocentric Point0_gc(Point_0.x, Point_0.y, Point_0.z);
            CGeocentric PointBase_gc(BasePoint.x, BasePoint.y, BasePoint.z);
            CGeodesic PointEnd_gd;
            CTopocentric Point0_tp;
            CTopocentric PointBase_tp;
            CSpherical Point0_sph;
            CSpherical PointBase_sph;

            bool bOK = GEOCENTRIC_GEO(&PointEnd_gc, &PointEnd_gd);
            bOK = bOK && GEOCENTRIC_TOPO(&PointEnd_gd, &Point0_gc, &Point0_tp);
            bOK = bOK && GEOCENTRIC_TOPO(&PointEnd_gd, &PointBase_gc, &PointBase_tp);

            bOK = bOK && TOPO_SPHERICAL(&Point0_tp, &Point0_sph);
            bOK = bOK && TOPO_SPHERICAL(&PointBase_tp, &PointBase_sph);

            GLPointDouble3D PointBase_gl_tp(PointBase_tp.m_dXt, PointBase_tp.m_dYt, PointBase_tp.m_dZt);            
//            GLPointDouble3D Point0_gl_tp(Point0_tp.m_dXt, Point0_tp.m_dYt, Point0_tp.m_dZt);

            GLPointDouble3D VectY_tp(0., 1., 0);

            TMatrix<3> RotationMatr(3,3); //combined rotation matrix
            TMatrix<3> HorRotationMatr(3,3); //horizontal rotation matrix
            TMatrix<3> VertRotationMatr(3,3); //vertical rotation matrix

            qreal ThetaHor = -(PointBase_sph.m_dB - Point0_sph.m_dB);
            HorRotationMatr.FormRotationMatrix(ThetaHor, VectY_tp.x, VectY_tp.y, VectY_tp.z);

            GLPointDouble3D VectNormToVert = PointBase_gl_tp.VectorProduct(VectY_tp);
            qreal ThetaVert = PointBase_sph.m_dE - Point0_sph.m_dE;
            VertRotationMatr.FormRotationMatrix(ThetaVert, VectNormToVert.x, VectNormToVert.y, VectNormToVert.z);

            if (fabs(ThetaHor) > con_eps || fabs(ThetaVert) > con_eps)
            {
                TCMatrixFunc<3> FMatr;
                bOK = bOK && FMatr.MatrXMatr(VertRotationMatr, HorRotationMatr, RotationMatr);

                qreal d0 = Point0_sph.m_dR;
                qreal dBase = PointBase_sph.m_dR;

                if (bOK && d0 > con_eps && dBase > con_eps)
                {
                    qreal Coeff = dBase / d0;

                    for (it = pBallData->ProlongedTrack.begin(); it != pBallData->ProlongedTrack.end(); it++)
                    {
                        M10 *pCurEl = &(*it);
                        if (t_end > pCurEl->M_info[0])
                        {
                            CGeocentric CurrPoint_gc(pCurEl->M_info[1], pCurEl->M_info[2], pCurEl->M_info[3]);
                            CGeocentric NewPoint_gc;
                            CTopocentric CurrPoint_tp;
                            CTopocentric NewPoint_tp;

                            bOK = GEOCENTRIC_TOPO(&PointEnd_gd, &CurrPoint_gc, &CurrPoint_tp);
                            TVector<3> CurrVect_tp(3);
                            TVector<3> NewVect_tp(3);
                            CurrVect_tp[0] = CurrPoint_tp.m_dXt;
                            CurrVect_tp[1] = CurrPoint_tp.m_dYt;
                            CurrVect_tp[2] = CurrPoint_tp.m_dZt;

                            bOK = bOK && FMatr.MatrXVec(RotationMatr, CurrVect_tp, NewVect_tp);

                            NewVect_tp = NewVect_tp * Coeff;

                            NewPoint_tp.init(NewVect_tp[0], NewVect_tp[1], NewVect_tp[2]);
                            bOK = bOK && TOPO_GEOCENTRIC(&PointEnd_gd, &NewPoint_tp, &NewPoint_gc);
//debug
//CSpherical NewPoint_sph, PCmpRotBeta, PCmpRotEps;
//TOPO_SPHERICAL(&NewPoint_tp, &NewPoint_sph);
//TVector<3> VectRotBeta, VectRotEps;
//FMatr.MatrXVec(HorRotationMatr, CurrVect_tp, VectRotBeta);
//FMatr.MatrXVec(VertRotationMatr, CurrVect_tp, VectRotEps);
//CTopocentric Point_tp_beta(VectRotBeta[0], VectRotBeta[1], VectRotBeta[2]);
//CTopocentric Point_tp_eps(VectRotEps[0], VectRotEps[1], VectRotEps[2]);
//TOPO_SPHERICAL(&Point_tp_beta, &PCmpRotBeta);
//TOPO_SPHERICAL(&Point_tp_eps, &PCmpRotEps);
//end of debug
                            if (bOK)
                            {
                                pCurEl->M_info[1] = NewPoint_gc.m_dX;
                                pCurEl->M_info[2] = NewPoint_gc.m_dY;
                                pCurEl->M_info[3] = NewPoint_gc.m_dZ;
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        qint32 ind_0 = pBallData->GetNearIndex(t_0_base);
        if (ind_0 >= 0)
        {
            pBallData->ProlongedTrack[ind_0].M_info[0] = t_0_base;
            pBallData->ProlongedTrack[ind_0].M_info[1] = BasePoint.x;
            pBallData->ProlongedTrack[ind_0].M_info[2] = BasePoint.y;
            pBallData->ProlongedTrack[ind_0].M_info[3] = BasePoint.z;
        }
    }
}


qint16 CBallisticProlongation::CalculateNforT(qreal tN, const std::vector<M10> &OldTrack)
{
    qint16 iN=0;
    const qint32 SizeOldTrack = OldTrack.size();
    qint32 i;
    for (i=0; i<SizeOldTrack-1; i++)
    {
        qreal tCur = OldTrack[i].M_info[0];
        qreal tNext = OldTrack[i+1].M_info[0];
        if (tN > tCur - con_eps && tNext > tN)
        {
            iN = i;
            break;
        }
    }
    return iN;
}


void CBallisticProlongation::CalcOutPoints(qint32 GTInd, qint32 ind_ball)
{
    m_OutData.arrPoints.Reset();

    if (0 <= ind_ball && ind_ball < AIR_BALL::BALL_FORM_AMOUNT
            && 0 <= GTInd && GTInd < AIR_BALL::GT_FORM_AMOUNT)
    {
        const std::vector<M10> ProlTrack = m_arProlData[ind_ball].ProlongedTrack;

        CAirBall_Proc_InnerData_GT *pGTData = &m_p_Arr_BallProc->arData_GT[GTInd];

        const qint32 TrSize = ProlTrack.size();
        if (TrSize < 1)
        {
            return;
        }

        bool bFromStart = false; //true - output track from start point
        bool bJoinIniTrackFromClst = false; //true - join initial points from the cluster to the prolonged trajectory points of current track

        if (pGTData->GTHasCluster())
        {
            if (m_p_Arr_BallProc->GTIsParentInCluster(GTInd) || m_p_Arr_BallProc->GTIsFirstWHInCluster(GTInd))
            {
                CAirBall_Proc_ClstGTData *pClstData = &m_p_Arr_BallProc->arData__Clst[pGTData->ClstNum];
                if (fabs(pClstData->tStart - AIR_BALL::Timer_INI) > con_eps
                        && pGTData->tFirst > pClstData->tStart
                        && pClstData->bArrPointsFilled
                        && !pClstData->InitialArrPoints.empty()                        
                        && (fabs(pClstData->tStart - m_arProlData[ind_ball].tStart) > con_eps
                            || !(pClstData->StartPoint - m_arProlData[ind_ball].StartPoint).IsZero()))
                {
                    TracksPoints P_last = pClstData->InitialArrPoints.back();
                    GLPointDouble3D P_prol;
                    bool bOK = m_arProlData[ind_ball].InterpolationProlongAtTime(P_last.time_point, P_prol);
                    double Dist_last_prol = P_last.P.getDistance(P_prol);
                    if (bOK && Dist_last_prol < std::min(m_arProlData[ind_ball].StartEll.aI, c_MaxStrobeSize))
                    {
                        bJoinIniTrackFromClst = true;
                    }
                    else
                    {
                        bFromStart = true;
                    }
                }
                else
                {
                    bFromStart = true;
                }
            }
        }
        else
        {
            bFromStart = true;
        }

        qreal t0; //initial time
        qint16 i_begin;
        qint16 N_Points_curr = AIR_BALL::N_OUT_POINTS;


        if (bFromStart)
        {
            t0 = ProlTrack[0].M_info[0];
            m_OutData.arrPoints[0].time_point = t0; //initial point
            m_OutData.arrPoints[0].P.x = ProlTrack[0].M_info[1];
            m_OutData.arrPoints[0].P.y = ProlTrack[0].M_info[2];
            m_OutData.arrPoints[0].P.z = ProlTrack[0].M_info[3];
            m_OutData.arrPoints[0].V.x = ProlTrack[0].M_info[4];
            m_OutData.arrPoints[0].V.y = ProlTrack[0].M_info[5];
            m_OutData.arrPoints[0].V.z = ProlTrack[0].M_info[6];
            m_OutData.arrPoints[0].A.x = ProlTrack[0].M_info[7];
            m_OutData.arrPoints[0].A.y = ProlTrack[0].M_info[8];
            m_OutData.arrPoints[0].A.z = ProlTrack[0].M_info[9];
            i_begin = 1;
        }
        else
        {
            t0 = pGTData->tFirst;
            i_begin = 0;

            if (bJoinIniTrackFromClst)
            {
                CAirBall_Proc_ClstGTData *pClstData = &m_p_Arr_BallProc->arData__Clst[pGTData->ClstNum];

                qint16 i = 0;
                std::vector<TracksPoints>::iterator it;
                for (it = pClstData->InitialArrPoints.begin(); it != pClstData->InitialArrPoints.end(); it++)
                {
                    if (i < AIR_BALL::N_OUT_POINTS)
                    {
                        TracksPoints CurrPoint = (*it);
                        m_OutData.arrPoints[i] = CurrPoint;
                        i++;
                    }
                }
                i_begin = i;
            }
        }

        N_Points_curr = AIR_BALL::N_OUT_POINTS - i_begin;

        //time interval between points; N_OUT_POINTS points i.e. (N_OUT_POINTS-1) intervals
        const qreal deltaT = (ProlTrack[TrSize-1].M_info[0] - t0) / static_cast<qreal>(N_Points_curr - 1);
        qint32 k=0;
        for (qint16 i0=0; i0<N_Points_curr; i0++)
        {
            qint16 i = i0 + i_begin;
            const qreal cur_t = t0 + static_cast<qreal>(i0) * deltaT;
            m_OutData.arrPoints[i].time_point = cur_t;
            while (ProlTrack[k].M_info[0] < cur_t && k < TrSize-1)
            {
                k++;
            }

            if (fabs(ProlTrack[k].M_info[0] - cur_t) < con_eps || k==0)
            {
                m_OutData.arrPoints[i].P.x = ProlTrack[k].M_info[1];
                m_OutData.arrPoints[i].P.y = ProlTrack[k].M_info[2];
                m_OutData.arrPoints[i].P.z = ProlTrack[k].M_info[3];
                m_OutData.arrPoints[i].V.x = ProlTrack[k].M_info[4];
                m_OutData.arrPoints[i].V.y = ProlTrack[k].M_info[5];
                m_OutData.arrPoints[i].V.z = ProlTrack[k].M_info[6];
                m_OutData.arrPoints[i].A.x = ProlTrack[k].M_info[7];
                m_OutData.arrPoints[i].A.y = ProlTrack[k].M_info[8];
                m_OutData.arrPoints[i].A.z = ProlTrack[k].M_info[9];
            }
            else
            {
                const qreal rel_T = (cur_t - ProlTrack[k-1].M_info[0])
                        / (ProlTrack[k].M_info[0] - ProlTrack[k-1].M_info[0]);
                m_OutData.arrPoints[i].P.x = ProlTrack[k-1].M_info[1]
                        + rel_T * (ProlTrack[k].M_info[1] - ProlTrack[k-1].M_info[1]);
                m_OutData.arrPoints[i].P.y = ProlTrack[k-1].M_info[2]
                        + rel_T * (ProlTrack[k].M_info[2] - ProlTrack[k-1].M_info[2]);
                m_OutData.arrPoints[i].P.z = ProlTrack[k-1].M_info[3]
                        + rel_T * (ProlTrack[k].M_info[3] - ProlTrack[k-1].M_info[3]);
                m_OutData.arrPoints[i].V.x = ProlTrack[k-1].M_info[4]
                        + rel_T * (ProlTrack[k].M_info[4] - ProlTrack[k-1].M_info[4]);
                m_OutData.arrPoints[i].V.y = ProlTrack[k-1].M_info[5]
                        + rel_T * (ProlTrack[k].M_info[5] - ProlTrack[k-1].M_info[5]);
                m_OutData.arrPoints[i].V.z = ProlTrack[k-1].M_info[6]
                        + rel_T * (ProlTrack[k].M_info[6] - ProlTrack[k-1].M_info[6]);
                m_OutData.arrPoints[i].A.x = ProlTrack[k-1].M_info[7]
                        + rel_T * (ProlTrack[k].M_info[7] - ProlTrack[k-1].M_info[7]);
                m_OutData.arrPoints[i].A.y = ProlTrack[k-1].M_info[8]
                        + rel_T * (ProlTrack[k].M_info[8] - ProlTrack[k-1].M_info[8]);
                m_OutData.arrPoints[i].A.z = ProlTrack[k-1].M_info[9]
                        + rel_T * (ProlTrack[k].M_info[9] - ProlTrack[k-1].M_info[9]);
            }
        }
        m_OutData.bArrPointsFilled = true;

//        if (fabs(pGTData->tEAP - AIR_BALL::Timer_INI) > con_eps && !pGTData->EAP_Point.coord.IsZero())
//        {
//            SubstituteOutPoint(pGTData->tEAP, pGTData->EAP_Point.coord);
//        }

//        if (fabs(pGTData->ManTime - AIR_BALL::Timer_INI) > con_eps && !pGTData->ManPoint.IsZero())
//        {
//            SubstituteOutPoint(pGTData->ManTime, pGTData->ManPoint);
//        }
    }
}


void CBallisticProlongation::SubstituteOutPoint(const qreal &t0, GLPointDouble3D &Point0)
{
    if (fabs(t0 - AIR_BALL::Timer_INI) > con_eps && !Point0.IsZero())
    {
        qint32 IndSubst = m_OutData.GetNearIndex(t0);
        if (IndSubst > 0 && IndSubst < AIR_BALL::N_OUT_POINTS - 1)
        {
            m_OutData.arrPoints[IndSubst].time_point = t0;
            m_OutData.arrPoints[IndSubst].P = Point0;
        }
    }
}


qreal CBallisticProlongation::CalcDistance()
{
    qreal d = -1;
    const qint32 ind_ball = m_pInData->IF_BALL;
    bool bOK = true;

    if (m_arProlData[ind_ball].StartPoint.module() < con_par_eps)
    {
        return d;
    }
    if (m_arProlData[ind_ball].FallPoint.module() < con_par_eps)
    {
        return d;
    }

    CGeocentric SP_gc(m_arProlData[ind_ball].StartPoint.x,
                      m_arProlData[ind_ball].StartPoint.y,
                      m_arProlData[ind_ball].StartPoint.z),
                FP_gc(m_arProlData[ind_ball].FallPoint.x,
                      m_arProlData[ind_ball].FallPoint.y,
                      m_arProlData[ind_ball].FallPoint.z);
    CGeodesic SP_gd;
    CGeodesic FP_gd;
    bOK = bOK && GEOCENTRIC_GEO(&SP_gc, &SP_gd);
    bOK = bOK && GEOCENTRIC_GEO(&FP_gc, &FP_gd);

    if (bOK)
    {
        d = SP_gd.GetGeodesicLine(SP_gd, FP_gd);
    }
    return d;
}


void CBallisticProlongation::ClearInnerData()
{
    //for (qint32 i=0; )
    for (qint32 i=0; i<AIR_BALL::BALL_FORM_AMOUNT; i++)
    {
        ClearInnerData_1Track(i);
    }
}


void CBallisticProlongation::ClearInnerData_1Track(const qint32 ind_Ball)
{
    if (0 <= ind_Ball && ind_Ball < AIR_BALL::BALL_FORM_AMOUNT)
    {
        m_arProlData[ind_Ball].Reset();
    }
}
