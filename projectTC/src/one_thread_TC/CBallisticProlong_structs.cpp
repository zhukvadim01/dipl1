#include "CBallisticProlong_structs.h"

#include "gl/GLFileLog.h"

using namespace GLFileLog;

CBallProl_Input::CBallProl_Input()
{
//    Reset();
    CovMatr.Reset();
}

void CBallProl_Input::Reset()
{
    memset(this, 0, sizeof(CBallProl_Input));
    IF_GT = -1;
    IF_BALL = -1;
    Coord.clear();
    Vel.clear();
    Acc.clear();
    CovMatr.Reset();
    bAeroballistic = false;
}

CBallProl_Output::CBallProl_Output()
{
//    Reset();
}

void CBallProl_Output::Reset()
{
//    memset(this, 0, sizeof(CBallProl_Output));
    p_StartIsChanged = false;
    p_FallIsChanged = false;
    p_FallEllIsChanged = false;
    SignProgn = 0;
    bArrPointsFilled = false;
    arrPoints.Reset();
    Polynoms.Reset();
    AB_Trapeze.Reset();
    bBoostSepPointsChanged = false;
    PointSepBoost1.clear();
    PointSepBoost2.clear();
}


CBallProl_Output& CBallProl_Output::operator=(const CBallProl_Output &Q)
{
    this->p_StartIsChanged      = Q.p_StartIsChanged;
    this->p_FallIsChanged       = Q.p_FallIsChanged;
    this->p_FallEllIsChanged    = Q.p_FallEllIsChanged;
    this->SignProgn             = Q.SignProgn;
    this->arrPoints             = Q.arrPoints;
    this->bArrPointsFilled      = Q.bArrPointsFilled;
    this->Polynoms              = Q.Polynoms;
    this->AB_Trapeze            = Q.AB_Trapeze;
    this->bBoostSepPointsChanged= Q.bBoostSepPointsChanged;
    this->PointSepBoost1        = Q.PointSepBoost1;
    this->PointSepBoost2        = Q.PointSepBoost2;
    return *this;
}


qint32 CBallProl_Output::GetNearIndex(qreal tCurr)
{
    qint32 ResInd = -1;

    if (bArrPointsFilled)
    {        
        if (arrPoints[0].time_point > tCurr)
        {
            ResInd = 0;
        }
        else if (tCurr > arrPoints[AIR_BALL::N_OUT_POINTS-1].time_point)
        {
            ResInd = AIR_BALL::N_OUT_POINTS - 1;
        }
        else
        {
            for (qint32 i=0; i< AIR_BALL::N_OUT_POINTS-2; i++)
            {
                if (tCurr > arrPoints[i].time_point
                        && arrPoints[i+1].time_point > tCurr)
                {
                    ResInd = i;
                    if (fabs(arrPoints[i+1].time_point - tCurr) < fabs(tCurr - arrPoints[i].time_point))
                    {
                        ResInd = i+1;
                    }
                    break;
                }
            }
        }
    }

    return ResInd;
}


void CBallProl_Output::LogTrackPoints(FILE *_pLogFile, qreal tLoc, qint32 NumGT)
{
    if (_pLogFile != nullptr && bArrPointsFilled)
    {
        Log(_pLogFile, "%d %f %f %d %d %d %d %d %d %d",
            2, CURR_TIME_START_OF_DAY_SEC, tLoc, NumGT, 0, 0, 0, 0, 0, 0);
        for (qint16 i=0; i<AIR_BALL::N_OUT_POINTS; i++)
        {
            TracksPoints *pCurrPoint = &arrPoints[i];
            pCurrPoint->LogData(_pLogFile);
        }
    }
}


CBallistProlong_InnerData::CBallistProlong_InnerData()
{
//    Reset();
    std::vector<M10>().swap(ProlongedTrack);
}


bool CBallistProlong_InnerData::InterpolationProlongAtTime(qreal CurTime, GLPointDouble3D &CoordRes)
{
    bool bRes = false;
    CoordRes.clear();

    qint32 Size = static_cast<qint32>(ProlongedTrack.size());
    if (Size > 1)
    {
        bRes = true;

        M10 PointLeft = ProlongedTrack[0];
        M10 PointRight = ProlongedTrack[1];

        for (qint32 i=0; i<Size-1; i++)
        {
            M10 PointCurr = ProlongedTrack[i];
            M10 PointNext = ProlongedTrack[i+1];
            if (CurTime > PointCurr.M_info[0])
            {
                PointLeft = PointCurr;
                PointRight = PointNext;
            }
            if (PointNext.M_info[0] > CurTime)
            {
                break;
            }
        }

        GLPointDouble3D CoordLeft(PointLeft.M_info[1], PointLeft.M_info[2], PointLeft.M_info[3]),
                CoordRight(PointRight.M_info[1], PointRight.M_info[2], PointRight.M_info[3]);

        qreal dtLeft = CurTime - PointLeft.M_info[0];
        qreal dtRight = CurTime - PointRight.M_info[0];

        if (fabs(dtLeft) < con_eps)
        {
            CoordRes = CoordLeft;
        }
        else if (fabs(dtRight) < con_eps)
        {
            CoordRes = CoordRight;
        }
        else
        {
            GLPointDouble3D VelLeft(PointLeft.M_info[4], PointLeft.M_info[5], PointLeft.M_info[6]),
                    AccLeft(PointLeft.M_info[7], PointLeft.M_info[8], PointLeft.M_info[9]),
                    VelRigt(PointRight.M_info[4], PointRight.M_info[5], PointRight.M_info[6]),
                    AccRight(PointRight.M_info[7], PointRight.M_info[8], PointRight.M_info[9]);
            GLPointDouble3D PointExtrLeft, PointExtrRight;

            PointExtrLeft = CoordLeft + (VelLeft * dtLeft) + (AccLeft * sqr(dtLeft) * 0.5);
            PointExtrRight = CoordRight + (VelRigt * dtRight) + (AccRight * sqr(dtRight) * 0.5);

            qreal CoefLeft = fabs(dtRight) / (fabs(dtLeft) + fabs(dtRight));
            qreal CoefRight = fabs(dtLeft) / (fabs(dtLeft) + fabs(dtRight));

            CoordRes = (PointExtrLeft * CoefLeft) + (PointExtrRight * CoefRight);
        }

    }

    return bRes;
}


qint32 CBallistProlong_InnerData::GetNearIndex(qreal CurTime)
{
    qint32 ResInd = -1; //resulting value of index

    qint32 Size = static_cast<qint32>(ProlongedTrack.size());

    if (ProlongedTrack[0].M_info[0] > CurTime)
    {
        ResInd = 0;
    }
    else if (CurTime > ProlongedTrack[Size-1].M_info[0])
    {
        ResInd = Size - 1;
    }
    else
    {
        for (qint32 i=0; i<Size-1; i++)
        {
            if (CurTime > ProlongedTrack[i].M_info[0]
                    && ProlongedTrack[i+1].M_info[0] > CurTime)
            {
                ResInd = i;
                if (fabs(ProlongedTrack[i+1].M_info[0] - CurTime) < fabs(CurTime - ProlongedTrack[i].M_info[0]))
                {
                    ResInd = i+1;
                }
                break;
            }
        }
    }

    return ResInd;
}


void CBallistProlong_InnerData::Reset()
{
    StartPoint.clear();
    FallPoint.clear();
    StartPointFindMethod = 0;
    bStartByTablesPerformed = false;
    bStartByTablesPerformed_multival = false;
    tStart = AIR_BALL::Timer_INI;
    tFall = AIR_BALL::Timer_INI;
    StartEll.Reset();
    FallEll.Reset();
    tSavedProl = AIR_BALL::Timer_INI;
    tSavedEll = AIR_BALL::Timer_INI;
    N_Prol = 0;
    tDetStartDownPr = AIR_BALL::Timer_INI;
    tPrevTryDetStartDownPr = AIR_BALL::Timer_INI;
    CounterTryDetStartPointDownPr = 0;
    tPrevTryDetStartByTables = AIR_BALL::Timer_INI;
    std::vector<M10>().swap(ProlongedTrack);
    PolData.Reset();
    Hapogee = 0;
    tApogee = AIR_BALL::Timer_INI;
    min_tApogee = AIR_BALL::Timer_INI;
    SignLastProgn = 0;
    D = 0;
    ThrowAngle = 0;
    Veap = 0;
    Teap_byTable = AIR_BALL::Timer_INI;
    bBMMarkChanged = false;
    DeltaVmin = 0;
    MarkEAPDet = 0;
    CounterBadInvProl = 0;
}


void CBallistProlong_InnerData::LogProlongedTrack(FILE *_pLogFile, qreal tLoc, qint32 NumGT)
{
    if (_pLogFile != nullptr)
    {
        qint32 Size = static_cast<qint32>(ProlongedTrack.size());
        if (Size > 0)
        {
            Log(_pLogFile, "%d %f %f %d %d %d %d %d %d %d",
                1, CURR_TIME_START_OF_DAY_SEC, tLoc, NumGT, 0, 0, 0, 0, 0, 0);
            for (qint32 i=0; i<Size; i++)
            {
                M10 *pCurrEl = &ProlongedTrack[i];
                pCurrEl->LogData(_pLogFile);
            }
        }
    }
}
