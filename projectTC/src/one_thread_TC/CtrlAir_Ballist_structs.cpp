#include<fstream>
#include <iomanip>

#include "CtrlAir_Ballist_structs.h"
#include "gl/GLFileLog.h"


// using namespace std;
using namespace GLFileLog;

namespace AIR_BALL{
std::ofstream out_polynom(qPrintable(QString("%1/Ball_Polynom.txt").arg(log_path)));
}

void CtrlAIR_BALL_Input_GT_Data::Reset()
{
    TrackingSign = 0;
    NumbGT = 0;
    tLoc = 0;
    SetAssocST.clear();
    Class = 0;
    Type = 0;
    ClstNum = 0;
    SignQuickReaction = 0;
    bAeroballistic = false;
    X = 0;
    Y = 0;
    Z = 0;
    VX = 0;
    VY = 0;
    VZ = 0;
    AX = 0;
    AY = 0;
    AZ = 0;

    SigX = 0;
    SigY = 0;
    SigZ = 0;
    SigVX = 0;
    SigVY = 0;
    SigVZ = 0;
    SigAX = 0;
    SigAY = 0;
    SigAZ = 0;

    KXY = 0;
    KXZ = 0;
    KYZ = 0;
    KVxVy = 0;
    KVxVz = 0;
    KVyVz = 0;
    KAxAy = 0;
    KAxAz = 0;
    KAyAz = 0;
    KXVx = 0;
    KXVy = 0;
    KXVz = 0;
    KXAx = 0;
    KXAy = 0;
    KXAz = 0;
    KYVx = 0;
    KYVy = 0;
    KYVz = 0;
    KYAx = 0;
    KYAy = 0;
    KYAz = 0;
    KZVx = 0;
    KZVy = 0;
    KZVz = 0;
    KZAx = 0;
    KZAy = 0;
    KZAz = 0;
    KVxAx = 0;
    KVxAy = 0;
    KVxAz = 0;
    KVyAx = 0;
    KVyAy = 0;
    KVyAz = 0;
    KVzAx = 0;
    KVzAy = 0;
    KVzAz = 0;

    AssignTgtNum = 0;
}


CtrlAIR_BALL_Input_GT_Data& CtrlAIR_BALL_Input_GT_Data::operator =(const CtrlAIR_BALL_Input_GT_Data &Q)
{
    this->TrackingSign      = Q.TrackingSign;
    this->NumbGT            = Q.NumbGT;
    this->tLoc              = Q.tLoc;
    this->SetAssocST.clear();
    this->SetAssocST        = Q.SetAssocST;
    this->Class             = Q.Class;
    this->Type              = Q.Type;
    this->ClstNum           = Q.ClstNum;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->bAeroballistic    = Q.bAeroballistic;

    this->X                 = Q.X;
    this->Y                 = Q.Y;
    this->Z                 = Q.Z;
    this->VX                = Q.VX;
    this->VY                = Q.VY;
    this->VZ                = Q.VZ;
    this->AX                = Q.AX;
    this->AY                = Q.AY;
    this->AZ                = Q.AZ;

    this->SigX              = Q.SigX;
    this->SigY              = Q.SigY;
    this->SigZ              = Q.SigZ;
    this->SigVX             = Q.SigVX;
    this->SigVY             = Q.SigVY;
    this->SigVZ             = Q.SigVZ;
    this->SigAX             = Q.SigAX;
    this->SigAY             = Q.SigAY;
    this->SigAZ             = Q.SigAZ;

    this->KXY               = Q.KXY;
    this->KXZ               = Q.KXZ;
    this->KYZ               = Q.KYZ;
    this->KVxVy             = Q.KVxVy;
    this->KVxVz             = Q.KVxVz;
    this->KVyVz             = Q.KVyVz;
    this->KAxAy             = Q.KAxAy;
    this->KAxAz             = Q.KAxAz;
    this->KAyAz             = Q.KAyAz;
    this->KXVx              = Q.KXVx;
    this->KXVy              = Q.KXVy;
    this->KXVz              = Q.KXVz;
    this->KXAx              = Q.KXAx;
    this->KXAy              = Q.KXAy;
    this->KXAz              = Q.KXAz;
    this->KYVx              = Q.KYVx;
    this->KYVy              = Q.KYVy;
    this->KYVz              = Q.KYVz;
    this->KYAx              = Q.KYAx;
    this->KYAy              = Q.KYAy;
    this->KYAz              = Q.KYAz;
    this->KZVx              = Q.KZVx;
    this->KZVy              = Q.KZVy;
    this->KZVz              = Q.KZVz;
    this->KZAx              = Q.KZAx;
    this->KZAy              = Q.KZAy;
    this->KZAz              = Q.KZAz;
    this->KVxAx             = Q.KVxAx;
    this->KVxAy             = Q.KVxAy;
    this->KVxAz             = Q.KVxAz;
    this->KVyAx             = Q.KVyAx;
    this->KVyAy             = Q.KVyAy;
    this->KVyAz             = Q.KVyAz;
    this->KVzAx             = Q.KVzAx;
    this->KVzAy             = Q.KVzAy;
    this->KVzAz             = Q.KVzAz;

    this->AssignTgtNum      = Q.AssignTgtNum;

    return *this;
}


CtrlAIR_BALL_Input_GT_Data::CtrlAIR_BALL_Input_GT_Data (const CtrlAIR_BALL_Input_GT_Data &Q)
{
    this->TrackingSign      = Q.TrackingSign;
    this->NumbGT            = Q.NumbGT;
    this->tLoc              = Q.tLoc;
    this->SetAssocST.clear();
    this->SetAssocST        = Q.SetAssocST;
    this->Class             = Q.Class;
    this->Type              = Q.Type;
    this->ClstNum           = Q.ClstNum;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->bAeroballistic    = Q.bAeroballistic;

    this->X                 = Q.X;
    this->Y                 = Q.Y;
    this->Z                 = Q.Z;
    this->VX                = Q.VX;
    this->VY                = Q.VY;
    this->VZ                = Q.VZ;
    this->AX                = Q.AX;
    this->AY                = Q.AY;
    this->AZ                = Q.AZ;

    this->SigX              = Q.SigX;
    this->SigY              = Q.SigY;
    this->SigZ              = Q.SigZ;
    this->SigVX             = Q.SigVX;
    this->SigVY             = Q.SigVY;
    this->SigVZ             = Q.SigVZ;
    this->SigAX             = Q.SigAX;
    this->SigAY             = Q.SigAY;
    this->SigAZ             = Q.SigAZ;

    this->KXY               = Q.KXY;
    this->KXZ               = Q.KXZ;
    this->KYZ               = Q.KYZ;
    this->KVxVy             = Q.KVxVy;
    this->KVxVz             = Q.KVxVz;
    this->KVyVz             = Q.KVyVz;
    this->KAxAy             = Q.KAxAy;
    this->KAxAz             = Q.KAxAz;
    this->KAyAz             = Q.KAyAz;
    this->KXVx              = Q.KXVx;
    this->KXVy              = Q.KXVy;
    this->KXVz              = Q.KXVz;
    this->KXAx              = Q.KXAx;
    this->KXAy              = Q.KXAy;
    this->KXAz              = Q.KXAz;
    this->KYVx              = Q.KYVx;
    this->KYVy              = Q.KYVy;
    this->KYVz              = Q.KYVz;
    this->KYAx              = Q.KYAx;
    this->KYAy              = Q.KYAy;
    this->KYAz              = Q.KYAz;
    this->KZVx              = Q.KZVx;
    this->KZVy              = Q.KZVy;
    this->KZVz              = Q.KZVz;
    this->KZAx              = Q.KZAx;
    this->KZAy              = Q.KZAy;
    this->KZAz              = Q.KZAz;
    this->KVxAx             = Q.KVxAx;
    this->KVxAy             = Q.KVxAy;
    this->KVxAz             = Q.KVxAz;
    this->KVyAx             = Q.KVyAx;
    this->KVyAy             = Q.KVyAy;
    this->KVyAz             = Q.KVyAz;
    this->KVzAx             = Q.KVzAx;
    this->KVzAy             = Q.KVzAy;
    this->KVzAz             = Q.KVzAz;

    this->AssignTgtNum      = Q.AssignTgtNum;
}




//Output polynom coefficient to the log file
bool Polynoms_data::LogPolynom()
{
    if(AIR_BALL::out_polynom.is_open())
    {
        AIR_BALL::out_polynom << std::setprecision(10);
        AIR_BALL::out_polynom << TimeChange << '\t'
         << AnchorTime_4deg << '\t' << AnchorTime_8deg << '\t'

         << Part0_4deg.x << '\t' << Part0_4deg.y << '\t' << Part0_4deg.z << '\t'
         << Part1_4deg.x << '\t' << Part1_4deg.y << '\t' << Part1_4deg.z << '\t'
         << Part2_4deg.x << '\t' << Part2_4deg.y << '\t' << Part2_4deg.z << '\t'
         << Part3_4deg.x << '\t' << Part3_4deg.y << '\t' << Part3_4deg.z << '\t'
         << Part4_4deg.x << '\t' << Part4_4deg.y << '\t' << Part4_4deg.z << '\t'

         << Part0_8deg.x << '\t' << Part0_8deg.y << '\t' << Part0_8deg.z << '\t'
         << Part1_8deg.x << '\t' << Part1_8deg.y << '\t' << Part1_8deg.z << '\t'
         << Part2_8deg.x << '\t' << Part2_8deg.y << '\t' << Part2_8deg.z << '\t'
         << Part3_8deg.x << '\t' << Part3_8deg.y << '\t' << Part3_8deg.z << '\t'
         << Part4_8deg.x << '\t' << Part4_8deg.y << '\t' << Part4_8deg.z << '\t'
         << Part5_8deg.x << '\t' << Part5_8deg.y << '\t' << Part5_8deg.z << '\t'
         << Part6_8deg.x << '\t' << Part6_8deg.y << '\t' << Part6_8deg.z << '\t'
         << Part7_8deg.x << '\t' << Part7_8deg.y << '\t' << Part7_8deg.z << '\t'
         << Part8_8deg.x << '\t' << Part8_8deg.y << '\t' << Part8_8deg.z << std::endl;
        AIR_BALL::out_polynom.close();
        return true;
    }
    else
    {
        return false;
    }
}


void Polynoms_data::LogData(FILE *_pLogFile, const char *_Name)
{
    if (_pLogFile != nullptr)
    {
        fprintf(_pLogFile, "%9.3lf %s %f %f %f "
                        "%.10G %.10G %.10G %G %G %G "
                        "%G %G %G %G %G %G "
                        "%G %G %G %.10G %.10G %.10G "
                        "%G %G %G %G %G %G "
                        "%G %G %G %G %G %G "
                        "%G %G %G %G %G %G "
                        "%G %G %G %G %G %G "
                        "%f %.10G %.10G %.10G "
                        "%G %G %G %G %G %G\n",
             CURR_TIME_START_OF_DAY_SEC, _Name, AnchorTime_4deg, AnchorTime_8deg, TimeChange,
             Part0_4deg.x, Part0_4deg.y, Part0_4deg.z, Part1_4deg.x, Part1_4deg.y, Part1_4deg.z,
             Part2_4deg.x, Part2_4deg.y, Part2_4deg.z, Part3_4deg.x, Part3_4deg.y, Part3_4deg.z,
             Part4_4deg.x, Part4_4deg.y, Part4_4deg.z, Part0_8deg.x, Part0_8deg.y, Part0_8deg.z,
             Part1_8deg.x, Part1_8deg.y, Part1_8deg.z, Part2_8deg.x, Part2_8deg.y, Part2_8deg.z,
             Part3_8deg.x, Part3_8deg.y, Part3_8deg.z, Part4_8deg.x, Part4_8deg.y, Part4_8deg.z,
             Part5_8deg.x, Part5_8deg.y, Part5_8deg.z, Part6_8deg.x, Part6_8deg.y, Part6_8deg.z,
             Part7_8deg.x, Part7_8deg.y, Part7_8deg.z, Part8_8deg.x, Part8_8deg.y, Part8_8deg.z,
             AnchorTime_2deg, Part0_2deg.x, Part0_2deg.y, Part0_2deg.z,
             Part1_2deg.x, Part1_2deg.y, Part1_2deg.z, Part2_2deg.x, Part2_2deg.y, Part2_2deg.z);
    }
}


void CtrlAIR_BALL_Input_GT_Data::LogData(FILE *_pLogFile, bool bSimplifiedOut)
{
    if (_pLogFile != nullptr)
    {
        if (!bSimplifiedOut)
        {
            tLog(_pLogFile, "G D TrSign %d Num %d %f | %d %d %d %d %d | "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f %f %f %f "
                            "%f %f %f %d",
                 TrackingSign, NumbGT, tLoc, Class, Type, SignQuickReaction, AssignTgtNum, ClstNum,
                 X, Y, Z, VX, VY, VZ, AX, AY, AZ,
                 SigX, SigY, SigZ, SigVX, SigVY, SigVZ, SigAX, SigAY, SigAZ,
                 KXY, KXZ, KYZ, KVxVy, KVxVz, KVyVz, KAxAy, KAxAz, KAyAz,
                 KXVx, KXVy, KXVz, KXAx, KXAy, KXAz, KYVx, KYVy, KYVz, KYAx, KYAy, KYAz,
                 KZVx, KZVy, KZVz, KZAx, KZAy, KZAz, KVxAx, KVxAy, KVxAz, KVyAx, KVyAy, KVyAz,
                 KVzAx, KVzAy, KVzAz, bAeroballistic);

            //log set of ST
            char OutString[GL_MATR::MAX_LEN_STR];
            sprintf(OutString, "G S ");
            std::map<qint16, qint32>::iterator it;
            for (it = SetAssocST.begin(); it != SetAssocST.end(); it++)
            {
                qint32 Len = strlen(OutString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                std::pair<qint16, qint32> CurrEl = (*it);
                char TmpString[GL_MATR::MAX_LEN_STR];
                strcpy(TmpString, OutString);
                sprintf(OutString, "%s %d %d |", TmpString, CurrEl.first, CurrEl.second);
            }
            //tLog(_pLogFile, "%s", OutString);
            fprintf(_pLogFile, "%9.3lf %s\n", CURR_TIME_START_OF_DAY_SEC, OutString);
        }
        else //simplified output
        {
            tLog(_pLogFile, "%d %d %f %d %d %d %d "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f",
                 TrackingSign, NumbGT, tLoc, Class, Type, SignQuickReaction, AssignTgtNum,
                 X, Y, Z, VX, VY, VZ, AX, AY, AZ,
                 SigX, SigY, SigZ, SigVX, SigVY, SigVZ, SigAX, SigAY, SigAZ,
                 KXY, KXZ, KYZ, KVxVy, KVxVz, KVyVz, KAxAy, KAxAz, KAyAz,
                 KXVx, KXVy, KXVz, KXAx, KXAy, KXAz, KYVx, KYVy, KYVz, KYAx, KYAy, KYAz,
                 KZVx, KZVy, KZVz, KZAx, KZAy, KZAz, KVxAx, KVxAy, KVxAz, KVyAx, KVyAy, KVyAz,
                 KVzAx, KVzAy, KVzAz, bAeroballistic);
        }
    }
}


CtrlAIR_BALL_Input_ST_Data::CtrlAIR_BALL_Input_ST_Data()
{
//    Reset();
    VectParam.Reset(9);
    CovMatr.Reset(9,9);
}


void CtrlAIR_BALL_Input_ST_Data::Reset()
{
    memset(this, 0, sizeof(CtrlAIR_BALL_Input_ST_Data));
    VectParam.Reset(9);
    CovMatr.Reset(9,9);
}


CtrlAIR_BALL_Input_ST_Data& CtrlAIR_BALL_Input_ST_Data::operator =(const CtrlAIR_BALL_Input_ST_Data &Q)
{
    this->TrackingSign      = Q.TrackingSign;
    this->SrcInd            = Q.SrcInd;
    this->NtrInSrc          = Q.NtrInSrc;
    this->NumGT             = Q.NumGT;
    this->tLoc              = Q.tLoc;
    this->bMeasured         = Q.bMeasured;
    this->bVDopplPresent    = Q.bVDopplPresent;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->VectParam         = Q.VectParam;
    this->CovMatr           = Q.CovMatr;
    this->N_StepSmooth      = Q.N_StepSmooth;
    this->GammaFlt          = Q.GammaFlt;
    this->R                 = Q.R;
    this->VDoppl            = Q.VDoppl;
    this->RMSE_R            = Q.RMSE_R;
    this->RMSE_VDoppl       = Q.RMSE_VDoppl;
    this->RMSE_El           = Q.RMSE_El;
    this->Class             = Q.Class;
    this->Type              = Q.Type;
    this->Prob_KF_3D_222_small  = Q.Prob_KF_3D_222_small;
    this->Prob_KF_3D_222_large  = Q.Prob_KF_3D_222_large;

    return *this;
}

CtrlAIR_BALL_Input_ST_Data::CtrlAIR_BALL_Input_ST_Data (const CtrlAIR_BALL_Input_ST_Data &Q)
{
    this->TrackingSign      = Q.TrackingSign;
    this->SrcInd            = Q.SrcInd;
    this->NtrInSrc          = Q.NtrInSrc;
    this->NumGT             = Q.NumGT;
    this->tLoc              = Q.tLoc;
    this->bMeasured         = Q.bMeasured;
    this->bVDopplPresent    = Q.bVDopplPresent;
    this->SignQuickReaction = Q.SignQuickReaction;
    this->VectParam         = Q.VectParam;
    this->CovMatr           = Q.CovMatr;
    this->N_StepSmooth      = Q.N_StepSmooth;
    this->GammaFlt          = Q.GammaFlt;
    this->R                 = Q.R;
    this->VDoppl            = Q.VDoppl;
    this->RMSE_R            = Q.RMSE_R;
    this->RMSE_VDoppl       = Q.RMSE_VDoppl;
    this->RMSE_El           = Q.RMSE_El;
    this->Class             = Q.Class;
    this->Type              = Q.Type;
    this->Prob_KF_3D_222_small  = Q.Prob_KF_3D_222_small;
    this->Prob_KF_3D_222_large  = Q.Prob_KF_3D_222_large;
}

void CtrlAIR_BALL_Input_ST_Data::LogData(FILE *_pLogFile, bool bSimplifiedOut)
{
    if (_pLogFile != nullptr)
    {
        if (!bSimplifiedOut)
        {
            tLog(_pLogFile, "S D TrSign %d Src %d Num %d %d | %f %d %d %d "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f "
                            "%d %d %f %f %d",
                 TrackingSign, SrcInd, NtrInSrc, NumGT, tLoc, bMeasured, bVDopplPresent, SignQuickReaction,
                 VectParam.Vec[0], VectParam.Vec[1], VectParam.Vec[2], VectParam.Vec[3], VectParam.Vec[4], VectParam.Vec[5], VectParam.Vec[6], VectParam.Vec[7], VectParam.Vec[8],
                 GammaFlt, R, VDoppl, RMSE_R, RMSE_VDoppl, RMSE_El,
                 Class, Type, Prob_KF_3D_222_small, Prob_KF_3D_222_large, N_StepSmooth);

            //log covariance matrix
            tLogMatrixToLine(_pLogFile, "S C", CovMatr);
        }
        else
        {
            tLog(_pLogFile, "%d %d %d %d %f %d %d %d "
                            "%f %f %f %f %f %f %f %f %f "
                            "%f %f %f %f %f %f "
                            "%d %d %f %f %d",
                 TrackingSign, SrcInd, NtrInSrc, NumGT, tLoc, bMeasured, bVDopplPresent, SignQuickReaction,
                 VectParam.Vec[0], VectParam.Vec[1], VectParam.Vec[2], VectParam.Vec[3], VectParam.Vec[4], VectParam.Vec[5], VectParam.Vec[6], VectParam.Vec[7], VectParam.Vec[8],
                 GammaFlt, R, VDoppl, RMSE_R, RMSE_VDoppl, RMSE_El,
                 Class, Type, Prob_KF_3D_222_small, Prob_KF_3D_222_large, N_StepSmooth);
        }
    }
}


CtrlAIR_BALL_traj_param::CtrlAIR_BALL_traj_param()
{
//    Reset();
    Cells_BMMark_diap1.ArrCells.Reset();
    Cells_BMMark_diap2.ArrCells.Reset();
    Cells_BMMark_diap3.ArrCells.Reset();
}


void CtrlAIR_BALL_traj_param::Reset()
{
    memset(this, 0, sizeof(CtrlAIR_BALL_traj_param));
    Cells_BMMark_diap1.ArrCells.Reset();
    Cells_BMMark_diap2.ArrCells.Reset();
    Cells_BMMark_diap3.ArrCells.Reset();
}

TracksPoints::TracksPoints()
{
//    Reset();
}


void TracksPoints::Reset()
{
    time_point = 0;
    P.clear();
}


TracksPoints& TracksPoints::operator=(const TracksPoints &Q)
{
    this->P             = Q.P;
    this->V             = Q.V;
    this->A             = Q.A;
    this->time_point    = Q.time_point;

    return *this;
}

TracksPoints::TracksPoints (const TracksPoints &Q)
{
    this->P             = Q.P;
    this->V             = Q.V;
    this->A             = Q.A;
    this->time_point    = Q.time_point;
}


void TracksPoints::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        Log(_pLogFile, "%f %f %f %f %f %f %f %f %f %f",
            time_point, P.x, P.y, P.z, V.x, V.y, V.z, A.x, A.y, A.z);
    }
}


Polynoms_data::Polynoms_data()
{
    //Reset();
}


void Polynoms_data::Reset()
{
    Part0_4deg.clear();    Part1_4deg.clear();    Part2_4deg.clear();    Part3_4deg.clear();
    Part4_4deg.clear();    Part0_8deg.clear();    Part1_8deg.clear();    Part2_8deg.clear();
    Part3_8deg.clear();    Part4_8deg.clear();    Part5_8deg.clear();    Part6_8deg.clear();
    Part7_8deg.clear();    Part8_8deg.clear();
    Part0_2deg.clear();    Part1_2deg.clear();    Part2_2deg.clear();
    AnchorTime_4deg = 0;   AnchorTime_8deg = 0;   AnchorTime_2deg = 0;   TimeChange = 0;
}


Polynoms_data& Polynoms_data::operator=(const Polynoms_data &Q)
{
    this->Part0_4deg    = Q.Part0_4deg;
    this->Part1_4deg    = Q.Part1_4deg;
    this->Part2_4deg    = Q.Part2_4deg;
    this->Part3_4deg    = Q.Part3_4deg;
    this->Part4_4deg    = Q.Part4_4deg;

    this->Part0_8deg    = Q.Part0_8deg;
    this->Part1_8deg    = Q.Part1_8deg;
    this->Part2_8deg    = Q.Part2_8deg;
    this->Part3_8deg    = Q.Part3_8deg;
    this->Part4_8deg    = Q.Part4_8deg;
    this->Part5_8deg    = Q.Part5_8deg;
    this->Part6_8deg    = Q.Part6_8deg;
    this->Part7_8deg    = Q.Part7_8deg;
    this->Part8_8deg    = Q.Part8_8deg;

    this->Part0_2deg    = Q.Part0_2deg;
    this->Part1_2deg    = Q.Part1_2deg;
    this->Part2_2deg    = Q.Part2_2deg;

    this->AnchorTime_4deg   = Q.AnchorTime_4deg;
    this->AnchorTime_8deg   = Q.AnchorTime_8deg;
    this->AnchorTime_2deg   = Q.AnchorTime_2deg;
    this->TimeChange        = Q.TimeChange;
    return *this;
}

Polynoms_data::Polynoms_data (const Polynoms_data &Q)
{
    this->Part0_4deg    = Q.Part0_4deg;
    this->Part1_4deg    = Q.Part1_4deg;
    this->Part2_4deg    = Q.Part2_4deg;
    this->Part3_4deg    = Q.Part3_4deg;
    this->Part4_4deg    = Q.Part4_4deg;

    this->Part0_8deg    = Q.Part0_8deg;
    this->Part1_8deg    = Q.Part1_8deg;
    this->Part2_8deg    = Q.Part2_8deg;
    this->Part3_8deg    = Q.Part3_8deg;
    this->Part4_8deg    = Q.Part4_8deg;
    this->Part5_8deg    = Q.Part5_8deg;
    this->Part6_8deg    = Q.Part6_8deg;
    this->Part7_8deg    = Q.Part7_8deg;
    this->Part8_8deg    = Q.Part8_8deg;

    this->Part0_2deg    = Q.Part0_2deg;
    this->Part1_2deg    = Q.Part1_2deg;
    this->Part2_2deg    = Q.Part2_2deg;

    this->AnchorTime_4deg   = Q.AnchorTime_4deg;
    this->AnchorTime_8deg   = Q.AnchorTime_8deg;
    this->AnchorTime_2deg   = Q.AnchorTime_2deg;
    this->TimeChange        = Q.TimeChange;
}


EllipseInfo::EllipseInfo()
{
//    Reset();
}


Predict_Info::Predict_Info()
{
//    Reset();
}


void EllipseInfo::Reset()
{
    memset(this, 0, sizeof(EllipseInfo));
}


EllipseInfo& EllipseInfo::operator=(const EllipseInfo &Q)
{
    this->aI    = Q.aI;
    this->bI    = Q.bI;
    this->BetaI = Q.BetaI;
    return *this;

}

EllipseInfo::EllipseInfo (const EllipseInfo &Q)
{
    this->aI    = Q.aI;
    this->bI    = Q.bI;
    this->BetaI = Q.BetaI;
}


bool EllipseInfo::IsZero()
{
    return (fabs(aI) < con_eps2 && fabs(bI) < con_eps2);
}


M10::M10()
{
    Reset();
}


M10::M10(const M10 &Q)
{
    for (qint16 i=0; i<10; i++)
    {
        this->M_info[i] = Q.M_info[i];
    }
}


void M10::Reset()
{
    memset(this, 0, sizeof(M10));
}


M10& M10::operator=(const M10 &Q)
{
    for (qint16 i=0; i<10; i++)
    {
        this->M_info[i] = Q.M_info[i];
    }
    return *this;
}


void M10::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        Log(_pLogFile, "%f %f %f %f %f "
            "%f %f %f %f %f ",
            M_info[0], M_info[1], M_info[2], M_info[3], M_info[4],
            M_info[5], M_info[6], M_info[7], M_info[8], M_info[9]);
    }
}


EAP_Param::EAP_Param()
{
//    Reset();
}


void EAP_Param::Reset()
{
    theta       = 0;
    H           = 0;
    L           = 0;
    V           = 0;
}


EAP_Param& EAP_Param::operator =(const EAP_Param &Q)
{
    this->theta     = Q.theta;
    this->H         = Q.H;
    this->L         = Q.L;
    this->V         = Q.V;
    return *this;
}

EAP_Param::EAP_Param (const EAP_Param &Q)
{
    this->theta     = Q.theta;
    this->H         = Q.H;
    this->L         = Q.L;
    this->V         = Q.V;
}


bool EAP_Param::IsEmpty()
{
    bool bRes = false;
    if (fabs(theta) < con_par_eps && fabs(H) < con_par_eps
            && fabs(L) < con_par_eps && fabs(V) < con_par_eps)
    {
        bRes = true;
    }
    return bRes;
}


void EAP_Param::LogData(FILE *_pLogFile, qint16 BallMark, qint16 Ownership)
{
    if (_pLogFile != nullptr && !this->IsEmpty())
    {
        tLog(_pLogFile, "E P %f %f %f %f %d %d",
             theta, H, L, V, BallMark, Ownership);
    }
}


void Predict_Info::Reset()
{
    tStart = 0;
    tFall = 0;
    StartPoint.clear();
    FallPoint.clear();
    StartEll.Reset();
    FallEll.Reset();
    Pol.Reset();
    D = 0;
    Hapogee = 0;
    arrPoints.Reset();
//    std::vector<GLTrackPointDouble>().swap(ProlTrackFragm);
}


Predict_Info& Predict_Info::operator=(const Predict_Info &Q)
{
    this->tStart        = Q.tStart;
    this->tFall         = Q.tFall;
    this->StartPoint    = Q.StartPoint;
    this->FallPoint     = Q.FallPoint;
    this->StartEll      = Q.StartEll;
    this->FallEll       = Q.FallEll;
    this->Pol           = Q.Pol;
    this->D             = Q.D;
    this->Hapogee       = Q.Hapogee;
    this->Gamma         = Q.Gamma;
    this->arrPoints     = Q.arrPoints;

    return *this;
}

Predict_Info::Predict_Info (const Predict_Info &Q)
{
    this->tStart        = Q.tStart;
    this->tFall         = Q.tFall;
    this->StartPoint    = Q.StartPoint;
    this->FallPoint     = Q.FallPoint;
    this->StartEll      = Q.StartEll;
    this->FallEll       = Q.FallEll;
    this->Pol           = Q.Pol;
    this->D             = Q.D;
    this->Hapogee       = Q.Hapogee;
    this->Gamma         = Q.Gamma;
    this->arrPoints     = Q.arrPoints;
}

void Predict_Info::LogData(FILE *_pLogFile, bool bOut50points, bool bOutPolynom)
{
    if (_pLogFile != nullptr)
    {
        LogStartPoint(_pLogFile);
        tLog(_pLogFile, "FALL POINT  tFall: %f FallPoint: %f %f %f FallEll.: a=%f b=%f Beta=%f",
             tFall, FallPoint.x, FallPoint.y, FallPoint.z, FallEll.aI, FallEll.bI, FallEll.BetaI);
        tLog(_pLogFile, "Distance: %f HApogee: %f Gamma: %f",
             D, Hapogee, Gamma);
        if (bOut50points)
        {
            for (qint16 i=0; i<50; i++)
            {
                if (!arrPoints[i].P.IsZero())
                {
                    tLog(_pLogFile, "Point %d t: %f Coord.: %f %f %f "
                         "Vel: %f %f %f "
                         "Acc: %f %f %f",
                         i, arrPoints[i].time_point, arrPoints[i].P.x, arrPoints[i].P.y, arrPoints[i].P.z,
                         arrPoints[i].V.x, arrPoints[i].V.y, arrPoints[i].V.z,
                         arrPoints[i].A.x, arrPoints[i].A.y, arrPoints[i].A.z);
                }
            }
        }
        if (bOutPolynom)
        {
            Pol.LogData(_pLogFile, "POLYNOM:");
        }
    }
}


void Predict_Info::LogStartPoint(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        tLog(_pLogFile, "START POINT  tStart: %f StartPoint: %f %f %f StartEll.: a=%f b=%f Beta=%f",
             tStart, StartPoint.x, StartPoint.y, StartPoint.z, StartEll.aI, StartEll.bI, StartEll.BetaI);
    }
}


//Trapeze_Parameters::Trapeze_Parameters()
//{
//    Reset();
//}


//void Trapeze_Parameters::Reset()
//{
//    memset(this, 0, sizeof(Trapeze_Parameters));
//    P_base.clear();
//}


Active_Pred_Info::Active_Pred_Info()
{
//    Reset();
}


void Active_Pred_Info::Reset()
{
    New_ActPrognoz = 0;
    Trapeze.Reset();
    EstStartPoint.clear();
    EstFallPoint.clear();
    TimeStartEst = AIR_BALL::Timer_INI;
    NumsThreat.clear();
}


Active_Pred_Info& Active_Pred_Info::operator=(const Active_Pred_Info &Q)
{
    this->New_ActPrognoz    = Q.New_ActPrognoz;
    this->Trapeze           = Q.Trapeze;
    this->EstStartPoint     = Q.EstStartPoint;
    this->EstFallPoint      = Q.EstFallPoint;
    this->TimeStartEst      = Q.TimeStartEst;
    this->NumsThreat.clear();
    this->NumsThreat        = Q.NumsThreat;
    return *this;
}

Active_Pred_Info::Active_Pred_Info (const Active_Pred_Info &Q)
{
    this->New_ActPrognoz    = Q.New_ActPrognoz;
    this->Trapeze           = Q.Trapeze;
    this->EstStartPoint     = Q.EstStartPoint;
    this->EstFallPoint      = Q.EstFallPoint;
    this->TimeStartEst      = Q.TimeStartEst;
    this->NumsThreat.clear();
    this->NumsThreat        = Q.NumsThreat;
}

void Active_Pred_Info::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        Trapeze.LogData(_pLogFile, "Trapeze: ");
        if (NumsThreat.size() > 0)
        {
            char OutString[GL_MATR::MAX_LEN_STR];
            sprintf(OutString, "Threatened objects: ");
            std::set<qint32>::iterator it;
            for (it = NumsThreat.begin(); it != NumsThreat.end(); it++)
            {
                qint32 Len = strlen(OutString);
                if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
                {
                    break;
                }
                char TmpString[GL_MATR::MAX_LEN_STR];
                strcpy(TmpString, OutString);
                sprintf(OutString, "%s %d", TmpString, (*it));
            }
            //tLog(_pLogFile, "%s", OutString);
            fprintf(_pLogFile, "%9.3lf %s\n", CURR_TIME_START_OF_DAY_SEC, OutString);
        }
    }
}


CtrlAIR_BALL_Output_GT_Data::CtrlAIR_BALL_Output_GT_Data()
{
//    Reset();
}


void CtrlAIR_BALL_Output_GT_Data::Reset()
{
    bNewPrediction = false;
    bNewPred_InternalUse = false;
    bNewSP = false;
    bNewPathBranch = false;
    bNewActPred = false;
    bNewTEAP = false;
    bNewSubcl = false;
    bNewTrajType = false;
    bNewABData = false;
    bNewPolynomSat = false;

    Path = 0;
    Branch = 0;
    tEAP = 0;
    TrajType = 0;

    BallSubclass = 0;
    minBallSubclass = 0;
    maxBallSubclass = 0;
    bSubclIsSingleValued = false;
    FlightTimeMax = 0;

    Predict.Reset();
    ActPred.Reset();
    EAP_parameters.Reset();
    EAP_Point.clear();
    PointSepBoost1.clear();
    PointSepBoost2.clear();

    bPossibleMaRV = false;
    bPossibleQBM = false;
    AB_Trapeze.Reset();
}


CtrlAIR_BALL_Output_GT_Data& CtrlAIR_BALL_Output_GT_Data::operator=(const CtrlAIR_BALL_Output_GT_Data &Q)
{
    this->bNewPrediction        = Q.bNewPrediction;
    this->bNewPred_InternalUse  = Q.bNewPred_InternalUse;
    this->bNewSP                = Q.bNewSP;
    this->bNewPathBranch        = Q.bNewPathBranch;
    this->bNewActPred           = Q.bNewActPred;
    this->bNewTEAP              = Q.bNewTEAP;
    this->bNewSubcl             = Q.bNewSubcl;
    this->bNewTrajType          = Q.bNewTrajType;
    this->bNewABData            = Q.bNewABData;
    this->bNewPolynomSat        = Q.bNewPolynomSat;

    this->Path                  = Q.Path;
    this->Branch                = Q.Branch;
    this->tEAP                  = Q.tEAP;
    this->TrajType              = Q.TrajType;

    this->BallSubclass          = Q.BallSubclass;
    this->minBallSubclass       = Q.minBallSubclass;
    this->maxBallSubclass       = Q.maxBallSubclass;
    this->bSubclIsSingleValued  = Q.bSubclIsSingleValued;
    this->FlightTimeMax         = Q.FlightTimeMax;

    this->Predict               = Q.Predict;
    this->ActPred               = Q.ActPred;
    this->EAP_parameters        = Q.EAP_parameters;
    this->EAP_Point             = Q.EAP_Point;
    this->PointSepBoost1        = Q.PointSepBoost1;
    this->PointSepBoost2        = Q.PointSepBoost2;

    this->bPossibleMaRV         = Q.bPossibleMaRV;
    this->bPossibleQBM          = Q.bPossibleQBM;
    this->AB_Trapeze            = Q.AB_Trapeze;

    return *this;
}

CtrlAIR_BALL_Output_GT_Data::CtrlAIR_BALL_Output_GT_Data (const CtrlAIR_BALL_Output_GT_Data &Q)
{
    this->bNewPrediction        = Q.bNewPrediction;
    this->bNewPred_InternalUse  = Q.bNewPred_InternalUse;
    this->bNewSP                = Q.bNewSP;
    this->bNewPathBranch        = Q.bNewPathBranch;
    this->bNewActPred           = Q.bNewActPred;
    this->bNewTEAP              = Q.bNewTEAP;
    this->bNewSubcl             = Q.bNewSubcl;
    this->bNewTrajType          = Q.bNewTrajType;
    this->bNewABData            = Q.bNewABData;
    this->bNewPolynomSat        = Q.bNewPolynomSat;

    this->Path                  = Q.Path;
    this->Branch                = Q.Branch;
    this->tEAP                  = Q.tEAP;
    this->TrajType              = Q.TrajType;

    this->BallSubclass          = Q.BallSubclass;
    this->minBallSubclass       = Q.minBallSubclass;
    this->maxBallSubclass       = Q.maxBallSubclass;
    this->bSubclIsSingleValued  = Q.bSubclIsSingleValued;
    this->FlightTimeMax         = Q.FlightTimeMax;

    this->Predict               = Q.Predict;
    this->ActPred               = Q.ActPred;
    this->EAP_parameters        = Q.EAP_parameters;
    this->EAP_Point             = Q.EAP_Point;
    this->PointSepBoost1        = Q.PointSepBoost1;
    this->PointSepBoost2        = Q.PointSepBoost2;

    this->bPossibleMaRV         = Q.bPossibleMaRV;
    this->bPossibleQBM          = Q.bPossibleQBM;
    this->AB_Trapeze            = Q.AB_Trapeze;
}


bool CtrlAIR_BALL_Output_GT_Data::AnyChanges()
{
    bool bRes = false;
    if (bNewPrediction || bNewPred_InternalUse || bNewSP || bNewPathBranch
            || bNewActPred || bNewTEAP || bNewSubcl || bNewTrajType || bNewABData || bNewPolynomSat)
    {
        bRes = true;
    }
    return bRes;
}


void CtrlAIR_BALL_Output_GT_Data::LogData(FILE *_pLogFile, qint32 NumGT, qint32 ID_GT, qreal tLoc, bool bOut50points, bool bOutPolynom)
{
    if (_pLogFile != nullptr)
    {
        if (AnyChanges())
        {
            tLog(_pLogFile, "===========================================================");
        }
        tLog(_pLogFile, "NumGT: %d ID_GT: %d tLoc: %f "
             "NewPrediction: %d NewPred.internal: %d NewStartPoint: %d NewPathBranch: %d NewActPred: %d "
             "NewT_EAP: %d NewSubclass: %d NewTrajType: %d NewABData: %d",
             NumGT, ID_GT, tLoc,
             bNewPrediction, bNewPred_InternalUse, bNewSP, bNewPathBranch, bNewActPred,
             bNewTEAP, bNewSubcl, bNewTrajType, bNewABData);
        if (bNewPrediction || bNewPred_InternalUse)
        {
            tLog(_pLogFile, "-----PREDICTION----- %s", (!bNewPrediction && bNewPred_InternalUse) ? "(internal)" : "");
            Predict.LogData(_pLogFile, bOut50points, bOutPolynom);
        }
        if (bNewSP)
        {
            tLog(_pLogFile, "-----NEW START POINT-----");
            Predict.LogStartPoint(_pLogFile);
        }
        if (bNewPathBranch)
        {
            tLog(_pLogFile, "-----PATH AND BRANCH-----   Path: %d Branch: %d",
                 Path, Branch);
        }
        if (bNewActPred)
        {
            tLog(_pLogFile, "-----PREDICTION ON ACTIVE PATH-----");
            ActPred.LogData(_pLogFile);
        }
        if (bNewTEAP)
        {
            tLog(_pLogFile, "-----END OF ACTIVE PATH-----");
            tLog(_pLogFile, "tEAP: %f Theta: %f H: %f L: %f V: %f",
                 tEAP, EAP_parameters.theta, EAP_parameters.H, EAP_parameters.L, EAP_parameters.V);
            EAP_Point.tLogData(_pLogFile, "EAP Point:");
            PointSepBoost1.tLogData(_pLogFile, "Booster1 separation point:");
            PointSepBoost2.tLogData(_pLogFile, "Booster2 separation point:");
        }
        if (bNewSubcl)
        {
            tLog(_pLogFile, "-----SUBCLASS-----");
            tLog(_pLogFile, "Single valued: %d Res.subclass: %d Min.subclass: %d Max.subclass: %d Max.flight time: %f",
                 bSubclIsSingleValued, BallSubclass, minBallSubclass, maxBallSubclass, FlightTimeMax);
        }
        if (bNewTrajType)
        {
            tLog(_pLogFile, "-----NEW TRAJECTORY TYPE: %d -----", TrajType);
        }
        if (bNewABData)
        {
            tLog(_pLogFile, "-----AEROBALLISTIC PARAMETERS-----");
            tLog(_pLogFile, "Possible MaRV: %d Possible QBM: %d", bPossibleMaRV, bPossibleQBM);
            AB_Trapeze.LogData(_pLogFile, "AB Trapeze:");
        }
    }
}


CtrlAIR_BALL_Inner_Data_GT::CtrlAIR_BALL_Inner_Data_GT()
{
//    Reset();
}


void CtrlAIR_BALL_Inner_Data_GT::Reset()
{
    t_waiting_begin = AIR_BALL::Timer_INI;
    Class = 0;
}


Str_Ar_EAP_Param_1Table::Str_Ar_EAP_Param_1Table()
{
//    Reset();
}


void Str_Ar_EAP_Param_1Table::Reset()
{
    Items.Reset();
    ItemsBoost1.Reset();
    ItemsBoost2.Reset();
    BallSubclass = 0;
    BallMark_inner = 0;
    t_EAP = 0;
    t_SepBoost1 = 0;
    t_SepBoost2 = 0;
    QuantityBoost = 0;
    Gamma = 0;
    N_el = 0;
    N_el_Boost1 = 0;
    N_el_Boost2 = 0;
}


Str_Ar_EAP_Param_1Table& Str_Ar_EAP_Param_1Table::operator =(const Str_Ar_EAP_Param_1Table &Q)
{
    this->Reset();
    this->Items             = Q.Items;
    this->ItemsBoost1       = Q.ItemsBoost1;
    this->ItemsBoost2       = Q.ItemsBoost2;
    this->BallSubclass      = Q.BallSubclass;
    this->BallMark_inner    = Q.BallMark_inner;
    this->t_EAP             = Q.t_EAP;
    this->t_SepBoost1       = Q.t_SepBoost1;
    this->t_SepBoost2       = Q.t_SepBoost2;
    this->QuantityBoost     = Q.QuantityBoost;
    this->Gamma             = Q.Gamma;
    this->N_el              = Q.N_el;
    this->N_el_Boost1       = Q.N_el_Boost1;
    this->N_el_Boost2       = Q.N_el_Boost2;

    return *this;
}

Str_Ar_EAP_Param_1Table::Str_Ar_EAP_Param_1Table (const Str_Ar_EAP_Param_1Table &Q)
{
    this->Items             = Q.Items;
    this->ItemsBoost1       = Q.ItemsBoost1;
    this->ItemsBoost2       = Q.ItemsBoost2;
    this->BallSubclass      = Q.BallSubclass;
    this->BallMark_inner    = Q.BallMark_inner;
    this->t_EAP             = Q.t_EAP;
    this->t_SepBoost1       = Q.t_SepBoost1;
    this->t_SepBoost2       = Q.t_SepBoost2;
    this->QuantityBoost     = Q.QuantityBoost;
    this->Gamma             = Q.Gamma;
    this->N_el              = Q.N_el;
    this->N_el_Boost1       = Q.N_el_Boost1;
    this->N_el_Boost2       = Q.N_el_Boost2;
}


void Str_Ar_EAP_Param_1Table::AddRow(const EAP_Param &NewRow)
{
    if (N_el >= 0 && N_el < AIR_BALL::N_ELEM_KAU_PARAM)
    {
        N_el ++;
        Items[N_el-1] = NewRow;
    }
}


void Str_Ar_EAP_Param_1Table::AddRowBoost1(const EAP_Param &NewRow)
{
    if (N_el_Boost1 >= 0 && N_el_Boost1 < AIR_BALL::N_ELEM_KAU_PARAM)
    {
        N_el_Boost1++;
        ItemsBoost1[N_el_Boost1-1] = NewRow;
    }
}


void Str_Ar_EAP_Param_1Table::AddRowBoost2(const EAP_Param &NewRow)
{
    if (N_el_Boost2 >= 0 && N_el_Boost2 < AIR_BALL::N_ELEM_KAU_PARAM)
    {
        N_el_Boost2++;
        ItemsBoost2[N_el_Boost2-1] = NewRow;
    }
}


void Str_Ar_EAP_Param_1Table::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        if (N_el > 0 && N_el <= AIR_BALL::N_ELEM_KAU_PARAM)
        {            
            tLog(_pLogFile, "E D %d %d %f %f %d %f %f %d",
                 BallSubclass, BallMark_inner, t_EAP, Gamma, N_el, t_SepBoost1, t_SepBoost2, QuantityBoost);
            for (qint16 i=0; i<N_el; i++)
            {
                Items[i].LogData(_pLogFile, BallMark_inner, EAP_Param::EAP_POINT);
            }
            for (qint16 i=0; i<N_el; i++)
            {
                ItemsBoost1[i].LogData(_pLogFile, BallMark_inner, EAP_Param::BOOST1_SEP_POINT);
            }
            for (qint16 i=0; i<N_el; i++)
            {
                ItemsBoost2[i].LogData(_pLogFile, BallMark_inner, EAP_Param::BOOST2_SEP_POINT);
            }
        }
    }
}


Str_Ar_EAP_Param_AllTables::Str_Ar_EAP_Param_AllTables()
{
//    Reset();
}


void Str_Ar_EAP_Param_AllTables::Reset()
{
    arData.Reset();
    N_el = 0;
}


Str_Ar_EAP_Param_1Table* Str_Ar_EAP_Param_AllTables::getCell(const qint16 BallSubcl)
{
    if (N_el == 0)
    {
        return nullptr;
    }
    for (qint32 i=0; i<N_el; i++)
    {
        if (arData[i].BallSubclass == BallSubcl)
        {
            return &arData[i];
        }
    }
    return nullptr;
}


Str_Ar_EAP_Param_1Table* Str_Ar_EAP_Param_AllTables::getCellMark(const qint16 BallMark)
{
    if (N_el == 0)
    {
        return nullptr;
    }
    for (qint32 i=0; i<N_el; i++)
    {
        if (arData[i].BallMark_inner == BallMark)
        {
            return &arData[i];
        }
    }
    return nullptr;
}


void Str_Ar_EAP_Param_AllTables::AddTable(const Str_Ar_EAP_Param_1Table &Q)
{
    if (0 <= N_el && N_el < AIR_BALL::N_BALL_SUBCL)
    {
        N_el++;
        arData[N_el-1] = Q;
    }
}


void Str_Ar_EAP_Param_AllTables::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr && N_el > 0 && N_el <= AIR_BALL::N_BALL_SUBCL)
    {
        for (qint16 i=0; i<N_el; i++)
        {
            arData[i].LogData(_pLogFile);
        }
    }
}


Str_Ar_BallCoef_Param_1Table::Str_Ar_BallCoef_Param_1Table()
{
//    Reset();
}


void Str_Ar_BallCoef_Param_1Table::Reset()
{
    H_Diapasons.Reset();
    Mach_Diapasons.Reset();
    GammaValues.Reset();
    BallSubclass = 0;
    BallMark_inner = 0;
    N_H_diap = 0;
    N_Mach_diap = 0;
}


Str_Ar_BallCoef_Param_1Table& Str_Ar_BallCoef_Param_1Table::operator =(const Str_Ar_BallCoef_Param_1Table &Q)
{
    this->H_Diapasons       = Q.H_Diapasons;
    this->Mach_Diapasons    = Q.Mach_Diapasons;
    this->GammaValues       = Q.GammaValues;
    this->BallSubclass      = Q.BallSubclass;
    this->BallMark_inner    = Q.BallMark_inner;
    this->N_H_diap          = Q.N_H_diap;
    this->N_Mach_diap       = Q.N_Mach_diap;
    return *this;
}

Str_Ar_BallCoef_Param_1Table::Str_Ar_BallCoef_Param_1Table (const Str_Ar_BallCoef_Param_1Table &Q)
{
    this->H_Diapasons       = Q.H_Diapasons;
    this->Mach_Diapasons    = Q.Mach_Diapasons;
    this->GammaValues       = Q.GammaValues;
    this->BallSubclass      = Q.BallSubclass;
    this->BallMark_inner    = Q.BallMark_inner;
    this->N_H_diap          = Q.N_H_diap;
    this->N_Mach_diap       = Q.N_Mach_diap;
}


void Str_Ar_BallCoef_Param_1Table::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        qint16 i, j;
        char OutString[GL_MATR::MAX_LEN_STR];
        //output H diapasons
        sprintf(OutString, "B H");
        for (i=0; i<AIR_BALL::N_H_DIAP_GAMMA; i++)
        {
            qint32 Len = strlen(OutString);
            if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
            {
                break;
            }
            char TmpString[GL_MATR::MAX_LEN_STR];
            strcpy(TmpString, OutString);
            sprintf(OutString, "%s %f", TmpString, H_Diapasons[i]);
        }
        //tLog(_pLogFile, "%s", OutString);
        fprintf(_pLogFile, "%9.3lf %s\n", CURR_TIME_START_OF_DAY_SEC, OutString);

        //output Mach diapasons
        sprintf(OutString, "B M");
        for (i=0; i<AIR_BALL::N_MACH_DIAP_GAMMA; i++)
        {
            qint32 Len = strlen(OutString);
            if (Len > GL_MATR::MAX_LEN_STR - GL_MATR::STR_UNDENT)
            {
                break;
            }
            char TmpString[GL_MATR::MAX_LEN_STR];
            strcpy(TmpString, OutString);
            sprintf(OutString, "%s %f", TmpString, Mach_Diapasons[i]);
        }
        //tLog(_pLogFile, "%s", OutString);
        fprintf(_pLogFile, "%9.3lf %s\n", CURR_TIME_START_OF_DAY_SEC, OutString);

        for (i=0; i<AIR_BALL::N_MACH_DIAP_GAMMA; i++)
        {
            sprintf(OutString, "B G");
            for (j=0; j<AIR_BALL::N_H_DIAP_GAMMA; i++)
            {
                char TmpString[GL_MATR::MAX_LEN_STR];
                strcpy(TmpString, OutString);
                sprintf(OutString, "%s %f", TmpString, GammaValues[i][j]);
            }
            //tLog(_pLogFile, "%s", OutString);
            fprintf(_pLogFile, "%9.3lf %s\n", CURR_TIME_START_OF_DAY_SEC, OutString);
        }

        tLog(_pLogFile, "B D %d %d %d %d", BallSubclass, BallMark_inner, N_H_diap, N_Mach_diap);
    }
}


Str_Ar_BallCoef_Param_AllTables::Str_Ar_BallCoef_Param_AllTables()
{
//    Reset();
}


void Str_Ar_BallCoef_Param_AllTables::Reset()
{
    arData.Reset();
    N_el = 0;
}


Str_Ar_BallCoef_Param_1Table* Str_Ar_BallCoef_Param_AllTables::getCell(const qint16 BallSubcl)
{
    if (N_el == 0)
    {
        return nullptr;
    }
    for (qint32 i=0; i<N_el; i++)
    {
        if (arData[i].BallSubclass == BallSubcl)
        {
            return &arData[i];
        }
    }
    return nullptr;
}


Str_Ar_BallCoef_Param_1Table* Str_Ar_BallCoef_Param_AllTables::getCellMark(const qint16 BallMark)
{
    if (N_el == 0)
    {
        return nullptr;
    }
    for (qint32 i=0; i<N_el; i++)
    {
        if (arData[i].BallMark_inner == BallMark)
        {
            return &arData[i];
        }
    }
    return nullptr;
}


void Str_Ar_BallCoef_Param_AllTables::AddTable(const Str_Ar_BallCoef_Param_1Table &Q)
{
    if (0 <= N_el && N_el < AIR_BALL::N_BALL_SUBCL)
    {
        N_el ++;
        arData[N_el-1] = Q;
    }
}


void Str_Ar_BallCoef_Param_AllTables::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr && N_el > 0 && N_el < AIR_BALL::N_BALL_SUBCL)
    {
        for (qint16 i=0; i<N_el; i++)
        {
            arData[i].LogData(_pLogFile);
        }
    }
}


AIR_BALL::sCovObj::sCovObj()
{
//    Reset();
}


void AIR_BALL::sCovObj::Reset()
{
    Num = 0;
    Radius = 0;
    CoordGeodez.m_dAltitude = 0;
    CoordGeodez.m_dLatitude = 0;
    CoordGeodez.m_dLongitude = 0;
}


AIR_BALL::sCovObj& AIR_BALL::sCovObj::operator =(const AIR_BALL::sCovObj &Q)
{
    this->Num           = Q.Num;
    this->Radius        = Q.Radius;
    this->CoordGeodez   = Q.CoordGeodez;
    return *this;
}

AIR_BALL::sCovObj::sCovObj (const sCovObj &Q)
{
    this->Num           = Q.Num;
    this->Radius        = Q.Radius;
    this->CoordGeodez   = Q.CoordGeodez;
}

void AIR_BALL::sCovObj::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        tLog(_pLogFile, "C %d %f %f %f %f",
             Num, CoordGeodez.m_dLatitude, CoordGeodez.m_dLongitude, CoordGeodez.m_dAltitude, Radius);
    }
}


AIR_BALL::s_arCovObj::s_arCovObj()
{
//    Reset();
}


void AIR_BALL::s_arCovObj::Reset()
{
    N_obj = 0;
    arCovObj.Reset();
}


//AIR_BALL::s_arCovObj AIR_BALL::s_arCovObj::operator =(const AIR_BALL::s_arCovObj &Q)
//{
//    this->N_obj     = Q.N_obj;
//    this->arCovObj  = Q.arCovObj;
//    return *this;
//}


void AIR_BALL::s_arCovObj::AddCovObj(const sCovObj &CovObj)
{
    if (N_obj >= 0 && N_obj < AIR_BALL::COVERED_OBJ_AMOUNT)
    {
        N_obj++;
        arCovObj[N_obj-1] = CovObj;
    }
}


void AIR_BALL::s_arCovObj::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr && N_obj > 0 && N_obj<=AIR_BALL::COVERED_OBJ_AMOUNT)
    {
        for(qint32 i=0; i<N_obj; i++)
        {
            arCovObj[i].LogData(_pLogFile);
        }
    }
}


AIR_BALL::Str_InpMsg::Str_InpMsg()
{
//    Reset();
}


AIR_BALL::Str_InpMsg& AIR_BALL::Str_InpMsg::operator =(const AIR_BALL::Str_InpMsg &Q)
{
    this->MsgType       = Q.MsgType;
    this->NumbGT_Tgt    = Q.NumbGT_Tgt;
    this->NumbGT_IC     = Q.NumbGT_IC;
    this->ActionPhase   = Q.ActionPhase;
    this->Time_APh      = Q.Time_APh;
    this->Gamma         = Q.Gamma;

    return *this;
}

AIR_BALL::Str_InpMsg::Str_InpMsg (const Str_InpMsg &Q)
{
    this->MsgType       = Q.MsgType;
    this->NumbGT_Tgt    = Q.NumbGT_Tgt;
    this->NumbGT_IC     = Q.NumbGT_IC;
    this->ActionPhase   = Q.ActionPhase;
    this->Time_APh      = Q.Time_APh;
    this->Gamma         = Q.Gamma;
}

void AIR_BALL::Str_InpMsg::Reset()
{
    MsgType = 0;
    NumbGT_Tgt = 0;
    NumbGT_IC = 0;
    ActionPhase = 0;
    Time_APh = AIR_BALL::Timer_INI;
}


void AIR_BALL::Str_InpMsg::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        tLog(_pLogFile, "M %d %d %d %d %f %f",
             MsgType, NumbGT_Tgt, NumbGT_IC, ActionPhase, Time_APh, Gamma);
    }
}


AIR_BALL::Str_InpMsgsBuffer::Str_InpMsgsBuffer()
{
//    Reset();
}


void AIR_BALL::Str_InpMsgsBuffer::Reset()
{
    MsgsAmount = 0;
    MsgsAr.Reset();
}


void AIR_BALL::Str_InpMsgsBuffer::AddMsg(const qint16 MsgType, const qint32 NumGT_Tgt, const qint32 NumGT_IC,
                                         const qint16 ActionPhase, const qreal Time_APh)
{
    {
        if (NumGT_Tgt > 0 /*&& NumGT_Tgt < AIR_BALL::GT_FORM_AMOUNT*/)
        {
            if (MsgType == Str_InpMsg::NEW_ACTION_PHASE)
            {
                if ((ActionPhase >= NO_ACTION) && (ActionPhase <= REASSIGNED))
                {                    
                    if (MsgsAmount < AIR_BALL::INP_MSGS_AMOUNT-1)
                    {
                        MsgsAmount ++;
                        MsgsAr[MsgsAmount-1].MsgType     = MsgType;
                        MsgsAr[MsgsAmount-1].NumbGT_Tgt  = NumGT_Tgt;
                        MsgsAr[MsgsAmount-1].NumbGT_IC   = NumGT_IC;
                        MsgsAr[MsgsAmount-1].ActionPhase = ActionPhase;
                        MsgsAr[MsgsAmount-1].Time_APh    = Time_APh;
                    }
                }
            }
        }
    }
}


void AIR_BALL::Str_InpMsgsBuffer::AddMsg(const qint16 MsgType, const qint32 Num1, const qint32 Num2)
{
    if (Num1 > 0 && Num1 < AIR_BALL::GT_FORM_AMOUNT
            && Num2 > 0 && Num2 < AIR_BALL::GT_FORM_AMOUNT)
    {
        if (MsgsAmount < AIR_BALL::INP_MSGS_AMOUNT-1)
        {
            if (MsgType == Str_InpMsg::SWAP_NUMBERS)
            {
                MsgsAmount++;
                MsgsAr[MsgsAmount-1].MsgType    = MsgType;
                MsgsAr[MsgsAmount-1].NumbGT_Tgt = Num1; //NumbGT_Tgt used instead of Num1
                MsgsAr[MsgsAmount-1].NumbGT_IC  = Num2; //NumbGT_IC used instead of Num2
            }
        }
    }
}


void AIR_BALL::Str_InpMsgsBuffer::AddMsg(const AIR_BALL::Str_InpMsg &Msg)
{
    if (MsgsAmount < AIR_BALL::INP_MSGS_AMOUNT-1)
    {
        MsgsAmount++;
        MsgsAr[MsgsAmount-1] = Msg;
    }
}


void AIR_BALL::Str_InpMsgsBuffer::AddMsgBallCoef(const qint32 NumGT, const qreal Gamma)
{
    if (NumGT > 0 && NumGT < AIR_BALL::GT_FORM_AMOUNT)
    {
        if (MsgsAmount < AIR_BALL::INP_MSGS_AMOUNT-1)
        {
            MsgsAmount++;
            MsgsAr[MsgsAmount-1].MsgType = AIR_BALL::Str_InpMsg::BALL_COEFF;
            MsgsAr[MsgsAmount-1].NumbGT_Tgt = NumGT;
            MsgsAr[MsgsAmount-1].Gamma = Gamma;
        }
    }
}


void AIR_BALL::Str_InpMsgsBuffer::LogData(FILE *_pLogFile)
{
    if (_pLogFile != nullptr)
    {
        if (MsgsAmount > 0)
        {
            for (qint16 i=0; i<MsgsAmount; i++)
            {
                MsgsAr[i].LogData(_pLogFile);
            }
        }
    }
}
