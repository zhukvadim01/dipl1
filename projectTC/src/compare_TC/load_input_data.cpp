#include "load_input_data.h"

#include <iostream>
#include <stdlib.h>
#include <cstring>

using namespace GLFileLog;

load_input_data::load_input_data(const char* data_file, const char* reference_file)
{
    strcpy(m_pFileName, data_file);
    strcpy(m_pFileNameRDID, reference_file);

    m_pLogGT = OpenLog("InputGT", input_data_dir::pViewFolder);
}

load_input_data::~load_input_data()
{
    if( m_pLogGT ) {
        fclose(m_pLogGT);
    }
}

bool load_input_data::ParseLine_GT(char* _pLine)
{
    bool to_update{false};
    try
    {
        double TimeProc = 0;
        char Symb1='\0', Symb2='\0';
        sscanf(_pLine, "%lf %c %c", &TimeProc, &Symb1, &Symb2);

        if (Symb1 == 'G')
        {
            if (Symb2 == 'D') //common data
            {
                m_dataInpGT.Reset();

                qint16 bSignQuickR = 0;
                sscanf(_pLine,
                       "%lf %*c %*c TrSign %hd Num %d %lf | %hd %hd %hd %d %d | "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
                       &TimeProc, &m_dataInpGT.TrackingSign, &m_dataInpGT.NumbGT, &m_dataInpGT.tLoc, &m_dataInpGT.Class, &m_dataInpGT.Type, &bSignQuickR, &m_dataInpGT.AssignTgtNum, &m_dataInpGT.ClstNum,
                       &m_dataInpGT.X, &m_dataInpGT.Y, &m_dataInpGT.Z, &m_dataInpGT.VX, &m_dataInpGT.VY, &m_dataInpGT.VZ, &m_dataInpGT.AX, &m_dataInpGT.AY, &m_dataInpGT.AZ,
                       &m_dataInpGT.SigX, &m_dataInpGT.SigY, &m_dataInpGT.SigZ, &m_dataInpGT.SigVX, &m_dataInpGT.SigVY, &m_dataInpGT.SigVZ, &m_dataInpGT.SigAX, &m_dataInpGT.SigAY, &m_dataInpGT.SigAZ,
                       &m_dataInpGT.KXY, &m_dataInpGT.KXZ, &m_dataInpGT.KYZ, &m_dataInpGT.KVxVy, &m_dataInpGT.KVxVz, &m_dataInpGT.KVyVz, &m_dataInpGT.KAxAy, &m_dataInpGT.KAxAz, &m_dataInpGT.KAyAz,
                       &m_dataInpGT.KXVx, &m_dataInpGT.KXVy, &m_dataInpGT.KXVz, &m_dataInpGT.KXAx, &m_dataInpGT.KXAy, &m_dataInpGT.KXAz, &m_dataInpGT.KYVx, &m_dataInpGT.KYVy, &m_dataInpGT.KYVz, &m_dataInpGT.KYAx, &m_dataInpGT.KYAy, &m_dataInpGT.KYAz,
                       &m_dataInpGT.KZVx, &m_dataInpGT.KZVy, &m_dataInpGT.KZVz, &m_dataInpGT.KZAx, &m_dataInpGT.KZAy, &m_dataInpGT.KZAz, &m_dataInpGT.KVxAx, &m_dataInpGT.KVxAy, &m_dataInpGT.KVxAz, &m_dataInpGT.KVyAx, &m_dataInpGT.KVyAy, &m_dataInpGT.KVyAz, &m_dataInpGT.KVzAx, &m_dataInpGT.KVzAy, &m_dataInpGT.KVzAz);
                m_dataInpGT.SignQuickReaction = static_cast<bool>(bSignQuickR);

//                printf("%lf GT\n", TimeProc);
            }
            else if (Symb2 == 'S') //associated single track
            {
                char* pPartLine = _pLine;

                while (*pPartLine != 'S')
                {
                    pPartLine++;
                }

                pPartLine++;
                pPartLine++;

                while(*pPartLine != '\n')
                {
                    try
                    {
                        qint16 SrcInd = -1; //index of source
                        qint32 NumST = -1; //number of single track
                        sscanf(pPartLine, " %hd %d", &SrcInd, &NumST);

                        std::pair<qint16, qint32> CurEl;
                        CurEl.first = SrcInd;
                        CurEl.second = NumST;
                        if (SrcInd >= 0 && SrcInd < AIR_BALL::SRC_AMOUNT) //&& NumST > 0 && NumST < AIR_BALL::ST_SRC_AMOUNT)
                        {
                            m_dataInpGT.SetAssocST.insert(CurEl);
                        }

                        while(*pPartLine != '|' && *pPartLine != '\n')
                        {
                            pPartLine++;
                        }
                        if (*pPartLine != '\n')
                        {
                            pPartLine++;
                        }
                    }
                    catch(...) {
                    }
                }
                m_dataInpGT.LogData(m_pLogGT, true);

                m_procTime = SEC_START_OF_DAY_to_SEC70(TimeProc);
                to_update = true;
            }
            else
            {
            }
        }
    }
    catch(...) {
    }
    return to_update;
}

bool load_input_data::ParseLine_ST(char* _pLine)
{
    bool to_update{false};
    try
    {
        double TimeProc = 0;
        char Symb1='\0', Symb2='\0';
        sscanf(_pLine, "%lf %c %c", &TimeProc, &Symb1, &Symb2);

        if (Symb1 == 'S')
        {
            if (Symb2 == 'D') //common data
            {
                m_dataInpST.Reset();

                qint16 bMeasured=0, bVDopplPresent=0, bSignQuickR=0;
                sscanf(_pLine,
                       "%lf %*c %*c TrSign %hd Src %hd Num %d %d | %lf %hd %hd %hd "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf "
                       "%hd %hd %lf %lf %d",
                       &TimeProc, &m_dataInpST.TrackingSign, &m_dataInpST.SrcInd, &m_dataInpST.NtrInSrc, &m_dataInpST.NumGT, &m_dataInpST.tLoc, &bMeasured, &bVDopplPresent, &bSignQuickR,
                       &m_dataInpST.VectParam.Vec[0], &m_dataInpST.VectParam.Vec[1], &m_dataInpST.VectParam.Vec[2], &m_dataInpST.VectParam.Vec[3], &m_dataInpST.VectParam.Vec[4], &m_dataInpST.VectParam.Vec[5], &m_dataInpST.VectParam.Vec[6], &m_dataInpST.VectParam.Vec[7], &m_dataInpST.VectParam.Vec[8],
                       &m_dataInpST.GammaFlt, &m_dataInpST.R, &m_dataInpST.VDoppl, &m_dataInpST.RMSE_R, &m_dataInpST.RMSE_VDoppl, &m_dataInpST.RMSE_El,
                       &m_dataInpST.Class, &m_dataInpST.Type, &m_dataInpST.Prob_KF_3D_222_small, &m_dataInpST.Prob_KF_3D_222_large, &m_dataInpST.N_StepSmooth);
                m_dataInpST.bMeasured = static_cast<bool>(bMeasured);
                m_dataInpST.bVDopplPresent = static_cast<bool>(bVDopplPresent);
                m_dataInpST.SignQuickReaction = static_cast<bool>(bSignQuickR);

//                printf("%lf ST\n", TimeProc);
            }


            else if (Symb2 == 'C') //covariance matrix
            {
                GLMatrix *pCM = &m_dataInpST.CovMatr;
                sscanf(_pLine,
                       "%lf %*c %*c %d rows %d columns "
                       "%lf %lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf "
                       "%lf %lf %lf %lf "
                       "%lf %lf %lf %lf %lf %lf ",
                       &TimeProc, &pCM->s_m, &pCM->s_n,
                       &pCM->M[0][0], &pCM->M[1][1], &pCM->M[2][2], &pCM->M[3][3], &pCM->M[4][4], &pCM->M[5][5], &pCM->M[6][6], &pCM->M[7][7], &pCM->M[8][8],
                       &pCM->M[0][1], &pCM->M[0][2], &pCM->M[0][3], &pCM->M[0][4], &pCM->M[0][5], &pCM->M[0][6], &pCM->M[0][7], &pCM->M[0][8],
                       &pCM->M[1][2], &pCM->M[1][3], &pCM->M[1][4], &pCM->M[1][5], &pCM->M[1][6], &pCM->M[1][7], &pCM->M[1][8],
                       &pCM->M[2][3], &pCM->M[2][4], &pCM->M[2][5], &pCM->M[2][6], &pCM->M[2][7], &pCM->M[2][8],
                       &pCM->M[3][4], &pCM->M[3][5], &pCM->M[3][6], &pCM->M[3][7], &pCM->M[3][8],
                       &pCM->M[4][5], &pCM->M[4][6], &pCM->M[4][7], &pCM->M[4][8],
                       &pCM->M[5][6], &pCM->M[5][7], &pCM->M[5][8], &pCM->M[6][7], &pCM->M[6][8], &pCM->M[7][8]);

                pCM->ReflectNonZeroRelativeDiag();

                m_procTime = SEC_START_OF_DAY_to_SEC70(TimeProc);
                to_update = true;
            }
            else
            {
            }
        }
    }
    catch(...) {
    }
    return to_update;
}

void load_input_data::ParseLine_MSG(char* _pLine)
{
    try
    {
        double TimeProc = 0;
        char Symb1='\0';
        sscanf(_pLine, "%lf %c", &TimeProc, &Symb1);

        if (Symb1 == 'M')
        {
            AIR_BALL::Str_InpMsg Msg;
            sscanf(_pLine, "%lf M %hd %d %d %hd %lf %lf",
                   &TimeProc, &Msg.MsgType, &Msg.NumbGT_Tgt, &Msg.NumbGT_IC, &Msg.ActionPhase, &Msg.Time_APh, &Msg.Gamma);

            m_procTime = SEC_START_OF_DAY_to_SEC70(TimeProc);
            m_dataMSG = Msg;
        }
    }
    catch(...)
    {
    }
}

void load_input_data::ParseLine(char* _pLine, arr_input_data &arrInputData)
{
    try
    {
        double TimeProc = 0;
        char Symb1='\0';
        sscanf(_pLine, "%lf %c", &TimeProc, &Symb1);

        input_data l_input_data;
        if (Symb1 == 'G') //generalized track
        {
            if( ParseLine_GT(_pLine) ) {
                l_input_data.type   = input_data::type_GT;
                l_input_data.data_GT = m_dataInpGT;
                arrInputData.push_back(std::make_pair(m_procTime *1000, l_input_data));
            }
        }
        else if (Symb1 == 'S') //single track
        {
            if( ParseLine_ST(_pLine) ) {
                l_input_data.type   = input_data::type_ST;
                l_input_data.data_ST = m_dataInpST;
                arrInputData.push_back(std::make_pair(m_procTime *1000, l_input_data));
            }
        }
        else if (Symb1 == 'M') //input message
        {
            ParseLine_MSG(_pLine);
            l_input_data.type   = input_data::type_MSG;
            l_input_data.data_MSG = m_dataMSG;
            arrInputData.push_back(std::make_pair(m_procTime, l_input_data));
        }
        else {
        }
    }
    catch(...)
    {
    }
}

void load_input_data::ParseLineRDID(char* _pLine, AIR_BALL::s_arCovObj &arrCovObj,
                                    Str_Ar_EAP_Param_AllTables &arrEAP_Tables)
{
    try
    {
        double TimeProc = 0;
        char Symb1 = '\0', Symb2 = '\0';
        sscanf(_pLine, "%lf %c", &TimeProc, &Symb1);

        if (Symb1 == 'C') //covered object
        {
            try
            {
                AIR_BALL::sCovObj CovObj;
                sscanf(_pLine, "%lf %*c %d %lf %lf "
                       "%lf %lf",
                       &TimeProc, &CovObj.Num, &CovObj.CoordGeodez.m_dLatitude, &CovObj.CoordGeodez.m_dLongitude,
                       &CovObj.CoordGeodez.m_dAltitude, &CovObj.Radius);
                arrCovObj.AddCovObj(CovObj);
            }
            catch(...) {
            }
        }
        else if (Symb1 == 'E') //end of active path
        {
            sscanf(_pLine, "%lf %c %c", &TimeProc, &Symb1, &Symb2);
            if (Symb2 == 'D') //common data
            {
                Str_Ar_EAP_Param_1Table CurTable;
                sscanf(_pLine, "%lf %*c %*c %hd %hd %lf %lf %hd "
                               "%lf %lf %hd",
                       &TimeProc, &CurTable.BallSubclass, &CurTable.BallMark_inner, &CurTable.t_EAP, &CurTable.Gamma, &CurTable.N_el,
                       &CurTable.t_SepBoost1, &CurTable.t_SepBoost2, &CurTable.QuantityBoost);
                CurTable.N_el = 0;
                arrEAP_Tables.AddTable(CurTable);
            }
            else if (Symb2 == 'P') //parameters at the end of active path
            {
                EAP_Param CurData;
                qint16 CurMark=0, Ownership=0;
                sscanf(_pLine, "%lf %*c %*c %lf %lf %lf %lf %hd %hd",
                       &TimeProc, &CurData.theta, &CurData.H, &CurData.L, &CurData.V, &CurMark, &Ownership);

                Str_Ar_EAP_Param_1Table *pTable = arrEAP_Tables.getCellMark(CurMark);
                switch (Ownership)
                {
                case EAP_Param::EAP_POINT:
                    pTable->AddRow(CurData);
                    break;
                case EAP_Param::BOOST1_SEP_POINT:
                    pTable->AddRowBoost1(CurData);
                    break;
                case EAP_Param::BOOST2_SEP_POINT:
                    pTable->AddRowBoost2(CurData);
                    break;
                default:
                    break;
                }
            }
        }
        else {
        }
    }
    catch(...) {
    }
}

void load_input_data::ParseRDIDFile(AIR_BALL::s_arCovObj &arrCovObj,
                                    Str_Ar_EAP_Param_AllTables &arrEAP_Tables)
{
    try
    {
        char path[200];
        sprintf(path, "%s/%s", input_data_dir::pWorkingFolder, m_pFileNameRDID);

        FILE* rpFileIn = fopen(path, "r");
        if (rpFileIn != NULL)
        {
            char CurLine[N_SYMB_IN_ROW];
            while( fgets(CurLine, N_SYMB_IN_ROW, rpFileIn) )
            {
                ParseLineRDID(CurLine, arrCovObj, arrEAP_Tables/*, arrBallCoeff_Tables*/);
            }
            fclose(rpFileIn);
        }
    }
    catch(...) {
    }
}

bool load_input_data::ParseInputFile(arr_input_data &arrInputData)
{
    try
    {
        char path[200];
        sprintf(path, "%s/%s", input_data_dir::pWorkingFolder, m_pFileName);

        FILE* rpFileIn = fopen(path, "r");
        if ( !rpFileIn ) {
            return false;
        }

        char CurLine[N_SYMB_IN_ROW];
        while (fgets(CurLine, N_SYMB_IN_ROW, rpFileIn) != NULL)
        {
            ParseLine(CurLine, arrInputData);
        }

        fclose(rpFileIn);
        return true;
    }
    catch(...)
    {
        return false;
    }
}


//int main()
//{
//    strcpy(pFileName, "Input_AIR_BALL.log");
//    strcpy(pFileNameRDID, "RD_ID_AIR_BALL.log");

//    g_pCtrlAIR_Ballist = new CtrlAIR_BALLIST();

//    pLogGT = OpenLog("InputGT", pViewFolder);
//    pLogPolynom = OpenLog("PolynomData", pViewFolder);

//    ParseRDIDFile();

//    bool res_parsing = ParseInputFile();
//    cout << res_parsing << endl;

//    fclose(pLogGT);
//    fclose(pLogPolynom);

//    delete g_pCtrlAIR_Ballist;

//    return 0;
//}

