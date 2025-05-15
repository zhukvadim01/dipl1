#ifndef LOAD_INPUT_DATA_H
#define LOAD_INPUT_DATA_H

#include "CtrlAir_Ballist_structs.h"

namespace input_data_dir {
    extern const char* pWorkingFolder;
    extern const char* pViewFolder;
}

class load_input_data
{
public:
    struct input_data
    {
        enum data_type{
            type_ST =1, type_GT = 2, type_MSG =3
        };
        quint8                      type{0};
        CtrlAIR_BALL_Input_ST_Data  data_ST;
        CtrlAIR_BALL_Input_GT_Data  data_GT;
        AIR_BALL::Str_InpMsg        data_MSG;
    };

    using arr_input_data  = std::vector<std::pair<qint64, input_data>>;     // time(msec), ST, GT or MSG data

public:
    load_input_data(const char*, const char*);
    ~load_input_data();

    void ParseRDIDFile(AIR_BALL::s_arCovObj &arrCovObj,
                       Str_Ar_EAP_Param_AllTables &arrEAP_Tables);
    bool ParseInputFile(arr_input_data&);

private:

    FILE *m_pLogGT;

    const int       N_SYMB_IN_ROW = 1200; //maximal number of symbols in the string in the file being read
    const double    SecInDay = 86400;

    char    m_pFileName[100];
    char    m_pFileNameRDID[100];

    double  m_procTime{0};

    CtrlAIR_BALL_Input_GT_Data      m_dataInpGT;
    CtrlAIR_BALL_Input_ST_Data      m_dataInpST;
    AIR_BALL::Str_InpMsg            m_dataMSG;


    bool ParseLine_GT(char* _pLine);
    bool ParseLine_ST(char* _pLine);
    void ParseLine_MSG(char* _pLine);
    void ParseLineRDID(char* _pLine, AIR_BALL::s_arCovObj &arrCovObj,
                       Str_Ar_EAP_Param_AllTables &arrEAP_Tables);

    void ParseLine(char* _pLine, arr_input_data&);
};

#endif // LOAD_INPUT_DATA_H
