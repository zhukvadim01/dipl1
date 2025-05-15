#include <QDateTime>

#include "GLFileLog.h"

#include <cstdarg>
#include <cstring>


//using namespace GLFileLog;

FILE*	GLFileLog::pLogFile;						// = NULL
FILE*	GLFileLog::pLogFiles[MAX_LOGS_AMOUNT];     // = NULL
int		GLFileLog::OpenFilesAmount = 0;


bool	GLFileLog::bDO_LOGGING		= 1;	// С‡С‚РѕР±С‹ РѕС‚РєР»СЋС‡РёС‚СЊ РІСЃРµ Р»РѕРіРё, РґРѕСЃС‚Р°С‚РѕС‡РЅРѕ СѓРґР°Р»РёС‚СЊ/РїРµСЂРµРёРјРµРЅРѕРІР°С‚СЊ РїР°РїРєСѓ rip_Log

//char* g_pVersion = "";


FILE* GLFileLog::OpenLog(const char *_LogName, const char *_FolderName)
{
    if (!bDO_LOGGING)
        return NULL;

    if (_LogName[0] == 0)
        return NULL;

    char FileName[100];
    sprintf(FileName, "%s/%s.log", _FolderName, _LogName);


    FILE* pLogFile =
#ifdef DRIP_OpenLogFilesForADDING
        fopen(FileName, "r+b");
    if (pLogFile == NULL)
        pLogFile =
#endif
        fopen(FileName, "wb");

    if (pLogFile != NULL)
    {
        if (OpenFilesAmount < MAX_LOGS_AMOUNT)
            pLogFiles[OpenFilesAmount++] = pLogFile;

        fseek(pLogFile, 0, SEEK_END);
        int FileSize = ftell(pLogFile);

        if (FileSize != 0)
            Log(pLogFile, "\r\n\r\n");
//        Log(pLogFile, "===== %s - %s ===== SC4I-2, version %s ====",
//                CRIP_CurTime::ConvDate2Str(CRIP_CurTime::GetDateSys()),
//                CRIP_CurTime::ConvTime2Str(CRIP_CurTime::GetTimeSys()),
//                g_pVersion);
    }

    return pLogFile;
}


FILE* GLFileLog::OpenLog_clean(const char *_LogName, const char *_FolderName)
{
    if (!bDO_LOGGING)
        return NULL;

    if (_LogName[0] == 0)
        return NULL;

    char FileName[100];
    sprintf(FileName, "%s/%s.log", _FolderName, _LogName);


    FILE* pLogFile =
#ifdef DRIP_OpenLogFilesForADDING
        fopen(FileName, "r+b");
    if (pLogFile == NULL)
        pLogFile =
#endif
        fopen(FileName, "wb");

    if (pLogFile != NULL)
    {
        GL_ASSERT(OpenFilesAmount < 40);
        pLogFiles[OpenFilesAmount++] = pLogFile;

        fseek(pLogFile, 0, SEEK_END);
        //int FileSize = ftell(pLogFile);
    }

    return pLogFile;
}


void GLFileLog::Debug(const char* _String, ...)
{
    if (PRINT_DEBUG_MSG == 0)
        return;

    DConvertVarArgToDebugStringFileLog;

    int length = strlen(DebugString);
    if (DebugString[length-1] == '_')		// '_' in the end of string suppresses \r\n
        DebugString[length-1] = 0;
    else
        strcat(DebugString, "\r\n");		// \r\n is added by default

    printf("%s", DebugString);
}



void GLFileLog::tDebug(const char* _String, ...)
{
    if (PRINT_DEBUG_MSG == 0)
        return;

    DConvertVarArgToDebugStringFileLog;

    // Debug("%9.3lf %s", CRIP_CurTime::GetTime(), DebugString);
    Debug("%9.3lf %s", qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., DebugString);
}



void GLFileLog::Log(FILE* _pLogFile, const char* _String, ...)
{
    if (_pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    int length = strlen(DebugString);

    bool bDoCall_fflush = true;
    if (length > 1 && DebugString[length-1] == '_')		// '_' in the end of string suppresses \r\n
    {
        DebugString[length-1] = 0;
        bDoCall_fflush = false;
    }
    else
        strcat(DebugString, "\r\n");		// \r\n is added by default

    fprintf(_pLogFile, "%s", DebugString);

    if (bDoCall_fflush)
        fflush(_pLogFile);
}


void GLFileLog::Log(const char* _String, ...)
{
    if (pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    Log(pLogFile, DebugString);
}



void GLFileLog::tLog(FILE* _pLogFile, const char* _String, ...)
{
    if (_pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    //Log(_pLogFile, "%9.3lf %s", CRIP_CurTime::GetTime(), DebugString);
    Log(_pLogFile, "%9.3lf %s", qreal(QDateTime::currentMSecsSinceEpoch()%86400000) /1000., DebugString);
}

void GLFileLog::tLog(const char* _String, ...)
{
    if (pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    tLog(pLogFile, DebugString);
}

void GLFileLog::dLog(FILE* _pLogFile, const char* _String, ...)
{
    if (_pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    Log(_pLogFile, "%s %9.3lf %s", QDateTime::currentDateTimeUtc().toString("hh:mm:ss.zzz").toLocal8Bit().data(), qreal(QDateTime::currentMSecsSinceEpoch()%86400000)/1000., DebugString);
}

void GLFileLog::dLog(const char* _String, ...)
{
    if (pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    tLog(pLogFile, DebugString);
}



void GLFileLog::DebugLog(FILE* _pLogFile, const char* _String, ...)
{
    if (	PRINT_DEBUG_MSG == 0
        &&	_pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    Debug(DebugString);
    tLog(_pLogFile, DebugString);
}


void GLFileLog::DebugLog(const char* _String, ...)
{
    if (	PRINT_DEBUG_MSG == 0
        &&	pLogFile == NULL)
        return;

    DConvertVarArgToDebugStringFileLog;

    Debug(DebugString);
    tLog(DebugString);
}
