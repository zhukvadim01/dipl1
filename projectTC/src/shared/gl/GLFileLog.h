#ifndef GLFILELOG_H
#define GLFILELOG_H

#include <cstdio>
#include <cstdarg>

namespace GLFileLog
{
    //PACKAGE		:GLFileLog
    //FUNCTION		:OpenLog
    //DESCRIPTION	:Open log-file
    //INPUTS		:_LogName - log-file name
    //RETURN		:Pointer to a file variable
    FILE*		OpenLog(const char* _LogName, const char* _FolderName = "_Log");
    FILE*		OpenLog_clean(const char* _LogName, const char* _FolderName = "_Log");

    //PACKAGE		:GLFileLog
    //FUNCTION		:Debug
    //DESCRIPTION	:Debug messages output (to console)
    //INPUTS		:_String - debug message
    //RETURN		:NONE
    void		Debug(const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:tDebug
    //DESCRIPTION	:Debug messages and current time output (to console)
    //INPUTS		:_String - debug message
    //RETURN		:NONE
    void		tDebug(const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:Log
    //DESCRIPTION	:Debug messages output (to registration file)
    //INPUTS		:_String - debug message
    //RETURN		:NONE
    void		Log(const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:tLog
    //DESCRIPTION	:Debug messages and current time output (to registration file)
    //INPUTS		:_String - debug message
    //RETURN		:NONE
    void		tLog(const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:dLog
    //DESCRIPTION	:Debug messages and current data and time output (to registration file)
    //INPUTS		:_String - debug message
    //RETURN		:NONE
    void		dLog(const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:DebugLog
    //DESCRIPTION	:Debug messages output (to console and registration file)
    //INPUTS		:_String - debug message
    //RETURN		:NONE
    void		DebugLog(const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:Log
    //DESCRIPTION	:Debug messages output (to a specified file)
    //INPUTS		:_pLogFile - specified file
    //				:_String - debug message
    //				:
    //RETURN		:NONE
    void		Log(FILE* _pLogFile, const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:tLog
    //DESCRIPTION	:Debug messages and current time output (to a specified file)
    //INPUTS		:_pLogFile - specified file
    //				:_String - debug message
    //RETURN		:NONE
    void		tLog(FILE* _pLogFile, const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:tLog
    //DESCRIPTION	:Debug messages and current data and time output (to a specified file)
    //INPUTS		:_pLogFile - specified file
    //				:_String - debug message
    //RETURN		:NONE
    void		dLog(FILE* _pLogFile, const char* _String, ...);

    //PACKAGE		:GLFileLog
    //FUNCTION		:DebugLog
    //DESCRIPTION	:Debug messages output (to console and specified file)
    //INPUTS		:_pLogFile - specified file
    //				:_String - debug message
    //RETURN		:NONE
    void		DebugLog(FILE* _pLogFile, const char* _String, ...);


    const int		MAX_LOGS_AMOUNT = 50;

    extern FILE*	pLogFile;					//pointer to RIP.log
    extern FILE*	pLogFiles[MAX_LOGS_AMOUNT];	//pointers to opened logs
    extern int		OpenFilesAmount;			//open registration files quantity

    const int		PRINT_DEBUG_MSG = 1;			//debug messages output attribute (to console)
    extern bool		bDO_LOGGING;				//debug messages output attribute (to registration files)

}

#define DConvertVarArgToDebugStringFileLog              \
        va_list va;                                     \
        va_start(va, _String);                          \
        const int c_size = 1000;                        \
        char DebugString[c_size];                       \
        if( vsnprintf(DebugString, c_size, _String, va) > c_size )                          \
            DebugString[c_size -2] = DebugString[c_size -3] = DebugString[c_size -4] = '.'; \
        va_end(va)



#ifdef DAutonom
    #include <cassert>
    #define GL_ASSERT(x)  assert(x)
#else
    #define GL_ASSERT(x)  ((x) ? (void)0 : DebugLog("ASSERT: \"%s\" in %s (%d)",#x,__FILE__,__LINE__))
#endif



//#define DBreak	int a = 0;

#endif // GLFILELOG_H
