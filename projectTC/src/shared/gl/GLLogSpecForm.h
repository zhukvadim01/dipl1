#ifndef LOGSPECFORM_H
#define LOGSPECFORM_H

#include "GLFileLog.h"
#include "GLMatrix.h"

namespace GLFileLog
{
    // PACKAGE		:   GL
    // FUNCTION 	:   GLFileLog::LogTestData
    // DESCRIPTION	:   Output data for test to the log file
    // INPUTS		:	Specified file; identificator of the track; location time; class; path;
    //              :   vector of coordinate parameters, covariance matrix
    // RETURNS		:	None
    void LogTestData(FILE* _pLogFile, qint32 ID, qreal tLoc, qint16 Class, qint16 Path, const GLVector &Vec, const GLMatrix &Matr);
}

// PACKAGE		:   GL
// FUNCTION 	:   GLReadIniValue
// DESCRIPTION	:   Reading a value of the given name from ini-file
// INPUTS		:	Value name; specified file
// RETURNS		:	Read value, or NO_VALUE if fails
qreal GLReadIniValue(const char *_pValueName, FILE* _pIniFile);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadIniValue
// DESCRIPTION	:   Reading a value of the given name from ini-file, with logging if fails
// INPUTS		:	Value name; specified ini file; specified log file
// RETURNS		:	Read value, or NO_VALUE if fails
qreal GLReadIniValue(const char *_pValueName, FILE* _pIniFile, FILE* _pLogFile);

#define GLDReadIniValue(_Value, _sValueName, _pIniFile)         \
    ReadValue = GLReadIniValue(_sValueName, _pIniFile);         \
    _Value = (ReadValue == GL::NO_VALUE) ? _Value : ReadValue;

#define GLDReadIniValueWithLog(_Value, _sValueName, _pIniFile, _pLogFile)      \
    ReadValue = GLReadIniValue(_sValueName, _pIniFile, _pLogFile);      \
    _Value = (ReadValue == GL::NO_VALUE) ? _Value : ReadValue;

#define GLDWriteIniValue(_Value, _sValueName, _sComment, _pLogFile) \
    Log(_pLogFile, " %-30s = %-6d \t\t // %s", _sValueName, _Value, _sComment);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadDoubleIniValue
// DESCRIPTION	:   Reading a value of the given name having type "double" from ini-file
// INPUTS		:	Value name; specified file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadDoubleIniValue(const char *_pValueName, FILE* _pIniFile, qreal &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadDoubleIniValue
// DESCRIPTION	:   Reading a value of the given name having type "double" from ini-file, with logging if fails
// INPUTS		:	Value name; specified ini file; specified log file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadDoubleIniValue(const char* _pValueName, FILE* _pIniFile, FILE* _pLogFile, qreal &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadIntIniValue
// DESCRIPTION	:   Reading a value of the given name having type "int" from ini-file
// INPUTS		:	Value name; specified file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadIntIniValue(const char *_pValueName, FILE* _pIniFile, qint32 &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadIntIniValue
// DESCRIPTION	:   Reading a value of the given name having type "int" from ini-file, with logging if fails
// INPUTS		:	Value name; specified ini file; specified log file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadIntIniValue(const char* _pValueName, FILE* _pIniFile, FILE* _pLogFile, qint32 &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadBoolIniValue
// DESCRIPTION	:   Reading a value of the given name having type "bool" from ini-file
// INPUTS		:	Value name; specified file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadBoolIniValue(const char *_pValueName, FILE* _pIniFile, bool &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadBoolIniValue
// DESCRIPTION	:   Reading a value of the given name having type "bool" from ini-file, with logging if fails
// INPUTS		:	Value name; specified ini file; specified log file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadBoolIniValue(const char* _pValueName, FILE* _pIniFile, FILE* _pLogFile, bool &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadBoolIniValue
// DESCRIPTION	:   Reading a value of the given name having type "short" from ini-file
// INPUTS		:	Value name; specified file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadShortIniValue(const char *_pValueName, FILE* _pIniFile, qint16 &Value);

// PACKAGE		:   GL
// FUNCTION 	:   GLReadBoolIniValue
// DESCRIPTION	:   Reading a value of the given name having type "short" from ini-file, with logging if fails
// INPUTS		:	Value name; specified ini file; specified log file; reference to the resulting value,
//              :   resulting value remains such as was set before if fail
// RETURNS		:	None
void GLReadShortIniValue(const char* _pValueName, FILE* _pIniFile, FILE* _pLogFile, qint16 &Value);

#endif // LOGSPECFORM_H
