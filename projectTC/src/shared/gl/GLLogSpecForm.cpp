#include <stdio.h>
#include "GLLogSpecForm.h"
#include "Constants.h"

void GLFileLog::LogTestData(FILE *_pLogFile, qint32 ID, qreal tLoc, qint16 Class, qint16 Path, const GLVector &Vec, const GLMatrix &Matr)
{
    if (_pLogFile != NULL)
    {
        Log(_pLogFile, "%d %f %f %f %f %f %f %f %f %f %f "
                       "%d %d %f %f %f "
                       "%f %f %f %f %f %f "
                       "%f %f %f %f %f %f %f %f "
                       "%f %f %f %f %f %f %f "
                       "%f %f %f %f %f %f "
                       "%f %f %f %f %f "
                       "%f %f %f %f "
                       "%f %f %f %f %f %f ",
            ID, tLoc, Vec.Vec[0], Vec.Vec[1], Vec.Vec[2], Vec.Vec[3], Vec.Vec[4], Vec.Vec[5], Vec.Vec[6], Vec.Vec[7], Vec.Vec[8],
            Class, Path, sqrt(Matr.M[0][0]), sqrt(Matr.M[1][1]), sqrt(Matr.M[2][2]),
            sqrt(Matr.M[3][3]), sqrt(Matr.M[4][4]), sqrt(Matr.M[5][5]), sqrt(Matr.M[6][6]), sqrt(Matr.M[7][7]), sqrt(Matr.M[8][8]),
            Matr.M[0][1], Matr.M[0][2], Matr.M[0][3], Matr.M[0][4], Matr.M[0][5], Matr.M[0][6], Matr.M[0][7], Matr.M[0][8],
            Matr.M[1][2], Matr.M[1][3], Matr.M[1][4], Matr.M[1][5], Matr.M[1][6], Matr.M[1][7], Matr.M[1][8],
            Matr.M[2][3], Matr.M[2][4], Matr.M[2][5], Matr.M[2][6], Matr.M[2][7], Matr.M[2][8],
            Matr.M[3][4], Matr.M[3][5], Matr.M[3][6], Matr.M[3][7], Matr.M[3][8],
            Matr.M[4][5], Matr.M[4][6], Matr.M[4][7], Matr.M[4][8],
            Matr.M[5][6], Matr.M[5][7], Matr.M[5][8], Matr.M[6][7], Matr.M[6][8], Matr.M[7][8]);
    }
}


qreal GLReadIniValue(const char *_pValueName, FILE *_pIniFile)
{
    qreal ResValue = GL::NO_VALUE;
    if (_pIniFile != NULL)
    {
        char str_[100];
        char buf[1000];
        sprintf(str_, " %s = %%lf", _pValueName);

        fseek(_pIniFile, 0, SEEK_SET);

        while (!feof(_pIniFile))
        {
            if (fscanf(_pIniFile, "%[^\n]\n", buf) > 0)
            {
                if (sscanf(buf, str_, &ResValue) == 1)
                {
                    return ResValue;
                }
            }
        }
    }
    return ResValue;
}


qreal GLReadIniValue(const char *_pValueName, FILE *_pIniFile, FILE *_pLogFile)
{
    qreal ResValue = GLReadIniValue(_pValueName, _pIniFile);
    if (fabs(ResValue - GL::NO_VALUE) < con_eps2)
    {
        if (_pLogFile != NULL)
        {
            GLFileLog::tLog(_pLogFile, "GLReadIniValue(). ERROR reading %s from ini-file, default value is used.", _pValueName);
        }
    }
    return ResValue;
}


void GLReadDoubleIniValue(const char *_pValueName, FILE *_pIniFile, qreal &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = ReadValue;
        }
    }
}


void GLReadDoubleIniValue(const char *_pValueName, FILE *_pIniFile, FILE *_pLogFile, qreal &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile, _pLogFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = ReadValue;
        }
    }
}


void GLReadIntIniValue(const char *_pValueName, FILE *_pIniFile, qint32 &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = static_cast<qint32>(ReadValue);
        }
    }
}


void GLReadIntIniValue(const char *_pValueName, FILE* _pIniFile, FILE* _pLogFile, qint32 &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile, _pLogFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = static_cast<qint32>(ReadValue);
        }
    }
}


void GLReadBoolIniValue(const char *_pValueName, FILE *_pIniFile, bool &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = static_cast<bool>(ReadValue);
        }
    }
}


void GLReadBoolIniValue(const char *_pValueName, FILE *_pIniFile, FILE *_pLogFile, bool &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile, _pLogFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = static_cast<bool>(ReadValue);
        }
    }
}


void GLReadShortIniValue(const char *_pValueName, FILE *_pIniFile, qint16 &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = static_cast<qint16>(ReadValue);
        }
    }
}


void GLReadShortIniValue(const char *_pValueName, FILE *_pIniFile, FILE *_pLogFile, qint16 &Value)
{
    if (_pIniFile != NULL)
    {
        qreal ReadValue = GLReadIniValue(_pValueName, _pIniFile, _pLogFile);
        if (ReadValue != GL::NO_VALUE)
        {
            Value = static_cast<qint16>(ReadValue);
        }
    }
}
