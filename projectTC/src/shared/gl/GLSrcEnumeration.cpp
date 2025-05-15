#include "GLSrcEnumeration.h"

GLSrcEnumeration::GLSrcEnumeration()
{
//    Reset();
}


GLSrcEnumeration::GLSrcEnumeration(qint32 _ST_Amount)
{
    Reset(_ST_Amount);
}


void GLSrcEnumeration::Reset()
{
    ST_Form_Amount = 0;
    CurrMaxInnerNum = -1;
    TrNumCorresp.clear();
    InnerNumValues.clear();
    bLooped = false;
}


void GLSrcEnumeration::Reset(qint32 _ST_Amount)
{
    Reset();
    ST_Form_Amount = _ST_Amount;
}


qint32 GLSrcEnumeration::GetInnerTrackNumber(qint32 NtrSrc)
{
    qint32 InnerNtr = -1;
    if (NtrSrc >= 0 && ST_Form_Amount > 0)
    {
        bool bExists = false; //true if the inner number exists
        std::map<qint32, qint32>::iterator it = TrNumCorresp.find(NtrSrc);
        if (it != TrNumCorresp.end())
        {
            InnerNtr = it->second;
            if (CheckInnerNum(InnerNtr))
            {
                bExists = true;
            }
        }

        if (!bExists)
        {
            bool bFound = false; //true if a new number is found
            CurrMaxInnerNum ++;

            if (!CheckInnerNum(CurrMaxInnerNum))
            {
                bLooped = true;
                CurrMaxInnerNum = 0;
            }
            else
            {
                if (!bLooped)
                {
                    bFound = true;
                }
            }


            if (bLooped)
            {                
                qint32 Counter = 0;
                while (!bFound && Counter < ST_Form_Amount)
                {
                    Counter ++;
                    std::set<qint32>::iterator it_inner = InnerNumValues.find(CurrMaxInnerNum);
                    if (it_inner == InnerNumValues.end())
                    {                        
                        bFound = true;
                    }
                    else
                    {
                        CurrMaxInnerNum++;
                        if (!CheckInnerNum(CurrMaxInnerNum))
                        {
                            CurrMaxInnerNum = 0;
                            std::set<qint32>::iterator it_inner2 = InnerNumValues.find(CurrMaxInnerNum);
                            if (it_inner2 == InnerNumValues.end())
                            {
                                bFound = true;
                            }
                            else
                            {
                                CurrMaxInnerNum++;
                            }
                        }
                    }
                }
            }
            if (bFound)
            {
                InnerNtr = CurrMaxInnerNum;
                TrNumCorresp.insert(std::pair<qint32, qint32>(NtrSrc, InnerNtr));
                InnerNumValues.insert(InnerNtr);
            }
        }
    }
    return InnerNtr;
}


qint32 GLSrcEnumeration::GetInnerTrackNumber_NotAssign(qint32 NtrSrc)
{
    qint32 InnerNtr = -1;
    if (NtrSrc >= 0 && ST_Form_Amount > 0)
    {
        std::map<qint32, qint32>::iterator it = TrNumCorresp.find(NtrSrc);
        if (it != TrNumCorresp.end())
        {
            InnerNtr = it->second;
        }
    }
    return InnerNtr;
}


void GLSrcEnumeration::DeleteTrack(qint32 NtrSrc)
{
    std::map<qint32, qint32>::iterator it_search = TrNumCorresp.find(NtrSrc);
    if (it_search != TrNumCorresp.end())
    {
        qint32 InnerNtr = it_search->second;
        TrNumCorresp.erase(it_search);
        std::set<qint32>::iterator it_inner = InnerNumValues.find(InnerNtr);
        if (it_inner != InnerNumValues.end())
        {
            InnerNumValues.erase(it_inner);
        }
    }
}


bool GLSrcEnumeration::CheckInnerNum(qint32 InnerNum)
{
    bool bCorrect = false;
    if (0 <= InnerNum && InnerNum < ST_Form_Amount)
    {
        bCorrect = true;
    }
    return bCorrect;
}


GLInnerEnumeration::GLInnerEnumeration()
{
//    Reset();
}


GLInnerEnumeration::GLInnerEnumeration(qint32 _Tr_Amount)
{
    Reset(_Tr_Amount);
}


void GLInnerEnumeration::Reset()
{
    Forms_Amount = 0;
    CurrMaxInnerNum = 0;
    TrNumCorresp.clear();
    InnerNumValues.clear();
    bLooped = false;
}


void GLInnerEnumeration::Reset(qint32 _Tr_Amount)
{
    Reset();
    Forms_Amount = _Tr_Amount;
}


qint32 GLInnerEnumeration::GetInnerTrackNumber(qint32 NumExt)
{
    qint32 InnerNum = -1;
    if (NumExt > 0 && Forms_Amount > 0)
    {
        bool bExists = false; //true if the inner number exists
        std::map<qint32, qint32>::iterator it = TrNumCorresp.find(NumExt);
        if (it != TrNumCorresp.end())
        {
            InnerNum = it->second;
            if (CheckInnerNum(InnerNum))
            {
                bExists = true;
            }
        }

        if (!bExists)
        {
            bool bFound = false; //true if a new number is found
            CurrMaxInnerNum ++;

            if (!CheckInnerNum(CurrMaxInnerNum))
            {
                bLooped = true;
                CurrMaxInnerNum = 1;
            }
            else
            {
                if (!bLooped)
                {
                    bFound = true;
                }
            }

            if (bLooped)
            {                
                qint32 Counter = 0;
                while (!bFound && Counter < Forms_Amount)
                {
                    Counter ++;
                    std::set<qint32>::iterator it_inner = InnerNumValues.find(CurrMaxInnerNum);
                    if (it_inner == InnerNumValues.end())
                    {                        
                        bFound = true;
                    }
                    else
                    {
                        CurrMaxInnerNum++;
                        if (!CheckInnerNum(CurrMaxInnerNum))
                        {
                            CurrMaxInnerNum = 1;
                            std::set<qint32>::iterator it_inner2 = InnerNumValues.find(CurrMaxInnerNum);
                            if (it_inner2 == InnerNumValues.end())
                            {
                                bFound = true;
                            }
                            else
                            {
                                CurrMaxInnerNum++;
                            }
                        }
                    }
                }
            }
            if (bFound)
            {
                InnerNum = CurrMaxInnerNum;
                TrNumCorresp.insert(std::pair<qint32, qint32>(NumExt, InnerNum));
                InnerNumValues.insert(InnerNum);
            }
        }
    }
    return InnerNum;
}


qint32 GLInnerEnumeration::GetInnerTrackNumber_NotAssign(qint32 NumExt)
{
    qint32 InnerNum = -1;
    if (NumExt > 0 && Forms_Amount > 0)
    {
        std::map<qint32, qint32>::iterator it = TrNumCorresp.find(NumExt);
        if (it != TrNumCorresp.end())
        {
            InnerNum = it->second;
        }
    }
    return InnerNum;
}


void GLInnerEnumeration::DeleteTrack(qint32 NumExt)
{
    std::map<qint32, qint32>::iterator it_search = TrNumCorresp.find(NumExt);
    if (it_search != TrNumCorresp.end())
    {
        qint32 InnerNtr = it_search->second;
        TrNumCorresp.erase(it_search);
        std::set<qint32>::iterator it_inner = InnerNumValues.find(InnerNtr);
        if (it_inner != InnerNumValues.end())
        {
            InnerNumValues.erase(it_inner);
        }
    }
}


bool GLInnerEnumeration::CheckInnerNum(qint32 InnerNum)
{
    bool bCorrect = false;
    if (0 < InnerNum && InnerNum < Forms_Amount)
    {
        bCorrect = true;
    }
    return bCorrect;
}
