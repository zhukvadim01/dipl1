#ifndef GLSRCENUMERATION_H
#define GLSRCENUMERATION_H

#include <map>
#include <set>

#include <QtGlobal>

// PACKAGE		:   GL
// STRUCTURE 	:   GLSrcEnumeration
// DESCRIPTION	:   Inner tracks enumeration for the source
struct GLSrcEnumeration
{
    qint32     ST_Form_Amount{0}; //total quantity of single tracks
    qint32     CurrMaxInnerNum{-1}; //current maximum value of the inner track number
    std::map<qint32, qint32> TrNumCorresp; //correspondence between the input and inner track numbers
    std::set<qint32> InnerNumValues; //set of busy inner track numbers
    bool bLooped{false}; //true if counter is looped

    GLSrcEnumeration();
    GLSrcEnumeration(qint32 _ST_Amount);

    void Reset();
    void Reset(qint32 _ST_Amount);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSrcEnumeration::GetInnerTrackNumber()
    // DESCRIPTION	:   Returns the inner track number for the given track number from source
    // INPUTS		:	Given track number from the source
    // RETURNS		:	Inner track number
    qint32 GetInnerTrackNumber(qint32 NtrSrc);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSrcEnumeration::GetInnerTrackNumber_NotAssign()
    // DESCRIPTION	:   Returns the inner track number for the given track number from source;
    //              :   does not assign a new number if the number does not exist
    // INPUTS		:	Given track number from the source
    // RETURNS		:	Inner track number
    qint32 GetInnerTrackNumber_NotAssign(qint32 NtrSrc);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSrcEnumeration::DeleteTrack()
    // DESCRIPTION	:   Deletes the track having the given number
    // INPUTS		:	Given track number from the source
    // RETURNS		:	None
    void DeleteTrack(qint32 NtrSrc);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSrcEnumeration::CheckInnerNum()
    // DESCRIPTION	:   Checks the inner track number
    // INPUTS		:	Inner track number
    // RETURNS		:	True if the inner track number is correct
    bool CheckInnerNum(qint32 InnerNum);
};


// PACKAGE		:   GL
// STRUCTURE 	:   GLInnerEnumeration
// DESCRIPTION	:   Inner tracks enumeration
struct GLInnerEnumeration
{
    qint32     Forms_Amount{0}; //total quantity of tracks
    qint32     CurrMaxInnerNum{0}; //current maximum value of the inner track number
    std::map<qint32, qint32> TrNumCorresp; //correspondence between the input and inner track numbers
    std::set<qint32> InnerNumValues; //set of busy inner track numbers
    bool bLooped{false}; //true if counter is looped

    GLInnerEnumeration();
    GLInnerEnumeration(qint32 _Tr_Amount);

    void Reset();
    void Reset(qint32 _Tr_Amount);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLInnerEnumeration::GetInnerTrackNumber()
    // DESCRIPTION	:   Returns the inner track number for the given external track number
    // INPUTS		:	Given track number from the source
    // RETURNS		:	Inner track number
    qint32 GetInnerTrackNumber(qint32 NumExt);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLInnerEnumeration::GetInnerTrackNumber_NotAssign()
    // DESCRIPTION	:   Returns the inner track number for the given external track number;
    //              :   does not assign a new number if the number does not exist
    // INPUTS		:	Given track number from the source
    // RETURNS		:	Inner track number
    qint32 GetInnerTrackNumber_NotAssign(qint32 NumExt);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLInnerEnumeration::DeleteTrack()
    // DESCRIPTION	:   Deletes the track having the given number
    // INPUTS		:	Given external track number
    // RETURNS		:	None
    void DeleteTrack(qint32 NumExt);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLInnerEnumeration::CheckInnerNum()
    // DESCRIPTION	:   Checks the inner track number
    // INPUTS		:	Inner track number
    // RETURNS		:	True if the inner track number is correct
    bool CheckInnerNum(qint32 InnerNum);
};

#endif // GLSRCENUMERATION_H
