#ifndef GLTRAPEZE_H
#define GLTRAPEZE_H

#include "GLGeometry.h"
#include "GLFileLog.h"

//PACKAGE       :   GL
//CLASS         :   GLTrapeze
//DESCRIPTION   :   Parameters of "trapeze" formed by 2 rays and 2 sectors of circles
class GLTrapeze
{
public:
    GLTrapeze();

    GLTrapeze(const GLTrapeze &Q);

    GLPointDouble3D P_base;     //the base point from which issues the rays (lateral sides of "trapeze"), in ECEF
    qreal           AzFly{0.};		//azimuth of flying direction (relative to base point) (degrees)
    qreal           SigmaAzFly{0.};	//RMSE of azimuth (degrees)
    qreal           SStrAzFly{0.};	//Size (1/2) of strobe by azimuth (degrees; 3*Sigma+DeltaCorr)
    qreal           MinRange{0.};	//minimal flying range (m)
    qreal           MaxRange{0.};	//maximal flying range (m)

    //PACKAGE		:  GL
    //FUNCTION		:  GLTrapeze::Reset
    //DESCRIPTION	:  Reset class attributes
    //INPUTS		:  None
    //RETURNS		:  None
    void Reset();

    //PACKAGE		:  GL
    //FUNCTION		:  GLTrapeze::operator=
    //DESCRIPTION	:  Assignming operator
    //INPUTS		:  Assignming data
    //RETURNS		:  Result of assignment
    GLTrapeze& operator=(const GLTrapeze &Q);

    //PACKAGE		:  GL
    //FUNCTION		:  GLTrapeze::IntersectsWithCircle
    //DESCRIPTION	:  Checks whether current trapeze intersects with the given circle
    //INPUTS		:  Center of circle in ECEF; radius of circle
    //RETURNS		:  True if current trapeze intersects with circle
    bool IntersectsWithCircle(GLPointDouble3D &Center, qreal R);

    //PACKAGE		:  GL
    //FUNCTION		:  GLTrapeze::IsEmpty
    //DESCRIPTION	:  Checks whether data is empty
    //INPUTS		:  None
    //RETURNS		:  True if data is empty
    bool IsEmpty();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLTrapeze::LogData()
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; name of string
    // RETURNS		:	None
    void LogData(FILE* _pLogFile, const char* _Name);

private:
    //PACKAGE		:  GL
    //FUNCTION		:  GLTrapeze::CalcDistanceToCircleInDir
    //DESCRIPTION	:  Calculates distance on surface from (0;0) to the circle in given direction,
    //              :  determined by azimuth
    //INPUTS		:  Given azimuth relative to (0;0) (radians); distance to the center of the circle from (0;0);
    //              :  azimuth of the center of the circle relative to (0;0) (radians); radius of the circle;
    //              :  reference to the resulting minimum distance; reference to the resulting maximum distance
    //RETURNS		:  True if result is ok
    bool CalcDistanceToCircleInDir(qreal Az, qreal Dc, qreal Az_c, qreal R,
                                   qreal &MinDist, qreal &MaxDist);
};

#endif // GLTRAPEZE_H
