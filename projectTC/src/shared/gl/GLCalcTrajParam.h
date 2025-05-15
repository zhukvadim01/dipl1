#ifndef GLCALCTRAJPARAM_H
#define GLCALCTRAJPARAM_H

#include "conversion/Geocentric_Geo.h"

#include "GLGeometry.h"
#include "GLMatrix.h"
#include "GLSolve_eq_2_3_4_deg.h"

#include <set>

//PACKAGE		:  GL
//CLASS 		:  GLCalcTrajParam
//DESCRIPTION	:  Calculation of different trajectory parameters: height, velocity,
//              :  vertical velocity, engergetic height etc; and RMSEs of these parameters
class GLCalcTrajParam
{
public:
    GLCalcTrajParam();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::Reset
    // DESCRIPTION	:   Cleaning of class attributes
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::Initialize
    // DESCRIPTION	:   Initialization of class attributes
    // INPUTS		:	Coordinates, velocities and accelerations in geocentric coordinate system (ECEF)
    // RETURNS		:	None
    void Initialize(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::Initialize
    // DESCRIPTION	:   Initialization of class attributes
    // INPUTS		:	Vector containing coordinates, velocities and accelerations in ECEF
    // RETURNS		:	None
    void Initialize(GLVector &PosECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVectMagnitude
    // DESCRIPTION	:   Calculation of magnitude (= length, = module, = absolute value) of vector
    // INPUTS		:	Coordinates X, Y, Z of 3D-vector in any rectangular coordinate system
    // RETURNS		:	Magnitude of vector
    double  CalcVectMagnitude(const double &X, const double &Y, const double &Z);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVectMagnitude
    // DESCRIPTION	:   Calculation of magnitude (= length, = module, = absolute value) of vector
    // INPUTS		:	3D-vector in any rectangular coordinate system as GLPointDouble3D
    // RETURNS		:	Magnitude of vector
    double CalcVectMagnitude(GLPointDouble3D &Vect)
    {
        return CalcVectMagnitude(Vect.x, Vect.y, Vect.z);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAbsVelocity
    // DESCRIPTION	:   Calculation of absolute velocity
    // INPUTS		:	Velocity components VX, VY, VZ in any rectangular coordinate system
    // RETURNS		:	Absolute velocity
    double  CalcAbsVelocity(double &VX, double &VY, double &VZ)
    {
        if (!IsInitialized)
        {
            V = CalcVectMagnitude(VX, VY, VZ);
        }
        return V;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAbsVelocity
    // DESCRIPTION	:   Calculation of absolute velocity
    // INPUTS		:	Vector containing coordinates, velocities and accelerations in ECEF
    // RETURNS		:	Absolute velocity
    double CalcAbsVelocity(GLVector &PosECEF)
    {
        if (!IsInitialized)
        {
            V = CalcVectMagnitude(PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5]);
        }
        return V;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAbsVelocity
    // DESCRIPTION	:   Calculation of absolute velocity
    // INPUTS		:	Velocity vector as GLPointDouble3D
    // RETURNS		:	Absolute velocity
    double CalcAbsVelocity(GLPointDouble3D &VelVect)
    {
        if (!IsInitialized)
        {
            V = CalcVectMagnitude(VelVect.x, VelVect.y, VelVect.z);
        }
        return V;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAbsAcceleration
    // DESCRIPTION	:   Calculation of absolute acceleration
    // INPUTS		:	Velocity components VX, VY, VZ in any rectangular coordinate system
    // RETURNS		:	Absolute acceleration
    double  CalcAbsAcceleration(double &AX, double &AY, double &AZ)
    {
        if (!IsInitialized)
        {
            A = CalcVectMagnitude(AX, AY, AZ);
        }
        return A;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAbsAcceleration
    // DESCRIPTION	:   Calculation of absolute acceleration
    // INPUTS		:	Vector containing coordinates, velocities and accelerations in ECEF
    // RETURNS		:	Absolute acceleration
    double CalcAbsAcceleration(GLVector &PosECEF)
    {
        if (!IsInitialized)
        {
            A = CalcVectMagnitude(PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8]);
        }
        return A;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAbsAcceleration
    // DESCRIPTION	:   Calculation of absolute acceleration
    // INPUTS		:	Acceleration vector as GLPointDouble3D
    // RETURNS		:	Absolute acceleration
    double CalcAbsAcceleration(GLPointDouble3D &AccVect)
    {
        if (!IsInitialized)
        {
            A = CalcVectMagnitude(AccVect.x, AccVect.y, AccVect.z);
        }
        return A;
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHeight
    // DESCRIPTION	:   Calculation of height
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m)
    // RETURNS		:	Height (m)
    double  CalcHeight(const double &X, const double &Y, const double &Z);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHeight
    // DESCRIPTION	:   Calculation of height
    // INPUTS		:	Vector containing coordinates in ECEF (0st, 1st and 2st elements, m)
    // RETURNS		:	Height (m)
    double  CalcHeight(GLVector &PosECEF)
    {
        return CalcHeight(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2]);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHeight
    // DESCRIPTION	:   Calculation of height
    // INPUTS		:	Coordinates in ECEF as GLPointDouble3D
    // RETURNS		:	Height (m)
    double CalcHeight(GLPointDouble3D &PosECEF)
    {
        return CalcHeight(PosECEF.x, PosECEF.y, PosECEF.z);
    }

    double CalcHeight(GLPointFloat3D &PosECEF)
    {
        return CalcHeight(static_cast<double>(PosECEF.x), static_cast<double>(PosECEF.y), static_cast<double>(PosECEF.z));
    }

    double CalcHeight(CGeocentric &PosGC)
    {
        return CalcHeight(PosGC.m_dX, PosGC.m_dY, PosGC.m_dZ);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHeight_NUE
    // DESCRIPTION	:   Calculation of height
    // INPUTS		:	Coordinates X, Y, Z in topocentric coordinate system (NUE) (m),
    //              :   height of source position, m
    // RETURNS		:	Height (m)
    double  CalcHeight_NUE(double &X, double &Y, double &Z, double &Hsrc);

    double CalcHeight_NUE(CTopocentric &PosTP, double &Hsrc)
    {
        return CalcHeight_NUE(PosTP.m_dXt, PosTP.m_dYt, PosTP.m_dZt, Hsrc);
    }

    double CalcHeight_NUE(CTopocentric &PosTP, CGeodesic &Center)
    {
        return CalcHeight_NUE(PosTP.m_dXt, PosTP.m_dYt, PosTP.m_dZt, Center.m_dAltitude);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertVel
    // DESCRIPTION	:   Calculation of vertical velocity
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   velocities VX, VY, VZ in ECEF (m/s)
    // RETURNS		:	Vertical velocity (m/s)
    double  CalcVertVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertVel
    // DESCRIPTION	:   Calculation of vertical velocity
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s)
    // RETURNS		:	Vertical velocity (m/s)
    double  CalcVertVel(GLVector &PosECEF)
    {
        return CalcVertVel(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                           PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5]);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertVel
    // DESCRIPTION	:   Calculation of vertical velocity
    // INPUTS		:	Coordinates and velocities in geocentric coordinate system (ECEF) (m), as GLPointDouble3D
    // RETURNS		:	Vertical velocity (m/s)
    double CalcVertVel(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF)
    {
        return CalcVertVel(PosECEF.x, PosECEF.y, PosECEF.z, VelECEF.x, VelECEF.y, VelECEF.z);
    }

    double CalcVertVel(CGeocentric &PosGC, CGeocentric &VelGC)
    {
        return CalcVertVel(PosGC.m_dX, PosGC.m_dY, PosGC.m_dZ, VelGC.m_dX, VelGC.m_dY, VelGC.m_dZ);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertAccel
    // DESCRIPTION	:   Calculation of vertical acceleration
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   accelerations AX, AY, AZ in ECEF (m/s^2)
    // RETURNS		:	Vertical acceleration (m/s^2)
    double  CalcVertAccel(double &X, double &Y, double &Z, double &AX, double &AY, double &AZ);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertAccel
    // DESCRIPTION	:   Calculation of vertical acceleration
    // INPUTS		:	Vector containing coordinates (m), velocities (m/s)
    //              :   and accelerations (m/s^2) in geocentric coordinate system (ECEF)
    // RETURNS		:	Vertical acceleration (m/s^2)
    double  CalcVertAccel(GLVector &PosECEF)
    {
        return CalcVertAccel(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                           PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8]);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertAccel
    // DESCRIPTION	:   Calculation of vertical acceleration
    // INPUTS		:	Coordinates (m) and accelerations (m/s^2) in geocentric coordinate system (ECEF),
    //              :   as GLPointDouble3D
    // RETURNS		:	Vertical acceleration (m/s^2)
    double CalcVertAccel(GLPointDouble3D &PosECEF, GLPointDouble3D &AccECEF)
    {
        return CalcVertAccel(PosECEF.x, PosECEF.y, PosECEF.z, AccECEF.x, AccECEF.y, AccECEF.z);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcLongitudinalAccel
    // DESCRIPTION	:   Calculation of longitudinal acceleration, i.e. absolute value of projection
    //              :   of the acceleration vector to the velocity vector, = time derivative of absolute velocity
    // INPUTS		:	Velocity components in rectangular coordinate system (m),
    //              :   acceleration components in rectangular coordinate system (m/s^2)
    // RETURNS		:	Longitudinal acceleration (m/s^2)
    double CalcLongitudinalAccel(double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ);
    double CalcLongitudinalAccel(GLPointDouble3D &Vel, GLPointDouble3D &Acc);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVertAccelWithoutCoriolis
    // DESCRIPTION	:   Calculation of vertical acceleration with subtraction of Coriolis acceleration
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   velocities VX, VY in ECEF (m/s)
    //              :   accelerations AX, AY, AZ in ECEF (m/s^2)
    // RETURNS		:	Vertical acceleration (m/s^2)
    double  CalcVertAccelWithoutCoriolis(double &X, double &Y, double &Z, double &VX, double &VY, double &AX, double &AY, double &AZ);

    double CalcVertAccelWithoutCoriolis(GLVector &PosECEF)
    {
        return CalcVertAccelWithoutCoriolis(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                            PosECEF.Vec[3], PosECEF.Vec[4],
                                            PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8]);
    }

    double CalcVertAccelWithoutCoriolis(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF, GLPointDouble3D &AccECEF)
    {
        return CalcVertAccelWithoutCoriolis(PosECEF.x, PosECEF.y, PosECEF.z,
                                            VelECEF.x, VelECEF.y,
                                            AccECEF.x, AccECEF.y, AccECEF.z);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHorizVel
    // DESCRIPTION	:   Calculation of horizontal velocity
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   velocities VX, VY, VZ in ECEF (m/s)
    // RETURNS		:	Horizontal velocity (m/s)
    double  CalcHorizVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHorizVel
    // DESCRIPTION	:   Calculation of horizontal velocity
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s)
    // RETURNS		:	Horizontal velocity (m/s)
    double  CalcHorizVel(GLVector &PosECEF)
    {
        return CalcHorizVel(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                           PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5]);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcHorizVel
    // DESCRIPTION	:   Calculation of horizontal velocity
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s) as GLPointDouble3D
    // RETURNS		:	Horizontal velocity (m/s)
    double CalcHorizVel(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF)
    {
        return CalcHorizVel(PosECEF.x, PosECEF.y, PosECEF.z, VelECEF.x, VelECEF.y, VelECEF.z);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAngleVelHorizont
    // DESCRIPTION	:   Calculation of angle between velocity vector and horizon
    // INPUTS		:	Vertical velocity (m/s), absolute velocity (m/s)
    // RETURNS		:	Angle between velocity vector and horizon (radians)
    double CalcAngleVelHorizont(double Vh, double V_absolute);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAngleVelHorizont
    // DESCRIPTION	:   Calculation of angle between velocity vector and horizon
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   velocities VX, VY, VZ in ECEF (m/s)
    // RETURNS		:	Angle between velocity vector and horizon (radians)
    double CalcAngleVelHorizont(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAngleVelHorizont
    // DESCRIPTION	:   Calculation of angle between velocity vector and horizon
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s))
    // RETURNS		:	Angle between velocity vector and horizon (radians)
    double CalcAngleVelHorizont(GLVector &PosECEF)
    {
        return CalcAngleVelHorizont(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5]);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcAngleVelHorizont
    // DESCRIPTION	:   Calculation of angle between velocity vector and horizon
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s) as GLPointDouble3D
    // RETURNS		:	Angle between velocity vector and horizon (radians)
    double CalcAngleVelHorizont(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF)
    {
        return CalcAngleVelHorizont(PosECEF.x, PosECEF.y, PosECEF.z, VelECEF.x, VelECEF.y, VelECEF.z);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of energetic height relative to the Earth surface
    // INPUTS		:	Absolute velocity V (m/s), height H (m)
    // RETURNS		:	Energetic height relative to the Earth surface (m)
    double  CalcEnHeight_EarthSurface(double &V, double &H);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of energetic height relative to the Earth surface
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s)
    // RETURNS		:	Energetic height relative to the Earth surface (m)
    double  CalcEnHeight_EarthSurface(GLVector &PosECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of energetic height relative to the Earth surface
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s) as GLPointDouble3D
    // RETURNS		:	Energetic height relative to the Earth surface (m)
    double CalcEnHeight_EarthSurface(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcEnHeight_EarthCenter
    // DESCRIPTION	:   Calculation of energetic height relative to the Earth center
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   absolute velocity V (m/s)
    // RETURNS		:	Energetic height relative to the Earth center (m)
    double  CalcEnHeight_EarthCenter(double &X, double &Y, double &Z, double &V);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcEnHeight_EarthCenter
    // DESCRIPTION	:   Calculation of energetic height relative to the Earth center
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s)
    // RETURNS		:	Energetic height relative to the Earth center (m)
    double  CalcEnHeight_EarthCenter(GLVector &PosECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcEnHeight_EarthCenter
    // DESCRIPTION	:   Calculation of energetic height relative to the Earth center
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s) as GLPointDouble3D
    // RETURNS		:	Energetic height relative to the Earth center (m)
    double CalcEnHeight_EarthCenter(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVelEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of rate of change of energetic height relative to the Earth surface
    // INPUTS		:	Velocities VX, VY, VZ in ECEF (m/s),
    //              :   accelerations AX, AY, AZ in ECEF (m/s^2),
    //              :   vertical velocity VH (m/s)
    // RETURNS		:	Rate of change of energetic height relative to the Earth surface (m)
    double  CalcVelEnHeight_EarthSurface(double &VX, double &VY, double &VZ, double &AX, double &AY, double &AZ, double &VH);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVelEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of rate of change of energetic height relative to the Earth surface
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2)
    // RETURNS		:	Rate of change of energetic height relative to the Earth surface (m)
    double  CalcVelEnHeight_EarthSurface(GLVector &PosECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcVelEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of rate of change of energetic height relative to the Earth surface
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2) as GLPointDouble3D
    // RETURNS		:	Rate of change of energetic height relative to the Earth surface (m)
    double  CalcVelEnHeight_EarthSurface(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF, GLPointDouble3D &AccECEF);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcDistance
    // DESCRIPTION	:   Calculation of distance between 2 points (in 3 dimensions)
    // INPUTS		:	Coordinates of 1st point, coordinates of 2nd point
    // RETURNS		:	Distance between 2 points
    double  CalcDistance(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcDistance
    // DESCRIPTION	:   Calculation of distance between 2 points (in 3 dimensions)
    // INPUTS		:	Vectors containing coordinates of 1st point and coordinates of 2nd point
    // RETURNS		:	Distance between 2 points
    double CalcDistance(GLVector &P1, GLVector &P2)
    {
        return CalcDistance(P1.Vec[0], P1.Vec[1], P1.Vec[2], P2.Vec[0], P2.Vec[1], P2.Vec[2]);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcDistance
    // DESCRIPTION	:   Calculation of distance between 2 points (in 3 dimensions)
    // INPUTS		:	Structures GLPointDouble3D containing coordinates of 1st point and coordinates of 2nd point
    // RETURNS		:	Distance between 2 points
    double CalcDistance(GLPointDouble3D &P1, GLPointDouble3D &P2)
    {
        return CalcDistance(P1.x, P1.y, P1.z, P2.x, P2.y, P2.z);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcDistanceFromPointToVector
    // DESCRIPTION	:   Calculates distance from the given point to the given vector
    // INPUTS		:	Coordinates of point; coordinates of vector
    // RETURNS		:	Distance between point and vector
    double CalcDistanceFromPointToVector(double PX, double PY, double PZ, double VX, double VY, double VZ);
    double CalcDistanceFromPointToVector(GLPointDouble3D &P, GLPointDouble3D &V);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcMahalanobisDistance
    // DESCRIPTION	:   Calculation of mahalanobis distance
    // INPUTS		:	Vector of observation; vector of mean values of set of observation;
    //              :   covariance matrix
    // RETURNS		:	Mahalanobis distance
    double CalcMahalanobisDistance(GLVector &V1, GLVector &V2, GLMatrix &K);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcMahalanobisDistance
    // DESCRIPTION	:   Calculation of mahalanobis distance
    // INPUTS		:	Vector of observation as GLPointDouble3D; vector of mean values of set
    //              :   of observation as GLPointDouble3D; covariance matrix
    // RETURNS		:	Mahalanobis distance
    double CalcMahalanobisDistance(GLPointDouble3D &P1, GLPointDouble3D &P2, GLMatrix &K);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAngle2Vect
    //DESCRIPTION	: Returns angle between two 3D vectors represented by given coordinates
    //INPUTS		: Coordinates of 1st vector, coordinates of 2nd vector
    //RETURNS		: angle between two vectors (radians)
    double CalcAngle2Vect(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAngle2Vect
    //DESCRIPTION	: Returns angle between two 3D vectors
    //INPUTS		: 1st vector, 2nd vector (3D)
    //RETURNS		: angle between two vectors (radians)
    double CalcAngle2Vect(GLVector &P1, GLVector &P2)
    {
        return CalcAngle2Vect(P1.Vec[0], P1.Vec[1], P1.Vec[2], P2.Vec[0], P2.Vec[1], P2.Vec[2]);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAngle2Vect
    //DESCRIPTION	: Returns angle between two 3D vectors
    //INPUTS		: 1st vector, 2nd vector represented by GLPointDouble3D
    //RETURNS		: angle between two vectors (radians)
    double CalcAngle2Vect(GLPointDouble3D &P1, GLPointDouble3D &P2)
    {
        return CalcAngle2Vect(P1.x, P1.y, P1.z, P2.x, P2.y, P2.z);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAerodynamicAccel
    //DESCRIPTION	: Returns aerodynamic components of acceleration (m/s^2): drag acceleration, lift acceleration, lateral acceleration
    //INPUTS		: Coordinates X,Y,Z (m) in ECEF, velocity components VX,VY,VZ (m/s) in ECEF, acceleration components AX,AY,AZ (m/s^2) in ECEF;
    //              : reference to the resulting absolute value of drag acceleration (m/s^2);
    //              : reference to the resulting absolute value of lift acceleration (m/s^2);
    //              : reference to the resulting absolute value of lateral acceleration (m/s^2)
    //RETURNS		: True if result is OK
    bool CalcAerodynamicAccel(double X, double Y, double Z, double VX, double VY, double VZ, double AX, double AY, double Az,
                              double &ModAccDrag, double &ModAccLift, double &ModAccLateral);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAerodynamicAccel
    //DESCRIPTION	: Returns aerodynamic components of acceleration (m/s^2): drag acceleration, lift acceleration, lateral acceleration
    //INPUTS		: Structures containing coordinates X,Y,Z (m) in ECEF, velocity components VX,VY,VZ (m/s) in ECEF, acceleration components AX,AY,AZ (m/s^2) in ECEF;
    //              : reference to the resulting absolute value of drag acceleration (m/s^2);
    //              : reference to the resulting absolute value of lift acceleration (m/s^2);
    //              : reference to the resulting absolute value of lateral acceleration (m/s^2)
    //RETURNS		: True if result is OK
    bool CalcAerodynamicAccel(GLPointDouble3D Coord, GLPointDouble3D Vel, GLPointDouble3D Accel,
                              double &ModAccDrag, double &ModAccLift, double &ModAccLateral)
    {
        return CalcAerodynamicAccel(Coord.x, Coord.y, Coord.z, Vel.x, Vel.y, Vel.z, Accel.x, Accel.y, Accel.z,
                                    ModAccDrag, ModAccLift, ModAccLateral);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::Calc_g
    //DESCRIPTION	: Calculates gravitational acceleration using distance to the Earth center
    //INPUTS		: Distance to the Earth center, m
    //RETURNS		: Gravitational acceleration, m/s
    double Calc_g(const double &_r);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::Calc_g
    //DESCRIPTION	: Calculates gravitational acceleration for given point in ECEF
    //INPUTS		: Point in ECEF, m
    //RETURNS		: Gravitational acceleration, m/s
    double Calc_g(GLPointDouble3D &_P);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::Calc_g_h
    //DESCRIPTION	: Calculates gravitational acceleration using height
    //INPUTS		: Height, m
    //RETURNS		: Gravitational acceleration, m/s
    double Calc_g_h(const double &_h);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRPergiee
    //DESCRIPTION	: Calculates radius of perigee
    //INPUTS		: Coordinates in the geocentric coordinate system (ECEF);
    //              : velocity components in the geocentric coordinate system;
    //              : reference to the resulting value of perigee radius
    //RETURNS		: True if result is OK
    bool CalcRPergiee(GLPointDouble3D &CoordGC, GLPointDouble3D &VelGC, double &ResRp);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAzVel
    //DESCRIPTION	: Calculates azimuth of velocity vector
    //INPUTS		: Coordinates X, Y, Z in ECEF, m; velocity components VX, VY, VZ in ECEF, m/s;
    //              : reference to the resulting value of azimuth of velocity vector, m/s
    //RETURNS		: True if result is OK
    bool CalcAzVel(double X, double Y, double Z, double VX, double VY, double VZ, double &ResAzVel);

    bool CalcAzVel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, double &ResAzVel);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::GetVector_VertVel
    //DESCRIPTION	: Returns vector of vertical velocity (collinear to (X; Y; Z) in ECEF; as GLPointDouble3D)
    //INPUTS		: Point in ECEF (m); velocity vector in ECEF (m/s)
    //              : reference to the resulting vector of vertical velocity in ECEF (m/s)
    //RETURNS		: True if result is ok
    bool GetVector_VertVel(GLPointDouble3D &P, GLPointDouble3D &V, GLPointDouble3D &ResVertV);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::GetVector_HorizVel
    //DESCRIPTION	: Returns vector of horizontal velocity (normal to (X; Y; Z) in ECEF; as GLPointDouble3D)
    //INPUTS		: Point in ECEF (m); velocity vector in ECEF (m/s)
    //              : reference to the resulting vector of horizontal velocity in ECEF (m/s)
    //RETURNS		: True if result is ok
    bool GetVector_HorizVel(GLPointDouble3D &P, GLPointDouble3D &V, GLPointDouble3D &ResHorizV);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VectMagnitude
    // DESCRIPTION	:   Calculation of RMSE of magnitude of vector
    // INPUTS		:	Vector components X, Y, Z in any rectangular coordinate system,
    //              :   covariance matrix of vector components K
    // RETURNS		:	RMSE of vector magnitude
    double  CalcRMSE_VectMagnitude(double &X, double &Y, double &Z, GLMatrix &K);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VectMagnitude
    // DESCRIPTION	:   Calculation of RMSE of magnitude of vector
    // INPUTS		:	Point containing X, Y, Z in any rectangular coordinate system,
    //              :   covariance matrix of point components K
    // RETURNS		:	RMSE of vector magnitude
    double CalcRMSE_VectMagnitude(GLPointDouble3D P, GLMatrix &K)
    {
        return CalcRMSE_VectMagnitude(P.x, P.y, P.z, K);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_AbsVelocity
    // DESCRIPTION	:   Calculation of RMSE of absolute velocity
    // INPUTS		:	Velocity components VX, VY, VZ in any rectangular coordinate system,
    //              :   covariance matrix of velocity components Kv
    // RETURNS		:	RMSE of absolute velocity
    double  CalcRMSE_AbsVelocity(double &VX, double &VY, double &VZ, GLMatrix &Kv);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_AbsVelocity
    // DESCRIPTION	:   Calculation of RMSE of absolute velocity
    // INPUTS		:	Velocity components VX, VY, VZ in any rectangular coordinate system,
    //              :   covariance matrix of velocity components Kv
    // RETURNS		:	RMSE of absolute velocity
    double  CalcRMSE_AbsVelocity(GLVector &Pos, GLMatrix &Kv)
    {
        return CalcRMSE_AbsVelocity(Pos.Vec[3], Pos.Vec[4], Pos.Vec[5], Kv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VectMagnitude
    // DESCRIPTION	:   Calculation of RMSE of absolute velocity
    // INPUTS		:	Point containing velocity components VX, VY, VZ in any rectangular coordinate system,
    //              :   covariance matrix of velocity components Kv
    // RETURNS		:	RMSE of vector magnitude
    double CalcRMSE_AbsVelocity(GLPointDouble3D P, GLMatrix &K)
    {
        return CalcRMSE_AbsVelocity(P.x, P.y, P.z, K);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Height
    // DESCRIPTION	:   Calculation of RMSE of height
    // INPUTS		:	Coordinates X, Y, Z in geocentric coordinate system (ECEF) (m),
    //              :   covariance matrix of coordinates Kc
    // RETURNS		:	RMSE of height
    double  CalcRMSE_Height(double &X, double &Y, double &Z, GLMatrix &Kc);

    double  CalcRMSE_Height(double &X, double &Y, double &Z, TMatrix<3> Kc)
    {
        GLMatrix Kc10;
        TCMatrixFunc<3> fMatr;
        fMatr.Copy(Kc10, Kc, 0, 2, 0, 2);
        return CalcRMSE_Height(X, Y, Z, Kc10);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Height
    // DESCRIPTION	:   Calculation of RMSE of height
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m),
    //              :   covariance matrix of coordinates Kc
    // RETURNS		:	RMSE of height
    double  CalcRMSE_Height(GLVector &PosECEF, GLMatrix &Kc)
    {
        return CalcRMSE_Height(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2], Kc);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Height
    // DESCRIPTION	:   Calculation of RMSE of height
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m),
    //              :   covariance matrix of coordinates Kc
    // RETURNS		:	RMSE of height
    double  CalcRMSE_Height(GLVector &PosECEF, TMatrix<3> &Kc)
    {
        GLMatrix Kc10;
        TCMatrixFunc<3> fMatr;
        fMatr.Copy(Kc10, Kc, 0, 2, 0, 2);
        return CalcRMSE_Height(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2], Kc10);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Height
    // DESCRIPTION	:   Calculation of RMSE of height
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m),
    //              :   covariance matrix of coordinates Kc
    // RETURNS		:	RMSE of height
    double  CalcRMSE_Height(GLPointDouble3D &PosECEF, GLMatrix &Kc)
    {
        return CalcRMSE_Height(PosECEF.x, PosECEF.y, PosECEF.z, Kc);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Height
    // DESCRIPTION	:   Calculation of RMSE of height
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m),
    //              :   covariance matrix of coordinates Kc
    // RETURNS		:	RMSE of height
    double  CalcRMSE_Height(GLPointDouble3D &PosECEF, TMatrix<3> &Kc)
    {
        GLMatrix Kc10;
        TCMatrixFunc<3> fMatr;
        fMatr.Copy(Kc10, Kc, 0, 2, 0, 2);
        return CalcRMSE_Height(PosECEF.x, PosECEF.y, PosECEF.z, Kc10);
    }

    double CalcRMSE_Height(CGeocentric &PosGC, GLMatrix &Kc)
    {
        return CalcRMSE_Height(PosGC.m_dX, PosGC.m_dY, PosGC.m_dZ, Kc);
    }

    double CalcRMSE_Height(CGeocentric &PosGC, TMatrix<3> &Kc)
    {
        return CalcRMSE_Height(PosGC.m_dX, PosGC.m_dY, PosGC.m_dZ, Kc);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VertVel
    // DESCRIPTION	:   Calculation of RMSE of vertical velocity
    // INPUTS		:	Coordinates X, Y, Z (m) and velocity components VX, VY, VZ (m/s)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of vertical velocity
    double  CalcRMSE_VertVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VertVel
    // DESCRIPTION	:   Calculation of RMSE of vertical velocity
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2)
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of vertical velocity
    double  CalcRMSE_VertVel(GLVector &PosECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_VertVel(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5], Kcv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VertVel
    // DESCRIPTION	:   Calculation of RMSE of vertical velocity
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2)
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of vertical velocity
    double CalcRMSE_VertVel(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_VertVel(PosECEF.x, PosECEF.y, PosECEF.z,
                                VelECEF.x, VelECEF.y, VelECEF.z, Kcv);
    }

    double CalcRMSE_VertVel(CGeocentric &PosGC, CGeocentric &VelGC, GLMatrix &Kcv)
    {
        return CalcRMSE_VertVel(PosGC.m_dX, PosGC.m_dY, PosGC.m_dZ,
                                VelGC.m_dX, VelGC.m_dY, VelGC.m_dZ, Kcv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_HorizVel
    // DESCRIPTION	:   Calculation of RMSE of horizontal velocity
    // INPUTS		:	Coordinates X, Y, Z (m) and velocity components VX, VY, VZ (m/s)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of horizontal velocity
    double  CalcRMSE_HorizVel(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_HorizVel
    // DESCRIPTION	:   Calculation of RMSE of horizontal velocity
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s);
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of horizontal velocity
    double  CalcRMSE_HorizVel(GLVector &PosECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_HorizVel(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5], Kcv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_HorizVel
    // DESCRIPTION	:   Calculation of RMSE of horizontal velocity
    // INPUTS		:	Coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s);
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of horizontal velocity
    double CalcRMSE_HorizVel(GLPointDouble3D &PosECEF, GLPointDouble3D &VelECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_HorizVel(PosECEF.x, PosECEF.y, PosECEF.z,
                                 VelECEF.x, VelECEF.y, VelECEF.z, Kcv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_HorizVel
    // DESCRIPTION	:   Calculation of RMSE of angle between velocity vector and horizon
    // INPUTS		:	Coordinates X, Y, Z (m) and velocity components VX, VY, VZ (m/s)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of angle between velocity vector and horizon (radians)
    double  CalcRMSE_AngleVelHorizont(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_HorizVel
    // DESCRIPTION	:   Calculation of RMSE of angle between velocity vector and horizon
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   and velocities in ECEF (m/s),
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of angle between velocity vector and horizon (radians)
    double  CalcRMSE_AngleVelHorizont(GLVector &PosECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_AngleVelHorizont(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                         PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5], Kcv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_AbsAccel
    // DESCRIPTION	:   Calculation of RMSE of absolute acceleration
    // INPUTS		:	Acceleration components AX, AY, AZ in any rectangular coordinate system,
    //              :   covariance matrix of acceleration components Ka
    // RETURNS		:	RMSE of absolute acceleration
    double  CalcRMSE_AbsAccel(double &AX, double &AY, double &AZ, GLMatrix &Ka)
    {
        return CalcRMSE_VectMagnitude(AX, AY, AZ, Ka);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VertAccel
    // DESCRIPTION	:   Calculation of RMSE of vertical acceleration
    // INPUTS		:	Coordinates X, Y, Z (m) and acceleration components AX, AY, AZ (m/s^2)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates and acceleration components Kca
    // RETURNS		:	RMSE of vertical acceleration
    double  CalcRMSE_VertAccel(double &X, double &Y, double &Z,
                               double &AX, double &AY, double &AZ, GLMatrix &Kca);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VertAccel
    // DESCRIPTION	:   Calculation of RMSE of vertical acceleration
    // INPUTS		:	Vector containing coordinates X, Y, Z (m) and acceleration components AX, AY, AZ (m/s^2)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates and acceleration components Kca
    // RETURNS		:	RMSE of vertical acceleration
    double  CalcRMSE_VertAccel(GLVector &PosECEF, GLMatrix &Kca)
    {
        return CalcRMSE_VertAccel(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                  PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8], Kca);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_LongitudinalAccel
    // DESCRIPTION	:   Calculation of RMSE of longitudinal acceleration
    // INPUTS		:	Velocity components and acceleration components in the rectangular coordinate system;
    //              :   covariance matrix of velocity and acceleration components Kva
    // RETURNS		:	RMSE of longitudinal acceleration
    double  CalcRMSE_LongitudinalAccel(double &VX, double &VY, double &VZ,
                                          double &AX, double &AY, double &AZ, TMatrix<6> &Kva);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_LongitudinalAccel
    // DESCRIPTION	:   Calculation of RMSE of longitudinal acceleration
    // INPUTS		:	Velocity components and acceleration components in the rectangular coordinate system;
    //              :   covariance matrix of coordinates, velocity and acceleration components Kcva
    // RETURNS		:	RMSE of longitudinal acceleration
    double  CalcRMSE_LongitudinalAccel(GLPointDouble3D &Vel, GLPointDouble3D &Acc, GLMatrix &Kcva);
    double  CalcRMSE_LongitudinalAccel(GLPointDouble3D &Vel, GLPointDouble3D &Acc, TMatrix<9> &Kcva);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_EnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of RMSE of energetic height relative to the Earth surface
    // INPUTS		:	Coordinates X, Y, Z (m) and velocity components VX, VY, VZ (m/s)
    //              :   in geocentric coordinate system (ECEF)
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of energetic height relative to the Earth surface (m)
    double  CalcRMSE_EnHeight_EarthSurface(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_EnHeight_EarthSurface
    // DESCRIPTION	:   Calculation of RMSE of energetic height relative to the Earth surface
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2)
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of energetic height relative to the Earth surface (m)
    double  CalcRMSE_EnHeight_EarthSurface(GLVector &PosECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_EnHeight_EarthSurface(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                              PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5], Kcv);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_EnHeight_EarthCenter
    // DESCRIPTION	:   Calculation of RMSE of energetic height relative to the Earth center
    // INPUTS		:	Coordinates X, Y, Z (m) and velocity components VX, VY, VZ (m/s)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of energetic height relative to the Earth center (m)
    double  CalcRMSE_EnHeight_EarthCenter(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, GLMatrix &Kcv);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_EnHeight_EarthCenter
    // DESCRIPTION	:   Calculation of RMSE of energetic height relative to the Earth center
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2)
    //              :   covariance matrix of coordinates and velocities Kcv
    // RETURNS		:	RMSE of energetic height relative to the Earth center (m)
    double  CalcRMSE_EnHeight_EarthCenter(GLVector &PosECEF, GLMatrix &Kcv)
    {
        return CalcRMSE_EnHeight_EarthCenter(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                             PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5], Kcv);
    }


    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VelEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation RMSE of rate of change of energetic height relative to the Earth surface
    // INPUTS		:	Coordinates X, Y, Z (m), velocity components VX, VY, VZ (m/s),
    //              :   acceleration components AX, AY, AZ (m/s^2)
    //              :   in geocentric coordinate system (ECEF),
    //              :   covariance matrix of coordinates, velocities and accererations Kcva
    // RETURNS		:	RMSE of rate of change of energetic height relative to the Earth surface (m)
    double  CalcRMSE_VelEnHeight_EarthSurface(double &X, double &Y, double &Z,
                                              double &VX, double &VY, double &VZ,
                                              double &AX, double &AY, double &AZ, GLMatrix &Kcva);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_VelEnHeight_EarthSurface
    // DESCRIPTION	:   Calculation RMSE of rate of change of energetic height relative to the Earth surface
    // INPUTS		:	Vector containing coordinates in geocentric coordinate system (ECEF) (m)
    //              :   velocities in ECEF (m/s) and accelerations in ECEF (m/s^2)
    //              :   covariance matrix of coordinates, velocities and accererations Kcva
    // RETURNS		:	RMSE of rate of change of energetic height relative to the Earth surface (m)
    double  CalcRMSE_VelEnHeight_EarthSurface(GLVector &PosECEF, GLMatrix &Kcva)
    {
        return CalcRMSE_VelEnHeight_EarthSurface(PosECEF.Vec[0], PosECEF.Vec[1], PosECEF.Vec[2],
                                                 PosECEF.Vec[3], PosECEF.Vec[4], PosECEF.Vec[5],
                                                 PosECEF.Vec[6], PosECEF.Vec[7], PosECEF.Vec[8], Kcva);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Distance
    // DESCRIPTION	:   Calculation RMSE of distance between 2 points (in 3 dimensions)
    // INPUTS		:	Coordinates of 1st point, coordinates of 2nd point,
    //              :   covariance matrix of coordinates of 1st point, covariance matrix of coordinates of 2nd point
    // RETURNS		:	RMSE of distance between 2 points
    double  CalcRMSE_Distance(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, GLMatrix &Kc1, GLMatrix &Kc2);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Distance
    // DESCRIPTION	:   Calculation RMSE of distance between 2 points (in 3 dimensions)
    // INPUTS		:	Vectors containing coordinates of 1st point and coordinates of 2nd point,
    //              :   covariance matrix of coordinates of 1st point, covariance matrix of coordinates of 2nd point
    // RETURNS		:	RMSE of distance between 2 points
    double  CalcRMSE_Distance(GLVector &P1, GLVector &P2, GLMatrix &Kc1, GLMatrix &Kc2)
    {
        return CalcRMSE_Distance(P1.Vec[0], P1.Vec[1], P1.Vec[2], P2.Vec[0], P2.Vec[1], P2.Vec[2], Kc1, Kc2);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Distance
    // DESCRIPTION	:   Calculation RMSE of distance between 2 points (in 3 dimensions)
    // INPUTS		:	Structures GLPointDouble3D containing coordinates of 1st point and coordinates of 2nd point,
    //              :   covariance matrix of coordinates of 1st point, covariance matrix of coordinates of 2nd point
    // RETURNS		:	RMSE of distance between 2 points
    double  CalcRMSE_Distance(GLPointDouble3D &P1, GLPointDouble3D &P2, GLMatrix &Kc1, GLMatrix &Kc2)
    {
        return CalcRMSE_Distance(P1.x, P1.y, P1.z, P2.x, P2.y, P2.z, Kc1, Kc2);
    }

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Distance
    // DESCRIPTION	:   Calculation RMSE difference of distances (D1-D2),
    //              :   where D1 is distance between points (X1;Y1;Z1) and (XC;YC;ZC),
    //              :         D2 is distance between points (X2;Y2;Z2) and (XC;YC;ZC)
    // INPUTS		:	Coordinates of 1st point, coordinates of 2nd point, coordinates of 3rd point "C"
    //              :   covariance matrix of coordinates of 1st point, covariance matrix of coordinates of 2nd point,
    //              :   covariance matrix of coordinates of 3rd point "C"
    // RETURNS		:	RMSE of difference of distances
    double CalcRMSE_Delta_Distance(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2,
                                   double &XC, double &YC, double &ZC, GLMatrix &Kc1, GLMatrix &Kc2, GLMatrix &KcC);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLCalcTrajParam::CalcRMSE_Distance
    // DESCRIPTION	:   Calculation RMSE difference of distances (D1-D2),
    //              :   where D1 is distance between points (X1;Y1;Z1) and (XC;YC;ZC),
    //              :         D2 is distance between points (X2;Y2;Z2) and (XC;YC;ZC)
    // INPUTS		:	Structures containing coordinates of 1st point, coordinates of 2nd point, coordinates of 3rd point "C"
    //              :   covariance matrix of coordinates of 1st point, covariance matrix of coordinates of 2nd point,
    //              :   covariance matrix of coordinates of 3rd point "C"
    // RETURNS		:	RMSE of difference of distances
    double CalcRMSE_Delta_Distance(GLPointDouble3D &P1, GLPointDouble3D &P2, GLPointDouble3D &PC,
                                   GLMatrix &Kc1, GLMatrix &Kc2, GLMatrix &KcC)
    {
        return CalcRMSE_Delta_Distance(P1.x, P1.y, P1.z, P2.x, P2.y, P2.z, PC.x, PC.y, PC.z, Kc1, Kc2, KcC);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_Angle2vec_1stRnd_2ndFixed
    //DESCRIPTION	: Returns RMSE of angle between two 3D vectors represented by given coordinates
    //INPUTS		: Coordinates of 1st vector, coordinates of 2nd vector, covariance matrix of 1st vector, covariance matrix of 2nd vector
    //RETURNS		: RMSE of angle between two vectors (radians)
    double CalcRMSE_Angle2vec(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, GLMatrix &Kc1, GLMatrix &Kc2);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_Angle2vec_1stRnd_2ndFixed
    //DESCRIPTION	: Returns RMSE of angle between two 3D vectors represented by given coordinates
    //INPUTS		: Coordinates of 1st vector, coordinates of 2nd vector, covariance matrix of 1st vector, covariance matrix of 2nd vector
    //RETURNS		: RMSE of angle between two vectors (radians)
    double CalcRMSE_Angle2vec(GLPointDouble3D &P1, GLPointDouble3D &P2, GLMatrix &Kc1, GLMatrix &Kc2)
    {
        return CalcRMSE_Angle2vec(P1.x, P1.y, P1.z, P2.x, P2.y, P2.z, Kc1, Kc2);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_Angle2vec_1stRnd_2ndFixed
    //DESCRIPTION	: Returns RMSE of angle between two 3D vectors represented by given coordinates,
    //              : where 1st vector is normally distributed random and 2nd vector is constant
    //INPUTS		: Coordinates of 1st vector, coordinates of 2nd vector, covariance matrix of 1st vector
    //RETURNS		: RMSE of angle between two vectors (radians)
    double CalcRMSE_Angle2vec_1stRnd_2ndConst(double &X1, double &Y1, double &Z1, double &X2, double &Y2, double &Z2, GLMatrix &Kc1);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_Angle2vec_1stRnd_2ndFixed
    //DESCRIPTION	: Returns RMSE of angle between two 3D vectors represented by given coordinates,
    //              : where 1st vector is normally distributed random and 2nd vector is constant
    //INPUTS		: 1st vector, 2nd vector, covariance matrix of 1st vector
    //RETURNS		: RMSE of angle between two vectors (radians)
    double CalcRMSE_Angle2vec_1stRnd_2ndConst(GLVector &V1, GLVector &V2, GLMatrix &Kc1)
    {
        return CalcRMSE_Angle2vec_1stRnd_2ndConst(V1.Vec[0], V1.Vec[1], V1.Vec[2], V2.Vec[0], V2.Vec[1], V2.Vec[2], Kc1);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_Angle2vec_1stRnd_2ndFixed
    //DESCRIPTION	: Returns RMSE of angle between two 3D vectors represented by given coordinates,
    //              : where 1st vector is normally distributed random and 2nd vector is constant
    //INPUTS		: 1st vector as GLPointDouble3D, 2nd vector as GLPointDouble3D, covariance matrix of 1st vector
    //RETURNS		: RMSE of angle between two vectors (radians)
    double CalcRMSE_Angle2vec_1stRnd_2ndConst(GLPointDouble3D &P1, GLPointDouble3D &P2, GLMatrix &Kc1)
    {
        return CalcRMSE_Angle2vec_1stRnd_2ndConst(P1.x, P1.y, P1.z, P2.x, P2.y, P2.z, Kc1);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_ProjectionVect1OnVect2
    //DESCRIPTION	: Returns RMSE of projection of Vect1 on Vect2
    //INPUTS		: Vectors Vect1, Vect2 represented by GLPointDouble3D;
    //              : covariance matrices of Vect1, Vect2
    //RETURNS		: RMSE of projection of Vect1 on Vect2
    double CalcRMSE_ProjectionVect1OnVect2(GLPointDouble3D &Vect1, GLPointDouble3D &Vect2,
                                              TMatrix<3> &Cov1, TMatrix<3> &Cov2);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcAngle_RMSE_Angle_OP1V1_P2P3
    //DESCRIPTION	: Returns angle (radians) and RMSE of angle (radians)
    //              : between plane (0;0;0)->P1->(P1+V1) and vector P2->P3
    //INPUTS		: point P1; velocity vector V1 starts from the the point P1;
    //              : point P2; point P3; covariance matrices of P1, V1, P2, P3;
    //              : reference to the resulting values of angle (radians) and RMSEs of angle (radians)
    //RETURNS		: true if result is OK
    bool CalcAngle_RMSE_Angle_OP1V1_P2P3(GLPointDouble3D &P1, GLPointDouble3D &V1,
                                         GLPointDouble3D &P2, GLPointDouble3D &P3,
                                         TMatrix<3> &CovP1, TMatrix<3> &CovV1, TMatrix<3> &CovP2, TMatrix<3> &CovP3,
                                         double &ResAngle, double &ResRMSE_Angle);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_AerodynamicAccel
    //DESCRIPTION	: Returns RMSE of aerodynamic components of acceleration (m/s^2): drag acceleration, lift acceleration, lateral acceleration
    //INPUTS		: Coordinates X,Y,Z (m) in ECEF, velocity components VX,VY,VZ (m/s) in ECEF, acceleration components AX,AY,AZ (m/s^2) in ECEF;
    //              : covariance matrix of coordinates, velocity components and acceleration components;
    //              : reference to the resulting RMSE of absolute value of drag acceleration (m/s^2);
    //              : reference to the resulting RMSE of absolute value of lift acceleration (m/s^2);
    //              : reference to the resulting RMSE of absolute value of lateral acceleration (m/s^2)
    //RETURNS		: True if result is OK
    bool CalcRMSE_AerodynamicAccel(double X, double Y, double Z, double VX, double VY, double VZ, double AX, double AY, double Az, GLMatrix &Kcva,
                              double &SigAccDrag, double &SigAccLift, double &SigAccLateral);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_AerodynamicAccel
    //DESCRIPTION	: Returns RMSE of aerodynamic components of acceleration (m/s^2): drag acceleration, lift acceleration, lateral acceleration
    //INPUTS		: Coordinates X,Y,Z (m) in ECEF, velocity components VX,VY,VZ (m/s) in ECEF, acceleration components AX,AY,AZ (m/s^2) in ECEF;
    //              : covariance matrix of coordinates, velocity components and acceleration components;
    //              : reference to the resulting RMSE of absolute value of drag acceleration (m/s^2);
    //              : reference to the resulting RMSE of absolute value of lift acceleration (m/s^2);
    //              : reference to the resulting RMSE of absolute value of lateral acceleration (m/s^2)
    //RETURNS		: True if result is OK
    bool CalcRMSE_AerodynamicAccel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, GLPointDouble3D &Acc, GLMatrix &Kcva,
                                   double &SigAccDrag, double &SigAccLift, double &SigAccLateral);
    bool CalcRMSE_AerodynamicAccel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, GLPointDouble3D &Acc, TMatrix<9> &Kcva,
                                   double &SigAccDrag, double &SigAccLift, double &SigAccLateral);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRPerigee_RMSE_RPergiee
    //DESCRIPTION	: Calculates radius of perigee and its RMSE
    //INPUTS		: Coordinates in the geocentric coordinate system (ECEF);
    //              : velocity components in the geocentric coordinate system;
    //              : covariance matrix;
    //              : reference to the resulting value of perigee radius;
    //              : reference to the resulting value of RMSE of perigee radius
    //RETURNS		: True if result is OK
    bool CalcRPerigee_RMSE_RPergiee(GLPointDouble3D &CoordGC, GLPointDouble3D &VelGC, GLMatrix &Cov, double &ResRp, double &ResRMSERp);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::CalcRMSE_AzVel
    //DESCRIPTION	: Calculates RMSE of the azimuth of velocity vector
    //INPUTS		: Coordinates in the geocentric coordinate system (ECEF);
    //              : velocity components in the geocentric coordinate system;
    //              : covariance matrix of the velocity vector;
    //              : reference to the resulting value of the azimuth of velocity vector (radians);
    //              : reference to the resulting value of the RMSE of azimuth of velocity vector (radians)
    //RETURNS		: True if result is OK
    bool CalcAzVel_RMSE_AzVel(double X, double Y, double Z, double VX, double VY, double VZ, TMatrix<3> &CovV, double &ResAzVel, double &ResSigAz);
    bool CalcAzVel_RMSE_AzVel(GLPointDouble3D &Coord, GLPointDouble3D &Vel, TMatrix<3> &CovV, double &ResAzVel, double &ResSigAz);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::ConvertEllipseToCovMatrNUE
    //DESCRIPTION	: Converts parameter of ellipse to the covariance matrix
    //              : in topocentric coordinate system (NUE) with the center in the given fall point
    //INPUTS		: large semiaxis of dispersion ellipse (without multiplicator 3);
    //              : small semiaxis of dispersion ellipse (without multiplicator 3);
    //              : azimuth of large semiaxis (in radians);
    //              : reference to the resulting matrix, in NUE with the center in the fall point
    //RETURNS		: true if result is OK
    bool ConvertEllipseToCovMatrNUE(double aI, double bI, double betaI, TMatrix<3> &ResCov);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::ConvertEllipseToCovMatrNUE
    //DESCRIPTION	: Converts parameter of ellipse to the covariance matrix
    //              : in topocentric coordinate system (NUE) with the center in the given fall point
    //INPUTS		: large semiaxis of dispersion ellipse (without multiplicator 3);
    //              : small semiaxis of dispersion ellipse (without multiplicator 3);
    //              : azimuth of large semiaxis (in radians);
    //              : reference to the resulting matrix, in NUE with the center in the fall point
    //RETURNS		: true if result is OK
    bool ConvertEllipseToCovMatrNUE(double aI, double bI, double betaI, GLMatrix &ResCov);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::ConvertEllipseToCovMatrECEF
    //DESCRIPTION	: Converts parameter of ellipse to the covariance matrix
    //              : in geocentric coordinate system (ECEF)
    //INPUTS		: large semiaxis of dispersion ellipse (without multiplicator 3);
    //              : small semiaxis of dispersion ellipse (without multiplicator 3);
    //              : azimuth of large semiaxis (in radians);
    //              : coordinates of fall point, in ECEF;
    //              : reference to the resulting matrix, in ECEF
    //RETURNS		: true if result is OK
    bool ConvertEllipseToCovMatrECEF(double aI, double bI, double betaI, GLPointDouble3D &FP, TMatrix<3> &ResCov);
    bool ConvertEllipseToCovMatrECEF(double aI, double bI, double betaI, GLPointDouble3D &FP, GLMatrix &ResCov);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::Weighting2PointsUsingCov
    //DESCRIPTION	: Weighting 2 points in 3D-space using covariance matrices
    //INPUTS		: 1st point; 2nd point; covariance matrices of 1st and 2nd points;
    //              : reference to the resulting point;
    //              : reference to the covariance matrix of resulting point
    //RETURNS		: True if result is OK
    bool Weighting2PointsUsingCov(GLPointDouble3D &P1, GLPointDouble3D &P2,
                                  TMatrix<3> &Cov1, TMatrix<3> &Cov2,
                                  GLPointDouble3D &PRes, TMatrix<3> &CovRes);

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::EstimateTimeEarthSurfaceIntersection
    //DESCRIPTION	: Estimates times (in the form of a complex numbers) of intersection
    //              : of the curve (X;Y;Z)+(VX;VY;VZ)*T+(AX;AY;AZ)*T^2/2 with the Earth surface;
    //              : each of the resulting values is applicable if imaginary part of complex number is zero
    //INPUTS		: Coordinates X,Y,Z (m); velocity components VX,VY,VZ (m/s);
    //              : acceleration components AX,AY,AZ (m/s^2) in ECEF;
    //              : reference to the resulting time values T1c, T2c, T3c, T4c (represented in the form of a complex numbers; s)
    //RETURNS		: None
    void EstimateTimeEarthSurfaceIntersection(const double &X, const double &Y, const double &Z,
                                              const double &VX, const double &VY, const double &VZ,
                                              const double &AX, const double &AY, const double &AZ,
                                              GLComplexNumb &T1c, GLComplexNumb &T2c, GLComplexNumb &T3c, GLComplexNumb &T4c);
    void EstimateTimeEarthSurfaceIntersection(const GLPointDouble3D &_P, const GLPointDouble3D &_V, const GLPointDouble3D &_A,
                                              GLComplexNumb &T1c, GLComplexNumb &T2c, GLComplexNumb &T3c, GLComplexNumb &T4c)
    {
        EstimateTimeEarthSurfaceIntersection(_P.x, _P.y, _P.z, _V.x, _V.y, _V.z, _A.x, _A.y, _A.z, T1c, T2c, T3c, T4c);
    }

    //PACKAGE		: GL
    //FUNCTION		: GLCalcTrajParam::EstimateTimeEarthSurfaceIntersection
    //DESCRIPTION	: Estimates times (in the form of a complex numbers) of intersection
    //              : of the line (X;Y;Z)+(VX;VY;VZ)*T with the Earth surface;
    //              : each of the resulting values is applicable if imaginary part of complex number is zero
    //INPUTS		: Coordinates X,Y,Z (m); velocity components VX,VY,VZ (m/s) in ECEF;
    //              : reference to the resulting time values T1c, T2c (represented in the form of a complex numbers; s)
    //RETURNS		: None
    void EstimateTimeEarthSurfaceIntersection(const double &X, const double &Y, const double &Z,
                                              const double &VX, const double &VY, const double &VZ,
                                              GLComplexNumb &T1c, GLComplexNumb &T2c);
    void EstimateTimeEarthSurfaceIntersection(const GLPointDouble3D &_P, const GLPointDouble3D &_V,
                                              GLComplexNumb &T1c, GLComplexNumb &T2c)
    {
        EstimateTimeEarthSurfaceIntersection(_P.x, _P.y, _P.z, _V.x, _V.y, _V.z, T1c, T2c);
    }

private:
    bool IsInitialized; //true if attributes are initialized

    double r; //magnitude of vector
    double S; //inner parameters needed for calculations
    double Vr; //inner parameters needed for calculations
    double V; //absolute velocity
    double A; //absolute acceleration

    void CalcS(double &Z, double &_r);

    void CalcVr(double &X, double &Y, double &Z, double &VX, double &VY, double &VZ, double &_r);
};


// PACKAGE		:   GL
// CLASS        :   EstimationMedian
// DESCRIPTION	:   Median estimation
class EstimationMedian
{
public:
    EstimationMedian();

    // PACKAGE		:   GL
    // FUNCTION 	:   EstimationMedian::clear()
    // DESCRIPTION	:   Data cleaning
    // INPUTS		:	None
    // RETURNS		:	None
    void clear();

    // PACKAGE		:   GL
    // FUNCTION 	:   EstimationMedian::SetConstants()
    // DESCRIPTION	:   Sets constants
    // INPUTS		:	Pointer to the constants
    // RETURNS		:	None
    void SetConstants(qint16 _DefaultSize, double _DefaultMedian, double _Lambda);

    // PACKAGE		:   GL
    // FUNCTION 	:   EstimationMedian::GetMedian()
    // DESCRIPTION	:   Returns median value
    // INPUTS		:	None
    // RETURNS		:	Median value
    double GetMedian();

    // PACKAGE		:   GL
    // FUNCTION 	:   EstimationMedian::AddValue()
    // DESCRIPTION	:   Adds new value
    // INPUTS		:	New value
    // RETURNS		:	None
    void AddValue(double _NewVal);

protected:
    qint16 Size; //current size
    std::multiset<double> ValuesSet; //set of values
    qint16 MaxSize; //maximum size
    double Median; //median
    double Lambda; //coefficient of exponential smoothing
};

#endif // GLCALCTRAJPARAM_H
