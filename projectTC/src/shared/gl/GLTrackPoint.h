// FILE		:	GLTrackPoint.h
// DATE		:	February, 2015
// AUTHOR	:	Domankova Vera

#ifndef GLTRACKPOINT_H
#define GLTRACKPOINT_H

#include "GLGeometry.h"
#include "GLFileLog.h"
#include <QDateTime>

// PACKAGE		:	GL
// STRUCTURE	:	GLTrackPoint
// DESCRIPTION	:	Структура, описывающая точку трассы
class GLTrackPoint
{
public:
    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPoint::GLTrackPoint
    // DESCRIPTION	:	Constructor.
    // INPUTS 		:	void
    GLTrackPoint();

    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPoint::Clear
    // DESCRIPTION	:	Clear structure.
    // INPUTS 		:	void

    void clear( void );

    GLPoint3D	   coord;	   // Координаты.
    GLVelocity3D   vel;	       // Составляющие скорости.
    GLAccel3D      accel;      // Ускорение.
    qint64		   time;	   // Время(msec).
};

class GLTrackPointDouble;

// PACKAGE		:	GL
// STRUCTURE	:	GLTrackPointFloat
// DESCRIPTION	:	Структура, описывающая точку трассы
class GLTrackPointFloat
{
public:
    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPointFloat::GLTrackPointFloat
    // DESCRIPTION	:	Constructor.
    // INPUTS 		:	void
    GLTrackPointFloat();

    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPoint::Clear
    // DESCRIPTION	:	Clear structure.
    // INPUTS 		:	void
    void clear( void );

    // PACKAGE		: GL.
    // FUNCTION		: GLTrackPointFloat::copy( const GLTrackPointDouble& Q ).
    // DESCRIPTION	: Assignment operator.
    // INPUTS		: Assignming data.
    // RETURNS		: Result of assignment.
    GLTrackPointFloat copy( const GLTrackPointDouble& Q );

    // PACKAGE		: GL.
    // FUNCTION		: GLTrackPointFloat::operator=( const GLTrackPointDouble& Q ).
    // DESCRIPTION	: Assignment operator.
    // INPUTS		: Assignming data.
    // RETURNS		: Result of assignment.
    GLTrackPointFloat& operator=( const GLTrackPointFloat& Q );

    GLPointFloat3D      coord;      // Координаты.
    GLVelocityFloat3D   vel;	    // Составляющие скорости.
    GLAccel3D           accel;      // Ускорение.
    qint64              time;       // Время(msec).
};

// PACKAGE		:	GL
// STRUCTURE	:	GLTrackPointDouble
// DESCRIPTION	:	Структура, описывающая точку трассы
class GLTrackPointDouble
{
    friend QDataStream& operator<<(QDataStream& stream, const GLTrackPointDouble& data);
    friend QDataStream& operator>>(QDataStream& stream, GLTrackPointDouble& data);

public:
    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPointDouble::GLTrackPointDouble
    // DESCRIPTION	:	Constructor.
    // INPUTS 		:	void
    GLTrackPointDouble();

    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPointDouble::Clear
    // DESCRIPTION	:	Clear structure.
    // INPUTS 		:	void
    void clear( void );

    // PACKAGE		: GL.
    // FUNCTION		: GLTrackPointDouble::copy( const GLTrackPointFloat& Q ).
    // DESCRIPTION	: Assignment operator.
    // INPUTS		: Assignming data.
    // RETURNS		: Result of assignment.
    GLTrackPointDouble copy( const GLTrackPointFloat& Q );

    // PACKAGE		: GL.
    // FUNCTION		: GLTrackPointDouble::operator=( const GLTrackPointFloat& Q ).
    // DESCRIPTION	: Assignment operator.
    // INPUTS		: Assignming data.
    // RETURNS		: Result of assignment.
    GLTrackPointDouble& operator=( const GLTrackPointDouble& Q );

    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPointDouble::Interpolation
    // DESCRIPTION	:	Interpolation at the given time using current point and given 2nd point
    // INPUTS 		:	Given time; reference to the 2nd point; reference to the resulting point
    // RETURNS		:   True if result is ok
    bool Interpolation(qint64 &GivenTime, GLTrackPointDouble &Point2, GLTrackPointDouble &ResPoint);

    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPointDouble::Extrapolation
    // DESCRIPTION	:	Extrapolation at the given time using current point
    // INPUTS 		:	Given time; reference to the resulting point
    // RETURNS		:   None
    void Extrapolation(qint64 GivenTime, GLTrackPointDouble &ResPoint);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLTrackPointDouble::tLogData
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; name of the data
    // RETURNS		:	None
    void tLogData(FILE* _pLogFile, const char* _Name);

    GLPointDouble3D     coord;      // Координаты.
    GLPointDouble3D     vel;	    // Составляющие скорости.
    GLPointDouble3D     accel;      // Ускорение.
    qint64              time{0};    // Время(msec)

private:
    // PACKAGE		:	GL
    // FUNCTION		:	GLTrackPointDouble::Interpolation_1coord
    // DESCRIPTION	:	Interpolation at the given time for one coordinate, using formula
    //              :   u(t) = p*t^5/5! + q*t^4/4! + r*t^3/3! + au0*t^2/2 + vu0*t + u0,
    //              :   where p, q, r are calculated using t1, t2, u1, vu1, au1, u2, vu2, au2
    // INPUTS 		:	Given time; time for 1st point; time for 2nd point;
    //              :   coordinate, velocity and acceleration for 1st and 2nd points;
    //              :   references to the coordinate, velocity and acceleration in the resulting point
    // RETURNS		:   True if result is ok
    bool Interpolation_1coord(double t_given, double t1, double t2, double u1, double vu1, double au1,
                              double u2, double vu2, double au2, double &u_res, double &vu_res, double &au_res);
};


// PACKAGE		:	GL
// CLASS    	:	GLPoint_tXYZVTh
// DESCRIPTION	:	Structure containing time, coordinates X,Y,Z, absolute velocity and angle of departure
class GLPoint_tXYZVTh
{
    friend QDataStream& operator<<(QDataStream& stream, const GLPoint_tXYZVTh& data);
    friend QDataStream& operator>>(QDataStream& stream, GLPoint_tXYZVTh& data);

public:
    qint64              time{0};   //time, milliseconds since midnight, Jan, 1, 1970
    GLPointDouble3D     coord;  //coordinates X, Y, Z, m
    qreal               V{0.};      //absolute velocity, m/s
    qreal               Theta{0.};  //angle of departure (throw angle), degrees

    GLPoint_tXYZVTh();

    void clear();

    // PACKAGE		: GL
    // FUNCTION		: bool GLPoint_tXYZVTh::operator=( GLPoint_tXYZVTh& Q )
    // DESCRIPTION	: Assignment operator
    // INPUTS		: Assignming data
    // RETURNS		: Result of assignment
    GLPoint_tXYZVTh& operator=( const GLPoint_tXYZVTh& Q );

    // PACKAGE		: GL
    // FUNCTION		: bool GLPoint_tXYZVTht::IsEqualTo
    // DESCRIPTION	: Compares current element with given element
    // INPUTS		: Compating data
    // RETURNS		: True if current element is equal to the given element
    bool isEqualTo(GLPoint_tXYZVTh &Q);

    // PACKAGE		: GL
    // FUNCTION		: bool GLPoint_tXYZVTh::IsEmpty
    // DESCRIPTION	: Checks whether the current data is empty
    // INPUTS		: None
    // RETURNS		: True if current data is empty
    bool isEmpty();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLPoint_tXYZVTh::tLogData
    // DESCRIPTION	:   Output data to the log file
    // INPUTS		:	Specified file; name of the data
    // RETURNS		:	None
    void tLogData(FILE* _pLogFile, const char* _Name);
};


#endif // GLTRACKPOINT_H
