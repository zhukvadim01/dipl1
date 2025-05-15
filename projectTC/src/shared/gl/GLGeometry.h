/*! \file GLGeometry.h
\brief Contains declarations required to work with
points, velocities, etc.
*/

#pragma once

#include <list>
#undef _OSF_SOURCE
#include <cmath>

#include "Constants.h"
#include "GLMatrix.h"

class GLSector;
struct GLPoint3D;

//DESCRIPTION	:  Represents points in 2-dimentional space.
struct GLPoint2D
{
    ~GLPoint2D() = default;
    constexpr GLPoint2D(int x0 = 0, int y0 = 0)
        : x(x0)
        , y(y0)
    {}
    GLPoint2D(const GLPoint2D&) = default;
    GLPoint2D(GLPoint2D&&) = default;

    qint32 x = 0;
    qint32 y = 0;

    bool operator==(GLPoint2D& p);
    GLPoint2D operator+(GLPoint2D p);
    GLPoint2D operator=(const GLPoint2D& p);
};

//DESCRIPTION	:  Represents points in 2-dimentional space.
struct GLPointDouble2D
{
    double x = 0.0;
    double y = 0.0;

    ~GLPointDouble2D() = default;
    GLPointDouble2D() = default;
    GLPointDouble2D(const double& X, const double& Y)
        : x(X)
        , y(Y)
    {}
    GLPointDouble2D(const GLPointDouble2D&) = default;
    GLPointDouble2D(GLPointDouble2D&&) = default;

    void clear();
    void init(const double& X, const double& Y);

    GLPointDouble2D operator+(GLPointDouble2D Q);
    GLPointDouble2D operator-(GLPointDouble2D Q);
    GLPointDouble2D operator*(const double& dig) const;
    GLPointDouble2D operator=(const GLPointDouble2D& Q);
    bool operator==(const GLPointDouble2D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble2D::ScalarProduct
    //DESCRIPTION	: Calculates scalar product
    //INPUTS		: Factor
    //RETURNS		: Scalar product
    double ScalarProduct(const GLPointDouble2D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble2D::module
    //DESCRIPTION	: length (absolute value) of the vector
    //INPUTS		: void
    //RETURNS		: module
    double module();

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble2D::getDistance
    //DESCRIPTION	: Returns distance between the current point and the given point
    //INPUTS		: Given point
    //RETURNS		: Distance between the current point and the given point
    double getDistance(GLPointDouble2D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble2D::getDistance
    //DESCRIPTION	: Returns angle between vectors represented by current point and given point
    //INPUTS		: Given point
    //RETURNS		: Angle between vectors represented by current point and given point (radians)
    double getAngle(GLPointDouble2D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble2D::getProjectionOn()
    //DESCRIPTION	: Returns projection of the vector represented by current GLPointDouble2D
    //              : on the given vector represented by GLPointDouble2D Q
    //INPUTS		: Given vector Q represented by GLPointDouble2D
    //RETURNS		: Projection of the current vector on the given vector Q
    double getProjectionOn(GLPointDouble2D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble2D::IsZero
    //DESCRIPTION	: Checks whether point is zero
    //INPUTS		: None
    //RETURNS		: True if point is zero
    bool IsZero();
};

//PACKAGE		:  GL
//FUNCTION		: GLdistance( const GLPointDouble2D& p1, const GLPointDouble2D& p2 );
//DESCRIPTION	: Calculates distance between two points.
//INPUTS		: p1 A reference to the first point, p2 A reference to the second point.
//RETURNS		: Distance between p1 and p2.
double GLdistance(const GLPointDouble2D& p1, const GLPointDouble2D& p2);

//PACKAGE		:  GL
//FUNCTION			: unsigned int GLdistance( const GLPoint2D& p1, const GLPoint2D& p2 );
//DESCRIPTION	: Calculates distance between two points.
//INPUTS		: p1 A reference to the first point, p2 A reference to the second point.
//RETURNS		: Distance between p1 and p2.
unsigned int GLdistance(
const GLPoint2D& p1,  //A reference to the first point
const GLPoint2D& p2   //A reference to the second point.
);

//PACKAGE		:  GL
//STRUCT			:  GLGeodeticHight
//DESCRIPTION	: Represents geodetic hight.
struct GLGeodeticHight
{
    //PACKAGE		:  GL
    //FUNCTION			: void GLGeodeticHight::GLGeodeticHight( );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: NONE
    //RETURNS		: NONE
    GLGeodeticHight() = default;

    GLGeodeticHight(const GLGeodeticHight& gh)
        : signExist(gh.signExist)
        , hight(gh.hight)
        , vel(gh.vel)
        , accel(gh.accel)
        , time(gh.time)
    {}

    //PACKAGE		:  GL
    //FUNCTION			: void GLGeodeticHight::clear();
    //DESCRIPTION	: Clears all data in this structure.
    //INPUTS		: NONE
    //RETURNS		: NONE
    void clear()
    {
        signExist = false;
        hight = 0;
        vel = 0;
        accel = 0;
        time = 0;
    }

    //PACKAGE		:  GL
    //FUNCTION		: void GLGeodeticHight:: operator = ;
    //DESCRIPTION	: Assignment operator.
    //INPUTS		: Assignment data
    //RETURNS		: GLGeodeticHight object
    GLGeodeticHight operator=(GLGeodeticHight data)
    {
        signExist = data.signExist;
        hight = data.hight;
        vel = data.vel;
        accel = data.accel;
        time = data.time;
        return *this;
    }

    bool signExist{ false };
    //Sign of presence info in this structure.

    int hight{ 0 };
    //Geodetic hight.

    short vel{ 0 };
    //Geodetic velocity.

    double accel{ 0 };
    //Geodetic acceleration.

    qint64 time{ 0 };
    //Time coordinates components relate to.
};

struct GLVelocityFloat3D;

//PACKAGE		:  GL
//STRUCT		:  GLVelocity3D
//DESCRIPTION	: Represents 3D - velocity.
struct GLVelocity3D
{
    //PACKAGE		:  GL
    //FUNCTION		: GLVelocity3D::GLVelocity3D( int vx0 = 0, int vy0 = 0, int vz0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: vx0 - Initial X velocity, vy0 - Initial Y velocity, vz0 - Initial Z velocity.
    //RETURNS		: NONE
    GLVelocity3D(
    qint16 vx0 = 0,  //Initial X velocity.
    qint16 vy0 = 0,  //Initial Y velocity.
    qint16 vz0 = 0   //Initial Z velocity.
    );

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator=(const qint16& Q);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data.
    //RETURNS		: Result of assignment
    GLVelocity3D operator=(const qint16& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator=(const GLPoint3D& Q);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data.
    //RETURNS		: Result of assignment
    GLVelocity3D operator=(const GLPoint3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator=(const GLVelocity3D& Q);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data.
    //RETURNS		: Result of assignment
    GLVelocity3D operator=(const GLVelocity3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator +(const GLVelocity3D& Q);
    //DESCRIPTION	: Add operator
    //INPUTS		: Adding data.
    //RETURNS		: Sum
    GLVelocity3D operator+(const GLVelocity3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator -(const GLVelocity3D& Q);
    //DESCRIPTION	: Subtraction operator
    //INPUTS		: Subtracting data.
    //RETURNS		: Result of Subtraction
    GLVelocity3D operator-(const GLVelocity3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator -(const GLVelocity3D& Q);
    //DESCRIPTION	: Subtraction operator
    //INPUTS		: Subtracting data.
    //RETURNS		: Result of Subtraction
    GLVelocity3D operator-(const GLVelocityFloat3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator *(int Q);
    //DESCRIPTION	: Multiply operator
    //INPUTS		: Factor.
    //RETURNS		: Result
    GLVelocity3D operator*(int Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocity3D::operator *(double Q);
    //DESCRIPTION	: Multiply operator
    //INPUTS		: Factor.
    //RETURNS		: Result
    GLVelocity3D operator*(double Q);

    //PACKAGE		:GL.
    //FUNCTION		:GLVelocity3D::operator !=(const int& dig);
    //DESCRIPTION	:Compare operator.
    //INPUT			:Reference to data
    //RETURNS		:Sign is false if equal, otherwise true.
    bool operator!=(const int& dig);

    qint16 vx;
    //Velocity X component.

    qint16 vy;
    //Velocity Y component.

    qint16 vz;
    //Velocity Z component.
};

unsigned int GLVdistance(
const GLVelocity3D& p1,  //A reference to the first point
const GLVelocity3D& p2   //A reference to the second point.
);
// Represents points in 3-dimentional space.
// Important!!!: If this class contain coordinates in Gaussian frame
// of reference then operators "+","-" not available.

struct GLPointFloat3D;
struct GLPointDouble3D;

//PACKAGE		:  GL
//STRUCT		:  GLVelocityFloat3D
//DESCRIPTION	: Represents 3D - velocity.
struct GLVelocityFloat3D
{
    //PACKAGE		:  GL
    //FUNCTION		: GLVelocityFloat3D::GLVelocityFloat3D( float vx0 = 0, float vy0 = 0, float vz0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: vx0 - Initial X velocity, vy0 - Initial Y velocity, vz0 - Initial Z velocity.
    //RETURNS		: NONE
    GLVelocityFloat3D(
    float vx0 = 0,  //Initial X velocity.
    float vy0 = 0,  //Initial Y velocity.
    float vz0 = 0   //Initial Z velocity.
    );

    void clear();

    // PACKAGE		: GL.
    // FUNCTION		: void GLVelocityFloat3D::init( const float& VX, const float& VY, const float& VZ ).
    // DESCRIPTION	: Structure constructor.
    // INPUTS		: VX - new VX coordinate, VY - new VY coordinate, VZ - new VZ coordinate.
    // RETURNS		: NONE.
    void init(
    const float& VX,  // New X coordinate.
    const float& VY,  // New Y coordinate.
    const float& VZ   // New Z coordinate.
    );

    // PACKAGE		: GL.
    // FUNCTION		: void GLVelocityFloat3D::init( double& VX, const double& VY, const double& VZ ).
    // DESCRIPTION	: Structure constructor.
    // INPUTS		: VX - new VX coordinate, VY - new VY coordinate, VZ - new VZ coordinate.
    // RETURNS		: NONE.
    void init(
    const double& VX,  // New VX coordinate.
    const double& VY,  // New VY coordinate.
    const double& VZ   // New VZ coordinate.
    );

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator=(const float& Q);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data.
    //RETURNS		: Result of assignment
    GLVelocityFloat3D operator=(const float& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator=(const GLPointDouble3D& Q);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data.
    //RETURNS		: Result of assignment
    GLVelocityFloat3D operator=(const GLPointFloat3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator=(const GLVelocityFloat3D& Q);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignming data.
    //RETURNS		: Result of assignment
    GLVelocityFloat3D operator=(const GLVelocityFloat3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator +(const GLVelocityFloat3D& Q);
    //DESCRIPTION	: Add operator
    //INPUTS		: Adding data.
    //RETURNS		: Sum
    GLVelocityFloat3D operator+(const GLVelocityFloat3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator -(const GLVelocityFloat3D& Q);
    //DESCRIPTION	: Subtraction operator
    //INPUTS		: Subtracting data.
    //RETURNS		: Result of Subtraction
    GLVelocityFloat3D operator-(const GLVelocityFloat3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator -(const GLVelocity3D& Q);
    //DESCRIPTION	: Subtraction operator
    //INPUTS		: Subtracting data.
    //RETURNS		: Result of Subtraction
    GLVelocityFloat3D operator-(const GLVelocity3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator *(int Q);
    //DESCRIPTION	: Multiply operator
    //INPUTS		: Factor.
    //RETURNS		: Result
    GLVelocityFloat3D operator*(int Q);

    //PACKAGE		: GL
    //FUNCTION		: GLVelocityFloat3D::operator *(double Q);
    //DESCRIPTION	: Multiply operator
    //INPUTS		: Factor.
    //RETURNS		: Result
    GLVelocityFloat3D operator*(double Q);

    // PACKAGE		: GL.
    // FUNCTION		: GLVelocityFloat3D::copy( const GLPointDouble3D& Q ).
    // DESCRIPTION	: Assignment operator.
    // INPUTS		: Assignming data.
    // RETURNS		: Result of assignment.
    GLVelocityFloat3D copy(const GLPointDouble3D& Q);

    // PACKAGE		: GL.
    // FUNCTION		: operator==(const GLVelocityFloat3D& p)
    // DESCRIPTION	: Compare operator.
    // INPUTS			: Comparing data.
    // RETURNS		: Result of compare.
    bool operator==(const GLVelocityFloat3D& p)
    {
        if (fabs(p.vx - this->vx) > 0.001)
            return false;
        if (fabs(p.vy - this->vy) > 0.001)
            return false;
        if (fabs(p.vz - this->vz) > 0.001)
            return false;
        return true;
    }

    float vx = 0.0;
    float vy = 0.0;
    float vz = 0.0;
};


struct GLBearing;

//PACKAGE		:  GL
//STRUCT			:  GLPoint3D
//DESCRIPTION	: Represents 3D - point.
struct GLPoint3D
{
    qint32 x = 0;
    qint32 y = 0;
    qint32 z = 0;

    GLPoint3D(qint32 x0 = 0, qint32 y0 = 0, qint32 z0 = 0);
    GLPoint3D(GLPoint3D& other) = default;
    GLPoint3D(const GLPoint3D& other) = default;
    GLPoint3D operator=(const int& Q);

    void clear();

    //PACKAGE		: GL
    //FUNCTION		: GLPoint3D::GLPoint3D( GLBearing bearing, int distance);
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: bearing - azimuth and elevation, distance - distance
    //RETURNS		: NONE
    GLPoint3D(GLBearing bearing, int distance);


    //PACKAGE		: GL
    //FUNCTION		: GLPoint3D::module
    //DESCRIPTION	: length (absolute value) of the vector
    //INPUTS		: void
    //RETURNS		: module
    quint64 module();

    GLPoint3D operator=(const GLPoint3D& Q);
    GLPoint3D operator+(const GLPoint3D& Q) const;
    GLPoint3D operator*(const GLPoint3D& Q) const;
    GLPoint3D operator*(const int& dig) const;
    GLPoint3D operator+(const GLVelocity3D& Q) const;
    GLPoint3D operator-(const GLPoint3D& Q) const;
    GLPoint3D operator-(const GLPointDouble3D& Q) const;
    GLPoint3D operator/(const int& dig) const;
    constexpr bool operator==(GLPoint3D& Q) { return (x == Q.x && y == Q.y && z == Q.z) ? (true) : (false); }
    constexpr bool operator!=(GLPoint3D& Q) { return !operator==(Q); }
    constexpr bool operator==(const GLPoint3D& other) { return (x == other.x && y == other.y && z == other.z) ? (true) : (false); }
    constexpr bool operator!=(const GLPoint3D& other) { return !operator==(other); }

public:
    //PACKAGE		: GL
    //FUNCTION		: GLPoint3D::init(const qint32& X,const qint32& Y,const qint32& Z);
    //DESCRIPTION	: Init memebers.
    //INPUTS		: Initial X coordinate, Initial Y coordinate, Initial Y coordinate..
    //RETURNS		: NONE
    void init(const qint32& X, const qint32& Y, const qint32& Z);
};

struct GLPointFloat3D;
struct GLAccel3D;

// PACKAGE		:  GL
// STRUCT		:  GLPointDouble3D
// DESCRIPTION	:  Represents points in 3-dimentional space.
struct GLPointDouble3D
{
    friend QDataStream& operator<<(QDataStream& stream, const GLPointDouble3D& data);
    friend QDataStream& operator>>(QDataStream& stream, GLPointDouble3D& data);

public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    GLPointDouble3D();
    GLPointDouble3D(const double& X, const double& Y, const double& Z);
    GLPointDouble3D(GLPointDouble3D& other) = default;
    GLPointDouble3D(const GLPointDouble3D& other) = default;
    GLPointDouble3D operator=(const GLPointDouble3D& Q);
    GLPointDouble3D operator=(int Q);

    void clear();
    void init(const double& X, const double& Y, const double& Z);

    // PACKAGE		: GL.
    // FUNCTION		: GLPointDouble3D GLPointDouble3D::operator+( GLPointDouble3D Q ).
    // DESCRIPTION	: Add operator.
    // INPUTS		: Adding data.
    // RETURNS		: Sum.
    GLPointDouble3D operator+(const GLPointDouble3D Q) const;
    GLPointDouble3D operator-(const GLPoint3D& Q) const;
    GLPointDouble3D operator-(const GLPointDouble3D& Q) const;
    GLPointDouble3D operator-() const;
    GLPointDouble3D operator*(const int& dig) const;
    GLPointDouble3D operator*(const double& dig) const;
    GLPointDouble3D operator*(const float& dig) const;
    GLPointDouble3D operator*(const GLPointDouble3D& Q) const;
    GLPointDouble3D operator/(const int& dig) const;
    GLPointDouble3D operator/(const double& div) const;
    bool operator==(const GLPointDouble3D& Q);
    bool operator!=(const GLPointDouble3D& Q);

    GLPointDouble3D copy(const GLPoint3D& Q);
    GLPointDouble3D copy(const GLPointFloat3D& Q);
    GLPointDouble3D copy(const GLVelocityFloat3D& Q);
    GLPointDouble3D copy(const GLAccel3D& Q);


    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::module
    //DESCRIPTION	: length (absolute value) of the vector
    //INPUTS		: void
    //RETURNS		: module
    quint64 module() const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::module
    //DESCRIPTION	: length (absolute value) of the vector, result as floating point number
    //INPUTS		: void
    //RETURNS		: module
    double moduleFloat() const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::ScalarProduct
    //DESCRIPTION	: Calculates scalar product
    //INPUTS		: Factor
    //RETURNS		: Scalar product
    double ScalarProduct(const GLPointDouble3D& Q) const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::VectorProduct
    //DESCRIPTION	: Calculates vector product (cross product)
    //INPUTS		: Factor
    //RETURNS		: Vector product (cross product)
    GLPointDouble3D VectorProduct(const GLPointDouble3D& Q) const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::getMax
    //DESCRIPTION	: Returns maximum of (x, y, z)
    //INPUTS		: None
    //RETURNS		: Maximum of (x, y, z)
    double getMax() const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::getMax
    //DESCRIPTION	: Returns minimum of (x, y, z)
    //INPUTS		: None
    //RETURNS		: Minimum of (x, y, z)
    double getMin() const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::getDistance
    //DESCRIPTION	: Returns distance between the current point and the given point
    //INPUTS		: Given point
    //RETURNS		: Distance between the current point and the given point
    double getDistance(GLPointDouble3D& Q) const;
    double getDistance(GLPointFloat3D& Q) const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::getDistanceOnSurface
    //DESCRIPTION	: Returns curvilinear distance between projections of the current point and the given point
    //              : on the Earth surface, when GLPointDouble3D contains ECEF coordinates
    //INPUTS		: Given point
    //RETURNS		: Curvilinear distance on the Earth surface between the current point and the given point
    double getDistanceOnSurface(GLPointDouble3D& Q) const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::getDistance
    //DESCRIPTION	: Returns angle between vectors represented by current point and given point
    //INPUTS		: Given point
    //RETURNS		: Angle between vectors represented by current point and given point (radians)
    double getAngle(GLPointDouble3D& Q) const;

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::getProjectionOn()
    //DESCRIPTION	: Returns projection of the vector represented by current GLPointDouble3D
    //              : on the given vector represented by GLPointDouble3D Q
    //INPUTS		: Given vector Q represented by GLPointDouble3D
    //RETURNS		: Projection of the current vector on the given vector Q
    double getProjectionOn(GLPointDouble3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointDouble3D::IsZero
    //DESCRIPTION	: Checks whether point is zero
    //INPUTS		: None
    //RETURNS		: True if point is zero
    bool IsZero() const;
};

// PACKAGE		:  GL
// STRUCT		:  GLPointFloat3D
// DESCRIPTION	:  Represents points in 3-dimentional space.
struct GLPointFloat3D
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    GLPointFloat3D();
    GLPointFloat3D(const float& X, const float& Y, const float& Z);
    GLPointFloat3D(GLPointFloat3D& other) = default;
    GLPointFloat3D(const GLPointFloat3D& other) = default;
    GLPointFloat3D operator=(const GLPointFloat3D& Q);
    GLPointFloat3D operator=(int Q);

    void clear();
    void init(const float& X, const float& Y, const float& Z);

    GLPointFloat3D operator+(GLPointFloat3D Q);
    GLPointFloat3D operator-(const GLPoint3D& Q) const;
    GLPointFloat3D operator-(const GLPointFloat3D& Q) const;
    GLPointFloat3D operator*(const int& dig) const;
    GLPointFloat3D operator*(const float& dig) const;
    GLPointFloat3D operator*(const GLPointFloat3D& Q) const;
    GLPointFloat3D operator/(const int& dig) const;
    GLPointFloat3D operator/(const float& dig) const;
    GLPointFloat3D operator/(const quint32& dig) const;
    bool operator==(const GLPointFloat3D& Q);

    GLPointFloat3D copy(const GLPoint3D& Q);
    GLPointFloat3D copy(const GLPointDouble3D& Q);

    //PACKAGE		: GL
    //FUNCTION		: GLPointFloat3D::module
    //DESCRIPTION	: length (absolute value) of the vector
    //INPUTS		: void
    //RETURNS		: module
    quint64 module();

    bool IsZero();
};

//PACKAGE		:  GL
//STRUCT			:  GLAccel3D
//DESCRIPTION	: Represents acceleration in 3-dimentional space.
struct GLAccel3D
{
    //PACKAGE		:  GL
    //FUNCTION			: GLAccel3D::GLAccel3D( qint16 x0 = 0, qint16 y0 = 0, qint16 z0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: x0 - Initial X acceleration, y0 - Initial Y acceleration, z0 - Initial Z acceleration.
    //RETURNS		: NONE
    //	GLAccel3D( qint16 x0 = 0,	//Initial X acceleration
    //		qint16 y0 = 0,	//Initial Y acceleration
    //		qint16 z0 = 0	//Initial Z acceleration
    //		);

    GLAccel3D() = default;

    GLAccel3D(const GLAccel3D& acc)
        : sAx(acc.sAx)
        , sAy(acc.sAy)
        , sAz(acc.sAz)
    {}

    GLAccel3D(const GLPointDouble3D& pnt)
        : sAx(pnt.x)
        , sAy(pnt.y)
        , sAz(pnt.z)
    {}

    //PACKAGE		:  GL
    //FUNCTION			: GLAccel3D::GLAccel3D( float x0 = 0, float y0 = 0, float z0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: x0 - Initial X acceleration, y0 - Initial Y acceleration, z0 - Initial Z acceleration.
    //RETURNS		: NONE
    GLAccel3D(float x0, float y0, float z0);

    void clear();

    // PACKAGE		: GL.
    // FUNCTION		: void GLAccel3D::init( const float& AX, const float& AY, const float& AZ ).
    // DESCRIPTION	: Structure constructor.
    // INPUTS		: AX - new AX coordinate, AY - new AY coordinate, AZ - new AZ coordinate.
    // RETURNS		: NONE.
    void init(
    const float& AX,  // New AX coordinate.
    const float& AY,  // New AY coordinate.
    const float& AZ   // New AZ coordinate.
    );

    // PACKAGE		: GL.
    // FUNCTION		: void GLAccel3D::init( double& AX, const double& AY, const double& AZ ).
    // DESCRIPTION	: Structure constructor.
    // INPUTS		: AX - new AX coordinate, AY - new AY coordinate, AZ - new AZ coordinate.
    // RETURNS		: NONE.
    void init(
    const double& AX,  // New AX coordinate.
    const double& AY,  // New AY coordinate.
    const double& AZ   // New AZ coordinate.
    );


    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator=
    //DESCRIPTION	: Operator =.
    //INPUTS		: Point
    //RETURNS		: Acceleration
    GLAccel3D operator=(GLPoint3D Q);

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator=
    //DESCRIPTION	: Operator =.
    //INPUTS		: Float point
    //RETURNS		: Acceleration
    GLAccel3D operator=(GLPointFloat3D Q);

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::copy
    //DESCRIPTION	: Copy.
    //INPUTS		: Double point
    //RETURNS		: Acceleration
    GLAccel3D copy(GLPointDouble3D Q);

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator=
    //DESCRIPTION	: Operator =.
    //INPUTS		: Acceleration
    //RETURNS		: Acceleration
    GLAccel3D operator=(GLAccel3D Q);

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator >
    //DESCRIPTION	: Compare operator.
    //INPUTS		: Acceleration.
    //RETURNS		: True if more
    bool operator>(GLAccel3D& accel) const { return sqrt((double)sAx * sAx + sAy * sAy + sAz * sAz) > sqrt((double)accel.sAx * accel.sAx + accel.sAy * accel.sAy + accel.sAz * accel.sAz); }

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator *
    //DESCRIPTION	: Scalar product.
    //INPUTS		: Vector to multiply.
    //RETURNS		: result
    float operator*(GLAccel3D mult) { return sAx * mult.sAx + sAy * mult.sAy + sAz * mult.sAz; }

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator *
    //DESCRIPTION	: Multiplication by a scalar.
    //INPUTS		: Scalar to multiply.
    //RETURNS		: result
    GLAccel3D operator*(float mult)
    {
        GLAccel3D tmp;
        tmp.sAx = sAx * mult;
        tmp.sAy = sAy * mult;
        tmp.sAz = sAz * mult;
        return tmp;
    }

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator +
    //DESCRIPTION	: Vectors sum.
    //INPUTS		: Vector.
    //RETURNS		: result
    GLAccel3D operator+(GLAccel3D add)
    {
        GLAccel3D tmp;
        tmp.sAx = sAx + add.sAx;
        tmp.sAy = sAy + add.sAy;
        tmp.sAz = sAz + add.sAz;
        return tmp;
    }

    //PACKAGE		: GL
    //FUNCTION		: GLAccel3D::operator ^
    //DESCRIPTION	: Vector product.
    //INPUTS		: Vector to multiply.
    //RETURNS		: result
    GLAccel3D operator^(GLAccel3D mult)
    {
        GLAccel3D tmp;
        tmp.sAx = sAy * mult.sAz - sAz * mult.sAy;
        tmp.sAy = -sAx * mult.sAz + sAz * mult.sAx;
        tmp.sAz = sAx * mult.sAy - sAy * mult.sAx;
        return tmp;
    }

    //PACKAGE		:GL.
    //FUNCTION		:bool operator !=( GLAccel3D& accel ) const
    //DESCRIPTION	:Compare operator.
    //INPUT			:Acceleration
    //RETURNS		:Sign is false if equal, otherwise true.
    bool operator!=(GLAccel3D& accel) const { return sqrt((double)sAx * sAx + sAy * sAy + sAz * sAz) != sqrt((double)accel.sAx * accel.sAx + accel.sAy * accel.sAy + accel.sAz * accel.sAz); }

    //PACKAGE		:GL.
    //FUNCTION		:bool operator !=( GLAccel3D& accel ) const
    //DESCRIPTION	:Compare operator.
    //INPUT			:Acceleration
    //RETURNS		:Sign is false if equal, otherwise true.
    bool operator==(const GLAccel3D& accel) const { return sqrt((double)sAx * sAx + sAy * sAy + sAz * sAz) == sqrt((double)accel.sAx * accel.sAx + accel.sAy * accel.sAy + accel.sAz * accel.sAz); }

    //PACKAGE		:  GL
    //FUNCTION			: qint16 GLAccel3D::HasModule();
    //DESCRIPTION	: .
    //INPUTS		: NONE.
    //RETURNS		: Retrieves acceleration module.
    float HasModule() const
    {
        double lMod = sqrt((double)(sAx * sAx + sAy * sAy + sAz * sAz));
        return float(lMod);
    }

    float sAx = 0.0; //X - acceleration
    float sAy = 0.0; //Y - acceleration
    float sAz = 0.0; //Z - acceleration
};

//PACKAGE		:  GL
//FUNCTION			: unsigned int GLdistance( const GLPoint3D& p1, const GLPoint3D& p2 );
//DESCRIPTION	: Calculates distance between two points.
//INPUTS		: p1  - A reference to the first point, p2 - A reference to the second point.
//RETURNS		: Distance between p1 and p2.
unsigned int GLdistance(
const GLPoint3D& p1,  //p1 A reference to the first point.
const GLPoint3D& p2   //A reference to the second point.
);

//PACKAGE		:  GL
//FUNCTION		: float GLdistance( const GLPointFloat3D& p1, const GLPointFloat3D& p2 );
//DESCRIPTION	: Calculates distance between two points.
//INPUTS		: p1  - A reference to the first point, p2 - A reference to the second point.
//RETURNS		: Distance between p1 and p2.
float GLdistance(const GLPointFloat3D& p1, const GLPointFloat3D& p2);

//PACKAGE		:  GL
//FUNCTION		: float GLdistance( const GLPoint3D& p1, const GLPointFloat3D& p2 );
//DESCRIPTION	: Calculates distance between two points.
//INPUTS		: p1  - A reference to the first point, p2 - A reference to the second point.
//RETURNS		: Distance between p1 and p2.
float GLdistance(const GLPoint3D& p1, const GLPointFloat3D& p2);


//PACKAGE		:  GL
//FUNCTION		: double GLdistance( const GLPoint3D& p);
//DESCRIPTION	: Calculates distance of point.
//INPUTS		: p  - A reference to the  point
//RETURNS		: Distance of point.
double GLdistance(const GLPoint3D& p);

//PACKAGE		:  GL
//FUNCTION		: double GLdistance(  const GLPointDouble3D& p1 , const GLPointDouble3D& p2);
//DESCRIPTION	: Calculates distance between two points.
//INPUTS		: p1  - A reference to the first point, p2 - A reference to the second point.
//RETURNS		: Distance between p1 and p2.
int GLdistance(const GLPointDouble3D& p1, const GLPointDouble3D& p2);


//PACKAGE		:  GL
//FUNCTION			: unsigned int GLdistanceProjection( const GLPoint3D& p );
//DESCRIPTION	: Calculates distance between point projection on XY and origin point.
//INPUTS		: p - A reference to the point.
//RETURNS		: Distance distance between point p projection on XY and point (0,0,0).
unsigned int GLdistanceProjection(const GLPoint3D& p  //A reference to the point
);

//PACKAGE		:  GL
//FUNCTION		: double GLbisectAngle(double begSect,double endSect);
//DESCRIPTION	: Calculates bisect angle (rad)
//INPUTS		: begSect(rad) - A Left boundary angle,endSect(rad) - A right boundary angle .
//RETURNS		: Bisect angle
double GLbaseAngle(double begSect, double endSect);

//PACKAGE		:  GL
//FUNCTION		: double GLbisectAngle(double begSect,double endSect);
//DESCRIPTION	: Calculates bisect of angle (rad)
//INPUTS		: begSect(rad) - A Left boundary angle,endSect(rad) - A right boundary angle .
//RETURNS		: Bisect angle
double GLbisectAngle(double begSect, double endSect);

//PACKAGE		: GL
//FUNCTION		: GLCoordToVector()
//DESCRIPTION	: Converts coordinates to the vector of dimension 3
//INPUTS		: Coordinates; reference to the resulting vector
//RETURNS		: None
void GLCoordToVector(const GLPointDouble3D& Coord, TVector<3>& ResVec);

//PACKAGE		: GL
//FUNCTION		: GLCoordVelToVector()
//DESCRIPTION	: Converts coordinates and velocity to the vector of dimension 6
//INPUTS		: Coordinates; velocity components; reference to the resulting vector
//RETURNS		: None
void GLCoordVelToVector(const GLPointDouble3D& Coord, const GLPointDouble3D& Vel, TVector<6>& ResVec);

//PACKAGE		: GL
//FUNCTION		: GLCoordVelAccToVector()
//DESCRIPTION	: Converts coordinates and velocity to the vector of dimension 9
//INPUTS		: Coordinates; velocity components; acceleration components; reference to the resulting vector
//RETURNS		: None
void GLCoordVelAccToVector(const GLPointDouble3D& Coord, const GLPointDouble3D& Vel, const GLPointDouble3D& Acc, TVector<9>& ResVec);

//PACKAGE		: GL
//FUNCTION		: GLCoordToVector()
//DESCRIPTION	: Converts the vector of dimension 3 to the GLPointDouble3D
//INPUTS		: Given vector; reference to the resulting coordinates GLPointDouble3D
//RETURNS		: None
void GLVectorToCoord(const TVector<3>& Vec, GLPointDouble3D& ResCoord);

//PACKAGE		:  GL
//STRUCT			:  GLPointLL
//DESCRIPTION	: Represents points in "latitude-longitude-height" coordinates.
struct GLPointLL
{
    //PACKAGE		:  GL
    //FUNCTION			: GLPointLL::GLPointLL( double lt0 = 0.0, double lg0 = 0.0, int h0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: lt0 Initial latitude (in radians), lg0 - Initial longitude (in radians), h0 - Initial height.
    //RETURNS		: NONE
    GLPointLL(
    double lt0 = 0.0,  //Initial latitude (in radians)
    double lg0 = 0.0,  //Initial longitude (in radians).
    int h0 = 0         //Initial height.
    );

    //PACKAGE		: GL
    //FUNCTION		: GLPointLL::operator=(const GLPointLL& pt);
    //DESCRIPTION	: Assignment operator
    //INPUTS		: Assignment data.
    //RETURNS		: Result of assignment
    GLPointLL operator=(const GLPointLL& pt);

    double lt = 0.0; //Latitude.
    double lg = 0.0; //Longitude.
    int h = 0; //Height.
};

//PACKAGE		:  GL
//FUNCTION			: unsigned int GLabs( const GLVelocity3D& v );
//DESCRIPTION	: Calculated absolute value of velocity.
//INPUTS		: v - A reference to velocity.
//RETURNS		: Absolute value of the velocity.
unsigned int GLabs(const GLVelocity3D& v  //A reference to velocity.
);

//PACKAGE		:  GL
//FUNCTION			: unsigned int GLabsFlat( const GLVelocity3D& v );
//DESCRIPTION	: Calculated absolute value of velocity by 2 - dimension.
//INPUTS		: v - A reference to velocity.
//RETURNS		: Absolute value of the velocity.
unsigned int GLabsFlat(const GLVelocity3D& v  //A reference to velocity.
);

//PACKAGE		:  GL
//STRUCT			:  GLBearing
//DESCRIPTION	: Represents bearing.
struct GLBearing
{
    //PACKAGE		:  GL
    //FUNCTION			: GLBearing::GLBearing( double b0 = 0, double e0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: b0 - Initial azimuth coordinate (beta), e0 - Initial elevation coordinate (epsilon).
    //RETURNS		: None.
    GLBearing(
    double b0 = 0,  //Initial azimuth coordinate (beta)
    double e0 = 0   //Initial elevation coordinate (epsilon).
    );

    //PACKAGE		:  GL
    //FUNCTION			: GLBearing::GLBearing(GLPoint3D p);
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: p - Bearing point.
    //RETURNS		: None.
    GLBearing(GLPoint3D p  //Bearing point.
    );

    //PACKAGE		:  GL
    //FUNCTION			: GLBearing::GLBearing(GLPoint2D p);
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: p - Bearing point.
    //RETURNS		: None.
    GLBearing(GLPoint2D p  //Bearing point.
    );

    void clear() { b = e = 0.; }

    void set(double in_b, double in_e)
    {
        b = in_b;
        e = in_e;
    }

    GLBearing& operator=(const GLBearing& brng)
    {
        b = brng.b;
        e = brng.e;
        return *this;
    }

    double b = 0.0; //Azimuth coordinate (beta).
    double e = 0.0; //Elevation coordinate (epsilon).
};

//PACKAGE		:  GL
//STRUCT			:  GLBearingVelocity
//DESCRIPTION	: Bearing velocity.
struct GLBearingVelocity
{
    //PACKAGE		:  GL
    //FUNCTION			: GLBearingVelocity::GLBearingVelocity( double vb0 = 0, double ve0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: vb0 - Initial azimuth velocity, ve0 - Initial elevation velocity.
    //RETURNS		: None.
    GLBearingVelocity(
    double vb0 = 0,  //Initial azimuth velocity.
    double ve0 = 0   //Initial elevation velocity.
    );

    void clear() { vb = ve = 0.; }

    void set(double in_vb, double in_ve)
    {
        vb = in_vb;
        ve = in_ve;
    }

    GLBearingVelocity& operator=(const GLBearingVelocity& brng)
    {
        vb = brng.vb;
        ve = brng.ve;
        return *this;
    }

    double vb = 0.0; //Azimuth component (beta velocity).
    double ve = 0.0; //Elevation component (epsilon velocity)
};

//PACKAGE		:  GL
//STRUCT		:  GLAngle3D
//DESCRIPTION	:  Measured angle coordinates.
struct GLAngle3D
{
    //PACKAGE		:  GL
    //FUNCTION		: GLAngle3D::GLAngle3D( double R0 = 0, double b0 = 0, double e0 = 0 );
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: D0 - Initial range, b0 - Initial azimuth, e0 - Initial elevation.
    //RETURNS		: None.
    GLAngle3D(
    double R0 = 0.,  //Initial range.
    double b0 = 0.,  //Initial azimuth velocity.
    double e0 = 0.   //Initial elevation velocity.
    );

    //PACKAGE		: GL
    //FUNCTION		: GLAngle3D::GLAngle3D(GLPoint3D p);
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: p - Bearing point.
    //RETURNS		: None.
    GLAngle3D(GLPoint3D p);


    //PACKAGE		:  GL
    //FUNCTION		: GLAngle3D::GLAngle3D(GLPoint2D p);
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: p - Bearing point.
    //RETURNS		: None.
    GLAngle3D(GLPoint2D p);

    //PACKAGE		:  GL
    //FUNCTION		: GLAngle3D::clear();
    //DESCRIPTION	: Structure constructor.
    //INPUTS		: Clear.
    //RETURNS		: None.
    void clear();

    //PACKAGE		: GL
    //FUNCTION		: GLAngle3D::operator=
    //DESCRIPTION	: Operator =.
    //INPUTS		: GLAngle3D
    //RETURNS		: Measured angle coordinates
    GLAngle3D operator=(GLAngle3D Q);

    double R = 0.0; // Range
    double b = 0.0; //Azimuth
    double e = 0.0; //Elevation
};

//PACKAGE		:  GL
//FUNCTION			: double GLazimuthChange(double newA, double prevA);
//DESCRIPTION	: Calculates difference between 2 azimuth.
//INPUTS		: newA - first azimuth, prevA - second azimuth.
//RETURNS		: difference between 2 azimuth.
double GLazimuthChange(
double newA,  //first azimuth.
double prevA  //second azimuth.
);

//PACKAGE		:  GL
//FUNCTION			: double GLmeetAngle(double a1, double a2);
//DESCRIPTION	: Calculates meet angle for two azimuth.
//INPUTS		: a1 - first azimuth, a2 - second azimuth.
//RETURNS		: difference between 2 azimuth.
double GLmeetAngle(
double a1,  //first azimuth.
double a2   //second azimuth.
);

//PACKAGE		:  GL
//FUNCTION			: double GLangleElevtn(const GLPoint3D& point)
//DESCRIPTION	: Calculates angle of elevation subject to earth radius.
//INPUTS		: point - Reference to point where find desired place.
//RETURNS		: angle of elevation.
inline double GLangleElevtn(const GLPoint3D& point  //Reference to point where find desired place.
)
{
    double D = sqrt((double)point.x * point.x + (double)point.y * point.y + (double)point.z * point.z);
    double Z = point.z - (D * D) / (2 * 8500000);
    return atan2(Z, D);
}

//PACKAGE		:  GL
//FUNCTION			: double GLangleElevtn(const GLPoint3D& point)
//DESCRIPTION	: Calculates angle of elevation subject to earth radius.
//INPUTS		: point1 Reference to point relative it calculate, point2 Reference to point where find desired place.
//RETURNS		: angle of elevation.
//inline
//double GLangleElevtn( const GLPoint3D& point1,	//Reference to point relative it calculate.
//	const GLPoint3D& point2	//Reference to point where find desired place.
//	)
//{
//  return 0;
//}

//extern const double PI;

//PACKAGE		: GL
//FUNCTION		: GLcroosLineCircuit(GLVelocity3D&  direct, GLPoint3D &firstPnt, GLPoint3D &secondPnt, quint32 radius)
//DESCRIPTION	: Calculates points of intersection of line and circuit
//INPUTS		: Direct of moving, firs and second points of line(relatively center of circuit), circuit radius
//RETURNS		: The pair of point of intersection
std::pair<GLPoint3D, GLPoint3D> GLcroosLineCircuit(GLVelocity3D& direct, GLPoint3D& firstPnt, GLPoint3D& secondPnt, quint32 radius);


double AB_2PI(double b1, double b2);     //add
double AB_PI(double b1, double b2);      //add
double SB_2PI(double b1, double b2);     //sub
double SMODB_2PI(double b1, double b2);  //abs. of sub
double SB_PI(double b1, double b2);      //sub
double SMODB_PI(double b1, double b2);   //abs. of sub

//PACKAGE		: GL
//FUNCTION		: AB_Interval_0_2Pi()
//DESCRIPTION	: Addition of 2 angles, result belongs to the interval [0, 2Pi)
//INPUTS		: 1st summand, radians; 2nd summand, radians
//RETURNS		: Sum, radians
double AB_Interval_0_2Pi(double b1, double b2);

//PACKAGE		: GL
//FUNCTION		: SB_Interval_0_2Pi()
//DESCRIPTION	: Subtraction of 2 angles, result belongs to the interval [0, 2Pi)
//INPUTS		: Minuend, radians; subtrahend, radians
//RETURNS		: Differense, radians
double SB_Interval_0_2Pi(double b1, double b2);

//PACKAGE		: GL
//FUNCTION		: SMODB_Interval_0_2Pi()
//DESCRIPTION	: Modulo subtraction of 2 angles, result belongs to the interval [0, 2Pi)
//INPUTS		: Minuend, radians; subtrahend, radians
//RETURNS		: Differense modulo, radians
double SMODB_Interval_0_2Pi(double b1, double b2);

//PACKAGE		: GL
//FUNCTION		: CorrAngle_Interval_MinusPi_Pi()
//DESCRIPTION	: Angle correction, result belongs to the interval (-2*Pi, 2*Pi)
//INPUTS		: Angle, radians
//RETURNS		: Corrected angle, radians
double CorrAngle_Interval_Minus2Pi_2Pi(double angle);

//PACKAGE		: GL
//FUNCTION		: calcGravitationalAcceleration()
//DESCRIPTION	: Calculates gravitational acceleration at a given height
//INPUTS		: Height, m
//RETURNS		: Gravitational acceleration, m/s^2
double calcGravitationalAcceleration(double H);

qint16 D2_I16(double dig);

quint16 count_Azim(GLPointDouble2D& pnt);

bool CourseInSector(qint16 course, qint16 storona1, qint16 storona2);
