/*! \file GLGeometry.cpp
\brief Implementation of everything declared in GLGeometry.h
*/

#undef _OSF_SOURCE

#include <cmath>
#include <QDataStream>

#include "conversion/Geocentric_Geo.h"
#include "Constants.h"
#include "GLGeometry.h"
#include "GLSector.h"

//const double PI = 3.1415926535897932384626433832795;

/*********************************************************************

                   GLPoint3D implementation

**********************************************************************/

void GLPoint3D::init(const qint32& X, const qint32& Y, const qint32& Z)
{
	x=X;
	y=Y;
	z=Z;
}

GLPoint3D GLPoint3D::operator =(const int& Q) 
{
	this->x=Q;
	this->y=Q;
	this->z=Q;
	return *this;

}

GLPoint3D GLPoint3D::operator =(const GLPoint3D& Q) 
{
	this->x=Q.x;
	this->y=Q.y;
	this->z=Q.z;
	return *this;

}

GLPoint3D GLPoint3D::operator +(const GLPoint3D& Q) const
{
	GLPoint3D temp;
	temp.x=this->x+Q.x;
	temp.y=this->y+Q.y;
	temp.z=this->z+Q.z;
	return temp;
}

GLPoint3D GLPoint3D::operator -(const GLPoint3D& Q) const
{
	GLPoint3D temp;
	temp.x=this->x-Q.x;
	temp.y=this->y-Q.y;
	temp.z=this->z-Q.z;
	return temp;
}

GLPoint3D GLPoint3D::operator -(const GLPointDouble3D& Q) const
{
    GLPoint3D temp;
    temp.x=this->x-(int)Q.x;
    temp.y=this->y-(int)Q.y;
    temp.z=this->z-(int)Q.z;
    return temp;
}

GLPoint3D GLPoint3D::operator *(const GLPoint3D& Q) const
{
	GLPoint3D temp;
	temp.x=this->x*Q.x;
	temp.y=this->y*Q.y;
	temp.z=this->z*Q.z;
	return temp;
}

GLPoint3D GLPoint3D::operator*(const int& dig) const
{
	GLPoint3D tmp;
	tmp.x=this->x*dig;
	tmp.y=this->y*dig;
	tmp.z=this->z*dig;
	return tmp;
}

GLPointFloat3D GLPointFloat3D::operator *(const float& dig)const
{
    GLPointFloat3D tmp;
    tmp.x=this->x*dig;
    tmp.y=this->y*dig;
    tmp.z=this->z*dig;
    return tmp;
}

GLPoint3D GLPoint3D::operator +(const GLVelocity3D& Q) const
{
	GLPoint3D temp;
	temp.x=this->x+(qint32)Q.vx;
	temp.y=this->y+(qint32)Q.vy;
	temp.z=this->z+(qint32)Q.vz;
	return temp;
}

GLPoint3D GLPoint3D::operator /(const int& dig) const
{
	GLPoint3D temp;
	if(((this->x/dig)-(qint32)(this->x/dig))>=0.5)
		temp.x=(qint32)(this->x/dig)+1;
	else
		temp.x=(qint32)(this->x/dig);
	if(((this->y/dig)-(qint32)(this->y/dig))>=0.5)
		temp.y=(qint32)(this->y/dig)+1;
	else
		temp.y=(qint32)(this->y/dig);
	if(((this->z/dig)-(qint32)(this->z/dig))>=0.5)
		temp.z=(qint32)(this->z/dig)+1;
	else
		temp.z=(qint32)(this->z/dig);
	return temp;
}
GLPointFloat3D GLPointFloat3D::operator /(const quint32& dig) const
{
    float dig1 = dig;
    GLPointFloat3D temp;
    temp.x=this->x/dig1;
    temp.y=(this->y/dig1);
    temp.z=(this->z/dig1);
    return temp;
}

GLPointFloat3D GLPointFloat3D::operator /(const float& dig) const
{
    GLPointFloat3D temp;
    temp.x=this->x/dig;
    temp.y=(this->y/dig);
    temp.z=(this->z/dig);
    return temp;
}

quint64 GLPoint3D::module()
{
	return sqrt( (double) x*x + y*y + z*z );
}

/*********************************************************************

GLPointDouble3D implementation

**********************************************************************/
GLPointDouble3D::GLPointDouble3D( void ):
	x( 0 ), y( 0 ), z( 0 )
{
	//
}

GLPointDouble3D::GLPointDouble3D( const double& X, const double& Y, const double& Z ):
	x( X ), y( Y ), z( Z )
{
	//
}

void GLPointDouble3D::clear( void )
{
    x = 0.;
    y = 0.;
    z = 0.;
}

void GLPointDouble3D::init( const double& X, const double& Y, const double& Z )
{
	x = X;
	y = Y;
	z = Z;
}

GLPointDouble3D GLPointDouble3D::operator+( const GLPointDouble3D Q ) const
{
	GLPointDouble3D tmp;
	tmp.x = this->x + Q.x;
	tmp.y = this->y + Q.y;
	tmp.z = this->z + Q.z;
	return tmp;
}

GLPointDouble3D GLPointDouble3D::operator=( const GLPointDouble3D& Q )
{
	this->x = Q.x;
	this->y = Q.y;
	this->z = Q.z;
	return *this;
}

GLPointDouble3D GLPointDouble3D::operator *(const int& dig)const
{
    GLPointDouble3D tmp;
    tmp.x=this->x*dig;
    tmp.y=this->y*dig;
    tmp.z=this->z*dig;
    return tmp;
}

GLPointDouble3D GLPointDouble3D::operator *(const double& dig)const
{
    GLPointDouble3D tmp;
    tmp.x=this->x*dig;
    tmp.y=this->y*dig;
    tmp.z=this->z*dig;
    return tmp;
}

GLPointDouble3D GLPointDouble3D::operator *(const float& dig)const
{
    GLPointDouble3D tmp;
    tmp.x=this->x*dig;
    tmp.y=this->y*dig;
    tmp.z=this->z*dig;
    return tmp;
}

GLPointDouble3D GLPointDouble3D::operator *(const GLPointDouble3D& Q) const
{
    GLPointDouble3D temp;
    temp.x=this->x*Q.x;
    temp.y=this->y*Q.y;
    temp.z=this->z*Q.z;
    return temp;
}


GLPointDouble3D GLPointDouble3D::operator /(const int& dig) const
{
    GLPointDouble3D temp;
    temp.x=(this->x/dig);
    temp.y=(this->y/dig);
    temp.z=(this->z/dig);
    return temp;
}


GLPointDouble3D GLPointDouble3D::operator /(const double& div) const
{
    GLPointDouble3D temp;
    temp.x = this->x / div;
    temp.y = this->y / div;
    temp.z = this->z / div;
    return temp;
}


GLPointDouble3D GLPointDouble3D::operator=(int Q )
{
    this->x = ( double )( Q );
    this->y = ( double )( Q );
    this->z = ( double )( Q );
	return *this;
}


GLPointDouble3D GLPointDouble3D::copy( const GLPoint3D& Q )
{
	this->x = ( double )( Q.x );
	this->y = ( double )( Q.y );
	this->z = ( double )( Q.z );
	return *this;
}

GLPointDouble3D GLPointDouble3D::copy( const GLPointFloat3D& Q )
{
    this->x = ( double )( Q.x );
    this->y = ( double )( Q.y );
    this->z = ( double )( Q.z );
    return *this;
}

GLPointDouble3D GLPointDouble3D::copy( const GLVelocityFloat3D& Q )
{
    this->x = ( double )( Q.vx );
    this->y = ( double )( Q.vy );
    this->z = ( double )( Q.vz );
    return *this;
}

GLPointDouble3D GLPointDouble3D::copy( const GLAccel3D& Q )
{
    this->x = ( double )( Q.sAx );
    this->y = ( double )( Q.sAy );
    this->z = ( double )( Q.sAz );
    return *this;
}

bool GLPointDouble3D::operator==(const GLPointDouble3D &Q )
{
    if (fabs(this->x - Q.x) < con_eps2 && fabs(this->y - Q.y) < con_eps2 && fabs(this->z - Q.z) < con_eps2)
    {
        return true;
    }
//	double tmp;
//	qint64 chek1, chek2, chek3;

//	tmp = ( this->x > Q.x ) ? ( this->x - Q.x ):( Q.x - this->x );
//	if ( ( tmp - ( qint64 )tmp ) >= 0.5 )
//	{
//		chek1 = ( qint64 )tmp + 1;
//	}
//	else
//	{
//		chek1 = ( qint64 )tmp;
//	}

//	tmp = ( this->y > Q.y ) ? ( this->y - Q.y ):( Q.y - this->y );
//	if ( ( tmp - ( qint64 )tmp ) >= 0.5 )
//	{
//		chek2 = ( qint64 )tmp + 1;
//	}
//	else
//	{
//		chek2 = ( qint64 )tmp;
//	}

//	tmp = ( this->z > Q.z ) ? ( this->z - Q.z ):( Q.z - this->z );
//	if ( ( tmp - ( qint64 )tmp ) >= 0.5 )
//	{
//		chek3 = ( qint64 )tmp + 1;
//	}
//	else
//	{
//		chek3 = ( qint64 )tmp;
//	}

//	if ( chek1 == 0 && chek2 == 0 && chek3 == 0 )
//	{
//		return true;
//	}
	return false;
}

bool GLPointDouble3D::operator!=(const GLPointDouble3D &Q)
{
    bool bRes = false; //result
    if (fabs(this->x - Q.x) > con_eps2
            || fabs(this->y - Q.y) > con_eps2
            || fabs(this->z - Q.z) > con_eps2)
    {
        bRes = true;
    }
    return bRes;
}

GLPointDouble3D GLPointDouble3D::operator -(const GLPoint3D& Q) const
{
    GLPointDouble3D temp;
    temp.x=this->x-(double)Q.x;
    temp.y=this->y-(double)Q.y;
    temp.z=this->z-(double)Q.z;
    return temp;
}

GLPointDouble3D GLPointDouble3D::operator -(const GLPointDouble3D& Q) const
{
    GLPointDouble3D temp;
    temp.x=this->x-Q.x;
    temp.y=this->y-Q.y;
    temp.z=this->z-Q.z;
    return temp;
}

GLPointDouble3D GLPointDouble3D::operator -() const
{
    GLPointDouble3D temp;
    temp.x = -x;
    temp.y = -y;
    temp.z = -z;
    return temp;
}

quint64 GLPointDouble3D::module() const
{
    return sqrt( x*x + y*y + z*z );
}

double GLPointDouble3D::moduleFloat() const
{
    return sqrt( x*x + y*y + z*z );
}

double GLPointDouble3D::ScalarProduct(const GLPointDouble3D &Q) const
{
    return x * Q.x + y * Q.y + z * Q.z;
}

GLPointDouble3D GLPointDouble3D::VectorProduct(const GLPointDouble3D &Q) const
{
    GLPointDouble3D Res;
    Res.x = this->y * Q.z - this->z * Q.y;
    Res.y = this->z * Q.x - this->x * Q.z;
    Res.z = this->x * Q.y - this->y * Q.x;
    return Res;
}

double GLPointDouble3D::getMax() const
{
    return std::max(x, std::max(y, z));
}

double GLPointDouble3D::getMin() const
{
    return std::min(x, std::min(y, z));
}

double GLPointDouble3D::getDistance(GLPointDouble3D &Q) const
{
    return sqrt( sqr(x-Q.x) + sqr(y-Q.y) + sqr(z-Q.z) );
}

double GLPointDouble3D::getDistance(GLPointFloat3D &Q) const
{
    return sqrt(  sqr(x - static_cast<double>(Q.x))
                + sqr(y - static_cast<double>(Q.y))
                + sqr(z - static_cast<double>(Q.z)));
}

double GLPointDouble3D::getDistanceOnSurface(GLPointDouble3D &Q) const
{
    double Dist = 0;
    if (!this->IsZero() && !Q.IsZero())
    {
        CGeocentric P1gc(x, y, z);
        CGeocentric P2gc(Q.x, Q.y, Q.z);
        CGeodesic P1gd, P2gd;
        bool bOK = GEOCENTRIC_GEO(&P1gc, &P1gd);
        bOK = bOK && GEOCENTRIC_GEO(&P2gc, &P2gd);
        if (bOK)
        {
            Dist = P1gd.GetGeodesicLine(P2gd);
        }
    }
    return Dist;
}

double GLPointDouble3D::getAngle(GLPointDouble3D &Q) const
{
    double Result = 0;
    double Denominator = this->moduleFloat() * Q.moduleFloat();
    if (fabs(Denominator) > con_par_eps)
    {
        double arg = this->ScalarProduct(Q) / Denominator;
        if (fabs(arg) > 1.)
        {            
        }
        else
        {
            Result = acos(arg);
        }
    }
    return Result;
}

double GLPointDouble3D::getProjectionOn(GLPointDouble3D &Q)
{
    double ProjThisOnQ = 0; //result
    double modQ = Q.moduleFloat();
    if (modQ > con_par_eps)
    {
        double AQ = this->ScalarProduct(Q); //scalar product of this and Q
        ProjThisOnQ = AQ / modQ;
    }
    return ProjThisOnQ;
}

bool GLPointDouble3D::IsZero() const
{
    bool bResult = false; //result
    if (fabs(x) < con_par_eps && fabs(y) < con_par_eps && fabs(z) < con_par_eps)
    {
        bResult = true;
    }
    return bResult;
}

QDataStream& operator<<(QDataStream& stream, const GLPointDouble3D& data)
{
    stream << data.x
           << data.y
           << data.z;
    return stream;
}

QDataStream& operator>>(QDataStream& stream, GLPointDouble3D& data)
{
    stream >> data.x
           >> data.y
           >> data.z;
    return stream;
}

/*********************************************************************

GLPointFloat3D implementation

**********************************************************************/
GLPointFloat3D::GLPointFloat3D( void ):
    x( 0 ), y( 0 ), z( 0 )
{
    //
}

GLPointFloat3D::GLPointFloat3D( const float& X, const float& Y, const float& Z ):
    x( X ), y( Y ), z( Z )
{
    //
}

void GLPointFloat3D::clear( void )
{
    x = 0.;
    y = 0.;
    z = 0.;
}

void GLPointFloat3D::init( const float& X, const float& Y, const float& Z )
{
    x = X;
    y = Y;
    z = Z;
}

//void GLPointFloat3D::init( const double& X, const double& Y, const double& Z )
//{
//    x = X;
//    y = Y;
//    z = Z;
//}

//void GLPointFloat3D::init( const qint32& X, const qint32& Y, const qint32& Z )
//{
//    x = (float) X;
//    y = (float) Y;
//    z = (float) Z;
//}

GLPointFloat3D GLPointFloat3D::operator+( GLPointFloat3D Q )
{
    GLPointFloat3D tmp;
    tmp.x = this->x + Q.x;
    tmp.y = this->y + Q.y;
    tmp.z = this->z + Q.z;
    return tmp;
}

GLPointFloat3D GLPointFloat3D::operator=( const GLPointFloat3D& Q )
{
    this->x = Q.x;
    this->y = Q.y;
    this->z = Q.z;
    return *this;
}
GLPointFloat3D GLPointFloat3D::operator *(const int& dig)const
{
    GLPointFloat3D tmp;
    tmp.x=this->x*dig;
    tmp.y=this->y*dig;
    tmp.z=this->z*dig;
    return tmp;
}

GLPointFloat3D GLPointFloat3D::operator *(const GLPointFloat3D& Q) const
{
    GLPointFloat3D temp;
    temp.x=this->x*Q.x;
    temp.y=this->y*Q.y;
    temp.z=this->z*Q.z;
    return temp;
}


GLPointFloat3D GLPointFloat3D::operator /(const int& dig) const
{
    GLPointFloat3D temp;
    temp.x=this->x/dig;
    temp.y=(this->y/dig);
    temp.z=(this->z/dig);
    return temp;
}


GLPointFloat3D GLPointFloat3D::operator=(int Q )
{
    this->x = ( float )( Q );
    this->y = ( float )( Q );
    this->z = ( float )( Q );
    return *this;
}


GLPointFloat3D GLPointFloat3D::copy( const GLPoint3D& Q )
{
    this->x = ( float )( Q.x );
    this->y = ( float )( Q.y );
    this->z = ( float )( Q.z );
    return *this;
}

GLPointFloat3D GLPointFloat3D::copy( const GLPointDouble3D& Q )
{
    this->x = ( float )( Q.x );
    this->y = ( float )( Q.y );
    this->z = ( float )( Q.z );
    return *this;
}

bool GLPointFloat3D::operator==( const GLPointFloat3D& Q )
{
    double tmp;
    qint64 chek1, chek2, chek3;

    tmp = ( this->x > Q.x ) ? ( this->x - Q.x ):( Q.x - this->x );
    if ( ( tmp - ( qint64 )tmp ) >= 0.5 )
    {
        chek1 = ( qint64 )tmp + 1;
    }
    else
    {
        chek1 = ( qint64 )tmp;
    }

    tmp = ( this->y > Q.y ) ? ( this->y - Q.y ):( Q.y - this->y );
    if ( ( tmp - ( qint64 )tmp ) >= 0.5 )
    {
        chek2 = ( qint64 )tmp + 1;
    }
    else
    {
        chek2 = ( qint64 )tmp;
    }

    tmp = ( this->z > Q.z ) ? ( this->z - Q.z ):( Q.z - this->z );
    if ( ( tmp - ( qint64 )tmp ) >= 0.5 )
    {
        chek3 = ( qint64 )tmp + 1;
    }
    else
    {
        chek3 = ( qint64 )tmp;
    }

    if ( chek1 == 0 && chek2 == 0 && chek3 == 0 )
    {
        return true;
    }

    return false;
}


GLPointFloat3D GLPointFloat3D::operator -(const GLPoint3D& Q) const
{
    GLPointFloat3D temp;
    temp.x=this->x-(float)Q.x;
    temp.y=this->y-(float)Q.y;
    temp.z=this->z-(float)Q.z;
    return temp;
}

GLPointFloat3D GLPointFloat3D::operator -(const GLPointFloat3D& Q) const
{
    GLPointFloat3D temp;
    temp.x=this->x-Q.x;
    temp.y=this->y-Q.y;
    temp.z=this->z-Q.z;
    return temp;
}

quint64 GLPointFloat3D::module()
{
    return sqrt( x*x + y*y + z*z );
}

bool GLPointFloat3D::IsZero()
{
//    if((qint32) x == 0 && (qint32) y == 0 && (qint32) z == 0)
//    {
//        return true;
//    }
//    return false;
    bool bResult = false; //result
    if (fabs(x) < con_par_eps && fabs(y) < con_par_eps && fabs(z) < con_par_eps)
    {
        bResult = true;
    }
    return bResult;
}

/*********************************************************************

                   GLAccel3D implementation

**********************************************************************/

GLAccel3D GLAccel3D::operator =(GLPoint3D Q)
{
 
	this->sAx=(float)Q.x;
	this->sAy=(float)Q.y;
	this->sAz=(float)Q.z;
	return *this;
}

GLAccel3D GLAccel3D::operator =(GLPointFloat3D Q)
{

    this->sAx=Q.x;
    this->sAy=Q.y;
    this->sAz=Q.z;
    return *this;
}

GLAccel3D GLAccel3D::copy(GLPointDouble3D Q)
{

    this->sAx=(float)Q.x;
    this->sAy=(float)Q.y;
    this->sAz=(float)Q.z;
    return *this;
}

GLAccel3D GLAccel3D::operator =(GLAccel3D Q)
{

	this->sAx = Q.sAx;
	this->sAy = Q.sAy;
	this->sAz = Q.sAz;
	return *this;
}

/*********************************************************************

                   GLVelocity3D implementation

**********************************************************************/

GLVelocity3D::GLVelocity3D( qint16 vx0, qint16 vy0, qint16 vz0 ) :
  vx(vx0), vy(vy0), vz(vz0)
{
  // Nothing to do.
}

GLVelocity3D GLVelocity3D::operator =(const qint16& Q)
{
	this->vx=Q;
	this->vy=Q;
	this->vz=Q;
	return *this;
}

GLVelocity3D GLVelocity3D::operator =(const GLVelocity3D& Q)
{
	this->vx=Q.vx;
	this->vy=Q.vy;
	this->vz=Q.vz;
	return *this;
}

GLVelocity3D GLVelocity3D::operator -(const GLVelocity3D& Q)
{
	GLVelocity3D temp;
	temp.vx=this->vx-Q.vx;
	temp.vy=this->vy-Q.vy;
	temp.vz=this->vz-Q.vz;
	return temp;
}

GLVelocity3D GLVelocity3D::operator -(const GLVelocityFloat3D& Q)
{
    GLVelocity3D temp;
    temp.vx=this->vx-(short)Q.vx;
    temp.vy=this->vy-(short)Q.vy;
    temp.vz=this->vz-(short)Q.vz;
    return temp;
}

GLVelocity3D GLVelocity3D::operator =(const GLPoint3D& Q)
{
	this->vx=(qint16)Q.x;
	this->vy=(qint16)Q.y;
	this->vz=(qint16)Q.z;
	return *this;
}

bool GLVelocity3D::operator !=(const int& dig)
{
	if((this->vx!=dig)||(this->vy!=dig)||(this->vz!=0))
		return true;
	return false;
}

GLVelocity3D GLVelocity3D::operator*(int Q)
{
	this->vx=this->vx*Q;
	this->vy=this->vy*Q;
	this->vz=this->vz*Q;
	return *this;
}
GLVelocity3D GLVelocity3D::operator*(double Q)
{
	this->vx=(int)this->vx*Q;
	this->vy=(int)this->vy*Q;
	this->vz=(int)this->vz*Q;
	return *this;
}

GLVelocity3D GLVelocity3D::operator +(const GLVelocity3D& Q)
{
	this->vx+=(qint32)Q.vx;
	this->vy+=(qint32)Q.vy;
	this->vz+=(qint32)Q.vz;
	return *this;
}
unsigned int GLVdistance( const GLVelocity3D& p1, const GLVelocity3D& p2 )
{
	return (unsigned int)sqrt(
		(double)(p1.vx - p2.vx)*(p1.vx - p2.vx) +
		(double)(p1.vy - p2.vy)*(p1.vy - p2.vy) +
		(double)(p1.vz - p2.vz)*(p1.vz - p2.vz));
}

/*********************************************************************

                   GLVelocityFloat3D implementation

**********************************************************************/

GLVelocityFloat3D::GLVelocityFloat3D( float vx0, float vy0, float vz0 ) :
  vx(vx0), vy(vy0), vz(vz0)
{
  // Nothing to do.
}

void GLVelocityFloat3D::clear( void )
{
    vx = 0.;
    vy = 0.;
    vz = 0.;
}

void GLVelocityFloat3D::init( const float& VX, const float& VY, const float& VZ )
{
    vx = VX;
    vy = VY;
    vz = VZ;
}

void GLVelocityFloat3D::init( const double& VX, const double& VY, const double& VZ )
{
    vx = VX;
    vy = VY;
    vz = VZ;
}

GLVelocityFloat3D GLVelocityFloat3D::operator =(const float& Q)
{
    this->vx=Q;
    this->vy=Q;
    this->vz=Q;
    return *this;
}

GLVelocityFloat3D GLVelocityFloat3D::operator =(const GLVelocityFloat3D& Q)
{
    this->vx=Q.vx;
    this->vy=Q.vy;
    this->vz=Q.vz;
    return *this;
}

GLVelocityFloat3D GLVelocityFloat3D::operator -(const GLVelocityFloat3D& Q)
{
    GLVelocityFloat3D temp;
    temp.vx=this->vx-Q.vx;
    temp.vy=this->vy-Q.vy;
    temp.vz=this->vz-Q.vz;
    return temp;
}

GLVelocityFloat3D GLVelocityFloat3D::operator -(const GLVelocity3D& Q)
{
    GLVelocityFloat3D temp;
    temp.vx=this->vx-(float)Q.vx;
    temp.vy=this->vy-(float)Q.vy;
    temp.vz=this->vz-(float)Q.vz;
    return temp;
}

GLVelocityFloat3D GLVelocityFloat3D::operator =(const GLPointFloat3D& Q)
{
    this->vx=(float)Q.x;
    this->vy=(float)Q.y;
    this->vz=(float)Q.z;
    return *this;
}

GLVelocityFloat3D GLVelocityFloat3D::operator*(int Q)
{
    this->vx=(float)this->vx*Q;
    this->vy=(float)this->vy*Q;
    this->vz=(float)this->vz*Q;
    return *this;
}
GLVelocityFloat3D GLVelocityFloat3D::operator*(double Q)
{
    this->vx=(float)this->vx*Q;
    this->vy=(float)this->vy*Q;
    this->vz=(float)this->vz*Q;
    return *this;
}

GLVelocityFloat3D GLVelocityFloat3D::operator +(const GLVelocityFloat3D& Q)
{
    this->vx+=Q.vx;
    this->vy+=Q.vy;
    this->vz+=Q.vz;
    return *this;
}

GLVelocityFloat3D GLVelocityFloat3D::copy( const GLPointDouble3D& Q )
{
    this->vx=(float)Q.x;
    this->vy=(float)Q.y;
    this->vz=(float)Q.z;
    return *this;
}


/*********************************************************************

                   GLBearing implementation

**********************************************************************/

GLBearing::GLBearing(GLPoint3D p)
{
	if (p.x == 0)
	{
		b = (p.z > 0) ? PI / 2.0 : 1.5 * PI;
	}
	else if (p.z == 0)
	{
		b = (p.x > 0) ? 0 : PI;
	}
	else
	{
		if ( (b = atan2( (double)p.z, (double)p.x )) < 0 ) b += 2.0 * PI;
	}

	e = atan2 ( (double)p.y,
						 sqrt( (double)p.x * (double)p.x + 
									 (double)p.z * (double)p.z ) );
}

GLBearing::GLBearing(GLPoint2D p)
{
	if (p.x == 0)
	{
		b = (p.y > 0) ? PI / 2.0 : 1.5 * PI;
	}
	else if (p.y == 0)
	{
		b = (p.x > 0) ? 0 : PI;
	}
	else
	{
		if ( (b = atan2( (double)p.y, (double)p.x )) < 0 ) b += 2.0 * PI;
	}
}

double GLbaseAngle(double begSect,double endSect)
{
	double BaseAngle = endSect - begSect;
	double PI = 3.1415926535897932384626433832795;
	if (PI < fabs(BaseAngle))
	{
		BaseAngle = (BaseAngle > 0) ? 	BaseAngle - 2*PI :	BaseAngle + 2*PI;
	}
	return fabs(BaseAngle);
}

double GLbisectAngle(double begSect,double endSect)
{
	double BaseAngle = GLbaseAngle( begSect, endSect);

	double PI = 3.1415926535897932384626433832795;

	double halfSect = BaseAngle / 2;
	double bisectAngle = 0.0;
	
	if ( ((endSect - begSect) == BaseAngle)
		|| (begSect - endSect) == (2*PI - BaseAngle) )
	{
		bisectAngle = begSect + halfSect;
	}
	else 
	{
		if ( (begSect - endSect) == BaseAngle
			|| (endSect - begSect) == (2*PI - BaseAngle) )
		{
			bisectAngle = endSect + halfSect;
		}
	}

	if(bisectAngle > 2*PI)  bisectAngle -= 2.0*PI;

	return bisectAngle;
}


void GLCoordToVector(const GLPointDouble3D &Coord, TVector<3> &ResVec)
{
    ResVec.Reset(3);
    ResVec.Vec[0] = Coord.x;
    ResVec.Vec[1] = Coord.y;
    ResVec.Vec[2] = Coord.z;
}


void GLCoordVelToVector(const GLPointDouble3D &Coord, const GLPointDouble3D &Vel, TVector<6> &ResVec)
{
    ResVec.Reset(6);
    ResVec.Vec[0] = Coord.x;
    ResVec.Vec[1] = Coord.y;
    ResVec.Vec[2] = Coord.z;
    ResVec.Vec[3] = Vel.x;
    ResVec.Vec[4] = Vel.y;
    ResVec.Vec[5] = Vel.z;
}


void GLCoordVelAccToVector(const GLPointDouble3D &Coord, const GLPointDouble3D &Vel, const GLPointDouble3D &Acc, TVector<9> &ResVec)
{
    ResVec.Reset(9);
    ResVec.Vec[0] = Coord.x;
    ResVec.Vec[1] = Coord.y;
    ResVec.Vec[2] = Coord.z;
    ResVec.Vec[3] = Vel.x;
    ResVec.Vec[4] = Vel.y;
    ResVec.Vec[5] = Vel.z;
    ResVec.Vec[6] = Acc.x;
    ResVec.Vec[7] = Acc.y;
    ResVec.Vec[8] = Acc.z;
}


void GLVectorToCoord(const TVector<3> &Vec, GLPointDouble3D &ResCoord)
{
    ResCoord.clear();
    ResCoord.x = Vec.Vec[0];
    ResCoord.y = Vec.Vec[1];
    ResCoord.z = Vec.Vec[2];
}


/*********************************************************************

                   GLBearingVelocity implementation

**********************************************************************/

double GLdistance( const GLPointDouble2D& p1, const GLPointDouble2D& p2 )
{
  return sqrt(
    (double)(p1.x - p2.x)*(p1.x - p2.x) +
    (double)(p1.y - p2.y)*(p1.y - p2.y));
}

unsigned int GLdistance( const GLPoint2D& p1, const GLPoint2D& p2 )
{
  return (unsigned int)sqrt(
    (double)(p1.x - p2.x)*(p1.x - p2.x) +
    (double)(p1.y - p2.y)*(p1.y - p2.y));
}

unsigned int GLdistance( const GLPoint3D& p1, const GLPoint3D& p2 )
{
  return (unsigned int)sqrt(
    (double)(p1.x - p2.x)*(p1.x - p2.x) +
    (double)(p1.y - p2.y)*(p1.y - p2.y) +
    (double)(p1.z - p2.z)*(p1.z - p2.z));
}

float GLdistance( const GLPointFloat3D& p1, const GLPointFloat3D& p2 )
{
  return sqrt(
    (float)(p1.x - p2.x)*(p1.x - p2.x) +
    (float)(p1.y - p2.y)*(p1.y - p2.y) +
    (float)(p1.z - p2.z)*(p1.z - p2.z));
}

float GLdistance( const GLPoint3D& p1, const GLPointFloat3D& p2 )
{
  return sqrt(
    (float)((float)p1.x - p2.x)*((float)p1.x - p2.x) +
    (float)((float)p1.y - p2.y)*((float)p1.y - p2.y) +
    (float)((float)p1.z - p2.z)*((float)p1.z - p2.z));
}

double GLdistance( const GLPoint3D& p )
{
  return sqrt( ((double)p.x * p.x) +
					((double)p.y * p.y) +
						 ((double)p.z * p.z) );
}

int GLdistance( const GLPointDouble3D& p1 , const GLPointDouble3D& p2)
{
    return (int)sqrt(
      (p1.x - p2.x)*(p1.x - p2.x) +
      (p1.y - p2.y)*(p1.y - p2.y) +
      (p1.z - p2.z)*(p1.z - p2.z));
}

unsigned int GLdistanceProjection( const GLPoint3D& p )
{
  return (unsigned int)sqrt( ((double)p.x * p.x) +
														 ((double)p.z * p.z) );
}

unsigned int GLabs( const GLVelocity3D& v )
{
  return (unsigned int)sqrt(
    (double)v.vx*v.vx + (double)v.vy*v.vy + (double)v.vz*v.vz);
}

unsigned int GLabsFlat( const GLVelocity3D& v )
{
  return (unsigned int)sqrt(
    (double)v.vx*v.vx + (double)v.vz*v.vz);
}

double GLazimuthChange(double newA, double prevA)
{
	if (newA >= 2*PI) newA -= 2*PI;
	if (prevA >= 2*PI) prevA -= 2*PI;

	double d = newA - prevA;
	if (PI < fabs(d)) d = (d > 0) ? d - 2*PI : d + 2*PI;

	return d;	
}

double GLmeetAngle(double a1, double a2)
{
	double meetAngle = GLazimuthChange(a1, a2);
	
	meetAngle = PI/2.0 < fabs(meetAngle) ? 
										PI - fabs(meetAngle):
										fabs(meetAngle);

	return meetAngle;
}

/*********************************************************************

                   GLPoint2D implementation

**********************************************************************/
bool GLPoint2D::operator==(GLPoint2D& p)
{
    if ((this->x == p.x) && (this->y == p.y))
        return true;
    return false;
}

GLPoint2D GLPoint2D::operator+(GLPoint2D p)
{
    GLPoint2D tmp;
    tmp.x = this->x + p.x;
    tmp.y = this->y + p.y;
    return tmp;
}

GLPoint2D GLPoint2D::operator=(const GLPoint2D& p)
{
    this->x = p.x;
    this->y = p.y;
    return *this;
}


/*********************************************************************

                   GLPointDouble2D implementation

**********************************************************************/
void GLPointDouble2D::init(const double& X,const double& Y)
{
	x=X;
	y=Y;
}

void GLPointDouble2D::clear()
{
    x = 0;
    y = 0;
}

GLPointDouble2D GLPointDouble2D::operator+(GLPointDouble2D Q)
{
	GLPointDouble2D tmp;
	tmp.x=this->x+Q.x;
	tmp.y=this->y+Q.y;
	return tmp;
}

GLPointDouble2D GLPointDouble2D::operator-(GLPointDouble2D Q)
{
    GLPointDouble2D tmp;
    tmp.x = this->x - Q.x;
    tmp.y = this->y - Q.y;
    return tmp;
}

GLPointDouble2D GLPointDouble2D::operator*(const double &dig) const
{
    GLPointDouble2D temp;
    temp.x = this->x * dig;
    temp.y = this->y * dig;
    return temp;
}

GLPointDouble2D GLPointDouble2D::operator=(const GLPointDouble2D& Q)
{
	this->x=Q.x;
	this->y=Q.y;
	return *this;
}

bool GLPointDouble2D::operator==(const GLPointDouble2D& Q)
{
	double tmp;
	qint64 chek1,chek2;
	tmp = (this->x > Q.x) ? (this->x-Q.x):(Q.x-this->x);
	if((tmp-(qint64)tmp)>=0.5)
			chek1=(qint64)tmp+1;
	else
		chek1=(qint64)tmp;
	tmp = (this->y > Q.y) ? (this->y-Q.y):(Q.y-this->y);
	if((tmp-(qint64)tmp)>=0.5)
			chek2=(qint64)tmp+1;
	else
		chek2=(qint64)tmp;
	
	if(chek1==0 && chek2==0)
		return true;
	return false;
}

double GLPointDouble2D::ScalarProduct(const GLPointDouble2D &Q)
{
    return this->x*Q.x + this->y*Q.y;
}

double GLPointDouble2D::module()
{
    return sqrt(x*x + y*y);
}

double GLPointDouble2D::getDistance(GLPointDouble2D &Q)
{
    return sqrt(sqr(x-Q.x) + sqr(y-Q.y));
}

double GLPointDouble2D::getAngle(GLPointDouble2D &Q)
{
    double Result = 0;
    double Denominator = this->module() * Q.module();
    if (fabs(Denominator) > con_par_eps)
    {
        double arg = this->ScalarProduct(Q) / Denominator;
        if (fabs(arg) > 1.)
        {
        }
        else
        {
            Result = acos(arg);
        }
    }
    return Result;
}

double GLPointDouble2D::getProjectionOn(GLPointDouble2D &Q)
{
    double ProjThisOnQ = 0; //result
    double modQ = Q.module();
    if (modQ > con_par_eps)
    {
        double AQ = this->ScalarProduct(Q); //scalar product of this and Q
        ProjThisOnQ = AQ / modQ;
    }
    return ProjThisOnQ;
}

bool GLPointDouble2D::IsZero()
{
    return (fabs(x) < con_par_eps && fabs(y) < con_par_eps);
}

//
std::pair<GLPoint3D, GLPoint3D> GLcroosLineCircuit(GLVelocity3D&  direct, GLPoint3D &firstPnt, GLPoint3D &secondPnt, quint32 radius)
{
	GLPoint3D onePnt;
	GLPoint3D otherPnt;


	if( firstPnt.x == secondPnt.x)
	{
		if(firstPnt.z != secondPnt.z)
		{
			onePnt.z = sqrt( (double)(radius * radius - firstPnt.x * firstPnt.x));
			otherPnt.z = -sqrt( (double)(radius * radius - firstPnt.x * firstPnt.x));
			onePnt.x = firstPnt.x;
			otherPnt.x = firstPnt.x;
		}
	}
	else
	{
		if( firstPnt.z == secondPnt.z)
		{
			onePnt.x = sqrt( (double)(radius * radius - firstPnt.z * firstPnt.z));
			otherPnt.x = -sqrt( (double)(radius * radius - firstPnt.z * firstPnt.z));
			onePnt.z = firstPnt.z;
			otherPnt.z = firstPnt.z;
		}
		else
		{
			//get coefficient of line x = z * k + m
			double k = (double)(firstPnt.x - secondPnt.x)/(double)(firstPnt.z - secondPnt.z);
			double m = (double)firstPnt.x - (double)firstPnt.z * k;
			
			// x*x + z*z = radius* radius
			// (z * k + m)*(z * k + m) + z*z - radius* radius = 0
			//get coefficints of quadratic z*z*a + z*b + c = 0
			double a = 1 + k * k;
			double b = 2 * k * m;
			double c = m * m - radius * radius;

			//get solution of quadratic
			onePnt.z = (sqrt( b*b - 4*a*c) - b) /(2*a);
			otherPnt.z = (-sqrt( b*b - 4*a*c) - b) /(2*a);
			onePnt.x = onePnt.z * k + m;
			otherPnt.x = otherPnt.z * k + m;
		}
	}
	if( (onePnt.x  * direct.vx + onePnt.z * direct.vz) < 0)
	{
        return std::make_pair( onePnt, otherPnt );
	}
	else
	{
        return std::make_pair( otherPnt, onePnt );
	}
}

GLPoint3D::GLPoint3D( qint32 x0, qint32 y0, qint32 z0 ) :
  x(x0), y(y0), z(z0)
{
  // Nothing to do.
}

void GLPoint3D::clear()
{
  x = 0;
  y = 0;
  z = 0;
}

/*GLPoint3D::GLPoint3D( GLPoint2D p ) :
  x(p.x), y(0), z(p.y)
{
  // Nothing to do.
}
*/

 GLPoint3D::GLPoint3D( GLBearing bearing, int distance)
{
    double proj = (double)distance * cos(bearing.e);

    y = qint32( (double)distance * sin(bearing.e) );
    x = qint32( proj * cos(bearing.b) );
    z = qint32( proj * sin(bearing.b) );
}

/* GLPoint2D::GLPoint2D( GLPoint3D p ) :
  x(p.x), y(p.z)
{
  // Nothing to do.
}
*/

// GLAccel3D::GLAccel3D( qint16 x0, qint16 y0, qint16 z0)
//{
//    sAx=x0; sAy=y0; sAz=z0;
//}

 GLAccel3D::GLAccel3D( float x0, float y0, float z0)
{
    sAx=x0; sAy=y0; sAz=z0;
}

 void GLAccel3D::clear()
 {
     sAx=0.; sAy=0.; sAz=0.;
 }

 void GLAccel3D::init( const float& AX, const float& AY, const float& AZ )
 {
     sAx = AX;
     sAy = AY;
     sAz = AZ;
 }

 void GLAccel3D::init( const double& AX, const double& AY, const double& AZ )
 {
     sAx = AX;
     sAy = AY;
     sAz = AZ;
 }

 GLBearing::GLBearing( double b0, double e0 ) :
  b(b0), e(e0)
{
  // Nothing to do.
}

 GLBearingVelocity::GLBearingVelocity( double vb0, double ve0 ) :
  vb(vb0), ve(ve0)
{
  // Nothing to do.
}

 GLPointLL::GLPointLL( double lt0, double lg0, int h0 ) :
  lt(lt0), lg(lg0), h(h0)
  {
      // Nothing to do.
  }

/*********************************************************************

            GLPointLL implementation

**********************************************************************/
GLPointLL GLPointLL::operator =(const GLPointLL& pt)
{
	this->lt=pt.lt;
	this->lg=pt.lg;
	this->h=pt.h;
	return *this;
}

/*********************************************************************

            GLAngle3D implementation

**********************************************************************/
GLAngle3D::GLAngle3D(double R0, double b0,	double e0)
{
    R = R0;
    b = b0;
    e = e0;
}

GLAngle3D::GLAngle3D(GLPoint3D p)
{
    R = sqrt((double)p.x * (double)p.x +
             (double)p.y * (double)p.y +
             (double)p.z * (double)p.z );

    if (p.x == 0)
    {
        b = (p.z > 0) ? PI / 2.0 : 1.5 * PI;
    }
    else if (p.z == 0)
    {
        b = (p.x > 0) ? 0 : PI;
    }
    else
    {
        if ( (b = atan2( (double)p.z, (double)p.x )) < 0 ) b += 2.0 * PI;
    }

    e = atan2 ( (double)p.y,
                         sqrt( (double)p.x * (double)p.x +
                                     (double)p.z * (double)p.z ) );
}

GLAngle3D::GLAngle3D(GLPoint2D p)
{
    R = 0.;

    if (p.x == 0)
    {
        b = (p.y > 0) ? PI / 2.0 : 1.5 * PI;
    }
    else if (p.y == 0)
    {
        b = (p.x > 0) ? 0 : PI;
    }
    else
    {
        if ( (b = atan2( (double)p.y, (double)p.x )) < 0 ) b += 2.0 * PI;
    }
}

void GLAngle3D::clear()
{
    R = 0.;
    b = 0.;
    e = 0.;
}


GLAngle3D GLAngle3D::operator =(GLAngle3D Q)
{

    this->R = Q.R;
    this->b = Q.b;
    this->e = Q.e;
    return *this;
}

/******************************************************/
/******************angle function**********************/

double AB_2PI(double b1, double b2)
{
    double b = b1 + b2;
    if(b >= 2*PI)
    {
        b -= 2*PI;
    }
    return b;
}


double AB_PI(double b1, double b2)
{
    double b = b1 + b2;
    if(b > PI)
    {
        b = 2*PI - b;
    }
    else if(b < -PI)
    {
        b = -PI - b;
    }
    else
    {
    }
    return b;
}


double SB_2PI(double b1, double b2)
{
    double b = b1 - b2;
    if(b>0)
    {
        if(b>PI)
        {
            b -= 2*PI;
        }
    }
    else
    {
        if(b<-PI)
        {
            b += 2*PI;
        }
    }
    return b;
}


double SB_PI(double b1, double b2)
{
    double b = b1 - b2;
    if(b > PI)
    {
        b = 2*PI - b;
    }
    else if(b < -PI)
    {
        b = -PI - b;
    }
    else
    {
    }
    return b;
}


double SMODB_2PI(double b1, double b2)
{
    return fabs(SB_2PI(b1,b2));
}


double SMODB_PI(double b1, double b2)
{
    return fabs(SB_PI(b1,b2));
}


double AB_Interval_0_2Pi(double b1, double b2)
{
    double b = b1 + b2;
    if (b >= 2.*PI || b < 0)
    {
        double quotient = floor(b/(2.*PI));
        b -= 2. * PI * quotient;
    }
    return b;
}


double SB_Interval_0_2Pi(double b1, double b2)
{
    double b = b1 - b2;
    if (b >= 2.*PI || b < 0)
    {
        double quotient = floor(b/(2.*PI));
        b -= 2. * PI * quotient;
    }
    return b;
}


double SMODB_Interval_0_2Pi(double b1, double b2)
{
    return fabs(SB_Interval_0_2Pi(b1, b2));
}


double CorrAngle_Interval_Minus2Pi_2Pi(double angle)
{
    double Res = angle;
    if (angle >= 2*PI || angle <= -2*PI)
    {
        double quotient = floor(angle/(2.*PI));
        Res -= 2.* PI * quotient;
    }
    return Res;
}


double calcGravitationalAcceleration(double H)
{
    double g_H = con_g;
    double R = con_Eath_middle_radius + H;
    if (fabs(R) > con_eps2)
    {
        g_H = con_Grav_Const * con_Earth_Mass / sqr(R);
    }
    return g_H;
}


/******************************************************/

qint16 D2_I16(double dig)
{
    if(dig>=0){
        if((dig-(quint16)dig)>=0.5)
            return (quint16)dig+1;
        return (quint16)dig;
    }
    else{
        if((dig-(quint16)dig)<=-0.5)
            return (quint16)dig-1;
        return (quint16)dig;
    }
}

quint16 count_Azim(GLPointDouble2D& pnt)
 {
    double Cosalfa;
    quint16 alfa;


    if(((pnt.x>=0)&&(pnt.y>=0))||((pnt.x<0)&&(pnt.y>0))){
        Cosalfa=pnt.x/sqrt((double)pnt.x*pnt.x + (double)pnt.y*pnt.y);
        alfa=60*D2_I16(acos(Cosalfa)*180/PI);
    }

    else{
        Cosalfa=-1.0*pnt.x/sqrt((double)pnt.x*pnt.x + (double)pnt.y*pnt.y);
        alfa=60*D2_I16(acos(Cosalfa)*180/PI)+10800;
    }
    return alfa;
 }

bool CourseInSector(qint16 course,qint16 storona1,qint16 storona2)
{
    if (abs(storona1 - storona2) < 180 * 60)
    {
        if ((std::min(storona1,storona2) <= course) && (course <= std::max(storona1,storona2)))
            return true;
        else
            return false;
    }
    else
    {
        if (((std::max(storona1,storona2) <= course) && (course <= 360 * 60)) && ((0 <= course) && (course <= std::min(storona1,storona2))))
            return true;
        else
            return false;
    }
}
