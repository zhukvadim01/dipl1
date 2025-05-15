#include <QDataStream>
#include "GLTrackPoint.h"
#include "common/CODefine.h"

using namespace GLFileLog;

GLTrackPoint::GLTrackPoint():
    coord(0,0,0),
    vel(0,0,0),
    accel(0,0,0),
    time(0)
{
    //nothing to do
}

void GLTrackPoint::clear()
{
    coord.x = 0;
    coord.y = 0;
    coord.z = 0;
    vel.vx = 0;
    vel.vy = 0;
    vel.vz = 0;
    accel.sAx = 0;
    accel.sAy = 0;
    accel.sAz = 0;
    time = 0;

}

GLTrackPointFloat::GLTrackPointFloat():
    coord(0.,0.,0.),
    vel(0.,0.,0.),
    accel(0.,0.,0.),
    time(0)
{
    //nothing to do
}

void GLTrackPointFloat::clear()
{
    coord.x = 0.;
    coord.y = 0.;
    coord.z = 0.;
    vel.vx = 0.;
    vel.vy = 0.;
    vel.vz = 0.;
    accel.sAx = 0.;
    accel.sAy = 0.;
    accel.sAz = 0.;
    time = 0;

}

GLTrackPointFloat GLTrackPointFloat::copy( const GLTrackPointDouble& Q )
{
    this->coord.copy(Q.coord);
    this->vel.copy(Q.vel);
    this->accel.copy(Q.accel);
    this->time = Q.time;
	return *this;
}

GLTrackPointFloat& GLTrackPointFloat::operator=( const GLTrackPointFloat& Q )
{
    this->coord = Q.coord;
    this->vel = Q.vel;
    this->accel = Q.accel;
    this->time = Q.time;
	return *this;
}


GLTrackPointDouble::GLTrackPointDouble()/*:
    coord(0.,0.,0.),
    vel(0.,0.,0.),
    accel(0.,0.,0.),
    time(0)*/
{
    //nothing to do
}

void GLTrackPointDouble::clear()
{
    coord.clear();
    vel.clear();
    accel.clear();
    time = 0;
}

GLTrackPointDouble GLTrackPointDouble::copy( const GLTrackPointFloat& Q )
{
    this->coord.copy(Q.coord);
    this->vel.copy(Q.vel);
    this->accel.copy(Q.accel);
    this->time = Q.time;
	return *this;
}

GLTrackPointDouble& GLTrackPointDouble::operator=( const GLTrackPointDouble& Q )
{
    this->coord = Q.coord;
    this->vel = Q.vel;
    this->accel = Q.accel;
    this->time = Q.time;
	return *this;
}


bool GLTrackPointDouble::Interpolation(qint64 &GivenTime, GLTrackPointDouble &Point2, GLTrackPointDouble &ResPoint)
{
    bool bOK = true;

    if (GivenTime == time)
    {
        ResPoint.time = time;
        ResPoint.coord = coord;
        ResPoint.vel = vel;
        ResPoint.accel = accel;
    }
    else if (GivenTime == Point2.time)
    {
        ResPoint.time = Point2.time;
        ResPoint.coord = Point2.coord;
        ResPoint.vel = Point2.vel;
        ResPoint.accel = Point2.accel;
    }
    else
    {
        if (fabs(double(time - Point2.time)) == 0)
        {
            bOK = false;
        }
        else
        {
            ResPoint.time = GivenTime;

            double GivenTime_s = MSEC70_to_SEC70(GivenTime); //given time in seconds
            double Time1_s = MSEC70_to_SEC70(time); //1st time in seconds
            double Time2_s = MSEC70_to_SEC70(Point2.time); //2nd time in seconds

            bool bOK_curr;
            bOK_curr = Interpolation_1coord(GivenTime_s, Time1_s, Time2_s, coord.x, vel.x, accel.x, Point2.coord.x, Point2.vel.x,
                                            Point2.accel.x, ResPoint.coord.x, ResPoint.vel.x, ResPoint.accel.x);
            bOK &= bOK_curr;
            bOK_curr = Interpolation_1coord(GivenTime_s, Time1_s, Time2_s, coord.y, vel.y, accel.y, Point2.coord.y, Point2.vel.y,
                                            Point2.accel.y, ResPoint.coord.y, ResPoint.vel.y, ResPoint.accel.y);
            bOK &= bOK_curr;
            bOK_curr = Interpolation_1coord(GivenTime_s, Time1_s, Time2_s, coord.z, vel.z, accel.z, Point2.coord.z, Point2.vel.z,
                                            Point2.accel.z, ResPoint.coord.z, ResPoint.vel.z, ResPoint.accel.z);
            bOK &= bOK_curr;
        }
    }
    return bOK;

}


void GLTrackPointDouble::Extrapolation(qint64 GivenTime, GLTrackPointDouble &ResPoint)
{
    ResPoint.time = GivenTime;
    double dt = static_cast<double>(GivenTime - this->time) /1000.;

    ResPoint.coord = coord + vel*dt + accel*dt*dt*0.5;
    ResPoint.vel = vel + accel*dt;
    ResPoint.accel = accel;
}


void GLTrackPointDouble::tLogData(FILE *_pLogFile, const char *_Name)
{
    if (_pLogFile != NULL)
    {
        tLog(_pLogFile, "%s %lld %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             _Name, time, coord.x, coord.y, coord.z, vel.x, vel.y, vel.z, accel.x, accel.y, accel.z);
    }
}


bool GLTrackPointDouble::Interpolation_1coord(double t_given, double t1, double t2,
                                              double u1, double vu1, double au1,
                                              double u2, double vu2, double au2,
                                              double &u_res, double &vu_res, double &au_res)
{
    bool bOK = true;

    double dt = t2 - t1;
    if (fabs(dt) < con_par_eps) {
        bOK = false;
    }
    else
    {
        double du21  = u2 - (0.5*au1*dt*dt + vu1*dt + u1);
        double dvu21 = vu2 - (au1*dt + vu1);
        double dau21 = au2 - au1;

        double p = 720.*du21/(dt*dt*dt*dt*dt) - 360.*dvu21/(dt*dt*dt*dt) + 60*dau21/(dt*dt*dt);
        double q = -360.*du21/(dt*dt*dt*dt) + 168.*dvu21/(dt*dt*dt) - 24.*dau21/(dt*dt);
        double r = 60.*du21/(dt*dt*dt) - 24.*dvu21/(dt*dt) + 3.*dau21/dt;

        double dt_res = t_given - t1;
        u_res = p*(dt_res*dt_res*dt_res*dt_res*dt_res)/120. + q*(dt_res*dt_res*dt_res*dt_res)/24.
                + r*(dt_res*dt_res*dt_res)/6. + au1*(dt_res*dt_res)/2. + vu1*dt_res + u1;
        vu_res = p*(dt_res*dt_res*dt_res*dt_res)/24. + q*(dt_res*dt_res*dt_res)/6.
                + r*(dt_res*dt_res)/2. + au1*dt_res + vu1;
        au_res = p*(dt_res*dt_res*dt_res)/6. + q*(dt_res*dt_res)/2. + r*dt_res + au1;
    }

    return bOK;
}


QDataStream& operator<<(QDataStream& stream, const GLTrackPointDouble& data)
{
    stream << data.coord
           << data.vel
           << data.accel
           << static_cast<const qint64>(data.time);
    return stream;
}

QDataStream& operator>>(QDataStream& stream, GLTrackPointDouble& data)
{
    stream >> data.coord
           >> data.vel
           >> data.accel;

    qint64 l_time;      stream >> l_time;
    data.time = l_time;
    return stream;
}


GLPoint_tXYZVTh::GLPoint_tXYZVTh()
{
//    clear();
}


void GLPoint_tXYZVTh::clear()
{
    time = 0;
    coord.clear();
    V = 0;
    Theta = 0;
}


GLPoint_tXYZVTh& GLPoint_tXYZVTh::operator=(const GLPoint_tXYZVTh &Q)
{
    this->time  = Q.time;
    this->coord = Q.coord;
    this->V     = Q.V;
    this->Theta = Q.Theta;
    return *this;
}


bool GLPoint_tXYZVTh::isEqualTo(GLPoint_tXYZVTh &Q)
{
    bool bRes = false;

    if (this->time == Q.time)
    {
        if (fabs(this->coord.x - Q.coord.x) < con_eps2
                && fabs(this->coord.y - Q.coord.y) < con_eps2
                && fabs(this->coord.z - Q.coord.z) < con_eps2
                && fabs(this->V - Q.V) < con_eps2
                && fabs(this->Theta - Q.Theta) < con_eps2)
        {
            bRes = true;
        }
    }
    return bRes;
}


bool GLPoint_tXYZVTh::isEmpty()
{
    bool bRes = false;
    if (time == 0 || coord.IsZero())
    {
        bRes = true;
    }
    return bRes;
}


void GLPoint_tXYZVTh::tLogData(FILE *_pLogFile, const char *_Name)
{
    if (_pLogFile != NULL)
    {
        tLog(_pLogFile, "%s %lld %f %f %f %f %f",
             _Name, time, coord.x, coord.y, coord.z, V, Theta);
    }
}

QDataStream& operator<<(QDataStream& stream, const GLPoint_tXYZVTh& data)
{
    stream << static_cast<const qint64>(data.time)
           << data.coord
           << data.V
           << data.Theta;
    return stream;
}

QDataStream& operator>>(QDataStream& stream, GLPoint_tXYZVTh& data)
{
    qint64 l_time;      stream >> l_time;
    data.time = l_time;

    stream >> data.coord
           >> data.V
           >> data.Theta;
    return stream;

}
