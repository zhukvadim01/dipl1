#include "cellipsepoint.h"

const qreal CEllipsePoint::epsilon = 0.0001;
const qreal CEllipsePoint::tolerance = 0.00001;
const qint32 CEllipsePoint::max_iterations = 100;
CEllipsePoint::CEllipsePoint(qreal b_, qreal a_, qreal Pz_, qreal Px_)
                :b(b_), a(a_), beta(0), Pz(Pz_), Px(Px_)
{

}
CEllipsePoint::CEllipsePoint(qreal bI, qreal aI, qreal betaI, const qreal PzI, const qreal PxI)
                :b(bI), a(aI), beta(betaI)
{
    //we need to rotate ellipse to align those major semiaxes with x axes
    qreal R[2][2] = {{cos(beta),-sin(beta)},{sin(beta),cos(beta)}};	//rotation matrix. rotate on beta CCW
    Pz = R[0][0] * PzI + R[0][1] * PxI;
    Px = R[1][0] * PzI + R[1][1] * PxI;

}
qreal CEllipsePoint::z(const qreal t)
{
    return b*cos(t);
}

qreal CEllipsePoint::x(const qreal t)
{
    return a*sin(t);
}

void CEllipsePoint::calcR(qreal &R1, qreal &R2)
{
    if(is_inside())
    {
        //std::cout << "\npoint inside ellipse";
        R1 = 0;
        R2 = 0;
        return;
    }
    qreal N;
    if(Px >= 0)
    {
        if(Pz >= 0)
        {
            N = 0;
        }
        else
        {
            N = 1;
        }
    }
    else
    {
        if(Pz >= 0)
        {
            N = 3;
        }
        else
        {
            N = 2;
        }
    }

    const qreal T1 = N*M_PI_2;
    const qreal T2 = N*M_PI_2 + M_PI_2;
    R1 = nsolve_radius(T1,T2);
    R2 = nsolve_radius(T1+M_PI, T2+M_PI);

}

qreal CEllipsePoint::norm_f(const qreal t) {return (a*a-b*b)*sin(t)*cos(t) - Px*a*cos(t) + Pz*b*sin(t);}
qreal CEllipsePoint::nsolve_radius(const qreal T1, const qreal T2)
{
    qreal t0 = T1;
    qreal t1 = T2;
    for(qint32 i = 0; i<max_iterations; i++)
    {
        const qreal dt = t1 - t0;
        const qreal df = norm_f(t1) - norm_f(t0);
        if(std::abs(t1-t0) <= tolerance)
        {
            t0 = t1;
            break;
        }
        if(std::abs(df) <= epsilon)
        {
            t0 = t1;
            break;
        }
        const qreal t2 = t1 - norm_f(t1)*dt/df;
        t0 = t1;
        t1 = t2;
    }
    return pow(pow(z(t0)-Pz,2)+pow(x(t0)-Px,2),0.5);

}


qreal CEllipsePoint::tan_f(const qreal t)
{
    return b*a - Pz*a*cos(t) - Px*b*sin(t);
}

qreal CEllipsePoint::nsolve_tangent(const qreal T0, const qreal T1)
{
    qreal t0 = T0;
    qreal t1 = T1;
    for(qint32 i = 0; i<max_iterations; i++)
    {
        const qreal dt = t1 - t0;
        const qreal df = tan_f(t1) - tan_f(t0);
        if(std::abs(t1-t0) <= tolerance)
        {
            t0 = t1;
            break;
        }
        if(std::abs(df) <= epsilon)
        {
            t0 = t1;
            break;
        }
        const qreal t2 = t1 - tan_f(t1)*dt/df;
        t0 = t1;
        t1 = t2;
    }
    return t0;
}
void CEllipsePoint::calcAngles(qreal &alpha, qreal &u)
{
    if(is_inside())
    {
        alpha = 0;
        u = 0;
        return;
    }
    const qreal tP1 = atan((b*Px)/(a*Pz));
    const qreal N = tP1 - fmod(tP1, M_PI_2);
    qreal T0 = tP1;
    qreal T1 = N*M_PI_2 + M_PI;
    const qreal t1 = nsolve_tangent(T0,T1);

    T0 = N*M_PI_2 - M_PI_2;
    T1 = tP1;
    const qreal t2 = nsolve_tangent(T0,T1);

    const qreal v1[] = {z(t1) - Pz, x(t1) - Px};
    const qreal v2[] = {z(t2) - Pz, x(t2) - Px};
    const qreal v0[] = {0,1};
    const qreal u1 = anglebetweenvectors(v0,v1);
    const qreal u2 = anglebetweenvectors(v0,v2);
    alpha = std::abs((u1 - u2));
    u = fmod((u1 + u2)/2 + beta, 2*M_PI);


}

qreal CEllipsePoint::anglebetweenvectors(const qreal v0[2], const qreal v1[2])
{

    const qreal nv0nv1 = pow((v0[0]*v0[0] + v0[1]*v0[1])*(v1[0]*v1[0] + v1[1]*v1[1]),0.5);
    const qreal cosA = (v0[0]*v1[0] + v0[1]*v1[1])/nv0nv1;
    const qreal sinA = (v0[0]*v1[1] - v0[1]*v1[0])/nv0nv1;
    qreal A;
    if(sinA <= 0)
    {
        A = acos(cosA);
    }
    else
    {
        A = 2*M_PI - acos(cosA);
    }

    return A;
}

bool CEllipsePoint::is_inside()
{
    if(pow(Pz,2)/pow(b,2)+pow(Px,2)/pow(a,2) <= 1.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
