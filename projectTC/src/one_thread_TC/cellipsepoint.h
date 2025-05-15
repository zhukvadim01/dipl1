#ifndef CELLIPSEPOINT_H
#define CELLIPSEPOINT_H

#include <cmath>
#include <iostream>
#include <algorithm>

#include <QtGlobal>

//PACKAGE       :   AirBallist
//STRUCTURE     :   CEllipsePoint
//DESCRIPTION	:   class utilizes info about ellipse and point on the plane
//                  gives functions to calculate
class CEllipsePoint
{
    // Ellipse and point P in Z-X plane.
    // Ellipse centered at the origin.
    // Its major and minor semi-axis have length a and b,
    // and beta is the angle between X axis and ellipses major semi-axis.
    // With ellipse and point specified, one can calculate:
    // CEllipsePoint::calcR(qreal &R1, qreal &R2)
    // R1, R2 - minimum and maximum range from P to ellipse,
    // CEllipsePoint::calcAngles(qreal &alpha, qreal &u)
    // alpha - angle between tangents to ellipse passed through P,
    // u - angle between bisector of alpha and axis X (azimuth of bisector of alpha)

public:
    static const qreal epsilon;
    static const qreal tolerance;
    static const qint32 max_iterations;

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CEllipsePoint::CEllipsePoint
    //DESCRIPTION	:   Constructor
    //INPUTS		:   b - length of minor semi-axis of ellipse (which aligned with Z axis)
    //                  a - length of major semi-axis of ellipse (which aligned with X axis)
    //                  (Pz,Px) - coordinates of point P in Z-X plane
    //RETURNS		:   None
    CEllipsePoint(qreal b, qreal a, qreal Pz, qreal Px);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CEllipsePoint::CEllipsePoint
    //DESCRIPTION	:   Constructor
    //INPUTS		:   bI - length of minor semi-axis of ellipse (which is NOT aligned with Z axis)
    //                  aI - length of major semi-axis of ellipse (which is NOT aligned with X axis)
    //                  betaI - angle between X axis and major semi-axis of ellipse
    //                  (PzI,PxI) - coordinates of point P in Z-X plane
    //RETURNS		:   None
    CEllipsePoint(qreal bI, qreal aI, qreal betaI, const qreal PzI, const qreal PxI);

private:
    qreal b;   // Length of minor semi-axis (which coinside with Z' axis after rotation)
    qreal a;   // Length of major semi-axis (which coinside with X' axis after rotation)
    qreal beta;    // Angle between major semi-axis and X axis (before rotation)
    qreal Pz;      // (Pz, Px) - coordinates of point P in Z'-X' plane (after rotation)
    qreal Px;

    //help functions
    qreal z(const qreal t);
    qreal x(const qreal t);
    qreal tan_f(const qreal t);
    qreal nsolve_radius_min(qreal T1, qreal T2);
    qreal nsolve_radius_max(qreal T1, qreal T2);
    qreal nsolve_tangent(const qreal T0, const qreal T1);
    qreal anglebetweenvectors(const qreal v0[], const qreal v1[]);
    qreal nsolve_radius(const qreal T1, const qreal T2);
    qreal norm_f(const qreal t);

public:


    //PACKAGE		:   AirBallist
    //FUNCTION		:   CEllipsePoint::calcR
    //DESCRIPTION	:   find minimal and maximal distances from point P to ellipse
    //INPUTS		:   reference to R1 - minimal radius form P to ellipse
    //                  reference to R1 - maximal radius form P to ellipse
    //RETURNS		:   void
    void calcR(qreal &R1, qreal &R2);

    //PACKAGE		:   AirBallist
    //FUNCTION		:   CEllipsePoint::calcAngles
    //DESCRIPTION	:   find minimal and maximal distances from point P to ellipse
    //INPUTS		:   reference to alpha - angle between tangents to ellipse passed through P
    //                  reference to u - angle between bisector of alpha and axis X
    //                  (azimuth of bisector of alpha)
    //RETURNS		:   void
    void calcAngles(qreal &alpha, qreal &u);

    bool is_inside();
};
#endif
