#ifndef GLELLIPSE2D_H
#define GLELLIPSE2D_H

#include <QtGlobal>

#include "GLMath.h"
#include "GLArray.h"
#include "GLSolve_eq_2_3_4_deg.h"

// PACKAGE		:   GL
// STRUCTURE 	:   GLEllipse2D
// DESCRIPTION	:   Represent ellipse in 2D space and some calculations for it
//              :   Usage is to perform calculations, not to store data
class GLEllipse2D
{
private:
    qreal m_R[2][2];
    qreal m_invR[2][2];

public:
    qreal m_b; //small semiaxis, m
    qreal m_a; //large semiaxis, m
    const qreal m_angle; //azimuth of large semiaxis, radians
    qreal m_Z0;
    qreal m_X0;

    qreal Quadratic (const qreal &Z, const qreal &X) const;

    GLEllipse2D(const qreal b,
                const qreal a,
                const qreal angle,
                const qreal Z0,
                const qreal X0);

    qreal unZt(const qreal t) const;    //Z(t) for unrotated ellipse
    qreal unXt(const qreal t) const;    //X(t) for unrotated ellipse

    qreal Zt(const qreal t) const;      //Z(t) paramitric equation of ellipse
    qreal Xt(const qreal t) const;      //X(t)

    bool IsPointInside(const qreal &Z, const qreal &X) const;
    friend bool IsEllipseIntersectEllipse ( const GLEllipse2D & e0, const GLEllipse2D & e1);

    // PACKAGE		: GL
    // FUNCTION		: GLEllipse2D::DistanceToPoint()
    // DESCRIPTION	: Calculates distance from current ellipse to the given point
    // INPUTS		: Coordinates Z and X of given point, m, in NUE coordinate system
    // RETURN		: Distance from current ellipse to the given point
    qreal DistanceToPoint(qreal &Zp, qreal &Xp);

    // PACKAGE		: GL
    // FUNCTION		: GLEllipse2D::IntersectsEllipse()
    // DESCRIPTION	: Checks whether the current ellipse intersects second given ellipse,
    //              : inclding the case when one ellipse is a subset of another
    // INPUTS		: Second given ellipse; reference to the resulting coordinates of intersection points;
    //              : reference to the resulting quantity of roots, if counted with their multiplicities (-1 in the case of infinity number of roots)
    // RETURN		: True if the current ellipse intersects second given ellipse
    bool IntersectsEllipse(GLEllipse2D &Ell2, CGLArrayFix<GLComplexNumb, 4> &arrX, CGLArrayFix<GLComplexNumb, 4> &arrZ, qint16 &NRoots);

    bool IntersectsEllipse(GLEllipse2D &Ell2);


};

#endif // GLELLIPSE2D_H
