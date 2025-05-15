#include "GLEllipse2D.h"
#include "Constants.h"

GLEllipse2D::GLEllipse2D(const qreal b,
                         const qreal a,
                         const qreal angle,
                         const qreal Z0,
                         const qreal X0):
    m_b(b), m_a(a), m_angle(angle), m_Z0(Z0), m_X0(X0)
{
    m_R[0][0] = cos(m_angle);   m_R[0][1] = sin(m_angle);
    m_R[1][0] =-sin(m_angle);   m_R[1][1] = cos(m_angle);

    m_invR[0][0] = cos(m_angle);   m_invR[0][1] =-sin(m_angle);
    m_invR[1][0] = sin(m_angle);   m_invR[1][1] = cos(m_angle);

    return;
}

qreal GLEllipse2D::Quadratic(const qreal &Z, const qreal &X) const
{//all const qualifiers demanded by MISRA standard
    const qreal z1 = Z - m_Z0;  //centralize
    const qreal x1 = X - m_X0;

    const qreal z2 = m_invR[0][0]*z1 + m_invR[0][1]*x1;   //rotate
    const qreal x2 = m_invR[1][0]*z1 + m_invR[1][1]*x1;

    const qreal f = pow(z2,2)/pow(m_b,2) + pow(x2,2)/pow(m_a,2) - 1;

    return f;
}

qreal GLEllipse2D::unZt(const qreal t) const
{
    return m_b*cos(t) + m_Z0;
}

qreal GLEllipse2D::unXt(const qreal t) const
{
    return m_a*sin(t) + m_X0;
}

qreal GLEllipse2D::Zt(const qreal t) const
{
    return m_R[0][0]*unZt(t) + m_R[0][1]*unXt(t);
}

qreal GLEllipse2D::Xt(const qreal t) const
{
    return m_R[1][0]*unZt(t) + m_R[1][1]*unXt(t);
}

bool GLEllipse2D::IsPointInside(const qreal &Z, const qreal &X) const
{
    return Quadratic(Z,X)<=0.0;
}

bool IsEllipseIntersectEllipse ( const GLEllipse2D & e0, const GLEllipse2D & e1)
{
    const qint32 N = 45;   //maximum amount of points to check
    if(e0.IsPointInside(e1.m_Z0,e1.m_X0))
    {
        return true;
    }

    if (e1.IsPointInside(e0.m_Z0, e0.m_X0))
    {
        return true;
    }

    qreal dCenters = sqrt(sqr(e0.m_X0 - e1.m_X0) + sqr(e0.m_Z0 - e1.m_Z0)); //distance between centers of the ellipses
    if (dCenters > std::max(e0.m_a, e0.m_b) + std::max(e1.m_a, e1.m_b))
    {
        return false;
    }

    for( qint32 i = 0; i < N; i++)
    {
        qreal t = static_cast<qreal>(i);
        if(e0.IsPointInside(e1.Zt(t),e1.Xt(t)))
        {
            return true;
        }
    }
    return false;
}


qreal GLEllipse2D::DistanceToPoint(qreal &Zp, qreal &Xp)
{
    qreal Dist = 0;
    qreal Zp_shift = Zp - this->m_Z0; //shift to the coordinate system with
    qreal Xp_shift = Xp - this->m_X0; //center in (m_Z0, m_X0)

    qreal f = (this->m_a - this->m_b) / this->m_a; //ellipse flattening

    qreal rE = sqrt(sqr(Zp_shift) + sqr(Xp_shift)); //distance from the center of ellipse to the given point

    if (rE > con_par_eps)
    {
        qreal ZpE = m_invR[0][0]*Zp_shift + m_invR[0][1]*Xp_shift;   //rotate
//        qreal XpE = m_invR[1][0]*Zp_shift + m_invR[1][1]*Xp_shift;

        Dist = rE - this->m_a * (1.-f) / sqrt(sqr(1.-f) + (2.*f-f*f)*sqr(ZpE)/sqr(rE));
    }

    return Dist;
}

bool GLEllipse2D::IntersectsEllipse(GLEllipse2D &Ell2, CGLArrayFix<GLComplexNumb, 4> &arrX, CGLArrayFix<GLComplexNumb, 4> &arrZ, qint16 &NRoots)
{
    //1. If the center of one ellipse is inside anoter ellipse: there is an intersection.
    //2. If the distance between centers of 2 ellipses exsceeds sum of large semiaxis:
    //   there is no intersection.
    //3. In other cases the following actions are performed:
    //      3.1. Scale is selected as the maximum semiaxes;
    //      3.2. Translation of the coordinate system origin to the current ellipse center;
    //      3.3. Rotation of the coordinate system: after rotation X axis runs along the large semiaxis
    //           of the current ellipse, Z axis runs along the small semiaxis.
    //           After coordinate system transformations: (0,0) is the center of current ellipse,
    //           (dX,dZ) is the center of 2nd ellipse, dTheta is the azimuth of large semiaxis of 2nd ellipse.
    //      3.4. Solution of the equations system: (1) X^2/a1^2 + Z^2/b1^2 = 1;
    //           (2) ((sin(dTheta)*(Z-dZ)+cos(dTheta)*(X-dX))/a2)^2 + ((cos(dTheta)*(Z-dZ)-sin(dTheta)*(X-dX))/b2)^2 = 1.
    //           In general case Z^2 is excluded from the system and after this Z is expressed as
    //           Z = (A1*X^2 + B1*X + C1) / (D1*X + E1).
    //           Afther this Z is substituted into 1st equation and we obtain 4th order equation
    //           A2*X^4 + B2*X^3 + C2*X^2 + D2*X + E2 = 0.
    //           In particular cases, when D1=0 and E1=0, Z^2 is excluded from the system
    //           and we obtain quadratic equation for X; after this equation solving
    //           X is substituted into 1st equation and we obtain quadratic equation for Z.
    //      3.5  After equations solving: if at least one real solution (pair (X,Z)) exists, there is an intersection;
    //           if all solutions are complex, there is no intersection.
    bool bIntersection = false;
    NRoots = 0;
    if (this->IsPointInside(Ell2.m_Z0, Ell2.m_X0)
            || Ell2.IsPointInside(this->m_Z0, this->m_X0))
    {
        bIntersection = true;
    }
    else
    {
        qreal dX1 = Ell2.m_X0 - this->m_X0;
        qreal dZ1 = Ell2.m_Z0 - this->m_Z0;
        qreal dCenters = sqrt(sqr(dX1) + sqr(dZ1)); //distance between centers of the ellipses
        if (dCenters <= std::max(this->m_a, this->m_b) + std::max(Ell2.m_a, Ell2.m_b)
                && this->m_a > con_eps2 && this->m_b > con_eps2
                && Ell2.m_a > con_eps2 && Ell2.m_b > con_eps2)
        {
            qreal Scale = std::max(std::max(this->m_a, this->m_b), std::max(Ell2.m_a, Ell2.m_b));

            qreal dX = (m_invR[1][0]*dZ1 + m_invR[1][1]*dX1) / Scale;
            qreal dZ = (m_invR[0][0]*dZ1 + m_invR[0][1]*dX1) / Scale;
            qreal a1 = this->m_a / Scale;
            qreal b1 = this->m_b / Scale;
            qreal a2 = Ell2.m_a / Scale;
            qreal b2 = Ell2.m_b / Scale;

            qreal dTheta = Ell2.m_angle - this->m_angle;
            qreal sq_a = sqr(a1);
            qreal sq_b = sqr(b1);
            qreal sq_a2 = sqr(a2);
            qreal sq_b2 = sqr(b2);

            GLSolve_eq_2_3_4_deg FuncSolveEq;
            CGLArrayFix<GLComplexNumb, 4> arRootsDX; //arrays of complex roots in the transformed coordinate system
            CGLArrayFix<GLComplexNumb, 4> arRootsDZ;
            qint16 i=0;

            bool bGeneralCase = false;
            if (fabs(dZ) < con_eps2)
            {
                //particular cases
                qreal A=0; qreal B=0; qreal C=0;
                if (fabs(sin(dTheta)) < con_eps2)
                {
                    A = sq_b2/sq_a2 - sq_b/sq_a;
                    B = -2.*dX*sq_b2/sq_a2;
                    C = (sqr(dX)/sq_a2 - 1.)*sq_b2 + sq_b;
                }
                else if (fabs(cos(dTheta)) < con_eps2)
                {
                    A = sq_a2/sq_b2 - sq_b/sq_a;
                    B = -2.*dX*sq_a2/sq_b2;
                    C = (sqr(dX)/sq_b2 -1.)*sq_a2 + sq_b;
                }
                else if (fabs(a2 - b2) < con_eps2)
                {
                    A = sq_b/sq_a - 1.;
                    B = 2.*dX;
                    C = -sqr(dX) - sq_b + sq_a2;
                }
                else
                {
                    bGeneralCase = true;
                }

                if (!bGeneralCase)
                {
                    qint16 NRoots2 = 0;
                    CGLArrayFix<GLComplexNumb, 2> arR2;
                    FuncSolveEq.Find_roots_2deg_AllCoeff(A, B, C, arR2[0], arR2[1], NRoots2);

                    if (NRoots2 == -1) //zero coefficients, infinity number of roots
                    {
                        bIntersection = true;
                    }
                    else
                    {
                        if (NRoots2 >= 1)
                        {
                            NRoots = 2 * NRoots2;
                            for (i=0; i<NRoots2; i++)
                            {
                                arRootsDX[2*i] = arR2[i];
                                arRootsDX[2*i+1] = arR2[i];
                                GLComplexNumb U;
                                U = (-arR2[i].Degree(2.) + sq_a).Degree(0.5) * (this->m_b/this->m_a);
                                arRootsDZ[2*i] = U;
                                arRootsDZ[2*i+1] = -U;
                            }
                        }
                    }
                }
            }
            else
            {
                bGeneralCase = true;
            }

            if (bGeneralCase) //general case
            {
                qreal mult1 = sqr(cos(dTheta)/b2) + sqr(sin(dTheta)/a2);
                qreal mult2 = sqr(cos(dTheta)/a2) + sqr(sin(dTheta)/b2);
                qreal mult3 = sin(dTheta)*cos(dTheta)*(1./sq_b2 - 1./sq_a2);

                qreal A1 = -mult1 * sq_b/sq_a + mult2;
                qreal B1 = 2.*(mult3*dZ - mult2*dX);
                qreal C1 = mult1*sq_b + mult1*sqr(dZ) + mult2*sqr(dX) - 2.*mult3*dX*dZ - 1.;
                qreal D1 = 2.*mult3;
                qreal E1 = 2.*(mult1*dZ - mult3*dX);

                qreal A2 = sq_a*sqr(A1) + sq_b*sqr(D1);
                qreal B2 = 2.*(sq_a*A1*B1  +  sq_b*D1*E1);
                qreal C2 = sq_b*sqr(E1) - sq_b*sq_a*sqr(D1) + 2.*sq_a*A1*C1 + sq_a*sqr(B1);
                qreal D2 = 2.*sq_a*(B1*C1 - sq_b*D1*E1);
                qreal E2 = sq_a*sqr(C1) - sq_b*sq_a*sqr(E1);

                qint16 NRoots4 = 0;
                CGLArrayFix<GLComplexNumb, 4> arR4;
                FuncSolveEq.Find_roots_4deg_AllCoeff(A2, B2, C2, D2, E2, arR4[0], arR4[1], arR4[2], arR4[3], NRoots4);
                for (i=0; i<NRoots4; i++)
                {
                    GLComplexNumb Denominator = arR4[i]*D1 + E1;
                    if (Denominator.Mod() > con_eps2)
                    {
                        NRoots ++;
                        arRootsDX[NRoots-1] = arR4[i];
                        GLComplexNumb Numerator = (arR4[i].Degree(2.))*A1 + arR4[i]*B1 + C1;
                        arRootsDZ[NRoots-1] = Numerator / Denominator;
                    }
                }
            }

            if (NRoots == -1) //zero coefficients, infinity number of roots
            {
                bIntersection = true;
            }
            else
            {
                if (NRoots >= 1)
                {
                    for (i=0; i<NRoots; i++)
                    {
                        if (arRootsDX[i].IsReal() && arRootsDZ[i].IsReal())
                        {
                            bIntersection = true;
                        }
                        arrZ[i] = (arRootsDZ[i]*this->m_R[0][0] + arRootsDX[i]*this->m_R[0][1])*Scale + this->m_Z0;
                        arrX[i] = (arRootsDZ[i]*this->m_R[1][0] + arRootsDX[i]*this->m_R[1][1])*Scale + this->m_X0;
                    }
                }
            }
        }
    }
    return bIntersection;
}


bool GLEllipse2D::IntersectsEllipse(GLEllipse2D &Ell2)
{
    bool bIntersection = false;
    CGLArrayFix<GLComplexNumb, 4> arrX;
    CGLArrayFix<GLComplexNumb, 4> arrZ;
    qint16 NRoots = 0;
    bIntersection = IntersectsEllipse(Ell2, arrX, arrZ, NRoots);
    return bIntersection;
}



