#include <QDataStream>

#include "GLEllipse.h"
#include "GLEllipse2D.h"
#include "GLConvertMatrices.h"

#include "conversion/ConstRecalc.h"
#include "conversion/Geocentric_Topo.h"
#include "conversion/Geocentric_Geo.h"
#include "conversion/Spherical_Topo.h"

GLEllipse::GLEllipse():
    Center(0.,0.,0.),
    time(0),
    a(0.),
    b(0.),
    angle(0.)
{
}


void GLEllipse::clear( void )
{
    Center.clear();
    time = 0.;
    a = 0.;
    b = 0.;
    angle = 0.;
}

GLEllipse& GLEllipse::operator =( const GLEllipse& data )
{
    this->a		= data.a;
    this->b		= data.b;
    this->angle	= data.angle;
    this->Center= data.Center;
    this->time  = data.time;

    return *this;
}

QDataStream& operator<<(QDataStream& stream, const GLEllipse& data)
{
    stream << data.Center
           << data.a
           << data.b
           << data.angle
           << static_cast<qint64>(data.time);
    return stream;
}

QDataStream& operator>>(QDataStream& stream, GLEllipse& data)
{
    stream  >> data.Center
            >> data.a
            >> data.b
            >> data.angle;
    qint64 l_time;     stream >> l_time;
    data.time = l_time;
    return stream;
}

bool GLEllipse::CheckEllipsesIntersection(GLEllipse &Ell2)
{
    bool bResult = false;

    if (!this->Center.IsZero() && !Ell2.Center.IsZero())
    {
        CGeocentric Point1gc(this->Center.x, this->Center.y, this->Center.z);
        CGeocentric Point2gc(Ell2.Center.x, Ell2.Center.y, Ell2.Center.z);
        CGeodesic Point1gd;

        bool bOK = GEOCENTRIC_GEO(&Point1gc, &Point1gd);
        if (bOK)
        {
            CTopocentric Point2tp;
            bOK = GEOCENTRIC_TOPO(&Point1gd, &Point2gc, &Point2tp);
            if (bOK)
            {
                GLEllipse2D Ell2D_1(this->b, this->a, this->angle, 0, 0);
                GLEllipse2D Ell2D_2(Ell2.b, Ell2.a, Ell2.angle, Point2tp.m_dZt, Point2tp.m_dXt);

                bResult = IsEllipseIntersectEllipse(Ell2D_1, Ell2D_2);
            }
        }
    }

    return bResult;
}


bool GLEllipse::FormPlaneEllipse(GLPointDouble3D &P_ECEF, TMatrix<3> &Cov_ECEF, qreal Multiplicator)
{
    bool bRes = true; //true if result is OK
    clear();
    if (!P_ECEF.IsZero() && Multiplicator > con_eps)
    {
        CGeocentric Pgc(P_ECEF.x, P_ECEF.y, P_ECEF.z);
        CGeodesic Pgd;
        bRes = GEOCENTRIC_GEO(&Pgc, &Pgd);
        if (bRes)
        {
            qreal Lat = Pgd.m_dLatitude,
                    Long  = Pgd.m_dLongitude,
                    sin_Lat = sin(Lat),
                    cos_Lat = cos(Lat),
                    sin_Long = sin(Long),
                    cos_Long = cos(Long),
                    sin_2Lat = sin(2.*Lat),
                    sin_2Long = sin(2.*Long),
                    cos_2Long = cos(2.*Long);

            qreal DXtp, DZtp, KXZtp; //dispersion on X, dispersion on Z, covariance moment on X,Z in topocentirc coordinate system
            DXtp =    sqr(sin_Lat*cos_Long) * Cov_ECEF.M[0][0]
                    + sqr(sin_Lat*sin_Long) * Cov_ECEF.M[1][1]
                    + sqr(cos_Lat) * Cov_ECEF.M[2][2]
                    + sin_2Long * sqr(sin_Lat) * Cov_ECEF.M[0][1]
                    - sin_2Lat * cos_Long * Cov_ECEF.M[0][2]
                    - sin_2Lat * sin_Long * Cov_ECEF.M[1][2];

            DZtp =    sqr(sin_Long) * Cov_ECEF.M[0][0]
                    + sqr(cos_Long) * Cov_ECEF.M[1][1]
                    - sin_2Long * Cov_ECEF.M[0][1];

            KXZtp = 0.5 * sin_2Long * sin_Lat * (Cov_ECEF.M[0][0] - Cov_ECEF.M[1][1])
                    - sin_Lat * cos_2Long * Cov_ECEF.M[0][1]
                    - sin_Long * cos_Lat * Cov_ECEF.M[0][2]
                    + cos_Long * cos_Lat * Cov_ECEF.M[1][2];

            bRes = CalcEllParamForCovMatr2x2(DXtp, DZtp, KXZtp, a, b, angle);
            if (bRes)
            {
                Center = P_ECEF;
                a *= Multiplicator;
                b *= Multiplicator;
            }
        }
    }
    else
    {
        bRes = false;
    }
    return bRes;
}


bool GLEllipse::FormPlaneEllipse(GLPointDouble3D &P_ECEF, GLMatrix &Cov_ECEF, qreal Multiplicator)
{
    bool bRes = true; //true if result is OK
    TMatrix<3> Cov_ECEF_3(3,3);
    TCMatrixFunc<3> fMatr;
    bRes = fMatr.Copy(Cov_ECEF_3, Cov_ECEF, 0, 2, 0, 2);
    if (bRes)
    {
        bRes = FormPlaneEllipse(P_ECEF, Cov_ECEF_3, Multiplicator);
    }
    return bRes;
}


bool GLEllipse::CalcEllParamForCovMatr2x2(qreal Dx, qreal Dz, qreal Kxz, qreal &Res_a, qreal &Res_b, qreal &Res_angle)
{
    bool bRes = true;
    Res_a = 0;
    Res_b = 0;
    Res_angle = 0;
    if (Dx > con_eps2 && Dz > con_eps2 && Kxz < Dx*Dz)
    {
        Res_a = sqrt(0.5 * (Dx + Dz + sqrt(sqr(Dx - Dz) + sqr(2. * Kxz))));
        Res_b = sqrt(0.5 * (Dx + Dz - sqrt(sqr(Dx - Dz) + sqr(2. * Kxz))));
        if (fabs(Dx - Dz) < con_eps2)
        {
            if (Kxz > con_eps2)
            {
                Res_angle = con_pi / 4.;
            }
            else
            {
                if (Kxz < -con_eps2)
                {
                    Res_angle = - con_pi / 4.;
                }
                else
                {
                    Res_angle = 0;
                }
            }
        }
        else
        {
            qreal Alpha0 = 0.5 * atan(2. * Kxz / (Dx - Dz));
            if (Dx > Dz)
            {
                Res_angle = Alpha0;
            }
            else
            {
                if (Kxz > - con_eps2)
                {
                    Res_angle = Alpha0 + con_half_pi;
                }
                else
                {
                    Res_angle = Alpha0 - con_half_pi;
                }
            }
        }
    }
    else
    {
        bRes = false;
    }
    return bRes;
}


void GLEllipse::RestrictionSemiaxes(qreal MaxSemiaxis)
{
    if (MaxSemiaxis > 0)
    {
        if (a > MaxSemiaxis)
        {
            a = MaxSemiaxis;
        }
        if (b > MaxSemiaxis)
        {
            b = MaxSemiaxis;
        }
    }
}

void GLEllipse::SetMinSemiaxesRatio(qreal _MinSemiaxesRatio)
{
    min_semiaxes_ratio = _MinSemiaxesRatio;
}


bool GLEllipse::CalcEllipse(TMatrix<3> &CovMatr, const GLPointDouble3D &Point, const GLPointDouble3D &Vel, qreal &aI, qreal &bI, qreal &betaI, bool bCorrRatio)
{
    bool bOK = true;
    a = 0;
    b = 0;
    angle = 0;
    Center = Point;

    GLPointDouble3D VelCorr = Vel;
    if (VelCorr.moduleFloat() > 0.5*cdSPHERIC_R_MAX)
    {
        VelCorr = Vel / (0.5*cdSPHERIC_R_MAX);
    }

    CGeocentric PointGC(Point.x, Point.y, Point.z);
    CGeocentric VelGC(VelCorr.x, VelCorr.y, VelCorr.z);
    CGeodesic PointGD;
    CTopocentric PointTP;
    CTopocentric VelTP;
    CSpherical VelSph;
    TMatrix<3> CovTP;


    bOK &= GEOCENTRIC_GEO(&PointGC, &PointGD);

    bOK &= GEOCENTRIC_TOPO(&PointGD, &PointGC, &VelGC, 0, &PointTP, &VelTP, 0);

    bOK &= TOPO_SPHERICAL(&VelTP, &VelSph); //calculation of azimuth and elevation of velocity vector, not range of azimuth and elevation

    bOK &= GL_MATR::Recount_CovMatr_GeocentricToTopo(&PointGD, &CovMatr, &CovTP);

    qreal beta2 = VelSph.m_dB; //azumuth of the velocity vector
    qreal eps2  = VelSph.m_dE; //elevation of the velocity vector

    TMatrix<3> A(3,3); TMatrix<3> AT(3,3);
    TMatrix<3> K_TGT(3,3); //covariance matrix in the target coordinate system
    TCMatrixFunc<3> FuncMatr;

    A.M[0][0] = cos(-eps2)*cos(-beta2);
    A.M[0][1] = -sin(-eps2);
    A.M[0][2] = -cos(-eps2)*sin(-beta2);
    A.M[1][0] = sin(-eps2)*cos(-beta2);
    A.M[1][1] = cos(-eps2);
    A.M[1][2] = -sin(-eps2)*sin(-beta2);
    A.M[2][0] = sin(-beta2);
    A.M[2][1] = 0;
    A.M[2][2] = cos(-beta2);

    bOK &= FuncMatr.Transpon(A, AT); //AT is A transposed
    bOK &= FuncMatr.MatrXMatrXMatr(A, CovTP, AT, K_TGT);

    TMatrix<2> A_reducted(2,2);
    TMatrix<2> A_reducted_Inv(2,2);
    TMatrix<2> A_reducted_Inv_T(2,2);
    TMatrix<2> K_TGT_reducted(2,2);
    TMatrix<2> KEll(2,2);
    TCMatrixFunc<2> FuncMatr2;

    A_reducted.M[0][0] = A.M[1][0];
    A_reducted.M[0][1] = A.M[1][2];
    A_reducted.M[1][0] = A.M[2][0];
    A_reducted.M[1][1] = A.M[2][2];

    K_TGT_reducted.M[0][0] = K_TGT.M[1][1];
    K_TGT_reducted.M[0][1] = K_TGT.M[1][2];
    K_TGT_reducted.M[1][0] = K_TGT.M[2][1];
    K_TGT_reducted.M[1][1] = K_TGT.M[2][2];

    bOK &= FuncMatr2.InvMatrix(A_reducted, A_reducted_Inv); //inverse matrix
    bOK &= FuncMatr2.Transpon(A_reducted_Inv, A_reducted_Inv_T); //transposed inverse matrix
    bOK &= FuncMatr2.MatrXMatrXMatr(A_reducted_Inv, K_TGT_reducted, A_reducted_Inv_T, KEll);

    bOK &= CalcEllParamForCovMatr2x2(KEll.M[0][0], KEll.M[1][1], KEll.M[0][1], a, b, angle);

    aI = fabs(a);
    bI = fabs(b);
    betaI = angle;

    if (bCorrRatio && min_semiaxes_ratio > 0)
    {
        double b_bound = min_semiaxes_ratio * aI;
        if (bI < b_bound)
        {
            bI = b_bound;
        }
    }

    return bOK;
}


bool GLEllipse::CalcEllipse(TMatrix<6> &CovMatr, const GLPointDouble3D &Point, const GLPointDouble3D &Vel, qreal &aI, qreal &bI, qreal &betaI, bool bCorrRatio)
{
    bool bOK = true;
    TMatrix<3> Cov3;
    TCMatrixFunc<3> FuncMatr;
    bOK &= FuncMatr.Copy(Cov3, CovMatr, 0, 2, 0, 2);
    bOK &= CalcEllipse(Cov3, Point, Vel, aI, bI, betaI, bCorrRatio);
    return bOK;
}


bool GLEllipse::CalcEllipse(GLMatrix &CovMatr, const GLPointDouble3D &Point, const GLPointDouble3D &Vel, qreal &aI, qreal &bI, qreal &betaI, bool bCorrRatio)
{
    bool bOK = true;
    TMatrix<3> Cov3;
    TCMatrixFunc<3> FuncMatr;
    bOK &= FuncMatr.Copy(Cov3, CovMatr, 0, 2, 0, 2);
    bOK &= CalcEllipse(Cov3, Point, Vel, aI, bI, betaI, bCorrRatio);
    return bOK;
}
