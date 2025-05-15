#include "GLSolve_eq_2_3_4_deg.h"

#include "Constants.h"
#include "GLMath.h"

using namespace SolveEq;

GLSolve_eq_2_3_4_deg::GLSolve_eq_2_3_4_deg()
{
}


GLSolve_eq_2_3_4_deg::~GLSolve_eq_2_3_4_deg()
{
}


void GLSolve_eq_2_3_4_deg::Find_roots_2deg(double p, double q, GLComplexNumb &root1, GLComplexNumb &root2)
{
    double D = p*p - c4 * q;
    GLComplexNumb D_cmpl(D, 0.), Sqrt_D;

    Sqrt_D = D_cmpl.Degree(c_0_5);

    root1 = (Sqrt_D - p) / c2;
    root2 = (-Sqrt_D - p) / c2;
}


double GLSolve_eq_2_3_4_deg::Find_root_3deg_Viete(double a, double b, double c)
{
    double phi, x1;

    double Q = (a*a - c3*b)/c9;
    double R = (c2*a*a*a - c9*a*b + c27*c)/c54;
    double S = Q*Q*Q - R*R;

    if (S > con_par_eps)
    {
        phi = acos(R / sqrt(Q*Q*Q)) / c3;
        x1 = - c2 * sqrt(Q)*cos(phi) - a/c3;
    }
    else if (S < - con_par_eps)
    {
        if (Q > con_par_eps)
        {
            phi = acosh(fabs(R) / sqrt(Q*Q*Q)) / c3;
            x1 = - c2 * Sign(R) * sqrt(Q) * cosh(phi) - a/c3;
        }
        else if (Q < - con_par_eps)
        {
            phi = asinh(fabs(R) / sqrt(fabs(Q*Q*Q))) / c3;
            x1 = -c2 * Sign(R) * sqrt(fabs(Q)) * sinh(phi) - a/c3;
        }
        else //Q is 0
        {
            x1 = - pow(c - a*a*a/c27, 1/c3) - a/c3;
        }
    }
    else //S is 0
    {
        x1 = -c2 * pow(R, 1/c3) - a/c3;
    }

    return x1;
}


void GLSolve_eq_2_3_4_deg::Find_roots_3deg(double a, double b, double c, GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3)
{
    double root1_d = Find_root_3deg_Viete(a, b, c);

    root1.Reset(root1_d, 0.);

    if ( fabs(root1_d) > con_par_eps )
    {
        double p, q;
        p = a + root1_d;
        q = - c / root1_d;
        Find_roots_2deg(p, q, root2, root3);
    }
    else
    {
        Find_roots_2deg(a, b, root2, root3);
    }
}


void GLSolve_eq_2_3_4_deg::Find_roots_4deg_Ferrari(double a, double b, double c, double d, GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3, GLComplexNumb &root4)
{
    double A1, B1, C1, y1;
    GLComplexNumb alpha, beta, sq_alpha, sq_beta,
            p1, q1, p2, q2, D1, D2, sqrtD1, sqrtD2;

    A1 = -b;
    B1 = a*c - c4*d;
    C1 = -a*a*d + c4*b*d - c*c;

    y1 = Find_root_3deg_Viete(A1, B1, C1);

    sq_alpha.SetReal(a*a/c4 - b + y1);
    sq_beta.SetReal(y1*y1/c4 - d);

    alpha = sq_alpha.Degree(c_0_5);
    if ((a*y1/c2 - c) * sq_alpha.GetReal() >= 0)
        beta = sq_beta.Degree(c_0_5);
    else
        beta = - sq_beta.Degree(c_0_5);

    p1 = - alpha + a/c2;
    q1 = - beta + y1/c2;
    p2 = alpha + a/c2;
    q2 = beta + y1/c2;

    D1 = p1*p1 - q1*c4;
    D2 = p2*p2 - q2*c4;
    sqrtD1 = D1.Degree(c_0_5);
    sqrtD2 = D2.Degree(c_0_5);

    root1 = (-p1 - sqrtD1) / c2;
    root2 = (-p1 + sqrtD1) / c2;
    root3 = (-p2 - sqrtD2) / c2;
    root4 = (-p2 + sqrtD2) / c2;

    return;
}


double GLSolve_eq_2_3_4_deg::Sign(double x)
{
    double res;
    if (x > 0)
        res = 1.;
    else if (x < 0)
        res = -1.;
    else
        res = 0.;
    return res;
}


void GLSolve_eq_2_3_4_deg::Find_roots_2deg_AllCoeff(double a, double b, double c, GLComplexNumb &root1, GLComplexNumb &root2, qint16 &NRoots)
{
    NRoots = 0;
    root1.Reset();
    root2.Reset();

    if (fabs(a) > con_eps2)
    {
        NRoots = 2;
        Find_roots_2deg(b/a, c/a, root1, root2);
    }
    else
    {
        if (fabs(b) > con_eps2)
        {
            NRoots = 1;
            root1.SetReal(-c/b);
        }
        else
        {
            if (fabs(c) < con_eps2)
            {
                NRoots = -1; //all coeffficients are 0, infinity number of roots
            }
        }
    }
}


void GLSolve_eq_2_3_4_deg::Find_roots_3deg_AllCoeff(double a, double b, double c, double d, GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3, qint16 &NRoots)
{
    NRoots = 0;
    root1.Reset();
    root2.Reset();
    root3.Reset();

    if (fabs(a) > con_eps2)
    {
        NRoots = 3;
        Find_roots_3deg(b/a, c/a, d/a, root1, root2, root3);
    }
    else
    {
        Find_roots_2deg_AllCoeff(b, c, d, root1, root2, NRoots);
    }
}


void GLSolve_eq_2_3_4_deg::Find_roots_4deg_AllCoeff(double a, double b, double c, double d, double e, GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3, GLComplexNumb &root4, qint16 &NRoots)
{
    NRoots = 0;
    root1.Reset();
    root2.Reset();
    root3.Reset();
    root4.Reset();

    if (fabs(a) > con_eps2)
    {
        NRoots = 4;
        Find_roots_4deg_Ferrari(b/a, c/a, d/a, e/a, root1, root2, root3, root4);
    }
    else
    {
        Find_roots_3deg_AllCoeff(b, c, d, e, root1, root2, root3, NRoots);
    }
}


GLComplexNumb::GLComplexNumb()
{
//    Reset();
}


GLComplexNumb::GLComplexNumb(double valRe, double valIm)
{
    Reset(valRe, valIm);
}


GLComplexNumb::~GLComplexNumb()
{
}


void GLComplexNumb::Reset()
{
    this->Re = 0.;
    this->Im = 0.;
}


void GLComplexNumb::Reset(double valRe, double valIm)
{
    this->Re = valRe;
    this->Im = valIm;
}


void GLComplexNumb::SetReal(double valRe)
{
    this->Re = valRe;
}


void GLComplexNumb::SetImaginary(double valIm)
{
    this->Im = valIm;
}


double GLComplexNumb::GetReal()
{
    return this->Re;
}


double GLComplexNumb::GetImaginary()
{
    return this->Im;
}


GLComplexNumb& GLComplexNumb::operator =( GLComplexNumb Q )
{
    this->Re = Q.GetReal();
    this->Im = Q.GetImaginary();
    return *this;
}


GLComplexNumb GLComplexNumb::operator+( GLComplexNumb Q )
{
    GLComplexNumb res;

    double sumRe = this->Re + Q.GetReal();
    double sumIm = this->Im + Q.GetImaginary();
    res.Reset(sumRe, sumIm);

    return res;
}


GLComplexNumb GLComplexNumb::operator +(double q)
{
    GLComplexNumb res;
    res.SetReal(this->Re + q);
    res.SetImaginary(this->Im);
    return res;
}


GLComplexNumb GLComplexNumb::operator-( GLComplexNumb Q )
{
    GLComplexNumb res;

    double diffRe = this->Re - Q.GetReal();
    double diffIm = this->Im - Q.GetImaginary();
    res.Reset(diffRe, diffIm);

    return res;
}


GLComplexNumb GLComplexNumb::operator -(double q)
{
    GLComplexNumb res;
    res.SetReal(this->Re - q);
    res.SetImaginary(this->Im);
    return res;
}


GLComplexNumb GLComplexNumb::operator-()
{
    GLComplexNumb res;
    res.SetReal(- this->Re);
    res.SetImaginary(- this->Im);
    return res;
}


GLComplexNumb GLComplexNumb::operator*( GLComplexNumb Q )
{
    GLComplexNumb res;
    double a = this->Re, b = this->Im,
           c = Q.GetReal(), d = Q.GetImaginary();
    res.SetReal(a*c - b*d);
    res.SetImaginary(a*d + b*c);

    return res;
}


GLComplexNumb GLComplexNumb::operator*(double q)
{
    GLComplexNumb res;
    res.SetReal(q * this->Re);
    res.SetImaginary(q * this->Im);
    return res;
}


GLComplexNumb GLComplexNumb::operator/( GLComplexNumb Q )
{
    GLComplexNumb res;
    double a = this->Re, b = this->Im,
           c = Q.GetReal(), d = Q.GetImaginary();
    double modQ = Q.Mod();
    res.SetReal((a*c + b*d) / sqr(modQ));
    res.SetImaginary((b*c - a*d) / sqr(modQ));

    return res;
}


GLComplexNumb GLComplexNumb::operator/(double q)
{
    GLComplexNumb res;
    res.SetReal(this->Re/q);
    res.SetImaginary(this->Im/q);
    return res;
}


std::ostream& operator<<(std::ostream &os, GLComplexNumb& Q)
{
    double valRe = Q.GetReal(), valIm = Q.GetImaginary();
    os << "Re = " << valRe << " Im = " << valIm << "\t";
    return os;
}


double GLComplexNumb::Mod()
{
    return sqrt(this->Re*this->Re + this->Im*this->Im);
}


double GLComplexNumb::Arg()
{
    double phi, phi_base=0;

    if (fabs(this->Mod()) > con_par_eps)
        phi_base = atan(this->Im / this->Re);

    if (this->Re > 0)
    {
        phi = phi_base;
    }
    else if (this->Re < 0)
    {
        if (this->Im >= 0)
            phi = phi_base + con_pi;
        else
            phi = phi_base - con_pi;
    }
    else //Re == 0
    {
        if (this->Im > 0)
            phi = con_half_pi;
        else if (this->Im < 0)
            phi = - con_half_pi;
        else //Im == 0
            phi = 0.;
    }
    return phi;
}


GLComplexNumb GLComplexNumb::Degree(double power)
{
    GLComplexNumb res;

    double r = this->Mod();
    double phi = this->Arg();

    double r_n = pow(r, power);

    res.SetReal(r_n * cos(power * phi));
    res.SetImaginary(r_n * sin(power * phi));

    return res;
}


bool GLComplexNumb::IsReal()
{
    if (fabs(this->Im) < GLComplNum::TolCompl )
        return true;
    else
        return false;
}
