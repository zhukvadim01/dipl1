#ifndef SOLVE_EQ_3_4_DEG_H
#define SOLVE_EQ_3_4_DEG_H

#include <QtGlobal>
#include <iostream>

//PACKAGE       :   GL
//CLASS         :   GLComplexNumb
//DESCRIPTION   :   Class for working with complex numbers
class GLComplexNumb
{
public:
    GLComplexNumb();
    ~GLComplexNumb();

    GLComplexNumb(double valRe, double valIm);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::Reset()
    // DESCRIPTION	:   Set class attributes to zero
    // INPUTS		:	None
    // RETURNS		:	None
    void Reset();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::Reset()
    // DESCRIPTION	:   Set class attributes to given parameters
    // INPUTS		:	Real part of complex number, imaginary part of complex number
    // RETURNS		:	None
    void Reset(double valRe, double valIm);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::SetReal()
    // DESCRIPTION	:   Set real part of complex number to given parameter
    // INPUTS		:	Real part of complex number
    // RETURNS		:	None
    void SetReal(double valRe);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::SetReal()
    // DESCRIPTION	:   Set imaginary part of complex number to given parameter
    // INPUTS		:	Imaginary part of complex number
    // RETURNS		:	None
    void SetImaginary(double valIm);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::GetReal()
    // DESCRIPTION	:   Returns real part of complex number
    // INPUTS		:	None
    // RETURNS		:	Real part of complex number
    double GetReal();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::GetReal()
    // DESCRIPTION	:   Returns imaginary part of complex number
    // INPUTS		:	None
    // RETURNS		:	Imaginary part of complex number
    double GetImaginary();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator=
    // DESCRIPTION	:   Assignment operator
    // INPUTS		:	Assignment data
    // RETURNS		:	Result of assignment
    GLComplexNumb& operator=( GLComplexNumb Q );

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator+
    // DESCRIPTION	:   Add operator
    // INPUTS		:	Adding data
    // RETURNS		:	Sum
    GLComplexNumb operator+( GLComplexNumb Q );

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator+
    // DESCRIPTION	:   Add operator (complex and real)
    // INPUTS		:	Adding data (real number)
    // RETURNS		:	Sum
    GLComplexNumb operator+(double q);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator-
    // DESCRIPTION	:   Subtraction operator
    // INPUTS		:	Subtracting data
    // RETURNS		:	Difference
    GLComplexNumb operator-(GLComplexNumb Q );

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator-
    // DESCRIPTION	:   Subtraction operator (complex and real)
    // INPUTS		:	Subtracting data (real number)
    // RETURNS		:	Difference
    GLComplexNumb operator-(double q);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator-
    // DESCRIPTION	:   Unary minus
    // INPUTS		:	None
    // RETURNS		:	Opposite complex number
    GLComplexNumb operator-();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator*
    // DESCRIPTION	:   Multiply operator
    // INPUTS		:	Factor
    // RETURNS		:	Product
    GLComplexNumb operator*( GLComplexNumb Q );

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator*
    // DESCRIPTION	:   Multiply operator (complex by real)
    // INPUTS		:	Factor, real number
    // RETURNS		:	Product
    GLComplexNumb operator*( double q );

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator/
    // DESCRIPTION	:   Division operator
    // INPUTS		:	Divisor
    // RETURNS		:	Quotient
    GLComplexNumb operator/( GLComplexNumb Q );

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::operator/
    // DESCRIPTION	:   Division operator (complex by real)
    // INPUTS		:	Divisor
    // RETURNS		:	Quotient
    GLComplexNumb operator/(double Q );

    // PACKAGE		:   GL
    // FUNCTION 	:   operator<<
    // DESCRIPTION	:   Output operator for complex number
    // INPUTS		:	Output stream; complex number
    // RETURNS		:	Output stream
    friend std::ostream& operator<<(std::ostream& os, GLComplexNumb& Q);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::Mod
    // DESCRIPTION	:   Calculate modulus (magnitude) of complex number
    // INPUTS		:	None
    // RETURNS		:	Modulus (magnitude) of complex number
    double Mod();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::Arg
    // DESCRIPTION	:   Calculate argument of complex number (in radians)
    // INPUTS		:	None
    // RETURNS		:	Argument of complex number (in radians)
    double Arg();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::Degree
    // DESCRIPTION	:   Exponentiation with complex base and real power (De Moivre's formula)
    // INPUTS		:	Power, real number
    // RETURNS		:	Result of exponentiation
    GLComplexNumb Degree(double power);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLComplexNumb::IsReal
    // DESCRIPTION	:   Determines whether this number is a real
    // INPUTS		:	None
    // RETURNS		:	True if number is real
    bool IsReal();

private:
    double Re{0.}; //real part of complex number
    double Im{0.}; //imaginary part of complex number
};


namespace GLComplNum
{
    const double    TolCompl = 0.1E-10; //tolerance
}


namespace SolveEq
{
    const double    c2 = 2.;
    const double    c3 = 3.;
    const double    c4 = 4.;
    const double    c9 = 9.;
    const double    c27 = 27.;
    const double    c54 = 54.;
    const double    c_0_5 = 0.5;
    const double    TolCompl = 0.1E-10; //tolerance
}


//PACKAGE       :   GL
//CLASS         :   GLSolve_eq_2_3_4_deg
//DESCRIPTION   :   Class for solving of equations of 3 and 4 degree
class GLSolve_eq_2_3_4_deg
{
public:
    GLSolve_eq_2_3_4_deg();

    ~GLSolve_eq_2_3_4_deg();

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_roots_2deg()
    // DESCRIPTION	:   Finding two complex roots of the equation of 2nd degree x^2 + p*x + q = 0
    // INPUTS		:	p, q - coefficients of the equation
    // RETURNS		:	Complex roots of the equation
    void Find_roots_2deg(double p, double q, GLComplexNumb &root1, GLComplexNumb &root2);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_root_3deg_Viete()
    // DESCRIPTION	:   Finding one real root of the equation of 3th degree
    //              :   x^3+a*x^2+b*x+c=0 using Viete trigonometric formula
    // INPUTS		:	a, b, c - coefficients of the polynomial
    // RETURNS		:	Real root (one of roots) of the equation
    double Find_root_3deg_Viete(double a, double b, double c);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_roots_3deg()
    // DESCRIPTION	:   Finding of 3 roots of the equation of 3th degree x^3+a*x^2+b*x+c=0
    // INPUTS		:	a, b, c - coefficients of the polynomial
    // RETURNS		:	Complex roots of the equation
    void Find_roots_3deg(double a, double b, double c,
                         GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_roots_4deg_Ferrari()
    // DESCRIPTION	:   Finding of 4 roots (maybe complex) of the equation of 4th degree
    //              :   x^4+a*x^3+b*x^2+c*x+d=0 using Ferrari method
    // INPUTS		:	a, b, c, d - coefficients of the polynomial
    // RETURNS		:	4 roots (maybe complex) of the equation
    void Find_roots_4deg_Ferrari(double a, double b, double c, double d,
                                 GLComplexNumb &root1, GLComplexNumb &root2,
                                 GLComplexNumb &root3, GLComplexNumb &root4);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Sign()
    // DESCRIPTION	:   Sign of number (1 - positive, -1 - negative, 0 - 0)
    // INPUTS		:	x - given number
    // RETURNS		:	Sign of number
    double Sign(double x);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_roots_2deg_AllCoeff()
    // DESCRIPTION	:   Finding the complex roots of the equation of 2nd degree a*x^2 + b*x + c = 0,
    //              :   including the case of zero leading coeffients
    // INPUTS		:	a, b, c - coefficients of the equation
    // RETURNS		:	Complex roots of the equation;
    //              :   quantity of roots, if counted with their multiplicities (-1 in the case of infinity number of roots)
    void Find_roots_2deg_AllCoeff(double a, double b, double c,
                                  GLComplexNumb &root1, GLComplexNumb &root2, qint16 &NRoots);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_roots_3deg_AllCoeff()
    // DESCRIPTION	:   Finding the complex roots of the equation of 2nd degree a*x^3 + b*x^2 + c*x + d = 0,
    //              :   including the case of zero leading coeffients
    // INPUTS		:	a, b, c, d - coefficients of the equation
    // RETURNS		:	Complex roots of the equation;
    //              :   quantity of roots, if counted with their multiplicities (-1 in the case of infinity number of roots)
    void Find_roots_3deg_AllCoeff(double a, double b, double c, double d,
                                  GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3, qint16 &NRoots);

    // PACKAGE		:   GL
    // FUNCTION 	:   GLSolve_eq_2_3_4_deg::Find_roots_4deg_AllCoeff()
    // DESCRIPTION	:   Finding the complex roots of the equation of 2nd degree a*x^4 + b*x^3 + c*x^2 + d*x + e = 0,
    //              :   including the case of zero leading coeffients
    // INPUTS		:	a, b, c, d, e - coefficients of the equation
    // RETURNS		:	Complex roots of the equation;
    //              :   quantity of roots, if counted with their multiplicities (-1 in the case of infinity number of roots)
    void Find_roots_4deg_AllCoeff(double a, double b, double c, double d, double e,
                                  GLComplexNumb &root1, GLComplexNumb &root2, GLComplexNumb &root3, GLComplexNumb &root4, qint16 &NRoots);
};

#endif // SOLVE_EQ_3_4_DEG_H
