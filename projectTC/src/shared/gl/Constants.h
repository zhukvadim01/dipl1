#ifndef GL_CONSTANTS_H
#define GL_CONSTANTS_H

constexpr double con_Eath_compress_coeff =  0.0033528107;           // compression of the Eath 1/298.257223563 WGS84
constexpr double con_b_half_axis         =  6378137.0;              // half of large axis WGS84 ellipsoid
constexpr double con_l_half_axis         =  6356752.3142;           // half of small axis WGS84 ellipsoid
constexpr double con_Eath_equator_radius =  6378137.0;              // equator radius of the Eath
constexpr double con_eccen_ell           =  0.00669437999;          // square of the eccentricity of the WGS84  ellipsoid
constexpr double con_Eath_angle_speed    =  7.2921158E-5;           // Angle speed of the Eath rotation
constexpr double con_mu                  =  3.986000E14;            // multiplication of the gravitation constant and the Eath mass
constexpr double con_c2                  = -2.194466E+10;           //
constexpr double con_Eath_middle_radius  =  6371000.0;              // middle radius of the Eath
constexpr double con_pi                  =  3.141592653589793238;
constexpr double con_2pi                 =  6.283185307179586477;
constexpr double con_half_pi             =  1.570796326794896619;
constexpr double con_light_speed         =  299792456.0;            // light speed in vacuum
constexpr double PI                      =  3.141592653589793238462643383;  // Constant PI with double precision
constexpr double con_g					 =	9.81;					// acceleration of gravity
constexpr double con_Grav_Const          =  6.67384E-11;            // gravitational constant, m^3/(kg*s^2)
constexpr double con_Earth_Mass          =  5.972E+24;              // Earth mass, kg

constexpr double con_pi_div_180          =  0.017453292519943296;
constexpr double con_180_div_PI          =  57.295779513082320876;
constexpr double con_par_eps             =  0.1E-14;                 // conditional real zero
constexpr double con_eps                 =  0.0001;
constexpr double con_eps2                =  0.0000001;
constexpr double con_PARDEK01            =  1.0E-7;
constexpr double con_large_value         =  1.0E+10;                 // abstract large value
constexpr int    con_vengr               =  4;                       // constant for limiting the number of iterations

namespace GL
{
    const double NO_VALUE            =  -1111.111111115285;      //constant indicating absence of a value
}

#endif
