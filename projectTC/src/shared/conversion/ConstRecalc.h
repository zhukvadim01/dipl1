// PACKAGE	   : Conversion
// FILE        : ConstRecalc.h
// AUTHOR      : Marina Mukhortova, 2003
// DESCRIPTION : Header file for inclusion of particular constants

#ifndef __ConstRecalc_H__
#define __ConstRecalc_H__

// All the parameters are in radians (angles) and metres (distanses).


// The first eccentricity of meridian ellipse: 
// cdFirstEccentricity=e*e=2*f-f*f, f=(a-b)/a     (WGS-84)
constexpr double cdFirstEccentricity = 0.00669437999013;

// The second eccentricity of meridian ellipse:
// cdSecondEccentricity=e'*e'=(a*a-b*b)/(b*b)     (WGS-84)
constexpr double cdSecondEccentricity = 0.00673949674226;

// Equatorial or semimajor axis of an ellipsoid   (WGS-84)
constexpr double cdEquatorAxis = 6378137.00000;    //a

// Polar or semiminor axis of an ellipsoid        (WGS-84)
constexpr double cdPolarAxis = 6356752.314245179;  //b

// Constant PI with double precision
constexpr double cdPi = 3.141592653589793238462643383;

// Main radius of the Earth
constexpr double cdMainRadius = 6371000.00000;

// Equatorial or semimajor axis of an ellipsoid   (Krasovsky)
constexpr double cdEquatorAxisKr = 6378245.00000;         //a

// Polar or semiminor axis of an ellipsoid        (Krasovsky)
constexpr double cdPolarAxisKr = 6356863.01877304726785;  //b

// The first eccentricity of meridian ellipse: 
// cdFirstEccentricityKr=e*e=2*f-f*f, f=1/(298.3)     (Krasovsky)
constexpr double cdFirstEccentricityKr = 0.006693421622965873;

// The second eccentricity of meridian ellipse:
// cdSecondEccentricityKr=e'*e'=(a*a-b*b)/(b*b)       (Krasovsky)
constexpr double cdSecondEccentricityKr = 0.0067385254;


// Imposed restrictions to coordinates:

// Gaus_Kruger system:
constexpr double cdGAUS_X_MIN = -6471000;
constexpr double cdGAUS_X_MAX = 6471000;

// Goga 20.11.2008 const double cdGAUS_Y_MIN = 167000;  
constexpr double cdGAUS_Y_MIN = 165000; // Goga 20.11.2008

constexpr double cdGAUS_Y_MAX = 833000;
constexpr double cd_H_MIN = -647100000000;
constexpr double cd_H_MAX = 647100000000.0;
const unsigned cuGAUS_Nz_MIN = 1;
const unsigned cuGAUS_Nz_MAX = 60;

// Geodesic system:
constexpr double cdGEODES_LATITUDE_MIN = -cdPi/2.0;
constexpr double cdGEODES_LATITUDE_MAX = cdPi/2.0;
constexpr double cdGEODES_LONGITUDE_MIN = -cdPi;
constexpr double cdGEODES_LONGITUDE_MAX = cdPi;

// Geocentric system:
constexpr double cdGEOCEN_X_Y_Z_MIN = -500000000000;
constexpr double cdGEOCEN_X_Y_Z_MAX =  500000000000;
// Goga 13.06.2006 const double cdGEOCEN_X_Y_Z_MIN = -6471000;
// Goga 13.06.2006 const double cdGEOCEN_X_Y_Z_MAX = 6471000;

// Start system:
constexpr double cdSTART_X_Y_Z_MIN = (-1)*(2*6371000.00000+200000);
constexpr double cdSTART_X_Y_Z_MAX = 2*6371000.00000+200000;

// Spherical system:
constexpr double cdSPHERIC_R_MIN = 0;
constexpr double cdSPHERIC_R_MAX = 2*6371000.00000+500000000;
constexpr double cdSPHERIC_E_MIN = -cdPi/2;
constexpr double cdSPHERIC_E_MAX = cdPi/2;
constexpr double cdSPHERIC_B_MIN = 0;
constexpr double cdSPHERIC_B_MAX = 2*cdPi;

// Topocentric system:
constexpr double cdTOPO_X_Y_Z_MIN = (-1)*cdSPHERIC_R_MAX;
constexpr double cdTOPO_X_Y_Z_MAX = cdSPHERIC_R_MAX;

// XZH system:
constexpr double cdXZH_TOPO_DELTA = 1.;

// Recount parameters (Krasovsky) (GeoKras_GeoWGS):
// shift in X-coordinate
constexpr double cdXKr = 24;
// shift in Y-coordinate
constexpr double cdYKr = -123;
// shift in Z-coordinate
constexpr double cdZKr = -94;

// Difference between Krasovsky and WGS papameters in equatorial axis
constexpr double cdAKr = -108;

// Difference between Krasovsky and WGS papameters in flatnees
constexpr double cdFKr = 0.000000480795;

// Recount parameters in GskWGS_GskKras:
constexpr double cdMu = -0.0000002263;
constexpr double cdEx = 0;
constexpr double cdEy = 0;
constexpr double cdEz = 0.000002685869;
constexpr double cdDeltaX = -27;
constexpr double cdDeltaY = 135;
constexpr double cdDeltaZ = 84.5;

// Recount parameters in GskKras_GskWGS:
constexpr double cdMu_KW = 0;
constexpr double cdEx_KW = 0;
constexpr double cdEy_KW = 0;
constexpr double cdEz_KW = -0.000002685869;
constexpr double cdDeltaX_KW = 27;
constexpr double cdDeltaY_KW = -135;
constexpr double cdDeltaZ_KW = -84.5;

// max number itteration of cycle
constexpr int NMaxCycle = 1000;

constexpr double cdEarth_FOM = 7.2921158e-5;    // An angular velocity of rotation of the Earth
constexpr double cdEarth_GM = 3.986004415E14;           // Gravitational parameter of the Earth In view of an atmosphere (m^3/sec^2) //  ,EGM96_GM  = 3.986004418E14

#endif
