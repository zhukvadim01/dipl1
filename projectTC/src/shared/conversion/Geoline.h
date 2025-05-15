// PACKAGE		: Conversion
// FILE       : Geoline.h  
//
// AUTHOR     : Marina Mukhortova, 2003
// DESCRIPTION		:Header file for CGEOLINE class
//#ifdef RECOUNT_EXPORTS
//#define   __declspec(dllexport)
//#else
//#define   __declspec(dllimport)
//#endif

#ifndef __GEOLINE_H__
#define __GEOLINE_H__

#include "Structures.h"
#include "Geodesic.h"
#include "Geocentric_Geo.h"

//#include <cmath>



// PACKAGE		: Conversion
// CLASS	    : CGEOLINE
//
// DESCRIPTION      : class of the calculation of the length of the geodesic line 
//   
class   CGEOLINE

{

public:

    // Length of the geodesic line
    double m_dLength{0.};

    //PACKAGE       :   Conversion
    //FUNCTION      :   CGEOLINE::CalculationGEOLINE()
    //DESCRIPTION   :   The function of calculation of the length of the geodesic line
    //INPUTS        :   Pointers to 2 points in Geodesic coordinate system
    //RETURNS       :   True if result is ok
    bool CalculationGEOLINE (const CGeodesic *pGEO1,const CGeodesic *pGEO2);

    //PACKAGE       :   Conversion
    //FUNCTION      :   CGEOLINE::CalculationGEOLINE()
    //DESCRIPTION   :   The function of calculation of the length of the geodesic line
    //INPUTS        :   Pointers to 2 points in Geocentric coordinate system (ECEF)
    //RETURNS       :   True if result is ok
    bool CalculationGEOLINE (const CGeocentric *pGEOCENTRIC1,const CGeocentric *pGEOCENTRIC2);

    //PACKAGE       :   Conversion
    //FUNCTION      :   CGEOLINE::CalculationGEOLINE()
    //DESCRIPTION   :   The function of calculation of the length of the geodesic line
    //INPUTS        :   Pointers to 2 points in Geodesic coordinate system
    //RETURNS       :   Value of geodesic line (as reference &GeoLine)
    //              :   returns true if result is ok
    bool CalculationGEOLINE(const CGeodesic *pGEO1, const CGeodesic *pGEO2, double &GeoLine);

    //PACKAGE       :   Conversion
    //FUNCTION      :   CGEOLINE::CalculationGEOLINE()
    //DESCRIPTION   :   The function of calculation of the length of the geodesic line
    //INPUTS        :   Pointers to 2 points in Geocentric coordinate system (ECEF)
    //RETURNS       :   Value of geodesic line (as reference &GeoLine)
    //              :   returns true if result is ok
    bool CalculationGEOLINE(const CGeocentric *pGEOCENTRIC1, const CGeocentric *pGEOCENTRIC2, double &GeoLine);
};

//#include "GEOLINE.inl"

#endif
