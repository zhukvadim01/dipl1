#ifndef GEOCENTRIC_ISC_H
#define GEOCENTRIC_ISC_H

#include "Geocentric.h"

// PACKAGE		: Conversion
// FUNCTION:        ICS_GEOCENTRIC
//
// DESCRIPTION:     Recalculation of trajectory parameters from ICS in GCS
//
// PARAM:           time   -  Time of binding
//                 pICS_Coord -  Coordinates in ICS
//                 pICS_Vel -  speed in ICS
//
// RETURNS:         pWGS_Coord -  Coordinates in GCS
//                 pWGS_Vel -  speed in GCS
//
bool   ICS_GEOCENTRIC (double time, const CGeocentric *pICS_Coord, CGeocentric *pWGS_Coord);

bool   ICS_GEOCENTRIC (double time, const CGeocentric *pICS_Coord, const CGeocentric *pICS_Vel,
                       CGeocentric *pWGS_Coord, CGeocentric *pWGS_Vel);

// PACKAGE		: Conversion
// FUNCTION:        GEOCENTRIC_ICS
// DESCRIPTION:     Recalculation of trajectory parameters from GCS in ICS
//
//                 time   -  Time of binding
//
// PARAM:           pWGS_Coord -  Coordinates in GCS
//                 pWGS_Vel -  Speed in GCS
//
// RETURNS:         pICS_Cord -  Coordinates in ICS
//                 pICS_Vel -  Speed in ICS
//
bool   GEOCENTRIC_ICS (double time, const CGeocentric *pWGS_Coord ,
                       CGeocentric *pICS_Coord);

bool   GEOCENTRIC_ICS (double time, const CGeocentric *pWGS_Coord, const CGeocentric *pWGS_Vel,
                       CGeocentric *pICS_Coord , CGeocentric *pICS_Vel );

#endif // GEOCENTRIC_ISC_H
