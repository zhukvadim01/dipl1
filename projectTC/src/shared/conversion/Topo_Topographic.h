// PACKAGE		: Conversion
// FILE         :   Topo_Topographic.h
//
// AUTHOR	: Lapushynski Aliaksandr, 2015
//DESCRIPTION :Header file for function of recalculation topographic coordinates system and topocentric
//                   coordinates system
#ifndef TOPO_TOPOGRAPHIC_H
#define TOPO_TOPOGRAPHIC_H

#include "Topocentric.h"
#include "Topographic.h"

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topocentric to topographic
//
// INPUTS          : TPC - topocentric coordinates;
//
// RETURNS	   : TPG - topographic coordinates;
//
//
bool RecountTOPOCENTRICtoTOPOGRAPHIC_coord(const CTopocentric* TPC, CTopographic* TPG);

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topocentric to topographic
//
// INPUTS          : TPC - topocentric coordinates, TPC_vel TPC - topocentric velocity;
//
// RETURNS	   : TPG - topographic coordinates, TPG - topographic velocity;
//
//


bool RecountTOPOCENTRICtoTOPOGRAPHIC_coord_vel(const CTopocentric* TPC, const CTopocentric* TPC_vel,
                                               CTopographic* TPG, CTopographic* TPG_vel);

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topographic to topocentric
//
// INPUTS          : TPG - topographic coordinates;
//
// RETURNS	   : TPC - topocentric coordinates;
//
//
//
bool RecountTOPOGRAPHICtoTOPOCENTRIC_coord(const CTopographic* TPG, CTopocentric* TPC);

// PACKAGE		: Conversion
// FUNCTION        : RecountTOPOCENTRICtoTOPOGRAPHIC_coord
//
// DESCRIPTION     : Function of recalculation from topographic to topocentric
//
// INPUTS          : TPG - topographic coordinates,TPG_vel - topographic velocity;
//
// RETURNS	   : TPC - topocentric coordinates,TPC_vel - topocentric velocity;
//
//
//
bool RecountTOPOGRAPHICtoTOPOCENTRIC_coord_vel(const CTopographic* TPG,const  CTopographic* TPG_vel, CTopocentric* TPC, CTopocentric* TPC_vel);
#endif // TOPO_TOPOGRAPHIC_H
