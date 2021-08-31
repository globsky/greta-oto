//----------------------------------------------------------------------
// SatManage.h:
//   declaration of satellite management related functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __SAT_MANAGE_H__
#define __SAT_MANAGE_H__

#include "CommonDefines.h"
#include "DataTypes.h"

void CalcSatelliteInfo(PCHANNEL_STATUS ObservationList[], int ObsCount);
int FilterObservation(PCHANNEL_STATUS ObservationList[], int ObsCount);
void ApplyCorrection(PCHANNEL_STATUS ObservationList[], int ObsCount);

#endif //__SAT_MANAGE_H__
