//----------------------------------------------------------------------
// GpsFrame.h:
//   Declaration of GPS frame process functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __GPS_FRAME_H__
#define __GPS_FRAME_H__

#include "DataTypes.h"

int GpsNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode);
//void GpsFrameSync(PCHANNEL_STATUS pChannelStatus, int data_count, unsigned int data0, unsigned data1, int EstimateTow);
void GpsFastFrameSync(PCHANNEL_STATUS pChannelStatus, PCHANNEL_STATUS pChannelRef, PBB_MEASUREMENT pMsr, PBB_MEASUREMENT pMsrRef);
void GpsPredictFrameSync(PCHANNEL_STATUS pChannelStatus, PBB_MEASUREMENT pMsr, int GpsMsCount);
unsigned int GetParity(unsigned int word);

#endif //__GPS_FRAME_H__
