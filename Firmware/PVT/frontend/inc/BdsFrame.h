//----------------------------------------------------------------------
// BdsFrame.h:
//   Declaration of BDS frame process functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __BDS_FRAME_H__
#define __BDS_FRAME_H__

#include "DataTypes.h"

int BdsNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode);

#endif //__BDS_FRAME_H__
