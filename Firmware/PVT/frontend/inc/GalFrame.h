//----------------------------------------------------------------------
// GalFrame.h:
//   Declaration of Galileo I/NAV frame process functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __GAL_FRAME_H__
#define __GAL_FRAME_H__

#include "DataTypes.h"

int GalNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode);

#endif //__GAL_FRAME_H__
