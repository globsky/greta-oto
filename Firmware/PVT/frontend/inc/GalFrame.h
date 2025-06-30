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

void GalFrameSync(PCHANNEL_STATUS pChannelStatus, int data_count, unsigned int *data, int FrameIndex);

#endif //__GAL_FRAME_H__
