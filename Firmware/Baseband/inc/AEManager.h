//----------------------------------------------------------------------
// AEManager.h:
//   Acquisition engine management functions and definitions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __AE_MANAGER_H__
#define __AE_MANAGER_H__

#include "CommonDefines.h"

typedef struct
{
	U8 FreqSvid;		// 2MSB as FreqID and 6LSB as SVID
	U8 CodeSpan;
	S16 CenterFreq;
} ACQ_SAT_CONFIG, *PACQ_SAT_CONFIG;

typedef struct
{
	int AcqChNumber;
	int CohNumber;
	int NoncohNumber;
	int StrideNumber;
	int StrideInterval;
	ACQ_SAT_CONFIG SatConfig[TOTAL_CHANNEL_NUMBER];
} ACQ_CONFIG, *PACQ_CONFIG;

ACQ_CONFIG AcqConfig;
int AcquisitionTask(void *Param);
int ProcessAcqResult(void *Param);

#endif // __AE_MANAGER_H__
