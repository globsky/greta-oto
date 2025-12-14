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
	U8 SignalSvid;		// 2MSB as signal and 6LSB as SVID
	U8 CodeSpan;
	S16 CenterFreq;
} ACQ_SAT_CONFIG, *PACQ_SAT_CONFIG;

typedef struct
{
	int CohNumber;
	int NoncohNumber;
	int StrideNumber;
	int StrideInterval;
} SEARCH_CONFIG, *PSEARCH_CONFIG;

typedef struct
{
//	int SignalType;	// 0: acquire BPSK signal, 1: acquire BOC signal
	int SearchMode;
	int AcqChNumber;
	const SEARCH_CONFIG *SearchConfig;
	ACQ_SAT_CONFIG SatConfig[AE_CHANNEL_NUMBER];
} ACQ_CONFIG, *PACQ_CONFIG;

// SearchMode definitions (bit0 for BPSK/BOC selection, bit1/2 for frequency range, bit3 for sensitivity, bit 4/5 for search stage)
#define SEARCH_MODE_TYPE_BPSK    (0 << 0)
#define SEARCH_MODE_TYPE_BOC     (1 << 0)
#define SEARCH_MODE_TYPE_MASK    (1 << 0)
#define SEARCH_MODE_FREQ_FULL    (0 << 1)
#define SEARCH_MODE_FREQ_WIDE    (1 << 1)
#define SEARCH_MODE_FREQ_NARROW  (2 << 1)
#define SEARCH_MODE_FREQ_SINGLE  (3 << 1)
#define SEARCH_MODE_FREQ_MASK    (3 << 1)
#define SEARCH_MODE_POWER_HI     (0 << 3)
#define SEARCH_MODE_POWER_LO     (1 << 3)
#define SEARCH_MODE_POWER_MASK   (1 << 3)
#define SEARCH_MODE_STAGE_ACQ    (0 << 4)
#define SEARCH_MODE_STAGE_VERIFY (3 << 4)	// always have verify stage as last search stage
#define SEARCH_MODE_STAGE_MASK   (3 << 4)

void AEInitialize(void);
PACQ_CONFIG GetFreeAcqTask(void);
int AddAcqTask(PACQ_CONFIG pAcqConfig);
void AeInterruptProc();
void StartAcquisition(void);
int AcqBufferReachTh(void);
int ProcessAcqResult(void *Param);

#endif // __AE_MANAGER_H__
