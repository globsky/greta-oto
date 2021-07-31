//----------------------------------------------------------------------
// BBTypes.h:
//   Type definitions and macros for baseband
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __BB_DEFINES_H__
#define __BB_DEFINES_H__

#include "CommonDefines.h"

//==========================
// type definitions
//==========================
typedef struct
{
	U32 CarrierFreq;    // 0
	U32 CodeFreq;		// 1
	U32 CorrConfig;		// 2
	U32 NHConfig;       // 3
	U32 CohConfig;		// 4
	U32 PrnConfig;      // 5 
	U32 PrnState;       // 6
	U32 PrnCount;       // 7
	U32 CarrierPhase;	// 8
	U32 CarrierCount;	// 9
	U32 CodePhase;		// 10
	U32 DumpCount;	    // 11
	U32 CorrState;		// 12
	U32 MsData;			// 13
	U32 PrnConfig2;     // 14 
	U32 PrnState2;      // 15
	U32 PartialAcc[8];  // 16~23
	U32 CoherentSum[8]; // 24~31
} STATE_BUFFER, *PSTATE_BUFFER;

//==========================
// macro for state buffer
//==========================
// configuration for whole DWORD
#define STATE_BUF_SET_CARRIER_FREQ(pStateBuffer, Freq) ((pStateBuffer)->CarrierFreq = (Freq))
#define STATE_BUF_SET_CODE_FREQ(pStateBuffer, Freq) ((pStateBuffer)->CodeFreq = (Freq))
#define STATE_BUF_SET_CORR_CONFIG(pStateBuffer, DumpLength, NarrowFactor, SecondPrn, DataInQ, EnableBOC, PreShift) (pStateBuffer)->CorrConfig = ((DumpLength << 16) | (NarrowFactor << 8) | (SecondPrn << 4) | (DataInQ << 3) | (EnableBOC << 2) | PreShift)
#define STATE_BUF_SET_NH_CONFIG(pStateBuffer, NHLength, NHCode) (pStateBuffer)->NHConfig = ((NHLength << 27) | NHCode)
#define STATE_BUF_SET_COH_CONFIG(pStateBuffer, PostShift, CohNumber, MsDataNumber, NHCode2) (pStateBuffer)->CohConfig = ((PostShift << 30) | (CohNumber << 25) | (MsDataNumber << 20) | NHCode2)

// configuration for individual field
#define STATE_BUF_SET_DUMP_LENGTH(pStateBuffer, DumpLength)     SET_FIELD((pStateBuffer)->CorrConfig, 16, 16, DumpLength)
#define STATE_BUF_SET_NARROW_FACTOR(pStateBuffer, NarrowFactor) SET_FIELD((pStateBuffer)->CorrConfig, 8, 2, NarrowFactor)
#define STATE_BUF_SET_FRE_SHIFT(pStateBuffer, PreShift)         SET_FIELD((pStateBuffer)->CorrConfig, 0, 2, PreShift)
#define STATE_BUF_ENABLE_PRN2(pStateBuffer)                     SET_BIT32((pStateBuffer)->CorrConfig, 4)
#define STATE_BUF_DISABLE_PRN2(pStateBuffer)                    CLEAR_BIT32((pStateBuffer)->CorrConfig, 4)
#define STATE_BUF_DATA_IN_I(pStateBuffer)                       CLEAR_BIT32((pStateBuffer)->CorrConfig, 3)
#define STATE_BUF_DATA_IN_Q(pStateBuffer)                       SET_BIT32((pStateBuffer)->CorrConfig, 3)
#define STATE_BUF_ENABLE_BOC(pStateBuffer)                      SET_BIT32((pStateBuffer)->CorrConfig, 2)
#define STATE_BUF_DISABLE_BOC(pStateBuffer)                     CLEAR_BIT32((pStateBuffer)->CorrConfig, 2)

#define STATE_BUF_SET_NH_LENGTH(pStateBuffer, NHLength)         SET_FIELD((pStateBuffer)->NHConfig, 27, 5, NHLength)
#define STATE_BUF_SET_NH_CODE(pStateBuffer, NHCode)             SET_FIELD((pStateBuffer)->NHConfig, 0, 25, NHCode)

#define STATE_BUF_SET_POST_SHIFT(pStateBuffer, PostShift)       SET_FIELD((pStateBuffer)->CohConfig, 30, 2, PostShift)
#define STATE_BUF_SET_COH_NUMBER(pStateBuffer, CohNumber)       SET_FIELD((pStateBuffer)->CohConfig, 25, 5, CohNumber)
#define STATE_BUF_SET_MSDATA_NUMBER(pStateBuffer, MsDataNumber) SET_FIELD((pStateBuffer)->CohConfig, 20, 5, MsDataNumber)
#define STATE_BUF_SET_NH_CODE2(pStateBuffer, NHCode2)           SET_FIELD((pStateBuffer)->CohConfig, 0, 20, NHCode2)

// configuration and status field get value
#define STATE_BUF_GET_CARRIER_FREQ(pStateBuffer)   ((pStateBuffer)->CarrierFreq)
#define STATE_BUF_GET_CODE_FREQ(pStateBuffer)      ((pStateBuffer)->CodeFreq)
#define STATE_BUF_GET_CARRIER_PHASE(pStateBuffer)  ((pStateBuffer)->CarrierPhase)
#define STATE_BUF_GET_CARRIER_COUNT(pStateBuffer)  ((pStateBuffer)->CarrierCount)
#define STATE_BUF_GET_CODE_PHASE(pStateBuffer)     ((pStateBuffer)->CodePhase)
#define STATE_BUF_GET_DUMP_COUNT(pStateBuffer)     EXTRACT_UINT((pStateBuffer)->DumpCount, 16, 16)
#define STATE_BUF_GET_NH_COUNT(pStateBuffer)       EXTRACT_UINT((pStateBuffer)->CorrState, 27, 5)
#define STATE_BUF_GET_COH_COUNT(pStateBuffer)      EXTRACT_UINT((pStateBuffer)->CorrState, 21, 5)
#define STATE_BUF_GET_MSDATA_COUNT(pStateBuffer)   EXTRACT_UINT((pStateBuffer)->CorrState, 16, 5)
#define STATE_BUF_GET_CODE_SUB_PHASE(pStateBuffer) EXTRACT_UINT((pStateBuffer)->CorrState, 8, 1)
#define STATE_BUF_GET_DUMPING(pStateBuffer)        EXTRACT_UINT((pStateBuffer)->CorrState, 7, 1)
#define STATE_BUF_GET_CUR_CORR(pStateBuffer)       EXTRACT_UINT((pStateBuffer)->CorrState, 4, 3)
#define STATE_BUF_IS_COH_DONE(pStateBuffer)        ((pStateBuffer)->CorrState & 1)
#define STATE_BUF_IS_MSDATA_DONE(pStateBuffer)     ((pStateBuffer)->CorrState & 2)
#define STATE_BUF_GET_MSDATA(pStateBuffer)         EXTRACT_INT((pStateBuffer)->MsData, 0, 16)

// status field set value
#define STATE_BUF_SET_CARRIER_COUNT(pStateBuffer, Count)     ((pStateBuffer)->CarrierCount = Count)
#define STATE_BUF_SET_CODE_PHASE(pStateBuffer, Phase)        ((pStateBuffer)->CodePhase = Phase)
#define STATE_BUF_SET_DUMP_COUNT(pStateBuffer, Count)        SET_FIELD((pStateBuffer)->DumpCount, 16, 16, Count)
#define STATE_BUF_SET_NH_COUNT(pStateBuffer, Count)          SET_FIELD((pStateBuffer)->CorrState, 27, 5, Count)
#define STATE_BUF_SET_COH_COUNT(pStateBuffer, Count)         SET_FIELD((pStateBuffer)->CorrState, 21, 5, Count)
#define STATE_BUF_SET_MSDATA_COUNT(pStateBuffer, Count)      SET_FIELD((pStateBuffer)->CorrState, 16, 5, Count)
#define STATE_BUF_SET_CODE_SUB_PHASE(pStateBuffer, SubPhase) SET_FIELD((pStateBuffer)->CorrState, 8, 1, SubPhase)

// PRN configuration
#define STATE_BUF_SET_PRN_L1CA(pStateBuffer, Svid, StartPhase) \
do{ pStateBuffer->PrnConfig = CAPrnInit[Svid-1]; \
	pStateBuffer->PrnCount = (StartPhase) << 14; } while(0)

#define STATE_BUF_SET_PRN_E1(pStateBuffer, Svid, StartPhase) \
do{ pStateBuffer->PrnConfig = 0xc0000004 + ((49 + Svid) << 6); \
	pStateBuffer->PrnCount = ((StartPhase / 1023) << 10) + (StartPhase % 1023); } while(0)

#define STATE_BUF_SET_PRN_B1C(pStateBuffer, Svid, StartPhase) \
do{ pStateBuffer->PrnConfig = B1CPilotInit[Svid-1]; \
	pStateBuffer->PrnCount = StartPhase; } while(0)

#define STATE_BUF_SET_PRN_L1C(pStateBuffer, Svid, StartPhase) \
do{ pStateBuffer->PrnConfig = B1CPilotInit[Svid-1]; \
	pStateBuffer->PrnCount = StartPhase; } while(0)


//==========================
// state definitions
//==========================
// bit0~3 for tracking stage, 2MSB not zero means tracking loop required
#define STAGE_RELEASE  0x0		// channel need to be released
#define STAGE_HOLD1    0x1		// tracking hold for AE acquisition, channel as place holder
#define STAGE_HOLD2    0x2		// tracking hold for TE acquisition, do correlation as normal
#define STAGE_HOLD3    0x3		// tracking hold for signal lost, do correlation as normal
#define STAGE_PULL_IN  0x4
#define STAGE_BIT_SYNC 0x5
#define STAGE_TRACK    0x8		// bit3 set as normal tracking, 3LSB for track stage

#define STATE_TRACKING_LOOP 0xc
#define STAGE_MASK          0xf

// bit6~7 for tracking signal
#define STATE_TRACK_I  0x40
#define STATE_TRACK_Q  0x80
#define STATE_TRACK_IQ 0xc0

// bit8~9 for decode data stream type
#define DATA_STREAM_NONE 0x000	// do not decode data stream
#define DATA_STREAM_1BIT 0x100	// each symbol occupies 1bit
#define DATA_STREAM_4BIT 0x200	// each symbol occupies 4bit
#define DATA_STREAM_8BIT 0x300	// each symbol occupies 8bit
#define DATA_STREAM_MASK 0x300

// bit16~18 for tracking loop update
#define TRACKING_UPDATE_PLL 0x10000
#define TRACKING_UPDATE_FLL 0x20000
#define TRACKING_UPDATE_DLL 0x40000
#define TRACKING_UPDATE (TRACKING_UPDATE_PLL | TRACKING_UPDATE_FLL | TRACKING_UPDATE_DLL)

// bit20~23 for cache state
#define STATE_CACHE_FREQ_DIRTY   0x100000
#define STATE_CACHE_CONFIG_DIRTY 0x200000
#define STATE_CACHE_CODE_DIRTY   0x400000
#define STATE_CACHE_STATUS_DIRTY 0x800000
#define STATE_CACHE_DIRTY (STATE_CACHE_FREQ_DIRTY | STATE_CACHE_CONFIG_DIRTY | STATE_CACHE_CODE_DIRTY | STATE_CACHE_STATUS_DIRTY)

#define SYNC_CACHE_READ_DATA   1
#define SYNC_CACHE_READ_STATUS 2

//==========================
// frequency ID definitions
//==========================
#define FREQ_L1CA 0
#define FREQ_E1   1
#define FREQ_B1C  2
#define FREQ_L1C  3
// 2MSB mark as data/pilot
#define FREQ_DATA_CHANNEL 0x80
#define FREQ_PILOT_CHANNEL 0x40
// data/pilot channel of frequency ID
#define FREQ_E1B  (FREQ_E1  | FREQ_DATA_CHANNEL)
#define FREQ_B1CD (FREQ_B1C | FREQ_DATA_CHANNEL)
#define FREQ_L1CD (FREQ_L1C | FREQ_DATA_CHANNEL)
#define FREQ_E1C  (FREQ_E1  | FREQ_PILOT_CHANNEL)
#define FREQ_B1CP (FREQ_B1C | FREQ_PILOT_CHANNEL)
#define FREQ_L1CP (FREQ_L1C | FREQ_PILOT_CHANNEL)

#endif	// __BB_DEFINES_H__
