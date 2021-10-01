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

// PRN field set value
#define STATE_BUF_SET_PRN_CONFIG(pStateBuffer, Config) ((pStateBuffer)->PrnConfig = Config)
#define STATE_BUF_SET_PRN_CONFIG2(pStateBuffer, Config) ((pStateBuffer)->PrnConfig2 = Config)
#define STATE_BUF_SET_PRN_COUNT(pStateBuffer, Count) ((pStateBuffer)->PrnCount = Count)

// PRN configuration and count for different signal
#define PRN_CONFIG_L1CA(Svid) (CAPrnInit[Svid-1])
#define PRN_CONFIG_E1(Svid)   (0xc0000004 + ((49 + Svid) << 6))
#define PRN_CONFIG_B1C(Svid)  (B1CPilotInit[Svid-1])
#define PRN_CONFIG_L1C(Svid)  (L1CPilotInit[Svid-1])
#define PRN_CONFIG2_E1(Svid)  (0xc0000004 + ((Svid-1) << 6))
#define PRN_CONFIG2_B1C(Svid) (B1CDataInit[Svid-1])
#define PRN_CONFIG2_L1C(Svid) (L1CDataInit[Svid-1])
#define PRN_COUNT_L1CA(StartPhase) ((StartPhase) << 14)
#define PRN_COUNT_E1(StartPhase)   (((StartPhase / 1023) << 10) + (StartPhase % 1023))
#define PRN_COUNT_B1C(StartPhase)  (StartPhase)
#define PRN_COUNT_L1C(StartPhase)  (StartPhase)

#endif	// __BB_DEFINES_H__
