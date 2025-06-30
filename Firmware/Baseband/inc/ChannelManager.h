//----------------------------------------------------------------------
// ChannelManager.h:
//   Channel management functions and definitions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __CHANNEL_MANAGER_H__
#define __CHANNEL_MANAGER_H__

#include "CommonDefines.h"
#include "BBDefines.h"

#define CORRELATOR_NUM 7
#define MAX_FFT_NUM    8
#define MAX_BIN_NUM    8
#define COH_BUF_LEN    (CORRELATOR_NUM * MAX_FFT_NUM)
#define NONCOH_BUF_LEN (CORRELATOR_NUM * MAX_BIN_NUM)

#pragma pack(push)	// push current alignment
#pragma pack(4)		// set alignment to 4-byte boundary

struct tag_CHANNEL_STATE;

typedef struct
{
	int CorDataCount;	// number of data in CorDat
	int TimeTag;		// time tag (assigned by TrackingTime) when sending CorData to bit sync task
	struct tag_CHANNEL_STATE *ChannelState;	// channel to send correlation result
	U32 PrevCorData;	// correlation result preceding to next 20 results (or flag for pilot data sync)
	U32 PolarityToggle;	// bit to indicate whether there is polarity toggle (LSB as latest)
//	U32 CorData[20];	// 20 correlation result of peak correlator (or pilot data symbols for pilot data sync)
} BIT_SYNC_DATA, *PBIT_SYNC_DATA;

// channel related functions and variables
typedef struct tag_CHANNEL_STATE
{
	U8 UserChannel;		// user channel number (reserved for future use)
	U8 LogicChannel;	// physical channel number (map to hardware logic channel)
	U8 FreqID;	// system and frequency
	U8 Svid;	// SVID start from 1
	U32 State;	// bitwise flags and indicators
	// following variable for tracking stage
	int TrackingTime;		// millisecond at current stage (reset at stage swith or at phase loss lock at final stage)
	int TrackingTimeout;	// millisecond for current stage timeout (-1 for final stage)
	// state buffer cache and pointer to hardware buffer
	STATE_BUFFER StateBufferCache;	// local image of state buffer
	volatile PSTATE_BUFFER StateBufferHW;	// pointer to hardware state buffer
	// accumulation number and counter
	int CoherentNumber;		// same as coherent number settings in CohConfig field of state buffer
	int FftNumber;			// 1 for PLL only (no FLL), 2 for cross-dot (FLL), 3~8 for FFT
	int NonCohNumber;
	int FftCount;
	int NonCohCount;
	int SkipCount;	// number of coherent data to skip
	int PendingCount;	// indicate how many correlation result stored but not processed
	U32 PendingCoh[8];	// coherent result stored but not processed
	// following variables for tracking loop
	int PhaseDiff;	// phase discriminator result
	int PhaseAcc;	// phase discriminator accumulation value
	int FrequencyDiff;	// frequency discriminator result
	int FrequencyAcc;	// frequency discriminator accumulation value
	int DelayDiff;	// delay discriminator result
	int DelayAcc;	// delay discriminator accumulation value
	U32 CarrierFreqBase;	// Wn for carrier frequency control word
	U32 CodeFreqBase;		// W0 for code frequency control word
	U32 CarrierFreqSave;	// carrier frequency control word when PLL/FLL lock
	U32 CodeFreqSave;		// code frequency control word when DLL lock
	int CodeSearchCount;	// counter on search range on correlator acquisition or tracking hold
	// buffer for coherent and non-coherent sum
	U32 CohBuffer[COH_BUF_LEN];	// buffer to hold coherent sums
	int NoncohBuffer[NONCOH_BUF_LEN];	// buffer to hold noncoherent sums
	// data for bit sync
	BIT_SYNC_DATA BitSyncData;
	int ToggleCount[20];	// toggle count for each position (only BitSyncTask will access this array)
	int BitSyncResult;		// set by BitSyncTask, -1 for bit sync fail, 0 for bit sync in process, 1~20 as bit sync position, 21 as switch to tracking from bit sync
	// data for data stream decode
	DATA_STREAM DataStream;
	int FrameCounter;		// current data/secondary code position in frame
	// tracking loop coefficients
	int pll_k1, pll_k2, pll_k3;	// maximum 3rd order
	int fll_k1, fll_k2;			// maximum 2nd order
	int dll_k1, dll_k2;			// maximum 2nd order
	// C/N0 calculation
	int PeakPower;		// peak amplitude
	int SmoothedPower;	// smoothed power
	int CN0, FastCN0;	// smoothed C/N0 and instantaneous C/N0
	int CN0HighCount, CNOLowCount;	// count for CN0 high and low
	// lock detector
	int PLD, FLD, DLD;	// 0 to 100 as indicator of lock quality
	int LoseLockCounter;
} CHANNEL_STATE, *PCHANNEL_STATE;

typedef struct
{
	int CoherentNumber;		// same as coherent number settings in CohConfig field of state buffer
	int FftNumber;			// 1 for PLL only (no FLL), 2 for cross-dot (FLL), 3~8 for FFT
	int NonCohNumber;
	int NarrowFactor;
	int PostShift;
	int BandWidthPLL16x;	// loop filter bandwidth for PLL with scale factor 1/16Hz, bit16~17 as order
	int BandWidthFLL16x;	// loop filter bandwidth for FLL with scale factor 1/16Hz, bit16~17 as order
	int BandWidthDLL16x;	// loop filter bandwidth for DLL with scale factor 1/16Hz, bit16~17 as order
	int TrackingTimeout;	// timeout for current tracking stage, -1 for no timeout
} TRACKING_CONFIG, *PTRACKING_CONFIG;

typedef struct
{
	int FreqBinDiff;
	int CorDiff;
	int PeakPower;
	int EarlyPower;
	int LatePower;
	int LeftBinPower;
	int RightBinPower;
} SEARCH_PEAK_RESULT, *PSEARCH_PEAK_RESULT;

#pragma pack(pop)	//restore original alignment

extern CHANNEL_STATE ChannelStateArray[TOTAL_CHANNEL_NUMBER];
void InitChannel(PCHANNEL_STATE pChannel);
void ConfigChannel(PCHANNEL_STATE pChannel, int Doppler, int CodePhase16x);
void SyncCacheWrite(PCHANNEL_STATE ChannelState);
void ProcessCohSum(int ChannelID, unsigned int OverwriteProtect);
int ComposeMeasurement(int ChannelID, PBB_MEASUREMENT Measurement, U32 *DataBuffer);

#endif // __CHANNEL_MANAGER_H__
