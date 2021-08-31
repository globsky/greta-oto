//----------------------------------------------------------------------
// CommonDefines.h:
//   Type definitions for entire firmware
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __COMMON_DEFINES_H__
#define __COMMON_DEFINES_H__

#if defined NULL
#undef NULL
#endif
#define NULL ((void *)0)

//==========================
// type definitions
//==========================
typedef signed char S8;
typedef unsigned char U8;
typedef signed short S16;
typedef unsigned short U16;
typedef int S32;
typedef unsigned int U32;
typedef long long S64;
typedef unsigned long long U64;

//==========================
// macros
//==========================
// macros for bit field manipulate
#if !defined BIT_MASK
#define BIT_MASK(n) ((1 << (n)) - 1)
#endif
#if !defined EXTRACT_UINT
#define EXTRACT_UINT(data, start_bit, length) ((U32)((data >> start_bit) & BIT_MASK(length)))
#endif
#if !defined EXTRACT_INT
#define EXTRACT_INT(data, start_bit, length) ((S32)(data << (32 - start_bit - length)) >> (32 - length))
#endif
#define SET_FIELD(dest, start_bit, length, data) \
do \
{ \
	(dest) &= ~(BIT_MASK(length) << start_bit); \
	(dest) |= (((data) & BIT_MASK(length)) << start_bit); \
} while(0)

#define SET_BIT32(Dest, Position) ((Dest) |= (1U << Position))
#define CLEAR_BIT32(Dest, Position) ((Dest) &= ~(1U << Position))
#define SET_BIT64(Dest, Position) ((Dest) |= (1ULL << Position))
#define CLEAR_BIT64(Dest, Position) ((Dest) &= ~(1ULL << Position))

//==========================
// baseband configurations
//==========================
#define TOTAL_CHANNEL_NUMBER 32

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
// front-end settings
//==========================
#if !defined IF_FREQ
#define IF_FREQ 141000
#endif
#if !defined SAMPLE_FREQ
#define SAMPLE_FREQ 4113000
#endif
#if (SAMPLE_FREQ % 1000) != 0
#error sample frequency should be multiple of 1000
#endif
#define RF_FREQ 1575420000
#define IF_FREQ_BOC (IF_FREQ + 1023000)
#define SAMPLE_COUNT (SAMPLE_FREQ / 1000)
#define GPS_L1_WAVELENGTH 0.19029367279836488

#define DIVIDE_ROUND(divident, divisor) (S32)(((divident) + divisor/2) / divisor)
#define CARRIER_FREQ(doppler) DIVIDE_ROUND(((S64)(IF_FREQ + (doppler))) << 32, SAMPLE_FREQ)	// multiply 2^32/fs
#define CARRIER_FREQ_BOC(doppler) DIVIDE_ROUND(((S64)(IF_FREQ_BOC + (doppler))) << 32, SAMPLE_FREQ)	// multiply 2^32/fs
#define CODE_FREQ(doppler) DIVIDE_ROUND((((S64)(RF_FREQ + (doppler))) << 32) / 770, SAMPLE_FREQ)	// (RF + Doppler)/770 multiply 2^32/fs
#define AE_STRIDE_INTERVAL(freq) DIVIDE_ROUND((S64)(freq) << 32, 2046000)	// multiply 2^32/2046000
#define AE_CENTER_FREQ(freq) DIVIDE_ROUND((S64)(freq) << 20, 2046000)	// multiply 2^20/2046000

//==========================
// baseband measurement send to PVT
//==========================
#pragma pack(push)	// push current alignment
#pragma pack(4)		// set alignment to 4-byte boundary

typedef struct
{	
	U8 UserChannel;		// user channel number (reserved for future use)
	U8 LogicChannel;	// physical channel number (map to hardware logic channel)
	U8 FreqID;	// system and frequency
	U8 Svid;	// SVID start from 1
	U32 State;	// bitwise flags and indicators
	int TrackingTime;	// millisecond at current stage (reset at stage swith or at phase loss lock at final stage)
	// above variables same as CHANNEL_STATE
	S32 CarrierFreq;	// carrier frequency control word
	U32 CarrierNCO;		// carrier NCO count
	S32 CarrierCount;	// carrier cycle count
	U32 CodeFreq;		// code frequency
	U32 CodeNCO;		// code NCO
	S32 CodeCount;		// code count in current bit, unit is 1/2 chip
	U16 DataNumber;		// data number in stream buffer
	U16 CN0;			// C/N0 in unit of 0.01dB
	U32 LockIndicator;	// carrier lock indicator
	U32 *DataStreamAddr;	// address of data in data stream buffer
} BB_MEASUREMENT, *PBB_MEASUREMENT;

typedef struct
{
	unsigned int MeasMask;
	unsigned int RunTimeAcc;
	int MeasInterval;
	PBB_MEASUREMENT Measurements;
} BB_MEAS_PARAM, *PBB_MEAS_PARAM;

#pragma pack(pop)	//restore original alignment

#endif	// __COMMON_DEFINES_H__
