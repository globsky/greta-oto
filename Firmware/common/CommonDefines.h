//----------------------------------------------------------------------
// CommonTypes:
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
	U32 CodeCount;		// code count in current bit, unit is 1/2 chip
	U16 DataNumber;		// data number in stream buffer
	U16 CN0;			// C/N0 in unit of 0.01dB
	U32 LockIndicator;	// carrier lock indicator
	U32 *DataStreamAddr;	// address of data in data stream buffer
} BB_MEASUREMENT, *PBB_MEASUREMENT;

typedef struct
{
	int MeasNumber;
	int MeasInterval;
	PBB_MEASUREMENT Measurements;
} BB_MEAS_PARAM, *PBB_MEAS_PARAM;

#pragma pack(pop)	//restore original alignment

#endif	// __COMMON_DEFINES_H__
