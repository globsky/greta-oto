//----------------------------------------------------------------------
// AcqEngine.h:
//   Acquisition engine class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __ACQ_ENGINE_H__
#define __ACQ_ENGINE_H__

#include "CommonOps.h"
#include "RegAddress.h"
#include "GeneralPrn.h"
#include "MemoryPrn.h"
#include "WeilPrn.h"
#include "PeakSorter.h"
#include "RateAdaptor.h"

// definitions for intermediate data output
#define INTERMEDIATE_RESULT_SAMPLE2BUFFER   0
#define INTERMEDIATE_RESULT_MF_OUTPUT_DEC   1
#define INTERMEDIATE_RESULT_MF_OUTPUT_HEX   2
#define INTERMEDIATE_RESULT_COH_ACC         3
#define INTERMEDIATE_RESULT_LAST_COH_ACC    4
#define INTERMEDIATE_RESULT_NONCOH_ACC      5
#define INTERMEDIATE_RESULT_LAST_NONCOH_ACC 6
#define INTERMEDIATE_RESULT_INSERT_PEAK     7
#define INTERMEDIATE_RESULT_MAX_PEAKS       8
#define TOTAL_INTERMEDIATE_RESULT           9

#define FULL_LENGTH 0

#define AE_BUFFER_SIZE 128*1024
#define MAX_CHANNEL 32
#define CHANNEL_CONFIG_LEN 8
#define MF_CORE_DEPTH (FULL_LENGTH ? 2046 : 682)
#define ADDER_TREE_WIDTH (MF_CORE_DEPTH/2)
#define DFT_NUMBER 8

struct complex_exp10 {
	int real;	// 10bit
	int imag;	// 10bit
	int exp;	// 4bit

	complex_exp10() {};
	complex_exp10(complex_int data);
	void operator = (complex_int data);
	complex_exp10 operator + (complex_int data);
	void operator += (complex_int data);
	complex_exp10 operator - (complex_int data);
	void operator -= (complex_int data);
};

class CAcqEngine
{
public:
	// global registers
	reg_uint ChannelNumber;		// 6bit
	reg_uint BufferThreshold;	// 7bit

	// channel search configuration registers
	reg_uint EarlyTerminate;	// 1bit
	reg_uint PeakRatioTh;		// 3bit
	reg_uint StrideNumber;		// 6bit
	reg_uint CoherentNumber;	// 6bit
	reg_uint NonCoherentNumber;	// 7bit
	reg_int  CenterFreq;		// 32bit
	reg_uint Svid;				// 6bit
	reg_uint PrnSelect;			// 2bit
	reg_uint CodeSpan;			// 5bit
	reg_uint ReadAddress;		// 5bit
	reg_uint DftFreq;			// 11bit
	reg_uint StrideInterval;	// 22bit

	CAcqEngine(unsigned int *MemCodeAddress);
	~CAcqEngine();

	// interface functions for acquisition
	void Reset();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);
	void DoAcquisition();
	// interface functions for AE buffer
	void StartFill() { WritePointer = 0; Filling = 1; }
	int WriteSample(int Length, unsigned char Sample[]);
	int IsFillingBuffer() { return Filling;}

	// internal functions
	complex_int ReadSampleFromFifo();
	complex_int ReadSampleToBuffer();
	void PreloadSample();
	void LoadSample();
	void LoadCode();
	void MatchFilterCore(int PhaseCount, complex_int CorResult[]);
	void GetDftFactor(complex_int DftFactor[DFT_NUMBER/2], int sign_cos[DFT_NUMBER/2], int sign_sin[DFT_NUMBER/2]);
	void NonCoherentAcc(unsigned int MaxCohExp, int NoncohCount);
	void DoNonCoherentSum();
	void InsertPeak(int Amp, int Exp, int PartialCorPos, int PartialFreq);
	int PeakFound();
	void SearchOneChannel();
	unsigned int Amplitude(complex_exp10 data);
	void InitPrnGen();
	void SetStartAddr(int Addr) { ReadPointer = Addr; }
	unsigned int ReadSample();

	static const int complex_mul_i[16][64];
	static const int complex_mul_q[16][64];
	static const int CAcqEngine::dft_table[128];
	static const unsigned int PrnPolySettings[2];	// GPS L1CA polynomial settings
	static const unsigned int GpsInit[32+19];	// WAAS placed after GPS
	static const unsigned int B1CInit[63];		// initial value for B1C code generation
	static const unsigned int L1CInit[63];		// initial value for L1C code generation

	// internal registers
	complex_int AcqSamples[MF_CORE_DEPTH*2];	// 6bit x 2(I/Q) x MF_CORE_DEPTH, software will double the length to avoid shift
	unsigned int AcqCode[ADDER_TREE_WIDTH];		// 1bit, two sets of AcqCode in RTL, toggle after each use
	complex_int LastInput;
	unsigned int CarrierNco;	// 32bit
	int CarrierFreq;			// 32bit
	unsigned int DftNco;		// 14bit
	unsigned int NoiseFloor;	// 18bit
	int Success;				// 1bit
	int CodeRoundCount;			// 5bit, counter for CodeSpan
	unsigned int StrideCount;	// 6bit unsigned
	int StrideOffset;			// 6bit signed
	unsigned int NoncohExp;		// 4bit unsigned
	unsigned int ExpIncPos;		// 10bit unsigned

	int ReadPointer;
	int WritePointer;
	int Filling;

	// internal RAM
	complex_exp10 CoherentBuffer[MF_CORE_DEPTH][DFT_NUMBER];
	unsigned long long NonCoherentBuffer[MF_CORE_DEPTH+2];
	unsigned int ChannelConfig[MAX_CHANNEL][CHANNEL_CONFIG_LEN];
	unsigned char AEBuffer[AE_BUFFER_SIZE];

	CPrnGen *PrnGen[4];
	CPeakSorter PeakSorter;
	CRateAdaptor RateAdaptor;

	// for reference intermediate result output
	FILE *fp_out[TOTAL_INTERMEDIATE_RESULT];
};

#endif //__ACQ_ENGINE_H__
