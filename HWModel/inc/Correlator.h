//----------------------------------------------------------------------
// Correlator.h:
//   Correlator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __CORRELATOR_H__
#define __CORRELATOR_H__

#include "CommonOps.h"
#include "GeneralPrn.h"
#include "WeilPrn.h"
#include "MemoryPrn.h"
#include "PrnGen.h"
#include "NoiseCalc.h"

typedef void (*DumpFunction)(S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int DumpDataLength);

#define PEAK_COR_INDEX 4

class CCorrelator
{
public:
	CCorrelator(unsigned int PolySettings[], unsigned int *MemCodeAddress);
	~CCorrelator();

	CPrnGen *PrnGen[4];				// pointer to difference PRN generator
	CPrnGen *PrnGen2[4];			// pointer to difference PRN generator
	CNoiseCalc *NoiseCalc;			// pointer to noise floor calculator

	reg_uint CarrierFreq;			// 32bit RO
	reg_uint CodeFreq;				// 32bit RO
	reg_uint PreShiftBits;			// 2bit RO
	reg_uint PostShiftBits;			// 2bit RO
	reg_uint DataInQBranch;			// 1bit RO
	reg_uint EnableSecondPrn;		// 1bit RO
	reg_uint EnableBOC;				// 1bit RO
	reg_uint DecodeBit;				// 2bit RO
	reg_uint NarrowFactor;			// 2bit RO
	reg_uint BitLength;				// 5bit RO
	reg_uint CoherentNumber;		// 6bit RO
	reg_uint NHCode;				// 25bit RO
	reg_uint NHLength;				// 5bit RO
	reg_uint DumpLength;			// 16bit RO

	reg_uint CarrierPhase;			// 32bit RW
	reg_uint CarrierCount;			// 32bit RW
	reg_uint CodePhase;				// 32bit RW
	reg_uint PrnCode;				// 7bit RW (7bit store in bit1~bit7, bit0 store output of PrnGen
	reg_int JumpCount;				// 8bit RW
	reg_uint DumpCount;				// 16bit RW
	reg_uint CoherentDone;			// 1bit RW
	reg_uint OverwriteProtect;		// 1bit RW
	reg_uint CurrentCor;			// 3bit RW
	reg_uint Dumping;				// 1bit RW
	reg_uint CodeSubPhase;			// 1bit RW
	reg_uint PrnCode2;				// 4bit RW
	reg_uint BitCount;				// 5bit RW
	reg_uint CoherentCount;			// 6bit RW
	reg_uint NHCount;				// 5bit RW
	reg_uint DecodeData;			// 32bit RW

	S16 AccDataI[8];				// 16bit RW
	S16 AccDataQ[8];				// 16bit RW
	int PrnIndex, PrnIndex2;

	// overwrite protect registers, internal use, will be cleared at the beginning of every round
	int FirstCorIndexValid;			// 1bit
	int FirstCorIndex;				// 4bit
	// data decode valid, internal use, will be cleared at the beginning of every round
	int DataDecodeValid;

	void Reset();
	int Correlation(int SampleNumber, complex_int SampleData[], S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int &DumpDataLength);
	void FillState(unsigned int *StateBuffer);
	void DumpState(unsigned int *StateBuffer);
	void DecodeDataAcc(unsigned int DataAcc);
	complex_int DownConvert(complex_int InputData);
	void AccumulateSample(complex_int Sample, int CorCount);
	int ProcessOverflow(S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int &CurrentLength);
	int DumpData(S16 DumpDataI[], S16 DumpDataQ[], int CorIndex[], int &CurrentLength);

	DumpFunction DumpDataOutput;	// for debug purpose

	static const complex_int DownConvertTable[64];
};

#endif // __CORRELATOR_H__
