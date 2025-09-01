//----------------------------------------------------------------------
// GnssTop.cpp:
//   GNSS baseband top module class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __BB_TOP_H__
#define __BB_TOP_H__

#include "CommonOps.h"
#include "IfFile.h"
//#include "IfInterface.h"
//#include "PreProcess.h"
//#include "NoiseCalculate.h"
#include "TeFifoMem.h"
//#include "AeFifo.h"
#include "TrackingEngine.h"
#include "AcqEngine.h"

#define MAX_BLOCK_SIZE 25000	// usually defined as samples per millisecond at maximum possible sampling rate

typedef void (*InterruptFunction)();

class CGnssTop
{
public:
	CGnssTop();
	~CGnssTop();
	void Reset(U32 ResetMask);
	void Clear(U32 ClearMask);
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);

	reg_uint TrackingEngineEnable;		// 1bit
//	reg_uint AcquireEngineEnable;		// 1bit
//	reg_uint PreProcessEnable;			// 8bit
	reg_uint MeasurementNumber;			// 10bit
	reg_uint MeasurementCount;			// 10bit
	reg_uint ReqCount;					// 10bit
	reg_uint InterruptFlag;				// 4bit, bit8~11
	reg_uint IntMask;					// 4bit, bit8~11
	reg_uint TickCount;					// 32bit

	unsigned int MemCodeBuffer[128*100];
	CIfFile IfFile;
	CTeFifoMem TeFifo;
	CTrackingEngine TrackingEngine;
	CAcqEngine AcqEngine;
	complex_int *FileData;
	unsigned char *SampleQuant;
	int AeProcessCount;		// simulate AE acquisition process delay

	int Process(int ReadBlockSize);
	void SetInputFile(char *FileName) { IfFile.OpenIfFile(FileName); }
	int GetAeProcessTime();

	InterruptFunction InterruptService;
};

#endif //__BB_TOP_H__
