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
//	reg_uint DataReadyIntMask;			// 1bit
	reg_uint ReqCount;					// 10bit
	reg_uint InterruptFlag;				// 3bit, bit8~10


	int Process(int ReadBlockSize, int RoundNumber);

	unsigned int MemCodeBuffer[128*100];
	CIfFile IfFile;
	CTeFifoMem TeFifo;
//	CNoiseCalculate NoiseCalculate[FREQ_NUMBER];
//	CAeFifo AeFifo;
	CTrackingEngine TrackingEngine;
//	CAcqEngine AcqEngine;
	complex_int *FileData;
//	unsigned char *SampleRequant;
//	U8 *FIFOInputData[FREQ_NUMBER];
//	S8 *IFData[FREQ_NUMBER];
//	S8 *ChDataI[FREQ_NUMBER], *ChDataQ[FREQ_NUMBER];
//	S8 *InputData[FREQ_NUMBER], *OutputI[FREQ_NUMBER], *OutputQ[FREQ_NUMBER];
//	S16 *ResampleI, *ResampleQ;
//	U8 *RequantData;
	InterruptFunction InterruptService;
};

#endif //__BB_TOP_H__
