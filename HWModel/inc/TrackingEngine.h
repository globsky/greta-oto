#if !defined __TRACKING_ENGINE_LITE_H__
#define __TRACKING_ENGINE_LITE_H__

#include "CommonOps.h"
#include "Correlator.h"
#include "TeFifoMem.h"

#define PHYSICAL_CHANNEL_NUMBER 4
#define LOGICAL_CHANNEL_NUMBER 32
#define TE_BUFFER_SIZE (LOGICAL_CHANNEL_NUMBER * 128)

class CTeFifoMem;

class CTrackingEngine
{
public:
	CTrackingEngine(CTeFifoMem *pTeFifo, unsigned int *MemCodeBuffer);
	~CTrackingEngine();
	void Reset();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);

	reg_uint ChannelEnable;				// 32bit
	reg_uint CohDataReady;				// 32bit
	reg_uint OverwriteProtectChannel;	// 32bit
	reg_uint OverwriteProtectAddr;		// 12bit
	reg_uint OverwriteProtectValue;		// 32bit
	reg_uint PrnPolyLength[4];			// 32bit;

	int ProcessData();
	int FindLeastIndex(unsigned int data);

	unsigned int *TEBuffer;
	CTeFifoMem *pTeFifo;
	CCorrelator *Correlator[PHYSICAL_CHANNEL_NUMBER];
	complex_int *FifoData;
	complex_int *DownConvertData;
};

#endif //__TRACKING_ENGINE_LITE_H__
