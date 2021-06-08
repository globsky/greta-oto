//----------------------------------------------------------------------
// TeFifoMem.h:
//   TE FIFO (with embedded FIFO memory) class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TE_FIFO_MEM_H__
#define __TE_FIFO_MEM_H__

#include "CommonOps.h"

typedef void (*TriggerFunction)(int Index);

class CTeFifoMem
{
public:
	CTeFifoMem(int Index, int Size);
	~CTeFifoMem();
	void Reset();
	void Clear();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);
	void RewindPointer();
	void SkipBlock();
	int WriteData(complex_int Data);
	void ReadData(int &ReadNumber, complex_int Data[]);
	void LatchWriteAddress(int Source);
	void SetFifoEnable(int Enable);
	void SetTrigger(int SrcIndex);
	
	reg_uint FifoEnable;				// this is only a 1bit wire, = FifoEnableFromTe & (~ FifoWaitTrigger)
	reg_uint FifoEnableFromTe;			// this is only a 1bit wire from outside
	reg_uint FifoWaitTrigger;			// 1bit
	reg_uint TriggerSource;				// 8bit
	reg_uint DummyWrite;				// 1bit
	reg_uint OverflowFlag;				// 2bit, bit0 for overflow, bit1 for guard
	reg_uint FifoGuard;					// 16bit
	reg_uint ReadAddress;				// 16bit
	reg_uint WriteAddress;				// 16bit
	reg_uint WriteAddressRound;			// 8bit
	reg_uint CurReadAddress;			// 16bit
	reg_uint BlockSize;					// 16bit
	reg_int  BlockSizeAdjust;			// 8bit
	reg_uint WriteAddressLatchCPU;		// 16bit
	reg_uint WriteAddressLatchEM;		// 16bit
	reg_uint WriteAddressLatchPPS;		// 16bit
	reg_uint WriteAddressLatchAE;		// 16bit
	reg_uint WriteAddressLatchCPURound;	// 8bit
	reg_uint WriteAddressLatchEMRound;	// 8bit
	reg_uint WriteAddressLatchPPSRound;	// 8bit
	reg_uint WriteAddressLatchAERound;	// 8bit
	
	int RealBlockSize;					// this is a wire instead of register, the value equals to BlockSize + BlockSizeAdjust
	int GuardThreshold;					// this is a wire instead of register, the value equals to FifoSize - FifoGuard

	complex_int *pBuffer;
	int DataCount;						// 17bit, count of sample in FIFO
	unsigned int FifoSize;

	TriggerFunction TriggerCallback;
	int FifoIndex;
};

#endif //__TE_FIFO_MEM_H__
