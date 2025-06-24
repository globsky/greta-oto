//----------------------------------------------------------------------
// TeFifoSim.h:
//   TE FIFO simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TE_FIFO_SIM_H__
#define __TE_FIFO_SIM_H__

#include "CommonDefines.h"

#define FIFO_SIZE 10240
#define ADDR_WIDTH 14
#define CLK_COUNT_WIDTH (20 - ADDR_WIDTH)

class CTeFifoSim
{
public:
	CTeFifoSim();
	~CTeFifoSim();
	void Reset();
	void Clear();
	void SetRegValue(int Address, U32 Value);
	U32 GetRegValue(int Address);
	void StepOneBlock(int BlockSize);
	void LatchWriteAddress(int Source);
	
	unsigned int FifoGuard;					// 16bit
	unsigned int ReadAddress;				// 14bit
	unsigned int WriteAddress;				// 14bit
	unsigned int WriteAddressRound;			// 12bit
	unsigned int BlockSize;					// 14bit
	unsigned int WriteAddressLatchCPU;		// 14bit
	unsigned int WriteAddressLatchEM;		// 14bit
	unsigned int WriteAddressLatchPPS;		// 14bit
	unsigned int WriteAddressLatchAE;		// 14bit
	unsigned int WriteAddressLatchCPURound;	// 12bit
	unsigned int WriteAddressLatchEMRound;	// 12bit
	unsigned int WriteAddressLatchPPSRound;	// 12bit
	unsigned int WriteAddressLatchAERound;	// 12bit
};

#endif //__TE_FIFO_SIM_H__
