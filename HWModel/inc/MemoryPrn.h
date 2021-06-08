//----------------------------------------------------------------------
// MemoryPrn.h:
//   Memory code PRN generator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __MEMORY_PRN_H__
#define __MEMORY_PRN_H__

#include "CommonOps.h"
#include "PrnGen.h"

class CMemoryPrn : public CPrnGen
{
public:
	CMemoryPrn(unsigned int *Address);
	~CMemoryPrn();

	reg_uint StartIndex;			// 12bit RO
	reg_uint Length;				// 4bit RO
	reg_uint CurrentCode;			// 32bit RW
	reg_uint CurrentCount;			// 14bit RW

	void FillState(unsigned int *StateBuffer);
	void DumpState(unsigned int *StateBuffer);
	int GetCode();
	int ShiftCode();
	void PhaseInit(unsigned int PrnConfig);
	void Reset();

	unsigned int *CodeMemory;		// Base address of memory code RAM, not needed in RTL
};

#endif // __MEMORY_PRN_H__
