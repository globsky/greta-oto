//----------------------------------------------------------------------
// PrnGen.h:
//   Declaration of virtual PRN generator base class
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __PRNGEN_H__
#define __PRNGEN_H__

#include "CommonOps.h"

class CPrnGen
{
public:
	CPrnGen();
	~CPrnGen();

	virtual void FillState(unsigned int *StateBuffer) = 0;
	virtual void DumpState(unsigned int *StateBuffer) = 0;
	virtual int GetCode() = 0;
	virtual int ShiftCode() = 0;
	virtual void PhaseInit(unsigned int PrnConfig) = 0;
	virtual void Reset() = 0;
};

#endif // __PRNGEN_H__
