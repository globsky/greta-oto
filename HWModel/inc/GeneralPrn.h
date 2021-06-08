//----------------------------------------------------------------------
// GeneralPrn.h:
//   General purpose PRN class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __GENERAL_PRN_H__
#define __GENERAL_PRN_H__

#include "CommonOps.h"
#include "PrnGen.h"

class CGeneralPrn : public CPrnGen
{
public:
	CGeneralPrn(const unsigned int PolySettings[]);
	~CGeneralPrn();

	reg_uint G1InitState, G2InitState;		// 14bit RO
	reg_uint G1GenPoly, G2GenPoly;			// 14bit RO
	reg_uint G1CurState, G2CurState;		// 14bit RW
	reg_uint G1Length;						// 14bit RO
	reg_uint G1CurCount;					// 14bit RW
	reg_uint GlobalLength;					// 32bit RO
	reg_uint GlobalCount;					// 32bit RW
	reg_uint ParallelSerial;				// 1bit RO

	const unsigned int *PrnPolySettings;			// 2 32bit registers for polynomial and length

	void FillState(unsigned int *StateBuffer);
	void DumpState(unsigned int *StateBuffer);
	int GetCode();
	int ShiftCode();
	void PhaseInit(unsigned int PrnConfig);
	void Reset();
};

#endif // __GENERAL_PRN_H__
