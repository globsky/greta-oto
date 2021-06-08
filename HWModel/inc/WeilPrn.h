//----------------------------------------------------------------------
// WeilPrn.h:
//   Weil code PRN generator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __WEIL_PRN_H__
#define __WEIL_PRN_H__

#include "CommonOps.h"
#include "PrnGen.h"

#define B1C_LEGENDRE_LENGTH 10243
#define L1C_LEGENDRE_LENGTH 10223
#define LEGENDRE_LENGTH(type) ((type == 0) ? B1C_LEGENDRE_LENGTH : L1C_LEGENDRE_LENGTH)

class CWeilPrn : public CPrnGen
{
public:
	CWeilPrn();
	~CWeilPrn();

	reg_uint WeilType;						// 1bit RO
	reg_uint InsertionIndex, WeilIndex;		// 14bit RO
	reg_uint LegendreCode1, LegendreCode2;	// 14bit RW
	reg_uint CurrentPhase;					// 14bit RW

	void FillState(unsigned int *StateBuffer);
	void DumpState(unsigned int *StateBuffer);
	int GetCode();
	int ShiftCode();
	void PhaseInit(unsigned int PrnConfig);
	void Reset();
	int ResetCounter();

	int CodeIndex1, CodeIndex2;				// 14bit
	int InsertCount;						// 3bit
	int InsertCode;							// 1bit
	static const unsigned short LegendreL1C[640];
	static const unsigned short LegendreB1C[640];
};

#endif // __WEIL_PRN_H__
