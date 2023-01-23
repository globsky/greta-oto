//----------------------------------------------------------------------
// TrackingEngine.cpp:
//   Tracking engine simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <memory.h>
#include <math.h>
#include "ConstVal.h"
#include "RegAddress.h"
#include "InitSet.h"
#include "ComplexNumber.h"
#include "GaussNoise.h"
#include "TrackingEngine.h"

#define COH_OFFSET(ch_index, cor_index) ((ch_index << 5) + 24 + (cor_index >> 2))

CTrackingEngine::CTrackingEngine()
{
	Reset();
	memset(TEBuffer, 0, TE_BUFFER_SIZE);
	memset(LogicChannel, 0, sizeof(LogicChannel));
}

CTrackingEngine::~CTrackingEngine()
{
}

void CTrackingEngine::Reset()
{
	ChannelEnable = CohDataReady = 0;
	OverwriteProtectAddr = 0;
	OverwriteProtectValue = 0;
	SmoothScale = 0;
	NoiseFloor = 784.0;
}

void CTrackingEngine::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;
	switch (Address)
	{
	case ADDR_OFFSET_TE_CHANNEL_ENABLE:
		ChannelEnable = Value;
		break;
	case ADDR_OFFSET_TE_COH_DATA_READY:
		CohDataReady = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_CHANNEL:
		OverwriteProtectChannel = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_ADDR:
		OverwriteProtectAddr = Value;
		break;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_VALUE:
		OverwriteProtectValue = Value;
		break;
	case ADDR_OFFSET_TE_POLYNOMIAL:
		PrnPolyLength[0] = Value;
		break;
	case ADDR_OFFSET_TE_CODE_LENGTH:
		PrnPolyLength[1] = Value;
		break;
	case ADDR_OFFSET_TE_POLYNOMIAL2:
		PrnPolyLength[2] = Value;
		break;
	case ADDR_OFFSET_TE_CODE_LENGTH2:
		PrnPolyLength[3] = Value;
		break;
	case ADDR_OFFSET_TE_NOISE_CONFIG:
		SmoothScale = Value & 3;
		break;
	case ADDR_OFFSET_TE_NOISE_FLOOR:
		NoiseFloor = (double)Value;
		break;
	default:
		break;
	}
}

U32 CTrackingEngine::GetRegValue(int Address)
{
	Address &= 0xff;
	switch (Address)
	{
	case ADDR_OFFSET_TE_CHANNEL_ENABLE:
		return ChannelEnable;
	case ADDR_OFFSET_TE_COH_DATA_READY:
		return CohDataReady;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_CHANNEL:
		return OverwriteProtectChannel;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_ADDR:
		return OverwriteProtectAddr;
	case ADDR_OFFSET_TE_OVERWRITE_PROTECT_VALUE:
		return OverwriteProtectValue;
	case ADDR_OFFSET_TE_POLYNOMIAL:
		return PrnPolyLength[0];
	case ADDR_OFFSET_TE_CODE_LENGTH:
		return PrnPolyLength[1];
	case ADDR_OFFSET_TE_POLYNOMIAL2:
		return PrnPolyLength[2];
	case ADDR_OFFSET_TE_CODE_LENGTH2:
		return PrnPolyLength[3];
	case ADDR_OFFSET_TE_NOISE_CONFIG:
		return SmoothScale;
	case ADDR_OFFSET_TE_NOISE_FLOOR:
		return (int)NoiseFloor;
	default:
		return 0;
	}
}

// set value of TE config buffer
// interprete channel configuration parameters as well
void CTrackingEngine::SetTEBuffer(unsigned int Address, U32 Value)
{
	int ChannelNumber = (Address >> 5) & 0x1f;
	unsigned int AddressOffset = Address & 0x1f;	// offset within channel

	TEBuffer[Address & 0xfff]= Value;
	if (AddressOffset < STATE_OFFSET_PARTIAL_ACC)	// config and state fields
		LogicChannel[ChannelNumber].SetChannelStates(AddressOffset, Value);
}

U32 CTrackingEngine::GetTEBuffer(unsigned int Address)
{
	int ChannelNumber = (Address >> 5) & 0x1f;
	unsigned int AddressOffset = Address & 0x1f;	// offset within channel

	if (AddressOffset >= STATE_OFFSET_PRN_COUNT && AddressOffset <= STATE_OFFSET_DECODE_DATA)
		return LogicChannel[ChannelNumber].GetChannelStates(AddressOffset);
	else
		return TEBuffer[Address & 0xfff];
}

// process one system
// input parameter specify the system index
// return 1 if any correlator in any channel has data ready
int CTrackingEngine::ProcessData(int BlockSize, GNSS_TIME CurTime, PSATELLITE_PARAM SatParam[], int SatNumber)
{
	unsigned int EnableMask;
	int i, j;
	SATELLITE_PARAM *pSatParam;
	int DumpDataI[16], DumpDataQ[16];
	int CorIndex[16], CorPos[16];
	int NHCode[16];
	int DataLength;
	unsigned int CohData;
	S16 CohDataI, CohDataQ;
	int DataValue;
	complex_number Noise;
	int ShiftBits = -1;

	// clear coherent data ready flag and overwrite protect flag
	CohDataReady = 0;
	OverwriteProtectChannel = 0;

	for (i = 0, EnableMask = 1; i < 32; i ++, EnableMask <<= 1)
	{
		if ((ChannelEnable & EnableMask) == 0)
			continue;
		if (ShiftBits < 0)
			ShiftBits = LogicChannel[i].PreShiftBits;

		// find whether there is visible satellite match current channel
		pSatParam = FindSatParam(i, SatParam, SatNumber);

		// recalculate corresponding counter of channel
		if (LogicChannel[i].CalculateCounter(BlockSize, CorIndex, CorPos, NHCode, DataLength))
			CohDataReady |= EnableMask;
		// calculate 1ms correlation result
		LogicChannel[i].GetCorrelationResult(CurTime, pSatParam, DumpDataI, DumpDataQ, CorIndex, CorPos, NHCode, DataLength);
		// do coherent sum
		for (j = 0; j < DataLength; j ++)
		{
			// if overwrite protect bit is set, set corresponding flag bit and set address and value. do NOT accumulate
			if (CorIndex[j] & 2)
			{
				OverwriteProtectChannel |= EnableMask;
				OverwriteProtectAddr = COH_OFFSET(i, CorIndex[j]) << 2;
				OverwriteProtectValue = ((DumpDataI[j] & 0xffff) << 16) | (DumpDataQ[j] & 0xffff);
				continue;
			}

			// if this is first epoch of coherent data accumulation, clear stored value
			if (CorIndex[j] & 1)
				CohData = 0;
			else
				CohData = TEBuffer[COH_OFFSET(i, CorIndex[j])];
			CohDataI = (S16)(CohData >> 16);
			CohDataQ = (S16)(CohData & 0xffff);
			CohDataI += (S16)DumpDataI[j];
			CohDataQ += (S16)DumpDataQ[j];
			CohData = ((unsigned int)CohDataI << 16) | ((unsigned int)CohDataQ & 0xffff);
			TEBuffer[COH_OFFSET(i, CorIndex[j])] = CohData;
			// do data decode
			if (LogicChannel[i].BitLength && LogicChannel[i].EnableSecondPrn && ((CorIndex[j] >> 2) == 0) && LogicChannel[i].BitCount == 0)
			{
				DataValue = LogicChannel[i].DataInQBranch ? CohDataQ : CohDataI;
				LogicChannel[i].DecodeDataAcc(DataValue);
			}
		}
		if (DataLength)
			LogicChannel[i].CurrentCor = ((CorIndex[DataLength-1] >> 2) + 1) & 0x7;	// CurrentCor is next to the last output correlator
	}

	// update noise floor
	if (ShiftBits >= 0)
	{
		Noise = GenerateNoise(NOISE_AMP);
		NoiseFloor += (Noise.abs() / pow(2.0, ShiftBits) - NoiseFloor) / ((double)(1 << (8 + SmoothScale * 2)));
	}
	return (CohDataReady != 0);
}

SATELLITE_PARAM* CTrackingEngine::FindSatParam(int ChannelId, PSATELLITE_PARAM SatParam[], int SatNumber)
{
	int i;
	SignalSystem SystemSel = LogicChannel[ChannelId].SystemSel;
	int Svid = LogicChannel[ChannelId].Svid, system;

	switch (SystemSel)
	{
	case SignalL1CA: system = GpsSystem; break;
	case SignalE1  : system = GalileoSystem; break;
	case SignalB1C : system = BdsSystem; break;
	case SignalL1C : system = GpsSystem; break;
	default:
		return (SATELLITE_PARAM *)0;
	}

	for (i = 0; i < SatNumber; i ++)
	{
		if (system == SatParam[i]->system && Svid == SatParam[i]->svid)
			return SatParam[i];
	}
	return (SATELLITE_PARAM *)0;
}
