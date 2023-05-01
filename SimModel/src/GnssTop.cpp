//----------------------------------------------------------------------
// GnssTop.cpp:
//   GNSS baseband top module simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include <memory.h>
#include <string.h>
#include "RegAddress.h"
#include "GnssTop.h"
#include "XmlElement.h"
#include "XmlInterpreter.h"
#include "Coordinate.h"

double CTrackingChannel::CovarMatrix[4][SUM_N(COR_NUMBER)];

CGnssTop::CGnssTop()
{
	InterruptService = (InterruptFunction)0;
	NavBitArray[0] = &GpsBits;	// for GPS L1C/A
	NavBitArray[1] = &GpsBits;	// for Galileo E1
	NavBitArray[2] = &BdsBits;	// for BDS B1C
	NavBitArray[3] = &GpsBits;	// for GPS L1C
	// calculate relative matrix for noise generation
	CalculateCovar(COR_NUMBER, 2, CTrackingChannel::CovarMatrix[0]);
	CalculateCovar(COR_NUMBER, 4, CTrackingChannel::CovarMatrix[1]);
	CalculateCovar(COR_NUMBER, 8, CTrackingChannel::CovarMatrix[2]);
	CalculateCovar(COR_NUMBER, 2, CTrackingChannel::CovarMatrix[3]);
}

CGnssTop::~CGnssTop()
{
}

void CGnssTop::Reset(U32 ResetMask)
{
//	if (ResetMask & 2)
//		TrackingEngine.Reset();
}

void CGnssTop::Clear(U32 ClearMask)
{
}

void CGnssTop::SetRegValue(int Address, U32 Value)
{
	int AddressOffset = Address & 0xfff;
	int ChannelNumber;
	SATELLITE_PARAM *pSatParam;

	switch (Address & 0xf000)
	{
	case ADDR_BASE_GLOBAL_REGS:
		switch (AddressOffset)
		{
		case ADDR_OFFSET_BB_ENABLE:
			if (Value & 0x100)
				TrackingEngineEnable = 1;
			break;
		case ADDR_OFFSET_BB_RESET:
			Reset(Value);
			break;
		case ADDR_OFFSET_FIFO_CLEAR:
			Clear(Value);
			break;
		case ADDR_OFFSET_TRACKING_START:
			break;	// has no effect in Sim model
		case ADDR_OFFSET_MEAS_NUMBER:
			MeasurementNumber = (Value & 0x3ff);
			break;
		case ADDR_OFFSET_MEAS_COUNT:
			MeasurementCount = (Value & 0x3ff);
			break;
		case ADDR_OFFSET_INTERRUPT_FLAG:
			InterruptFlag &= ~Value;
			break;
		case ADDR_OFFSET_REQUEST_COUNT:
			ReqCount = (Value & 0x3ff);
			break;
		case ADDR_OFFSET_INTERRUPT_MASK:
			IntMask = Value;
		default:
			break;
		}
		break;
	case ADDR_BASE_ACQUIRE_ENGINE:
		AcqEngine.SetRegValue(AddressOffset, Value);
		if (AddressOffset == ADDR_OFFSET_AE_BUFFER_CONTROL && (Value & 0x100))	// latch write address when starting fill AE buffer
		{
			AcqEngine.SetBufferParam(SatParamList, TotalSatNumber, CurTime, NavBitArray);	// set AE buffer parameters
			TeFifo.LatchWriteAddress(3);	// set AE latch time
		}
		if (AddressOffset == ADDR_OFFSET_AE_CONTROL && (Value & 0x100))	// if do acquisition, set AE finished interrupt
			AeProcessCount = GetAeProcessTime();
		break;
	case ADDR_BASE_TE_FIFO:
		TeFifo.SetRegValue(AddressOffset, Value);
		break;
	case ADDR_BASE_TRACKING_ENGINE:
		TrackingEngine.SetRegValue(AddressOffset & 0xff, Value);
		break;
	case ADDR_BASE_PERIPHERIAL:
		break;
	case ADDR_BASE_TE_BUFFER:
		TrackingEngine.SetTEBuffer(AddressOffset >> 2, Value);
		if (((AddressOffset >> 2) & 0x1f) == STATE_OFFSET_PRN_CONFIG)	// if set PRN_CONFIG, assume initial channel with new SVID
		{
			ChannelNumber = (Address >> 7) & 0x1f;
			if ((pSatParam = TrackingEngine.FindSatParam(ChannelNumber, SatParamList, TotalSatNumber)) != NULL)
				TrackingEngine.LogicChannel[ChannelNumber].Initial(CurTime, pSatParam, NavBitArray[TrackingEngine.LogicChannel[ChannelNumber].SystemSel]);
		}
		break;
	case ADDR_BASE_AE_BUFFER:
		AcqEngine.ChannelConfig[AddressOffset >> 5][(AddressOffset >> 2) & 0x7] = Value;
		break;
	default:
		break;
	}
}

U32 CGnssTop::GetRegValue(int Address)
{
	int AddressOffset = Address & 0xfff;

	switch (Address & 0xf000)
	{
	case ADDR_BASE_GLOBAL_REGS:
		switch (AddressOffset)
		{
		case ADDR_OFFSET_BB_ENABLE:
			return TrackingEngineEnable << 8;
		case ADDR_OFFSET_TRACKING_START:
			return 0;	// TE engine is always waiting in C model
		case ADDR_OFFSET_MEAS_NUMBER:
			return MeasurementNumber;
		case ADDR_OFFSET_MEAS_COUNT:
			return MeasurementCount;
		case ADDR_OFFSET_INTERRUPT_FLAG:
			return InterruptFlag;
		case ADDR_OFFSET_REQUEST_COUNT:
			return ReqCount;
		case ADDR_OFFSET_INTERRUPT_MASK:
			return IntMask;
		default:
			return 0;
		}
	case ADDR_BASE_ACQUIRE_ENGINE:
		return AcqEngine.GetRegValue(AddressOffset);
	case ADDR_BASE_TE_FIFO:
		return TeFifo.GetRegValue(AddressOffset);
	case ADDR_BASE_TRACKING_ENGINE:
		return TrackingEngine.GetRegValue(AddressOffset & 0xff);
	case ADDR_BASE_PERIPHERIAL:
		return 0;
	case ADDR_BASE_TE_BUFFER:
		return TrackingEngine.GetTEBuffer(AddressOffset >> 2);
	case ADDR_BASE_AE_BUFFER:
		return AcqEngine.ChannelConfig[AddressOffset >> 5][(AddressOffset >> 2) & 0x7];
	default:
		return 0;
	}
}

void CGnssTop::SetInputFile(char *FileName)
{
	int i = 0, index;
	CXmlElementTree XmlTree;
	CXmlElement *RootElement, *Element;
	int ListCount;
	PSIGNAL_POWER PowerList;

	XmlTree.parse(FileName);
	RootElement = XmlTree.getroot();

	while ((Element = RootElement->GetElement(i ++)) != NULL)
	{
		if (strcmp(Element->GetTag(), "Time") == 0)
			AssignStartTime(Element, UtcTime);
		else if (strcmp(Element->GetTag(), "Trajectory") == 0)
			SetTrajectory(Element, StartPos, StartVel, Trajectory);
		else if (strcmp(Element->GetTag(), "Ephemeris") == 0)
			NavData.ReadNavFile(Element->GetText());
		else if (strcmp(Element->GetTag(), "Output") == 0)
			SetOutputParam(Element, OutputParam);
		else if (strcmp(Element->GetTag(), "PowerControl") == 0)
			SetPowerControl(Element, PowerControl);
	}
	Trajectory.ResetTrajectoryTime();
	CurTime = UtcToGpsTime(UtcTime);
	CurPos = LlaToEcef(StartPos);
	SpeedLocalToEcef(StartPos, StartVel, CurPos);

	for (i = 0; i < TOTAL_GPS_SAT; i ++)
		GpsSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (i = 0; i < TOTAL_BDS_SAT; i ++)
		BdsSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);
	for (i = 0; i < TOTAL_GAL_SAT; i ++)
		GalSatParam[i].CN0 = (int)(PowerControl.InitCN0 * 100 + 0.5);

	// Find ephemeris match current time and fill in data to generate bit stream
	for (i = 1; i <= TOTAL_GPS_SAT; i ++)
	{
		GpsEph[i-1] = NavData.FindEphemeris(GpsSystem, CurTime, i);
		GpsBits.SetEphemeris(i, GpsEph[i-1]);
	}
	for (i = 1; i <= TOTAL_BDS_SAT; i ++)
	{
		BdsEph[i-1] = NavData.FindEphemeris(BdsSystem, CurTime, i);
		BdsBits.SetEphemeris(i, BdsEph[i-1]);
	}
	for (i = 1; i <= TOTAL_GAL_SAT; i ++)
		GalEph[i-1] = NavData.FindEphemeris(GalileoSystem, CurTime, i);
	GpsBits.SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	// calculate visible satellite at start time and calculate satellite parameters
	GpsSatNumber = (OutputParam.SystemSelect & (1 << GpsSystem)) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, 32, GpsEphVisible) : 0;
	BdsSatNumber = (OutputParam.SystemSelect & (1 << BdsSystem)) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, BdsSystem, BdsEph, TOTAL_BDS_SAT, BdsEphVisible) : 0;
	GalSatNumber = (OutputParam.SystemSelect & (1 << GalileoSystem)) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GalileoSystem, GalEph, TOTAL_GAL_SAT, GalEphVisible) : 0;
	TotalSatNumber = GpsSatNumber + BdsSatNumber + GalSatNumber;
	ListCount = PowerControl.GetPowerControlList(0, PowerList);
	TotalSatNumber = 0;
	for (i = 0; i < GpsSatNumber; i ++)
	{
		index = GpsEphVisible[i]->svid - 1;
		GetSatelliteParam(CurPos, StartPos, CurTime, GpsSystem, GpsEphVisible[i], NavData.GetGpsIono(), &GpsSatParam[index]);
		GetSatelliteCN0(ListCount, PowerList, PowerControl.InitCN0, PowerControl.Adjust, &GpsSatParam[index]);
		SatParamList[TotalSatNumber ++] = &GpsSatParam[index];
	}
	for (i = 0; i < BdsSatNumber; i ++)
	{
		index = BdsEphVisible[i]->svid - 1;
		GetSatelliteParam(CurPos, StartPos, CurTime, BdsSystem, BdsEphVisible[i], NavData.GetGpsIono(), &BdsSatParam[index]);
		GetSatelliteCN0(ListCount, PowerList, PowerControl.InitCN0, PowerControl.Adjust, &BdsSatParam[index]);
		SatParamList[TotalSatNumber ++] = &BdsSatParam[index];
	}
	for (i = 0; i < GalSatNumber; i ++)
	{
		index = GalEphVisible[i]->svid - 1;
		GetSatelliteParam(CurPos, StartPos, CurTime, GalileoSystem, GalEphVisible[i], NavData.GetGpsIono(), &GalSatParam[index]);
		GetSatelliteCN0(ListCount, PowerList, PowerControl.InitCN0, PowerControl.Adjust, &GalSatParam[index]);
		SatParamList[TotalSatNumber ++] = &GalSatParam[index];
	}
}

int CGnssTop::Process(int BlockSize)
{
	int ScenarioFinish = StepToNextTime();

	TeFifo.StepOneBlock(BlockSize);	// to have TE FIFO progress 1 block of data (1ms)
	if (TrackingEngineEnable)
	{
		InterruptFlag |= TrackingEngine.ProcessData(BlockSize, CurTime, SatParamList, TotalSatNumber) ? 0x100 : 0;
		MeasurementCount ++;
		if (MeasurementCount == MeasurementNumber)
		{
			MeasurementCount = 0;
			InterruptFlag |= (1 << 9);
		}
	}
	if (ReqCount)
	{
		if (--ReqCount == 0)
			InterruptFlag |= (1 << 10);
	}
	if (AeProcessCount)
	{
		if (--AeProcessCount == 0)
			InterruptFlag |= (1 << 11);
	}

	if ((InterruptFlag & IntMask) && InterruptService != NULL )
		InterruptService();

	// proceed to next millisecond and recalculate satellite paramter
	return ScenarioFinish;
}

int CGnssTop::StepToNextTime()
{
	int i, index;
	LLA_POSITION PosLLA;
	int ListCount;
	PSIGNAL_POWER PowerList;

	if (!Trajectory.GetNextPosVelECEF(0.001, CurPos))
		return -1;

	ListCount = PowerControl.GetPowerControlList(1, PowerList);
	// calculate new satellite parameter
	PosLLA = EcefToLla(CurPos);
	CurTime.MilliSeconds ++;
	if (CurTime.MilliSeconds > 604800000)
	{
		CurTime.Week ++;
		CurTime.MilliSeconds -= 604800000;
	}
	if ((CurTime.MilliSeconds % 60000) == 0)	// recalculate visible satellite at minute boundary
	{
		GpsSatNumber = (OutputParam.SystemSelect & (1 << GpsSystem)) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, 32, GpsEphVisible) : 0;
		BdsSatNumber = (OutputParam.SystemSelect & (1 << BdsSystem)) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, BdsSystem, BdsEph, TOTAL_BDS_SAT, BdsEphVisible) : 0;
		GalSatNumber = (OutputParam.SystemSelect & (1 << GalileoSystem)) ? GetVisibleSatellite(CurPos, CurTime, OutputParam, GalileoSystem, GalEph, TOTAL_GAL_SAT, GalEphVisible) : 0;
	}
	TotalSatNumber = 0;
	for (i = 0; i < GpsSatNumber; i ++)
	{
		index = GpsEphVisible[i]->svid - 1;
		GetSatelliteParam(CurPos, PosLLA, CurTime, GpsSystem, GpsEphVisible[i], NavData.GetGpsIono(), &GpsSatParam[index]);
		GetSatelliteCN0(ListCount, PowerList, PowerControl.InitCN0, PowerControl.Adjust, &GpsSatParam[index]);
		SatParamList[TotalSatNumber ++] = &GpsSatParam[index];
	}
	for (i = 0; i < BdsSatNumber; i ++)
	{
		index = BdsEphVisible[i]->svid - 1;
		GetSatelliteParam(CurPos, PosLLA, CurTime, BdsSystem, BdsEphVisible[i], NavData.GetGpsIono(), &BdsSatParam[index]);
		GetSatelliteCN0(ListCount, PowerList, PowerControl.InitCN0, PowerControl.Adjust, &BdsSatParam[index]);
		SatParamList[TotalSatNumber ++] = &BdsSatParam[index];
	}
	for (i = 0; i < GalSatNumber; i ++)
	{
		index = GalEphVisible[i]->svid - 1;
		GetSatelliteParam(CurPos, PosLLA, CurTime, GalileoSystem, GalEphVisible[i], NavData.GetGpsIono(), &GalSatParam[index]);
		GetSatelliteCN0(ListCount, PowerList, PowerControl.InitCN0, PowerControl.Adjust, &GalSatParam[index]);
		SatParamList[TotalSatNumber ++] = &GalSatParam[index];
	}
	return 0;
}

#define AE_CLK_FREQ_MHz 100		// clock frequency for AE module
#define BLOCK_LENGTH_US 1000	// length in us for each data block
#define CLK_NUMBER_IN_BLOCK (AE_CLK_FREQ_MHz * BLOCK_LENGTH_US)
// each channel need number of clock cycles to search is about:
//   S*R*(682+4096*C*N)
//   fill sample needs about 2048 clock cycles
//   match filter needs about 8192 clock cycles for each round
//   S is tride number
//   C is coherent number
//   N is non-coherent number
//   R is code span
// one last step of force output needs about 1364 clock cycles
int CGnssTop::GetAeProcessTime()
{
	int i;
	int TotalCycles = 2;
	unsigned int (*ChannelConfig)[CHANNEL_CONFIG_LEN] = AcqEngine.ChannelConfig;
	int StrideNumber, CoherentNumber, NonCoherentNumber, CodeSpan;
	double ProcessTime;

	for (i = 0; i < (int)AcqEngine.ChannelNumber; i ++)
	{
		// fill in config registers
		StrideNumber = (int)EXTRACT_UINT(ChannelConfig[i][0], 0, 6);
		CoherentNumber = (int)EXTRACT_UINT(ChannelConfig[i][0], 8, 6);
		NonCoherentNumber = (int)EXTRACT_UINT(ChannelConfig[i][0], 16, 7);
		CodeSpan = (int)EXTRACT_UINT(ChannelConfig[i][2], 0, 5);
		TotalCycles += StrideNumber * CodeSpan * (1 + 6 * CoherentNumber * NonCoherentNumber);
	}

	ProcessTime = 682. * TotalCycles / CLK_NUMBER_IN_BLOCK;
	return (int)(ProcessTime + 1);	// round up
}
