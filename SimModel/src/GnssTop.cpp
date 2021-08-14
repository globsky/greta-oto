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

CGnssTop::CGnssTop()
{
	InterruptService = (InterruptFunction)0;
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
			AcqEngine.SetBufferParam(GpsSatParam, GpsSatNumber, CurTime, &GpsBits);	// set AE buffer parameters
			TeFifo.LatchWriteAddress(3);	// set AE latch time
		}
		if (AddressOffset == ADDR_OFFSET_AE_CONTROL && (Value & 0x100))	// if do acquisition, set AE finished interrupt
			InterruptFlag |= (1 << 11);
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
			TrackingEngine.InitChannel((Address >> 7) & 0x1f, CurTime, GpsSatParam, GpsSatNumber);
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
	LLA_POSITION StartPos;
	int i = 0;
	CXmlElementTree XmlTree;
	CXmlElement *RootElement, *Element;

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
	}
	Trajectory.ResetTrajectoryTime();
	CurTime = UtcToGpsTime(UtcTime);
	CurPos = LlaToEcef(StartPos);
	SpeedLocalToEcef(StartPos, StartVel, CurPos);
	// Find ephemeris match current time and fill in data to generate bit stream
	for (i = 1; i <= 32; i ++)
	{
		GpsEph[i-1] = NavData.FindEphemeris(CNavData::SystemGps, CurTime, i);
		GpsBits.SetEphemeris(i, GpsEph[i-1]);
	}
	GpsBits.SetIonoUtc(NavData.GetGpsIono(), NavData.GetGpsUtcParam());
	// calculate visible satellite at start time and calculate satellite parameters
	GpsSatNumber = GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, 32, GpsEphVisible);
	for (i = 0; i < GpsSatNumber; i ++)
	{
		GpsSatParam[i] = GetSatelliteParam(CurPos, StartPos, CurTime, GpsSystem, GpsEphVisible[i], NavData.GetGpsIono());
	}
	TrackingEngine.SetNavBit(&GpsBits);
}

int CGnssTop::Process(int ReadBlockSize)
{
	int ScenarioFinish = StepToNextTime(1);

	TeFifo.StepOneBlock();	// to have TE FIFO progress 1 block of data (1ms)
	if (TrackingEngineEnable)
	{
		InterruptFlag |= TrackingEngine.ProcessData(CurTime, GpsSatParam, GpsSatNumber) ? 0x100 : 0;
		MeasurementCount ++;
		if (MeasurementCount == MeasurementNumber)
		{
			MeasurementCount = 0;
			InterruptFlag |= (1 << 9);
		}
	}
	if(ReqCount)
	{
		if(ReqCount == 1)
			InterruptFlag |= (1 << 10);
		ReqCount--;
	}

	if ((InterruptFlag & IntMask) && InterruptService != NULL )
		InterruptService();

	// proceed to next millisecond and recalculate satellite paramter
	return ScenarioFinish;
}

int CGnssTop::StepToNextTime(int TimeInterval)
{
	int i;
	LLA_POSITION PosLLA;

	if (!Trajectory.GetNextPosVelECEF(TimeInterval / 1000., CurPos))
		return -1;

	// calculate new satellite parameter
	PosLLA = EcefToLla(CurPos);
	CurTime.MilliSeconds += TimeInterval;
	if (CurTime.MilliSeconds > 604800000)
	{
		CurTime.Week ++;
		CurTime.MilliSeconds -= 604800000;
	}
	if ((CurTime.MilliSeconds % 60000) == 0)	// recalculate visible satellite at minute boundary
		GpsSatNumber = GetVisibleSatellite(CurPos, CurTime, OutputParam, GpsSystem, GpsEph, 32, GpsEphVisible);
	for (i = 0; i < GpsSatNumber; i ++)
	{
		GpsSatParam[i] = GetSatelliteParam(CurPos, PosLLA, CurTime, GpsSystem, GpsEphVisible[i], NavData.GetGpsIono());
	}
	return 0;
}
