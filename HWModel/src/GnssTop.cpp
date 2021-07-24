#include <stdio.h>
#include <malloc.h>
#include <memory.h>
#include "CommonOps.h"
#include "RegAddress.h"
#include "GnssTop.h"
#include "E1_code.h"

CGnssTop::CGnssTop() : TeFifo(0, 10240), TrackingEngine(&TeFifo, MemCodeBuffer), AcqEngine(MemCodeBuffer)
{
	TrackingEngineEnable = 0;
	MeasurementNumber = 0;
	MeasurementCount = 0;
	ReqCount = 0;
	InterruptFlag = 0;

	memcpy(MemCodeBuffer, GalE1Code, sizeof(GalE1Code));	// memory code put in MemCodeBuffer as ROM
	FileData = (complex_int *)malloc(MAX_BLOCK_SIZE * sizeof(complex_int));
	SampleQuant = (unsigned char *)malloc(MAX_BLOCK_SIZE * sizeof(unsigned char));

	InterruptService = NULL;
}

CGnssTop::~CGnssTop()
{
	free(FileData);
	free(SampleQuant);
}

void CGnssTop::Reset(U32 ResetMask)
{
	if (ResetMask & 2)
		TrackingEngine.Reset();
	if (ResetMask & 0x100)
		TeFifo.Reset();
}

void CGnssTop::Clear(U32 ClearMask)
{
	if (ClearMask & 0x100)
		TeFifo.Clear();
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
			{
				TrackingEngineEnable = 1;
				TeFifo.SetFifoEnable(Value >> 8);
			}
			break;
		case ADDR_OFFSET_BB_RESET:
			Reset(Value);
			break;
		case ADDR_OFFSET_FIFO_CLEAR:
			Clear(Value);
			break;
		case ADDR_OFFSET_TRACKING_START:
			break;	// has no effect in C model
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
//	case ADDR_BASE_IF_INTERFACE:
//		IfInterface.SetRegValue(AddressOffset & 0xff, Value);
//		break;
//	case ADDR_BASE_PRE_PROCESS:
//		NoiseCalculate[TEindex].SetRegValue(AddressOffset & 0xff, Value);
//		break;
//	case ADDR_BASE_AE_FIFO:
//		AeFifo.SetRegValue(AddressOffset, Value);
//		break;
	case ADDR_BASE_ACQUIRE_ENGINE:
		AcqEngine.SetRegValue(AddressOffset, Value);
		if (AddressOffset == ADDR_OFFSET_AE_BUFFER_CONTROL && (Value & 0x100))	// latch write address when starting fill AE buffer
			TeFifo.LatchWriteAddress(3);
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
		TrackingEngine.TEBuffer[AddressOffset >> 2] = Value;
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
//	case ADDR_BASE_IF_INTERFACE:
//		return IfInterface.GetRegValue(AddressOffset & 0xff);
//	case ADDR_BASE_PRE_PROCESS:
//		return NoiseCalculate[TEindex].GetRegValue(AddressOffset & 0xff);
//	case ADDR_BASE_AE_FIFO:
//		return AeFifo.GetRegValue(AddressOffset);
	case ADDR_BASE_ACQUIRE_ENGINE:
		return AcqEngine.GetRegValue(AddressOffset);
	case ADDR_BASE_TE_FIFO:
		return TeFifo.GetRegValue(AddressOffset);
	case ADDR_BASE_TRACKING_ENGINE:
		return TrackingEngine.GetRegValue(AddressOffset & 0xff);
	case ADDR_BASE_PERIPHERIAL:
		return 0;
	case ADDR_BASE_TE_BUFFER:
		return TrackingEngine.TEBuffer[AddressOffset >> 2];
		break;
	case ADDR_BASE_AE_BUFFER:
		return AcqEngine.ChannelConfig[AddressOffset >> 5][(AddressOffset >> 2) & 0x7];
		break;
	default:
		return 0;
	}
}

int CGnssTop::Process(int ReadBlockSize)
{
	int i;
	int ReachThreshold = 0;
	int SampleNumber;

	if (!IfFile.ReadFile(ReadBlockSize, FileData))
		return -1;
	if (AcqEngine.IsFillingBuffer())
	{
		SampleNumber = AcqEngine.RateAdaptor.DoRateAdaptor(FileData, ReadBlockSize, SampleQuant);
		AcqEngine.WriteSample(SampleNumber, SampleQuant);
	}
	for (i = 0; i < ReadBlockSize; i ++)
		ReachThreshold |= TeFifo.WriteData(FileData[i]);

	if (TrackingEngineEnable)
	{
		InterruptFlag |= TrackingEngine.ProcessData() ? 0x100 : 0;
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

	return 0;
}
