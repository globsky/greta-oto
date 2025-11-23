//----------------------------------------------------------------------
// FirmwarePortal.c:
//   Firmware portal functions and support functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonDefines.h"
#include "RegAddress.h"
#include "BBDefines.h"
#include "HWCtrl.h"
#include "PlatformCtrl.h"
#include "TaskManager.h"
#include "AEManager.h"
#include "TEManager.h"
#include "ChannelManager.h"
#include "PvtEntry.h"
#include "SupportPackage.h"
#include "GlobalVar.h"
#include "SystemConfig.h"

//*************** Baseband interrupt service routine ****************
//* this function is a ISR function
// Parameters:
//   none
// Return value:
//   none
void InterruptService()
{
	unsigned int IntFlag = GetRegValue(ADDR_INTERRUPT_FLAG);
	unsigned int MeasCount = GetRegValue(ADDR_MEAS_COUNT);
	
	if (IntFlag & (1 << 9))
		MeasIntCounter++;
	BasebandTickCount = GetRegValue(ADDR_TICK_COUNT);

	if (IntFlag & (1 << 8))		// coherent sum interrupt
		CohSumInterruptProc();
	if (IntFlag & (1 << 9))		// measurement interrupt
		MeasurementProc();
	if (IntFlag & (1 << 10))	// request interrupt
		DoRequestTask();
	if (IntFlag & (1 << 11))	// AE interrupt
		AeInterruptProc();
	// clear interrupt
	SetRegValue(ADDR_INTERRUPT_FLAG, IntFlag);
	// if it is TE interrupt, resume TE
	if (IntFlag & 0x700)
		SetRegValue(ADDR_TRACKING_START, 1);
}

//*************** firmware initialization ****************
// Parameters:
//   Start: receiver start type ColdStart/WarmStart/HotStart
//   CurTime: pointer to SYSTEM_TIME get from RTC or NULL if no valid time
//   CurPosition: pointer to LLH receiver position if force set else NULL
// Return value:
//   none
void FirmwareInitialize(StartType Start, PSYSTEM_TIME CurTime, LLH *CurPosition)
{
	int i, sv_list[32] = {
#if 1
		FREQ_SVID(FREQ_L1CA, 3),
//		FREQ_SVID(FREQ_L1CA, 4),
		FREQ_SVID(FREQ_L1CA, 7),
		FREQ_SVID(FREQ_L1CA, 8),
		FREQ_SVID(FREQ_L1CA, 9),
//		FREQ_SVID(FREQ_L1CA, 14),
		FREQ_SVID(FREQ_L1CA, 16),
		FREQ_SVID(FREQ_L1CA, 27),
		FREQ_SVID(FREQ_L1CA, 30),
#endif
#if 1
		FREQ_SVID(FREQ_B1C, 8),
		FREQ_SVID(FREQ_B1C, 19),
		FREQ_SVID(FREQ_B1C, 21),
		FREQ_SVID(FREQ_B1C, 22),
		FREQ_SVID(FREQ_B1C, 26),
#endif
#if 1
		FREQ_SVID(FREQ_E1, 1),
		FREQ_SVID(FREQ_E1, 4),
		FREQ_SVID(FREQ_E1, 19),
		FREQ_SVID(FREQ_E1, 21),
		FREQ_SVID(FREQ_E1, 27),
#endif
	0 };	// for debug use only
	int SatNumber;
	SAT_PREDICT_PARAM SatList[32];
	PACQ_CONFIG pAcqConfig = NULL;

	NominalMeasInterval = DEFAULT_MEAS_INTERVAL;

	AttachBasebandISR(InterruptService);
	LoadAllParameters();

	// initial baseband hardware
	SetRegValue(ADDR_BB_ENABLE, 0x00000100);		// enable TE
	SetRegValue(ADDR_FIFO_CLEAR, 0x00000100);		// clear FIFO
	SetRegValue(ADDR_MEAS_NUMBER, NominalMeasInterval);	// measurement number
	SetRegValue(ADDR_MEAS_COUNT, 0);				// measurement count
//	SetRegValue(ADDR_REQUEST_COUNT, 8);				// request count
	SetRegValue(ADDR_INTERRUPT_MASK, 0x00000f00);	// enable all interrupts
	SetRegValue(ADDR_TE_FIFO_CONFIG, 1);			// FIFO config, enable dummy write
	SetRegValue(ADDR_TE_FIFO_BLOCK_SIZE, 4113);		// FIFO block size
	SetRegValue(ADDR_TE_CHANNEL_ENABLE, 0);			// disable all channels
	SetRegValue(ADDR_TE_POLYNOMIAL, 0x00e98204);	// set L1CA polynomial
	SetRegValue(ADDR_TE_CODE_LENGTH, 0x00ffc000);	// set L1CA code length
	SetRegValue(ADDR_TE_NOISE_CONFIG, 1);			// set noise smooth factor
	SetRegValue(ADDR_TE_NOISE_FLOOR, 784 >> PRE_SHIFT_BITS);	// set initial noise floor

	// initialize firmware modules
	InitStreamPorts();
	TaskInitialize();
	TEInitialize();
	AEInitialize();
	MsrProcInit();
	PvtProcInit(Start, CurTime, CurPosition);
	if (Start != ColdStart)
	{
/*		if (Start == HotStart)
		{
			UtcToGpsTime(CurTime, &(g_ReceiverInfo.WeekNumber), &(g_ReceiverInfo.GpsMsCount), &g_GpsUtcParam);
			g_ReceiverInfo.GpsTimeQuality = g_ReceiverInfo.BdsTimeQuality = g_ReceiverInfo.GalileoTimeQuality = ExtSetTime;
		}
		else
			g_ReceiverInfo.GpsTimeQuality = g_ReceiverInfo.BdsTimeQuality = g_ReceiverInfo.GalileoTimeQuality = UnknownTime;*/
//		SatNumber = GetSatelliteInView(SatList);
//		for (i = 0; i < SatNumber; i ++)
//			sv_list[i] = FREQ_SVID(SatList[i].FreqID, SatList[i].Svid);
//		sv_list[i] = 0;
	}

	// start acquisition
	// first task search L1C/A signal
	SatNumber = 0;
	if ((pAcqConfig = GetFreeAcqTask()) != NULL)
	{
		for (i = 0; i < 32; i ++)
		{
			if (sv_list[i] == 0)
				break;
			if (GET_FREQ_ID(sv_list[i]) != FREQ_L1CA)
				continue;
			pAcqConfig->SatConfig[SatNumber].FreqSvid = (U8)sv_list[i];
			pAcqConfig->SatConfig[SatNumber].CodeSpan = 3;
			pAcqConfig->SatConfig[SatNumber].CenterFreq = (Start == ColdStart) ? 0 : (int)SatList[i].Doppler;
			SatNumber ++;
		}
		pAcqConfig->SignalType = 0;	// for BPSK acquisition
		pAcqConfig->AcqChNumber = SatNumber;
		pAcqConfig->CohNumber = 4;
		pAcqConfig->NoncohNumber = 1;
		pAcqConfig->StrideNumber = (Start == ColdStart) ? 19 : (Start == WarmStart) ? 3 : 1;
		pAcqConfig->StrideInterval = 500;
		if (pAcqConfig->AcqChNumber > 0)
			AddAcqTask(pAcqConfig);
	}
	// second task search BOC signal
	if (SatNumber != 0)	// first AE task added, get a new task
		pAcqConfig = GetFreeAcqTask();
	SatNumber = 0;
	if (pAcqConfig != NULL)
	{
		for (i = 0; i < 32; i ++)
		{
			if (sv_list[i] == 0)
				break;
			if (GET_FREQ_ID(sv_list[i]) == FREQ_L1CA)
				continue;
			pAcqConfig->SatConfig[SatNumber].FreqSvid = (U8)sv_list[i];
			pAcqConfig->SatConfig[SatNumber].CodeSpan = (GET_FREQ_ID(sv_list[i]) == FREQ_E1) ? 12 : 30;
			pAcqConfig->SatConfig[SatNumber].CenterFreq = (Start == ColdStart) ? 0 : (int)SatList[i].Doppler;
			SatNumber ++;
		}
		pAcqConfig->SignalType = 1;	// for BOC acquisition
		pAcqConfig->AcqChNumber = SatNumber;
		pAcqConfig->CohNumber = 4;
		pAcqConfig->NoncohNumber = 2;
		pAcqConfig->StrideNumber = (Start == ColdStart) ? 19 : (Start == WarmStart) ? 3 : 1;
		pAcqConfig->StrideInterval = 500;
		if (pAcqConfig->AcqChNumber > 0)
			AddAcqTask(pAcqConfig);
	}
}
