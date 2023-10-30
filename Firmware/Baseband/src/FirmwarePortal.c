//----------------------------------------------------------------------
// FirmwarePortal.c:
//   Firmware portal functions and support functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <memory.h>
#include <stdio.h>
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

#define PARAM_OFFSET_CONFIG		1024*0
#define PARAM_OFFSET_RCVRINFO	1024*1
#define PARAM_OFFSET_IONOUTC	1024*2
#define PARAM_OFFSET_GPSALM		1024*4
#define PARAM_OFFSET_BDSALM		1024*8
#define PARAM_OFFSET_GALALM		1024*16
#define PARAM_OFFSET_GPSEPH		1024*24
#define PARAM_OFFSET_BDSEPH		1024*32
#define PARAM_OFFSET_GALEPH		1024*48

void LoadAllParameters();

//*************** Baseband interrupt service routine ****************
//* this function is a task function
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
	BasebandTickCount = MeasIntCounter * MeasurementInterval + MeasCount;

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
//* this function is a task function
// Parameters:
//   none
// Return value:
//   none
void FirmwareInitialize(StartType Start, PSYSTEM_TIME CurTime, LLH *CurPosition)
{
	int i, sv_list[32] = {
		FREQ_SVID(FREQ_L1CA, 3),
//		FREQ_SVID(FREQ_L1CA, 4),
		FREQ_SVID(FREQ_L1CA, 7),
		FREQ_SVID(FREQ_L1CA, 8),
		FREQ_SVID(FREQ_L1CA, 9),
//		FREQ_SVID(FREQ_L1CA, 14),
		FREQ_SVID(FREQ_L1CA, 16),
		FREQ_SVID(FREQ_L1CA, 27),
		FREQ_SVID(FREQ_L1CA, 30),
		FREQ_SVID(FREQ_B1C, 8),
		FREQ_SVID(FREQ_B1C, 19),
		FREQ_SVID(FREQ_B1C, 21),
		FREQ_SVID(FREQ_B1C, 22),
		FREQ_SVID(FREQ_B1C, 26),
/*		FREQ_SVID(FREQ_E1, 1),
		FREQ_SVID(FREQ_E1, 4),
		FREQ_SVID(FREQ_E1, 19),
		FREQ_SVID(FREQ_E1, 21),
		FREQ_SVID(FREQ_E1, 27),
		FREQ_SVID(FREQ_E1, 27),*/
	0 };	// for debug use only
	int SatNumber;
	SAT_PREDICT_PARAM SatList[32];
	PACQ_CONFIG pAcqConfig;

	fp_debug = stdout;

	MeasurementInterval = 100;

	AttachBasebandISR(InterruptService);

	// initial baseband hardware
	SetRegValue(ADDR_BB_ENABLE, 0x00000100);		// enable TE
	SetRegValue(ADDR_FIFO_CLEAR, 0x00000100);		// clear FIFO
	SetRegValue(ADDR_MEAS_NUMBER, MeasurementInterval);	// measurement number
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
	TaskInitialize();
	TEInitialize();
	AEInitialize();
	BdsDecodeInit();
	MsrProcInit();
	PvtProcInit((PRECEIVER_INFO)0);
	if (Start != ColdStart)
	{
		LoadAllParameters();
		if (Start == HotStart)
		{
			UtcToGpsTime(CurTime, &(g_ReceiverInfo.WeekNumber), &(g_ReceiverInfo.GpsMsCount), &g_GpsUtcParam);
			g_ReceiverInfo.GpsTimeQuality = g_ReceiverInfo.BdsTimeQuality = g_ReceiverInfo.GalileoTimeQuality = ExtSetTime;
		}
		else
			g_ReceiverInfo.GpsTimeQuality = g_ReceiverInfo.BdsTimeQuality = g_ReceiverInfo.GalileoTimeQuality = UnknownTime;
		g_ReceiverInfo.PosQuality = ExtSetPos;
		g_ReceiverInfo.PosVel.vx = g_ReceiverInfo.PosVel.vy = g_ReceiverInfo.PosVel.vz = 0.0;
		SatNumber = GetSatelliteInView(SatList);
		for (i = 0; i < SatNumber; i ++)
			sv_list[i] = FREQ_SVID(SatList[i].FreqID, SatList[i].Svid);
		sv_list[i] = 0;
	}
	g_PvtConfig.PvtConfigFlags |= PVT_CONFIG_USE_KF;

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
			switch (GET_FREQ_ID(sv_list[i]))
			{
			case FREQ_L1CA:	pAcqConfig->SatConfig[SatNumber].CodeSpan = 3; break;
			case FREQ_E1:	pAcqConfig->SatConfig[SatNumber].CodeSpan = 12; break;
			case FREQ_B1C:
			case FREQ_L1C:	pAcqConfig->SatConfig[SatNumber].CodeSpan = 30; break;
			}
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
	SatNumber = 0;
	if ((pAcqConfig = GetFreeAcqTask()) != NULL)
	{
		for (i = 0; i < 32; i ++)
		{
			if (sv_list[i] == 0)
				break;
			if (GET_FREQ_ID(sv_list[i]) == FREQ_L1CA)
				continue;
			pAcqConfig->SatConfig[SatNumber].FreqSvid = (U8)sv_list[i];
			switch (GET_FREQ_ID(sv_list[i]))
			{
			case FREQ_L1CA:	pAcqConfig->SatConfig[SatNumber].CodeSpan = 3; break;
			case FREQ_E1:	pAcqConfig->SatConfig[SatNumber].CodeSpan = 12; break;
			case FREQ_B1C:
			case FREQ_L1C:	pAcqConfig->SatConfig[SatNumber].CodeSpan = 30; break;
			}
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

void LoadAllParameters()
{
	LoadParameters(PARAM_OFFSET_CONFIG, &g_PvtConfig, sizeof(g_PvtConfig));
	LoadParameters(PARAM_OFFSET_RCVRINFO, &g_ReceiverInfo, sizeof(g_ReceiverInfo));
	LoadParameters(PARAM_OFFSET_IONOUTC, &g_GpsIonoParam, sizeof(g_GpsIonoParam));
	LoadParameters(PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam), &g_BdsIonoParam, sizeof(g_BdsIonoParam));
	LoadParameters(PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam)+sizeof(g_BdsIonoParam), &g_GpsUtcParam, sizeof(g_GpsUtcParam));
	LoadParameters(PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam)+sizeof(g_BdsIonoParam)+sizeof(g_GpsUtcParam), &g_BdsUtcParam, sizeof(g_BdsUtcParam));
	LoadParameters(PARAM_OFFSET_GPSALM, &g_GpsAlmanac, sizeof(g_GpsAlmanac));
	LoadParameters(PARAM_OFFSET_BDSALM, &g_BdsAlmanac, sizeof(g_BdsAlmanac));
	LoadParameters(PARAM_OFFSET_GALALM, &g_GalileoAlmanac, sizeof(g_GalileoAlmanac));
	LoadParameters(PARAM_OFFSET_GPSEPH, &g_GpsEphemeris, sizeof(g_GpsEphemeris));
	LoadParameters(PARAM_OFFSET_BDSEPH, &g_BdsEphemeris, sizeof(g_BdsEphemeris));
	LoadParameters(PARAM_OFFSET_GALEPH, &g_GalileoEphemeris, sizeof(g_GalileoEphemeris));
}

void SaveAllParameters()
{
	SaveParameters(PARAM_OFFSET_CONFIG, &g_PvtConfig, sizeof(g_PvtConfig));
	SaveParameters(PARAM_OFFSET_RCVRINFO, &g_ReceiverInfo, sizeof(g_ReceiverInfo));
	SaveParameters(PARAM_OFFSET_IONOUTC, &g_GpsIonoParam, sizeof(g_GpsIonoParam));
	SaveParameters(PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam), &g_BdsIonoParam, sizeof(g_BdsIonoParam));
	SaveParameters(PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam)+sizeof(g_BdsIonoParam), &g_GpsUtcParam, sizeof(g_GpsUtcParam));
	SaveParameters(PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam)+sizeof(g_BdsIonoParam)+sizeof(g_GpsUtcParam), &g_BdsUtcParam, sizeof(g_BdsUtcParam));
	SaveParameters(PARAM_OFFSET_GPSALM, &g_GpsAlmanac, sizeof(g_GpsAlmanac));
	SaveParameters(PARAM_OFFSET_BDSALM, &g_BdsAlmanac, sizeof(g_BdsAlmanac));
	SaveParameters(PARAM_OFFSET_GALALM, &g_GalileoAlmanac, sizeof(g_GalileoAlmanac));
	SaveParameters(PARAM_OFFSET_GPSEPH, &g_GpsEphemeris, sizeof(g_GpsEphemeris));
	SaveParameters(PARAM_OFFSET_BDSEPH, &g_BdsEphemeris, sizeof(g_BdsEphemeris));
	SaveParameters(PARAM_OFFSET_GALEPH, &g_GalileoEphemeris, sizeof(g_GalileoEphemeris));
}
