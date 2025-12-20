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

static void DoAllSearch();
static void SearchSatelliteRange(U8 Signal, int StartSv, int SatNumber);
static void SearchSatelliteInView(int SatNumber, PSAT_PREDICT_PARAM SatList[], U8 SignalSvid[]);

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
	int SatNumber;
	PSAT_PREDICT_PARAM SatList[AE_CHANNEL_NUMBER];
	U8 SignalSvid[AE_CHANNEL_NUMBER];
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

	// start acquisition
	if (Start == ColdStart)
		DoAllSearch();
	else
	{
		UtcToGpsTime(CurTime, &(g_ReceiverInfo.ReceiverTime->GpsWeekNumber), &(g_ReceiverInfo.ReceiverTime->GpsMsCount), &g_GpsUtcParam);
		g_ReceiverInfo.ReceiverTime->BdsWeekNumber = g_ReceiverInfo.ReceiverTime->GpsWeekNumber - 1356;
		g_ReceiverInfo.ReceiverTime->BdsMsCount = g_ReceiverInfo.ReceiverTime->GpsMsCount - 14000;
		if (g_ReceiverInfo.ReceiverTime->GpsMsCount < 14000)	// BDS at previous week due to leap second difference
		{
			g_ReceiverInfo.ReceiverTime->BdsWeekNumber --;
			g_ReceiverInfo.ReceiverTime->BdsMsCount += MS_IN_WEEK;
		}
		g_ReceiverInfo.ReceiverTime->TimeQuality = ExtSetTime;
		SatNumber = GetSatelliteInView(SatList, SignalSvid);
		SearchSatelliteInView(SatNumber, SatList, SignalSvid);
//		for (i = 0; i < SatNumber; i ++)
//			sv_list[i] = FREQ_SVID(SatList[i].FreqID, SatList[i].Svid);
//		sv_list[i] = 0;
	}
}

//*************** add acquisition tasks for all satellites ****************
// Parameters:
//   none
// Return value:
//   none
void DoAllSearch()
{
	if (g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_GPS)
		SearchSatelliteRange(SIGNAL_L1CA, 1, 32);
	if (g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_GAL)
		SearchSatelliteRange(SIGNAL_E1, 1, 32);
	if (g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_BDS)
	{
		SearchSatelliteRange(SIGNAL_B1C, 1, 32);
		SearchSatelliteRange(SIGNAL_B1C, 33, 31);
	}
}

//*************** add an acquisition task for a given of satellites ****************
// Parameters:
//   Signal: signal to search
//   StartSv: first SVID of the search range
//   SatNumber: number of satellites to search
// Return value:
//   none
void SearchSatelliteRange(U8 Signal, int StartSv, int SatNumber)
{
	U8 CodeSpan = SIGNAL_IS_L1CA(Signal) ? 3 : SIGNAL_IS_E1(Signal) ? 12 : 30;
	int i;
	PACQ_CONFIG pAcqConfig;

	if ((pAcqConfig = GetFreeAcqTask()) != NULL)
	{
		for (i = 0; i < SatNumber; i ++)
		{
			pAcqConfig->SatConfig[i].SignalSvid = SIGNAL_SVID(Signal, (StartSv+i));
			pAcqConfig->SatConfig[i].CodeSpan = CodeSpan;
			pAcqConfig->SatConfig[i].CenterFreq = 0;
		}
		pAcqConfig->SearchMode = (SIGNAL_IS_L1CA(Signal) ? SEARCH_MODE_TYPE_BPSK : SEARCH_MODE_TYPE_BOC) | SEARCH_MODE_FREQ_FULL | SEARCH_MODE_POWER_HI | SEARCH_MODE_STAGE_ACQ;
		pAcqConfig->AcqChNumber = SatNumber;
		AddAcqTask(pAcqConfig);
	}
}

//*************** Add acquisition task for all satellites in view ****************
//* assume GPS L1C/A satellites put first and followed by E1/B1C satellites
// Parameters:
//   SatNumber: number of satellites in view
//   SatList: pointer array of predict parameters for valid satellites
//   SignalSvid: array of signal/svid combination for valid satellites
// Return value:
//   none
void SearchSatelliteInView(int SatNumber, PSAT_PREDICT_PARAM SatList[], U8 SignalSvid[])
{
	int i = 0, SatCount;
	PACQ_CONFIG pAcqConfig;

	// first task search L1C/A signal
	SatCount = 0;
	if ((pAcqConfig = GetFreeAcqTask()) != NULL)
	{
		for (; i < SatNumber; i ++)
		{
			if (!SIGNAL_IS_L1CA(GET_SIGNAL(SignalSvid[i])))
				break;
			if (!(g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_GPS))
				continue;
			pAcqConfig->SatConfig[SatCount].SignalSvid = SignalSvid[i];
			pAcqConfig->SatConfig[SatCount].CodeSpan = 3;
			pAcqConfig->SatConfig[SatCount].CenterFreq = SatList[i]->Doppler;
			SatCount ++;
		}
		pAcqConfig->SearchMode = SEARCH_MODE_TYPE_BPSK | SEARCH_MODE_FREQ_NARROW | SEARCH_MODE_POWER_HI | SEARCH_MODE_STAGE_ACQ;
		pAcqConfig->AcqChNumber = SatCount;
		if (SatCount)
			AddAcqTask(pAcqConfig);
	}

	// next task search E1/B1C signal
	if (SatCount != 0)
		pAcqConfig = GetFreeAcqTask();	// get new acq task if first task valid
	SatCount = 0;
	if (pAcqConfig != NULL)
	{
		for (; i < SatNumber; i ++)
		{
			if (!(g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_GAL) && SIGNAL_IS_E1(GET_SIGNAL(SignalSvid[i])))
				continue;
			if (!(g_PvtConfig.PvtConfigFlags & PVT_CONFIG_USE_BDS) && SIGNAL_IS_B1C(GET_SIGNAL(SignalSvid[i])))
				continue;
			pAcqConfig->SatConfig[SatCount].SignalSvid = SignalSvid[i];
			pAcqConfig->SatConfig[SatCount].CodeSpan = SIGNAL_IS_E1(GET_SIGNAL(SignalSvid[i])) ? 12 : 30;
			pAcqConfig->SatConfig[SatCount].CenterFreq = SatList[i]->Doppler;
			SatCount ++;
		}
		pAcqConfig->SearchMode = SEARCH_MODE_TYPE_BOC | SEARCH_MODE_FREQ_NARROW | SEARCH_MODE_POWER_HI | SEARCH_MODE_STAGE_ACQ;
		pAcqConfig->AcqChNumber = SatCount;
		if (SatCount)
			AddAcqTask(pAcqConfig);
	}
}
