#include <stdio.h>
#include <string.h>

#include "ConstTable.h"
#include "TaskManager.h"
#include "TEManager.h"
#include "TimeManager.h"
#include "PvtEntry.h"
#include "SystemConfig.h"

CHANNEL_STATE ChannelStateArray[TOTAL_CHANNEL_NUMBER];
int FrameInfoInit[TOTAL_CHANNEL_NUMBER];
BB_MEASUREMENT BasebandMeasurement[TOTAL_CHANNEL_NUMBER];
int NominalMeasInterval;
extern FILE *fp_debug;

BB_MEAS_PARAM MeasurementParam;
DATA_FOR_DECODE DataForDecode;

extern int DoDataDecode(void* Param);
extern int MeasProcTask(void *Param);
extern void DoAllTasks();

enum TimeAccuracy GetTimeQuality(char TimeQuanlity);

int main(int argc, char *argv[])
{
	char MessageBuffer[256];
	FILE *fp_bbmsg;
	int i, MeasurementNumber = 0;
	int LogicChannel, Svid, FreqID;
	PBB_MEASUREMENT BasebandMeas;
	int CodeRate;
	char TimeQuality;
	int DataNumber;

	if (argc > 1)
		strcpy(MessageBuffer, argv[1]);
	else
		strcpy(MessageBuffer, "test_obs2.bbo");

	if ((fp_bbmsg = fopen(MessageBuffer, "r")) == NULL)
		return 1;

	MeasurementParam.MeasMask = 0;
	memset(ChannelStateArray, 0, sizeof(ChannelStateArray));
	for (i = 0; i < TOTAL_CHANNEL_NUMBER; i ++)
	{
		ChannelStateArray[i].LogicChannel = i;
		BasebandMeasurement[i].ChannelState = &ChannelStateArray[i];
	}
	TaskInitialize();
	MsrProcInit();
	PvtProcInit(NULL);
	fp_debug = stdout;
	NominalMeasInterval = DEFAULT_MEAS_INTERVAL;

	while (!feof(fp_bbmsg))
	{
		fgets(MessageBuffer, 255, fp_bbmsg);
		if (strstr(MessageBuffer, "$PDATA"))	// baseband data message
		{
			if ((DataNumber = sscanf(MessageBuffer + 7, "%d,%d,%d\r\n", &LogicChannel, &Svid, &FreqID)) != 3)
				continue;
			ChannelStateArray[LogicChannel].Svid = Svid;
			ChannelStateArray[LogicChannel].FreqID = FreqID;
			DataForDecode.ChannelState = &ChannelStateArray[LogicChannel];
			if ((DataNumber = sscanf(MessageBuffer + 16, "%d,%u,%x\r\n", &(DataForDecode.SymbolIndex), &(DataForDecode.TickCount), &(DataForDecode.DataStream))) == 3)
				DoDataDecode((void *)(&DataForDecode));
		}
		else if (strstr(MessageBuffer, "$PMSRP"))	// begin of baseband measurement
		{
			if ((DataNumber = sscanf(MessageBuffer + 7, "%d,%d,%d,%d", &MeasurementNumber, &(MeasurementParam.TickCount), &(MeasurementParam.Interval), &(MeasurementParam.ClockAdjust))) != 4)
				continue;
			MeasurementParam.MeasMask = 0;
		}
		else if (strstr(MessageBuffer, "$PBMSR"))	// baseband measurement of one channel
		{
			if ((DataNumber = sscanf(MessageBuffer + 7, "%d,%d,%d\r\n", &LogicChannel, &Svid, &FreqID)) != 3)
				continue;
			ChannelStateArray[LogicChannel].Svid = Svid;
			ChannelStateArray[LogicChannel].FreqID = FreqID;
			MeasurementParam.MeasMask |= (1 << LogicChannel);
			BasebandMeas = &BasebandMeasurement[LogicChannel];
			if ((DataNumber = sscanf(MessageBuffer + 16, "%u,%u,%u,%d,%u,%d,%d,%x,%d,%u\r\n",
				&(BasebandMeas->CarrierFreq), &(BasebandMeas->CarrierPhase), &(BasebandMeas->CarrierCount), &(BasebandMeas->CodeCount), &(BasebandMeas->CodePhase), &CodeRate,
				&(BasebandMeas->WeekMsCount), &(BasebandMeas->ChannelState->State), &(BasebandMeas->ChannelState->CN0), &(BasebandMeas->ChannelState->TrackingTime))) != 10)
				continue;
		}
		else if (strstr(MessageBuffer, "$PMSRE"))	// end of baseband measurement
		{
			if ((DataNumber = sscanf(MessageBuffer + 7, "%c,%d,%d", &TimeQuality, &(GnssTime.GpsMsCount), &(GnssTime.BdsMsCount))) == 3)
			{
				GnssTime.TickCount = MeasurementParam.TickCount;
				GnssTime.TimeQuality = GetTimeQuality(TimeQuality);
				MeasProcTask((void *)(&MeasurementParam));
			}
		}
		DoAllTasks();
	}
}

PCHANNEL_STATE GetChannelStateArray(int Group)
{
	return &ChannelStateArray[Group * 32];
}

enum TimeAccuracy GetTimeQuality(char TimeQuality)
{
	switch (TimeQuality)
	{
	case 'U':
		return UnknownTime;
	case 'E':
		return ExtSetTime;
	case 'C':
		return CoarseTime;
	case 'K':
		return KeepTime;
	case 'A':
		return AccurateTime;
	default:
		return UnknownTime;
	}
}
