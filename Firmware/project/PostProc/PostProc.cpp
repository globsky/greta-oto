#include <stdio.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "CommonDefines.h"
#include "PvtEntry.h"
#include "GlobalVar.h"
}

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

BB_MEASUREMENT BasebandMeasurement[TOTAL_CHANNEL_NUMBER];
U32 DataStreamBuffer[100/4*TOTAL_CHANNEL_NUMBER];		// 100 8bit symbols x 32 channels
BB_MEAS_PARAM MeasurementParam;

void main()
{
	FILE *fp;
	int i;
	char InputLine[256], *p;
	U32 *BufferPointer, *DataStreamAddr;
	BB_MEASUREMENT Meas, *CurrentMeas = 0;
	int LogicChannel, Svid, FreqID;
	int DataNumber, FrameIndex;
	DATA_STREAM DataStream;

	if ((fp = fopen("test_obs2.bbo", "r")) == NULL)
		return;

	GpsDecodeInit();
	BdsDecodeInit();
	MsrProcInit();
	PvtProcInit((PRECEIVER_INFO)0);

	if (1)	// hot start
	{
		LoadAllParameters();
		g_ReceiverInfo.GpsTimeQuality = g_ReceiverInfo.BdsTimeQuality = g_ReceiverInfo.GalileoTimeQuality = UnknownTime;
		g_ReceiverInfo.PosQuality = ExtSetPos;
		g_ReceiverInfo.PosVel.vx = g_ReceiverInfo.PosVel.vy = g_ReceiverInfo.PosVel.vz = 0.0;
	}
	g_PvtConfig.PvtConfigFlags |= PVT_CONFIG_USE_KF;

	MeasurementParam.Measurements = BasebandMeasurement;
	BufferPointer = DataStreamBuffer;
	MeasurementParam.MeasMask = 0;

	while (!feof(fp))
	{
		fgets(InputLine, 255, fp);
		if (strncmp(InputLine, "$PBMSR", 6) == 0)
		{
			sscanf(InputLine + 7, "%d,%d,%d,%u,%u,%u,%d,%u,%u,%d,%x,%d,%d,%u", &LogicChannel, &Svid, &FreqID,
				&(Meas.CarrierFreq), &(Meas.CarrierNCO), &(Meas.CarrierCount), &(Meas.CodeFreq), &(Meas.CodeCount), &(Meas.CodeNCO),
				&DataNumber, &(Meas.State), &(Meas.LockIndicator), &(Meas.CN0), &(Meas.TrackingTime));
			if (LogicChannel >= 0 && LogicChannel < TOTAL_CHANNEL_NUMBER)
			{
				Meas.LogicChannel = (U8)LogicChannel;
				Meas.Svid = (U8)Svid;
				Meas.FreqID = (U8)FreqID;
				Meas.DataNumber = 0;
				Meas.DataStreamAddr = BufferPointer;
				CurrentMeas = &BasebandMeasurement[LogicChannel];
				memcpy(CurrentMeas, &Meas, sizeof(BB_MEASUREMENT));
				MeasurementParam.MeasMask |= (1 << Meas.LogicChannel);
			}
		}
		else if (strncmp(InputLine, "$PDATA", 6) == 0)
		{
			sscanf(InputLine + 7, "%d,%d", &DataNumber, &FrameIndex);
			p = InputLine + 7;
			while (*p && *p != ',') p ++;	// skip first ','
			if (*p == 0) continue; else p ++;
			while (*p && *p != ',') p ++;	// skip second ','
			if (*p == 0) continue; else p ++;
			DataStreamAddr = BufferPointer;
			while (*p)
			{
				sscanf(p, "%x", BufferPointer ++);
				while (*p && *p != ',') p ++;	// skip following ','
				if (*p == 0) break; else p ++;
			}
			CurrentMeas->DataNumber = DataNumber;
			CurrentMeas->FrameIndex = FrameIndex;
			CurrentMeas->DataStreamAddr = DataStreamAddr;
		}
		else if (strncmp(InputLine, "$PMSRP", 6) == 0)
		{
			sscanf(InputLine + 7, "%d,%d", &(MeasurementParam.MeasInterval), &(MeasurementParam.RunTimeAcc));
			// frame process
			for (i = 0; i < TOTAL_CHANNEL_NUMBER; i ++)
			{
				CurrentMeas = &BasebandMeasurement[i];
				if ((MeasurementParam.MeasMask & (1 << i)) && CurrentMeas->DataNumber > 0)
				{
					if (FREQ_ID_IS_B1C(CurrentMeas->FreqID) && CurrentMeas->FrameIndex >= 0)
					{
						DataStream.DataCount = CurrentMeas->DataNumber;
						DataStream.StartIndex = CurrentMeas->FrameIndex;
						DataStream.PrevSymbol = i;
						memcpy(DataStream.DataBuffer, CurrentMeas->DataStreamAddr, 128);
						BdsDecodeTask((void *)(&DataStream));
					}
				}
			}
			// calculate raw measurement and do PVT
			if (MeasurementParam.RunTimeAcc == 1153000)
				MeasurementParam.RunTimeAcc = 1153000;
			MsrProc(BasebandMeasurement, MeasurementParam.MeasMask, MeasurementParam.MeasInterval, MeasurementParam.MeasInterval);
			PvtProc(MeasurementParam.MeasInterval);
			// reset current measurement set
			BufferPointer = DataStreamBuffer;
			MeasurementParam.MeasMask = 0;
		}
	}
//	SaveAllParameters();
}

void LoadAllParameters()
{
	FILE *fp;
	if ((fp = fopen("ParamFile.bin", "rb")) == NULL)
		return;
	fseek(fp, PARAM_OFFSET_CONFIG, SEEK_SET); fread(&g_PvtConfig, 1, sizeof(g_PvtConfig), fp);
	fseek(fp, PARAM_OFFSET_RCVRINFO, SEEK_SET); fread(&g_ReceiverInfo, 1, sizeof(g_ReceiverInfo), fp);
	fseek(fp, PARAM_OFFSET_IONOUTC, SEEK_SET); fread(&g_GpsIonoParam, 1, sizeof(g_GpsIonoParam), fp);
	fseek(fp, PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam), SEEK_SET); fread(&g_BdsIonoParam, 1, sizeof(g_BdsIonoParam), fp);
	fseek(fp, PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam)+sizeof(g_BdsIonoParam), SEEK_SET); fread(&g_GpsUtcParam, 1, sizeof(g_GpsUtcParam), fp);
	fseek(fp, PARAM_OFFSET_IONOUTC+sizeof(g_GpsIonoParam)+sizeof(g_BdsIonoParam)+sizeof(g_GpsUtcParam), SEEK_SET); fread(&g_BdsUtcParam, 1, sizeof(g_BdsUtcParam), fp);
	fseek(fp, PARAM_OFFSET_GPSALM, SEEK_SET); fread(&g_GpsAlmanac, 1, sizeof(g_GpsAlmanac), fp);
	fseek(fp, PARAM_OFFSET_BDSALM, SEEK_SET); fread(&g_BdsAlmanac, 1, sizeof(g_BdsAlmanac), fp);
	fseek(fp, PARAM_OFFSET_GALALM, SEEK_SET); fread(&g_GalileoAlmanac, 1, sizeof(g_GalileoAlmanac), fp);
	fseek(fp, PARAM_OFFSET_GPSEPH, SEEK_SET); fread(&g_GpsEphemeris, 1, sizeof(g_GpsEphemeris), fp);
	fseek(fp, PARAM_OFFSET_BDSEPH, SEEK_SET); fread(&g_BdsEphemeris, 1, sizeof(g_BdsEphemeris), fp);
	fseek(fp, PARAM_OFFSET_GALEPH, SEEK_SET); fread(&g_GalileoEphemeris, 1, sizeof(g_GalileoEphemeris), fp);
	fclose(fp);
}
