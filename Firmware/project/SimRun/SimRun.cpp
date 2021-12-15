#include <stdio.h>
#include <math.h>
#include <string.h>

#include "HWCtrl.h"
extern "C" {
#include "FirmwarePortal.h"
}

#include "ConstVal.h"
#include "GnssTop.h"

void DebugOutput(void *DebugParam, int DebugValue);

FILE *DebugFile = 0;

void main()
{
	DebugFile = fopen("TrackState.txt", "w");
	SetInputFile("test_obs2.xml");

	AttachDebugFunc(DebugOutput);
	FirmwareInitialize();
	EnableRF();
	if (DebugFile)
		fclose(DebugFile);
}

void DebugOutput(void *DebugParam, int DebugValue)
{
	int i;
	unsigned int EnableMask;
	CGnssTop *GnssTop = (CGnssTop *)DebugParam;
	CTrackingEngine *TrackingEngine = &(GnssTop->TrackingEngine);
	SATELLITE_PARAM *pSatParam;
	int PrnCount2x;
	double CarrierPhase, Doppler, PeakPosition;
	double LocalPhase, LocalDoppler, CodePhase;
	double PhaseDiff, DopplerDiff, CodeDiff;
	GNSS_TIME TransmitTime;
	ChannelConfig *ChannelParam; 

	if (!DebugFile)
		return;
	if ((DebugValue % 100) != 0 && !(DebugValue >= 29000 && DebugValue <= 30100))
		return;
	fprintf(DebugFile, "Time %6d\n", DebugValue);
	for (i = 0, EnableMask = 1; i < 32; i ++, EnableMask <<= 1)
	{
		if (((TrackingEngine->ChannelEnable) & EnableMask) == 0)
			continue;
		// find whether there is visible satellite match current channel
		pSatParam = TrackingEngine->FindSatParam(i, GnssTop->SatParamList, GnssTop->TotalSatNumber);
		if (pSatParam)
		{
			ChannelParam = &(TrackingEngine->ChannelParam[i]);
			CarrierPhase = pSatParam->TravelTime * 1575.42e6 - pSatParam->IonoDelay / GPS_L1_WAVELENGTH;
			CarrierPhase -= (int)CarrierPhase;
			CarrierPhase = 1 - CarrierPhase;
			Doppler = -pSatParam->RelativeSpeed / GPS_L1_WAVELENGTH;
			TransmitTime = GetTransmitTime(GnssTop->CurTime, pSatParam->TravelTime + pSatParam->IonoDelay / LIGHT_SPEED + 0.001);	// add one extra millisecond to get previous finished millisecond
			PeakPosition = TransmitTime.SubMilliSeconds * 2046;
			LocalPhase = ChannelParam->CarrierPhase / 4294967296.;
			LocalDoppler = ChannelParam->CarrierFreq - IF_FREQ;
			PrnCount2x = ChannelParam->PrnCount * 2 + ChannelParam->CodeSubPhase - 4;
			CodePhase = PrnCount2x + (double)ChannelParam->CodePhase / 4294967296.;
			PhaseDiff = LocalPhase - CarrierPhase;
			if (PhaseDiff < -0.25)
				PhaseDiff += 1.0;
			else if (PhaseDiff > 0.75)
				PhaseDiff -= 1.0;
			DopplerDiff = LocalDoppler - Doppler;
			CodeDiff = CodePhase - PeakPosition;
			fprintf(DebugFile, "%c%02d %8.6f %9.3f %8.3f %8.6f %9.3f %8.3f %9.6f %8.3f %8.3f\n", (pSatParam->system == GpsSystem) ? 'G' : (pSatParam->system == BdsSystem) ? 'B' : 'E',
				pSatParam->svid, CarrierPhase, Doppler, PeakPosition, LocalPhase, LocalDoppler, CodePhase, PhaseDiff, DopplerDiff, CodeDiff);
		}
	}
}
