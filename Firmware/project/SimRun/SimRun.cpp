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
	fprintf(DebugFile, "SV# SatPhase SatDoppler SatCode LocalPhase LocalFre LocalCode PhaseDiff FreqDiff  PsrDiff\n");

	AttachDebugFunc(DebugOutput);
	FirmwareInitialize(HotStart, &InitTime, &InitPosition);
	EnableRF();
	if (DebugFile)
		fclose(DebugFile);
//	SaveAllParameters();
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
	CTrackingChannel *TrackingChannel;
	double CodeLength = 299792458 / 1.023e6;

	if (!DebugFile)
		return;
//	if ((DebugValue % 100) != 0 && !(DebugValue >= 29000 && DebugValue <= 30100))
	if ((DebugValue % 100) != 0)
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
			TrackingChannel = &(TrackingEngine->LogicChannel[i]);
			CarrierPhase = GetCarrierPhase(pSatParam, 0);
			CarrierPhase -= (int)CarrierPhase;
			CarrierPhase = 1 - CarrierPhase;
			Doppler = GetDoppler(pSatParam, 0);
			TransmitTime = GetTransmitTime(GnssTop->CurTime, GetTravelTime(pSatParam, 0));
			PeakPosition = TransmitTime.SubMilliSeconds * 2046;
			if (TrackingChannel->SystemSel == SignalE1)	// Galileo E1
				PeakPosition += (TransmitTime.MilliSeconds % 4) * 2046;
			else if (TrackingChannel->SystemSel >= SignalB1C)	// B1C or L1C
				PeakPosition += (TransmitTime.MilliSeconds % 10) * 2046;
			LocalPhase = TrackingChannel->CarrierPhase / 4294967296.;
			LocalDoppler = TrackingChannel->CarrierFreq - IF_FREQ;
			if (TrackingChannel->SystemSel != SignalL1CA && TrackingChannel->EnableBOC == 0)	// using sidelobe tracking
				LocalDoppler -= 1023000.;
			PrnCount2x = TrackingChannel->PrnCount * 2 + TrackingChannel->CodeSubPhase - 4;
			CodePhase = PrnCount2x + (double)TrackingChannel->CodePhase / 4294967296.;
			PhaseDiff = LocalPhase - CarrierPhase;
			if (PhaseDiff < -0.25)
				PhaseDiff += 1.0;
			else if (PhaseDiff > 0.75)
				PhaseDiff -= 1.0;
			DopplerDiff = LocalDoppler - Doppler;
			CodeDiff = CodePhase - PeakPosition;
			fprintf(DebugFile, "%c%02d %8.6f %9.3f %9.3f %8.6f %9.3f %9.3f %9.6f %8.3f %8.3f\n", (pSatParam->system == GpsSystem) ? 'G' : (pSatParam->system == BdsSystem) ? 'C' : 'E',
				pSatParam->svid, CarrierPhase, Doppler, PeakPosition, LocalPhase, LocalDoppler, CodePhase, PhaseDiff, DopplerDiff, CodeDiff * CodeLength / 2);
		}
	}
}
