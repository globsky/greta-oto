#include <stdio.h>
#include "PlatformCtrl.h"
#include "TEManager.h"
#include "TimeManager.h"

#define DEFAULT_LEAP_SECOND 18

static void AlignReceiverTime(unsigned int TickCount);
static void PredictReceiverTime(unsigned int TickCount, int RcvrIntervalMs);

static U32 TimeMutex;
RECEIVER_TIME GnssTime;

//*************** Initialize GnssTime structure ****************
// Parameters:
//   none
// Return value:
//   none
void TimeInitialize()
{
	GnssTime.TimeQuality = UnknownTime;
	GnssTime.GpsMsCount = GnssTime.BdsMsCount = -1;
	GnssTime.GpsWeekNumber = GnssTime.BdsWeekNumber = -1;
	GnssTime.GpsClkError = GnssTime.BdsClkError = GnssTime.GalClkError = 0.0;
	GnssTime.ClkDrifting = 0.0;
	GnssTime.TimeFlag = GnssTime.TickCount = 0;
	TimeMutex = MutexCreate();
}

//*************** Update GnssTime to from previouos epoch to current epoch ****************
// Parameters:
//   TickCount: baseband tick count coincide to current epoch
//   RcvrIntervalMs: ms interval from previouos epoch to current epoch
// Return value:
//   none
void UpdateReceiverTime(unsigned int TickCount, int RcvrIntervalMs)
{
	if (GnssTime.TimeQuality == CoarseTime)
		AlignReceiverTime(TickCount);
	else if (GnssTime.TimeQuality != UnknownTime)
		PredictReceiverTime(TickCount, RcvrIntervalMs);
}

//*************** Align receiver time from coarse time to current measurement epoch ****************
// Parameters:
//   TickCount: baseband tick count coincide to current epoch
// Return value:
//   none
void AlignReceiverTime(unsigned int TickCount)
{
	int MsDiff = (TickCount - GnssTime.TickCount) & (~1);	// force to be multiple of 2ms

	MutexTake(TimeMutex);

	if (GnssTime.TimeFlag & TIME_WEEK_MS_VALID)
	{
		GnssTime.GpsMsCount += MsDiff;
		GnssTime.BdsMsCount += MsDiff;
	}
	GnssTime.TickCount = TickCount;
	GnssTime.TimeQuality = KeepTime;

	MutexGive(TimeMutex);
}

//*************** Predict receiver time from previous measurement epoch to current measurement epoch ****************
// Parameters:
//   TickCount: baseband tick count coincide to current epoch
//   RcvrIntervalMs: ms interval from previouos epoch to current epoch
// Return value:
//   none
void PredictReceiverTime(unsigned int TickCount, int RcvrIntervalMs)
{
	double ClkDrifting;

	MutexTake(TimeMutex);

	GnssTime.TickCount = TickCount;
	// update clock error, XXXClkError in unit of second
	ClkDrifting = GnssTime.ClkDrifting * RcvrIntervalMs / LIGHT_SPEED / 1000.;
	GnssTime.GpsClkError += ClkDrifting;
	GnssTime.BdsClkError += ClkDrifting;
	GnssTime.GalClkError += ClkDrifting;

	// update current millisecond count
//	if (GnssTime.TimeFlag & TIME_WEEK_MS_VALID)
	{
		GnssTime.GpsMsCount += RcvrIntervalMs;
		GnssTime.BdsMsCount += RcvrIntervalMs;
		if (GnssTime.GpsMsCount >= MS_IN_WEEK)
		{
			GnssTime.GpsMsCount -= MS_IN_WEEK;
			if (GnssTime.TimeFlag & TIME_WEEK_NUM_VALID)
				GnssTime.GpsWeekNumber ++;
		}
		if (GnssTime.BdsMsCount >= MS_IN_WEEK)
		{
			GnssTime.BdsMsCount -= MS_IN_WEEK;
			if (GnssTime.TimeFlag & TIME_WEEK_NUM_VALID)
				GnssTime.BdsWeekNumber ++;
		}
	}
	MutexGive(TimeMutex);
}

//*************** Set week millisecond of GnssTime ****************
//* This funcvtion will be called when week ms of receiver time is initialize
//* usually when TOW first get in frame process
// Parameters:
//   FreqID: Frequency ID of the week ms belongs
//   WeekNumber: week number, -1 if parameter invalid
//   CurTimeMs: week milliseond, -1 if parameter invalid
//   TickCount: baseband tick count coincide to current epoch
// Return value:
//   0
int SetReceiverTime(U8 FreqID, int WeekNumber, int CurWeekMs, unsigned int TickCount)
{
	MutexTake(TimeMutex);

	if (CurWeekMs >= 0)	// week millisecond parameter valid
	{
		if (CurWeekMs >= MS_IN_WEEK)	// in case predict receiver time go beyond week boundary
		{
			CurWeekMs -= MS_IN_WEEK;
			if (WeekNumber >= 0)
				WeekNumber ++;
		}
		if (FREQ_ID_IS_B1C(FreqID))	// BDS time
		{
			GnssTime.BdsMsCount = CurWeekMs;
			GnssTime.GpsMsCount = CurWeekMs + 14000;
			if (GnssTime.GpsMsCount >= MS_IN_WEEK)
				GnssTime.GpsMsCount -= MS_IN_WEEK;
		}
		else	// GPS/Galileo time
		{
			GnssTime.GpsMsCount = CurWeekMs;
			GnssTime.BdsMsCount = CurWeekMs - 14000;
			if (GnssTime.BdsMsCount < 0)
				GnssTime.BdsMsCount += MS_IN_WEEK;
		}
		GnssTime.TimeFlag |= TIME_WEEK_MS_VALID;

		if (GnssTime.TimeQuality < CoarseTime)
		{
			GnssTime.TickCount = TickCount;
			GnssTime.TimeQuality = CoarseTime;
		}
	}

	if (WeekNumber < 0 || (GnssTime.TimeFlag & TIME_WEEK_MS_VALID) == 0)	// week number invalid or week millisecond not valid
	{
		MutexGive(TimeMutex);
		return 0;
	}

	if (FREQ_ID_IS_E1(FreqID))	// Galileo week is 1024 smaller than GPS week
		WeekNumber += 1024;

	if (FREQ_ID_IS_B1C(FreqID))
	{
		if (GnssTime.BdsWeekNumber < 0 || GnssTime.BdsWeekNumber != WeekNumber)	// BDS week not valid or does not match
		{
			GnssTime.BdsWeekNumber = WeekNumber;
			GnssTime.GpsWeekNumber = GnssTime.BdsWeekNumber + 1356;
			if (GnssTime.BdsMsCount > (MS_IN_WEEK - 14000))	// GPS at next week due to leap second difference
				GnssTime.GpsWeekNumber ++;
		}
	}
	else
	{
		if (GnssTime.GpsWeekNumber < 0 || GnssTime.GpsWeekNumber != WeekNumber)	// GPS week not valid or does not match
		{
			GnssTime.GpsWeekNumber = WeekNumber;
			GnssTime.BdsWeekNumber = GnssTime.GpsWeekNumber - 1356;
			if (GnssTime.GpsMsCount < 14000)	// BDS at previous week due to leap second difference
				GnssTime.BdsWeekNumber --;
		}
	}
	GnssTime.TimeFlag |= TIME_WEEK_NUM_VALID;

	MutexGive(TimeMutex);
	return 0;
}

//*************** Get week millisecond of at specified epoch ****************
// Parameters:
//   FreqID: Frequency ID of the week ms to be get
//   TickCount: baseband tick count indicating the time of epoch
// Return value:
//   Week ms at the epoch or -1 if week ms is invalid
int GetReceiverWeekMs(U8 FreqID, unsigned int TickCount)
{
	int WeekMs = -1;

	if (GnssTime.TimeQuality < CoarseTime)
		return -1;

	if (FREQ_ID_IS_B1C(FreqID))
	{
		WeekMs = GnssTime.BdsMsCount + (int)(TickCount - GnssTime.TickCount);
		if (WeekMs < 0)
			WeekMs += MS_IN_WEEK;
		else if (WeekMs > MS_IN_WEEK)
			WeekMs -= MS_IN_WEEK;
	}
	else
	{
		WeekMs = GnssTime.GpsMsCount + (int)(TickCount - GnssTime.TickCount);
		if (WeekMs < 0)
			WeekMs += MS_IN_WEEK;
		else if (WeekMs > MS_IN_WEEK)
			WeekMs -= MS_IN_WEEK;
	}

	return WeekMs;
}

//*************** Get current week number ****************
// Parameters:
//   FreqID: Frequency ID of the week ms to be get
// Return value:
//   Current week number or -1 if week number is invalid
int GetReceiverWeekNumber(U8 FreqID)
{
	if (GnssTime.TimeFlag & TIME_WEEK_NUM_VALID)
		return FREQ_ID_IS_B1C(FreqID) ? GnssTime.BdsWeekNumber : FREQ_ID_IS_E1(FreqID) ? GnssTime.GpsWeekNumber - 1024 : GnssTime.GpsWeekNumber;
	else
		return -1;
}

//*************** Determine whether week millisecond valid ****************
// Parameters:
//   none
// Return value:
//   not 0 if valid, 0 if invalid
int ReceiverWeekMsValid()
{
	return GnssTime.TimeFlag & TIME_WEEK_MS_VALID;
}

//*************** Determine whether week number ****************
// Parameters:
//   none
// Return value:
//   not 0 if valid, 0 if invalid
int ReceiverWeekNumberValid()
{
	return GnssTime.TimeFlag & TIME_WEEK_NUM_VALID;
}
