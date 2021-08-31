//----------------------------------------------------------------------
// Convert.c:
//   functions to convert between different data
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "DataTypes.h"
#include <math.h>

static int DaysAcc[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define CUBIC(x) ((x)*(x)*(x))

//*************** convert from ECEF coordinate to LLH coordinate ****************
// Parameters:
//   ecef_pos: pointer to ECEF coordinate
//   llh_pos: pointer to LLH coordinate
// Return value:
//   none
void EcefToLlh(const KINEMATIC_INFO *ecef_pos, LLH *llh_pos)
{
	double p;
	double theta;
	double n;

	if (!llh_pos || !ecef_pos)
		return;
	
	p = sqrt(ecef_pos->x * ecef_pos->x + ecef_pos->y * ecef_pos->y);

	if( p < 1e-10)	// north or south pole
	{
		llh_pos->lon = 0;
		llh_pos->lat = PI / 2;
		llh_pos->hae = (ecef_pos->z > 0) ? ecef_pos->z - WGS_AXIS_B : -ecef_pos->z - WGS_AXIS_B;
		return;
	}

	theta = atan(ecef_pos->z * WGS_AXIS_A / (p * WGS_AXIS_B));
	llh_pos->lat = atan((ecef_pos->z + WGS_E2_SQR * WGS_AXIS_B * CUBIC(sin (theta))) /
								  (p - WGS_E1_SQR * WGS_AXIS_A * CUBIC(cos (theta))));
	llh_pos->lon = atan2(ecef_pos->y, ecef_pos->x);
	
	n = WGS_AXIS_A / sqrt(1.0 - WGS_E1_SQR * sin(llh_pos->lat) * sin(llh_pos->lat));
	llh_pos->hae = p / cos(llh_pos->lat) - n;
}

//*************** convert from LLH coordinate to ECEF coordinate ****************
// Parameters:
//   llh_pos: pointer to LLH coordinate
//   ecef_pos: pointer to ECEF coordinate
// Return value:
//   none
void LlhToEcef(const LLH *llh_pos, KINEMATIC_INFO *ecef_pos)
{
	double n;

	if (!llh_pos || !ecef_pos)
		return;

	n = WGS_AXIS_A / sqrt(1.0 - WGS_E1_SQR * sin(llh_pos->lat) * sin(llh_pos->lat));

	ecef_pos->x = (n + llh_pos->hae) * cos (llh_pos->lat) * cos (llh_pos->lon);
	ecef_pos->y = (n + llh_pos->hae) * cos (llh_pos->lat) * sin (llh_pos->lon);
	ecef_pos->z = (n * (1.0 - WGS_E1_SQR) + llh_pos->hae) * sin (llh_pos->lat);  
}

//*************** convert from GLONASS time to UTC time ****************
// Parameters:
//   LeapYears: leap year
//   DayNumber: day number within 4 year
//   DayMsCount: millisecond within day
//   pUtcTime: pointer to UTC time
// Return value:
//   none
void GlonassTimeToUtc(int LeapYears, int DayNumber, int DayMsCount, PSYSTEM_TIME pUtcTime)
{
	int i, Seconds, LeapDay = 0;

	DayMsCount -= 10800000;
	if (DayMsCount < 0)
	{
		DayMsCount += 86400000;
		DayNumber --;
	}
	Seconds = DayMsCount / 1000;
	pUtcTime->Millisecond = DayMsCount - Seconds * 1000;
	LeapYears *= 4;
	DayNumber --;
	if (DayNumber >= (366 + 365 * 2))
	{
		DayNumber -= (366 + 365 * 2);
		LeapYears += 3;
	}
	else if (DayNumber >= (366 + 365))
	{
		DayNumber -= (366 + 365);
		LeapYears += 2;
	}
	else if (DayNumber >= 366)
	{
		DayNumber -= 366;
		LeapYears ++;
	}
	else if (DayNumber >= 60)
		DayNumber --;
	else if (DayNumber == 59)
		LeapDay = 1;

	for (i = 1; i < 12; i ++)
	{
		if (DayNumber < DaysAcc[i])
			break;
	}
	if (LeapDay)
	{
		pUtcTime->Month = 2;
		pUtcTime->Day = 29;
	}
	else
	{
		pUtcTime->Month = i;
		pUtcTime->Day = DayNumber - (DaysAcc[i-1] - 1);
	}
	pUtcTime->Year = 1992 + LeapYears;
	pUtcTime->Hour = Seconds / 3600;
	Seconds -= pUtcTime->Hour * 3600;
	pUtcTime->Minute = Seconds / 60;
	pUtcTime->Second = Seconds - pUtcTime->Minute * 60;
}

//*************** convert from UTC time to GLONASS time ****************
// Parameters:
//   pUtcTime: pointer to UTC time
//   pLeapYears: pointer to leap year
//   pDayNumber: pointer to day number within 4 year
//   pDayMsCount: pointer to millisecond within day
// Return value:
//   none
void UtcToGlonassTime(PSYSTEM_TIME pUtcTime, int *LeapYears, int *DayNumber, int *DayMsCount)
{
	int Years, Days;

	*DayMsCount = (((pUtcTime->Hour * 60) + pUtcTime->Minute) * 60 + pUtcTime->Second) * 1000 + pUtcTime->Millisecond + 10800000;
	Years = pUtcTime->Year - 1992;
	Days = DaysAcc[pUtcTime->Month - 1] + pUtcTime->Day - 1;
	if ((Years % 4) != 0 || Days >= 59)
		Days ++;
	Days += (Years % 4) * 365;
	*DayNumber = Days + 1;
	*LeapYears = Years / 4;
}

//*************** convert from GPS time to UTC time ****************
//* This program handles the date from Jan. 1, 1984 00:00:00.00 UTC
//* till year 2099 !!! (do not treat year 2100 as common year)
// Parameters:
//   GpsWeek: GPS week number
//   WeekMsCount: millisecond within week
//   pUtcTime: pointer to UTC time
//   pUtcParam: pointer to UTC parameter (if it is NULL, do not apply leap second)
// Return value:
//   none
void GpsTimeToUtc(int GpsWeek, int WeekMsCount, PSYSTEM_TIME pUtcTime, PUTC_PARAM pUtcParam)
{
	int LeapYears, TotalDays, MsSeconds;

	// calculate total days and seconds
	// to prevent seconds less than zero after leap second adjust
	// add seconds of one week
	TotalDays = (GpsWeek - 1) * 7;
	MsSeconds = WeekMsCount + 604800000;
	if (pUtcParam)
	{
		if (pUtcParam->flag)
		{
			MsSeconds -= pUtcParam->TLS * 1000;
			if (pUtcParam->TLS != pUtcParam->TLSF)
			{
				if ((GpsWeek > pUtcParam->WNLSF) ||
					((GpsWeek == pUtcParam->WNLSF) && (MsSeconds / 86400000) > (pUtcParam->DN + 7)))
				{
					MsSeconds -= (pUtcParam->TLSF - pUtcParam->TLS) * 1000;
				}
			}
		}
		else	// default value
		{
			MsSeconds -= 18 * 1000;
		}
	}
	TotalDays += MsSeconds / 86400000;
	MsSeconds %= 86400000;

	// calculate year
	TotalDays -= 208 * 7;
	LeapYears = TotalDays / (366 + 365 * 3);
	TotalDays -= LeapYears * (366 + 365 * 3);

	GlonassTimeToUtc(LeapYears - 2, TotalDays + 1, MsSeconds + 10800000, pUtcTime);
}

//*************** convert from UTC time to GPS time ****************
// Parameters:
//   pUtcTime: pointer to UTC time
//   pGpsWeek: pointer to GPS week number
//   pWeekMsCount: pointer to millisecond within week
//   pUtcParam: pointer to UTC parameter
// Return value:
//   none
void UtcToGpsTime(PSYSTEM_TIME pUtcTime, int *pGpsWeek, int *pWeekMsCount, PUTC_PARAM pUtcParam)
{
	int LeapYears, TotalDays, MsSeconds;

	UtcToGlonassTime(pUtcTime, &LeapYears, &TotalDays, &MsSeconds);
	MsSeconds -= 10800000;
	TotalDays --;	// convert to 1 based day count
	if (pUtcParam && pUtcParam->flag)
	{
		MsSeconds += pUtcParam->TLS * 1000;
	}
	else	// default value
	{
		MsSeconds += 18 * 1000;
	}
	if (MsSeconds >= 86400000)
	{
		MsSeconds -= 86400000;
		TotalDays ++;
	}
	else if (MsSeconds < 0)
	{
		MsSeconds += 86400000;
		TotalDays --;
	}
	TotalDays += (LeapYears + 2) * (366 + 365 * 3);
	*pGpsWeek  = TotalDays / 7 + 208;
	*pWeekMsCount = (TotalDays % 7) * 86400000 + MsSeconds;
}

//*************** calculate onvertion matrix based on ECEF position ****************
//* this is a approximation calculation to avoid sin/cos calculation
//* |e|   |-y/P      x/P     0  | |x|
//* |n| = |-x*z/P/R -y*z/P/R P/R|*|y|
//* |u|   | x/R      y/R     z/R| |z|
// Parameters:
//   pReceiverPos: pointer to ECEF position
//   pConvertMatrix: pointer to convertion matrix
// Return value:
//   none
void CalcConvMatrix(KINEMATIC_INFO *pReceiverPos, PCONVERT_MATRIX pConvertMatrix)
{
	double P, R;

	P = pReceiverPos->x * pReceiverPos->x + pReceiverPos->y * pReceiverPos->y;
	R = P + pReceiverPos->z * pReceiverPos->z;
	P = sqrt(P);
	R = sqrt(R);

	if (R < 1e-5)
		return;
	if (P < 1e-5)
	{
		pConvertMatrix->x2e = 0.0;
		pConvertMatrix->y2e = 1.0;
	}
	else
	{
		pConvertMatrix->x2e = -pReceiverPos->y / P;
		pConvertMatrix->y2e =  pReceiverPos->x / P;
	}
	pConvertMatrix->x2u =  pReceiverPos->x / R;
	pConvertMatrix->y2u =  pReceiverPos->y / R;
	pConvertMatrix->z2u =  pReceiverPos->z / R;
	pConvertMatrix->x2n = -pConvertMatrix->y2e * pConvertMatrix->z2u;
	pConvertMatrix->y2n =  pConvertMatrix->x2e * pConvertMatrix->z2u;
	pConvertMatrix->z2n =  P / R;
}
