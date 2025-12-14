//----------------------------------------------------------------------
// SatCoord.c:
//   satellite coordinate related functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "DataTypes.h"
#include <math.h>

#define COS_5 0.99619469809174553
#define SIN_5 0.087155742747658173559

//*************** Calculate satellite clock correction ****************
// Parameters:
//   pEph: pointer to ephemeris
//   TransmitTime: transmit time within week
// Return value:
//   satellite clock correction
double ClockCorrection(PGNSS_EPHEMERIS pEph, double TransmitTime)
{
	double TimeDiff = TransmitTime - pEph->toc;
	double ClockAdj;

	// protection for time ring back at week end
	if (TimeDiff > 302400.0)
		TimeDiff -= 604800;
	if (TimeDiff < -302400.0)
		TimeDiff += 604800;

	ClockAdj = pEph->af0 + (pEph->af1 + pEph->af2 * TimeDiff) * TimeDiff;
	return ClockAdj;
}

//*************** Calculate satellite position and velocity using ephemeris ****************
// Parameters:
//   TransmitTime: transmit time within week
//   pEph: pointer to ephemeris
//   pointer to satellite position and velocity
// Return value:
//   0 if ephemeris expire, otherwise 1
int SatPosSpeedEph(double TransmitTime, PGNSS_EPHEMERIS pEph, PKINEMATIC_INFO pPosVel)
{
	int i;
	double delta_t;
	double Mk, Ek, Ek1, Ek_dot;
	double phi, phi_dot;
	double uk, rk, ik;
	double uk_dot, rk_dot, ik_dot;
	double xp, yp, omega;
	double xp_dot, yp_dot;
	double sin_temp, cos_temp;

	// calculate time difference
	delta_t = TransmitTime - pEph->toe;
	// protection for time ring back at week end
	if (delta_t > 302400.0)
		delta_t -= 604800;
	if (delta_t < -302400.0)
		delta_t += 604800;

	// get Ek from Mk with recursive algorithm
	Ek1 = Ek = Mk = pEph->M0 + (pEph->n * delta_t);
	for (i = 0; i < 10; i ++)
	{
		Ek = Mk + pEph->ecc * sin(Ek);
		if (fabs(Ek - Ek1) < 1e-14)
			break;
		Ek1 = Ek;
	}
	pEph->Ek = Ek;

	// assign Ek1 as 1-e*cos(Ek)
	Ek1 = 1.0 - (pEph->ecc * cos(Ek));

	// get phi(k) with atan2
	phi = atan2(pEph->root_ecc * sin(Ek), cos(Ek) - pEph->ecc) + pEph->w;
	sin_temp = sin(phi + phi);
	cos_temp = cos(phi + phi);

	// get u(k), r(k) and i(k)
	uk = phi;
	rk = pEph->axis * Ek1;
	ik = pEph->i0 + (pEph->idot * delta_t);
	// apply 2nd order correction to u(k), r(k) and i(k)
	uk += (pEph->cuc * cos_temp) + (pEph->cus * sin_temp);
	rk += (pEph->crc * cos_temp) + (pEph->crs * sin_temp);
	ik += (pEph->cic * cos_temp) + (pEph->cis * sin_temp);
	// calculate derivatives of r(k) and u(k)
	Ek_dot = pEph->n / Ek1;
	uk_dot = phi_dot = Ek_dot * pEph->root_ecc / Ek1;
	phi_dot = phi_dot * 2.0;
	rk_dot = pEph->axis * pEph->ecc * sin(Ek) * Ek_dot;
	rk_dot += ((pEph->crs * cos_temp) - (pEph->crc * sin_temp)) * phi_dot;
	uk_dot += ((pEph->cus * cos_temp) - (pEph->cuc * sin_temp)) * phi_dot;
	ik_dot = pEph->idot + ((pEph->cis * cos_temp) - (pEph->cic * sin_temp)) * phi_dot;

	// calculate Xp and Yp and corresponding derivatives
	sin_temp = sin(uk);
	cos_temp = cos(uk);
	xp = rk * cos_temp;
	yp = rk * sin_temp;
	xp_dot = rk_dot * cos_temp - yp * uk_dot;
	yp_dot = rk_dot * sin_temp + xp * uk_dot;

	// get final position and speed in ECEF coordinate
	omega = pEph->omega_t + pEph->omega_delta * delta_t;
	sin_temp = sin(omega);
	cos_temp = cos(omega);
	phi = sin(ik);
	pPosVel->z = yp * phi;
	pPosVel->vz = yp_dot * phi;

	phi = cos(ik);
	pPosVel->x = xp * cos_temp - yp * phi * sin_temp;
	pPosVel->y = xp * sin_temp + yp * phi * cos_temp;
	// phi_dot assign as yp_dot * cos(ik) - z * ik_dot
	phi_dot = yp_dot * phi - pPosVel->z * ik_dot;
	pPosVel->vx = xp_dot * cos_temp - phi_dot * sin_temp;
	pPosVel->vy = xp_dot * sin_temp + phi_dot * cos_temp;
	pPosVel->vx -= pPosVel->y * pEph->omega_delta;
	pPosVel->vy += pPosVel->x * pEph->omega_delta;
	pPosVel->vz += yp * ik_dot * phi;

/*	if (pEph->svid >= MIN_BD2_SVID && pEph->svid < MIN_BD2_SVID + 5)
	{
		// first rotate -5 degree
		yp = pPosVel->y * COS_5 - pPosVel->z * SIN_5; // rotated y
		pPosVel->z = pPosVel->z * COS_5 + pPosVel->y * SIN_5; // rotated z
		yp_dot = pPosVel->vy * COS_5 - pPosVel->vz * SIN_5; // rotated vy
		pPosVel->vz = pPosVel->vz * COS_5 + pPosVel->vy * SIN_5; // rotated vz
		// rotate delta_t * CGS2000_OMEGDOTE
		omega = CGCS2000_OMEGDOTE * delta_t;
		sin_temp = sin(omega);
		cos_temp = cos(omega);
		pPosVel->y = yp * cos_temp - pPosVel->x * sin_temp;
		pPosVel->x = pPosVel->x * cos_temp + yp * sin_temp;
		pPosVel->vy = yp_dot * cos_temp - pPosVel->vx * sin_temp;
		pPosVel->vx = pPosVel->vx * cos_temp + yp_dot * sin_temp;
		// earth rotate compensation on velocity
		pPosVel->vx += pPosVel->y * CGCS2000_OMEGDOTE;
		pPosVel->vy -= pPosVel->x * CGCS2000_OMEGDOTE;
	}*/

	// if ephemeris expire, return 0
	if (delta_t < -7200.0 || delta_t > 7200.0)
		return 0;
	else
		return 1;
}

//*************** Calculate satellite position and velocity using almanac ****************
// Parameters:
//   WeekNumber: current week number
//   TransmitTime: transmit time within week
//   pAlm: pointer to almanac
//   pointer to satellite position and velocity
// Return value:
//   none
void SatPosSpeedAlm(int WeekNumber, int TransmitTime, PMIDI_ALMANAC pAlm, PKINEMATIC_INFO pPosVel)
{
	int i;
	int delta_t;
	double Mk, Ek, Ek1;
	double phi;
	double uk, rk, ik;
	double uk_dot, rk_dot;
	double xp, yp, omega;
	double xp_dot, yp_dot;
	double sin_temp, cos_temp;

	// calculate time difference with week number considered
	delta_t = TransmitTime - pAlm->toa;
	delta_t += (WeekNumber - pAlm->week) * 604800;

	// get Ek from Mk with recursive algorithm
	Ek1 = Ek = Mk = pAlm->M0 + (pAlm->n * delta_t);
	for (i = 0; i < 10; i ++)
	{
		Ek = Mk + pAlm->ecc * sin(Ek);
		if (fabs(Ek - Ek1) < 1e-14)
			break;
		Ek1 = Ek;
	}

	// assign Ek1 as 1-e*cos(Ek)
	Ek1 = 1.0 - (pAlm->ecc * cos(Ek));

	// get u(k), r(k) and i(k)
	phi = atan2(pAlm->root_ecc * sin(Ek), cos(Ek) - pAlm->ecc) + pAlm->w;
	uk = phi;
	rk = pAlm->axis * Ek1;
	ik = pAlm->i0;
	rk_dot = pAlm->axis * pAlm->ecc * sin(Ek) * pAlm->n / Ek1;
	uk_dot = pAlm->n * pAlm->root_ecc / (Ek1 * Ek1);

	// calculate Xp and Yp and corresponding derivatives
	sin_temp = sin(uk);
	cos_temp = cos(uk);
	xp = rk * cos_temp;
	yp = rk * sin_temp;
	xp_dot = rk_dot * cos_temp - yp * uk_dot;
	yp_dot = rk_dot * sin_temp + xp * uk_dot;

	omega = pAlm->omega_t + pAlm->omega_delta * delta_t;
	sin_temp = sin(omega);
	cos_temp = cos(omega);
	// get final position and speed in ECEF coordinate
	ik = cos(pAlm->i0);
	pPosVel->x = xp * cos_temp - yp * ik * sin_temp;
	pPosVel->y = xp * sin_temp + yp * ik * cos_temp;
	pPosVel->vx = xp_dot * cos_temp - ik * yp_dot * sin_temp;
	pPosVel->vy = xp_dot * sin_temp + ik * yp_dot * cos_temp;
	pPosVel->vx -= pPosVel->y * pAlm->omega_delta;
	pPosVel->vy += pPosVel->x * pAlm->omega_delta;
	ik = sin(pAlm->i0);
	pPosVel->z = yp * ik;
	pPosVel->vz = yp_dot * ik;
}

//*************** Calculate geometry distance between receiver and satellite ****************
//* earth rotate correction is applied here
// Parameters:
//   ReceiverPos: pointer to receiver position array
//   SatellitePos: pointer to satellite position array
// Return value:
//   geometry distance
double GeometryDistanceXYZ(const double *ReceiverPos, const double *SatellitePos)
{
	double dx, dy, dz;
	double r, rotate_corr;

	// earth rotate compensation
	rotate_corr = (SatellitePos[0] * ReceiverPos[1] - SatellitePos[1] * ReceiverPos[0]) * (WGS_OMEGDOTE / LIGHT_SPEED);
	// geodistance
	dx = (*ReceiverPos ++) - (*SatellitePos ++);
	dy = (*ReceiverPos ++) - (*SatellitePos ++);
	dz = (*ReceiverPos ++) - (*SatellitePos ++);
	r = sqrt(dx * dx + dy * dy + dz * dz);
	r += rotate_corr;
	return r;
}

//*************** Calculate geometry distance between receiver and satellite ****************
//* earth rotate correction is applied here
// Parameters:
//   pReceiver: pointer to receiver position/velocity structure
//   pSatellite: pointer to satellite position/velocity structure
// Return value:
//   geometry distance
double GeometryDistance(const PKINEMATIC_INFO pReceiver, const PKINEMATIC_INFO pSatellite)
{
	return GeometryDistanceXYZ(pReceiver->PosVel, pSatellite->PosVel);
}

//*************** Calculate relative radiation speed between receiver and satellite ****************
// Parameters:
//   pReceiver: pointer to receiver position/velocity structure
//   pSatellite: pointer to satellite position/velocity structure
// Return value:
//   relative speed in m/s (negative value means departing)
double SatRelativeSpeed(PKINEMATIC_INFO pReceiver, PKINEMATIC_INFO pSatellite)
{
	double dx, dy, dz;
	double dvx, dvy, dvz;
	double Distance;

	dx = pReceiver->x - pSatellite->x;
	dy = pReceiver->y - pSatellite->y;
	dz = pReceiver->z - pSatellite->z;
	dvx = pReceiver->vx - pSatellite->vx;
	dvy = pReceiver->vy - pSatellite->vy;
	dvz = pReceiver->vz - pSatellite->vz;
	Distance = (double)sqrt(dx * dx + dy * dy + dz * dz);
	return (dx * dvx + dy * dvy + dz * dvz) / Distance;
}

//*************** Calculate relative radiation speed between receiver and satellite ****************
// Parameters:
//   ReceiverState: pointer to receiver position/velocity state vector (vx vy vz tdot x y z)
//   SatPosVel: pointer to satellite position/velocity array (x y z vx vy vz)
// Return value:
//   relative speed in m/s (negative value means departing)
double SatRelativeSpeedXYZ(double *ReceiverState, double *SatPosVel)
{
	double dx, dy, dz;
	double dvx, dvy, dvz;
	double Distance;

	ReceiverState += 4;
	dx = (*ReceiverState ++) - (*SatPosVel ++);
	dy = (*ReceiverState ++) - (*SatPosVel ++);
	dz = (*ReceiverState ++) - (*SatPosVel ++);
	ReceiverState -= 7;
	dvx = (*ReceiverState ++) - (*SatPosVel ++);
	dvy = (*ReceiverState ++) - (*SatPosVel ++);
	dvz = (*ReceiverState ++) - (*SatPosVel ++);
	Distance = (double)sqrt(dx * dx + dy * dy + dz * dz);
	return (dx * dvx + dy * dvy + dz * dvz) / Distance;
}

//*************** Calculate satellite elevation and azimuth ****************
//* satellite position uses data in satellite information structure
//* result also put in satellite information structure
//* set corresponding flags
// Parameters:
//   pReceiver: pointer to receiver position/velocity structure
//   SatPosVel: pointer to satellite information structure
// Return value:
//   none
void SatElAz(PKINEMATIC_INFO pReceiver, PSATELLITE_INFO pSatellite)
{
	double dx, dy, dz;
	double P;
	double Distance, R, S;
	double SinEl, North, East;

	if ((fabs(pSatellite->PosVel.x) < 1e-10) && (fabs(pSatellite->PosVel.y) < 1e-10) && (fabs(pSatellite->PosVel.z) < 1e-10))
	{
		return;
	}

	dx = (pReceiver->x - pSatellite->PosVel.x);
	dy = (pReceiver->y - pSatellite->PosVel.y);
	dz = (pReceiver->z - pSatellite->PosVel.z);
	Distance = sqrt(dx * dx + dy * dy + dz * dz);
	P = pReceiver->x * pReceiver->x + pReceiver->y * pReceiver->y;
	R = sqrt(P + pReceiver->z * pReceiver->z);
	S = pReceiver->x * dx + pReceiver->y * dy;
	SinEl = (-S - pReceiver->z * dz) / R / Distance;

	if (R < 1e-10)
	{
		pSatellite->el = (PI / 2);
		pSatellite->az = 0;
	}
	else
	{
		if(SinEl >= 1.)
		{
			pSatellite->el = (PI / 2);
		}
		else if(SinEl <= -1.)
		{
			pSatellite->el = - (PI / 2);
		}
		else
		{
			pSatellite->el = asin(SinEl);
		}
		North = (pReceiver->z * S - P * dz) / R;
		East = pReceiver->y * dx - pReceiver->x * dy;
		pSatellite->az = atan2(East, North);
		if (pSatellite->az < 0)
			pSatellite->az += (2 * PI);
	}

	pSatellite->SatInfoFlag |= (SAT_INFO_ELAZ_VALID | SAT_INFO_ELAZ_MATCH);
}
