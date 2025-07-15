//----------------------------------------------------------------------
// DataTypes.h:
//   PVT data type and structure definitions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include "CommonDefines.h"
#include "ChannelManager.h"
#include "PvtConst.h"

#pragma pack(push)	// push current alignment
#pragma pack(4)		// set alignment to 4-byte boundary

#define EXTERN extern

#if !defined BOOL
typedef int BOOL;
#endif

typedef enum {	UnknownTime = 0,	// no time information
				ExtSetTime,			// time from external source (eg. RTC, network etc.)
				CoarseTime,			// time from satellite signal transmit time minus average travel time
				KeepTime,			// time after aligned to epoch of observation
				AccurateTime,		// time from successful LSQ or Kalman update
} TimeAccuracy;

typedef enum {	UnknownPos = 0,		// no position information
				ExtSetPos,			// position from external source (eg. last known position, GSM/WIFI/Bluetooth positioning etc.)
				KeepPos,			// position in keep mode after Kalman filter exceed predition threshold
				PredictPos,			// position from Kalman predition without Kalman update
				FixedUpPos,			// position in static mode
				CoarsePos,			// position from LSQ without converge
				FlexTimePos,		// position from 5 satellite positioning
				KalmanPos,			// position from Kalman update
				AccuratePos,		// position from LSQ that has converged
				EstimatePos,		// position from last RTK fixed position + carrier phase change
				DifferentialPos,	// position from DGNSS mode
				FloatSolution,		// position from RTK float solution mode
				FixedSolution,		// position from RTK fixed solution mode
} PosAccuracy;

typedef enum {	PosTypeNone = 0,	// can not do PVT
				PosTypeFlexTime,	// can do 5 satellite positioning
				PosType2D,			// can do 2D positioning (with given height)
				PosTypeLSQ,			// can do LSQ positioning
				PosTypeToKF,		// reserved for LSQ to KF transfer
				PosTypeKFPos,		// can do KF positioning
				PosTypeEstimate,	// can do positioning using carrier phase change
				PosTypeUseAmb,		// can do positioning with ambiguity calculated in previous epoch
} PositionType;

typedef struct
{
	double ve, vn, vu;
	double Speed, Course;
} GROUND_SPEED, *PGROUND_SPEED;

typedef struct
{
	union
	{
		struct
		{
			double x, y, z;
			double vx, vy, vz;
		};
		double PosVel[6];
	};
} KINEMATIC_INFO, *PKINEMATIC_INFO;

typedef struct CONVERT_MATRIX
{
	double x2e, y2e;
	double x2n, y2n, z2n;
	double x2u, y2u, z2u;
} CONVERT_MATRIX, *PCONVERT_MATRIX;

typedef struct tag_FRAME_INFO
{
	signed short SymbolNumber;		// number of symbols in buffer
	signed char FrameStatus;		// frame sync status -1: not sync
	unsigned char FrameFlag;		// bit 6: stream polarity 0 - Positive, 1 - Negative
									// bit 7: polarity valid 0 - Invalid, 1 - Valid
	int TimeTag;					// definition varies, time tag within week/day of start of current frame (GPS), week number decoded (BDS)
	unsigned int TickCount;			// baseband tick count at end of last symbol
	unsigned int SymbolData[12];	// buffer to store current symbol stream
	unsigned int FrameData[57];		// store frame data (LNAV need 30, I/NAV need 50 and CNAV1 need 57)
} FRAME_INFO, *PFRAME_INFO;
#define NEGATIVE_STREAM		0x40
#define POLARITY_VALID		0x80

typedef struct
{
	PCHANNEL_STATE ChannelState;
	PFRAME_INFO FrameInfo;
	int PayloadLength;
	unsigned int Symbols[0];	// length varies, at least 1
} SYMBOL_PACKAGE, *PSYMBOL_PACKAGE;

typedef struct
{
	unsigned char FreqID;			// System ID
	unsigned char svid;				// GNSS SVID
	unsigned char SatID;			// Satellite ID for internal use, different for each satellite
									// 1 to 32 for GPS
									// 33 to 64 for SBAS
									// 65 to 96 for GLONASS (0 for slot number unknown)
									// 97 to 160 reserved for Galileo and QZSS
									// 161 to 224 for BDS
	unsigned char Channel;			// Logic channel number
	unsigned short cn0;				// same as C/N0 field in BB measurement with unit of 0.01dB
	unsigned short ChannelFlag;		// see below for details
	double PseudoRangeOrigin;		// pseudo range (without correction)
	double CarrierPhase;			// carrier phase
	double DopplerHz;				// Doppler in unit of Hz
	double PseudoRange;				// pseudo range (with correction)
	double DeltaT;					// correction to pseudo range
	double Doppler;					// Doppler in unit of m/s
	double PsrVariance;				// PSR observation variance
	double DopplerVariance;			// Doppler observation variance
	double TransmitTime;			// signal transmit time at latch epoch (fractional part)
	int TransmitTimeMs;				// transmit time millisecond part or above
	int CarrierCountAcc;			// accumulated carrier count (norminal IF removed)
	unsigned int CarrierCountOld;	// carrier count of last epoch
	int LockTime;					// Continuous track time in ms without cycle slip
	unsigned int state;				// same as state field in BB measurement
	unsigned int ChannelErrorFlag;	// error flags that PVT feedback to baseband, see below for details
	FRAME_INFO FrameInfo;			// frame info structure of different system
} CHANNEL_STATUS, *PCHANNEL_STATUS;
// definitions for ChannelFlag field
#define CHANNEL_ACTIVE		0x01	// this is an active channel in baseband
#define MEASUREMENT_VALID	0x10	// raw measurement valid
#define ADR_VALID			0x20	// carrier phase observation valid
#define HALF_CYCLE			0x40	// may have half cycle
#define CYCLE_SLIP			0x80	// possible cycle between epochs
#define TRANSTIME_ESTIMATE	0x100	// TransmitTimeMs is estimated instead of from navigation data
#define MEASUREMENT_FLAGS	0xff0	// bit 4~11 reserved for raw measurement related flags
// definitions for ChannelErrorFlag field
#define CHANNEL_ERR_BIT_SYNC		0x01	// bit sync error
#define CHANNEL_ERR_BIT_NUMBER		0x02	// bit number in frame error
#define CHANNEL_ERR_XCORR           0x04	// track on cross correlation signal
#define CHANNEL_ERR_SIDELOBE		0x08	// track on side lobe in BOC signal
#define CHANNEL_ERR_NOTINVIEW		0x10	// tracking a satellite not in view	
#define CHANNEL_ERR_PSR_GAP         0x20	// PSR large gap not due to bit sync or bit number error
#define CHANNEL_ERR_DOPPLER_GAP     0x40	// Doppler large gap

typedef struct // GPS ephemeris, also used by GPS/Galileo/BDS/QZSS
{
	unsigned short	iodc;
	unsigned char	iode2;
	unsigned char	iode3;

	unsigned char	ura;
	unsigned char	flag;	// bit0 means ephemeris valid
	unsigned char	health;
	unsigned char	svid;

	int	toe;
	int	toc;
	int	week;

	// variables decoded from stream data
	double M0;			// Mean Anomaly at Reference Time
	double delta_n;		// Mean Motion Difference from Computed Value
	double ecc;			// Eccentricity
	double sqrtA;		// Square Root of the Semi-Major Axis or delta axis to reference value
	double axis_dot;	// Change rate of axis, valid for L1C/B1C
	double omega0;		// Longitude of Ascending Node of Orbit Plane at Weekly Epoch
	double i0;			// Inclination Angle at Reference Time
	double w;			// Argument of Perigee
	double omega_dot;	// Rate of Right Ascension
	double idot;		// Rate of Inclination Angle
	double cuc;			// Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
	double cus;			// Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude
	double crc;			// Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
	double crs;			// Amplitude of the Sine Harmonic Correction Term to the Orbit Radius
	double cic;			// Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
	double cis;			// Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination
	double tgd;			// Group Delay for L1C/A, L1C, E1 and B1C
	double tgd2;		// Group Delay for B2a
	double af0;			// Satellite Clock Correction
	double af1;			// Satellite Clock Correction
	double af2;			// Satellite Clock Correction

	// variables derived from basic data, avoid calculate every time
	double axis;		// Semi-major Axis of Orbit
	double n;			// Corrected Mean Angular Rate
	double root_ecc;	// Square Root of One Minus Ecc Square
	double omega_t;		// Longitude of Ascending Node of Orbit Plane at toe
	double omega_delta;	// Delta Between omega_dot and WGS_OMEGDOTE
	double Ek;			// Ek, derived from Mk
} GNSS_EPHEMERIS, *PGNSS_EPHEMERIS;

typedef struct        			
{
	unsigned char	flag;
	unsigned char	dummy;
	unsigned char	health;
	unsigned char	svid;

	int	toa;
	int	week;

	// variables decoded from stream data
	double M0;			// Mean Anomaly at Reference Time
	double ecc;			// Eccentricity
	double sqrtA;		// Square Root of the Semi-Major Axis
	double omega0;		// Longitude of Ascending Node of Orbit Plane at Weekly Epoch
	double i0;			// Inclination Angle at Reference Time
	double w;			// Argument of Perigee
	double omega_dot;	// Rate of Right Ascension
	double af0;			// Satellite Clock Correction
	double af1;			// Satellite Clock Correction

	// variables derived from basic data, avoid calculate every time
	double axis;		// Semi-major Axis of Orbit
	double n;			// Mean Angular Rate
	double root_ecc;	// Square Root of One Minus Ecc Square
	double omega_t;		// Longitude of Ascending Node of Orbit Plane at toe
	double omega_delta;	// Delta Between omega_dot and WGS_OMEGDOTE
} MIDI_ALMANAC, *PMIDI_ALMANAC;

typedef struct        			
{
	unsigned char	flag;
	unsigned char	dummy;
	unsigned char	health;
	unsigned char	svid;

	int	toa;
	int	week;

	// variables decoded from stream data
	double DeltaA;		// Square Root of the Semi-Major Axis
	double Omega0;		// Longitude of Ascending Node of Orbit Plane at Weekly Epoch
	double Phi0;		// Argument of Perigee (M0 + w)

	// variables derived from basic data, avoid calculate every time
	double axis;		// Semi-major Axis of Orbit
	double n;			// Mean Angular Rate
	double root_ecc;	// Square Root of One Minus Ecc Square
	double omega_t;		// Longitude of Ascending Node of Orbit Plane at toe
	double omega_delta;	// Delta Between omega_dot and WGS_OMEGDOTE
} REDUCED_ALMANAC, *PREDUCED_ALMANAC;

typedef struct tag_RECEIVER_TIME
{
	enum TimeAccuracy TimeQuality;
	unsigned int TimeFlag;
	unsigned int TickCount;	// Baseband tick count of observation at current receiver time
	int GpsMsCount;			// millisecond count within a week for GPS/Galileo
	int BdsMsCount;			// millisecond count within a week for BDS
	int GpsWeekNumber;		// week number of receiver time used by GPS/Galileo
	int BdsWeekNumber;		// week number of receiver time used by BDS
	double GpsClkError;		// receiver clock error to GPS time, in second
	double BdsClkError;		// receiver clock error to BDS time, in second
	double GalClkError;		// receiver clock error to Galileo time, in second
	double ClkDrifting;		// receiver clock drifting, in m/s
} RECEIVER_TIME, *PRECEIVER_TIME;

// definitions for TimeFlag
#define TIME_WEEK_MS_VALID  0x1
#define TIME_WEEK_NUM_VALID 0x2
#define GPS_CLK_ERR_VALID   0x10
#define BDS_CLK_ERR_VALID   0x20
#define GAL_CLK_ERR_VALID   0x40

typedef struct
{
	KINEMATIC_INFO PosVel;	// receiver position/velocity in ECEF coordinate
	LLH PosLLH;				// receiver position
	GROUND_SPEED GroundSpeed;// receiver group speed
//	double GpsClkError;		// receiver clock error to GPS time, in second
//	double GalileoClkError;	// receiver clock error to Galileo time, in second
//	double BdsClkError;		// receiver clock error to BDS time, in second
//	double ClkDrifting;		// receiver clock drifting, in m/s
	double DopArray[8];		// HDOP, VDOP, PDOP, TDOP, SigmaEE, SigmaNN, SigmaEN, reserved

//	int GpsMsCount;			// millisecond count within a week, identical to all systems
//	int WeekNumber;			// week number of receiver time used by GPS
//	TimeAccuracy GpsTimeQuality;
//	TimeAccuracy BdsTimeQuality;
//	TimeAccuracy GalileoTimeQuality;
	RECEIVER_TIME *ReceiverTime;
	PosAccuracy PosQuality;
	unsigned int PosFlag;	// Positioning flag
	PositionType PrevPosType;
	PositionType CurrentPosType;
//	unsigned int PosUseSat; //sats used to positioning
//	unsigned int VelUseSat; //sats used to velocity
	CONVERT_MATRIX ConvertMatrix;
} RECEIVER_INFO, *PRECEIVER_INFO;
// definitions for PosFlag field
#define SYSTEM_GPS			0
#define SYSTEM_BDS			1
#define SYSTEM_GAL			2
#define PVT_USE_GPS			(1 << SYSTEM_GPS)
#define PVT_USE_BDS			(1 << SYSTEM_BDS)
#define PVT_USE_GAL			(1 << SYSTEM_GAL)

typedef struct
{
	unsigned int PvtConfigFlags;
	unsigned int GpsSatMaskOut;
	unsigned long long GalileoSatMaskOut;
	unsigned long long BdsSatMaskOut;
	double ElevationMask;	// in radian
} PVT_CONFIG, *PPVT_CONFIG;
// definitions for PvtConfigFlags field
#define PVT_CONFIG_USE_GPS			(1 << SYSTEM_GPS)
#define PVT_CONFIG_USE_BDS			(1 << SYSTEM_BDS)
#define PVT_CONFIG_USE_GAL			(1 << SYSTEM_GAL)
#define PVT_CONFIG_USE_KF			(0x100)
#define PVT_CONFIG_WEIGHTED_LSQ		(0x200)

typedef struct
{
	KINEMATIC_INFO PosVel;
	double GeoDistance, PsrPredict;		// Geometry distance and predicted PSR with satellite clock error compensation
	double SatDoppler;					// Doppler comes from satellite movement, in m/s
	double VectorX, VectorY, VectorZ;	// unit LOS vector in XYZ coordinate
	double el, az;
	int Time;
	unsigned char SatInfoFlag;	// bit0: 1: satellite position and velocity valid
								// bit1: 1: calculated by ephemeris, 0: calculated by almanac
								// bit2: 1: elevation and azimuth calculated
								// bit3: 1: elevation and azimuth using latest position
								// bit4: 1: ephemeris expired
								// bit5: 1: has valid predicted psr
	                            // bit6: 1: unhealth
	unsigned char FreqNumber;
	unsigned char HealthFlag;	// bit0~7:  healthy flag of ephemeris
								// bit8~15: healthy flag of almanac
	unsigned short CN0;
} SATELLITE_INFO, *PSATELLITE_INFO;
// definitions for SatInfoFlag field
#define SAT_INFO_POSVEL_VALID	0x01	// satellite position and velocity in structure is valid
#define SAT_INFO_BY_EPH			0x02	// satellite position and velocity calculated using ephemeris(1) or almanac(0)
#define SAT_INFO_ELAZ_VALID		0x04	// satellite elevation and azimuth in structure is valid
#define SAT_INFO_ELAZ_MATCH		0x08	// satellite elevation and azimuth match recent satellite position
#define SAT_INFO_EPH_EXPIRE		0x10	// satellite ephemeris has expired
#define SAT_INFO_PSR_VALID		0x20	// satellite predicted PSR, geometry distance and Doppler by satellite movement is valid
#define SAT_INFO_LOS_VALID		0x40	// satellite LOS vector is valid
#define SAT_INFO_LOS_MATCH		0x80	// satellite LOS vector match recent satellite position

typedef struct
{
    double	a0;  // 2**-30
    double	a1;  // 2**-27
    double	a2;  // 2**-24
    double	a3;  // 2**-24
    double	b0;  // 2**11
    double	b1;  // 2**14
    double	b2;  // 2**16
    double	b3;  // 2**16
    unsigned long	flag; // 1, availble   
} GPS_IONO_PARAM, *PGPS_IONO_PARAM;

typedef struct
{
    double	alpha1;  // 2**-3
    double	alpha2;  // 2**-3
    double	alpha3;  // 2**-3
    double	alpha4;  // 2**-3
    double	alpha5;  // -2**-3
    double	alpha6;  // 2**-3
    double	alpha7;  // 2**-3
    double	alpha8;  // 2**-3
    double	alpha9;  // 2**3
    unsigned long	flag; // 1, availble   
} BDS_IONO_PARAM, *PBDS_IONO_PARAM;

// UTC parameters
typedef  struct _PACKED_
{
	double	A0;  // second, 2**-30
	double	A1;  // second/second, 2**-50
	short	WN;
	short	WNLSF;
	unsigned char	tot; // 2**12
	unsigned char	TLS; // leap second
	unsigned char	TLSF;
	unsigned char	DN;
	unsigned long	flag; // 1, availble   
} UTC_PARAM, *PUTC_PARAM;

// H matrix used for PVT
typedef struct
{
	int length[PVT_MAX_SYSTEM_ID];		// number of 1s
	BOOL Is2D;							// whether is 2D positioning, this will give one extra equation
	double weight[DIMENSION_MAX_X];		// weight of the satellite, for weight LSQ
	double data[3][DIMENSION_MAX_X];	// H matrix value
} HMATRIX, *PHMATRIX;

// PVT core data for internal use
typedef struct
{
	PCHANNEL_STATUS		ChannelList[MAX_RAW_MSR_NUMBER];
	int					SystemSatCount[PVT_MAX_SYSTEM_ID];
	unsigned long long	PosUseSat[PVT_MAX_SYSTEM_ID];

	double StateVector[STATE_VECTOR_SIZE];		// [vx vy vz tdot x y z dt1 dt2 dt3]
	double PMatrix[P_MATRIX_SIZE];

	HMATRIX h;

	double PosInvMatrix[(PVT_MAX_SYSTEM_ID + 3) * (PVT_MAX_SYSTEM_ID + 4) / 2];	// Inv(HtH) for position
	double VelInvMatrix[10];	// Inv(HtH) for velocity
	double DOPInvMatrix[10];	// Inv(HtH) with all weight set to 1

	CONVERT_MATRIX ConvertMatrix;
} PVT_CORE_DATA, *PPVT_CORE_DATA;

#pragma pack(pop)	//restore original alignment

#endif //__DATA_TYPES_H__
