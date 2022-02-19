//----------------------------------------------------------------------
// PvtConst.h:
//   PVT constant value definitions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __PVT_CONST_H__
#define __PVT_CONST_H__

#define PI				3.1415926535897932

#define WGS_AXIS_A		6378137.0				// A - WGS-84 earth's semi major axis
#define	WGS_AXIS_B		6356752.3142451795		// B - WGS-84 earth's semi minor axis
#define WGS_E1_SQR		0.006694379990141317	// (A/B)^2-1, 1st numerical eccentricity
#define WGS_E2_SQR		0.006739496742276435	// 1-(B/A)^2, 2nd numerical eccentricity
#define WGS_SQRT_GM		19964981.8432173887		// square root of GM
#define WGS_OMEGDOTE	7.2921151467e-5			// earth rotate rate
#define WGS_F_GTR		-4.442807633e-10		// factor of general theory of relativity

#define CGS2000_SQRT_GM		19964980.3856652962	// square root of GM
#define CGS2000_OMEGDOTE	7.292115e-5			// earth rotate rate

#define LIGHT_SPEED		299792458.0				// speed of light
#define GPS_L1_WAVELENGTH 0.19029367279836488
#define LIGHT_SPEED_MS (LIGHT_SPEED * 0.001)		// distance light travels within 1ms

#define PVT_MAX_SYSTEM_ID 3						// max system used in PVT
#define MAX_RAW_MSR_NUMBER 32					// maximum total raw measurement number
#define DIMENSION_MAX_X MAX_RAW_MSR_NUMBER
#define DIMENSION_MAX_Y 3
#define STATE_VECTOR_SIZE (7 + PVT_MAX_SYSTEM_ID)	// 3 position, 3 velocity, 1 clock drift plus clock error
#define P_MATRIX_SIZE (STATE_VECTOR_SIZE * (STATE_VECTOR_SIZE + 1) / 2)

#define MAX_GPS_TOW		100799
#define MAX_BDS_TOW		604799

// universal SatID for GNSS systems
#define TOTAL_GPS_SAT_NUMBER 32
#define TOTAL_GAL_SAT_NUMBER 50
#define TOTAL_BDS_SAT_NUMBER 63

#define MIN_GPS_SAT_ID 1
#define MAX_GPS_SAT_ID (MIN_GPS_SAT_ID + TOTAL_GPS_SAT_NUMBER - 1)
#define MIN_GAL_SAT_ID 97
#define MAX_GAL_SAT_ID (MIN_GAL_SAT_ID + TOTAL_GAL_SAT_NUMBER - 1)
#define MIN_BDS_SAT_ID 161
#define MAX_BDS_SAT_ID (MIN_BDS_SAT_ID + TOTAL_BDS_SAT_NUMBER - 1)

#define GET_SAT_ID(FreqID, svid) ((FreqID == FREQ_E1) ? (MIN_GAL_SAT_ID + (svid) - 1) : ((FreqID == FREQ_B1C) ? (MIN_BDS_SAT_ID + (svid) - 1) : (svid)))

#endif//__PVT_CONST_H__
