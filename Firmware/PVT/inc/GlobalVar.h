//----------------------------------------------------------------------
// GlobalVar.h:
//   Global variables declaration used by PVT
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __GLOBAL_VAR_H__
#define __GLOBAL_VAR_H__

#include "CommonDefines.h"

// channel status used by PVT
EXTERN CHANNEL_STATUS g_ChannelStatus[TOTAL_CHANNEL_NUMBER];
// Ephemeris, Almanac and satellite info
EXTERN GNSS_EPHEMERIS g_GpsEphemeris[TOTAL_GPS_SAT_NUMBER];
EXTERN MIDI_ALMANAC g_GpsAlmanac[TOTAL_GPS_SAT_NUMBER];
EXTERN SATELLITE_INFO g_GpsSatelliteInfo[TOTAL_GPS_SAT_NUMBER];
EXTERN GNSS_EPHEMERIS g_GalileoEphemeris[TOTAL_GAL_SAT_NUMBER];
EXTERN MIDI_ALMANAC g_GalileoAlmanac[TOTAL_GAL_SAT_NUMBER];
EXTERN SATELLITE_INFO g_GalileoSatelliteInfo[TOTAL_GAL_SAT_NUMBER];
EXTERN GNSS_EPHEMERIS g_BdsEphemeris[TOTAL_BDS_SAT_NUMBER];
EXTERN MIDI_ALMANAC g_BdsAlmanac[TOTAL_BDS_SAT_NUMBER];
EXTERN SATELLITE_INFO g_BdsSatelliteInfo[TOTAL_BDS_SAT_NUMBER];
// ionosphere and UTC parameters
EXTERN GPS_IONO_PARAM g_GpsIonoParam;
EXTERN BDS_IONO_PARAM g_BdsIonoParam;
EXTERN UTC_PARAM g_GpsUtcParam;
EXTERN UTC_PARAM g_BdsUtcParam;
// positioning result and internal data
EXTERN RECEIVER_INFO g_ReceiverInfo;
EXTERN PVT_CONFIG g_PvtConfig;
EXTERN PVT_CORE_DATA g_PvtCoreData;
EXTERN unsigned int g_GpsSatInView;
EXTERN unsigned long long g_GalileoSatInView;
EXTERN unsigned long long g_BdsSatInView;

#endif //__GLOBAL_VAR_H__
