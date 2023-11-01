//----------------------------------------------------------------------
// BdsFrame.c:
//   BDS frame sync and frame data decode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "PlatformCtrl.h"
#include "ChannelManager.h"
#include "DataTypes.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

// this is for one bit symbol decode, 8bit symbol LDPC decode need to used larger array
#define BUFFER_LENGTH 120
static U16 DataFrame[BUFFER_LENGTH*32];	// 6 WORDs holding 72bit subframe1, 75 WORDs holding subframe2, 33 WORDs holding subframe3

static void SetBit(U16 *FrameBuffer, int Bit, int BitPos);
static void BdsFrameDecode(PBDS_FRAME_INFO BdsFrameInfo, unsigned short *FrameBuffer);
static int DecodeBdsEphemeris(PGNSS_EPHEMERIS pEph, const unsigned int *FrameData);

//*************** BDS data decode initialization ****************
// Parameters:
//   none
// Return value:
//   none
void BdsDecodeInit()
{
	memset(DataFrame, 0, sizeof(DataFrame));
}

//*************** Task to decode navigation data ****************
// Parameters:
//   Param: Pointer to data stream structure
// Return value:
//   0
int BdsDecodeTask(void *Param)
{
	PDATA_STREAM DataStream = (PDATA_STREAM)Param;
	int ChannelIndex = DataStream->ChannelState->LogicChannel;	// get logic channel ID
	PBDS_FRAME_INFO BdsFrameInfo = (PBDS_FRAME_INFO)g_ChannelStatus[ChannelIndex].FrameInfo;
	U16 *FrameBuffer = DataFrame + ChannelIndex * BUFFER_LENGTH;
	int StartPos = 0;	// position of symbol in data stream to decode
	S8 Symbol;

	if (!(BdsFrameInfo->FrameFlag & 1))	// decode not yet started
	{
		if (DataStream->StartIndex == 0)
			BdsFrameInfo->FrameFlag |= 1;
		else if ((DataStream->StartIndex + DataStream->DataCount) > 1800)
		{
			BdsFrameInfo->FrameFlag |= 1;
			StartPos = 1800 - DataStream->StartIndex;
			DataStream->DataBuffer[StartPos/4] <<= ((StartPos % 4) * 8);
		}
		else
			return 0;
	}
	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "BDS stream:");
	while (StartPos < DataStream->DataCount)
	{
		// extra data symbol from data buffer
		Symbol = (S8)(DataStream->DataBuffer[StartPos/4] >> 24);
		DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), " %02x", Symbol & 0xff);
		DataStream->DataBuffer[StartPos/4] <<= 8;
		// set corresponding bit
		SetBit(FrameBuffer, ((Symbol & 0x80) ? 1 : 0), BdsFrameInfo->NavBitNumber);
		StartPos ++;
		if (++BdsFrameInfo->NavBitNumber >= 1800)
		{
			// decode frame
			BdsFrameDecode(BdsFrameInfo, FrameBuffer);
			// clear buffer
			memset(FrameBuffer, 0, sizeof(unsigned short) * BUFFER_LENGTH);
			BdsFrameInfo->NavBitNumber = 0;
		}
	}
	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "\n");

	return 0;
}

//*************** Set corresponding bit to do de-interleaving of B-CNAV1 ****************
// Parameters:
//   Param: Pointer to data stream structure
// Return value:
//   0
void SetBit(U16 *FrameBuffer, int Bit, int BitPos)
{
	int ColumnPos, RowPos, Segment;

	if (BitPos < 72)	// subframe1
	{
		FrameBuffer[BitPos/16] |= ((U16)Bit) << (15 - (BitPos & 0xf));
		return;
	}

	// subframe2/3, calculate position to set bit
	FrameBuffer += 6;	// skip 6WORD
	BitPos -= 72;
	ColumnPos = BitPos / 36;
	RowPos = BitPos - ColumnPos * 36;
	Segment = RowPos / 3;
	RowPos -= Segment * 3;
	if (RowPos == 0)	// subframe2 first row
		FrameBuffer += 6 * Segment;
	else if (RowPos == 1)	// subframe2 second row
		FrameBuffer += (6 * Segment + 3);
	else	// subframe3 or subframe2 last row
		FrameBuffer += ((Segment == 11) ? 72 : (75 + Segment * 3));

	// set bit in a row (each row is 3 WORDs)
	if (ColumnPos < 16)
		FrameBuffer[0] |= ((U16)Bit) << (15 - ColumnPos);
	else if (ColumnPos < 32)
		FrameBuffer[1] |= ((U16)Bit) << (31 - ColumnPos);
	else
		FrameBuffer[2] |= ((U16)Bit) << (47 - ColumnPos);
}

//*************** Do BDS B-CNAV1 LDPC decode ****************
// Parameters:
//   BdsFrameInfo: Pointer to BDS frame info structure
//   FrameBuffer: raw data symbols after de-interleaving
// Return value:
//   none
void BdsFrameDecode(PBDS_FRAME_INFO BdsFrameInfo, unsigned short *FrameBuffer)
{
	int i, svid, soh, week, how, type;
	unsigned int *FrameData;

	// decode subframe 1
	svid = (int)(FrameBuffer[0] >> 10);	// TODO: BCH decode
	soh = (int)((FrameBuffer[1] >> 3) & 0xff);	// TODO: BCH decode

	// TODO: LDPC decode
	FrameBuffer += 6;
	FrameData = BdsFrameInfo->SubFrame2Data;
	// TODO: CRC check
	// re-arrange to 32bit DWORD instead
	for (i = 0; i < 19; i ++)
		FrameData[i] = ((unsigned int)FrameBuffer[i*2] << 16) + FrameBuffer[i*2+1];

	week = GET_UBITS(FrameData[0], 19, 13);
	how = GET_UBITS(FrameData[0], 11, 8);
	type = GET_UBITS(FrameData[1], 12, 2);
	BdsFrameInfo->tow = how * 3600 + soh * 18 + 18;	// start of frame plus one whole frame length
	BdsFrameInfo->FrameFlag &= ~0xc;
	BdsFrameInfo->FrameFlag |= (type << 2);
	BdsFrameInfo->FrameFlag |= 2;	// indicate new frame data ready

	// if week number not valid, decode week number
	if ((g_ReceiverInfo.PosFlag & GPS_WEEK_VALID) == 0)
	{
		g_ReceiverInfo.WeekNumber = 1356 + week;
		g_ReceiverInfo.PosFlag |= (GPS_WEEK_VALID);// | CALC_INVIEW_GPS);
	}
}

//*************** BDS Frame process ****************
// Parameters:
//   pChannelStatus: pointer to channel status structure
// Return value:
//   none
void BdsFrameProc(PCHANNEL_STATUS pChannelStatus)
{
	PBDS_FRAME_INFO pFrameInfo = (PBDS_FRAME_INFO)(pChannelStatus->FrameInfo);
	int svid = pChannelStatus->svid;

	if (!(pFrameInfo->FrameFlag & 2))
		return;

	g_BdsEphemeris[svid - 1].svid = svid;
	DecodeBdsEphemeris(&g_BdsEphemeris[svid - 1], pFrameInfo->SubFrame2Data);
	pFrameInfo->FrameFlag &= ~2;
}

//*************** Decode BDS frame data to get ephemeris ****************
// Parameters:
//   pEph: pointer to ephemeris structure
//   FrameData: array of 600bit subframe2 stream
// Return value:
//   1 if decode success, otherwise 0
int DecodeBdsEphemeris(PGNSS_EPHEMERIS pEph, const unsigned int *FrameData)
{
	int idata;
	unsigned int type;
	long long ilong;
	unsigned long long ulong;

/*	pEph->health = GET_UBITS(WORD3, 8, 6);
	if (pEph->health != 0)
	{
		pEph->flag = 0;
		return 0;
	}*/
	pEph->health = 0;
	pEph->flag = 1;

	pEph->week = GET_UBITS(FrameData[0], 19, 13);
	pEph->iodc = GET_UBITS(FrameData[0], 1, 10);
	pEph->iode2 = GET_UBITS(FrameData[1], 25, 7) | ((FrameData[0] & 1) ? 128 : 0);
	//	pEph->ura = GET_UBITS(WORD3, 14, 4);

	// Ephemeris I:
	type = GET_UBITS(FrameData[1], 12, 2);
	pEph->toe = (int)GET_UBITS(FrameData[1], 14, 11) * 300;
	idata = (GET_BITS(FrameData[1], 0, 12) << 14) | GET_UBITS(FrameData[2], 18, 14);
	pEph->axis = ((type == 3) ? 27906100.0 : 42162200.0) + ScaleDouble(idata, 9);	// major-axis
	idata = (GET_BITS(FrameData[2], 0, 18) << 7) | GET_UBITS(FrameData[3], 25, 7);
	pEph->axis_dot = ScaleDouble(idata, 21);
	pEph->delta_n = ScaleDouble(GET_BITS(FrameData[3], 8, 17), 44) * PI; // .44 * PI
	idata = (GET_BITS(FrameData[3], 0, 8) << 15) | GET_UBITS(FrameData[4], 15, 17);
//	pEph->delta_n_dot = ScaleDouble(idata, 58) * PI;	// .57 * PI * 1/2
	ilong = GET_BITS(FrameData[4], 0, 17);
	ilong  = (ilong << 16) | GET_UBITS(FrameData[5], 16, 16);
	pEph->M0 = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI
	ulong = GET_UBITS(FrameData[5], 0, 16);
	ulong  = (ulong << 17) | GET_UBITS(FrameData[6], 15, 17);
	pEph->ecc = ScaleDoubleULong(ulong, 34);	// .34
	ilong = GET_BITS(FrameData[6], 0, 15);
	ilong  = (ilong << 18) | GET_UBITS(FrameData[7], 14, 18);
	pEph->w = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI

	// Ephemeris II:
	ilong = GET_BITS(FrameData[7], 0, 14);
	ilong  = (ilong << 19) | GET_UBITS(FrameData[8], 13, 19);
	pEph->omega0 = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI
	ilong = GET_BITS(FrameData[8], 0, 13);
	ilong  = (ilong << 20) | GET_UBITS(FrameData[9], 12, 20);
	pEph->i0 = ScaleDoubleLong(ilong, 32) * PI;	// .32 * PI
	idata = (GET_BITS(FrameData[9], 0, 12) << 7) | GET_UBITS(FrameData[10], 25, 7);
	pEph->omega_dot = ScaleDouble(idata, 44) * PI;	// .44 * PI
	pEph->idot = ScaleDouble(GET_BITS(FrameData[10], 10, 15), 44) * PI; // .44 * PI
	idata = (GET_BITS(FrameData[10], 0, 10) << 6) | GET_UBITS(FrameData[11], 26, 6);
	pEph->cis = ScaleDouble(idata, 30);	// .30
	pEph->cic = ScaleDouble(GET_BITS(FrameData[11], 10, 16), 30); // .30
	idata = (GET_BITS(FrameData[11], 0, 10) << 14) | GET_UBITS(FrameData[12], 18, 14);
	pEph->crs = ScaleDouble(idata, 8);	// .8
	idata = (GET_BITS(FrameData[12], 0, 18) << 6) | GET_UBITS(FrameData[13], 26, 6);
	pEph->crc = ScaleDouble(idata, 8);	// .8
	pEph->cus = ScaleDouble(GET_BITS(FrameData[13], 5, 21), 30); // .30
	idata = (GET_BITS(FrameData[13], 0, 5) << 16) | GET_UBITS(FrameData[14], 16, 16);
	pEph->cuc = ScaleDouble(idata, 30);	// .30


	// clock
	pEph->tgd = ScaleDouble(GET_BITS(FrameData[17], 7, 12), 34); // .34
	pEph->toc = (int)GET_UBITS(FrameData[14], 5, 11) * 300;
	idata = (GET_BITS(FrameData[14], 0, 5) << 20) | GET_UBITS(FrameData[15], 12, 20);
	pEph->af0 = ScaleDouble(idata, 34); // .34
	idata = (GET_BITS(FrameData[15], 0, 12) << 10) | GET_UBITS(FrameData[16], 22, 10);
	pEph->af1 = ScaleDouble(idata, 50); // .50
	pEph->af2 = ScaleDouble(GET_BITS(FrameData[16], 11, 11), 66); // .66

	// calculate derived variables
	pEph->sqrtA = sqrt(pEph->axis);
	pEph->n = WGS_SQRT_GM / (pEph->sqrtA * pEph->axis) + pEph->delta_n;
	pEph->root_ecc = sqrt(1.0 - pEph->ecc * pEph->ecc);
	pEph->omega_t = pEph->omega0 - WGS_OMEGDOTE * pEph->toe;
	pEph->omega_delta = pEph->omega_dot - WGS_OMEGDOTE;
	return 1;
}
