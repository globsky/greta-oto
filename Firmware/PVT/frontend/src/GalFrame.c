//----------------------------------------------------------------------
// GalFrame.c:
//   Galileo I/NAV frame sync and frame data decode
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "PlatformCtrl.h"
#include "DataTypes.h"
#include "TaskManager.h"
#include "GlobalVar.h"
#include "SupportPackage.h"

#define PAYLOAD_LENGTH 4	// 128bit page contents
#define PACKAGE_LENGTH (sizeof(SYMBOL_PACKAGE) + sizeof(unsigned int)*(PAYLOAD_LENGTH))	// 3 variables + 4 payload

extern U32 EphAlmMutex;

static int GalPageProc(PFRAME_INFO GalFrameInfo, PDATA_FOR_DECODE DataForDecode);
static int INavPageDecode(void* Param);
static int DecodeGalileoEphemeris(int svid, const unsigned int data[20]);

// for Viterbi decode
static int Distance[64], DistanceNew[64];
static unsigned long long Trace[64], TraceNew[64];
static int GalViterbiDecode(unsigned int SymbolBuffer[30], unsigned int DecodeResult[4]);
static void ViterbiDecodePair(unsigned int SymbolPair);
static void MergeBranches(const int StateArray[8], int DistanceSum);
static int FindMinIndex();
// for CRC24Q check
static unsigned int Crc24qEncode(unsigned int *BitStream, int Length);
static const unsigned int Crc24QTable[256] = {
	0x00000000u, 0x01864CFBu, 0x028AD50Du, 0x030C99F6u, 0x0493E6E1u, 0x0515AA1Au, 0x061933ECu, 0x079F7F17u,
	0x08A18139u, 0x0927CDC2u, 0x0A2B5434u, 0x0BAD18CFu, 0x0C3267D8u, 0x0DB42B23u, 0x0EB8B2D5u, 0x0F3EFE2Eu,
	0x10C54E89u, 0x11430272u, 0x124F9B84u, 0x13C9D77Fu, 0x1456A868u, 0x15D0E493u, 0x16DC7D65u, 0x175A319Eu,
	0x1864CFB0u, 0x19E2834Bu, 0x1AEE1ABDu, 0x1B685646u, 0x1CF72951u, 0x1D7165AAu, 0x1E7DFC5Cu, 0x1FFBB0A7u,
	0x200CD1E9u, 0x218A9D12u, 0x228604E4u, 0x2300481Fu, 0x249F3708u, 0x25197BF3u, 0x2615E205u, 0x2793AEFEu,
	0x28AD50D0u, 0x292B1C2Bu, 0x2A2785DDu, 0x2BA1C926u, 0x2C3EB631u, 0x2DB8FACAu, 0x2EB4633Cu, 0x2F322FC7u,
	0x30C99F60u, 0x314FD39Bu, 0x32434A6Du, 0x33C50696u, 0x345A7981u, 0x35DC357Au, 0x36D0AC8Cu, 0x3756E077u,
	0x38681E59u, 0x39EE52A2u, 0x3AE2CB54u, 0x3B6487AFu, 0x3CFBF8B8u, 0x3D7DB443u, 0x3E712DB5u, 0x3FF7614Eu,
	0x4019A3D2u, 0x419FEF29u, 0x429376DFu, 0x43153A24u, 0x448A4533u, 0x450C09C8u, 0x4600903Eu, 0x4786DCC5u,
	0x48B822EBu, 0x493E6E10u, 0x4A32F7E6u, 0x4BB4BB1Du, 0x4C2BC40Au, 0x4DAD88F1u, 0x4EA11107u, 0x4F275DFCu,
	0x50DCED5Bu, 0x515AA1A0u, 0x52563856u, 0x53D074ADu, 0x544F0BBAu, 0x55C94741u, 0x56C5DEB7u, 0x5743924Cu,
	0x587D6C62u, 0x59FB2099u, 0x5AF7B96Fu, 0x5B71F594u, 0x5CEE8A83u, 0x5D68C678u, 0x5E645F8Eu, 0x5FE21375u,
	0x6015723Bu, 0x61933EC0u, 0x629FA736u, 0x6319EBCDu, 0x648694DAu, 0x6500D821u, 0x660C41D7u, 0x678A0D2Cu,
	0x68B4F302u, 0x6932BFF9u, 0x6A3E260Fu, 0x6BB86AF4u, 0x6C2715E3u, 0x6DA15918u, 0x6EADC0EEu, 0x6F2B8C15u,
	0x70D03CB2u, 0x71567049u, 0x725AE9BFu, 0x73DCA544u, 0x7443DA53u, 0x75C596A8u, 0x76C90F5Eu, 0x774F43A5u,
	0x7871BD8Bu, 0x79F7F170u, 0x7AFB6886u, 0x7B7D247Du, 0x7CE25B6Au, 0x7D641791u, 0x7E688E67u, 0x7FEEC29Cu,
	0x803347A4u, 0x81B50B5Fu, 0x82B992A9u, 0x833FDE52u, 0x84A0A145u, 0x8526EDBEu, 0x862A7448u, 0x87AC38B3u,
	0x8892C69Du, 0x89148A66u, 0x8A181390u, 0x8B9E5F6Bu, 0x8C01207Cu, 0x8D876C87u, 0x8E8BF571u, 0x8F0DB98Au,
	0x90F6092Du, 0x917045D6u, 0x927CDC20u, 0x93FA90DBu, 0x9465EFCCu, 0x95E3A337u, 0x96EF3AC1u, 0x9769763Au,
	0x98578814u, 0x99D1C4EFu, 0x9ADD5D19u, 0x9B5B11E2u, 0x9CC46EF5u, 0x9D42220Eu, 0x9E4EBBF8u, 0x9FC8F703u,
	0xA03F964Du, 0xA1B9DAB6u, 0xA2B54340u, 0xA3330FBBu, 0xA4AC70ACu, 0xA52A3C57u, 0xA626A5A1u, 0xA7A0E95Au,
	0xA89E1774u, 0xA9185B8Fu, 0xAA14C279u, 0xAB928E82u, 0xAC0DF195u, 0xAD8BBD6Eu, 0xAE872498u, 0xAF016863u,
	0xB0FAD8C4u, 0xB17C943Fu, 0xB2700DC9u, 0xB3F64132u, 0xB4693E25u, 0xB5EF72DEu, 0xB6E3EB28u, 0xB765A7D3u,
	0xB85B59FDu, 0xB9DD1506u, 0xBAD18CF0u, 0xBB57C00Bu, 0xBCC8BF1Cu, 0xBD4EF3E7u, 0xBE426A11u, 0xBFC426EAu,
	0xC02AE476u, 0xC1ACA88Du, 0xC2A0317Bu, 0xC3267D80u, 0xC4B90297u, 0xC53F4E6Cu, 0xC633D79Au, 0xC7B59B61u,
	0xC88B654Fu, 0xC90D29B4u, 0xCA01B042u, 0xCB87FCB9u, 0xCC1883AEu, 0xCD9ECF55u, 0xCE9256A3u, 0xCF141A58u,
	0xD0EFAAFFu, 0xD169E604u, 0xD2657FF2u, 0xD3E33309u, 0xD47C4C1Eu, 0xD5FA00E5u, 0xD6F69913u, 0xD770D5E8u,
	0xD84E2BC6u, 0xD9C8673Du, 0xDAC4FECBu, 0xDB42B230u, 0xDCDDCD27u, 0xDD5B81DCu, 0xDE57182Au, 0xDFD154D1u,
	0xE026359Fu, 0xE1A07964u, 0xE2ACE092u, 0xE32AAC69u, 0xE4B5D37Eu, 0xE5339F85u, 0xE63F0673u, 0xE7B94A88u,
	0xE887B4A6u, 0xE901F85Du, 0xEA0D61ABu, 0xEB8B2D50u, 0xEC145247u, 0xED921EBCu, 0xEE9E874Au, 0xEF18CBB1u,
	0xF0E37B16u, 0xF16537EDu, 0xF269AE1Bu, 0xF3EFE2E0u, 0xF4709DF7u, 0xF5F6D10Cu, 0xF6FA48FAu, 0xF77C0401u,
	0xF842FA2Fu, 0xF9C4B6D4u, 0xFAC82F22u, 0xFB4E63D9u, 0xFCD11CCEu, 0xFD575035u, 0xFE5BC9C3u, 0xFFDD8538u,
};

//*************** Galileo navigation data process ****************
//* Do Galileo navigation data process and page sync
//* meaning of FrameStatus:
//* -1: frame sync not completed
//* >=0: bit0 for even page valid, bit 1 for check sync pattern again (confirm page sync correct)
// Parameters:
//   pFrameInfo: pointer to frame info structure
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current millisecond count within current week (negative if unknown)
int GalNavDataProc(PFRAME_INFO pFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	int data_count = 8;	// DataStream contais 8 4bit data
	int SymbolCount = -1;
	U32 DataStream = DataForDecode->DataStream;
	unsigned int *pBuffer;
	int PosIndex;
	pFrameInfo->TimeTag = -1;	// reset decoded week number to invalid

	// meaning of FrameState:
	// -1: frame sync not completed
	// >=0: bit 0 reserved, bit 1 for check sync pattern again
	// if not frame sync, find sync pattern 0101100000 first
	if (pFrameInfo->FrameStatus < 0)
	{
		while (data_count > 0)
		{
			// move in sign bit of the symbol
			pFrameInfo->SymbolData[0] = (pFrameInfo->SymbolData[0] << 1) | (DataStream >> 31);
			DataStream <<= 4;
			data_count --;
			pFrameInfo->SymbolNumber ++;
			if (pFrameInfo->SymbolNumber >= 10)
			{
				if ((pFrameInfo->SymbolData[0] & 0x3ff) == 0x160)	// sync pattern match
				{
					if (((DataForDecode->StartIndex + 8 - data_count) % 25) == 10)	// check sync pattern align to NH boundary, add 25 to avoid negative value
					{
						// sync pattern found, change frame status to 2
						pFrameInfo->FrameStatus = 2;
						// force symbol number to 0
						pFrameInfo->SymbolNumber = 0;
						break;
					}
				}
				pFrameInfo->SymbolNumber = 9;	// discard oldest symbol
			}
		}
	}
	// put data in FrameData
	if (pFrameInfo->FrameStatus >= 0)
	{
		pBuffer = &(pFrameInfo->FrameData[0]);
		while (data_count > 0)
		{
			if (pFrameInfo->SymbolNumber < 0)	// sync pattern
			{
				pFrameInfo->SymbolData[0] = (pFrameInfo->SymbolData[0] << 1) | (DataStream >> 31);
				if ((pFrameInfo->FrameStatus & 2) && pFrameInfo->SymbolNumber == -1)	// need to double check sync pattern
				{
					pFrameInfo->SymbolData[0] ^= 0x160;
					PosIndex = __builtin_popcount(pFrameInfo->SymbolData[0] & 0x3ff);	// count number of symbols that sign does not match sync pattern
					if (PosIndex == 0)	// sync pattern double check passed
						pFrameInfo->FrameStatus &= ~2;	// clear check sync pattern flag
					else if (PosIndex >= 2)	// sync pattern double check failed
					{
						pFrameInfo->FrameStatus = -1;	// need to search sync pattern again
						break;
					}
					// otherwise wait to next sync pattern and check again
				}
			}
			else
			{
				PosIndex = (pFrameInfo->SymbolNumber & 7) * 4;
				pBuffer[pFrameInfo->SymbolNumber / 8] &= ~(0xf0000000 >> PosIndex);	// clear 4bit to put symbol
				pBuffer[pFrameInfo->SymbolNumber / 8] |= ((DataStream & 0xf0000000) >> PosIndex);	// put symbol (in 4MSB of DataStream) in
			}
			DataStream <<= 4;
			data_count --;
			pFrameInfo->SymbolNumber ++;
			if (pFrameInfo->SymbolNumber == 240)	// one page completed
			{
				// do Viterbi decode on page data
				GalViterbiDecode(pBuffer, pFrameInfo->FrameData);
				pFrameInfo->SymbolNumber = -10;	// skip first 10 symbols for next page as sync pattern
				SymbolCount = GalPageProc(pFrameInfo, DataForDecode) + data_count;
			}
		}
	}

	return SymbolCount * 4;
}

//*************** Do Galileo page part process ****************
// Parameters:
//   GalFrameInfo: pointer to frame info structure
//   PageData: 
//   DataForDecode: pointer to structure of symbols to be decoded
// Return value:
//   current symbols within week, <0 if unknown
int GalPageProc(PFRAME_INFO GalFrameInfo, PDATA_FOR_DECODE DataForDecode)
{
	unsigned int *PageData = GalFrameInfo->FrameData;
	unsigned int crc;
	unsigned int Package[PACKAGE_LENGTH];
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Package;
	int WordType;
	int tow = -1, wn = -1;

	// after decode, MSB of first DWORD is even/odd flag
	// each page part has 120 valid bit, leaving 8LSB of last DWORD unused
 	if (PageData[0] & 0x80000000)	// odd page
	{
		if (GalFrameInfo->FrameStatus & 1)	// even page valid
		{
			// put decoded odd page part (remove 6bit tail) into GalFrameInfo->SymbolData
			// starting bit17 of GalFrameInfo->SymbolData[5] until bit0 of GalFrameInfo->SymbolData[7] (totally 82bits)
			GalFrameInfo->SymbolData[5] |= PageData[0] >> 14;
			GalFrameInfo->SymbolData[6] = (PageData[0] << 18) | (PageData[1] >> 14);
			GalFrameInfo->SymbolData[7] = (PageData[1] << 18) | ((PageData[2] >> 6) & 0x3ffff);
			// extract 24bit CRC
			crc = ((PageData[2] << 18) & 0xfc0000) | (PageData[3] >> 14);
			GalFrameInfo->FrameStatus &= ~1;	// clear even page valid flag
			if (Crc24qEncode(&GalFrameInfo->SymbolData[1], 196) != crc)	// CRC check fail
				return -1;
			// send whole nominal page to decode
			SymbolPackage->ChannelState = DataForDecode->ChannelState;
			SymbolPackage->FrameInfo = GalFrameInfo;	// put pointer of FrameInfo in FrameIndex
			SymbolPackage->PayloadLength = 4;
			SymbolPackage->Symbols[0] = (GalFrameInfo->SymbolData[1] << 30) | (GalFrameInfo->SymbolData[2] >> 2);
			SymbolPackage->Symbols[1] = (GalFrameInfo->SymbolData[2] << 30) | (GalFrameInfo->SymbolData[3] >> 2);
			SymbolPackage->Symbols[2] = (GalFrameInfo->SymbolData[3] << 30) | (GalFrameInfo->SymbolData[4] >> 2);
			SymbolPackage->Symbols[3] = (GalFrameInfo->SymbolData[4] << 30) | ((GalFrameInfo->SymbolData[5] >> 2) & 0x3fff0000);
			SymbolPackage->Symbols[3] |= (GalFrameInfo->SymbolData[5] & 0xffff);
			AddToTask(TASK_POSTMEAS, INavPageDecode, SymbolPackage, PACKAGE_LENGTH);
			// decode week number and week millisecond
			WordType = SymbolPackage->Symbols[0] >> 26;
			if (WordType == 0)
			{
				tow = (SymbolPackage->Symbols[3] & 0xfffff) + 2;	// plus current decoded page
				wn = (SymbolPackage->Symbols[3] >> 20) & 0xfff;
			}
			else if (WordType == 5)
			{
				tow = (((SymbolPackage->Symbols[2] & 0x7ff) << 9) | (SymbolPackage->Symbols[3] >> 23)) + 2;	// plus current decoded page
				wn = (SymbolPackage->Symbols[2] >> 11) & 0xfff;
			}
			else if (WordType == 6)
			{
				tow = ((SymbolPackage->Symbols[3] >> 3) & 0xfffff) + 2;	// plus current decoded page
			}
			GalFrameInfo->TimeTag = wn;	// set decoded week number to TimeTag
		}
	}
	else	// even page
	{
		// put decoded even page part (remove 6bit tail) into GalFrameInfo->SymbolData
		// starting bit3 of GalFrameInfo->SymbolData[1] until bit18 of GalFrameInfo->SymbolData[5] (totally 114bits)
		GalFrameInfo->SymbolData[1] = PageData[0] >> 28;
		GalFrameInfo->SymbolData[2] = (PageData[0] << 4) | (PageData[1] >> 28);
		GalFrameInfo->SymbolData[3] = (PageData[1] << 4) | ((PageData[2] >> 20) & 0xf);
		GalFrameInfo->SymbolData[4] = (PageData[2] << 12) | (PageData[3] >> 20);
		GalFrameInfo->SymbolData[5] = (PageData[3] << 12) & 0xfffc0000;
		GalFrameInfo->FrameStatus |= 1;	// set even page valid flag
	}

	return tow * 250;	// 250 symbols per second
}

//*************** Task function to process one Galileo nominal page ****************
// Parameters:
//   Param: pointer to structure holding one page data
// Return value:
//   0
int INavPageDecode(void* Param)
{
	PSYMBOL_PACKAGE SymbolPackage = (PSYMBOL_PACKAGE)Param;
	int Svid = SymbolPackage->ChannelState->Svid;
	PFRAME_INFO FrameInfo = SymbolPackage->FrameInfo;
	int WordType = SymbolPackage->Symbols[0] >> 26;

	if (WordType >= 1 && WordType <= 5)	// Ephemeris/SISA/Clock/Iono/BGD
	{
		FrameInfo->FrameData[WordType * 4 + 26] = SymbolPackage->Symbols[0];
		FrameInfo->FrameData[WordType * 4 + 27] = SymbolPackage->Symbols[1];
		FrameInfo->FrameData[WordType * 4 + 28] = SymbolPackage->Symbols[2];
		FrameInfo->FrameData[WordType * 4 + 29] = SymbolPackage->Symbols[3];
		FrameInfo->FrameFlag |= (1 << (WordType - 1));
	}
	else if (WordType >= 7 && WordType <= 10)	// Almanac
	{
		// TODO: decode E1 almanac
	}

	if ((FrameInfo->FrameFlag & 0x1f) == 0x1f)	// bit0~4 set, WordType 1~5 completed
	{
		DecodeGalileoEphemeris(Svid, FrameInfo->FrameData + 30);
		FrameInfo->FrameFlag &= ~0x1f;
	}

	return 0;
}

int DecodeGalileoEphemeris(int svid, const unsigned int data[20])
{
	PGNSS_EPHEMERIS pEph = &g_GalileoEphemeris[svid-1];
	unsigned int iod;

	// check IOD identical and svid matches source signal
	if (svid != GET_UBITS(data[12], 10, 6))
		return 0;
	iod = GET_UBITS(data[12], 16, 10);
	if (pEph->flag == 1 && pEph->iodc == iod)	// do not decode repeat ephemeris
		return 1;
	if (iod != GET_UBITS(data[0], 16, 10) || iod != GET_UBITS(data[4], 16, 10) || iod != GET_UBITS(data[8], 16, 10))
		return 0;

	DEBUG_OUTPUT(OUTPUT_CONTROL(DATA_DECODE, INFO), "Decode ephemeris of E%02d\n", svid);
	MutexTake(EphAlmMutex);

	pEph->svid = svid - 1 + MIN_GAL_SAT_ID;
	pEph->health = 0;
	pEph->flag = 1;
	pEph->iodc = (unsigned short)iod;

	pEph->toe = GET_UBITS(data[0], 2, 14) * 60;
	pEph->M0 = ScaleDouble(((data[0] << 30) & 0xc0000000) | GET_UBITS(data[1], 2, 30), 31) * PI;
	pEph->ecc = ScaleDoubleU(((data[1] << 30) & 0xc0000000) | GET_UBITS(data[2], 2, 30), 33);
	pEph->sqrtA = ScaleDoubleU(((data[2] << 30) & 0xc0000000) | GET_UBITS(data[3], 2, 30), 19);
	pEph->omega0 = ScaleDouble(((data[4] << 16) & 0xffff0000) | GET_UBITS(data[5], 16, 16), 31) * PI;
	pEph->i0 = ScaleDouble(((data[5] << 16) & 0xffff0000) | GET_UBITS(data[6], 16, 16), 31) * PI;
	pEph->w = ScaleDouble(((data[6] << 16) & 0xffff0000) | GET_UBITS(data[7], 16, 16), 31) * PI;
	pEph->idot = ScaleDouble(GET_BITS(data[7], 2, 14), 43) * PI;
	pEph->omega_dot = ScaleDouble((GET_BITS(data[8], 0, 16) << 8) | GET_UBITS(data[9], 24, 8), 43) * PI;
	pEph->delta_n = ScaleDouble(GET_BITS(data[9], 8, 16), 43) * PI;
	pEph->cuc = ScaleDouble((GET_BITS(data[9], 0, 8) << 8) | GET_UBITS(data[10], 24, 8), 29);
	pEph->cus = ScaleDouble(GET_BITS(data[10], 8, 16), 29);
	pEph->crc = ScaleDouble((GET_BITS(data[10], 0, 8) << 8) | GET_UBITS(data[11], 24, 8), 5);
	pEph->crs = ScaleDouble(GET_BITS(data[11], 8, 16), 5);
	pEph->cic = ScaleDouble((GET_BITS(data[12], 0, 10) << 6) | GET_UBITS(data[13], 26, 6), 29);
	pEph->cis = ScaleDouble(GET_BITS(data[13], 10, 16), 29);
	pEph->toc = (((data[13] & 0x3ff) << 4) | GET_UBITS(data[14], 28, 4)) * 60;
	pEph->af0 = ScaleDouble((GET_BITS(data[14], 0, 28) << 3) | GET_UBITS(data[15], 29, 3), 34);
	pEph->af1 = ScaleDouble(GET_BITS(data[15], 8, 21), 46);
	pEph->af2 = ScaleDouble(GET_BITS(data[15], 2, 6), 59);
	pEph->tgd = ScaleDouble(GET_BITS(data[17], 7, 10), 32);
	pEph->tgd2 = ScaleDouble((GET_BITS(data[17], 0, 7) << 3) | GET_UBITS(data[18], 29, 3), 32);
	pEph->week = GET_UBITS(data[18], 11, 12);

	// calculate derived variables
	pEph->axis = pEph->sqrtA * pEph->sqrtA;
	pEph->n = WGS_SQRT_GM / (pEph->sqrtA * pEph->axis) + pEph->delta_n;
	pEph->root_ecc = sqrt(1.0 - pEph->ecc * pEph->ecc);
	pEph->omega_t = pEph->omega0 - WGS_OMEGDOTE * pEph->toe;
	pEph->omega_delta = pEph->omega_dot - WGS_OMEGDOTE;

	MutexGive(EphAlmMutex);
	return 1;
}

//*************** Galileo Viterbi decode for one page ****************
//* assume input symbol is 4bit, totally 240 symbols placed from MSB with interleaving
// Parameters:
//   SymbolBuffer: array of input symbols
//   DecodeResult: decoded result, 120bits MSB first (8LSB of DecodeResult[1] and 8MSB of DecodeResult[2] will overlap)
// Return value:
//   minimum distance
int GalViterbiDecode(unsigned int SymbolBuffer[30], unsigned int DecodeResult[4])
{
	unsigned int Symbols[30], DataWord = SymbolBuffer[0];	// store deinterleaved symbols
	int i, j, index = 0;
	int TotalMinDistance, MinState;

	memset(Distance, 1, sizeof(Distance));
	Distance[0] = 0;	// set Distance a big value except index 0 to ensure start state is 0
	// deinterleaving
	for (i = 0; i < 8; i ++)
	{
		for (j = 0; j < 30; j ++)
		{
			Symbols[j] = (Symbols[j] << 4) | (DataWord >> 28);
			DataWord <<= 4;
			if (((++index) & 7) == 0)
				DataWord = SymbolBuffer[index >> 3];
		}
	}

	for (i = 0; i < 30 * 4; i ++)
	{
		ViterbiDecodePair(Symbols[i>>2] >> (24 - (i & 3)*8));
		if (i >= 63 && ((i & 7) == 7))	// to reduce the number of comparision, do it every 8 bits
		{
			MinState = FindMinIndex();
			DecodeResult[(i - 56) / 32] = (DecodeResult[(i - 56) / 32] << 8) | ((unsigned int)(Trace[MinState] >> 56));
		}
	}
	MinState = FindMinIndex();
	DecodeResult[2] = (unsigned int)(Trace[MinState] >> 32);
	DecodeResult[3] = (unsigned int)(Trace[MinState]);
	TotalMinDistance = Distance[MinState];

	return 0;
}
static const int OutputTable[4][8] = {
	{ 3, 5, 11, 13, 16, 22, 24, 30, },	// output 00 if input 0
	{ 0, 6,  8, 14, 19, 21, 27, 29, },	// output 01 if input 0
	{ 2, 4, 10, 12, 17, 23, 25, 31, },	// output 10 if input 0
	{ 1, 7,  9, 15, 18, 20, 26, 28, },	// output 11 if input 0
};

//*************** Viterbi decoder to decode one pair of symbols ****************
//* assume input symbol is 4bit, first symbol in bit7~4, second symbol in bit3~0
// Parameters:
//   SymbolPair: input symbols
// Return value:
//   Minimum increased distance
void ViterbiDecodePair(unsigned int SymbolPair)
{
	int DistanceSum;
	int Symbol1, Symbol2;

	Symbol1 = (int)((SymbolPair >> 4) & 0xf);
	Symbol2 = (int)(SymbolPair & 0xf);
	DistanceSum = (Symbol1 ^ 0x7) + (Symbol2 ^ 0x7);	// distance for output 00
	MergeBranches(OutputTable[0], DistanceSum);
	MergeBranches(OutputTable[3], 30 - DistanceSum);
	DistanceSum = (Symbol1 ^ 0x7) + (Symbol2 ^ 0x8);	// distance for output 01
	MergeBranches(OutputTable[1], DistanceSum);
	MergeBranches(OutputTable[2], 30 - DistanceSum);

//	printf("Decode one pair completed with MinState=%d\n", MinState);
	// copy back distances and trace
	memcpy(Distance, DistanceNew, sizeof(Distance));
	memcpy(Trace, TraceNew, sizeof(Trace));
//	for (state = 0; state < 64; state ++)
//		printf("Distance = %8d Trace = %08x%08x\n", Distance[state], (unsigned int)(Trace[state] >> 32), (unsigned int)(Trace[state] & 0xffffffff));
}

//*************** Viterbi decoder to decode one pair of symbols ****************
//* assume input symbol is 4bit, first symbol in bit7~4, second symbol in bit3~0
// Parameters:
//   SymbolPair: input symbols
// Return value:
//   none
void MergeBranches(const int StateArray[8], int DistanceSum)
{
	int Distance00, Distance01, Distance10, Distance11;
	int DistanceCmp = 30 - DistanceSum;	// complement distance
	int i, state;

	for (i = 0; i < 8; i ++)
	{
		state = StateArray[i];
		Distance00 = Distance[state] + DistanceSum;	// distance for state with input 0
		Distance01 = Distance[state] + DistanceCmp;	// distance for state with input 1
		Distance10 = Distance[state+32] + DistanceCmp;	// distance for state+32 with input 0
		Distance11 = Distance[state+32] + DistanceSum;	// distance for state+32 with input 1

		// merge branch to state*2 by comparing Distance00 and Distance10, last bit of trace is 0
		if (Distance00 <= Distance10)	// from first branch (state)
		{
			TraceNew[state*2] = Trace[state] << 1;
			DistanceNew[state*2] = Distance00;
		}
		else	// from first branch (state+32)
		{
			TraceNew[state*2] = Trace[state+32] << 1;
			DistanceNew[state*2] = Distance10;
		}

		// merge branch to state*2+1 by comparing Distance01 and Distance11, last bit of trace is 1
		if (Distance01 <= Distance11)	// from first branch (state)
		{
			TraceNew[state*2+1] = (Trace[state] << 1) + 1;
			DistanceNew[state*2+1] = Distance01;
		}
		else	// from first branch (state+32)
		{
			TraceNew[state*2+1] = (Trace[state+32] << 1) + 1;
			DistanceNew[state*2+1] = Distance11;
		}
	}
}

//*************** Find state index with minimum distance ****************
//* assume input symbol is 4bit, first symbol in bit7~4, second symbol in bit3~0
// Parameters:
//   none
// Return value:
//   state with minimum distance
int FindMinIndex()
{
	int i, MinState = 0;
	int MinDistance = 32 * 250;	// maximum distance 32 * 250

	// calculate minimum distance
	for (i = 0; i < 64; i ++)
	{
		if (MinDistance > Distance[i])
		{
			MinDistance = Distance[i];
			MinState = i;
		}
	}

	return MinState;
}

// Length is number of bits in BitStream to do CRC24Q encode
// input BitStream filled with MSB first and start from index 0
// if encoded bits does not fit all bits in BitStream (Length is not multiple of 32)
// 0s need to be filled first (at MSBs of BitStream[0]) to make sure last encoded bits
// in bit0 of last index of BitStream array (this is because encoding 0 into all zero
// state CRC24Q encode does not change encoder status)
unsigned int Crc24qEncode(unsigned int *BitStream, int Length)
{
	int i, ByteNum;
	unsigned int Data, crc_result = 0;

	ByteNum = ((Length + 31) / 32) * 4;
	Data = BitStream[0];
	for (i = 0; i < ByteNum; i ++)
	{
		crc_result = (crc_result << 8) ^ Crc24QTable[(Data >> 24) ^ (unsigned char)(crc_result >> 16)];
		Data <<= 8;
		if ((i & 3) == 3)	// move to next bit
			Data = BitStream[(i >> 2) + 1];
	}

	return crc_result & 0xffffff;
}
