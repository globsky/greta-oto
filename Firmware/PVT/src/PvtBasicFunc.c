//----------------------------------------------------------------------
// PvtBasicFunc.c:
//   Commonly used functions in PVT
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <math.h>
#include "DataTypes.h"

static unsigned int GetParity(unsigned int word);

// play a trick here for ScaleDouble, ScaleDoubleU
// ScaleFloat and ScaleFloatU
// directly subtract scale value from exponent
// refer to IEEE 754 floating point format
typedef union
{
	double d_data;
	unsigned int i_data[2];
} DOUBLE_INT_UNION;

//*************** Fast convert signed integer to double value with 2^x scale ****************
//* directly subtract scale value from exponent
//* refer to IEEE 754 floating point format
// Parameters:
//   value: integer value to be scaled
//   scale: exponential scale factor
// Return value:
//   double format of value*2^(-scale)
double ScaleDouble(int value, int scale)
{
	DOUBLE_INT_UNION data;

	data.d_data = (double)value;
	if (value != 0)
	{
		data.i_data[1] -= (scale << 20);
	}

	return data.d_data;
}

//*************** Fast convert unsigned integer to double value with 2^x scale ****************
//* directly subtract scale value from exponent
//* refer to IEEE 754 floating point format
// Parameters:
//   value: integer value to be scaled
//   scale: exponential scale factor
// Return value:
//   double format of value*2^(-scale)
double ScaleDoubleU(unsigned int value, int scale)
{
	DOUBLE_INT_UNION data;

	data.d_data = (double)value;
	if (value != 0)
	{
		data.i_data[1] -= (scale << 20);
	}

	return data.d_data;
}

//*************** Fast convert signed long long to double value with 2^x scale ****************
//* directly subtract scale value from exponent
//* refer to IEEE 754 floating point format
// Parameters:
//   value: long long value to be scaled
//   scale: exponential scale factor
// Return value:
//   double format of value*2^(-scale)
double ScaleDoubleLong(long long value, int scale)
{
	DOUBLE_INT_UNION data;

	data.d_data = (double)value;
	if (value != 0)
	{
		data.i_data[1] -= (scale << 20);
	}

	return data.d_data;
}

//*************** Fast convert unsigned long long to double value with 2^x scale ****************
//* directly subtract scale value from exponent
//* refer to IEEE 754 floating point format
// Parameters:
//   value: long long value to be scaled
//   scale: exponential scale factor
// Return value:
//   double format of value*2^(-scale)
double ScaleDoubleULong(unsigned long long value, int scale)
{
	DOUBLE_INT_UNION data;

	data.d_data = (double)value;
	if (value != 0)
	{
		data.i_data[1] -= (scale << 20);
	}

	return data.d_data;
}

static const unsigned char ParityTable[6][16] = {
	{ 0x00, 0x13, 0x25, 0x36, 0x0B, 0x18, 0x2E, 0x3D, 0x16, 0x05, 0x33, 0x20, 0x1D, 0x0E, 0x38, 0x2B, }, 
	{ 0x00, 0x2C, 0x19, 0x35, 0x32, 0x1E, 0x2B, 0x07, 0x26, 0x0A, 0x3F, 0x13, 0x14, 0x38, 0x0D, 0x21, },
	{ 0x00, 0x0E, 0x1F, 0x11, 0x3E, 0x30, 0x21, 0x2F, 0x3D, 0x33, 0x22, 0x2C, 0x03, 0x0D, 0x1C, 0x12, },
	{ 0x00, 0x38, 0x31, 0x09, 0x23, 0x1B, 0x12, 0x2A, 0x07, 0x3F, 0x36, 0x0E, 0x24, 0x1C, 0x15, 0x2D, },
	{ 0x00, 0x0D, 0x1A, 0x17, 0x37, 0x3A, 0x2D, 0x20, 0x2F, 0x22, 0x35, 0x38, 0x18, 0x15, 0x02, 0x0F, },
	{ 0x00, 0x1C, 0x3B, 0x27, 0x34, 0x28, 0x0F, 0x13, 0x2A, 0x36, 0x11, 0x0D, 0x1E, 0x02, 0x25, 0x39,}};
 
//*************** calculate parity bits of GPS LNAV data ****************
// Parameters:
//   word: 32bit data word to do parity, d29* at bit31, d30* at bit30, current word at bit29 to bit0, 6LSB is ignored
// Return value:
//   parity bits in 6LSB
unsigned int GetParity(unsigned int word)
{
	int i;
	unsigned int parity = 0;

	// first skip 6LSB of word
	word >>= 6;
	for (i = 0; i < 6; i ++)
	{
		parity ^= (unsigned int)ParityTable[i][word&0xf];
		word >>= 4;
	}
	// add d29* and d30*
	if (word & 1)
		parity ^= 0x15;
	if (word & 2)
		parity ^= 0x29;

	return parity;
}

//*************** check parity of one WORD in GPS LNAV data ****************
// Parameters:
//   word: 32bit data word to check parity, d29* at bit31, d30* at bit30, current word at bit29 to bit0
// Return value:
//   parity check result
BOOL GpsParityCheck(unsigned int word)
{
	return (GetParity(word) == (word & 0x3f));
}
