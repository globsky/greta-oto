//----------------------------------------------------------------------
// BBCommonFunc.c:
//   Commonly used functions for baseband
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "PlatformCtrl.h"

//*************** Calculate base 2 logarithm ****************
//* The algorithm uses floating point convertion
// Parameters:
//   data: input value
// Return value:
//   log2(data) with 2^23 scale
/*int Log2(unsigned int data)
{
	float fdata = (float)data;
	unsigned int idata = *((int *)(&fdata));
	int adjust = ((idata & 0x7ffff) >> 12) - 995;

	// calculate adjustment to log2(data)
	adjust = ((1 << 20) - adjust * adjust);
	adjust *= 707;
	idata += ((adjust >> 10) - (127 << 23));

	return (int)idata;	// 2^23 scale of log2(data)
}*/

//*************** Calculate base 2 logarithm ****************
// Parameters:
//   data: input value
// Return value:
//   log2(data) with 2^23 scale
int Log2(unsigned int data)
{
	int exp = 31 - __builtin_clz(data);
	int fraction = ((exp <= 11) ? data : (data >> (exp - 11))) & 0x7ff;	// 11bit
	int adjust = fraction - 995;

	// calculate adjustment to log2(data)
	adjust = ((1 << 20) - adjust * adjust);
	adjust *= 707;
	data = (exp << 23) + (fraction << 12) + (adjust >> 10);

	return (int)data;	// 2^23 scale of log2(data)
}

//*************** Calculate base 10 logarithm ****************
// Parameters:
//   data: input value
// Return value:
//   log10(data) with 1000 scale (or in unit of 0.01dB)
int IntLog10(unsigned int data)
{
	int logdata = Log2(data);

	// convert to log10(data)*100
	logdata = 9633 * (logdata >> 12);
	return (logdata >> 16);
}

//*************** Calculate square root of a integer value ****************
// Parameters:
//   data: input value
// Return value:
//   largest integer value not exceed sqrt(data)
int IntSqrt(unsigned int Data)
{
	int Result, Diff1, Diff2;
	int Exp;

	if (Data == 0)
		return 0;
	Exp = 32 - __builtin_clz(Data);
	Exp = (Exp - 1) >> 1;
	Result = (1 << Exp);
	Diff1 = Data - (1 << Exp * 2);
	while (--Exp >= 0)
	{
		Diff2 = (Result << (Exp + 1)) + (1 << Exp * 2);
		if (Diff1 >= Diff2)
		{
			Diff1 -= Diff2;
			Result += (1 << Exp);
		}
	}

	return Result;
}

//*************** Calculate amplitude (absolute value) of a complex number ****************
//* The algorithm uses JPL method
// Parameters:
//   Real: real part of complex value
//   Imag: imaginary part of complex value
// Return value:
//   absolute value
int AmplitudeJPL(int Real, int Imag)
{
	int Amplitude;

	Real = (Real >= 0) ? Real : -Real;
	Imag = (Imag >= 0) ? Imag : -Imag;
	if (Real < Imag)
	{
		Real ^= Imag; Imag ^= Real; Real ^= Imag;	// swap
	}
	if (Real > Imag * 3)
		Amplitude = Real + (Imag >> 3);
	else
		Amplitude = Real - (Real >> 3) + (Imag >> 1);

	return Amplitude;
}
