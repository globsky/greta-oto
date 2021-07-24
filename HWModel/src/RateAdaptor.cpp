#include <memory.h>
#include <stdio.h>
#include "RateAdaptor.h"

#define OUTPUT_DOWN_CONVERT_DATA 0
#define OUTPUT_FILTER_DATA 0
#define OUTPUT_QUANT_DATA 0

const complex_int CRateAdaptor::DownConvertTable[64] = {
complex_int( 12,  -1), complex_int( 12,  -2), complex_int( 12,  -3), complex_int( 11,  -4),
complex_int( 11,  -5), complex_int( 10,  -6), complex_int( 10,  -7), complex_int(  9,  -8),
complex_int(  8,  -9), complex_int(  7, -10), complex_int(  6, -10), complex_int(  5, -11),
complex_int(  4, -11), complex_int(  3, -12), complex_int(  2, -12), complex_int(  1, -12),
complex_int( -1, -12), complex_int( -2, -12), complex_int( -3, -12), complex_int( -4, -11),
complex_int( -5, -11), complex_int( -6, -10), complex_int( -7, -10), complex_int( -8,  -9),
complex_int( -9,  -8), complex_int(-10,  -7), complex_int(-10,  -6), complex_int(-11,  -5),
complex_int(-11,  -4), complex_int(-12,  -3), complex_int(-12,  -2), complex_int(-12,  -1),
complex_int(-12,   1), complex_int(-12,   2), complex_int(-12,   3), complex_int(-11,   4),
complex_int(-11,   5), complex_int(-10,   6), complex_int(-10,   7), complex_int( -9,   8),
complex_int( -8,   9), complex_int( -7,  10), complex_int( -6,  10), complex_int( -5,  11),
complex_int( -4,  11), complex_int( -3,  12), complex_int( -2,  12), complex_int( -1,  12),
complex_int(  1,  12), complex_int(  2,  12), complex_int(  3,  12), complex_int(  4,  11),
complex_int(  5,  11), complex_int(  6,  10), complex_int(  7,  10), complex_int(  8,   9),
complex_int(  9,   8), complex_int( 10,   7), complex_int( 10,   6), complex_int( 11,   5),
complex_int( 11,   4), complex_int( 12,   3), complex_int( 12,   2), complex_int( 12,   1),
};

const int CRateAdaptor::CodeRateFilterCoef[CODE_RATE_FILTER_STAGE/2] = {-5, 8, 20};

CRateAdaptor::CRateAdaptor()
{
	Reset();
}

void CRateAdaptor::Reset()
{
	CodeRateAdjustNco = CarrierNco = 0;
	Threshold = 37;
	memset(CodeRateFilterBuffer, 0, sizeof(CodeRateFilterBuffer));
}

int CRateAdaptor::DoRateAdaptor(complex_int InputSignal[], int Length, unsigned char OutputSignal[])
{
	int i;
	int FillToNext, FillCount;
	int InputSignalIndex = 0, OutputSignalIndex = 0;
	int DoFilter;
	unsigned int Nco;
	complex_int FilteredSignal;

	while (Length > 0)
	{
		// determine how many input samples to generate next down sampled data
		FillToNext = 0;
		Nco = CodeRateAdjustNco;
		while (((Nco & 0xff000000) == 0) && FillToNext <= Length)
		{
			FillToNext ++;
			Nco += CodeRateAdjustRatio;
		}
		if (Length >= FillToNext)
		{
			FillCount = FillToNext;
			DoFilter = 1;
		}
		else
		{
			FillCount = Length;
			DoFilter = 0;
		}
		CodeRateAdjustNco += CodeRateAdjustRatio * FillCount;
		CodeRateAdjustNco &= 0xffffff;
		// shift buffer FillCount samples
		for (i = CODE_RATE_FILTER_STAGE-1; i >= FillCount; i --)
			CodeRateFilterBuffer[i] = CodeRateFilterBuffer[i-FillCount];
		// fill in input signal
		for (i = FillCount - 1; i >= 0; i --)
		{
			CodeRateFilterBuffer[i] = InputSignal[InputSignalIndex] * DownConvertTable[CarrierNco>>26];
#if OUTPUT_DOWN_CONVERT_DATA
			printf("%3d %3d %2d %4d %4d\n", InputSignal[InputSignalIndex].real, InputSignal[InputSignalIndex].imag, CarrierNco>>26, CodeRateFilterBuffer[i].real, CodeRateFilterBuffer[i].imag);
#endif
			InputSignalIndex ++;
			CarrierNco += CarrierFreq;
		}
		if (DoFilter)
		{
			// calculate filter output
			FilteredSignal = complex_int(0,0);
			for (i = 0; i < CODE_RATE_FILTER_STAGE/2; i ++)
				FilteredSignal += (CodeRateFilterBuffer[i] + CodeRateFilterBuffer[CODE_RATE_FILTER_STAGE-1-i]) * CodeRateFilterCoef[i];
#if OUTPUT_FILTER_DATA
			printf("%d %d\n", FilteredSignal.real, FilteredSignal.imag);
#endif
			// right shift
			FilteredSignal.real = ROUND_SHIFT_RAW(FilteredSignal.real, 6);
			FilteredSignal.imag = ROUND_SHIFT_RAW(FilteredSignal.imag, 6);
			
			OutputSignal[OutputSignalIndex] = Quant2Bit(FilteredSignal);
#if OUTPUT_QUANT_DATA
			printf("%1d\n", OutputSignal[OutputSignalIndex]);
#endif
			OutputSignalIndex ++;
		}

		Length -= FillCount;
	}

	return OutputSignalIndex;
}

unsigned char CRateAdaptor::Quant2Bit(complex_int Sample)
{
	unsigned char QuantSample = 0;
	unsigned int AbsValue;

	QuantSample |= (Sample.real >= 0) ? 0 : 8;
	AbsValue = (QuantSample & 8) ? -Sample.real : Sample.real;
	QuantSample |= (AbsValue > Threshold) ? 4 : 0;
	QuantSample |= (Sample.imag >= 0) ? 0 : 2;
	AbsValue = (QuantSample & 2) ? -Sample.imag : Sample.imag;
	QuantSample |= (AbsValue > Threshold) ? 1 : 0;

	return QuantSample;
}
