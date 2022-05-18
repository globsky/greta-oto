//----------------------------------------------------------------------
// NoiseCalc.cpp:
//   Implementation of noise floor calculation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#include <stdio.h>
#include "CommonOps.h"
#include "NoiseCalc.h"

CNoiseCalc::CNoiseCalc(int CodeLength) : CodeLength(CodeLength)
{
	Reset();
	SmoothFactor= 0;
}

CNoiseCalc::~CNoiseCalc()
{
}

void CNoiseCalc::Reset()
{
	PrnCode = (1 << CodeLength) - 1;	// reset to all 1
	NoiseAcc = complex_int(0,0);
	SmoothedNoise = (CodeLength == 14) ? (1920 << 8) : (784 << 8);
}

void CNoiseCalc::SetSmoothFactor(int Factor)
{
	SmoothFactor = Factor & 0x3;
}

void CNoiseCalc::SetNoise(unsigned int Value)
{
	SmoothedNoise = Value << 8;
}

unsigned int CNoiseCalc::GetNoise()
{
	return 	SmoothedNoise >> 8;
}

void CNoiseCalc::AccumulateSample(complex_int Sample)
{
	// accumulate input sample correlated with m serial
	if (PrnCode & (1 << (CodeLength-1)))
		NoiseAcc -= Sample;
	else
		NoiseAcc += Sample;
}

void CNoiseCalc::ShiftCode()
{
	int Feedback;
	int Adjust;

	// shift m serial
	if (CodeLength == 14)
		Feedback = ((PrnCode >> 13) ^ (PrnCode >> 7) ^ (PrnCode >> 5) ^ PrnCode) & 1;
	else
		Feedback = ((PrnCode >> 9) ^ (PrnCode >> 2)) & 1;
	PrnCode <<= 1;
	PrnCode += Feedback ? 1 : 0;
	// determine end of sequence
	if ((PrnCode & ((1 << CodeLength) - 1)) == ((CodeLength == 14) ? 0xade : 0x3ff))
	{
		PrnCode = (1 << CodeLength) - 1;	// reset to all 1
		// calculate smoothed value
		Adjust = Amplitude(NoiseAcc) - (SmoothedNoise >> 8);
		Adjust >>= (SmoothFactor * 2);
		SmoothedNoise += Adjust;
//		printf("%d %d %d %d\n", NoiseAcc.real, NoiseAcc.imag, Amplitude(NoiseAcc), SmoothedNoise);
		// reset accumulator
		NoiseAcc = complex_int(0,0);
	}
}

unsigned int CNoiseCalc::Amplitude(complex_int data)
{
	unsigned int max, min, amp;

	max = (data.real & 0x8000) ? ~data.real : data.real;
	min = (data.imag & 0x8000) ? ~data.imag : data.imag;
	if (max < min)
	{
		amp = max; max = min; min = amp;
	}
	if (max > min * 3)
		amp = max + (min >> 3);
	else
		amp = max - (max >> 3) + (min >> 1);

	return amp;
}
