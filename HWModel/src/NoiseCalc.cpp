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

CNoiseCalc::CNoiseCalc()
{
	Reset();
	SmoothFactor= 0;
}

CNoiseCalc::~CNoiseCalc()
{
}

void CNoiseCalc::Reset()
{
	PrnCode = 0x3ff;	// reset to all 1
	NoiseAcc = complex_int(0,0);
	SmoothedNoise = 784 << 8;
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
	if (PrnCode & 0x200)
		NoiseAcc -= Sample;
	else
		NoiseAcc += Sample;
}

void CNoiseCalc::ShiftCode()
{
	int Feedback = ((PrnCode << 7) ^ PrnCode) & 0x200;
	int Adjust;

	// shift m serial
	PrnCode <<= 1;
	PrnCode += Feedback ? 1 : 0;
	// determine end of sequence
	if ((PrnCode & 0x3ff) == 0x3ff)
	{
		// calculate smoothed value
		Adjust = Amplitude(NoiseAcc) - (SmoothedNoise >> 8);
		Adjust >>= (SmoothFactor * 2);
		SmoothedNoise += Adjust;
		printf("%d %d %d\n", NoiseAcc.real, NoiseAcc.imag, SmoothedNoise >> 8);
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
