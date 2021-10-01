//----------------------------------------------------------------------
// NoiseCalc.h:
//   Declaration of noise floor calculation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __NOISE_CALC_H__
#define __NOISE_CALC_H__

#include "CommonOps.h"

class CNoiseCalc
{
public:
	CNoiseCalc();
	~CNoiseCalc();

	void Reset();
	void AccumulateSample(complex_int Sample);
	void ShiftCode();
	void SetSmoothFactor(int Factor);
	void SetNoise(unsigned int Value);
	unsigned int GetNoise();
	unsigned int Amplitude(complex_int data);

	reg_uint SmoothFactor;			// 2bit
	reg_uint SmoothedNoise;			// 24bit
	reg_uint PrnCode;				// 10bit

	complex_int NoiseAcc;
};

#endif // __NOISE_CALC_H__
