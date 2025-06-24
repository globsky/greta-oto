//----------------------------------------------------------------------
// GaussNoise.cpp:
//   Implementation of functions to generate Gauss Noise
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <math.h>
#include <memory.h>
#include <stdlib.h>

#include "GaussNoise.h"
#include "ConstVal.h"

static double CorValue(int diff, int Interval);

complex_number GenerateNoise(double Sigma)
{
	int value1, value2;
	double fvalue1, fvalue2;
	complex_number noise;

	value1 = RAND_MAX + 1 - rand();	// range from 1 to RAND_MAX
	value2 = RAND_MAX + 1 - rand();	// range from 1 to RAND_MAX
	fvalue1 = (double)value1 / (RAND_MAX + 1);
	fvalue2 = (double)value2 / (RAND_MAX + 1);
	// scale noise power to be Sigma^2
	noise.real = sqrt(-log(fvalue1) * 2) * cos(PI2 * fvalue2) * Sigma;
	noise.imag = sqrt(-log(fvalue1) * 2) * sin(PI2 * fvalue2) * Sigma;

	return noise;
}

void CalculateCovar(int dim, int Interval, double CovarMatrix[])
{
	int i, j, k, end;
	double *p = CovarMatrix, *p1, *p2;
	double diag, value;

	memset(CovarMatrix, 0, sizeof(double) * SUM_N(dim));
	// first column
	for (i = 0; i < Interval; i ++)
	{
		*p++ = CorValue(i, Interval);
		p += i;
	}

	for (i = 1; i < dim; i ++)
	{
		// diagonal element
		p = CovarMatrix + DIAG_INDEX(i);
		p1 = CovarMatrix + SUM_N(i);
		diag = 1.0;
		for (k = 0; k < i; k ++)
		{
			diag -= (*p1) * (*p1);
			p1 ++;
		}
		diag = sqrt(diag);
		*p++ = diag;
		p += i;

		end = i + Interval;
		end = (end > dim) ? dim : end;
		for (j = i + 1; j < end; j ++)
		{
			p1 = CovarMatrix + SUM_N(i);
			p2 = CovarMatrix + SUM_N(j);
			value = CorValue(i - j, Interval);
			for (k = 0; k < i; k ++)
			{
				value -= (*p1) * (*p2);
				p1 ++; p2 ++;
			}
			*p++ = value / diag;
			p += j;
		}
	}
}

void GenerateRelativeNoise(int dim, int max_index, double CovarMatrix[], double Sigma, complex_number Noise[])
{
	int i, j, start;
	complex_number noise[MAX_DIM];
	double *p;

	// first generate normalized complex Gauss noise
	for (i = 0; i < dim; i ++)
		noise[i] = GenerateNoise(Sigma);

	// generate relative noise
	for (i = 0; i < dim; i ++)
	{
		start = i - max_index + 1;
		if (start < 0) start = 0;
		Noise[i].real = Noise[i].imag = 0.0;
		p = CovarMatrix + SUM_N(i) + start;
		for (j = start; j <= i; j ++, p ++)
		{
			Noise[i].real += noise[j].real * (*p);
			Noise[i].imag += noise[j].imag * (*p);
		}
	}
}

double CorValue(int diff, int Interval)
{
	diff = (diff >= 0) ? diff : -diff;
	return (diff >= Interval) ? 0.0 : ((diff == 0) ? 1.0 : 1 - (double)diff / Interval);
}
