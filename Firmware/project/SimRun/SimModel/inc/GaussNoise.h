//----------------------------------------------------------------------
// GaussNoise.h:
//   Definition of functions to generate Gauss Noise
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined (__GAUSS_NOISE_H__)
#define __GAUSS_NOISE_H__

#include "ComplexNumber.h"

#define MAX_DIM 32
#define SUM_N(n) (((n)*(n+1))>>1)	// 1 sum to n
#define DIAG_INDEX(n) (SUM_N(n)+n)	// diagonal index of n

complex_number GenerateNoise(double Sigma);
void CalculateCovar(int dim, int Interval, double CovarMatrix[]);
void GenerateRelativeNoise(int dim, int max_index, double CovarMatrix[], double Sigma, complex_number Noise[]);

#endif //!defined(__GAUSS_NOISE_H__)
