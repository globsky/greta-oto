//----------------------------------------------------------------------
// BBCommonFunc.c:
//   Declaration of commonly used functions for baseband
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#define ABS(x) ((x) >= 0 ? (x) : -(x))

int Log2(unsigned int data);
int IntLog10(unsigned int data);
int IntSqrt(unsigned int Data);
int AmplitudeJPL(int Real, int Imag);
