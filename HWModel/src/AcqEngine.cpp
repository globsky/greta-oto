//----------------------------------------------------------------------
// AcqEngine.cpp:
//   Acquisition engine class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <intrin.h>

#include "AcqEngine.h"

complex_exp10::complex_exp10(complex_int data)
{
	int real_exp, imag_exp;

	if (data.real >= 0)
		real_exp = 23 - __builtin_clz(data.real);
	else
		real_exp = 23 - __builtin_clz(~data.real);
	if (data.imag >= 0)
		imag_exp = 23 - __builtin_clz(data.imag);
	else
		imag_exp = 23 - __builtin_clz(~data.imag);
	this->exp = (real_exp > imag_exp) ? real_exp : imag_exp;
	if (this->exp > 0)
	{
		this->real = (data.real) >> (this->exp);
		this->imag = (data.imag) >> (this->exp);
	}
	else
	{
		this->real = data.real;
		this->imag = data.imag;
		this->exp = 0;
	}
}

void complex_exp10::operator = (complex_int data)
{
	*this = complex_exp10(data);
}

complex_exp10 complex_exp10::operator + (complex_int data)
{
	complex_exp10 result = *this;
	result += data;
	return result;
}

void complex_exp10::operator += (complex_int data)
{
	complex_exp10 temp = data;
	int exp;

	if (temp.exp > this->exp)
	{
		this->real >>= (temp.exp - this->exp);
		this->imag >>= (temp.exp - this->exp);
		exp = temp.exp;
	}
	else
	{
		temp.real >>= (this->exp - temp.exp);
		temp.imag >>= (this->exp - temp.exp);
		exp = this->exp;
	}
	this->real += temp.real;
	this->imag += temp.imag;
	this->exp = exp;
	if (this->real > 511 || this->imag > 511 || this->real < -512 || this->imag < -512)
	{
		this->exp ++;
		this->real >>= 1;
		this->imag >>= 1;
	}
}

complex_exp10 complex_exp10::operator - (complex_int data)
{
	complex_exp10 result = *this;
	result -= data;
	return result;
}

void complex_exp10::operator -= (complex_int data)
{
	complex_exp10 temp = data;
	int exp;

	if (temp.exp > this->exp)
	{
		this->real >>= (temp.exp - this->exp);
		this->imag >>= (temp.exp - this->exp);
		exp = temp.exp;
	}
	else
	{
		temp.real >>= (this->exp - temp.exp);
		temp.imag >>= (this->exp - temp.exp);
		exp = this->exp;
	}
	this->real -= temp.real;
	this->imag -= temp.imag;
	this->exp = exp;
	if (this->real > 511 || this->imag > 511 || this->real < -512 || this->imag < -512)
	{
		this->exp ++;
		this->real >>= 1;
		this->imag >>= 1;
	}
}

const int CAcqEngine::complex_mul_i[16][64] = {
{   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7,   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7,  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7,  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7},
{   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21,  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7,  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21, -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7},
{   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7,  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7,  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7,   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7},
{   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21, -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7,  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21,  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7},
{  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7,   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21, -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7,  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21},
{  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21,  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21, -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21, -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21},
{  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7,  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21, -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7,   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21},
{  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21, -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21, -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21,  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21},
{  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7,   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7,   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7,  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7},
{  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21,  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7,   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21, -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7},
{  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7,  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7,   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7,   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7},
{  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21, -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7,   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21,  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7},
{ -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7,   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21,  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7,  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21},
{ -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21,  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21,  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21, -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21},
{ -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7,  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21,  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7,   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21},
{ -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21, -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21,  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21,  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21},
};

const int CAcqEngine::complex_mul_q[16][64] = {
{   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7,  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7,  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7,   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7},
{  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7,  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21, -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7,   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21},
{  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7,  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7,   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7,   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7},
{ -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7,  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21,  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7,   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21},
{   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21, -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7,  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21,  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7},
{  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21, -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21, -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21,  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21},
{  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21, -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7,   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21,  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7},
{ -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21, -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21,  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21,  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21},
{   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7,   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7,  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7,  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7},
{  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7,   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21, -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7,  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21},
{  -7,  -6,  -5,  -5,  -3,  -2,  -2,   0,   0,   2,   2,   3,   5,   5,   6,   7,   7,   8,   9,   9,   9,  10,  10,  10,  10,  10,  10,   9,   9,   9,   8,   7,   7,   6,   5,   5,   3,   2,   2,   0,   0,  -2,  -2,  -3,  -5,  -5,  -6,  -7,  -7,  -8,  -9,  -9,  -9, -10, -10, -10, -10, -10, -10,  -9,  -9,  -9,  -8,  -7},
{ -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7,   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21,  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7,  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21},
{   7,  10,  13,  13,  15,  18,  18,  20,  20,  22,  22,  21,  23,  23,  22,  21,  21,  20,  19,  19,  15,  14,  14,  10,  10,   6,   6,   3,  -1,  -1,  -4,  -7,  -7, -10, -13, -13, -15, -18, -18, -20, -20, -22, -22, -21, -23, -23, -22, -21, -21, -20, -19, -19, -15, -14, -14, -10, -10,  -6,  -6,  -3,   1,   1,   4,   7},
{  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21,  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21, -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21, -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21},
{  -7,  -4,  -1,  -1,   3,   6,   6,  10,  10,  14,  14,  15,  19,  19,  20,  21,  21,  22,  23,  23,  21,  22,  22,  20,  20,  18,  18,  15,  13,  13,  10,   7,   7,   4,   1,   1,  -3,  -6,  -6, -10, -10, -14, -14, -15, -19, -19, -20, -21, -21, -22, -23, -23, -21, -22, -22, -20, -20, -18, -18, -15, -13, -13, -10,  -7},
{ -21, -18, -15, -15,  -9,  -6,  -6,   0,   0,   6,   6,   9,  15,  15,  18,  21,  21,  24,  27,  27,  27,  30,  30,  30,  30,  30,  30,  27,  27,  27,  24,  21,  21,  18,  15,  15,   9,   6,   6,   0,   0,  -6,  -6,  -9, -15, -15, -18, -21, -21, -24, -27, -27, -27, -30, -30, -30, -30, -30, -30, -27, -27, -27, -24, -21},
};

const int CAcqEngine::dft_table[128] = {
   0,  13,  25,  38,  50,  63,  75,  87, 100, 112, 124, 136, 148, 160, 172, 184,
 196, 207, 218, 230, 241, 252, 263, 273, 284, 294, 304, 314, 324, 334, 343, 352,
 361, 370, 379, 387, 395, 403, 410, 418, 425, 432, 438, 445, 451, 456, 462, 467,
 472, 477, 481, 485, 489, 492, 496, 499, 501, 503, 505, 507, 509, 510, 510, 511,
 511, 511, 510, 510, 509, 507, 505, 503, 501, 499, 496, 492, 489, 485, 481, 477,
 472, 467, 462, 456, 451, 445, 438, 432, 425, 418, 410, 403, 395, 387, 379, 370,
 361, 352, 343, 334, 324, 314, 304, 294, 284, 273, 263, 252, 241, 230, 218, 207,
 196, 184, 172, 160, 148, 136, 124, 112, 100,  87,  75,  63,  50,  38,  25,  13,
};

const unsigned int CAcqEngine::PrnPolySettings[2] = { (0x3a6 << 14) | 0x204, 1023 << 14 };

const unsigned int CAcqEngine::GpsInit[32+19] = {
	(0 << 28) | 0x037ffff1,		// for PRN1
	(0 << 28) | 0x01bffff1,		// for PRN2
	(0 << 28) | 0x00dffff1,		// for PRN3
	(0 << 28) | 0x006ffff1,		// for PRN4
	(0 << 28) | 0x06903ff1,		// for PRN5
	(0 << 28) | 0x03483ff1,		// for PRN6
	(0 << 28) | 0x069bbff1,		// for PRN7
	(0 << 28) | 0x034dfff1,		// for PRN8
	(0 << 28) | 0x01a6fff1,		// for PRN9
	(0 << 28) | 0x02eefff1,		// for PRN10
	(0 << 28) | 0x01777ff1,		// for PRN11
	(0 << 28) | 0x005dfff1,		// for PRN12
	(0 << 28) | 0x002efff1,		// for PRN13
	(0 << 28) | 0x00177ff1,		// for PRN14
	(0 << 28) | 0x000bbff1,		// for PRN15
	(0 << 28) | 0x0005fff1,		// for PRN16
	(0 << 28) | 0x06447ff1,		// for PRN17
	(0 << 28) | 0x03223ff1,		// for PRN18
	(0 << 28) | 0x01913ff1,		// for PRN19
	(0 << 28) | 0x00c8bff1,		// for PRN20
	(0 << 28) | 0x00647ff1,		// for PRN21
	(0 << 28) | 0x00323ff1,		// for PRN22
	(0 << 28) | 0x07333ff1,		// for PRN23
	(0 << 28) | 0x00e67ff1,		// for PRN24
	(0 << 28) | 0x00733ff1,		// for PRN25
	(0 << 28) | 0x0039bff1,		// for PRN26
	(0 << 28) | 0x001cfff1,		// for PRN27
	(0 << 28) | 0x000e7ff1,		// for PRN28
	(0 << 28) | 0x06a23ff1,		// for PRN29
	(0 << 28) | 0x03513ff1,		// for PRN30
	(0 << 28) | 0x01a8bff1,		// for PRN31
	(0 << 28) | 0x00d47ff1,		// for PRN32
	(0 << 28) | 0x091a7ff1,		// for PRN120
	(0 << 28) | 0x0a863ff1,		// for PRN121
	(0 << 28) | 0x02dcfff1,		// for PRN122
	(0 << 28) | 0x02693ff1,		// for PRN123
	(0 << 28) | 0x0e3e3ff1,		// for PRN124
	(0 << 28) | 0x08f87ff1,		// for PRN125
	(0 << 28) | 0x0fd27ff1,		// for PRN126
	(0 << 28) | 0x073d7ff1,		// for PRN127
	(0 << 28) | 0x0d6afff1,		// for PRN128
	(0 << 28) | 0x0aa37ff1,		// for PRN129
	(0 << 28) | 0x03857ff1,		// for PRN130
	(0 << 28) | 0x05a57ff1,		// for PRN131
	(0 << 28) | 0x05433ff1,		// for PRN132
	(0 << 28) | 0x0f67bff1,		// for PRN133
	(0 << 28) | 0x07183ff1,		// for PRN134
	(0 << 28) | 0x0a387ff1,		// for PRN135
	(0 << 28) | 0x07833ff1,		// for PRN136
	(0 << 28) | 0x081e3ff1,		// for PRN137
	(0 << 28) | 0x04a13ff1,		// for PRN138
};

const unsigned int CAcqEngine::B1CInit[63] = {
	(8 << 28) +  796 + (( 7575 - 1) << 14),	// for PRN01
	(8 << 28) +  156 + (( 2369 - 1) << 14),	// for PRN02
	(8 << 28) + 4198 + (( 5688 - 1) << 14),	// for PRN03
	(8 << 28) + 3941 + ((  539 - 1) << 14),	// for PRN04
	(8 << 28) + 1374 + (( 2270 - 1) << 14),	// for PRN05
	(8 << 28) + 1338 + (( 7306 - 1) << 14),	// for PRN06
	(8 << 28) + 1833 + (( 6457 - 1) << 14),	// for PRN07
	(8 << 28) + 2521 + (( 6254 - 1) << 14),	// for PRN08
	(8 << 28) + 3175 + (( 5644 - 1) << 14),	// for PRN09
	(8 << 28) +  168 + (( 7119 - 1) << 14),	// for PRN10
	(8 << 28) + 2715 + (( 1402 - 1) << 14),	// for PRN11
	(8 << 28) + 4408 + (( 5557 - 1) << 14),	// for PRN12
	(8 << 28) + 3160 + (( 5764 - 1) << 14),	// for PRN13
	(8 << 28) + 2796 + (( 1073 - 1) << 14),	// for PRN14
	(8 << 28) +  459 + (( 7001 - 1) << 14),	// for PRN15
	(8 << 28) + 3594 + (( 5910 - 1) << 14),	// for PRN16
	(8 << 28) + 4813 + ((10060 - 1) << 14),	// for PRN17
	(8 << 28) +  586 + (( 2710 - 1) << 14),	// for PRN18
	(8 << 28) + 1428 + (( 1546 - 1) << 14),	// for PRN19
	(8 << 28) + 2371 + (( 6887 - 1) << 14),	// for PRN20
	(8 << 28) + 2285 + (( 1883 - 1) << 14),	// for PRN21
	(8 << 28) + 3377 + (( 5613 - 1) << 14),	// for PRN22
	(8 << 28) + 4965 + (( 5062 - 1) << 14),	// for PRN23
	(8 << 28) + 3779 + (( 1038 - 1) << 14),	// for PRN24
	(8 << 28) + 4547 + ((10170 - 1) << 14),	// for PRN25
	(8 << 28) + 1646 + (( 6484 - 1) << 14),	// for PRN26
	(8 << 28) + 1430 + (( 1718 - 1) << 14),	// for PRN27
	(8 << 28) +  607 + (( 2535 - 1) << 14),	// for PRN28
	(8 << 28) + 2118 + (( 1158 - 1) << 14),	// for PRN29
	(8 << 28) + 4709 + (( 526  - 1) << 14),	// for PRN30
	(8 << 28) + 1149 + (( 7331 - 1) << 14),	// for PRN31
	(8 << 28) + 3283 + (( 5844 - 1) << 14),	// for PRN32
	(8 << 28) + 2473 + (( 6423 - 1) << 14),	// for PRN33
	(8 << 28) + 1006 + (( 6968 - 1) << 14),	// for PRN34
	(8 << 28) + 3670 + (( 1280 - 1) << 14),	// for PRN35
	(8 << 28) + 1817 + (( 1838 - 1) << 14),	// for PRN36
	(8 << 28) +  771 + (( 1989 - 1) << 14),	// for PRN37
	(8 << 28) + 2173 + (( 6468 - 1) << 14),	// for PRN38
	(8 << 28) +  740 + (( 2091 - 1) << 14),	// for PRN39
	(8 << 28) + 1433 + (( 1581 - 1) << 14),	// for PRN40
	(8 << 28) + 2458 + (( 1453 - 1) << 14),	// for PRN41
	(8 << 28) + 3459 + (( 6252 - 1) << 14),	// for PRN42
	(8 << 28) + 2155 + (( 7122 - 1) << 14),	// for PRN43
	(8 << 28) + 1205 + (( 7711 - 1) << 14),	// for PRN44
	(8 << 28) +  413 + (( 7216 - 1) << 14),	// for PRN45
	(8 << 28) +  874 + (( 2113 - 1) << 14),	// for PRN46
	(8 << 28) + 2463 + (( 1095 - 1) << 14),	// for PRN47
	(8 << 28) + 1106 + (( 1628 - 1) << 14),	// for PRN48
	(8 << 28) + 1590 + (( 1713 - 1) << 14),	// for PRN49
	(8 << 28) + 3873 + (( 6102 - 1) << 14),	// for PRN50
	(8 << 28) + 4026 + (( 6123 - 1) << 14),	// for PRN51
	(8 << 28) + 4272 + (( 6070 - 1) << 14),	// for PRN52
	(8 << 28) + 3556 + (( 1115 - 1) << 14),	// for PRN53
	(8 << 28) +  128 + (( 8047 - 1) << 14),	// for PRN54
	(8 << 28) + 1200 + (( 6795 - 1) << 14),	// for PRN55
	(8 << 28) +  130 + (( 2575 - 1) << 14),	// for PRN56
	(8 << 28) + 4494 + ((   53 - 1) << 14),	// for PRN57
	(8 << 28) + 1871 + (( 1729 - 1) << 14),	// for PRN58
	(8 << 28) + 3073 + (( 6388 - 1) << 14),	// for PRN59
	(8 << 28) + 4386 + ((  682 - 1) << 14),	// for PRN60
	(8 << 28) + 4098 + (( 5565 - 1) << 14),	// for PRN61
	(8 << 28) + 1923 + (( 7160 - 1) << 14),	// for PRN62
	(8 << 28) + 1176 + (( 2277 - 1) << 14),	// for PRN63
};
const unsigned int CAcqEngine::L1CInit[63] = {
	(10 << 28) + 5097 + ((  181 - 1) << 14),	// for PRN01
	(10 << 28) + 5110 + ((  359 - 1) << 14),	// for PRN02
	(10 << 28) + 5079 + ((   72 - 1) << 14),	// for PRN03
	(10 << 28) + 4403 + (( 1110 - 1) << 14),	// for PRN04
	(10 << 28) + 4121 + (( 1480 - 1) << 14),	// for PRN05
	(10 << 28) + 5043 + (( 5034 - 1) << 14),	// for PRN06
	(10 << 28) + 5042 + (( 4622 - 1) << 14),	// for PRN07
	(10 << 28) + 5104 + ((    1 - 1) << 14),	// for PRN08
	(10 << 28) + 4940 + (( 4547 - 1) << 14),	// for PRN09
	(10 << 28) + 5035 + ((  826 - 1) << 14),	// for PRN10
	(10 << 28) + 4372 + (( 6284 - 1) << 14),	// for PRN11
	(10 << 28) + 5064 + (( 4195 - 1) << 14),	// for PRN12
	(10 << 28) + 5084 + ((  368 - 1) << 14),	// for PRN13
	(10 << 28) + 5048 + ((    1 - 1) << 14),	// for PRN14
	(10 << 28) + 4950 + (( 4796 - 1) << 14),	// for PRN15
	(10 << 28) + 5019 + ((  523 - 1) << 14),	// for PRN16
	(10 << 28) + 5076 + ((  151 - 1) << 14),	// for PRN17
	(10 << 28) + 3736 + ((  713 - 1) << 14),	// for PRN18
	(10 << 28) + 4993 + (( 9850 - 1) << 14),	// for PRN19
	(10 << 28) + 5060 + (( 5734 - 1) << 14),	// for PRN20
	(10 << 28) + 5061 + ((   34 - 1) << 14),	// for PRN21
	(10 << 28) + 5096 + (( 6142 - 1) << 14),	// for PRN22
	(10 << 28) + 4983 + ((  190 - 1) << 14),	// for PRN23
	(10 << 28) + 4783 + ((  644 - 1) << 14),	// for PRN24
	(10 << 28) + 4991 + ((  467 - 1) << 14),	// for PRN25
	(10 << 28) + 4815 + (( 5384 - 1) << 14),	// for PRN26
	(10 << 28) + 4443 + ((  801 - 1) << 14),	// for PRN27
	(10 << 28) + 4769 + ((  594 - 1) << 14),	// for PRN28
	(10 << 28) + 4879 + (( 4450 - 1) << 14),	// for PRN29
	(10 << 28) + 4894 + (( 9437 - 1) << 14),	// for PRN30
	(10 << 28) + 4985 + (( 4307 - 1) << 14),	// for PRN31
	(10 << 28) + 5056 + (( 5906 - 1) << 14),	// for PRN32
	(10 << 28) + 4921 + ((  378 - 1) << 14),	// for PRN33
	(10 << 28) + 5036 + (( 9448 - 1) << 14),	// for PRN34
	(10 << 28) + 4812 + (( 9432 - 1) << 14),	// for PRN35
	(10 << 28) + 4838 + (( 5849 - 1) << 14),	// for PRN36
	(10 << 28) + 4855 + (( 5547 - 1) << 14),	// for PRN37
	(10 << 28) + 4904 + (( 9546 - 1) << 14),	// for PRN38
	(10 << 28) + 4753 + (( 9132 - 1) << 14),	// for PRN39
	(10 << 28) + 4483 + ((  403 - 1) << 14),	// for PRN40
	(10 << 28) + 4942 + (( 3766 - 1) << 14),	// for PRN41
	(10 << 28) + 4813 + ((    3 - 1) << 14),	// for PRN42
	(10 << 28) + 4957 + ((  684 - 1) << 14),	// for PRN43
	(10 << 28) + 4618 + (( 9711 - 1) << 14),	// for PRN44
	(10 << 28) + 4669 + ((  333 - 1) << 14),	// for PRN45
	(10 << 28) + 4969 + (( 6124 - 1) << 14),	// for PRN46
	(10 << 28) + 5031 + ((10216 - 1) << 14),	// for PRN47
	(10 << 28) + 5038 + (( 4251 - 1) << 14),	// for PRN48
	(10 << 28) + 4740 + (( 9893 - 1) << 14),	// for PRN49
	(10 << 28) + 4073 + (( 9884 - 1) << 14),	// for PRN50
	(10 << 28) + 4843 + (( 4627 - 1) << 14),	// for PRN51
	(10 << 28) + 4979 + (( 4449 - 1) << 14),	// for PRN52
	(10 << 28) + 4867 + (( 9798 - 1) << 14),	// for PRN53
	(10 << 28) + 4964 + ((  985 - 1) << 14),	// for PRN54
	(10 << 28) + 5025 + (( 4272 - 1) << 14),	// for PRN55
	(10 << 28) + 4579 + ((  126 - 1) << 14),	// for PRN56
	(10 << 28) + 4390 + ((10024 - 1) << 14),	// for PRN57
	(10 << 28) + 4763 + ((  434 - 1) << 14),	// for PRN58
	(10 << 28) + 4612 + (( 1029 - 1) << 14),	// for PRN59
	(10 << 28) + 4784 + ((  561 - 1) << 14),	// for PRN60
	(10 << 28) + 3716 + ((  289 - 1) << 14),	// for PRN61
	(10 << 28) + 4703 + ((  638 - 1) << 14),	// for PRN62
	(10 << 28) + 4851 + (( 4353 - 1) << 14),	// for PRN63
};

CAcqEngine::CAcqEngine(unsigned int *MemCodeAddress)
{
	int i;

	for (i = 0; i < TOTAL_INTERMEDIATE_RESULT; i ++)
		fp_out[i] = NULL;

	PrnGen[0] = new CGeneralPrn(PrnPolySettings);
	PrnGen[1] = new CMemoryPrn(MemCodeAddress);
	PrnGen[2] = PrnGen[3] = new CWeilPrn;
	Reset();
	memset(ChannelConfig, 0, sizeof(ChannelConfig));
}

CAcqEngine::~CAcqEngine()
{
	delete PrnGen[0];
	delete PrnGen[1];
	delete PrnGen[2];
}

void CAcqEngine::Reset()
{
	BufferThreshold = 0;
	ReadPointer = WritePointer = 0;
	Filling = 0;
	CarrierNco = 0;
	DftNco = 0;
	LastInput = complex_int(0, 0);
	EarlyTerminate = 0;
	PeakRatioTh = 3;
}

void CAcqEngine::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_AE_CONTROL:
		ChannelNumber = EXTRACT_UINT(Value, 0, 6);
		if (Value & 0x100)
			DoAcquisition();
		break;
	case ADDR_OFFSET_AE_BUFFER_CONTROL:
		BufferThreshold = EXTRACT_UINT(Value, 0, 7);
		if (Value & 200)
			RateAdaptor.Reset();
		if (Value & 100)
			StartFill();
		break;
	case ADDR_OFFSET_AE_CARRIER_FREQ:
		RateAdaptor.CarrierFreq = Value;
		break;
	case ADDR_OFFSET_AE_CODE_RATIO:
		RateAdaptor.CodeRateAdjustRatio = EXTRACT_UINT(Value, 0, 24);
		break;
	case ADDR_OFFSET_AE_THRESHOLD:
		RateAdaptor.Threshold = EXTRACT_UINT(Value, 0, 8);
		break;
	default:
		break;
	}
}

U32 CAcqEngine::GetRegValue(int Address)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_AE_CONTROL:
		return 	ChannelNumber;
	case ADDR_OFFSET_AE_BUFFER_CONTROL:
		return BufferThreshold;
	case ADDR_OFFSET_AE_STATUS:
		return 	(Filling ? 0x10000 : 0) | (((WritePointer >> 11) >= (int)BufferThreshold)  ? 0x20000 : 0) | ((WritePointer >= AE_BUFFER_SIZE) ? 0x40000 : 0) | 0x80000;
	case ADDR_OFFSET_AE_CARRIER_FREQ:
		return 	RateAdaptor.CarrierFreq;
	case ADDR_OFFSET_AE_CODE_RATIO:
		return 	RateAdaptor.CodeRateAdjustRatio;
	case ADDR_OFFSET_AE_THRESHOLD:
		return 	RateAdaptor.Threshold;
	default:
		return 0;
	}
}

complex_int CAcqEngine::ReadSampleFromFifo()
{
	int TableIndex;
	unsigned int BufferSample;
	complex_int InputSample;

	TableIndex = CarrierNco >> 26;
	BufferSample = ReadSample();
	InputSample = complex_int(complex_mul_i[BufferSample][TableIndex], complex_mul_q[BufferSample][TableIndex]);	// to improve the speed of software, use LUT instead of complex multiply
	CarrierNco += CarrierFreq;

	return InputSample;
}

complex_int CAcqEngine::ReadSampleToBuffer()
{
	complex_int InputSample, SampleToBuffer;

	InputSample = ReadSampleFromFifo();
	SampleToBuffer = LastInput + InputSample;
	SampleToBuffer.real >>= 1;
	SampleToBuffer.imag >>= 1;
	LastInput = InputSample;
	if (fp_out[INTERMEDIATE_RESULT_SAMPLE2BUFFER])
		fprintf(fp_out[INTERMEDIATE_RESULT_SAMPLE2BUFFER], "%3d %3d\n", SampleToBuffer.real, SampleToBuffer.imag);

	return SampleToBuffer;
}

void CAcqEngine::PreloadSample()
{
	int i;
	int StartAddr = ReadAddress * 682 + CodeRoundCount * MF_CORE_DEPTH;

	SetStartAddr(StartAddr);
	LastInput = ReadSampleFromFifo();
	for (i = 0; i < MF_CORE_DEPTH * 2; i ++)
		AcqSamples[i] = ReadSampleToBuffer();
}

void CAcqEngine::LoadSample()
{
	int i;

	for (i = MF_CORE_DEPTH; i < MF_CORE_DEPTH * 2; i ++)
		AcqSamples[i] = ReadSampleToBuffer();
}

void CAcqEngine::LoadCode()
{
	int i;
	for (i = 0; i < ADDER_TREE_WIDTH; i ++)
	{
		AcqCode[i] = PrnGen[PrnSelect]->GetCode();
		PrnGen[PrnSelect]->ShiftCode();
	}
}

void CAcqEngine::MatchFilterCore(int PhaseCount, complex_int CorResult[])
{
	int i, j;
/*	unsigned int TreeInput[344];
	int Sum;

	for (i = 0; i < PhaseCount; i ++)
	{
		TreeInput[0] = TreeInput[1] = TreeInput[2] = Sum = 0;
		for (j = 0; j < MF_CORE_DEPTH; j += 2)
			Sum += (TreeInput[j/2+3] = (AcqSamples[i+j].real & 1) ^ AcqCode[j/2]);
		for (j = 0; j < 86; j ++)
			TreeInput[4*j] = TreeInput[4*j] * 8 + TreeInput[4*j+1] * 4 + TreeInput[4*j+2] * 2 + TreeInput[4*j+3] * 1;
		for (j = 0; j < 86; j ++)
			printf("%c", (TreeInput[4*j] < 10) ? TreeInput[4*j] + '0' : TreeInput[4*j] - 10 + 'a');
		printf(" %x\n", Sum);
		TreeInput[0] = TreeInput[1] = TreeInput[2] = Sum = 0;
		for (j = 0; j < MF_CORE_DEPTH; j += 2)
			Sum += (TreeInput[j/2+3] = (AcqSamples[i+j].imag & 1) ^ AcqCode[j/2]);
		for (j = 0; j < 86; j ++)
			TreeInput[4*j] = TreeInput[4*j] * 8 + TreeInput[4*j+1] * 4 + TreeInput[4*j+2] * 2 + TreeInput[4*j+3] * 1;
		for (j = 0; j < 86; j ++)
			printf("%c", (TreeInput[4*j] < 10) ? TreeInput[4*j] + '0' : TreeInput[4*j] - 10 + 'a');
		printf(" %x\n", Sum);
	}
	*/
	for (i = 0; i < PhaseCount; i ++)
	{
		CorResult[i] = complex_int(0, 0);
		for (j = 0; j < MF_CORE_DEPTH; j += 2)
		{
			if (AcqCode[j/2])
			{
				CorResult[i] += ~(AcqSamples[i+j]);
			}
			else
				CorResult[i] += AcqSamples[i+j];
		}
	}
}

void CAcqEngine::GetDftFactor(complex_int DftFactor[DFT_NUMBER/2], int sign_cos[DFT_NUMBER/2], int sign_sin[DFT_NUMBER/2])
{
	int i;
	unsigned int NCO;
	int index;

	for (i = 0; i < DFT_NUMBER/2; i ++)
	{
		NCO = (i*2+1) * DftNco;
		NCO &= 0x3fff;	// 14bit
		NCO = (NCO >> 6);// + ((NCO & 0x20) ? 1 : 0);	// round shift 6bit
		index = NCO & 0x7f;		// 7LSB used as table index;
		DftFactor[i].imag = dft_table[index];
		DftFactor[i].real = dft_table[index^0x40];
		sign_cos[i] = ((NCO >> 7) & 1) ^ ((NCO >> 6) & 1);
		sign_sin[i] = ((~NCO >> 7) & 1);
	}
}

void CAcqEngine::NonCoherentAcc(unsigned int MaxCohExp, int NoncohCount)
{
	int CorCount, FreqCount, MaxFreq;
	unsigned int AmpCoh, AmpNoncoh[DFT_NUMBER], MaxAmp;
	int ExpIncCor;
	unsigned char *NonCoherentData = (unsigned char *)(NonCoherentBuffer);
	int ShiftCoh, ShiftNoncoh, ShiftBit;
	int ExtraShift, Exceed;
	int AmpSumCor;

	NoiseFloor = 0;
	// non-coherent acc
	ExtraShift = 0;
	ExpIncCor = 0;
	if (NoncohCount == 0)	// first round of non-coherent clear NoncohExp and ExpIncPos
	{
		NoncohExp = 0;
		ExpIncPos = 0;
	}
	// determine shift bits for non-coherent result and coherent result, maximum exp in NoncohExp;
	if (NoncohExp > MaxCohExp)
	{
		ShiftCoh = NoncohExp - MaxCohExp;
		ShiftNoncoh = 0;
	}
	else
	{
		ShiftCoh = 0;
		ShiftNoncoh = MaxCohExp - NoncohExp;
		NoncohExp = MaxCohExp;
	}
	for (CorCount = 0; CorCount < MF_CORE_DEPTH; CorCount ++)
	{
		AmpSumCor = 0;
		Exceed = 0;
		MaxAmp = 0;
		MaxFreq = 0;
		for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
		{
			// calculate shifted coherent acc amplitude
			AmpCoh = Amplitude(CoherentBuffer[CorCount][FreqCount]);	// Amp is 10bit unsigned, largest possible value is 704
			AmpCoh >>= (ShiftCoh + MaxCohExp - CoherentBuffer[CorCount][FreqCount].exp);	// shift to appreciate exp
			AmpCoh = (AmpCoh + 1) >> 1;	// round shift one extra bit to reduce Amp to maximum 9bit, maximum value is 352

			// get non coherent acc and shift to appreciate exp
			AmpNoncoh[FreqCount] = (NoncohCount == 0) ? 0 : NonCoherentData[CorCount * 8 + FreqCount];
			ShiftBit = (CorCount < (int)ExpIncPos) ? (ShiftNoncoh + 1) : ShiftNoncoh;
			if (ShiftBit)
				AmpNoncoh[FreqCount] = ROUND_SHIFT_RAW(AmpNoncoh[FreqCount], ShiftBit);

			AmpNoncoh[FreqCount] += AmpCoh;	// accumulate coherent amplitude, largest possible value may exceed 511
			if (ExtraShift)
				AmpNoncoh[FreqCount] = (AmpNoncoh[FreqCount] + 1) >> 1;
			if (AmpNoncoh[FreqCount] & 0x200)	// clip to maximum 510, this is rare case
				AmpNoncoh[FreqCount] = 510;
			// detect overflow (larger than 255)
			if (AmpNoncoh[FreqCount] & 0x100)
				Exceed = 1;
			if (AmpNoncoh[FreqCount] > MaxAmp)
			{
				MaxAmp = AmpNoncoh[FreqCount];
				MaxFreq = FreqCount;
			}
		}
		// if any frequency bin acc result exceed 255, do extra shift
		if (Exceed)
		{
			ExtraShift = 1;
			NoncohExp ++;
			ExpIncCor = CorCount;
			for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
				AmpNoncoh[FreqCount] = (AmpNoncoh[FreqCount] + 1) >> 1;	// round shift
			NoiseFloor >>= 1;	// compensate existing noise floor
			MaxAmp = (MaxAmp + 1) >> 1;
		}
		// write back noncoh acc result
		for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
			AmpSumCor += (NonCoherentData[CorCount * 8 + FreqCount] = AmpNoncoh[FreqCount]);

		if (fp_out[INTERMEDIATE_RESULT_NONCOH_ACC])
		{
			for (FreqCount = DFT_NUMBER-1; FreqCount >= 0; FreqCount --)
				fprintf(fp_out[INTERMEDIATE_RESULT_NONCOH_ACC], "%02x", NonCoherentData[CorCount*8+FreqCount]);
			fprintf(fp_out[INTERMEDIATE_RESULT_NONCOH_ACC], "\n");
		}

		AmpSumCor >>= 3;	// to simplify, use truncate instead of round shift, this will introduce less than 3% loss for strong peak and less than 0.5% for weak peak
		if (NoncohCount == (NonCoherentNumber - 1) && CodeRoundCount == (CodeSpan / (FULL_LENGTH ? 3 : 1) - 1) && (StrideCount == StrideNumber))		// last round
			NoiseFloor += AmpSumCor;

		// insert the maximum amplitude value within frequency bins to peak sorter
		InsertPeak(MaxAmp, NoncohExp, CorCount, MaxFreq);
	}
	ExpIncPos = ExpIncCor;
}

void CAcqEngine::DoNonCoherentSum()
{
	int i;
	int SegmentCount, CorCount, FreqCount;
	unsigned int CohCount, NoncohCount;
	complex_int InputSample;
	complex_int CorResult[MF_CORE_DEPTH];
	complex_int CorOutput;
	complex_int DftFactorMag[DFT_NUMBER/2], MulCos, MulSin, MulAdd, MulSub;
	int sign_cos[DFT_NUMBER/2], sign_sin[DFT_NUMBER/2];
	complex_exp10 CorData;
	unsigned int MaxExp;
	unsigned int CoherentBufferData;

	CarrierNco = 0;
	PreloadSample();

	for (NoncohCount = 0; NoncohCount < NonCoherentNumber; NoncohCount ++)
	{
		DftNco = 0;		// reset DFT NCO for each coherent round
		MaxExp = 0;
		for (CohCount = 0; CohCount < CoherentNumber;)
		{
			GetDftFactor(DftFactorMag, sign_cos, sign_sin);
			for (SegmentCount = 0; SegmentCount < (FULL_LENGTH ? 1 : 3); SegmentCount ++)
			{
				MatchFilterCore(MF_CORE_DEPTH, CorResult);
				for (i = 0; i < MF_CORE_DEPTH; i ++)
					CorResult[i] += FULL_LENGTH ? complex_int(512, 512) : ((SegmentCount == 2) ? complex_int(192, 192) : complex_int(160, 160));

				if (fp_out[INTERMEDIATE_RESULT_MF_OUTPUT_DEC])
					for (i = 0; i < MF_CORE_DEPTH; i ++)
						fprintf(fp_out[INTERMEDIATE_RESULT_MF_OUTPUT_DEC], "%6d %6d\n", CorResult[i].real, CorResult[i].imag);

				if (fp_out[INTERMEDIATE_RESULT_MF_OUTPUT_HEX])
					for (i = 0; i < MF_CORE_DEPTH; i ++)
						fprintf(fp_out[INTERMEDIATE_RESULT_MF_OUTPUT_HEX], "%04x%04x\n", CorResult[i].real & 0xffff, CorResult[i].imag & 0xffff);

				for (CorCount = 0; CorCount < MF_CORE_DEPTH; CorCount ++)
				{
					if (CohCount == 0)	// for the first round of coherent sum, DFT twiddle factor are all 1
					{
						CorOutput = CorResult[CorCount];
						if (SegmentCount != 0)
							CorData = CoherentBuffer[CorCount][0] + CorOutput;
						else
							CorData = CorOutput;
						for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
							CoherentBuffer[CorCount][FreqCount] = CorData;	// write in coherent buffer with same value

						if (CoherentBuffer[CorCount][0].exp > (int)MaxExp)
								MaxExp = CoherentBuffer[CorCount][0].exp;

					}
					else	// for other round of coherent sum
					{
						for (FreqCount = 0; FreqCount < DFT_NUMBER/2; FreqCount ++)
						{
							CorOutput = CorResult[CorCount];
							// multiply the four products
							MulCos = CorOutput * DftFactorMag[FreqCount].real;
							MulSin = CorOutput * DftFactorMag[FreqCount].imag;
							// drop 3LSB and use unbiased shift
							MulCos.real >>= 3;
							MulCos.imag >>= 3;
							MulSin.real >>= 3;
							MulSin.imag >>= 3;
							MulCos.real = CONVERGENT_ROUND_SHIFT(MulCos.real, 6);
							MulCos.imag = CONVERGENT_ROUND_SHIFT(MulCos.imag, 6);
							MulSin.real = CONVERGENT_ROUND_SHIFT(MulSin.real, 6);
							MulSin.imag = CONVERGENT_ROUND_SHIFT(MulSin.imag, 6);
							// calculate positive frequency (MulAdd) result and negative frequency (MulSub) result
							MulAdd.real = (sign_cos[FreqCount] ? (-MulCos.real) : MulCos.real) + (sign_sin[FreqCount] ? MulSin.imag : (-MulSin.imag));
							MulAdd.imag = (sign_cos[FreqCount] ? (-MulCos.imag) : MulCos.imag) + (sign_sin[FreqCount] ? (-MulSin.real) : MulSin.real);
							MulSub.real = (sign_cos[FreqCount] ? (-MulCos.real) : MulCos.real) + (sign_sin[FreqCount] ? (-MulSin.imag) : MulSin.imag);
							MulSub.imag = (sign_cos[FreqCount] ? (-MulCos.imag) : MulCos.imag) + (sign_sin[FreqCount] ? MulSin.real : (-MulSin.real));

							// accumulate cosine part
							CoherentBuffer[CorCount][4+FreqCount] += MulAdd;
							CoherentBuffer[CorCount][3-FreqCount] += MulSub;
							// calculate maximum exp
							if (CoherentBuffer[CorCount][4+FreqCount].exp > (int)MaxExp)
								MaxExp = CoherentBuffer[CorCount][4+FreqCount].exp;
							if (CoherentBuffer[CorCount][3-FreqCount].exp > (int)MaxExp)
								MaxExp = CoherentBuffer[CorCount][3-FreqCount].exp;
						}
					}
				}

				if (fp_out[INTERMEDIATE_RESULT_COH_ACC])
					for (CorCount = 0; CorCount < MF_CORE_DEPTH; CorCount ++)
					{
						for (FreqCount = DFT_NUMBER-1; FreqCount >= 0; FreqCount --)
						{
							CoherentBufferData = ((CoherentBuffer[CorCount][FreqCount].real & 0x3ff) << 14) + ((CoherentBuffer[CorCount][FreqCount].imag & 0x3ff) << 4) + (CoherentBuffer[CorCount][FreqCount].exp & 0xf);
							fprintf(fp_out[INTERMEDIATE_RESULT_COH_ACC], "%06x", CoherentBufferData);
						}
						fprintf(fp_out[INTERMEDIATE_RESULT_COH_ACC], "\n");
					}

				// load next segment data
				memcpy(AcqSamples, AcqSamples + MF_CORE_DEPTH, sizeof(complex_int) * MF_CORE_DEPTH);
				LoadSample();
				LoadCode();
			}
			CohCount ++;
			DftNco += DftFreq; 
		}
		if (fp_out[INTERMEDIATE_RESULT_LAST_COH_ACC])
			for (CorCount = 0; CorCount < MF_CORE_DEPTH; CorCount ++)
			{
				for (FreqCount = DFT_NUMBER-1; FreqCount >= 0; FreqCount --)
				{
					CoherentBufferData = ((CoherentBuffer[CorCount][FreqCount].real & 0x3ff) << 14) + ((CoherentBuffer[CorCount][FreqCount].imag & 0x3ff) << 4) + (CoherentBuffer[CorCount][FreqCount].exp & 0xf);
					fprintf(fp_out[INTERMEDIATE_RESULT_LAST_COH_ACC], "%06x", CoherentBufferData);
				}
				fprintf(fp_out[INTERMEDIATE_RESULT_LAST_COH_ACC], "\n");
			}

		NonCoherentAcc(MaxExp, NoncohCount);
		if (PeakFound() && EarlyTerminate)
			break;
	}
}

void CAcqEngine::InsertPeak(int Amp, int Exp, int PartialCorPos, int PartialFreq)
{
	PeakData Peak;

	Peak.Amp = Amp;
	Peak.Exp = Exp;
	Peak.PhasePos = CodeRoundCount * MF_CORE_DEPTH + PartialCorPos;
	Peak.FreqPos = (StrideOffset << 3) + PartialFreq;

	PeakSorter.InsertValue(Peak);
	if (fp_out[INTERMEDIATE_RESULT_INSERT_PEAK])
		fprintf(fp_out[INTERMEDIATE_RESULT_INSERT_PEAK], "%3d %2d %5d %3d\n", Peak.Amp, Peak.Exp, Peak.PhasePos, Peak.FreqPos);
	if (fp_out[INTERMEDIATE_RESULT_MAX_PEAKS])
		fprintf(fp_out[INTERMEDIATE_RESULT_MAX_PEAKS], "%2d %3d %5d %3d %3d %5d %3d %3d %5d %3d\n", PeakSorter.Peaks[0].Exp,
		PeakSorter.Peaks[0].Amp, PeakSorter.Peaks[0].PhasePos, PeakSorter.Peaks[0].FreqPos,
		PeakSorter.Peaks[1].Amp, PeakSorter.Peaks[1].PhasePos, PeakSorter.Peaks[1].FreqPos,
		PeakSorter.Peaks[2].Amp, PeakSorter.Peaks[2].PhasePos, PeakSorter.Peaks[2].FreqPos);
}

int CAcqEngine::PeakFound()
{
	unsigned int PeakThreshold;

	PeakThreshold = PeakSorter.Peaks[2].Amp + (PeakSorter.Peaks[2].Amp >> 3) + 1 +
		            ((PeakRatioTh & 1) ? (PeakSorter.Peaks[2].Amp >> 3) : 0) +
		            ((PeakRatioTh & 2) ? (PeakSorter.Peaks[2].Amp >> 2) : 0) +
		            ((PeakRatioTh & 4) ? (PeakSorter.Peaks[2].Amp >> 1) : 0);
	// early terminate acquisition if amplitude of maximum peak larger than threshold
	Success = (PeakSorter.Peaks[0].Amp >= PeakThreshold) ? 1 : 0;

	return Success;
}

void CAcqEngine::SearchOneChannel()
{
	int CodeRoundNumber = CodeSpan / (FULL_LENGTH ? 3 : 1);
	int i, k;
	unsigned char *NonCoherentData = (unsigned char *)(NonCoherentBuffer);

	Success = 0;

	PeakSorter.Clear();

	for (StrideCount = 1; StrideCount <= StrideNumber; StrideCount ++)
	{
		StrideOffset = (StrideCount >> 1);
		if (StrideCount & 1)
			StrideOffset = ~StrideOffset;
		StrideOffset += (StrideCount & 1);
		CarrierFreq = CenterFreq + StrideInterval * StrideOffset;

		for (CodeRoundCount = 0; CodeRoundCount < CodeRoundNumber; CodeRoundCount ++)
		{
			InitPrnGen();
			LoadCode();
			DoNonCoherentSum();

			if (fp_out[INTERMEDIATE_RESULT_LAST_NONCOH_ACC])
			{
				for (i = 0; i < MF_CORE_DEPTH; i ++)
					for (k = 0; k < 8; k ++)
						fprintf(fp_out[INTERMEDIATE_RESULT_LAST_NONCOH_ACC], "%d\n", NonCoherentData[i*8+k] << ((i < (int)ExpIncPos) ? (NoncohExp) : (NoncohExp + 1)));
			}

			if (Success && EarlyTerminate)
				return;
		}
	}
}

void CAcqEngine::DoAcquisition()
{
	unsigned int i;

	for (i = 0; i < ChannelNumber; i ++)
	{
		// fill in config registers
		StrideNumber = EXTRACT_UINT(ChannelConfig[i][0], 0, 6);
		CoherentNumber = EXTRACT_UINT(ChannelConfig[i][0], 8, 6);
		NonCoherentNumber = EXTRACT_UINT(ChannelConfig[i][0], 16, 7);
		PeakRatioTh = EXTRACT_UINT(ChannelConfig[i][0], 24, 3);
		EarlyTerminate = EXTRACT_UINT(ChannelConfig[i][0], 27, 1);
		CenterFreq = EXTRACT_INT(ChannelConfig[i][1], 0, 20) << 12;
		Svid = EXTRACT_UINT(ChannelConfig[i][1], 24, 6);
		PrnSelect = EXTRACT_UINT(ChannelConfig[i][1], 30, 2);
		CodeSpan = EXTRACT_UINT(ChannelConfig[i][2], 0, 5);
		ReadAddress = EXTRACT_UINT(ChannelConfig[i][2], 8, 5);
		DftFreq = EXTRACT_UINT(ChannelConfig[i][2], 20, 11);
		StrideInterval = EXTRACT_UINT(ChannelConfig[i][3], 0, 22);

		// Do searching
		SearchOneChannel();

		// write back result
		NoiseFloor >>= (PeakSorter.Peaks[0].Exp - NoncohExp);	// adjust noise floor exp to be same as peaks
		ChannelConfig[i][4] = (Success << 31) | (PeakSorter.Peaks[0].Exp << 24) | (NoiseFloor & 0x7ffff);
		ChannelConfig[i][5] = (PeakSorter.Peaks[0].Amp << 24) | ((PeakSorter.Peaks[0].FreqPos & 0x1ff) << 15) | PeakSorter.Peaks[0].PhasePos;
		ChannelConfig[i][6] = (PeakSorter.Peaks[1].Amp << 24) | ((PeakSorter.Peaks[1].FreqPos & 0x1ff) << 15) | PeakSorter.Peaks[1].PhasePos;
		ChannelConfig[i][7] = (PeakSorter.Peaks[2].Amp << 24) | ((PeakSorter.Peaks[2].FreqPos & 0x1ff) << 15) | PeakSorter.Peaks[2].PhasePos;
	}
}

unsigned int CAcqEngine::Amplitude(complex_exp10 data)
{
	unsigned int max, min, amp;

	max = (data.real & 0x200) ? ~data.real : data.real;
	min = (data.imag & 0x200) ? ~data.imag : data.imag;
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

void CAcqEngine::InitPrnGen()
{
	if (Svid == 0)
	{
		PrnGen[PrnSelect]->PhaseInit(0);
		return;
	}
	switch (PrnSelect)
	{
	case 0:	// for GPS L1CA
		PrnGen[0]->PhaseInit(Svid < 52 ? GpsInit[Svid-1] : 0);
		break;
	case 1:	// for memory code
		PrnGen[1]->PhaseInit(((49 + Svid) << 6) + 0xc0000004);
		break;
	case 2:	// for BDS B1C
		PrnGen[2]->PhaseInit(B1CInit[Svid-1]);
		break;
	case 3:	// for GPS L1C
		PrnGen[3]->PhaseInit(L1CInit[Svid-1]);
		break;
	}
}

unsigned int CAcqEngine::ReadSample()
{
	if (ReadPointer >= AE_BUFFER_SIZE)
		ReadPointer = 0;
	return (unsigned int)AEBuffer[ReadPointer++];
}

int CAcqEngine::WriteSample(int Length, unsigned char Sample[])
{
	int i;

	for (i = 0; i < Length && WritePointer < AE_BUFFER_SIZE; i ++)
		AEBuffer[WritePointer ++] = Sample[i];

	Filling = (WritePointer < AE_BUFFER_SIZE) ? 1 : 0;
	return !Filling;
}
