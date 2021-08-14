//----------------------------------------------------------------------
// ComplexNumber.h:
//   complex number operator implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <math.h>
#include "ComplexNumber.h"

complex_number::complex_number(double real_part, double imag_part) : real(real_part), imag(imag_part)
{
}

complex_number complex_number::operator + (complex_number data)
{
	complex_number result = *this;
	result += data;
	return result;
}

void complex_number::operator += (complex_number data)
{
	this->real += data.real;
	this->imag += data.imag;
}

complex_number complex_number::operator - (complex_number data)
{
	complex_number result = *this;
	result -= data;
	return result;
}

void complex_number::operator -= (complex_number data)
{
	this->real -= data.real;
	this->imag -= data.imag;
}

complex_number complex_number::operator * (complex_number data)
{
	complex_number result = *this;
	result *= data;
	return result;
}

void complex_number::operator *= (complex_number data)
{
	complex_number temp = *this;

	this->real = temp.real * data.real - temp.imag * data.imag;
	this->imag = temp.real * data.imag + temp.imag * data.real;
}

double complex_number::abs()
{
	return sqrt(this->real * this->real + this->imag * this->imag);
}
