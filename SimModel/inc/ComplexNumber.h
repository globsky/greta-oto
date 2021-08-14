//----------------------------------------------------------------------
// ComplexNumber.h:
//   Definition of complex number and operators
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined (__COMPLEX_NUMBER_H__)
#define __COMPLEX_NUMBER_H__

class complex_number
{
public:
	double real;
	double imag;

	complex_number::complex_number() {};
	complex_number(double real_part, double imag_part);
	complex_number operator + (complex_number data);
	void operator += (complex_number data);
	complex_number operator - (complex_number data);
	void operator -= (complex_number data);
	complex_number operator * (complex_number data);
	void operator *= (complex_number data);
	double abs();
};

#endif //!defined(__COMPLEX_NUMBER_H__)
