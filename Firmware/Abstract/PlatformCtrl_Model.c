//----------------------------------------------------------------------
// PlatformCtrl_Model.cpp:
//   Implementation of OS and platform related functions using C model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>

void EnableInt() {}
void DisableInt() {}

#if defined _MSC_VER	// implementation of __builtin_xxx in Visual Studio

int __builtin_popcount(unsigned int data)
{
	data = (data & 0x55555555) + ((data >> 1) & 0x55555555);
	data = (data & 0x33333333) + ((data >> 2) & 0x33333333);
	data = (data & 0x0f0f0f0f) + ((data >> 4) & 0x0f0f0f0f);
	data = (data & 0x00ff00ff) + ((data >> 8) & 0x00ff00ff);
	data = (data & 0x0000ffff) + ((data >> 16) & 0x0000ffff);
	return data;
}

int __builtin_clz(unsigned int data)
{
	if (data == 0)
		return 32;
	data |= data >> 1;
	data |= data >> 2;
	data |= data >> 4;
	data |= data >> 8;
	data |= data >> 16;
	return 32 - __builtin_popcount(data);
}

#endif
