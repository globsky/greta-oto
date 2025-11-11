//----------------------------------------------------------------------
// PlatformCtrl_Model.cpp:
//   Implementation of OS and platform related functions using C model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "PlatformCtrl.h"

static FILE *fp_debug = (FILE *)0;
static FILE *SreamFile[MAX_STREAM_ID] = { 0 };

void CreateThread(ThreadFunction Thread, int Priority, void *Param) {}
void ENTER_CRITICAL() {}
void EXIT_CRITICAL() {}
U32 EventCreate() { return 0; }
void EventSet(U32 Event) {}
void EventWait(U32 Event) {}
U32 MutexCreate() { return 0; }
void MutexTake(U32 Mutex) {}
void MutexGive(U32 Mutex) {}

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

//*************** Load parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file read
//* in real system, read from flash or host
// Parameters:
//   Buffer: address to store load parameters
int LoadParameters(int Offset, void *Buffer, int Size)
{
	FILE *fp;
	int ReturnValue;

	if ((fp = fopen("ParamFile.bin", "rb")) == NULL)
	{
		memset(Buffer, 0, Size);
		return 0;
	}
	fseek(fp, Offset, SEEK_SET);
	ReturnValue = fread(Buffer, 1, Size, fp);
	fclose(fp);

	return ReturnValue;
}

//*************** Save parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file write
//* in real system, write to flash or host
// Parameters:
//   Buffer: address to store load parameters
void SaveParameters(int Offset, void *Buffer, int Size)
{
	FILE *fp;

	if ((fp = fopen("ParamFile.bin", "rb+")) == NULL)
		return;
	fseek(fp, Offset, SEEK_SET);
	fwrite(Buffer, 1, Size, fp);
	fclose(fp);
}

void DebugPrintf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
    
	if (fp_debug != NULL)
		vfprintf(fp_debug, format, args);
    
	va_end(args);
}

void InitStreamPorts()
{
	int i;
	char StreamFileName[32];

	for (i = 0; i < MAX_STREAM_ID; i ++)
	{
		sprintf(StreamFileName, STREAM_FILE_PREFIX, i);
		SreamFile[i] = (USE_STDOUT_AS_STREAM0 && (i == 0)) ? stdout : fopen(StreamFileName, "wb");
	}
	fp_debug = (DEFAULT_DEBUG_OUTPUT_PORT < 0) ? stdout : SreamFile[DEFAULT_DEBUG_OUTPUT_PORT];
}

int WriteStreamPort(int PortNumber, unsigned char *Stream, int Length)
{
	if (PortNumber >= 0 && PortNumber < MAX_STREAM_ID)
		return fwrite(Stream, sizeof(unsigned char), Length, SreamFile[PortNumber]);
	else
		return -1;
}

int PortOpened(int PortNumber)
{
	if (PortNumber >= 0 && PortNumber < MAX_STREAM_ID)
		return SreamFile[PortNumber] != NULL;
	else
		return 0;
}
