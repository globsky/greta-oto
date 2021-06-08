//----------------------------------------------------------------------
// IfFile.cpp:
//   IF data file input class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include "IfFile.h"

CIfFile::CIfFile()
{
	fpIfFile = NULL;
}

CIfFile::~CIfFile()
{
	CloseIfFile();
}

int CIfFile::OpenIfFile(char *FileName)
{
	fpIfFile = fopen(FileName, "rb");
	return (fpIfFile != NULL);
}

void CIfFile::CloseIfFile()
{
	if (fpIfFile)
		fclose(fpIfFile);
	fpIfFile = NULL;
}

// return 1 for read success
// return 0 for file end
int CIfFile::ReadFile(int Count, complex_int Data[])
{
	int i;
	unsigned char *pBuf;

	if (fpIfFile == NULL)
		return 0;

	pBuf = (unsigned char *)malloc(sizeof(char) * Count);
	if ((fread(pBuf, 1, Count, fpIfFile)) != Count)
			return 0;
	for (i = 0; i < Count; i ++)
	{
		Data[i].real = ((pBuf[i] & 0x70) >> 3) + 1;
		if (pBuf[i] & 0x80)
			Data[i].real = -Data[i].real;
		Data[i].imag = ((pBuf[i] & 0x7) << 1) + 1;
		if (pBuf[i] & 0x8)
			Data[i].imag = -Data[i].imag;
	}
	free(pBuf);

	return 1;
}
