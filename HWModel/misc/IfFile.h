//----------------------------------------------------------------------
// IfFile.h:
//   IF data file input class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __IF_FILE_H__
#define __IF_FILE_H__

#include <stdio.h>
#include "CommonOps.h"

class CIfFile
{
public:
	CIfFile();
	~CIfFile();
	int OpenIfFile(char *FileName);
	void CloseIfFile();

	FILE *fpIfFile;
	
	int CIfFile::ReadFile(int Count, complex_int Data[]);
};

#endif //__IF_FILE_H__
