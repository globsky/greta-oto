//----------------------------------------------------------------------
// InitSet.h:
//   Declaration of initial value for PRN generator
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef _INIT_SET_H_
#define _INIT_SET_H_

#ifdef __cplusplus
extern "C" {
#endif
extern unsigned int CAPrnInit[33];
extern unsigned int WaasPrnInit[19];
extern unsigned int L5IInit[38];
extern unsigned int L5QInit[38];
extern unsigned int E5aIInit[50];
extern unsigned int E5aQInit[50];
extern unsigned int B1CDataInit[63];
extern unsigned int B1CPilotInit[63];
extern unsigned int L1CDataInit[63];
extern unsigned int L1CPilotInit[63];
extern unsigned int B2aDataInit[63];
extern unsigned int B2aPilotInit[63];
#ifdef __cplusplus
}
#endif

#endif