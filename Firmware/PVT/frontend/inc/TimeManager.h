#ifndef __TIME_MANAGER_H__
#define __TIME_MANAGER_H__

#include "DataTypes.h"

extern RECEIVER_TIME GnssTime;

void TimeInitialize();
void UpdateReceiverTime(unsigned int TickCount, int RcvrIntervalMs);
int SetReceiverTime(U8 FreqID, int WeekNumber, int CurWeekMs, unsigned int TickCount);
int GetReceiverWeekMs(U8 FreqID, unsigned int TickCount);
int GetReceiverWeekNumber(U8 FreqID);
int ReceiverWeekMsValid();
int ReceiverWeekNumberValid();

#endif //__TIME_MANAGER_H__
