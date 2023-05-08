//----------------------------------------------------------------------
// TaskManager.h:
//   Task management functions and definitions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#include "CommonDefines.h"
#include "TaskQueue.h"

#define TASK_REQUEST 0
#define TASK_BASEBAND 1
#define TASK_POSTMEAS 2
#define TASK_INOUT 3

#define WAIT_TASK_AE 0

#define REQUEST_SCAN_INTERVAL 5	// check wait request every 5ms
#define MAX_REQ_WAIT_TASK 1
typedef int (*ConditionFunction)(void);
typedef void (*WaitRequestFunction)(void);

void TaskInitialize();
int AddToTask(int TaskType, TaskFunction TaskFunc, void *Param, int ParamSize);
int AddWaitRequest(int TaskType, int WaitDelayMs);
void DoRequestTask();

#endif // __TASK_MANAGER_H__
