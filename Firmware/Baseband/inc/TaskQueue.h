//----------------------------------------------------------------------
// TaskQueue.h:
//   Task queue management
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __TASK_QUEUE_H__
#define __TASK_QUEUE_H__

#include "CommonDefines.h"

typedef int (*TaskFunction) (void *param);

typedef struct tag_TASK_ITEM
{
	TaskFunction CallbackFunction;
	void *ParamAddr;	// address of parameter in buffer (align to DWORD)
	int ParamSize;	// size of parameter (in DWORD)
	struct tag_TASK_ITEM *pNextItem;	// pointer to next item in link list
} TASK_ITEM, *PTASK_ITEM;

typedef struct
{
	PTASK_ITEM TaskItemArray;
	int ItemNumber;
	U32 *ParamBuffer;
	int BufferSize;
	int ReadPosition;
	int WritePosition;
	U32 Event;
	PTASK_ITEM AvailableQueue;	// pointer to available queue list
	PTASK_ITEM WaitQueue;		// pointer to wait queue list
	PTASK_ITEM QueueTail;		// pointer to last item in wait list
} TASK_QUEUE, *PTASK_QUEUE;

void InitTaskQueue(PTASK_QUEUE TaskQueue, TASK_ITEM ItemArray[], int ItemNumber, U32 *ParamBuffer, int BufferSize, U32 Event);
int AddTaskToQueue(PTASK_QUEUE TaskQueue, TaskFunction TaskFunc, void *Param, int ParamSize);
int DoTaskQueue(PTASK_QUEUE TaskQueue);

#endif	// __TASK_QUEUE_H__
