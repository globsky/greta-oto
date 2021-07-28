//----------------------------------------------------------------------
// TaskQueue.c:
//   Task queue management
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <string.h>

#include "CommonDefines.h"
#include "PlatformCtrl.h"
#include "TaskQueue.h"

static void ReleaseWaitItem(PTASK_QUEUE TaskQueue);

//*************** Initial task queue ****************
// Parameters:
//   TaskQueue: pointer to task queue structure
//   ItemArray: task item array
//   ItemNumber: size of task item array
//   ParamBuffer: task parameter buffer
//   BufferSize: size of task parameter buffer in BYTE
// Return value:
//   none
void InitTaskQueue(PTASK_QUEUE TaskQueue, TASK_ITEM ItemArray[], int ItemNumber, U32 *ParamBuffer, int BufferSize)
{
	int i;
	
	TaskQueue->TaskItemArray = ItemArray;
	TaskQueue->ItemNumber = ItemNumber;
	TaskQueue->ParamBuffer = ParamBuffer;
	TaskQueue->BufferSize = BufferSize / 4;	// convert to DWORD size
	// read/write position at the beginning of ParamBuffer
	TaskQueue->ReadPosition = TaskQueue->WritePosition = 0;
	// initial link list
	for (i = 0; i < ItemNumber - 1; i ++)
		ItemArray[i].pNextItem = &ItemArray[i+1];
	ItemArray[ItemNumber].pNextItem = 0;
	// put link list in empty queue and wait queue as empty
	TaskQueue->AvailableQueue = ItemArray;
	TaskQueue->WaitQueue = TaskQueue->QueueTail = 0;
}

//*************** Add one task to task queue ****************
// Parameters:
//   TaskQueue: pointer to task queue structure
//   TaskFunction: pointer to task function
//   Param: pointer to parameter passed to task function
//   ParamSize: size of parameter in bytes
// Return value:
//   return none zero if success
int AddTaskToQueue(PTASK_QUEUE TaskQueue, TaskFunction TaskFunction, void *Param, int ParamSize)
{
	PTASK_ITEM NewTask;
	int ParamSpace = (ParamSize + 3) / 4;	// convert to DWORD
	void *ParamPointer;
	int NewWritePosition;

	// determine whether there is available task
	if (TaskQueue->AvailableQueue == NULL)
		return 0;
	
	ENTER_CRITICAL();
	// determine whether buffer space is enough to hold parameter
	if (TaskQueue->WaitQueue == 0)	// empty task queue
	{
		// write and read pointer can start from beginning
		TaskQueue->ReadPosition = TaskQueue->WritePosition = 0;
		if (TaskQueue->BufferSize >= ParamSpace)
		{
			ParamPointer = TaskQueue->ParamBuffer;
			NewWritePosition = ParamSpace;
		}
		else
		{
			EXIT_CRITICAL();
			return 0;
		}
	}
	else if (TaskQueue->WritePosition > TaskQueue->ReadPosition)	// write position does not roll back
	{
		if ((TaskQueue->BufferSize - TaskQueue->WritePosition) >= ParamSpace)	// space to end can hold parameters
		{
			ParamPointer = TaskQueue->ParamBuffer + TaskQueue->WritePosition;
			NewWritePosition = TaskQueue->WritePosition + ParamSpace;
		}
		else if (TaskQueue->ReadPosition >= ParamSpace)	// space from beginning can hold parameters
		{
			ParamPointer = TaskQueue->ParamBuffer;
			NewWritePosition = 0;
		}
		else
		{
			EXIT_CRITICAL();
			return 0;
		}
	}
	// write position roll back or wait queue empty
	else if (ParamSpace <= (TaskQueue->ReadPosition - TaskQueue->WritePosition))	// enough space between write position and read position
	{
		if ((TaskQueue->BufferSize - TaskQueue->WritePosition) >= ParamSpace)	// space to end can hold parameters
		{
			ParamPointer = TaskQueue->ParamBuffer + TaskQueue->WritePosition;
			NewWritePosition = TaskQueue->WritePosition + ParamSpace;
		}
		else if (TaskQueue->BufferSize >= ParamSpace)	// space from beginning can hold parameters
		{
			ParamPointer = TaskQueue->ParamBuffer;
			NewWritePosition = 0;
		}
		else
		{
			EXIT_CRITICAL();
			return 0;
		}
	}
	else		
	{
		EXIT_CRITICAL();
		return 0;
	}
	if (NewWritePosition >= TaskQueue->BufferSize)
		TaskQueue->WritePosition = 0;
	else
		TaskQueue->WritePosition = NewWritePosition;
	
	// get a new task item
	NewTask = TaskQueue->AvailableQueue;
	TaskQueue->AvailableQueue = NewTask->pNextItem;
	// add task item to wait queue
	if (TaskQueue->QueueTail)
		TaskQueue->QueueTail->pNextItem = NewTask;
	TaskQueue->QueueTail = NewTask;
	if (TaskQueue->WaitQueue == 0)
		TaskQueue->WaitQueue = NewTask;
	
	// fill content of new task item
	NewTask->CallbackFunction = TaskFunction;
	NewTask->ParamAddr = (void *)ParamPointer;
	NewTask->ParamSize = ParamSize;
	NewTask->pNextItem = 0;
	memcpy(ParamPointer, Param, ParamSize);

	EXIT_CRITICAL();
	return 1;	
}

//*************** Release first item in wait list to available list, release buffer space occupied by task item ****************
// Parameters:
//   TaskQueue: pointer to task queue structure
// Return value:
//   none
void ReleaseWaitItem(PTASK_QUEUE TaskQueue)
{
	U32 *ParamAddrEnd;
	PTASK_ITEM Task = TaskQueue->WaitQueue;

	if (!Task)
		return;
	ENTER_CRITICAL();
	// remove current wait head task from wait list
	TaskQueue->WaitQueue = Task->pNextItem;	// wait queue move to next item
	if (TaskQueue->QueueTail == Task)	// this is the last item in wait queue
	{
		TaskQueue->QueueTail = (PTASK_ITEM)NULL;	// clear queue tail pointer
		TaskQueue->ReadPosition = TaskQueue->WritePosition = 0;	// initial read/write pointer from beginning
	}
	// put current item at the head of available link list
	Task->pNextItem = TaskQueue->AvailableQueue;
	TaskQueue->AvailableQueue = Task;
	// release parameter space
	ParamAddrEnd = (U32 *)Task->ParamAddr + (Task->ParamSize) / 4;
	TaskQueue->ReadPosition = ParamAddrEnd - TaskQueue->ParamBuffer;
	if (TaskQueue->ReadPosition >= TaskQueue->BufferSize)
		TaskQueue->ReadPosition = 0;

	EXIT_CRITICAL();
}

//*************** Do tasks in wait list until it is empty ****************
// Parameters:
//   TaskQueue: pointer to task queue structure
// Return value:
//   number of tasks performed
int DoTaskQueue(PTASK_QUEUE TaskQueue)
{
	PTASK_ITEM Task;
	int TaskNumber = 0;
	
	while ((Task = TaskQueue->WaitQueue) != NULL)
	{
		TaskNumber ++;
		Task->CallbackFunction(Task->ParamAddr);
		ReleaseWaitItem(TaskQueue);
	}
	return TaskNumber;
}
