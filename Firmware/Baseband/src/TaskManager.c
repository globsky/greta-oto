//----------------------------------------------------------------------
// TaskManager.c:
//   Task management functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
#include "TaskQueue.h"
#include "HWCtrl.h"
#include "PlatformCtrl.h"
#include "TaskManager.h"
#include "AEManager.h"

TASK_QUEUE RequestTask;
TASK_ITEM RequestItems[32];
U32 RequestBuffer[1024];
TASK_QUEUE BasebandTask;
TASK_ITEM BasebandItems[32];
U32 BasebandBuffer[1024];
TASK_QUEUE PostMeasTask;
TASK_ITEM PostMeasItems[32];
U32 PostMeasBuffer[1024];
TASK_QUEUE InputOutputTask;
TASK_ITEM InputOutputItems[8];
U32 InputOutputBuffer[1024];

U32 EventBaseband, EventPostMeas, EventInputOutput;

U32 ReqPendingFlag;
ConditionFunction ConditionFunc[MAX_REQ_WAIT_TASK];
WaitRequestFunction WaitRequestFunc[MAX_REQ_WAIT_TASK];

void TaskProcThread(void *Param);

//*************** Initialize Task manager ****************
//* this function is called at initialization stage to initialize tasks
// Parameters:
//   none
// Return value:
//   none
void TaskInitialize()
{
	EventBaseband = EventCreate();
	EventPostMeas = EventCreate();
	EventInputOutput = EventCreate();
	InitTaskQueue(&RequestTask, RequestItems, 32, RequestBuffer, sizeof(RequestBuffer), 0);
	InitTaskQueue(&BasebandTask, BasebandItems, 32, BasebandBuffer, sizeof(BasebandBuffer), EventBaseband);
	InitTaskQueue(&PostMeasTask, PostMeasItems, 32, PostMeasBuffer, sizeof(PostMeasBuffer), EventInputOutput);
	InitTaskQueue(&InputOutputTask, InputOutputItems, 8, InputOutputBuffer, sizeof(InputOutputBuffer), EventInputOutput);
	CreateThread(TaskProcThread, 0, &BasebandTask);
	CreateThread(TaskProcThread, 1, &PostMeasTask);
	CreateThread(TaskProcThread, 2, &InputOutputTask);
	ReqPendingFlag = 0;
	ConditionFunc[0] = AcqBufferReachTh;
	WaitRequestFunc[0] = StartAcquisition;
}

//*************** Add a task to designated task queue ****************
//* this function is a task function
//   TaskType: type of task queue to add
//   TaskFunc: pointer to task function
//   Param: pointer to parameter passed to task function
//   ParamSize: size of parameter in bytes
// Return value:
//   return none zero if success
int AddToTask(int TaskType, TaskFunction TaskFunc, void *Param, int ParamSize)
{
	int ReturnValue = 0;

	switch (TaskType)
	{
	case TASK_REQUEST:
		ReturnValue = AddTaskToQueue(&RequestTask, TaskFunc, Param, ParamSize);
		SetRequestCount(1);	// set request count to 1 to enable an immediate request interrupt
		break;
	case TASK_BASEBAND:
		ReturnValue = AddTaskToQueue(&BasebandTask, TaskFunc, Param, ParamSize);
		EventSet(EventBaseband);
		break;
	case TASK_POSTMEAS:
		ReturnValue = AddTaskToQueue(&PostMeasTask, TaskFunc, Param, ParamSize);
		EventSet(EventBaseband);
		break;
	case TASK_INOUT:
		ReturnValue = AddTaskToQueue(&InputOutputTask, TaskFunc, Param, ParamSize);
		EventSet(EventBaseband);
		break;
	}

	return ReturnValue;
}

//*************** Add a wait task request task queue ****************
// Parameters:
//   TaskType: type of task to add
//   WaitDelayMs: wait how many millisecond to first check condition
// Return value:
//   always 0
int AddWaitRequest(int TaskType, int WaitDelayMs)
{
	ENTER_CRITICAL();
	ReqPendingFlag |= (1 << TaskType);	// set enable flag
	EXIT_CRITICAL();
	if (!GetRequestCount())	// if request count not set, set a counter
		SetRequestCount(WaitDelayMs);

	return 0;
}

//*************** Process request task queue ****************
//* this function should be called at request ISR
// Parameters:
//   none
// Return value:
//   none
void DoRequestTask()
{
	int i;
	int SetNewRequest = 0;

	if (ReqPendingFlag)	// scan wait requests
	{
		for (i = 0; i < MAX_REQ_WAIT_TASK; i ++)
		{
			if (!(ReqPendingFlag & (1 << i)))
				continue;
			if (ConditionFunc[i]())	// meet condition, do task and clear wait enable flag
			{
				WaitRequestFunc[i]();
				ENTER_CRITICAL();
				ReqPendingFlag &= ~(1 << i);	// clear enable flag
				EXIT_CRITICAL();
			}
			else
				SetNewRequest = 1;	// set a new round request wait
		}
	}
	if (SetNewRequest)
		SetRequestCount(REQUEST_SCAN_INTERVAL);
	DoTaskQueue(&RequestTask);	// process immediate request
}

//*************** Task process thread ****************
//* this function the thread function to process task
// Parameters:
//   Param: pointer to corresponding task queue
// Return value:
//   none
void TaskProcThread(void *Param)
{
	PTASK_QUEUE TaskQueue = (PTASK_QUEUE)Param;

	while (1)
	{
		EventWait(TaskQueue->Event);
		DoTaskQueue(TaskQueue);
	}
}

//*************** Do all tasks ****************
//* this function only called in PC platform
// Parameters:
//   none
// Return value:
//   none
void DoAllTasks()
{
	DoTaskQueue(&BasebandTask);
	DoTaskQueue(&PostMeasTask);
	DoTaskQueue(&InputOutputTask);
}
