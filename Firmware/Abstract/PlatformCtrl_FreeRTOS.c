//----------------------------------------------------------------------
// PlatformCtrl_FreeRTOS.cpp:
//   Implementation of OS and platform related functions in FreeRTOS
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "PlatformCtrl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#define BASEBAND_STACK_SIZE 4096
#define POST_MEAS_STACK_SIZE 8192
#define INOUT_STACK_SIZE 4096
#define HIGHEST_PRIORITY 10	// highest priority for GNSS

void EnableInt() {}	// will not call this function directly
void DisableInt() {}	// will not call this function directly
void ENTER_CRITICAL() { taskENTER_CRITICAL(); }
void EXIT_CRITICAL() { taskEXIT_CRITICAL(); }

int StackSizes[] = {BASEBAND_STACK_SIZE, POST_MEAS_STACK_SIZE, INOUT_STACK_SIZE };

void CreateThread(ThreadFunction Thread, int Priority, void *Param)
{
	xTaskCreate(Thread, "", StackSizes[Priority], Param, HIGHEST_PRIORITY - Priority, NULL);
}

U32 EventCreate() { return (U32)xEventGroupCreate(); }

void EventSet(U32 Event)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
  xResult = xEventGroupSetBitsFromISR((EventGroupHandle_t)Event, 0x01, &xHigherPriorityTaskWoken);
}

void EventWait(U32 Event)
{
	xEventGroupWaitBits((EventGroupHandle_t)Event, 0x01,  pdTRUE, pdFALSE, portMAX_DELAY);
}

//*************** Load parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file read
//* in real system, read from flash or host
// Parameters:
//   Buffer: address to store load parameters
int LoadParameters(int Offset, void *Buffer, int Size)
{
	int ReturnValue = 0;
	// TODO: call corresponding device driver function to load data from non-volatile memory
	return ReturnValue;
}

//*************** Save parameter (ephemeris/almanac, receiver position etc.) ****************
//* in PC platform, this is a file write
//* in real system, write to flash or host
// Parameters:
//   Buffer: address to store load parameters
void SaveParameters(int Offset, void *Buffer, int Size)
{
	// TODO: call corresponding device driver function to save data to non-volatile memory
}
