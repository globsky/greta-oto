#include <stdio.h>
#include <math.h>
#include <string.h>

#include "RegAddress.h"
#include "InitSet.h"
#include "HWCtrl.h"
extern "C" {
#include "FirmwarePortal.h"
}

void main()
{
	SetInputFile("..\\..\\..\\data\\sim_signal_L1CA.bin");

	FirmwareInitialize();
	EnableRF();
}
