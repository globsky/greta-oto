#include "CommonOps.h"

#define CODE_RATE_FILTER_STAGE 6

class CRateAdaptor
{
public:
	CRateAdaptor();

	reg_uint CodeRateAdjustNco;		// 24U
	reg_uint CodeRateAdjustRatio;	// 24U
	reg_uint CarrierNco;			// 32U
	reg_uint CarrierFreq;			// 32U
	reg_uint Threshold;				// ?U

	void Reset();
	int DoRateAdaptor(complex_int InputSignal[], int Length, unsigned char OutputSignal[]);
	unsigned char Quant2Bit(complex_int Sample);

	static const complex_int DownConvertTable[64];
	static const int CodeRateFilterCoef[CODE_RATE_FILTER_STAGE/2];

	complex_int CodeRateFilterBuffer[CODE_RATE_FILTER_STAGE];
};
