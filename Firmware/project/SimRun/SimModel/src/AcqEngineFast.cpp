//----------------------------------------------------------------------
// AcqEngineFast.cpp:
//   Acquisition engine simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <intrin.h>

#include "ConstVal.h"
#include "RegAddress.h"
#include "AcqEngineFast.h"
#include "SatelliteParam.h"
#include "Coordinate.h"
#include "CommonDefines.h"
#include "ComplexNumber.h"
#include "GaussNoise.h"

const double CAcqEngine::Bpsk2PeakValues[160] = {
  0.875000,  0.874878,  0.874512,  0.873901,  0.873047,  0.871948,  0.870605,  0.869019,  0.867188,  0.865112,
  0.862793,  0.860229,  0.857422,  0.854370,  0.851074,  0.847534,  0.843750,  0.839722,  0.835449,  0.830933,
  0.826172,  0.821167,  0.815918,  0.810425,  0.804687,  0.798706,  0.792480,  0.786011,  0.779297,  0.772339,
  0.765137,  0.757690,  0.750000,  0.742188,  0.734375,  0.726563,  0.718750,  0.710938,  0.703125,  0.695313,
  0.687500,  0.679688,  0.671875,  0.664063,  0.656250,  0.648438,  0.640625,  0.632813,  0.625000,  0.617188,
  0.609375,  0.601563,  0.593750,  0.585938,  0.578125,  0.570313,  0.562500,  0.554688,  0.546875,  0.539063,
  0.531250,  0.523438,  0.515625,  0.507813,  0.500000,  0.492188,  0.484375,  0.476563,  0.468750,  0.460937,
  0.453125,  0.445312,  0.437500,  0.429688,  0.421875,  0.414063,  0.406250,  0.398438,  0.390625,  0.382813,
  0.375000,  0.367188,  0.359375,  0.351563,  0.343750,  0.335937,  0.328125,  0.320313,  0.312500,  0.304687,
  0.296875,  0.289063,  0.281250,  0.273438,  0.265625,  0.257813,  0.250000,  0.242249,  0.234619,  0.227112,
  0.219727,  0.212463,  0.205322,  0.198303,  0.191406,  0.184631,  0.177979,  0.171448,  0.165039,  0.158752,
  0.152588,  0.146545,  0.140625,  0.134827,  0.129150,  0.123596,  0.118164,  0.112854,  0.107666,  0.102600,
  0.097656,  0.092834,  0.088135,  0.083557,  0.079102,  0.074768,  0.070557,  0.066467,  0.062500,  0.058655,
  0.054932,  0.051331,  0.047852,  0.044495,  0.041260,  0.038147,  0.035156,  0.032288,  0.029541,  0.026917,
  0.024414,  0.022034,  0.019775,  0.017639,  0.015625,  0.013733,  0.011963,  0.010315,  0.008789,  0.007385,
  0.006104,  0.004944,  0.003906,  0.002991,  0.002197,  0.001526,  0.000977,  0.000549,  0.000244,  0.000061,
};

const double CAcqEngine::Boc2PeakValues[160] = {
  0.588997,  0.588857,  0.588438,  0.587739,  0.586761,  0.585506,  0.583974,  0.582168,  0.580088,  0.577737,
  0.575118,  0.572233,  0.569085,  0.565677,  0.562013,  0.558096,  0.553932,  0.549524,  0.544877,  0.539997,
  0.534888,  0.529558,  0.524011,  0.518256,  0.512298,  0.506145,  0.499806,  0.493288,  0.486601,  0.479753,
  0.472756,  0.465618,  0.458351,  0.450968,  0.443480,  0.435900,  0.428242,  0.420522,  0.412754,  0.404955,
  0.397143,  0.389336,  0.381555,  0.373819,  0.366151,  0.358574,  0.351114,  0.343796,  0.336647,  0.329914,
  0.323820,  0.318358,  0.313518,  0.309282,  0.305625,  0.302520,  0.299932,  0.297823,  0.296150,  0.294869,
  0.293931,  0.293288,  0.292890,  0.292687,  0.292632,  0.292676,  0.292772,  0.292878,  0.292951,  0.292952,
  0.292844,  0.292592,  0.292164,  0.291533,  0.290670,  0.289553,  0.288160,  0.286472,  0.284473,  0.282149,
  0.279490,  0.276574,  0.273493,  0.270247,  0.266839,  0.263271,  0.259546,  0.255664,  0.251630,  0.247445,
  0.243112,  0.238633,  0.234012,  0.229250,  0.224352,  0.219320,  0.214157,  0.208867,  0.203452,  0.197916,
  0.192263,  0.186496,  0.180619,  0.174635,  0.168549,  0.162364,  0.156085,  0.149715,  0.143260,  0.136723,
  0.130108,  0.123422,  0.116669,  0.109960,  0.103407,  0.097015,  0.090789,  0.084734,  0.078856,  0.073158,
  0.067646,  0.062323,  0.057195,  0.052264,  0.047536,  0.043014,  0.038702,  0.034602,  0.030719,  0.027056,
  0.023614,  0.020398,  0.017410,  0.014652,  0.012126,  0.009835,  0.007779,  0.005962,  0.004384,  0.003047,
  0.001951,  0.001098,  0.000488,  0.000122,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,
};

const double CAcqEngine::BasecPdfSegment[257] = {
2.74857687, 2.87310316, 2.91886384, 2.94840912, 2.97086683, 2.98926342, 3.00500069, 3.01884975, 
3.03128290, 3.04261159, 3.05305270, 3.06276368, 3.07186265, 3.08044065, 3.08856943, 3.09630658, 
3.10369909, 3.11078584, 3.11759937, 3.12416722, 3.13051294, 3.13665676, 3.14261631, 3.14840696, 
3.15404222, 3.15953406, 3.16489314, 3.17012893, 3.17524998, 3.18026398, 3.18517788, 3.18999799, 
3.19473005, 3.19937934, 3.20395065, 3.20844842, 3.21287672, 3.21723932, 3.22153970, 3.22578110, 
3.22996653, 3.23409879, 3.23818048, 3.24221407, 3.24620183, 3.25014592, 3.25404837, 3.25791107, 
3.26173581, 3.26552429, 3.26927812, 3.27299880, 3.27668777, 3.28034639, 3.28397597, 3.28757773, 
3.29115285, 3.29470245, 3.29822759, 3.30172929, 3.30520855, 3.30866627, 3.31210338, 3.31552073, 
3.31891913, 3.32229940, 3.32566228, 3.32900853, 3.33233884, 3.33565391, 3.33895439, 3.34224092, 
3.34551413, 3.34877461, 3.35202294, 3.35525968, 3.35848539, 3.36170059, 3.36490579, 3.36810150, 
3.37128820, 3.37446638, 3.37763650, 3.38079901, 3.38395436, 3.38710297, 3.39024528, 3.39338169, 
3.39651261, 3.39963845, 3.40275958, 3.40587641, 3.40898930, 3.41209863, 3.41520476, 3.41830805, 
3.42140887, 3.42450755, 3.42760445, 3.43069991, 3.43379426, 3.43688785, 3.43998100, 3.44307405, 
3.44616731, 3.44926111, 3.45235578, 3.45545164, 3.45854899, 3.46164816, 3.46474947, 3.46785323, 
3.47095975, 3.47406934, 3.47718233, 3.48029901, 3.48341971, 3.48654474, 3.48967441, 3.49280903, 
3.49594893, 3.49909442, 3.50224581, 3.50540343, 3.50856760, 3.51173864, 3.51491688, 3.51810264, 
3.52129627, 3.52449808, 3.52770843, 3.53092765, 3.53415608, 3.53739407, 3.54064198, 3.54390016, 
3.54716898, 3.55044880, 3.55373999, 3.55704293, 3.56035800, 3.56368559, 3.56702610, 3.57037992, 
3.57374748, 3.57712918, 3.58052544, 3.58393671, 3.58736341, 3.59080601, 3.59426495, 3.59774072, 
3.60123378, 3.60474462, 3.60827375, 3.61182168, 3.61538893, 3.61897604, 3.62258356, 3.62621205, 
3.62986209, 3.63353428, 3.63722922, 3.64094754, 3.64468988, 3.64845690, 3.65224929, 3.65606775, 
3.65991299, 3.66378576, 3.66768683, 3.67161698, 3.67557703, 3.67956782, 3.68359022, 3.68764514, 
3.69173349, 3.69585623, 3.70001438, 3.70420894, 3.70844101, 3.71271167, 3.71702208, 3.72137343, 
3.72576697, 3.73020397, 3.73468579, 3.73921382, 3.74378950, 3.74841436, 3.75308998, 3.75781799, 
3.76260012, 3.76743819, 3.77233405, 3.77728969, 3.78230717, 3.78738866, 3.79253642, 3.79775285, 
3.80304045, 3.80840186, 3.81383986, 3.81935739, 3.82495754, 3.83064358, 3.83641897, 3.84228737, 
3.84825266, 3.85431897, 3.86049068, 3.86677245, 3.87316926, 3.87968640, 3.88632954, 3.89310477, 
3.90001859, 3.90707800, 3.91429052, 3.92166428, 3.92920805, 3.93693133, 3.94484442, 3.95295854, 
3.96128592, 3.96983993, 3.97863524, 3.98768796, 3.99701589, 4.00663873, 4.01657837, 4.02685921, 
4.03750861, 4.04855736, 4.06004026, 4.07199692, 4.08447261, 4.09751947, 4.11119794, 4.12557867, 
4.14074491, 4.15679578, 4.17385059, 4.19205471, 4.21158788, 4.23267600, 4.25560854, 4.28076505, 
4.30865721, 4.33999866, 4.37582861, 4.41774754, 4.46841620, 4.53277620, 4.62178913, 4.76981894, 
5.18991397, };

const double CAcqEngine::BasicPdfValues[257] = {
0.00000000, 0.06273775, 0.11134814, 0.15462218, 0.19410336, 0.23097386, 0.26580190, 0.29854928, 
0.32974988, 0.35984054, 0.38839562, 0.41591257, 0.44244322, 0.46809838, 0.49273022, 0.51683361, 
0.53969300, 0.56216823, 0.58388385, 0.60510718, 0.62579594, 0.64558820, 0.66510123, 0.68398391, 
0.70221360, 0.72009675, 0.73729241, 0.75443467, 0.77086422, 0.78689454, 0.80251685, 0.81772397, 
0.83282365, 0.84718246, 0.86142145, 0.87522525, 0.88859311, 0.90182455, 0.91461543, 0.92696793, 
0.93917404, 0.95094177, 0.96255715, 0.97401408, 0.98503319, 0.99562102, 1.00605009, 1.01631567, 
1.02615644, 1.03583355, 1.04509497, 1.05419392, 1.06312697, 1.07189082, 1.08025242, 1.08844804, 
1.09647499, 1.10433069, 1.11180170, 1.11910637, 1.12624266, 1.13301201, 1.13981078, 1.14624887, 
1.15252144, 1.15844992, 1.16439229, 1.17016526, 1.17560539, 1.18088395, 1.18600007, 1.19095298, 
1.19574199, 1.20022878, 1.20469324, 1.20886438, 1.21287956, 1.21685657, 1.22055418, 1.22409519, 
1.22737615, 1.23060871, 1.23368465, 1.23651522, 1.23928319, 1.24189507, 1.24427675, 1.24651254, 
1.24866782, 1.25060833, 1.25240425, 1.25405609, 1.25556444, 1.25692991, 1.25819027, 1.25926748, 
1.26020397, 1.26100054, 1.26165803, 1.26216272, 1.26254914, 1.26279920, 1.26291390, 1.26289427, 
1.26274137, 1.26245631, 1.26204021, 1.26149423, 1.26081958, 1.26001747, 1.25908916, 1.25803593, 
1.25685908, 1.25555996, 1.25413990, 1.25260030, 1.25088714, 1.24910894, 1.24721550, 1.24514168, 
1.24301860, 1.24078478, 1.23836436, 1.23582925, 1.23326557, 1.23050963, 1.22764407, 1.22467065, 
1.22168893, 1.21850828, 1.21512077, 1.21173351, 1.20824725, 1.20466377, 1.20086842, 1.19697359, 
1.19310365, 1.18901868, 1.18484014, 1.18057002, 1.17621031, 1.17176301, 1.16709144, 1.16247241, 
1.15762813, 1.15269943, 1.14768846, 1.14259739, 1.13742834, 1.13202810, 1.12670739, 1.12115551, 
1.11553014, 1.10983356, 1.10406803, 1.09806824, 1.09216975, 1.08603792, 1.07984271, 1.07341188, 
1.06709543, 1.06054494, 1.05393754, 1.04727570, 1.04056189, 1.03361508, 1.02661860, 1.01957502, 
1.01229977, 1.00516846, 0.99780841, 0.99021914, 0.98278271, 0.97512051, 0.96742542, 0.95950663, 
0.95155890, 0.94358500, 0.93539232, 0.92717789, 0.91894450, 0.91049834, 0.90203809, 0.89356657, 
0.88488929, 0.87600862, 0.86732210, 0.85824044, 0.84935876, 0.84008814, 0.83102339, 0.82157675, 
0.81214565, 0.80273310, 0.79314666, 0.78358563, 0.77366453, 0.76397079, 0.75392698, 0.74392464, 
0.73377574, 0.72348605, 0.71325060, 0.70288443, 0.69239386, 0.68178545, 0.67106598, 0.66042520, 
0.64950338, 0.63849207, 0.62757699, 0.61640802, 0.60517282, 0.59387947, 0.58236530, 0.57081349, 
0.55923265, 0.54763144, 0.53585617, 0.52392245, 0.51184635, 0.49979956, 0.48763859, 0.47538034, 
0.46304200, 0.45049650, 0.43791163, 0.42530525, 0.41242424, 0.39956994, 0.38663131, 0.37351033, 
0.36024505, 0.34699261, 0.33355061, 0.32007990, 0.30640375, 0.29258339, 0.27877954, 0.26475464, 
0.25059373, 0.23638125, 0.22195356, 0.20743207, 0.19279004, 0.17796696, 0.16295586, 0.14786041, 
0.13255240, 0.11708251, 0.10136005, 0.08545318, 0.06927541, 0.05282289, 0.03597996, 0.01859698, 
0.00000000, };

const double CAcqEngine::BaseModeValues[20] = {
 3.441000,  5.475000,  7.320000,  9.070000, 10.759000, 12.405000, 14.020000, 15.609000, 17.178000, 18.729000,
20.266000, 21.789000, 23.302000, 24.805000, 26.299000, 27.785000, 29.264000, 30.736000, 32.203000, 33.663000, };

const double CAcqEngine::BaseVariance[20] = {
 1.000000,  1.808210,  2.550646,  3.258418,  3.944646,  4.615832,  5.275755,  5.926844,  6.570767,  7.208724,
 7.841626,  8.470164,  9.094893,  9.716262, 10.334636, 10.950321, 11.563575, 12.174621, 12.783641, 13.390808, };

const double CAcqEngine::ModeSlope[20] = {
 1.984518,  4.178341,  6.544118,  9.064082, 11.726323, 14.517455, 17.429300, 20.456051, 23.591179, 26.827282,
30.162383, 33.593357, 37.111603, 40.715420, 44.398458, 48.172027, 52.024318, 55.951181, 59.951620, 64.020176, };

 const double CAcqEngine::VarianceSlope[20] = {
 5.930173, 11.481949, 16.672399, 21.629395, 26.445471, 31.181710, 35.878070, 40.561959, 45.253160, 49.966676,
54.714287, 59.505624, 64.348871, 69.250976, 74.218293, 79.256443, 84.370713, 89.566063, 94.847255, 100.218799, };

CAcqEngine::CAcqEngine()
{
	Reset();
	memset(ChannelConfig, 0, sizeof(ChannelConfig));
}

CAcqEngine::~CAcqEngine()
{
}

void CAcqEngine::Reset()
{
	EarlyTerminate = 0;
	PeakRatioTh = 3;
}

void CAcqEngine::SetRegValue(int Address, U32 Value)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_AE_CONTROL:
		ChannelNumber = (int)EXTRACT_UINT(Value, 0, 6);
		if (Value & 0x100)
			DoAcquisition();
		break;
	case ADDR_OFFSET_AE_BUFFER_CONTROL:
		BufferThreshold = (int)EXTRACT_UINT(Value, 0, 7);
		break;
	case ADDR_OFFSET_AE_CARRIER_FREQ:
		RateCarrierFreq = Value;
		break;
	case ADDR_OFFSET_AE_CODE_RATIO:
		CodeRateAdjustRatio = EXTRACT_UINT(Value, 0, 24);
		break;
	case ADDR_OFFSET_AE_THRESHOLD:
		Threshold = EXTRACT_UINT(Value, 0, 8);
		break;
	default:
		break;
	}
}

U32 CAcqEngine::GetRegValue(int Address)
{
	Address &= 0xff;

	switch (Address)
	{
	case ADDR_OFFSET_AE_CONTROL:
		return 	ChannelNumber;
	case ADDR_OFFSET_AE_BUFFER_CONTROL:
		return BufferThreshold;
	case ADDR_OFFSET_AE_STATUS:
		return 	0xe0000;	// set finish, full and reach threshold flag
	case ADDR_OFFSET_AE_CARRIER_FREQ:
		return 	RateCarrierFreq;
	case ADDR_OFFSET_AE_CODE_RATIO:
		return 	CodeRateAdjustRatio;
	case ADDR_OFFSET_AE_THRESHOLD:
		return 	Threshold;
	default:
		return 0;
	}
}

#define NONCOH_TABLE_VALUE(N, Table) ((N <= 20) ? Table[N-1] : Table[19] + (Table[19]-Table[18]) * (N-20));
void CAcqEngine::GetNoisePeaks(double NoisePeaks[3])
{
	int RandomValue, Segment;
	double SegmentWidth, k, RamdomBasic;
	double Sigma = SIGMA0 * sqrt((double)CoherentNumber);
	int SampleFactor = ((CoherentNumber <= DFT_NUMBER) ? CoherentNumber : DFT_NUMBER) * StrideNumber * CodeSpan;
	double logn = log((double)SampleFactor);
	double xn, sn2, kxn, ksn, xnn, snn2;
	double lambda2;

	// generate a basic distributed random variable
//	for (int i = 0; i < 500000; i ++) {
	RandomValue = rand();	// a value between 0 and RAND_MAX
	Segment = RandomValue & 0xff;	// 8bit ramdom value as segment
	SegmentWidth = BasecPdfSegment[Segment+1] - BasecPdfSegment[Segment];	// xb-xa
	k = (BasicPdfValues[Segment+1] - BasicPdfValues[Segment]) / SegmentWidth;	// (b-a)/(xb-xa)
	RandomValue = rand();	// a value between 0 and RAND_MAX
	RamdomBasic = BasicPdfValues[Segment] * BasicPdfValues[Segment] + k * RandomValue / (RAND_MAX+1) / 128;	// a^2+2AkR (A=1/256)
	RamdomBasic = (sqrt(RamdomBasic) - BasicPdfValues[Segment]) / k + BasecPdfSegment[Segment];	// (sqrt(a^2+2AkR)-a)/k+xa
//	RamdomBasic = (BasecPdfSegment[Segment+1] - BasecPdfSegment[Segment]) * RandomValue / (RAND_MAX+1) + BasecPdfSegment[Segment];	// evenly distributed within segment
//	printf("%.8f\n", RamdomBasic); } exit(0);

	// calculate model parameters
	xn  = NONCOH_TABLE_VALUE(NonCoherentNumber, BaseModeValues);
	sn2 = NONCOH_TABLE_VALUE(NonCoherentNumber, BaseVariance);
	kxn = NONCOH_TABLE_VALUE(NonCoherentNumber, ModeSlope);
	ksn = NONCOH_TABLE_VALUE(NonCoherentNumber, VarianceSlope);

	// get maximum noise value
	xnn = sqrt(xn * xn + kxn * logn);
	snn2 = 1.0 / (1.0 / sn2 + logn / ksn);
	NoisePeaks[0] = ((RamdomBasic - BaseModeValues[0]) * sqrt(snn2) + xnn) * Sigma;

	// get second and third maximum value
	lambda2 = 1.0 / (LAMBDA_PARAM1 + NonCoherentNumber * LAMBDA_PARAM2);
	lambda2 += LAMBDA_SLOPE / NonCoherentNumber * logn;
	RandomValue = RAND_MAX + 1 - rand();	// a value between 1 and RAND_MAX+1
	NoisePeaks[1] = NoisePeaks[0] + Sigma * log((double)RandomValue / (RAND_MAX+1)) / sqrt(lambda2);
	RandomValue = RAND_MAX + 1 - rand();	// a value between 1 and RAND_MAX+1
	NoisePeaks[2] = NoisePeaks[1] + Sigma * log((double)RandomValue / (RAND_MAX+1)) / sqrt(lambda2) / 1.9;
}

double CAcqEngine::GetSignalPeak(AeBufferSatParam *pSatParam, int &FreqBin, int &Cor)
{
	int PhaseStart, PhaseEnd, PhasePeak, OffsetIndex;
	int StrideCount, StrideOffset;
	double Time2CodeEnd;
	double FreqDiff, FreqDiffMin = 1e5, CarrierFreqMin = CenterFreq;
	double DftTwiddleFreq = DftTwiddlePhase * 500 / PI;
	int StrideOffsetMin, DftBinMin = 0;
	int CohCount, NoncohCount, FreqCount;
	double DftTwiddleFactor, DftPhase;
	double DopplerPhase;
	double FreqFade, Amplitude;
	const double *PeakValues;
	complex_number Value, DftResult[DFT_NUMBER];
	int CurBit = 0;
	double AmplitudeAcc[DFT_NUMBER], AmplitudeMax = 0.0;

	if (pSatParam == NULL)
		return 0.0;

	// determine phase search range from PhaseStart to PhaseEnd
	PhaseStart = (ReadAddress * MF_DEPTH) % PhaseCount;
	PhaseEnd = PhaseStart + MF_DEPTH * CodeSpan;
	// code phase number to PRN code boundary (if less than start search range, set to next boundary)
	Time2CodeEnd = pSatParam->Time2CodeEnd;
	if (Time2CodeEnd < PhaseStart)
		Time2CodeEnd += PhaseCount;
	// if not in search range, there is no signal
	if (Time2CodeEnd < PhaseStart || Time2CodeEnd > PhaseEnd)
		return 0.0;
	// find nearest code phase
	PhasePeak = (int)Time2CodeEnd;
	if ((Time2CodeEnd - PhasePeak) > 0.5 && PhasePeak < (PhaseEnd - 1))
		PhasePeak ++;
	// determine amplitude at peak code phase
	PeakValues = (pSatParam->PrnSelect == 0) ? Bpsk2PeakValues : Boc2PeakValues;
	OffsetIndex = (int)(fabs(Time2CodeEnd - PhasePeak) * 64 + 0.5);
	Amplitude = PeakValues[OffsetIndex] * pSatParam->Amplitude;
	// assign Cor field of signal
	Cor = PhasePeak - PhaseStart;

	// scan the nearest frequency bin
	for (StrideCount = 1; StrideCount <= StrideNumber; StrideCount ++)
	{
		StrideOffset = (StrideCount >> 1);
		if (StrideCount & 1)
			StrideOffset = ~StrideOffset;
		StrideOffset += (StrideCount & 1);
		CarrierFreq = CenterFreq + StrideInterval * StrideOffset;
		FreqDiff = fabs(CarrierFreq - pSatParam->Doppler);
		if (FreqDiff < FreqDiffMin)
		{
			FreqDiffMin = FreqDiff;
			CarrierFreqMin = CarrierFreq;
			StrideOffsetMin = StrideOffset;
		}
	}
	// determine frequency fade and amplitude adjust
	FreqDiff = fabs(PI * (CarrierFreqMin - pSatParam->Doppler) / 1000);
	FreqFade = (FreqDiff > 1e-3) ? (sin(FreqDiff) / FreqDiff) : 1.0;
	Amplitude *= FreqFade;

	// do non-coherent and DFT accumulation loop
	memset(AmplitudeAcc, 0, sizeof(AmplitudeAcc));
	for (NoncohCount = 0; NoncohCount < NonCoherentNumber; NoncohCount ++)
	{
		DopplerPhase = 0;		// reset Doppler phase for each non-coherent round
		DftTwiddleFactor = 0.;		// reset DFT twiddle factor for each non-coherent round
		for (CohCount = 0; CohCount < CoherentNumber; CohCount ++)
		{
			// determine modulation bit
			CurBit = pSatParam->BitArray[CurBitIndex];
			Value = complex_number(CurBit ? -Amplitude : Amplitude, 0);
			// rotate in DFT
			Value *= complex_number(cos(DopplerPhase), sin(DopplerPhase));
			// add noise
			Value += GenerateNoise(SIGMA0);
			// add to DFT result
			if (CohCount == 0)	// for the first round of coherent sum, do not apply DFT twiddle factor
				for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
					DftResult[FreqCount] = Value;
			else	// for other round of coherent sum
			{
				for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
				{
					DftPhase = (FreqCount * 2 - 7) * DftTwiddleFactor;
					DftResult[FreqCount] += Value * complex_number(cos(DftPhase), sin(DftPhase));
				}
			}

			DopplerPhase += 2 * PI * (CarrierFreqMin - pSatParam->Doppler) / 1000;
			DftTwiddleFactor += DftTwiddlePhase;
			// increase CurMsCount and determine bit position
			if (pSatParam && ++CurMsCount == pSatParam->BitLength)
			{
				CurMsCount = 0;
				CurBitIndex ++;
			}
		}
		for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
			AmplitudeAcc[FreqCount] += DftResult[FreqCount].abs();
	}

	for (FreqCount = 0; FreqCount < DFT_NUMBER; FreqCount ++)
		if (AmplitudeMax < AmplitudeAcc[FreqCount])
		{
			AmplitudeMax = AmplitudeAcc[FreqCount];
			DftBinMin = FreqCount;
		}
	// assign FreqBin field of signal
	FreqBin = (StrideOffsetMin << 3) + DftBinMin;
	return AmplitudeMax;
}

void CAcqEngine::SearchOneChannel(AeBufferSatParam *pSatParam, int Channel)
{
	double NoisePeaks[3];
	double SignalPeak, NoiseFloor, Sigma;
	complex_number NoiseShift;
	int FreqBin, Cor, GlobalExp, NoiseFloorInt;
	int PeakAmp, PeakFreqBin, PeakCor, MaxCor = MF_DEPTH * CodeSpan;
	int i, SignalIndex;
	double PeakThreshold;

	// initialize CurMsCount and CurBitIndex
	if (pSatParam)
	{
		CurMsCount = pSatParam->MsCount + ReadAddress / 3;
		CurBitIndex = CurMsCount / pSatParam->BitLength;
		CurMsCount = (PrnSelect == FREQ_L1CA) ? (CurMsCount % pSatParam->BitLength) : 0;
	}
	else
	{
		CurMsCount = 0;
		CurBitIndex = 0;
	}

	// get noise floor
	NoiseFloor = 682 * SIGMA0 * NonCoherentNumber;
	NoiseFloor *= sqrt(PI * CoherentNumber / 2);
	Sigma = sqrt(682 * CoherentNumber * NonCoherentNumber * (4 - PI) / 2) * SIGMA0;
	NoiseShift = GenerateNoise(Sigma);
	NoiseFloor += NoiseShift.real;
	NoiseFloorInt = (int)NoiseFloor;
	// generate noise peaks
	GetNoisePeaks(NoisePeaks);
	// generate signal peaks
	SignalPeak = GetSignalPeak(pSatParam, FreqBin, Cor);

	// find signal at sorted position
	if (SignalPeak > NoisePeaks[0])
	{
		SignalIndex = 0;
		NoisePeaks[2] = NoisePeaks[1]; NoisePeaks[1] = NoisePeaks[0];
		NoisePeaks[0] = SignalPeak;
	}
	else if (SignalPeak > NoisePeaks[1])
	{
		SignalIndex = 1;
		NoisePeaks[2] = NoisePeaks[1];
	}
	else if (SignalPeak > NoisePeaks[2])
		SignalIndex = 2;
	else
		SignalIndex = 3;

	// determine global exp
	GlobalExp = int(log10(NoisePeaks[0]) / 0.3010 + 1) - 8;	// this is number of shift to have max amplitude fit in 8bit
	NoiseFloorInt >>= GlobalExp;
	// determine success flag
	PeakThreshold = NoisePeaks[2] * (9 + PeakRatioTh) / 8;
	// early terminate acquisition if amplitude of maximum peak larger than threshold
	Success = (NoisePeaks[0] >= PeakThreshold) ? 1 : 0;

	// assign result in channel config buffer
	ChannelConfig[Channel][4] = (Success << 31) | (GlobalExp << 24) | (NoiseFloorInt & 0x7ffff);
	for (i = 0; i < 3; i ++)
	{
		if (i == SignalIndex)	// select signal in current peak
		{
			PeakAmp = ((int)SignalPeak) >> GlobalExp;
			PeakFreqBin = FreqBin;
			PeakCor = Cor;
		}
		else
		{
			PeakAmp = ((int)NoisePeaks[i]) >> GlobalExp;
			// random FreqBin
			PeakFreqBin = (rand() % (8 * StrideNumber)) - (StrideNumber - 1) / 2 * 8;
			if (CoherentNumber == 1)
				PeakFreqBin &= ~7;	// no DFT, DFT bin field always 0
			PeakCor = rand() % MaxCor;
		}
		ChannelConfig[Channel][5+i] = (PeakAmp << 24) | ((PeakFreqBin & 0x1ff) << 15) | PeakCor;
	}
//	printf("Ch%02d %08x %08x %08x %08x ", Channel, ChannelConfig[Channel][4], ChannelConfig[Channel][5], ChannelConfig[Channel][6], ChannelConfig[Channel][7]);
//	printf("Svid%2d Amp=%f Cor=%4d Freq=%f\n", Svid, SignalPeak, Cor, (FreqBin - 3.5) * 62.5 + CenterFreq);
}

void CAcqEngine::DoAcquisition()
{
	int i, j;
	int Freq;
	AeBufferSatParam *pSatParam;

	for (i = 0; i < ChannelNumber; i ++)
	{
		// fill in config registers
		StrideNumber = EXTRACT_UINT(ChannelConfig[i][0], 0, 6);
		CoherentNumber = EXTRACT_UINT(ChannelConfig[i][0], 8, 6);
		NonCoherentNumber = EXTRACT_UINT(ChannelConfig[i][0], 16, 7);
		PeakRatioTh = EXTRACT_UINT(ChannelConfig[i][0], 24, 3);
		EarlyTerminate = EXTRACT_UINT(ChannelConfig[i][0], 27, 1);
		Freq = EXTRACT_INT(ChannelConfig[i][1], 0, 20);
		CenterFreq = Freq * 2.046e6 / 1048576.;
		Svid = EXTRACT_UINT(ChannelConfig[i][1], 24, 6);
		PrnSelect = EXTRACT_UINT(ChannelConfig[i][1], 30, 2);
		CodeSpan = EXTRACT_UINT(ChannelConfig[i][2], 0, 5);
		ReadAddress = EXTRACT_UINT(ChannelConfig[i][2], 8, 5);
		Freq = (int)EXTRACT_UINT(ChannelConfig[i][2], 20, 11);
		DftTwiddlePhase = Freq * PI / 8192;
		Freq = (int)EXTRACT_UINT(ChannelConfig[i][3], 0, 22);
		StrideInterval = Freq * 2.046e6 / 4294967296.;
		PhaseCount = 2046;	// default value for total phase count

		// find whether SV to be acquired exisst
		pSatParam = (AeBufferSatParam *)NULL;
		for (j = 0; j < SatNumber; j ++)
			if (Svid == SatParam[j].svid && PrnSelect == SatParam[j].PrnSelect)
			{
				switch (PrnSelect)
				{
				case FREQ_L1CA:	PhaseCount = 2046; break;
				case FREQ_E1:	PhaseCount = 2046 * 4; break;
				case FREQ_B1C:
				case FREQ_L1C:	PhaseCount = 2046 * 10; break;
				}
				pSatParam = &SatParam[j];
				break;
			}

		// Do searching
		SearchOneChannel(pSatParam, i);
	}
}

void CAcqEngine::SetBufferParam(PSATELLITE_PARAM SatelliteParam[], int SatVisible, GNSS_TIME Time, NavBit *NavData[])
{
	int i;

	SatNumber = 0;
	for (i = 0; i < SatVisible; i ++)
	{
		if (SatelliteParam[i]->system == GpsSystem)
		{
			AssignChannelParam(SatelliteParam[i], Time, NavData[0], 0, &SatParam[SatNumber++]);	// add L1C/A
//			AssignChannelParam(SatelliteParam[i], Time, NavData[3], 3, &SatParam[SatNumber++]);	// add L1C
		}
		else if (SatelliteParam[i]->system == BdsSystem)
			AssignChannelParam(SatelliteParam[i], Time, NavData[2], 2, &SatParam[SatNumber++]);	// add B1C
		else if (SatelliteParam[i]->system == GalileoSystem)
			AssignChannelParam(SatelliteParam[i], Time, NavData[1], 1, &SatParam[SatNumber++]);	// add E1C
	}
}

void CAcqEngine::AssignChannelParam(PSATELLITE_PARAM pSatelliteParam, GNSS_TIME Time, NavBit *NavData, int PrnSelect, AeBufferSatParam *pSatAcqParam)
{
	int i;
	int FrameLength, BitLength, SecondaryLength;
	int FrameNumber, BitNumber, MilliSeconds;
	int TotalBits, BitCount;
	GnssSystem System;
	const unsigned int *SecondaryCode;
	int Bits[300];
	GNSS_TIME TransmitTime;
	double Time2CodeEnd;

	switch (PrnSelect)
	{
	case 0:	// GPS L1C/A
		FrameLength = 6000;
		BitLength = 20;
		TotalBits = 8;	// maximum 8 bits within 128ms for GPS
		System = GpsSystem;
		break;
	case 1:	// Galileo E1C
		FrameLength = 100;
		BitLength = 4;
		TotalBits = 32;
		System = GalileoSystem;
		break;
	case 2:	// BDS B1C
		FrameLength = 18000;
		BitLength = 10;
		TotalBits = 14;
		System = BdsSystem;
		Time.MilliSeconds -= 14000;	// compensate BDS leap second difference
		break;
	case 3:	// GPS L1C
		FrameLength = 18000;
		BitLength = 10;
		TotalBits = 14;
		System = GpsSystem;
		break;
	}
	// calculate TransmitTimeMs, TransmitTime as Time - TravelTime
	TransmitTime = GetTransmitTime(Time, GetTravelTime(pSatelliteParam, 0));
	// calculate frame count and bit count
//	if (PrnSelect == 0)
		TransmitTime.MilliSeconds ++;	// time of NEXT code round
//	else
//		TransmitTime.MilliSeconds += BitLength;	// time of NEXT code round
	FrameNumber = TransmitTime.MilliSeconds / FrameLength;	// frame number
	MilliSeconds = TransmitTime.MilliSeconds - FrameNumber * FrameLength;	// remaining time in current frame in millisecond
	BitNumber = MilliSeconds / BitLength;	// bit number
	MilliSeconds -= BitNumber * BitLength;	// remaining time in current bit
	// assign parameters
	pSatAcqParam->PrnSelect = PrnSelect;
	pSatAcqParam->svid = pSatelliteParam->svid;
	pSatAcqParam->BitLength = BitLength;
	pSatAcqParam->MsCount = MilliSeconds;
	Time2CodeEnd = 1 - TransmitTime.SubMilliSeconds;
	if (PrnSelect != 0 && MilliSeconds != 0)
		Time2CodeEnd += (BitLength - MilliSeconds);
	pSatAcqParam->Time2CodeEnd = (Time2CodeEnd + 2.5 / SAMPLES_1MS) * 2046;	// convert to unit of 1/2 code chip with compensation of filter delay (2.5 samples)
	pSatAcqParam->Doppler = GetDoppler(pSatelliteParam, 0);
	pSatAcqParam->Amplitude = pow(10, (pSatelliteParam->CN0 - 3000) / 2000.) * SIGMA0;
	// generate bits
	if (PrnSelect == 0)	// L1C/A, get one subframe data, to simplify, if bit passed end of subframe, round back to beginning
	{
		NavData->GetFrameData(TransmitTime, pSatelliteParam->svid, 0, Bits);
		for (i = 0; i < TotalBits; i ++)
			pSatAcqParam->BitArray[i] = Bits[(i + BitNumber) % 300];
	}
	else	// get pilot bits
	{
		SecondaryCode = GetPilotBits(System, 0, pSatelliteParam->svid, SecondaryLength);
		for (i = 0; i < TotalBits; i ++)
		{
			BitCount = (i + BitNumber) % SecondaryLength;
			pSatAcqParam->BitArray[i] = (SecondaryCode[BitCount/32] & (1 << (BitCount&0x1f))) ? 1 : 0;
		}
	}
}
