//----------------------------------------------------------------------
// PeakSorter.h:
//   AE peak sorter simulator class declaration
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#if !defined __PEAK_SORTER_SIM_H__
#define __PEAK_SORTER_SIM_H__

#define PEAK_NUMBER 3

struct PeakData
{
	double Amp;
	int PhasePos;
	int FreqPos;
};

class CPeakSorter
{
public:
	CPeakSorter();
	~CPeakSorter();

	static const int SortIndex[7][3];
	static const int SelectMatrix[4][4];
	PeakData Peaks[PEAK_NUMBER];
	
	void Clear();
	int InsertValue(PeakData Peak);
	int CompareValue(PeakData Peak);
	int CompareMatch(PeakData Peak);
};

#endif //__PEAK_SORTER_SIM_H__
