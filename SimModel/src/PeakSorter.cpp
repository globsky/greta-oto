//----------------------------------------------------------------------
// PeakSorter.cpp:
//   AE peak sorter simulator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include <string.h>

#include "PeakSorter.h"

const int CPeakSorter::SortIndex[7][3] = {
	{ 0, 1, 2},
	{ 3, 0, 1},
	{ 0, 3, 1},
	{ 0, 1, 3},
	{ 3, 1, 2},
	{ 3, 0, 2},
	{ 0, 3, 2},
};

const int CPeakSorter::SelectMatrix[4][4] = {
	{ 4, 5, 1, 1},
	{ 0, 6, 2, 2},
	{ 0, 0, 3, 3},
	{ 0, 0, 0, 0},
};

CPeakSorter::CPeakSorter()
{
	Clear();
}

CPeakSorter::~CPeakSorter()
{
}

void CPeakSorter::Clear()
{
	memset(Peaks, 0, sizeof(Peaks));
}

int CPeakSorter::InsertValue(PeakData Peak)
{
	int ValueIndex = CompareValue(Peak);
	int MatchIndex = CompareMatch(Peak);
	int SelectIndex = SelectMatrix[ValueIndex][MatchIndex];

	if (SelectIndex)
	{
		// assign Peaks depend on SelectIndex
		Peaks[2] = (SortIndex[SelectIndex][2] == 3) ? Peak : Peaks[SortIndex[SelectIndex][2]];
		Peaks[1] = (SortIndex[SelectIndex][1] == 3) ? Peak : Peaks[SortIndex[SelectIndex][1]];
		Peaks[0] = (SortIndex[SelectIndex][0] == 3) ? Peak : Peaks[SortIndex[SelectIndex][0]];
	}
	return 0;
}

int CPeakSorter::CompareValue(PeakData Peak)
{
	int i;

	for (i = PEAK_NUMBER; i > 0; i --)
		if (Peaks[i - 1].Amp >= Peak.Amp)
			return i;
	return i;
}

int CPeakSorter::CompareMatch(PeakData Peak)
{
	int i;
	int diff;

	for (i = 0; i < PEAK_NUMBER; i ++)
	{
		diff = Peak.FreqPos - Peaks[i].FreqPos;
		if (diff != -1 && ((diff & ~1) != 0))	// diff != -1/0/1
			continue;
		diff = Peak.PhasePos - Peaks[i].PhasePos;
		if (diff == -1 || ((diff & ~1) == 0))	// diff == -1/0/1, ignore the case of first and last code phase are adjacent
			return i;
	}
	return PEAK_NUMBER;
}
