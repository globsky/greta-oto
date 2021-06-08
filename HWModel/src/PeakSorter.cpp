//----------------------------------------------------------------------
// PeakSorter.cpp:
//   AE peak sorter class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <stdio.h>
#include <malloc.h>
#include <string.h>

#include "PeakSorter.h"

bool PeakData::operator > (const PeakData &data)
{
	int ShiftLeft, ShiftRight;

	if (this->Exp >= data.Exp)
	{
		ShiftLeft = 0;
		ShiftRight = this->Exp - data.Exp;
	}
	else
	{
		ShiftLeft = data.Exp - this->Exp;
		ShiftRight = 0;
	}
	return ((this->Amp >> ShiftLeft) > (data.Amp >> ShiftRight));
}

bool PeakData::operator >= (const PeakData &data)
{
	int ShiftLeft, ShiftRight;

	if (this->Exp >= data.Exp)
	{
		ShiftLeft = 0;
		ShiftRight = this->Exp - data.Exp;
	}
	else
	{
		ShiftLeft = data.Exp - this->Exp;
		ShiftRight = 0;
	}
	return ((this->Amp >> ShiftLeft) >= (data.Amp >> ShiftRight));
}

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
		// first normalize the Peak with Peaks if Peak.Exp less than Peaks[0].Exp (assume all Peaks have same exp)
		if (Peak.Exp < Peaks[0].Exp)
		{
			Peak.Amp >>= (Peaks[0].Exp - Peak.Exp);
			Peak.Exp = Peaks[0].Exp;
		}
		// assign Peaks depend on SelectIndex
		Peaks[2] = (SortIndex[SelectIndex][2] == 3) ? Peak : Peaks[SortIndex[SelectIndex][2]];
		Peaks[1] = (SortIndex[SelectIndex][1] == 3) ? Peak : Peaks[SortIndex[SelectIndex][1]];
		Peaks[0] = (SortIndex[SelectIndex][0] == 3) ? Peak : Peaks[SortIndex[SelectIndex][0]];
		// normalize to keep all peaks with the same exp, Peak.Exp will be the largest
		Peaks[0].Amp >>= (Peak.Exp - Peaks[0].Exp);
		Peaks[1].Amp >>= (Peak.Exp - Peaks[1].Exp);
		Peaks[2].Amp >>= (Peak.Exp - Peaks[2].Exp);
		Peaks[2].Exp = Peaks[1].Exp = Peaks[0].Exp = Peak.Exp;
	}
//	printf("%3d %3d %3d %d\n", Peaks[0].Amp, Peaks[1].Amp >> (Peaks[0].Exp - Peaks[1].Exp), Peaks[2].Amp >> (Peaks[0].Exp - Peaks[2].Exp), Peaks[0].Exp);
	return 0;
}

int CPeakSorter::CompareValue(PeakData Peak)
{
	int i;

	for (i = PEAK_NUMBER; i > 0; i --)
		if (Peaks[i - 1] >= Peak)
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
