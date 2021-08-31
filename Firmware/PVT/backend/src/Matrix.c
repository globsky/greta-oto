//----------------------------------------------------------------------
// Matrix.c:
//   functions to do matrix operations
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include <string.h>

#include "DataTypes.h"

#define SUM_N(n) (((n)*(n+1))>>1)	// 1 sum to n
#define DIAG_INDEX(n) (SUM_N(n)+n)	// diagonal index of n

static void LDLTDecompose(double *L, int dim);
static void InvL(double *L, double *Inv, int dim);
static void LTDLMultiply(double *L, double *Inv, int dim);

//*************** Calculate column vector of Delta=Ht*W*MsrDelta ****************
//* in which Ht is transpose of H matrix, W=Inv(R) is the weight matrix
//* MsrDelta is measurement difference
// Parameters:
//   Delta: pointer to result vector
//   H: pointer to H matrix
//   MsrDelta: pointer to measurement difference vector
//   dim: number of system participated
// Return value:
//   none
void ComposeDelta(double *Delta, PHMATRIX H, double *MsrDelta, int dim)
{
	double *p1, *p2, *p3;
	int i, j, SizeX = 0;

	for(i=0; i< dim; i++)
		SizeX += H->length[i];

	if(H->Is2D)
		SizeX += 1;

	// calculate transpose of H multiply MsrDelta (either Psr or Doppler)
	for (i = 0; i < 3; i ++)
	{
		Delta[i] = 0;
		p1 = MsrDelta;
		p2 = H->data[i];
		p3 = H->weight;
		for (j = SizeX; j > 0; j --)
			Delta[i] += (*p1 ++) * (*p2 ++) * (*p3 ++);
	}
	p1 = MsrDelta;
	p3 = H->weight;
	for (i = 0; i < dim; i ++)
	{
		Delta[i + 3] = 0;
		for (j = H->length[i]; j > 0; j --)
			Delta[i + 3] += (*p1 ++) * (*p3 ++);
	}
}

//*************** Calculate product of Ht and H ****************
//* Input transpose of matrix H and x size of Ht
//* assuming the 4/5/6 column of H is length[0/1/2] respectively
//* weights is diagnose matrix and given as vector
//* the format Ht is shown below: (dim=2 as example)
//* / H(1,1) H(1,2) H(1,3) ... H(1,n) \
//* | H(2,1) H(2,2) H(2,3) ... H(2,n) |
//* | H(3,1) H(3,2) H(3,3) ... H(3,n) |
//* |    1      1      1...1  0...0   |
//* \    0      0      0...0  1...1   /
//*      \_______  ________/  \_  _/
//*              \/             \/
//*            Length1        Length2
//* because HtH is a symmetrical matrix
//* return value put into a vector HtH with following order
//* HtH(1,1), HtH(2,1), HtH(2,2), HtH(3,1), HtH(3,2), ...
//* the total length is 10 or 15 or 21 depend on dimension
//* If InvP is not NULL, InvP is added to HtH
//* In weighted LSQ, this function computes the Ht*W*H
// Parameters:
//   H: pointer to H matrix
//   InvP: pointer to InvP matrix
//   HtH: pointer to result matrix
//   dim: number of system participated
// Return value:
//   none
void GetHtH(PHMATRIX H, double *InvP, double *HtH, int dim)
{
	double *p1, *p2, *p3, *result = HtH, *pa = InvP;
	int i, j, k;
	int SizeX = 0, AddrOffset;

	for(i=0; i < dim; i ++)
		SizeX += H->length[i];

	//if 2D positioning
	if (H->Is2D)
		SizeX ++;
	
	for (i = 0; i < 3; i ++)
	{
		for (j = 0; j <= i; j ++)
		{
			p1 = H->data[i];
			p2 = H->data[j];
			p3 = H->weight;
			*result = InvP ? (*pa ++) : 0;
			for (k = SizeX; k > 0; k --)
				*result += (*p1 ++) * (*p2 ++) * (*p3 ++);
			result ++;
		}
	}
	
	AddrOffset = 0;
	for (i = 0; i < dim; i ++)
	{
		for (j = 0; j < 3; j ++)
		{
			*result = InvP ? (*pa ++) : 0;
			p2 = H->data[j] + AddrOffset;
			p3 = H->weight + AddrOffset;
			for (k = H->length[i]; k > 0; k --)
				*result += (*p2 ++) * (*p3 ++);
			result ++;
		}
		for (j = i; j > 0; j --)
			*result ++ = InvP ? (*pa ++) : 0;
	
		*result = InvP ? (*pa ++) : 1e-11;
		p3 = H->weight + AddrOffset;
		for (k = H->length[i]; k > 0; k --)
			*result += (*p3 ++);
		result ++;
		AddrOffset += H->length[i];
	}
}

//*************** Calculate inversion of a symmetrical matrix ****************
//* input symmetrical matrix as vector format
//* output symmetrical matrix occupies the same place
// Parameters:
//   SymMat: pointer to symmetrical matrix
//   WorkSpace: pointer to work space to do inversion
//   dim: number of system participated
// Return value:
//   none
void SymMatrixInv(double *SymMat, double *WorkSpace, int dim)
{
	LDLTDecompose(SymMat, dim);				// in place calculate matrix L
	InvL(SymMat, WorkSpace, dim);			// matrix Inv(L) in TempVector
	LTDLMultiply(WorkSpace, SymMat, dim);	// put Inv(HtH) back to SymMat
}

//*************** Do Cholesky decomposition of a symmetrical matrix ****************
//* output lower triangular matrix occupies the same place
//* The result is HtH=L*D*L' in which
//* L is a triangle matrix with diagonal element equals 1
//* D is a diagonal matrix with diagonal element as
//* reciprocal of diagonal element in matrix L
//* output triangle matrix L*D as vector
//* dim = 1: it is a 4x4 matrix
//* dim = 2: it is a 5x5 matrix
//* dim = 3: it is a 6x6 matrix
//* Cholesky decomposition MATLAB algorithm is:
//* for i=1:dim
//* 	for j=1:i
//* 		for k=1:j-1
//* 			l(i,j)=l(i,j)-l(i,k)*l(j,k)/l(k,k);
//* 		end
//* 	end
//* end
// Parameters:
//   L: pointer to symmetrical matrix
//   dim: number of system participated
// Return value:
//   none
void LDLTDecompose(double *L, int dim)
{
	int i, j, k, index = 0;
	double *p1, *p2;

	for (i = 0; i < dim; i ++)
	{
		for (j = 0; j <= i; j ++)
		{
			p1 = L + SUM_N(i);
			p2 = L + SUM_N(j);
			for (k = 0; k < j; k ++)
				L[index] -= ((*p1 ++) * (*p2 ++) / L[SUM_N(k+1)-1]);
			index ++;
		}
	}
}

//*************** Calculate inversion of a lower triangular matrix ****************
//* Input triangle matrix L as vector, do matrix inversion
//* output triangle matrix Inv as vector
//* dim = 1: it is a 4x4 matrix
//* dim = 2: it is a 5x5 matrix
//* dim = 3: it is a 6x6 matrix
//* triangle matrix inversion MATLAB algorithm is:
//* linv=zeros(dim,dim);
//* for i=1:dim
//* 	linv(i,i)=1;
//* end
//* for i=1:dim
//* 	for j=i-1:-1:1
//* 		for k=j:-1:1
//* 			linv(i,k)=linv(i,k)-linv(j,k)*l(i,j);
//* 		end
//* 	end
//* 	for j=i:-1:1
//* 		linv(i,j)=linv(i,j)/l(i,i);
//* 	end
//* end
// Parameters:
//   L: pointer to lower triangular matrix
//   Inv: pointer to inverse matrix of L
//   dim: number of system participated
// Return value:
//   none
void InvL(double *L, double *Inv, int dim)
{
	int i, j, k;
	double *dest, *src, *factor, f;

	for (i = 0; i < dim; i ++)
	{
		factor = L + DIAG_INDEX(i) - 1;
		dest = Inv + DIAG_INDEX(i) - 1;

		for (j = i - 1; j >= 0; j --)
			(*dest --) = 0;
		for (j = i - 1; j >= 0; j --)
		{
			dest = Inv + SUM_N(i) + j;
			src = Inv + DIAG_INDEX(j);
			for (k = 0; k <= j; k ++)
				(*dest --) -= (*src --) * (*factor);
			factor --;
		}
		dest = Inv + DIAG_INDEX(i);
		factor = L + DIAG_INDEX(i);
		f = 1 / *factor;
		(*dest --) = f;
		for (j = i - 1; j >= 0; j --)
			(*dest --) *= f;
	}
}

//*************** Calculate product a triangular matrix and its transpose matrix ****************
//* This is the last step to do matrix inversion
//* using Cholesky decomposition
//* HtH=L*D*L' so
//* Inv(HtH)=InvL'*InvD*InvL
//* because D is a diagonal matrix with diagonal element as
//* reciprocal of diagonal element in matrix L,
//* so InvD is a diagonal matrix with diagonal element as
//* reciprocal of diagonal element in matrix InvL
//* input triangle matrix InvL as vector format
//* output vector with following order in result
//* (1,1) (2,1) (2,2) (3,1) (3,2) (3,3) (4,1) ...
//* dim = 1: it is a 4x4 matrix
//* dim = 2: it is a 5x5 matrix
//* dim = 3: it is a 6x6 matrix
//* MATLAB algorithm is:
//* inv=l;
//* for i=3:-1:1
//*		for j=i:-1:1
//*			for k=j:-1:1
//*				inv(j,k)=inv(j,k)+l(i+1,j)*l(i+1,k)/l(i+1,i+1);
//*			end
//*		end
//* end
// Parameters:
//   L: pointer to work space (copy of InvL)
//   Inv: pointer to input InvL matrix and final result
//   dim: number of system participated
// Return value:
//   none
void LTDLMultiply(double *L, double *Inv, int dim)
{
	int i, j, k;
	double *dest, *src, factor;

	memcpy(Inv, L, sizeof(double) * SUM_N(dim));
	for (i = dim - 2; i >= 0; i --)
	{
		dest = Inv + DIAG_INDEX(i);
		for (j = i; j >= 0; j --)
		{
			src = L + DIAG_INDEX(i) + j + 1;
			factor = (*src) / (*(L + DIAG_INDEX(i+1)));
			for (k = 0; k <= j; k ++)
				(*dest --) += (*src --) * (factor);
		}
	}
}

//*************** Calculate symmetrical matrix multiply a column vector ****************
//* input symmetrical matrix Inv as vector format
//* with following order
//* (1,1) (2,1) (2,2) (3,1) (3,2) (3,3) (4,1) ...
// Parameters:
//   DeltaPos: pointer to result vector
//   Inv: pointer to symmetrical matrix
//   Delta: pointer to column vector
//   dim: number of system participated
// Return value:
//   none
void SymMatrixMultiply(double *DeltaPos, double *Inv, double *Delta, int dim)
{
	int i, j;
	double *p1, *p2;

	for (i = 0; i < dim; i ++)
	{
		DeltaPos[i] = 0;
		p1 = Inv + SUM_N(i);
		p2 = Delta;
		DeltaPos[i] = 0;
		for (j = 0; j < i; j ++)
			DeltaPos[i] += (*p1 ++) * (*p2 ++);
		for (j ++; j <= dim; j ++)
		{
			DeltaPos[i] += (*p1) * (*p2 ++);
			p1 += j;
		}
	}
}
