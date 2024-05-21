#pragma once

#include "data.h"
#include "HexDecoder.h"

/**************************************************************************************************************
  RTK Class

  Purpose: RTK Solution.

  Member Functions:
  - RTK: Constructor for the RTK class.
  - SynObs: Synchronize the observation data between the rover and the base station.
  - DetectLockTime: Lock time and parity detection.
  - FormSDEpochObs: Calculate single-difference observations.
  - DetectCycleSlip: Cycle slip detection.
  - DetRefSat: Detect reference satellites.
  - RTKFloat: Float solution.
  - RtkFixed: Fixed solution.
  - Distance: Calculate the distance.
  - MatToPtr: Convert a matrix to a pointer.
**************************************************************************************************************/

class RTK 
{
public:
	RTK(const GNSSConfig& config);

	bool SynObs(const EpoRange& range, FILE* file1, HexDecoder& decoder);

	void DetectLockTime(const EpoRange& BeObs, EpoRange& Obs);

	void FormSDEpochObs(const EpoRange& EpkA, const EpoRange& EpkB, SDEPOCHOBS& SDObs);

	void DetectCycleSlip(SDEPOCHOBS& Obs);

	bool DetRefSat(const EpoRange& EpkA, const EpoRange& EpkB, SDEPOCHOBS& SDObs, DDCOBS& DDObs);

	bool RTKFloat(RAWDAT& Raw, const POSRES& Base, const POSRES& Rov);

	bool RtkFixed(RAWDAT& Raw, const POSRES& Base, const POSRES& Rov);

private:
	double* QPtr, *aPtr;

	double RovPos[3];

	int Qrow;

	double Distance(const double* p1, const double* p2);

	void MatToPtr(const Matrix& m, double* p);

	GNSSConfig configRTK;
};

// lambda
int LD(int n, const double* Q, double* L, double* D);
void gauss(int n, double* L, double* Z, int i, int j);
void perm(int n, double* L, double* D, int j, double del, double* Z);
void reduction(int n, double* L, double* D, double* Z);
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);

/****************************************************************************
  MatrixInv

  Ŀ�ģ���������,����ȫѡ��Ԫ��˹-Լ����

  ����:
  n      M1������������
  a      �������
  b      �������   b=inv(a)
  ����ֵ��1=������0=��������

****************************************************************************/

int MatrixInv(int n, double a[], double b[]);

/****************************************************************************
  MatrixMultiply

  Ŀ�ģ�����˷�

  ����:
  m,n    Z������������
  p,q    a������������
  z      �������   z=Z*a
  ����ֵ��1=������0=��������

****************************************************************************/

int MatrixMultiply(const int m1, const int n1, const int m2, const int n2, const double* A, const double* B, double* C);