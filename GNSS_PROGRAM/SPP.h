#ifndef SPP_H
#define SPP_H

#include "data.h"
#include "Matrix.h"
#include "SatellitePositionAndClockError.h"
#include "Coordinate.h"
#include "ErrorCorrection.h"

/**************************************************************************************************************
  SPP Class

  Purpose: Perform standard point positioning using the least squares method.

  Member Functions:
  - SPP: Constructor for the SPP class.
  - SolvePosition: Solve for the receiver position.
  - SolveVelocity: Solve for the receiver velocity.
  - RotationCorrection: Earth rotation correction.
**************************************************************************************************************/

class SPP 
{
public:
	SPP();

    bool SolvePosition(EpoRange& range, const std::vector<GPSEPHREC>& GPSEph, const std::vector<BDSEPHREC>& BDSEph);

	bool SolveVelocity(const EpoRange& range, const std::vector<GPSEPHREC>& GPSEph, const std::vector<BDSEPHREC>& BDSEph);

	POSRES PosRes;

private:
	double Pos_0[3];
	int ValidSat;
	int GPScount;
	int BDScount;
	double dt[2];
	int Iterator;
	int paraNum;
	double PDOP;
	double sigma_0;

	void RotationCorrection(Matrix& Pos, SATMIDRES& SATMID, const GNSSSys& system);
};

#endif
