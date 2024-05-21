#ifndef SATELLITE_POSITION_AND_CLOCK_ERROR
#define SATELLITE_POSITION_AND_CLOCK_ERROR

#include "data.h"
#include "Matrix.h"
#include <iostream>
#include <cmath>

/**************************************************************************************************************
  SatellitePositionAndClockError Class

  Purpose: Calculate satellite position, velocity, clock offset, and clock speed offset.

  Member Functions:
  - SatellitePositionAndClockError: Constructor for the SatellitePositionAndClockError class.
  - CalculatePVCE: Differentiates between GPS and BDS satellites, and returns the calculation results.
  - calculateToe: Find the most recent toe (time of ephemeris).
  - calculate: Responsible for computing position, velocity, clock offset, and clock speed offset.
**************************************************************************************************************/

class SatellitePositionAndClockError 
{
public:
    SatellitePositionAndClockError();

    std::vector<SATMIDRES> CalculatePVCE(const EpoRange& range, const std::vector<GPSEPHREC>& GPSEph, const std::vector<BDSEPHREC>& BDSEph, const int& index);

private:
    std::vector<SATMIDRES> SatMidRes;
    
    template <typename EPHCAL>
    double calculateToe(const SatRange& satRange, const EPHCAL& Eph_cal, const double& t);

    template <typename EPHCAL>
    void calculate(const SatRange& satRange, const EPHCAL& Eph_cal, double t, const double& t_oe);
};

#endif