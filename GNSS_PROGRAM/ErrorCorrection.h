#ifndef ERROR_CORRECTION_H
#define ERROR_CORRECTION_H

#include "data.h"
#include <cmath>
#include <iostream>

/**************************************************************************************************************
  ErrorCorrection Class

  Purpose: Error correction.

  Member Functions:
  - ErrorCorrection: Constructor for ErrorCorrection class.
  - Hopfield: Hopfield Tropospheric Correction.
**************************************************************************************************************/

class ErrorCorrection 
{
public:
    ErrorCorrection(const double& H, const double& Elev);

    double Hopfield();

private:
    double H = 0.0;
    double Elev = 0.0;
};

#endif
