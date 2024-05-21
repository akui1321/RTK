#ifndef OUTLIER_DETECTION_H
#define OUTLIER_DETECTION_H

#include "data.h"
#include <iostream>

/**************************************************************************************************************
  OutlierDetection Class

  Purpose: Outlier detection.

  Member Functions:
  - OutlierDetection: Constructor for OutlierDetection class.
  - outlierDetection: Compute combined observations and detect outliers.
**************************************************************************************************************/

class OutlierDetection
{
public:
    OutlierDetection();

    void outlierDetection(EpoRange& range);

private:
    std::vector<MWGF> ComObsTemp;
};

#endif