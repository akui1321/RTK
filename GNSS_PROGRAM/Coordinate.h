#ifndef COORDINATE_H
#define COORDINATE_H

#include "Matrix.h"
#include "data.h"
#include <iostream>
#include <cmath>

/**************************************************************************************************************
  Coordinate Class

  Purpose: Perform various coordinate transformations.

  Member Functions:
  - Coordinate: Constructor for Coordinate class.
  - BLHtoXYZ: Convert Geodetic Coordinates to Cartesian Coordinates.
  - XYZtoBLH: Convert Cartesian Coordinates to Geodetic Coordinates.
  - BLHtoNEUMat: Compute the transformation matrix for converting Geodetic Coordinates to Horizontal Coordinates.
  - PositioningError: Positioning error in Horizontal Coordinate System.
  - CompSatElAz: Calculate Satellite Elevation and Azimuth.
**************************************************************************************************************/

class Coordinate 
{
public:
    double X = 0.0, Y = 0.0, Z = 0.0;
    double B = 0.0, L = 0.0, H = 0.0;
    double N = 0.0, E = 0.0, U = 0.0;

    double N1 = 0.0;
    double Elev = 0.0;
    double Azim = 0.0;

    Coordinate(const int& type, double& x, double& y, double& z);

    void BLHtoXYZ(const double& R, const double& f);

    void XYZtoBLH(const double& R, const double& f);

    void PositioningError(const double X0[], const double Xr[]);

    void CompSatElAz(const double X0[], const double Xs[]);

private:
    double a = 6378137.0;
    double b = 6356752.3142;
    double e = sqrt((pow(a, 2) - pow(b, 2)) / (pow(a, 2)));

    Matrix BLHtoNEUMat();
};

#endif