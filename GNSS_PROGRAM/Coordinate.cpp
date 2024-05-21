#include "Coordinate.h"

Coordinate::Coordinate(const int& type, double& x, double& y, double& z) 
{
    if (type == 1) 
    {
        X = x;
        Y = y;
        Z = z;
    }
    else if (type == 2) 
    {
        B = x;
        L = y;
        H = z;
    }
    else if (type == 3) 
    {
        N = x;
        E = y;
        U = z;
    }
    else 
    {
        std::cerr << "Invalid type." << std::endl;
    }
}

//Convert BLH to XYZ
void Coordinate::BLHtoXYZ(const double& R, const double& f)
{
    a = R;
    e = sqrt(2 * f - f * f);
    double B_tran = B / 180.0 * PI;
    double L_tran = L / 180.0 * PI;
    N1 = a / sqrt(1 - e * e * sin(B_tran) * sin(B_tran));
    X = (N1 + H) * cos(B_tran) * cos(L_tran);
    Y = (N1 + H) * cos(B_tran) * sin(L_tran);
    Z = (N1 * (1 - e * e) + H) * sin(B_tran);
}

//Convert XYZ to BLH
void Coordinate::XYZtoBLH(const double& R,const double& f)
{
    a = R;
    e = sqrt(2 * f - f * f);
    double B0 = 0.0;
    L = atan2(Y, X);

    B0 = atan2(Z, sqrt(pow(X, 2) + pow(Y, 2)));
    B = B0;

    //Iterative Calculation of B
    do
    {
        B0 = B;
        N1 = a / sqrt(1 - e * e * sin(B) * sin(B));
        B = atan2((Z + N1 * sin(B) * pow(e, 2)), sqrt(pow(X, 2) + pow(Y, 2)));
    } while (abs(B - B0) > 0.000001);

    N1 = a / sqrt(1 - e * e * sin(B) * sin(B));
    H = sqrt(pow(X, 2) + pow(Y, 2)) / cos(B) - N1;
}

//Calculate NEU Matrix
Matrix Coordinate::BLHtoNEUMat()
{
    Matrix R(3, 3);
    R(0, 0) = -sin(L);
    R(0, 1) = cos(L);
    R(0, 2) = 0;
    R(1, 0) = -sin(B) * cos(L);
    R(1, 1) = -sin(B) * sin(L);
    R(1, 2) = cos(B);
    R(2, 0) = cos(B) * cos(L);
    R(2, 1) = cos(B) * sin(L);
    R(2, 2) = sin(B);

    return R;
}

//Calculate Positioning Error
void Coordinate::PositioningError(const double X0[], const double Xr[])
{
    Matrix X0r(3, 1);
    X0r(0, 0) = Xr[0] - X0[0];
    X0r(1, 0) = Xr[1] - X0[1];
    X0r(2, 0) = Xr[2] - X0[2];

    Matrix NEU(3, 1);
    Matrix R = BLHtoNEUMat();
    NEU.MatrixMultiplication(R, X0r);
    E = NEU(0, 0);
    N = NEU(1, 0);
    U = NEU(2, 0);
}

//Calculate Azimuth and Elevation Angles
void Coordinate::CompSatElAz(const double X0[], const double Xs[])
{
    Matrix X0s(3, 1);
    X0s(0, 0) = Xs[0] - X0[0];
    X0s(1, 0) = Xs[1] - X0[1];
    X0s(2, 0) = Xs[2] - X0[2];

    Matrix NEU(3, 1);
    Matrix R = BLHtoNEUMat();
    NEU.MatrixMultiplication(R, X0s);
    E = NEU(0, 0);
    N = NEU(1, 0);
    U = NEU(2, 0);

    Elev = atan2(U, sqrt(N * N + E * E));
    Azim = atan2(E, N);
}
