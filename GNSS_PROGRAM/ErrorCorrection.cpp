#include "data.h"
#include "ErrorCorrection.h"

ErrorCorrection::ErrorCorrection(const double& H, const double& Elev) :H(H), Elev(Elev)
{
}

//Tropospheric Correction
double ErrorCorrection::Hopfield() 
{
    if (H >= 0 && H < 20000)
    {
        Elev = Elev / PI * 180;
        double RH = RH_0 * exp(-0.0006396 * (H - H_0));
        double p = p_0 * pow((1 - 0.0000226 * (H - H_0)), 5.225);
        double T = T_0 - 0.0065 * (H - H_0);
        double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
        double h_w = 11000;
        double h_d = 40136 + 148.72 * (T_0 - 273.16);
        double K_w = 155.2 * 1e-7 * 4810 / T / T * e * (h_w - H);
        double K_d = 155.2 * 1e-7 * p / T * (h_d - H);
        double delta_Trop = K_d / sin(sqrt(Elev * Elev + 6.25) * PI / 180.0) + K_w / sin(sqrt(Elev * Elev + 2.25) * PI / 180.0);
        return delta_Trop;
    }
    else
    {
        return 0;
    }
}
