#include "data.h"
#include "OutlierDetection.h"

OutlierDetection::OutlierDetection()
{
}

//Outlier Detection
void OutlierDetection::outlierDetection(EpoRange& range)
{
    if (range.satRange.size() == 0)
    {
        std::cout << "No available observations!" << std::endl;
        return;
    }

    for (int i = 0; i < range.satRange.size(); ++i)
    {
        MWGF mw_gf;
        mw_gf.Prn = range.satRange[i].PRN;
        mw_gf.Sys = range.satRange[i].system;

        //Check if Observations are Valid
        bool valid_PLD = true;
        for (int j = 0; j < 2; ++j)
        {
            if (range.satRange[i].D[j] == 0 || range.satRange[i].L[j] == 0 || range.satRange[i].P[j] == 0)
            {
                range.satRange[i].valid = false;
                valid_PLD = false;
                break;
            }
        }
        if (!valid_PLD)
        {
            mw_gf.GF = 0;
            mw_gf.MW = 0;
            mw_gf.PIF = 0;
            mw_gf.n = 0;
            range.ComObs.push_back(mw_gf);
            continue;
        }

        //GF
        mw_gf.GF = range.satRange[i].L[0] - range.satRange[i].L[1];

        double f1, f2;
        if (mw_gf.Sys == GPS)
        {
            f1 = FG1_GPS;
            f2 = FG2_GPS;
        }
        else
        {
            f1 = FG1_BDS;
            f2 = FG3_BDS;
        }

        //MW
        mw_gf.MW = (range.satRange[i].L[0] * f1 - range.satRange[i].L[1] * f2) / (f1 - f2) - (range.satRange[i].P[0] * f1 + range.satRange[i].P[1] * f2) / (f1 + f2);

        //PIF
        mw_gf.PIF = (range.satRange[i].P[0] * f1 * f1 - range.satRange[i].P[1] * f2 * f2) / (f1 * f1 - f2 * f2);

        if (ComObsTemp.size() != 0)
        {
            bool find = false;
            for (int j = 0; j < ComObsTemp.size(); ++j)
            {
                if ((ComObsTemp[j].Prn == mw_gf.Prn) && (ComObsTemp[j].Sys == mw_gf.Sys) && (ComObsTemp[j].GF != 0))
                {
                    find = true;
                    double dGF = mw_gf.GF - ComObsTemp[j].GF;
                    mw_gf.n = ComObsTemp[j].n;

                    double dMW = mw_gf.MW - ComObsTemp[j].MW;

                    //Outlier Detection
                    if (abs(dGF) < 0.05 && abs(dMW) < 3)
                    {
                        range.satRange[i].valid = true;
                        mw_gf.n++;

                        //Smoothing of MW Observations
                        mw_gf.MW = (ComObsTemp[j].MW * (mw_gf.n - 1) + mw_gf.MW) / mw_gf.n;
                    }
                    else
                    {
                        mw_gf.MW = ComObsTemp[j].MW;
                    }
                }
            }
            if (!find)
            {
                mw_gf.n = 1;
                range.satRange[i].valid = true;
            }
        }
        else
        {
            mw_gf.n = 1;
            range.satRange[i].valid = true;
        }

        range.ComObs.push_back(mw_gf);
    }
    ComObsTemp.clear();
    ComObsTemp = range.ComObs;
}
