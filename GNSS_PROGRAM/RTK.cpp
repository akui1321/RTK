#include "RTK.h"
#include "HexDecoder.h"
#include "data.h"

RTK::RTK(const GNSSConfig& config)
{
    configRTK = config;
}

bool RTK::SynObs(const EpoRange& range, FILE* file1, HexDecoder& decoder)
{
    //Preparation for Solution
    static int bufferLength = 0;
    static unsigned char buffer[MAXRAWLEN];
    static FILE* file = file1;

    while (!feof(file))
    {
        //Read Data into Buffer
        int remainingSpace = MAXRAWLEN - bufferLength;
        size_t bytesRead = fread(buffer + bufferLength, sizeof(unsigned char), remainingSpace, file);
        bufferLength += bytesRead;

        //Decode
        int blockEnd = 0;
        int blockStart = 0;
        bool isIncompleteBlock = false;
        int success = decoder.BufferReader(bufferLength, buffer, blockEnd, blockStart, isIncompleteBlock);

        //Move Undecoded Data to the Front of the Buffer, Prepare for Next Data Reading
        if (isIncompleteBlock)
        {
            if (blockStart >= 0)
            {
                //Discard Processed Data
                int remainingLength = bufferLength - blockStart;
                memmove(buffer, buffer + blockStart, remainingLength);
                memset(buffer + remainingLength, 0, blockStart);
                bufferLength = remainingLength;
            }
        }
        else
        {
            if (blockEnd >= 0)
            {
                //Discard Processed data
                int remainingLength = bufferLength - blockEnd;
                memmove(buffer, buffer + blockEnd, remainingLength);
                memset(buffer + remainingLength, 0, blockEnd);
                bufferLength = remainingLength;
            }
        }

        if (success == 1)
        {
            if (abs(decoder.range.Time.gpsSeconds - range.Time.gpsSeconds) < 0.1)
            {
                return true;
            }
            else if (decoder.range.Time.gpsSeconds > range.Time.gpsSeconds)
            {
                return false;
            }
            else
            {
                continue;
            }
        }
        else
        {
            continue;
        }
    }
}

void RTK::DetectLockTime(const EpoRange& BeObs, EpoRange& Obs)
{
    for (int i = 0; i < Obs.SatNum; i++)
    {
        bool found = false;
        int index = -1;
        // Search for the current observed satellite in the last observations
        for (int j = 0; j < BeObs.SatNum; j++)
        {
            if (Obs.satRange[i].system == BeObs.satRange[j].system && Obs.satRange[i].PRN == BeObs.satRange[j].PRN)
            {
                found = true;
                index = j;
                break;
            }
        }

        // Check lock time and parity for each frequency type
        for (int type = 0; type < 2; type++)
        {
            if (Obs.satRange[i].L[type] != 0)
            {
                if (Obs.satRange[i].Parity[type] == 0)
                {
                    Obs.satRange[i].valid = false;
                }
            }

            if (found)
            {
                if (BeObs.satRange[index].L[type] != 0 && Obs.satRange[i].L[type] != 0)
                {
                    if (Obs.satRange[i].LockTime[type] < BeObs.satRange[index].LockTime[type])
                    {
                        Obs.satRange[i].valid = false;
                    }
                }
            }
        }
    }
}

void RTK::FormSDEpochObs(const EpoRange& EpkA, const EpoRange& EpkB, SDEPOCHOBS& SDObs)
{
    SDObs.SdSatObs.clear();
    SDObs.Time = EpkA.Time;
    unsigned short n = 0;

    for (unsigned short satcount = 0; satcount < EpkA.SatNum; ++satcount)
    {
        GNSSSys sys = EpkA.satRange[satcount].system;
        unsigned short prn = EpkA.satRange[satcount].PRN;

        for (size_t i = 0; i < EpkB.satRange.size(); ++i)
        {
            if (EpkB.satRange[i].PRN == prn && EpkB.satRange[i].system == sys)
            {
                SDSATOBS SdSatObsTemp;
                SdSatObsTemp.Prn = prn;
                SdSatObsTemp.System = sys;
                SdSatObsTemp.nBas = i;
                SdSatObsTemp.nRov = satcount;

                // The same satellite, the same frequency, the same type of observation values
                for (int f = 0; f < 2; ++f)
                {
                    if (EpkA.satRange[satcount].valid && EpkB.satRange[i].valid)
                    {
                        if (EpkA.satRange[satcount].P[f] != 0 && EpkB.satRange[i].P[f] != 0)
                        {
                            SdSatObsTemp.dP[f] = EpkA.satRange[satcount].P[f] - EpkB.satRange[i].P[f];
                        }

                        if (EpkA.satRange[satcount].L[f] != 0 && EpkB.satRange[i].L[f] != 0)
                        {
                            SdSatObsTemp.dL[f] = EpkA.satRange[satcount].L[f] - EpkB.satRange[i].L[f];
                        }
                    }
                }

                SDObs.SdSatObs.push_back(SdSatObsTemp);
                n++;
                break;
            }
        }
    }
    SDObs.SatNum = n;
}

void RTK::DetectCycleSlip(SDEPOCHOBS& Obs)
{
    std::vector<MWGF> coobs = Obs.SdCObs;
    Obs.SdCObs.resize(Obs.SatNum);
    for (int i = 0; i < Obs.SatNum; ++i)
    {
        GNSSSys sys = Obs.SdSatObs[i].System;
        int prn = Obs.SdSatObs[i].Prn;
        double GF, MW = 0.0;

        if (Obs.SdSatObs[i].dP[0] == 0 || Obs.SdSatObs[i].dP[1] == 0 || Obs.SdSatObs[i].dL[0] == 0 || Obs.SdSatObs[i].dL[1] == 0)
        {
            Obs.SdSatObs[i].Valid = false;
            continue;
        }

        // Calculate GF and MW combination
        GF = Obs.SdSatObs[i].dL[0] - Obs.SdSatObs[i].dL[1];
        if (sys == GPS)
        {
            MW = (FG1_GPS * Obs.SdSatObs[i].dL[0] - FG2_GPS * Obs.SdSatObs[i].dL[1]) / (FG1_GPS - FG2_GPS) - (FG1_GPS * Obs.SdSatObs[i].dP[0] + FG2_GPS * Obs.SdSatObs[i].dP[1]) / (FG1_GPS + FG2_GPS);
        }
        else
        {
            MW = (FG1_BDS * Obs.SdSatObs[i].dL[0] - FG3_BDS * Obs.SdSatObs[i].dL[1]) / (FG1_BDS - FG3_BDS) - (FG1_BDS * Obs.SdSatObs[i].dP[0] + FG3_BDS * Obs.SdSatObs[i].dP[1]) / (FG1_BDS + FG3_BDS);
        }

        // Find previous observations if any
        auto prevObsIt = std::find_if
        (
            coobs.begin(), coobs.end(), [sys, prn](const MWGF& mwgf)
            {
                return mwgf.Sys == sys && mwgf.Prn == prn;
            }
        );

        if (prevObsIt != coobs.end())
        {
            double dGF = GF - prevObsIt->GF;
            double dMW = MW - prevObsIt->MW;

            if (fabs(dGF) > 0.05 || fabs(dMW) > 3)
            {
                Obs.SdSatObs[i].Valid = false;
            }
            else
            {
                Obs.SdSatObs[i].Valid = true;
                Obs.SdCObs[i].Sys = sys;
                Obs.SdCObs[i].Prn = prn;
                Obs.SdCObs[i].GF = GF;
                Obs.SdCObs[i].MW = (Obs.SdCObs[i].n * prevObsIt->MW + MW) / (Obs.SdCObs[i].n + 1);
                Obs.SdCObs[i].n += 1;
            }
        }
        else
        {
            // Not found
            Obs.SdSatObs[i].Valid = true;
            Obs.SdCObs[i].Sys = sys;
            Obs.SdCObs[i].Prn = prn;
            Obs.SdCObs[i].GF = GF;
            Obs.SdCObs[i].MW = MW;
            Obs.SdCObs[i].n += 1;
        }
    }
}

bool RTK::DetRefSat(const EpoRange& EpkA, const EpoRange& EpkB, SDEPOCHOBS& SDObs, DDCOBS& DDObs)
{
    const int SYSTEM_COUNT = 2;
    int RefPrn[SYSTEM_COUNT] = { -1, -1 }, RefPos[SYSTEM_COUNT] = { -1, -1 };   
    int Sats = 0;
    int DDSatNum[SYSTEM_COUNT] = { 0, 0 };
    double MAXElev[SYSTEM_COUNT] = { 0, 0 };

    // Find the Maximum Elevation Angle
    for (int i = 0; i < SDObs.SatNum; i++)
    {
        int RovIndex = SDObs.SdSatObs[i].nRov; 
        int BasIndex = SDObs.SdSatObs[i].nBas;

        if (EpkA.SatMidRes.size() != 0 && EpkB.SatMidRes.size() != 0)
        {
            if (SDObs.SdSatObs[i].Valid && EpkA.SatMidRes[RovIndex].Valid && EpkB.SatMidRes[BasIndex].Valid)
            {
                Sats++;
                if (SDObs.SdSatObs[i].System == GPS)
                {
                    DDSatNum[0]++;
                    if (MAXElev[0] < EpkA.SatMidRes[RovIndex].Elevation)
                    {
                        MAXElev[0] = EpkA.SatMidRes[RovIndex].Elevation;
                        RefPrn[0] = SDObs.SdSatObs[i].Prn;
                        RefPos[0] = i;
                    }
                }
                else if (SDObs.SdSatObs[i].System == BDS)
                {
                    DDSatNum[1]++;
                    if (MAXElev[1] < EpkA.SatMidRes[RovIndex].Elevation)
                    {
                        MAXElev[1] = EpkA.SatMidRes[RovIndex].Elevation;
                        RefPrn[1] = SDObs.SdSatObs[i].Prn;
                        RefPos[1] = i;
                    }
                }
            }
            else
            {
                SDObs.SdSatObs[i].Valid = false;
            }
        }
        else
        {
            return false;
        }
    }

    // Position of the Reference Satellite
    for (int i = 0; i < 2; i++) 
    {
        DDObs.DDSatNum[i] = DDSatNum[i] - 1;
        DDObs.RefPrn[i] = RefPrn[i];
        DDObs.RefPos[i] = RefPos[i];
    }

    DDObs.Sats = Sats;

    return true;
}

double RTK::Distance(const double* p1, const double* p2)
{
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));
}

bool RTK::RTKFloat(RAWDAT& Raw, const POSRES& Base, const POSRES& Rov)
{
    for (int m = 0; m < 3; m++)
    {
        RovPos[m] = Rov.Pos[m];
    }

    // Number of double-difference satellites
    int SatNum;
    if (Raw.DDObs.DDSatNum[0] == -1 || Raw.DDObs.DDSatNum[1] == -1)
    {
        SatNum = Raw.DDObs.Sats - 1;
    }
    else
    {
        SatNum = Raw.DDObs.Sats - 2;
    }

    if (SatNum <= 0)
    {
        return false;
    }

    Matrix B(4 * SatNum, 3 + 2 * SatNum);
    Matrix W(4 * SatNum, 1), W_T(1, 4 * SatNum);
    Matrix P(4 * SatNum, 4 * SatNum);
    Matrix X(3 + 2 * SatNum, 1);
    Matrix Q(3 + 2 * SatNum, 3 + 2 * SatNum);
    Matrix Na(2 * SatNum, 1);

    double N1, N2;

    bool isFirst = true, isSuccess = false;

    // Iterative computation of the float solution
    for (size_t times = 0; times < 100; times++)
    {
        int index = -4, PIndex = 0;
        bool FirstSys = false;
        GNSSSys lastSys = UNKS;

        for (int i = 0; i < Raw.SdObs.SatNum; i++)
        {
            if (Raw.SdObs.SdSatObs[i].Valid == true)
            {
                if (!FirstSys)
                {
                    FirstSys = true;
                    lastSys = Raw.SdObs.SdSatObs[i].System;
                }

                double WL1 = 0.0, WL2 = 0.0;
                int RefPos = -1;
                int n = 0, lastn = 0;
                if (Raw.SdObs.SdSatObs[i].System == GPS)
                {
                    WL1 = WL1_GPS;
                    WL2 = WL2_GPS;
                    RefPos = Raw.DDObs.RefPos[0];
                    n = Raw.DDObs.DDSatNum[0];
                    lastn = Raw.DDObs.DDSatNum[1];
                }
                else
                {
                    WL1 = WL1_BDS;
                    WL2 = WL3_BDS;
                    RefPos = Raw.DDObs.RefPos[1];
                    n = Raw.DDObs.DDSatNum[1];
                    lastn = Raw.DDObs.DDSatNum[0];
                }

                if (RefPos == i)
                {
                    continue;
                }
                
                index += 4;
                double rovSatDis = Distance(RovPos, Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[i].nRov].SatPos);
                double rovRefSatDis = Distance(RovPos, Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[RefPos].nRov].SatPos);

                for (int p = 0; p < 4; p++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        B(index + p, j) = (RovPos[j] - Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[i].nRov].SatPos[j]) / rovSatDis - (RovPos[j] - Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[RefPos].nRov].SatPos[j]) / rovRefSatDis;
                    }
                    if (p == 2)
                    {
                        B(index + p, 3 + index / 2) = WL1;
                    }
                    if (p == 3)
                    {
                        B(index + p, 3 + index / 2 + 1) = WL2;
                    }
                }

                // Double-difference Observations
                double P1 = Raw.SdObs.SdSatObs[i].dP[0] - Raw.SdObs.SdSatObs[RefPos].dP[0];
                double P2 = Raw.SdObs.SdSatObs[i].dP[1] - Raw.SdObs.SdSatObs[RefPos].dP[1];
                double L1 = Raw.SdObs.SdSatObs[i].dL[0] - Raw.SdObs.SdSatObs[RefPos].dL[0];
                double L2 = Raw.SdObs.SdSatObs[i].dL[1] - Raw.SdObs.SdSatObs[RefPos].dL[1];

                // Ambiguity
                if (isFirst)
                {
                    N1 = (L1 - P1) / WL1;
                    N2 = (L2 - P2) / WL2;
                }
                else
                {
                    N1 = Na(index / 2, 0) + X(3 + index / 2, 0);
                    N2 = Na(1 + index / 2, 0) + X(4 + index / 2, 0);
                }
                Na(index / 2, 0) = N1;
                Na(1 + index / 2, 0) = N2;

                double baseSatDis = Distance(Base.Pos, Raw.BasEpk.SatMidRes[Raw.SdObs.SdSatObs[i].nBas].SatPos);
                double baseRefSatDis = Distance(Base.Pos, Raw.BasEpk.SatMidRes[Raw.SdObs.SdSatObs[RefPos].nBas].SatPos);

                double rho = (rovSatDis - rovRefSatDis) - (baseSatDis - baseRefSatDis);

                W(index, 0) = P1 - rho;
                W(index + 1, 0) = P2 - rho;
                W(index + 2, 0) = L1 - rho - WL1 * N1;
                W(index + 3, 0) = L2 - rho - WL2 * N2;

                double sigmaL = sqrt(configRTK.code_carrier_phase_noise);
                double sigmaP = sqrt(configRTK.code_pseudorange_noise);

                if (Raw.SdObs.SdSatObs[i].System != lastSys)
                {
                    PIndex += 4 * lastn;
                    lastSys = Raw.SdObs.SdSatObs[i].System;
                }

                for (size_t i = 0; i < n; i++)
                {
                    if (index == PIndex + 4 * i)
                    {
                        P(index, index) = n / (2 * sigmaP * sigmaP * (n + 1));
                        P(index + 1, index + 1) = n / (2 * sigmaP * sigmaP * (n + 1));
                        P(index + 2, index + 2) = n / (2 * sigmaL * sigmaL * (n + 1));
                        P(index + 3, index + 3) = n / (2 * sigmaL * sigmaL * (n + 1));
                        continue;
                    }
                    P(index, PIndex + 4 * i) = -1 / (2 * sigmaP * sigmaP * (n + 1));
                    P(index + 1, PIndex + 4 * i + 1) = -1 / (2 * sigmaP * sigmaP * (n + 1));
                    P(index + 2, PIndex + 4 * i + 2) = -1 / (2 * sigmaL * sigmaL * (n + 1));
                    P(index + 3, PIndex + 4 * i + 3) = -1 / (2 * sigmaL * sigmaL * (n + 1));
                }
            }
            else
            {
                continue;
            }
        }

        // Least squares estimation
        Matrix B_T(3 + 2 * SatNum, 4 * SatNum), tempB(3 + 2 * SatNum, 4 * SatNum), N_BB(3 + 2 * SatNum, 3 + 2 * SatNum), BTPW(3 + 2 * SatNum, 1);
        B_T.MatrixTransposition(B);
        tempB.MatrixMultiplication(B_T, P);
        N_BB.MatrixMultiplication(tempB, B);
        Q.InverseMatrix(N_BB);
        BTPW.MatrixMultiplication(tempB, W);
        X.MatrixMultiplication(Q, BTPW);

        RovPos[0] += X(0, 0);
        RovPos[1] += X(1, 0);
        RovPos[2] += X(2, 0);

        isFirst = false;

        if (X.Magnitude() < 1e-6)
        {
            isSuccess = true;
            break;
        }
    }
    
    if (!isSuccess)
    {
        return false;
    }

    // Preparation for the fixed solution
    Matrix Q2(2 * SatNum, 2 * SatNum);
    for (size_t i = 0; i < Q2.getRows(); i++)
    {
        for (size_t j = 0; j < Q2.getCols(); j++)
        {
            Q2(i, j) = Q(i + 3, j + 3);
        }
    }
    Q = Q2;

    Qrow = Q.getRows();
    QPtr = new double[Qrow * Qrow];
    aPtr = new double[Qrow];

    MatToPtr(Q, QPtr);
    MatToPtr(Na, aPtr);

    Raw.DDObs.Time = Base.Time;
    Raw.DDObs.bFixed = false;
    for (int i = 0; i < 3; i++)
    {
        Raw.DDObs.Result.Pos[i] = RovPos[i];
    }

    // Precision evaluation
    Matrix tempW(1, 4 * SatNum), sigmaMat(1, 1);
    W_T.MatrixTransposition(W);
    tempW.MatrixMultiplication(W_T, P);
    sigmaMat.MatrixMultiplication(tempW, W);
    Raw.DDObs.Result.SigmaPos = sqrt((sigmaMat)(0, 0) / (2 * SatNum - 3));
    Raw.DDObs.Result.PDOP = sqrt(Q2(0, 0) + Q2(1, 1) + Q2(2, 2));

    return true;
}

void RTK::MatToPtr(const Matrix& m, double* p)
{
    for (int i = 0; i < m.getRows(); i++)
    {
        for (int j = 0; j < m.getCols(); j++)
        {
            p[i + j * m.getRows()] = m(i, j);
        }
    }
}

bool RTK::RtkFixed(RAWDAT& Raw, const POSRES& Base, const POSRES& Rov)
{
    for (int m = 0; m < 3; m++)
    {
        RovPos[m] = Rov.Pos[m];
    }

    double* F = new double[Qrow * 2];
    double* s = new double[2];

    // Lambda
    if (lambda(Qrow, 2, aPtr, QPtr, F, s) != 0)
    {
        delete[] QPtr, aPtr, F, s;
        return false;
    }
    double ratio = s[1] / s[0];
    Raw.DDObs.Ratio = ratio;
    if (ratio <= 3)
    {
        delete[] QPtr, aPtr, F, s;
        return false;
    }
    Matrix f(Qrow, 1);
    for (size_t i = 0; i < Qrow; i++)
    {
        f(i, 0) = lround(F[i]);
    }
    int satNum = Qrow / 2;

    Matrix B(2 * satNum, 3);
    Matrix W(2 * satNum, 1);
    Matrix P(2 * satNum, 2 * satNum);
    Matrix X(3, 1);

    // Iterative computation of the fixed solution
    for (int times = 0; times < 100; times++)
    {
        bool FirstSys = false;
        GNSSSys lastSys = UNKS;
        int satIndex = 0, columnStartIndex = 0;
        for (int i = 0; i < Raw.SdObs.SatNum; i++)
        {
            if (Raw.SdObs.SdSatObs[i].Valid == true)
            {
                if (!FirstSys)
                {
                    FirstSys = true;
                    lastSys = Raw.SdObs.SdSatObs[i].System;
                }

                double WL1 = 0.0, WL2 = 0.0;
                int RefPos = -1;
                int n = 0, lastn = 0;
                if (Raw.SdObs.SdSatObs[i].System == GPS)
                {
                    WL1 = WL1_GPS;
                    WL2 = WL2_GPS;
                    RefPos = Raw.DDObs.RefPos[0];
                    n = Raw.DDObs.DDSatNum[0];
                    lastn = Raw.DDObs.DDSatNum[1];
                }
                else
                {
                    WL1 = WL1_BDS;
                    WL2 = WL3_BDS;
                    RefPos = Raw.DDObs.RefPos[1];
                    n = Raw.DDObs.DDSatNum[1];
                    lastn = Raw.DDObs.DDSatNum[0];
                }

                if (RefPos == i)
                {
                    continue;
                }

                double rovSatDis = Distance(RovPos, Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[i].nRov].SatPos);
                double rovRefSatDis = Distance(RovPos, Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[RefPos].nRov].SatPos);

                auto rowStartIndex = 2 * satIndex;
                for (int mm = 0; mm < 2; mm++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        B(rowStartIndex + mm, j) = (RovPos[j] - Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[i].nRov].SatPos[j]) / rovSatDis - (RovPos[j] - Raw.RovEpk.SatMidRes[Raw.SdObs.SdSatObs[RefPos].nRov].SatPos[j]) / rovRefSatDis;
                    }
                }

                double L1 = Raw.SdObs.SdSatObs[i].dL[0] - Raw.SdObs.SdSatObs[RefPos].dL[0];
                double L2 = Raw.SdObs.SdSatObs[i].dL[1] - Raw.SdObs.SdSatObs[RefPos].dL[1];

                // Integer ambiguity
                double N1 = f(2 * satIndex, 0);
                double N2 = f(2 * satIndex + 1, 0);

                double baseSatDis = Distance(Base.Pos, Raw.BasEpk.SatMidRes[Raw.SdObs.SdSatObs[i].nBas].SatPos);
                double baseRefSatDis = Distance(Base.Pos, Raw.BasEpk.SatMidRes[Raw.SdObs.SdSatObs[RefPos].nBas].SatPos);

                double rho = (rovSatDis - rovRefSatDis) - (baseSatDis - baseRefSatDis);

                W(rowStartIndex, 0) = L1 - rho - WL1 * N1;
                W(rowStartIndex + 1, 0) = L2 - rho - WL2 * N2;

                double sigmaL = 1;

                if (Raw.SdObs.SdSatObs[i].System != lastSys)
                {
                    columnStartIndex += 2 * lastn;
                    lastSys = Raw.SdObs.SdSatObs[i].System;
                }

                for (auto i = 0; i < n; i++)
                {
                    if (rowStartIndex == columnStartIndex + 2 * i)
                    {
                        P(rowStartIndex, rowStartIndex) = n / (2 * sigmaL * sigmaL * (n + 1));
                        P(rowStartIndex + 1, rowStartIndex + 1) = n / (2 * sigmaL * sigmaL * (n + 1));
                        continue;
                    }
                    P(rowStartIndex, columnStartIndex + 2 * i) = -1 / (2 * sigmaL * sigmaL * (n + 1));
                    P(rowStartIndex + 1, columnStartIndex + 2 * i + 1) = -1 / (2 * sigmaL * sigmaL * (n + 1));
                }
                satIndex++;
            }
            else
            {
                continue;
            }
        }

        // Least squares estimation
        Matrix Bt(3, 2 * satNum), tempB(3, 2 * satNum), N_BB(3, 3), Q(3, 3), BTPW(3, 1);
        Bt.MatrixTransposition(B);
        tempB.MatrixMultiplication(Bt, P);
        N_BB.MatrixMultiplication(tempB, B);
        Q.InverseMatrix(N_BB);
        BTPW.MatrixMultiplication(tempB, W);
        X.MatrixMultiplication(Q, BTPW);

        RovPos[0] += X(0, 0);
        RovPos[1] += X(1, 0);
        RovPos[2] += X(2, 0);

        // Precision evaluation
        if (X.Magnitude() <= 1e-6)
        {
            Raw.DDObs.bFixed = true;
            for (int i = 0; i < 3; i++)
            {
                Raw.DDObs.Result.Pos[i] = RovPos[i];
            }
            Matrix W_T(1, 2 * satNum), tempW(1, 2 * satNum), sigmaMat(1, 1);
            W_T.MatrixTransposition(W);
            tempW.MatrixMultiplication(W_T, P);
            sigmaMat.MatrixMultiplication(tempW, W);
            Raw.DDObs.Result.SigmaPos = sqrt((sigmaMat)(0, 0) / (2 * satNum - 3));
            Raw.DDObs.Result.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
            delete[] F, s;
            return true;
        }
    }

    delete[] F, s;
    return false;
}
