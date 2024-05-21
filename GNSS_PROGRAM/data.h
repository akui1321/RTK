#pragma once

#include <vector>
#include <string>
#include "DateTime.h"
#include "Matrix.h"

//Definition of Constants
#define C_Light 299792458.0 
#define PI 3.1415926535897932384626433832795028841971
#define GPS_GM 3.986005e14
#define GPS_OMEGA_DOT_e 7.2921151467e-5
#define BDS_GM 3.986004418e14
#define BDS_OMEGA_DOT_e 7.292115e-5
#define POLYCRC32 0xEDB88320u
#define FG1_GPS 1575.42E6             /* L1 signal frequency */
#define FG2_GPS 1227.60E6             /* L2 signal frequency */
#define WL1_GPS (C_Light/FG1_GPS)
#define WL2_GPS (C_Light/FG2_GPS)
#define FG1_BDS 1561.098E6               /* The reference frequency of the B1 signal */
#define FG2_BDS 1207.140E6               /* The reference frequency of the B2 signal */
#define FG3_BDS 1268.520E6               /* The reference frequency of the B3 signal */
#define WL1_BDS (C_Light/FG1_BDS)
#define WL2_BDS (C_Light/FG2_BDS)
#define WL3_BDS (C_Light/FG3_BDS)
#define T_0 288.16
#define H_0 0
#define p_0 1013.25
#define RH_0 0.5
#define F_ -4.442807633e-10
#define Omega_WGS 7.2921151467e-5
#define Omega_BDS 7.2921150e-5 
#define R_WGS84 6378137.0  
#define F_WGS84 1.0/298.257223563
#define R_CGS2K 6378137.0 
#define F_CGS2K 1.0/298.257222101
#define MAXRAWLEN 40960
#define OBSERVATION_DATA_TYPE 43
#define GPS_EPHEMERIS_TYPE 7
#define BDS_EPHEMERIS_TYPE 1696
#define POSITION_RESULT_TYPE 42
#define MAXCHANNUM 36
#define MAXGPSNUM  32
#define MAXBDSNUM 63

//Satellite System
enum GNSSSys
{
    UNKS = 0,
    GPS,
    BDS
};

//Solution Mode
enum MODE
{
    NONEMode = 0,
    SPPMode,
    RTKMode
};

//Satellite Information
struct SATMIDRES
{
    double t;
    double SatPos[3], SatVel[3];
    double SatClkOft, SatClkSft;
    double Elevation, Azimuth;
    double TropCorr;
    double Tgd1, Tgd2;
    bool Valid;
    SATMIDRES()
    {
        t = 0.0;
        SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
        SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
        Elevation = PI / 2.0;
        Azimuth = 0.0;
        SatClkOft = SatClkSft = 0.0;
        Tgd1 = Tgd2 = TropCorr = 0.0;
        Valid = false;
    }
};

//Combined Observations
struct MWGF
{
    short Prn;
    GNSSSys Sys;
    double MW;
    double GF;
    double PIF;
    int n;
    MWGF()
    {
        Sys = UNKS;
        Prn = n = 0;
        MW = GF = PIF = 0.0;
    }
};

//The Satellite Observations
struct SatRange
{
    short PRN;
    GNSSSys system;
    double P[2], L[2], D[2];
    bool valid;
    double cn0[2];
    double LockTime[2];
    double Parity[2];

    SatRange()
    {
        PRN = 0;
        system = UNKS;
        valid = false;
        for (int i = 0; i < 2; ++i)
        {
            P[i] = 0.0;
            L[i] = 0.0;
            D[i] = 0.0;
            cn0[i] = 0.0;
            LockTime[i] = 0.0;
            Parity[i] = 0.0;
        }
    }
};

//Observations at Each Epoch
struct EpoRange
{
    DateTime Time;
    short SatNum;
    std::vector<SatRange> satRange;
    std::vector<SATMIDRES> SatMidRes;
    std::vector<MWGF> ComObs;

    EpoRange() : Time(0, 0.0)
    {
        SatNum = 0;
    }
};

//GPS Ephemeris
struct GPSEPHREC
{
    short PRN;
    GNSSSys System;
    double TOC, TOE;
    double ClkBias, ClkDrift, ClkDriftRate;
    double IODE, IODC;
    double SqrtA, M0, e, OMEGA, i0, omega;
    double Crs, Cuc, Cus, Cic, Cis, Crc;
    double DeltaN, OMEGADot, iDot;
    int SVHealth;
    double TGD1, TGD2;
    GPSEPHREC() 
    {
        PRN = 0;
        SVHealth = 0;
        System = UNKS;
        ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
        SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
        Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;
    }
};

//BDS Ephemeris
struct BDSEPHREC
{
    short PRN;
    GNSSSys System;
    double TOC, TOE;
    double ClkBias, ClkDrift, ClkDriftRate;
    double IODE, IODC;
    double SqrtA, M0, e, OMEGA, i0, omega;
    double Crs, Cuc, Cus, Cic, Cis, Crc;
    double DeltaN, OMEGADot, iDot;
    int SVHealth;
    double TGD1, TGD2;
    BDSEPHREC()
    {
        PRN = 0;
        SVHealth = 0;
        System = UNKS;
        ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
        SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
        Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;
    }
};

//Position Result
struct POSRES
{
    DateTime Time;
    double Pos[3], Vel[3];
    double BLHPos[3];
    double N, E, U;
    int GPSCount, BDSCount, SatNum;
    double PDOP, SigmaPos, SigmaVel;
    double Baseline_X, Baseline_Y, Baseline_Z;

    POSRES() : Time(0, 0.0)
    {
        Pos[0] = Pos[1] = Pos[2] = 0.0;
        Vel[0] = Vel[1] = Vel[2] = 0.0;
        BLHPos[0] = BLHPos[1] = BLHPos[2] = 0.0;
        N = E = U = 0.0;
        PDOP = SigmaPos = SigmaVel = 0.0;
        GPSCount = BDSCount = SatNum = 0;
        Baseline_X = Baseline_Y = Baseline_Z = 0.0;
    }
};

//Single Difference Observation
struct SDSATOBS
{
    short    Prn;
    GNSSSys  System;
    short    Valid;
    double   dP[2], dL[2];   
    short    nBas, nRov;

    SDSATOBS()
    {
        Prn = nBas = nRov = 0;
        System = UNKS;
        dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
        Valid = -1;
    }
};

//Single Difference Observation at Each Epoch
struct SDEPOCHOBS
{
    DateTime   Time;
    short      SatNum;
    std::vector<SDSATOBS>   SdSatObs;
    std::vector<MWGF>       SdCObs;

    SDEPOCHOBS() : Time(0, 0.0)
    {
        SatNum = 0;
    }
};

//Configuration Information
struct GNSSConfig 
{
    int com_or_file;                 // 0 for file, 1 for COM
    int com_port;
    int com_baud_rate;
    std::string ip_address_B;        // IP address as string
    std::string ip_address_R;
    int port_R;                      // Port number
    int port_B;
    std::string obsdata_rfile;       // OBSDATA source file for RFILE
    std::string obsdata_bfile;       // OBSDATA source file for BFILE
    std::string position_result_file; // Position result file path
    std::string position_diff_file;   // Position diff file path
    double code_pseudorange_noise;   // Code pseudorange noise level
    double code_carrier_phase_noise;
    double elevation_mask_threshold;  // Elevation mask threshold
    MODE Mode;

    GNSSConfig()
    {
        com_or_file = 0;
        com_port = 0;
        com_baud_rate = 0;
        ip_address_R = " ";
        ip_address_B = " ";
        port_R = port_B = 0;
        obsdata_rfile = " ";
        obsdata_bfile = " ";
        position_result_file = " ";
        position_diff_file = " ";
        code_pseudorange_noise = 0.0;
        code_carrier_phase_noise = 0.0;
        elevation_mask_threshold = 0.0;
        Mode = NONEMode;
    }
};

//Double Difference Observation
struct DDCOBS
{
    DateTime Time;
    int RefPrn[2], RefPos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
    int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
    double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
    double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
    float  FixRMS[2];                 // 固定解定位中rms误差
    double dPos[3];                   // 基线向量
    bool bFixed;                      // true为固定，false为未固定
    POSRES Result;

    DDCOBS():Time(0,0.0)
    {
        for (int i = 0; i < 2; i++) {
            DDSatNum[i] = 0;    // 各卫星系统的双差数量
            RefPos[i] = RefPrn[i] = -1;
        }
        for (int i = 0; i < MAXCHANNUM * 2; i++)
        {
            FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
        }
        Sats = 0;              // 双差卫星总数
        dPos[0] = dPos[1] = dPos[2] = 0.0;
        ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
        bFixed = false;
    }
};

//RTK Solution Information
struct RAWDAT 
{
    EpoRange BasEpk;
    EpoRange RovEpk;
    SDEPOCHOBS SdObs;
    DDCOBS DDObs;
    std::vector<GPSEPHREC> GpsEph;
    std::vector<BDSEPHREC> BdsEph;
};

//EKF Solution Information
struct RTKEKF
{
    DateTime Time;
    double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];
    int Index[MAXCHANNUM], nSats, nPos[MAXCHANNUM];
    int FixAmb[MAXCHANNUM];          // 时间更新后上个历元已经固定并传递的模糊度， 1=已固定，-1=未固定或有周跳
    DDCOBS DDObs, CurDDObs;           // 上一个历元和当前历元的双差观测值信息
    SDEPOCHOBS SDObs;                 // 上一个历元的单差观测值
    double X0[3 + MAXCHANNUM * 2], P0[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];  // 状态备份
    bool IsInit;                      // 滤波是否初始化

    RTKEKF(): Time(0,0.0)
    {
        IsInit = false;
        nSats = 0;
        for (int i = 0; i < MAXCHANNUM; i++) nPos[i] = Index[i] = FixAmb[i] = -1;
        for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
            X[i] = X0[i] = 0.0;
            for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P[i * (3 + MAXCHANNUM * 2) + j] = P0[i * (3 + MAXCHANNUM * 2) + j] = 0.0;
        }
    }
};
