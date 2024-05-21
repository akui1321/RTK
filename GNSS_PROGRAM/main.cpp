#include "HexDecoder.h"
#include "ErrorCorrection.h"
#include "OutlierDetection.h"
#include "SPP.h"
#include "sockets.h"
#include "RTK.h"
#include "ProcessCondig.h"
#include <fstream>
#include <iomanip>
#include <string>

bool SynObsRealTime(const EpoRange& range, HexDecoder& decoder, SOCKET NetGpsB)
{
    //Preparation for Solution
    static int bufferLength = 0;
    static unsigned char buffer[MAXRAWLEN * 3];
    static unsigned char Buff[MAXRAWLEN];
    static bool recvSuccess = false;

    while (1)
    {
        Sleep(245);

        //Input Data into the Buffer When There is Sufficient Space in the Buffer
        int temp = bufferLength;
        if (temp < MAXRAWLEN * 2)
        {
            if ((bufferLength = recv(NetGpsB, (char*)Buff, MAXRAWLEN, 0)) > 0)
            {
                if (!recvSuccess)
                {
                    recvSuccess = true;
                }
                memcpy(buffer + temp, Buff, bufferLength);
                bufferLength += temp;
            }
            else
            {
                bufferLength = 0;
                if (recvSuccess)
                {
                    std::cout << "Failed to read port data, attempting to retry..." << std::endl;
                    recvSuccess = false;
                }
                continue;
            }
        }

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

int main() 
{
    std::cout << std::endl
        << "*****************************************************************" << std::endl
        << std::endl
        << "Welcome to the Satellite Navigation Positioning Solution Program! " << std::endl
        << std::endl
        << "*****************************************************************" << std::endl
        << std::endl;

    // Load Configuration
    GNSSConfig config;
    ReadConfig("D:\\GNSS_PROGRAM\\RTKConfig.cfg", config);

    std::ofstream output(config.position_result_file);
    if (!output.is_open())
    {
        std::cerr << "Unable to open the output file!" << std::endl;
    }

    std::cout << std::endl
        << std::setw(6) << "NO." << std::setw(5) << "MODE" << std::setw(10) << "STATUS"
        << std::setw(8) << "GPST_WN" << std::setw(11) << "GPST_SOW"
        << std::setw(15) << "ROV_X" << std::setw(15) << "ROV_Y" << std::setw(15) << "ROV_Z"
        << std::setw(15) << "ROV_B" << std::setw(15) << "ROV_L" << std::setw(15) << "ROV_H"
        << std::setw(15) << "BASELINE_X" << std::setw(15) << "BASELINE_Y" << std::setw(15) << "BASELINE_Z"
        << std::setw(9) << "RATIO" << std::setw(7) << "PDOP" << std::setw(7) << "SIGMA"
        << std::setw(4) << "GS" << std::setw(4) << "BS" << std::setw(4) << "n"
        << std::endl;

    output << std::endl
        << std::setw(6) << "NO." << std::setw(5) << "MODE" << std::setw(10) << "STATUS"
        << std::setw(8) << "GPST_WN" << std::setw(11) << "GPST_SOW"
        << std::setw(15) << "ROV_X" << std::setw(15) << "ROV_Y" << std::setw(15) << "ROV_Z"
        << std::setw(15) << "ROV_B" << std::setw(15) << "ROV_L" << std::setw(15) << "ROV_H"
        << std::setw(15) << "BASELINE_X" << std::setw(15) << "BASELINE_Y" << std::setw(15) << "BASELINE_Z"
        << std::setw(9) << "RATIO" << std::setw(7) << "PDOP" << std::setw(7) << "SIGMA"
        << std::setw(4) << "GS" << std::setw(4) << "BS" << std::setw(4) << "n"
        << std::endl;

    if (!config.com_or_file)
    {
        /********************************Post-Processing********************************/

        //Open the RFile
        std::string inputRFile = config.obsdata_rfile;
        FILE* Rfile;
        errno_t err2 = fopen_s(&Rfile, inputRFile.c_str(), "rb");
        if (!Rfile)
        {
            std::cerr << "Error: Unable to open Rfile." << std::endl;
            return 0;
        }

        //Open the BFile
        std::string inputBFile = config.obsdata_bfile;
        FILE* Bfile;
        errno_t err1 = fopen_s(&Bfile, inputBFile.c_str(), "rb");
        if (!Bfile)
        {
            std::cerr << "Error: Unable to open Bfile." << std::endl;
            return 0;
        }

        //Preparation for Solution
        int bufferLength = 0;
        unsigned char buffer[MAXRAWLEN];
        HexDecoder decoder, decoderBase;
        OutlierDetection detection, detectionBase;
        SPP Solution, SolutionBase;
        bool solveSuccess = false, solveSuccessBase = false;

        RAWDAT Rov;
        EpoRange BeObsB, BeObsR;
        int NO = 0;
        std::string Mode;
        if (config.Mode == RTKMode)
        {
            Mode = "RTK";
        }
        else
        {
            Mode = "SPP";
        }
        std::string STATUS;

        bool FIXEDSuccess = false;
        bool DetSuccess = false, FLOATSuccess = false;

        //Repeatedly Read the File
        while (!feof(Rfile))
        {
            //Read Data into Buffer
            int remainingSpace = MAXRAWLEN - bufferLength;
            size_t bytesRead = fread(buffer + bufferLength, sizeof(unsigned char), remainingSpace, Rfile);
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

            //Decoding Results
            if (!success)
            {
                std::cout << "Decoding Failed!" << std::endl;
            }
            else if (success == 1)
            {
                RTK rtk(config);

                //Syn
                int successSyn = rtk.SynObs(decoder.range, Bfile, decoderBase);

                if (!successSyn)
                {
                    std::cout << "No Base!" << std::endl;
                    continue;
                }

                if (BeObsB.satRange.size() > 0)
                {
                    rtk.DetectLockTime(BeObsB, decoderBase.range);
                }
                if (BeObsR.satRange.size() > 0)
                {
                    rtk.DetectLockTime(BeObsR, decoder.range);
                }

                //Solution Using Observations
                detection.outlierDetection(decoder.range);
                detectionBase.outlierDetection(decoderBase.range);

                if (Solution.SolvePosition(decoder.range, decoder.GPSEph, decoder.BDSEph))
                {
                    if (Solution.SolveVelocity(decoder.range, decoder.GPSEph, decoder.BDSEph))
                    {
                        Rov.BdsEph = decoder.BDSEph;
                        Rov.GpsEph = decoder.GPSEph;
                        Rov.RovEpk = decoder.range;
                        solveSuccess = true;

                        if (SolutionBase.SolvePosition(decoderBase.range, decoderBase.GPSEph, decoderBase.BDSEph))
                        {
                            if (SolutionBase.SolveVelocity(decoderBase.range, decoderBase.GPSEph, decoderBase.BDSEph))
                            {
                                Rov.BasEpk = decoderBase.range;
                                solveSuccessBase = true;
                            }
                        }

                        rtk.FormSDEpochObs(decoder.range, decoderBase.range, Rov.SdObs);

                        rtk.DetectCycleSlip(Rov.SdObs);

                        DetSuccess = rtk.DetRefSat(decoder.range, decoderBase.range, Rov.SdObs, Rov.DDObs);

                        if (DetSuccess)
                        {
                            FLOATSuccess = rtk.RTKFloat(Rov, decoderBase.PosRes, Solution.PosRes);

                            if (FLOATSuccess)
                            {
                                FIXEDSuccess = rtk.RtkFixed(Rov, decoderBase.PosRes, Solution.PosRes);
                            }
                        }

                        BeObsB = decoderBase.range;
                        BeObsR = decoder.range;
                    }
                }

                decoder.range.satRange.clear();
                decoder.range.ComObs.clear();
                decoderBase.range.satRange.clear();
                decoderBase.range.ComObs.clear();
            }
            else
            {
                NO++;
                //Upon Reading BestPosition, Compute Positioning Error, and Output Solution Results
                Coordinate BLHRes(1, Rov.DDObs.Result.Pos[0], Rov.DDObs.Result.Pos[1], Rov.DDObs.Result.Pos[2]);
                BLHRes.XYZtoBLH(R_CGS2K, F_CGS2K);
                Solution.PosRes.BLHPos[0] = BLHRes.B / PI * 180.0;
                Solution.PosRes.BLHPos[1] = BLHRes.L / PI * 180.0;
                Solution.PosRes.BLHPos[2] = BLHRes.H;

                Coordinate BLHRef(1, decoder.PosRes.Pos[0], decoder.PosRes.Pos[1], decoder.PosRes.Pos[2]);
                BLHRef.XYZtoBLH(R_CGS2K, F_CGS2K);
                BLHRef.PositioningError(decoder.PosRes.Pos, Solution.PosRes.Pos);
                Solution.PosRes.N = BLHRef.N;
                Solution.PosRes.E = BLHRef.E;
                Solution.PosRes.U = BLHRef.U;

                Rov.DDObs.Result.Baseline_X = Rov.DDObs.Result.Pos[0] - decoderBase.PosRes.Pos[0];
                Rov.DDObs.Result.Baseline_Y = Rov.DDObs.Result.Pos[1] - decoderBase.PosRes.Pos[1];
                Rov.DDObs.Result.Baseline_Z = Rov.DDObs.Result.Pos[2] - decoderBase.PosRes.Pos[2];

                if (FIXEDSuccess)
                {
                    STATUS = "FIXED";
                }
                else
                {
                    STATUS = "FLOAT";
                }

                if (solveSuccess && DetSuccess && FLOATSuccess)
                {
                    std::cout << std::setw(6) << NO << std::setw(5) << Mode << std::setw(10) << STATUS
                        << std::setw(8) << Solution.PosRes.Time.gpsWeek << std::setw(11) << std::fixed << std::setprecision(3) << Solution.PosRes.Time.gpsSeconds
                        << std::setw(15) << Rov.DDObs.Result.Pos[0] << std::setw(15) << Rov.DDObs.Result.Pos[1] << std::setw(15) << Rov.DDObs.Result.Pos[2]
                        << std::setw(15) << std::setprecision(8) << Solution.PosRes.BLHPos[0] << std::setw(15) << Solution.PosRes.BLHPos[1] << std::setw(15) << std::setprecision(3) << Solution.PosRes.BLHPos[2]
                        << std::setw(15) << Rov.DDObs.Result.Baseline_X << std::setw(15) << Rov.DDObs.Result.Baseline_Y << std::setw(15) << Rov.DDObs.Result.Baseline_Z
                        << std::setw(9) << Rov.DDObs.Ratio << std::setw(7) << Rov.DDObs.Result.PDOP << std::setw(7) << Rov.DDObs.Result.SigmaPos
                        << std::setw(4) << std::setprecision(0) << Solution.PosRes.GPSCount << std::setw(4) << Solution.PosRes.BDSCount << std::setw(4) << Solution.PosRes.SatNum
                        << std::endl;

                    output << std::setw(6) << NO << std::setw(5) << Mode << std::setw(10) << STATUS
                        << std::setw(8) << Solution.PosRes.Time.gpsWeek << std::setw(11) << std::fixed << std::setprecision(3) << Solution.PosRes.Time.gpsSeconds
                        << std::setw(15) << Rov.DDObs.Result.Pos[0] << std::setw(15) << Rov.DDObs.Result.Pos[1] << std::setw(15) << Rov.DDObs.Result.Pos[2]
                        << std::setw(15) << std::setprecision(8) << Solution.PosRes.BLHPos[0] << std::setw(15) << Solution.PosRes.BLHPos[1] << std::setw(15) << std::setprecision(3) << Solution.PosRes.BLHPos[2]
                        << std::setw(15) << Rov.DDObs.Result.Baseline_X << std::setw(15) << Rov.DDObs.Result.Baseline_Y << std::setw(15) << Rov.DDObs.Result.Baseline_Z
                        << std::setw(9) << Rov.DDObs.Ratio << std::setw(7) << Rov.DDObs.Result.PDOP << std::setw(7) << Rov.DDObs.Result.SigmaPos
                        << std::setw(4) << std::setprecision(0) << Solution.PosRes.GPSCount << std::setw(4) << Solution.PosRes.BDSCount << std::setw(4) << Solution.PosRes.SatNum
                        << std::endl;
                    solveSuccess = false;
                    DetSuccess = false;
                    FLOATSuccess = false;
                }
                else
                {
                    std::cout << std::setw(6) << NO << "  Solution Unsuccessful!" << std::endl;
                }

                FIXEDSuccess = false;
            }
        }

        fclose(Rfile);
        fclose(Bfile);
    }
    else
    {
        /******************************Real-Time Processing******************************/

        //Open the Ip & Port
        SOCKET NetGpsB, NetGpsR;
        int PortB, PortR;
        PortB = config.port_B;
        PortR = config.port_R;
        const char* IP_ADDRESS_R = config.ip_address_R.c_str();
        const char* IP_ADDRESS_B = config.ip_address_B.c_str();
        if (OpenSocket(NetGpsR, IP_ADDRESS_R, PortR) == false)
        {
            std::cout << "This ip & port was not opened." << std::endl;
            return 0;
        }

        if (OpenSocket(NetGpsB, IP_ADDRESS_B, PortB) == false)
        {
            std::cout << "This ip & port was not opened." << std::endl;
            return 0;
        }

        //Preparation for Solution
        int bufferLength = 0;
        unsigned char buffer[MAXRAWLEN * 3];
        unsigned char Buff[MAXRAWLEN];
        HexDecoder decoder, decoderBase;
        OutlierDetection detection, detectionBase;
        SPP Solution, SolutionBase;
        bool solveSuccess = false, solveSuccessBase = false;
        bool recvSuccess = true;

        RAWDAT Rov;
        EpoRange BeObsB, BeObsR;
        int NO = 0;
        std::string Mode;
        if (config.Mode == RTKMode)
        {
            Mode = "RTK";
        }
        else
        {
            Mode = "SPP";
        }
        std::string STATUS;

        bool FIXEDSuccess = false;

        Sleep(980);

        while (1)
        {
            Sleep(245);

            //Input Data into the Buffer When There is Sufficient Space in the Buffer
            int temp = bufferLength;
            if (temp < MAXRAWLEN * 2)
            {
                if ((bufferLength = recv(NetGpsR, (char*)Buff, MAXRAWLEN, 0)) > 0)
                {
                    if (!recvSuccess)
                    {
                        recvSuccess = true;
                    }
                    memcpy(buffer + temp, Buff, bufferLength);
                    bufferLength += temp;
                }
                else
                {
                    bufferLength = 0;
                    if (recvSuccess)
                    {
                        std::cout << "Failed to read port data, attempting to retry..." << std::endl;
                        recvSuccess = false;
                    }
                    continue;
                }
            }

            //Decode
            int blockEnd = 0;
            int blockStart = 0;
            bool isIncompleteBlock = false;
            int success = decoder.BufferReader(bufferLength, buffer, blockEnd, blockStart, isIncompleteBlock);
            bool DetSuccess = false, FLOATSuccess = false;

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
                    //Discard Processed Data
                    int remainingLength = bufferLength - blockEnd;
                    memmove(buffer, buffer + blockEnd, remainingLength);
                    memset(buffer + remainingLength, 0, blockEnd);
                    bufferLength = remainingLength;
                }
            }

            //Decoding Results
            if (!success)
            {
                std::cout << "Decoding Failed!" << std::endl;
            }
            else if (success == 1)
            {
                RTK rtk(config);

                //Syn
                int successSyn = SynObsRealTime(decoder.range, decoderBase, NetGpsB);

                if (!successSyn)
                {
                    std::cout << "No Base!" << std::endl;
                    continue;
                }

                if (BeObsB.satRange.size() > 0)
                {
                    rtk.DetectLockTime(BeObsB, decoderBase.range);
                }
                if (BeObsR.satRange.size() > 0)
                {
                    rtk.DetectLockTime(BeObsR, decoder.range);
                }

                //Solution Using Observations
                detection.outlierDetection(decoder.range);
                detectionBase.outlierDetection(decoderBase.range);

                if (Solution.SolvePosition(decoder.range, decoder.GPSEph, decoder.BDSEph))
                {
                    if (Solution.SolveVelocity(decoder.range, decoder.GPSEph, decoder.BDSEph))
                    {
                        Rov.BdsEph = decoder.BDSEph;
                        Rov.GpsEph = decoder.GPSEph;
                        Rov.RovEpk = decoder.range;
                        solveSuccess = true;

                        if (SolutionBase.SolvePosition(decoderBase.range, decoderBase.GPSEph, decoderBase.BDSEph))
                        {
                            if (SolutionBase.SolveVelocity(decoderBase.range, decoderBase.GPSEph, decoderBase.BDSEph))
                            {
                                Rov.BasEpk = decoderBase.range;
                                solveSuccessBase = true;
                            }
                        }

                        rtk.FormSDEpochObs(decoder.range, decoderBase.range, Rov.SdObs);

                        rtk.DetectCycleSlip(Rov.SdObs);

                        DetSuccess = rtk.DetRefSat(decoder.range, decoderBase.range, Rov.SdObs, Rov.DDObs);

                        if (DetSuccess)
                        {
                            FLOATSuccess = rtk.RTKFloat(Rov, decoderBase.PosRes, Solution.PosRes);
                            
                            if (FLOATSuccess)
                            {
                                FIXEDSuccess = rtk.RtkFixed(Rov, decoderBase.PosRes, Solution.PosRes);
                            }
                        }

                        BeObsB = decoderBase.range;
                        BeObsR = decoder.range;
                    }
                }

                decoder.range.satRange.clear();
                decoder.range.ComObs.clear();
                decoderBase.range.satRange.clear();
                decoderBase.range.ComObs.clear();

                NO++;
                Coordinate BLHRes(1, Rov.DDObs.Result.Pos[0], Rov.DDObs.Result.Pos[1], Rov.DDObs.Result.Pos[2]);
                BLHRes.XYZtoBLH(R_CGS2K, F_CGS2K);
                Solution.PosRes.BLHPos[0] = BLHRes.B / PI * 180.0;
                Solution.PosRes.BLHPos[1] = BLHRes.L / PI * 180.0;
                Solution.PosRes.BLHPos[2] = BLHRes.H;

                Rov.DDObs.Result.Baseline_X = Rov.DDObs.Result.Pos[0] - decoderBase.PosRes.Pos[0];
                Rov.DDObs.Result.Baseline_Y = Rov.DDObs.Result.Pos[1] - decoderBase.PosRes.Pos[1];
                Rov.DDObs.Result.Baseline_Z = Rov.DDObs.Result.Pos[2] - decoderBase.PosRes.Pos[2];

                if (FIXEDSuccess)
                {
                    STATUS = "FIXED";
                }
                else
                {
                    STATUS = "FLOAT";
                }

                if (solveSuccess && DetSuccess && FLOATSuccess)
                {
                    std::cout << std::setw(6) << NO << std::setw(5) << Mode << std::setw(10) << STATUS
                        << std::setw(8) << Solution.PosRes.Time.gpsWeek << std::setw(11) << std::fixed << std::setprecision(3) << Solution.PosRes.Time.gpsSeconds
                        << std::setw(15) << Rov.DDObs.Result.Pos[0] << std::setw(15) << Rov.DDObs.Result.Pos[1] << std::setw(15) << Rov.DDObs.Result.Pos[2]
                        << std::setw(15) << std::setprecision(8) << Solution.PosRes.BLHPos[0] << std::setw(15) << Solution.PosRes.BLHPos[1] << std::setw(15) << std::setprecision(3) << Solution.PosRes.BLHPos[2]
                        << std::setw(15) << Rov.DDObs.Result.Baseline_X << std::setw(15) << Rov.DDObs.Result.Baseline_Y << std::setw(15) << Rov.DDObs.Result.Baseline_Z
                        << std::setw(9) << Rov.DDObs.Ratio << std::setw(7) << Rov.DDObs.Result.PDOP << std::setw(7) << Rov.DDObs.Result.SigmaPos
                        << std::setw(4) << std::setprecision(0) << Solution.PosRes.GPSCount << std::setw(4) << Solution.PosRes.BDSCount << std::setw(4) << Solution.PosRes.SatNum
                        << std::endl;

                    output << std::setw(6) << NO << std::setw(5) << Mode << std::setw(10) << STATUS
                        << std::setw(8) << Solution.PosRes.Time.gpsWeek << std::setw(11) << std::fixed << std::setprecision(3) << Solution.PosRes.Time.gpsSeconds
                        << std::setw(15) << Rov.DDObs.Result.Pos[0] << std::setw(15) << Rov.DDObs.Result.Pos[1] << std::setw(15) << Rov.DDObs.Result.Pos[2]
                        << std::setw(15) << std::setprecision(8) << Solution.PosRes.BLHPos[0] << std::setw(15) << Solution.PosRes.BLHPos[1] << std::setw(15) << std::setprecision(3) << Solution.PosRes.BLHPos[2]
                        << std::setw(15) << Rov.DDObs.Result.Baseline_X << std::setw(15) << Rov.DDObs.Result.Baseline_Y << std::setw(15) << Rov.DDObs.Result.Baseline_Z
                        << std::setw(9) << Rov.DDObs.Ratio << std::setw(7) << Rov.DDObs.Result.PDOP << std::setw(7) << Rov.DDObs.Result.SigmaPos
                        << std::setw(4) << std::setprecision(0) << Solution.PosRes.GPSCount << std::setw(4) << Solution.PosRes.BDSCount << std::setw(4) << Solution.PosRes.SatNum
                        << std::endl;
                    solveSuccess = false;
                    DetSuccess = false;
                    FLOATSuccess = false;
                }
                else
                {
                    std::cout << std::setw(6) << NO << "  Solution Unsuccessful!" << std::endl;
                }

                FIXEDSuccess = false;
            }
        }
    }

    output.close();

    return 0;
}
