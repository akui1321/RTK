#include "HexDecoder.h"

HexDecoder::HexDecoder()
{
}

//Decoding Function
int HexDecoder::BufferReader(const int& bufferLength, const unsigned char* buffer, int& blockEnd, int& blockStart, bool& isIncompleteBlock)
{
    isIncompleteBlock = false;
    int success = 0;
    if (bufferLength > 0)
    {
        unsigned short blockLength = 0;

        while (blockEnd + 2 < bufferLength)
        {
            //Locate the Beginning of the Data Block
            if (buffer[blockEnd] == (unsigned char)0xAA && buffer[blockEnd + 1] == (unsigned char)0x44 && buffer[blockEnd + 2] == (unsigned char)0x12)
            {
                blockStart = blockEnd;
                blockEnd += 28;
                if (blockStart + 9 <= bufferLength)
                {
                    //Calculate file body length
                    blockLength = UI2(buffer + blockStart + 8);
                    blockEnd += (blockLength + 4);
                    if (blockLength > MAXRAWLEN)
                    {
                        blockEnd = blockStart;
                        do
                        {
                            blockEnd++;
                        } while (buffer[blockEnd] != (unsigned char)0xAA || buffer[blockEnd + 1] != (unsigned char)0x44 || buffer[blockEnd + 2] != (unsigned char)0x12);
                        continue;
                    }
                }
                else
                {
                    isIncompleteBlock = true;
                    break;
                }

                //Locate the data block
                if (blockStart + 28 + blockLength + 4 <= bufferLength)
                {
                    //Cyclic Redundancy Check
                    if (crc32(buffer, blockLength, blockStart) == UI4(buffer + blockStart + 28 + blockLength))
                    {
                        HeadReader(buffer, blockStart);

                        if (MessageID == OBSERVATION_DATA_TYPE)
                        {
                            RangeReader(buffer, blockStart);
                            success = 1;
                            break;
                        }
                        else if (MessageID == GPS_EPHEMERIS_TYPE)
                        {
                            GPSEphReader(buffer, blockStart);
                        }
                        else if (MessageID == POSITION_RESULT_TYPE)
                        {
                            BestPosReader(buffer, blockStart);
                            success = 2;
                            break;
                        }
                        else if (MessageID == BDS_EPHEMERIS_TYPE)
                        {
                            BDSEphReader(buffer, blockStart);
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
                else
                {
                    isIncompleteBlock = true;
                    break;
                }
            }
            else
            {
                ++blockEnd;
            }
        }
    }

    return success;
}

//Read the Head
void HexDecoder::HeadReader(const unsigned char* buffer, const int& blockStart)
{
     MessageID = UI2(buffer + blockStart + 4);
     Week = UI2(buffer + blockStart + 14);
     GPSsecond = UI4(buffer + blockStart + 16) * 0.001;
     range.Time = DateTime(Week, GPSsecond);
     PosRes.Time = DateTime(Week, GPSsecond);
}

//Read the Observations
void HexDecoder::RangeReader(const unsigned char* buffer, const int& blockStart)
{
    SatRange SatData;
    int bodyStart = blockStart + 28;
    range.SatNum = 0;

    //Observation Count
    obs = UI4(buffer + bodyStart);
    for (int i = 0; i < obs; ++i) 
    {
        //Systems and Signal Types
        unsigned long sysData = (unsigned char)(buffer[bodyStart + 44 + i * 44]) | ((unsigned char)buffer[bodyStart + 45 + i * 44] << 8) | ((unsigned char)buffer[bodyStart + 46 + i * 44] << 16) | ((unsigned char)buffer[bodyStart + 47 + i * 44] << 24);
        unsigned long sys = (sysData >> 16) & 0x7;
        int type;
        double WL;

        if (sys == 0) 
        {
            SatData.system = GPS;
            SatData.PRN = UI2(buffer + bodyStart + 4 + i * 44);
            unsigned long SignalType = (sysData >> 21) & 0x1F;

            if (SignalType == 0) 
            {
                type = 0;
                WL = WL1_GPS;
            }
            else if (SignalType==9) 
            {
                type = 1;
                WL = WL2_GPS;
            }
            else 
            {
                continue;
            }
        }
        else if (sys == 4) 
        {
            SatData.system = BDS;
            SatData.PRN = UI2(buffer + bodyStart + 4 + i * 44);
            unsigned long SignalType = (sysData >> 21) & 0x1F;

            if (SignalType == 0) 
            {
                type = 0;
                WL = WL1_BDS;
            }
            else if (SignalType == 2) 
            {
                type = 1;
                WL = WL3_BDS;
            }
            else if (SignalType == 4) 
            {
                type = 0;
                WL = WL1_BDS;
            }
            else if (SignalType == 6) 
            {
                type = 1;
                WL = WL3_BDS;
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

        //Pseudorange, Phase, and Doppler Observations
        SatData.P[type] = R8(buffer + bodyStart + 8 + i * 44);
        SatData.L[type] = -R8(buffer + bodyStart + 20 + i * 44) * WL;
        SatData.D[type] = -R4(buffer + bodyStart + 32 + i * 44) * WL;
        SatData.cn0[type] = UI4(buffer + bodyStart + 36 + i * 44);
        SatData.LockTime[type] = UI4(buffer + bodyStart + 40 + i * 44);
        SatData.Parity[type] = (sysData >> 11) & 0x1;

        //Store Observations
        bool find = false;
        if (range.satRange.size() != 0)
        {
            for (int j = 0; j < range.satRange.size(); ++j)
            {
                if ((range.satRange[j].PRN == SatData.PRN) && (range.satRange[j].system == SatData.system))
                {
                    find = true;
                    range.satRange[j].P[type] = SatData.P[type];
                    range.satRange[j].L[type] = SatData.L[type];
                    range.satRange[j].D[type] = SatData.D[type];
                    SatData.PRN = 0;
                    for (int i = 0; i < 2; ++i)
                    {
                        SatData.P[i] = 0.0;
                        SatData.L[i] = 0.0;
                        SatData.D[i] = 0.0;
                    }
                    break;
                }
            }
        }

        if (!find)
        {
            range.satRange.push_back(SatData);
            SatData.PRN = 0;
            for (int i = 0; i < 2; ++i)
            {
                SatData.P[i] = 0.0;
                SatData.L[i] = 0.0;
                SatData.D[i] = 0.0;
            }
        }
    }

    //Checking Observational Data Completeness
    for (int i = 0; i < range.satRange.size(); ++i)
    {
        if ((range.satRange[i].PRN != 0) && (range.satRange[i].L[0] != 0) && (range.satRange[i].L[1] != 0) && (range.satRange[i].P[0] != 0) && (range.satRange[i].P[1] != 0) && (range.satRange[i].D[0] != 0) && (range.satRange[i].D[1] != 0))
        {
            range.satRange[i].valid = true;
        }
        range.SatNum++;
    }
}

//Read GPS Ephemeris
void HexDecoder::GPSEphReader(const unsigned char* buffer, const int& blockStart)
{
    //Periodically Clear Previous GPS Ephemerides
    if (GPSEph.size() >= 120)
    {
        GPSEph.erase(GPSEph.begin(), GPSEph.begin() + 50);
    }

    GPSEPHREC GPSEphTemp;
    int bodyStart = blockStart + 28;
    GPSEphTemp.PRN = UI4(buffer + bodyStart);
    GPSEphTemp.System = GPS;
    GPSEphTemp.SVHealth = UI4(buffer + bodyStart + 12);
    GPSEphTemp.IODC = UI4(buffer + bodyStart + 16);
    GPSEphTemp.IODE = UI4(buffer + bodyStart + 20);
    GPSEphTemp.TOE = R8(buffer + bodyStart + 32);
    GPSEphTemp.SqrtA = R8(buffer + bodyStart + 40);
    GPSEphTemp.DeltaN = R8(buffer + bodyStart + 48);
    GPSEphTemp.M0 = R8(buffer + bodyStart + 56);
    GPSEphTemp.e = R8(buffer + bodyStart + 64);
    GPSEphTemp.omega = R8(buffer + bodyStart + 72);
    GPSEphTemp.Cuc = R8(buffer+bodyStart + 80);
    GPSEphTemp.Cus = R8(buffer + bodyStart + 88);
    GPSEphTemp.Crc = R8(buffer + bodyStart + 96);
    GPSEphTemp.Crs = R8(buffer + bodyStart + 104);
    GPSEphTemp.Cic = R8(buffer + bodyStart + 112);
    GPSEphTemp.Cis = R8(buffer + bodyStart + 120);
    GPSEphTemp.i0 = R8(buffer + bodyStart + 128);
    GPSEphTemp.iDot = R8(buffer + bodyStart + 136);
    GPSEphTemp.OMEGA = R8(buffer + bodyStart + 144);
    GPSEphTemp.OMEGADot = R8(buffer + bodyStart + 152);
    GPSEphTemp.TOC = R8(buffer + bodyStart + 164);
    GPSEphTemp.TGD1 = R8(buffer + bodyStart + 172);
    GPSEphTemp.ClkBias = R8(buffer + bodyStart + 180);
    GPSEphTemp.ClkDrift = R8(buffer + bodyStart + 188);
    GPSEphTemp.ClkDriftRate = R8(buffer + bodyStart + 196);

    GPSEph.push_back(GPSEphTemp);
}

//Read BDS Ephemeris
void HexDecoder::BDSEphReader(const unsigned char* buffer, const int& blockStart)
{
    //Periodically Clear Previous BDS Ephemerides
    if (BDSEph.size() >= 120)
    {
        BDSEph.erase(BDSEph.begin(), BDSEph.begin() + 50);
    }

    BDSEPHREC BDSEphTemp;
    int bodyStart = blockStart + 28;
    BDSEphTemp.PRN = UI4(buffer + bodyStart);
    BDSEphTemp.System = BDS;
    BDSEphTemp.SVHealth = UI4(buffer + bodyStart + 16);
    BDSEphTemp.TGD1 = R8(buffer + bodyStart + 20);
    BDSEphTemp.TGD2 = R8(buffer + bodyStart + 28);
    BDSEphTemp.IODC = UI4(buffer + bodyStart + 36);
    BDSEphTemp.IODE = UI4(buffer + bodyStart + 68);
    BDSEphTemp.TOE = UI4(buffer + bodyStart + 72);
    BDSEphTemp.SqrtA = R8(buffer + bodyStart + 76);
    BDSEphTemp.DeltaN = R8(buffer + bodyStart + 100);
    BDSEphTemp.M0 = R8(buffer + bodyStart + 108);
    BDSEphTemp.e = R8(buffer + bodyStart + 84);
    BDSEphTemp.omega = R8(buffer + bodyStart + 92);
    BDSEphTemp.Cuc = R8(buffer + bodyStart + 148);
    BDSEphTemp.Cus = R8(buffer + bodyStart + 156);
    BDSEphTemp.Crc = R8(buffer + bodyStart + 164);
    BDSEphTemp.Crs = R8(buffer + bodyStart + 172);
    BDSEphTemp.Cic = R8(buffer + bodyStart + 180);
    BDSEphTemp.Cis = R8(buffer + bodyStart + 188);
    BDSEphTemp.i0 = R8(buffer + bodyStart + 132);
    BDSEphTemp.iDot = R8(buffer + bodyStart + 140);
    BDSEphTemp.OMEGA = R8(buffer + bodyStart + 116);
    BDSEphTemp.OMEGADot = R8(buffer + bodyStart + 124);
    BDSEphTemp.TOC = UI4(buffer + bodyStart + 40);
    BDSEphTemp.ClkBias = R8(buffer + bodyStart + 44);
    BDSEphTemp.ClkDrift = R8(buffer + bodyStart + 52);
    BDSEphTemp.ClkDriftRate = R8(buffer + bodyStart + 60);

    BDSEph.push_back(BDSEphTemp);
}

//Read Best Position
void HexDecoder::BestPosReader(const unsigned char* buffer, const int& blockStart)
{
    int bodyStart = blockStart + 28;
    PosRes.Pos[0] = R8(buffer + bodyStart + 8);
    PosRes.Pos[1] = R8(buffer + bodyStart + 16);
    PosRes.Pos[2] = R8(buffer + bodyStart + 24);
    PosRes.Pos[2] += R4(buffer + bodyStart + 32);
    unsigned char SatNumByte = buffer[bodyStart + 65];
    PosRes.SatNum = static_cast<int>(SatNumByte);

    //BLH to XYZ
    Coordinate trans(2, PosRes.Pos[0], PosRes.Pos[1], PosRes.Pos[2]);
    trans.BLHtoXYZ(R_WGS84, F_WGS84);
    PosRes.Pos[0] = trans.X;
    PosRes.Pos[1] = trans.Y;
    PosRes.Pos[2] = trans.Z;
}

//Cyclic Redundancy Check
unsigned int HexDecoder::crc32(const unsigned char* buffer, const int& blockLength, const int& blockStart)
{
    unsigned short len = 28 + blockLength;
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++) 
    {
        crc ^= (unsigned char)buffer[blockStart + i];

        for (j = 0; j < 8; j++) 
        {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

//8-byte decoding
double HexDecoder::R8(const unsigned char* p)
{
    double r;
    memcpy(&r, p, 8);
    return r;
}

//4-byte decoding
float HexDecoder::R4(const unsigned char* p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

unsigned long HexDecoder::UI4(const unsigned char* p)
{
    unsigned long r;
    memcpy(&r, p, 4);
    return r;
}

//2-byte decoding
unsigned short HexDecoder::UI2(const unsigned char* p)
{
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}
