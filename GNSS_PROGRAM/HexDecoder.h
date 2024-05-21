#ifndef HEX_DECODER_H
#define HEX_DECODER_H

#include "data.h"
#include "Coordinate.h"

/**************************************************************************************************************
  HexDecoder Class

  Purpose: Decode hexadecimal data and extract information for various navigation and positioning data types.

  Member Functions:
  - HexDecoder: Constructor for HexDecoder class.
  - BufferReader: Decode hexadecimal buffer and extract information for navigation and positioning data.
  - HeadReader: Extract header information from the decoded data.
  - RangeReader: Extract satellite observation data from the decoded data.
  - GPSEphReader: Extract GPS ephemeris data from the decoded data.
  - BDSEphReader: Extract BDS ephemeris data from the decoded data.
  - BestPosReader: Extract best position data from the decoded data.
  - crc32: Perform cyclic redundancy check on the decoded data.
  - R8: Decode 8-byte data from the hexadecimal buffer.
  - R4: Decode 4-byte data from the hexadecimal buffer.
  - UI4: Decode unsigned 4-byte data from the hexadecimal buffer.
  - UI2: Decode unsigned 2-byte data from the hexadecimal buffer.
**************************************************************************************************************/

class HexDecoder 
{
public:
    HexDecoder();

    int BufferReader(const int& bufferLength, const unsigned char* buffer, int& blockEnd, int& blockStart, bool& isIncompleteBlock);

    EpoRange range;
    std::vector<GPSEPHREC> GPSEph;
    std::vector<BDSEPHREC> BDSEph;
    POSRES PosRes;

private:
    unsigned short MessageID;
    unsigned short Week;
    double GPSsecond;
    unsigned long obs;

    void HeadReader(const unsigned char* buffer, const int& blockStart);

    void RangeReader(const unsigned char* buffer, const int& blockStart);

    void GPSEphReader(const unsigned char* buffer, const int& blockStart);

    void BDSEphReader(const unsigned char* buffer, const int& blockStart);

    void BestPosReader(const unsigned char* buffer, const int& blockStart);

    unsigned int crc32(const unsigned char* buffer, const int& blockLength, const int& blockStart);

    double R8(const unsigned char* p);

    float R4(const unsigned char* p);

    unsigned long UI4(const unsigned char* p);

    unsigned short UI2(const unsigned char* p);
};

#endif 
