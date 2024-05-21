#ifndef DATE_TIME_H
#define DATE_TIME_H

/**************************************************************************************************************
  DateTime Class

  Purpose: Perform various time system conversions.

  Member Functions:
  - DateTime: Constructor for DateTime class.
  - UTCtoJD: Convert Coordinated Universal Time to Julian Date.
  - JDtoUTC: Convert Julian Date to Coordinated Universal Time.
  - MJDtoGPST: Convert Modified Julian Date to GPS Time.
  - GPSTtoMJD: Convert GPS Time to Modified Julian Date.
  - JDtoMJD: Convert Julian Date to Modified Julian Date.
  - MJDtoJD: Convert Modified Julian Date to Julian Date.
**************************************************************************************************************/

class DateTime 
{
public:
    short year = 0;
    unsigned short month = 0;
    unsigned short day = 0;
    unsigned short hour = 0;
    unsigned short minute = 0;
    double second = 0.0;
    double JD = 0.0;
    double MJD = 0.0;
    unsigned short gpsWeek = 0;
    double gpsSeconds = 0.0;

    DateTime(short year, unsigned short month, unsigned short day, unsigned short hour, unsigned short minute, double second);
    DateTime(double JD);
    DateTime(unsigned short gpsWeek, double gpsSeconds);

    void UTCtoJD();

    void JDtoUTC();

    void MJDtoGPST();

    void GPSTtoMJD();

    void JDtoMJD();

    void MJDtoJD();
};

#endif