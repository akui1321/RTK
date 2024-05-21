#include "DateTime.h"

//Overload Constructor
DateTime::DateTime(short year, unsigned short month, unsigned short day, unsigned short hour, unsigned short minute, double second)
    : year(year), month(month), day(day), hour(hour), minute(minute), second(second)
{
}

DateTime::DateTime(double JD) 
    :JD(JD)
{
}

DateTime::DateTime(unsigned short gpsWeek, double gpsSeconds)
    :gpsWeek(gpsWeek), gpsSeconds(gpsSeconds)
{
}

//Convert UTC to Julian Date
void DateTime::UTCtoJD() 
{
    int y, m;
    if (month <= 2)
    {
        y = year - 1;
        m = month + 12;
    }
    else
    {
        y = year;
        m = month;
    }
    JD = static_cast<int>(365.25 * y) + static_cast<int>(30.6001 * (m + 1)) + day + (hour + minute / 60.0 + second / 3600.0) / 24.0 + 1720981.5;
}

//Convert Julian Date to UTC
void DateTime::JDtoUTC()
{
    int a = static_cast<int>(JD + 0.5);
    int b = a + 1537;
    int c = static_cast<int>((b - 122.1) / 365.25);
    int d = static_cast<int>(365.25 * c);
    int e = static_cast<int>((b - d) / 30.6001);
    day = b - d - static_cast<int>(30.6001 * e) + (JD + 0.5) - static_cast<int>(JD + 0.5);
    month = e - 1 - 12 * static_cast<int>(e / 14.0);
    year = c - 4715 - static_cast<int>((7 + month) / 10.0);
}

//Convert MJD to GPST
void DateTime::MJDtoGPST()
{
    gpsWeek = static_cast<int>((MJD - 44244) / 7.0);
    gpsSeconds = (MJD - 44244 - gpsWeek * 7) * 86400;
}

//Convert GPST to MJD
void DateTime::GPSTtoMJD()
{
    MJD = 44244 + gpsWeek * 7 + gpsSeconds / 86400.0;
}

//Convert JD to MJD
void DateTime::JDtoMJD()
{
    MJD = JD - 2400000.5;
}

//Convert MJD to JD
void DateTime::MJDtoJD()
{
    JD = MJD + 2400000.5;
}
