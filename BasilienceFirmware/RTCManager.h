
#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <RTClib.h>

class RTCManager
{
public:

    void begin();

    void update();

    uint8_t getHour();

    uint8_t getMinute();

    uint8_t getSecond();

    void setDateTime(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second);
    bool isConnected();
    uint16_t getYear();

uint8_t getMonth();

uint8_t getDay();


private:

    RTC_DS3231 rtc;
        bool connected = false;
};



#endif