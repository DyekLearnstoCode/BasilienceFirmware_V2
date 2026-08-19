
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

    // True only when the DS3231 is present AND has never lost power since it
    // was last set (rtc.lostPower(), never previously checked anywhere in
    // this firmware) - i.e. its clock is trustworthy, not just "responding."
    // Additive validity check only; does not alter begin()/update() or any
    // existing getter's behavior.
    bool hasValidTime();

    // Unix epoch seconds, only when hasValidTime() is true; 0 otherwise. Callers
    // needing a wall-clock timestamp (e.g. for an event that must survive
    // reboot/offline queueing) should use this instead of assembling one from
    // the individual field getters below, and must check hasValidTime() first
    // rather than trusting a non-zero-looking but actually-invalid time.
    uint32_t getEpochTime();

    uint16_t getYear();

uint8_t getMonth();

uint8_t getDay();


private:

    RTC_DS3231 rtc;
        bool connected = false;
};



#endif