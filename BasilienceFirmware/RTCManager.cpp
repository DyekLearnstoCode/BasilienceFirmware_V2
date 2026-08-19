#include "RTCManager.h"
#include <Wire.h>
#include "Config.h"

void RTCManager::begin()
{
    Wire.begin(
        RTC_SDA_PIN,
        RTC_SCL_PIN);

    connected = rtc.begin();

    if(connected)
    {
        Serial.println(
            "DS3231 FOUND");
    }
    else
    {
        Serial.println(
            "DS3231 NOT FOUND");
    }
}

void RTCManager::update()
{
}

bool RTCManager::isConnected()
{
    return connected;
}

bool RTCManager::hasValidTime()
{
    if (!connected) return false;
    return !rtc.lostPower();
}

uint32_t RTCManager::getEpochTime()
{
    if (!hasValidTime()) return 0;

    DateTime now = rtc.now();
    return now.unixtime();
}

uint8_t RTCManager::getHour()
{
    if(!connected)
        return 0;

    DateTime now = rtc.now();

    return now.hour();
}

uint8_t RTCManager::getMinute()
{
    if(!connected)
        return 0;

    DateTime now = rtc.now();

    return now.minute();
}


uint8_t RTCManager::getSecond()
    {
        if(!connected)
            return 0;

        DateTime now = rtc.now();

        return now.second();
    }

    uint16_t RTCManager::getYear()
{
    if (!connected)
        return 0;

    DateTime now = rtc.now();

    return now.year();
}

uint8_t RTCManager::getMonth()
{
    if (!connected)
        return 0;

    DateTime now = rtc.now();

    return now.month();
}

uint8_t RTCManager::getDay()
{
    if (!connected)
        return 0;

    DateTime now = rtc.now();

    return now.day();
}

void RTCManager::setDateTime(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second)
{
    if(!connected)
        return;

    rtc.adjust(
        DateTime(
            year,
            month,
            day,
            hour,
            minute,
            second));
}

