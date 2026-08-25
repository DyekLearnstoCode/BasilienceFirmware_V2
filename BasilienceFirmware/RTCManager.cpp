#include "RTCManager.h"
#include <Wire.h>
#include "Config.h"
#include "Globals.h"

namespace
{
    // Basilience-specific plausibility floor - not a generic calendar-math
    // limit. A real unit was observed reporting lostPower()==false with a
    // calendar of 2000-00-00, so year alone must be constrained too, not
    // just month/day/hour/minute/second ranges.
    constexpr uint16_t MIN_PLAUSIBLE_YEAR = 2025;
    // Generous defensive ceiling (not a product requirement) - catches a
    // corrupted/garbage year the same way the floor catches an uninitialized
    // one, without ever needing to be updated for ordinary operation.
    constexpr uint16_t MAX_PLAUSIBLE_YEAR = 2100;

    bool isLeapYear(uint16_t year)
    {
        return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    }

    // Validates day-of-month against the ACTUAL month/year, not a blanket
    // 1-31 range - rejects e.g. 2026-02-31 and 2026-04-31, and correctly
    // allows Feb 29 only in a real leap year.
    uint8_t daysInMonth(uint16_t year, uint8_t month)
    {
        static const uint8_t DAYS_IN_MONTH[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        if (month < 1 || month > 12) return 0;
        if (month == 2 && isLeapYear(year)) return 29;
        return DAYS_IN_MONTH[month - 1];
    }

    bool isPlausibleCalendar(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second)
    {
        if (year < MIN_PLAUSIBLE_YEAR || year > MAX_PLAUSIBLE_YEAR) return false;
        if (month < 1 || month > 12) return false;
        if (day < 1 || day > daysInMonth(year, month)) return false;
        if (hour > 23) return false;
        if (minute > 59) return false;
        if (second > 59) return false;
        return true;
    }
}

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

        // One-time boot diagnostic - not repeated in update() (there is no
        // per-loop RTC logging in this firmware). lostPower() reflects the
        // DS3231's own oscillator-stop flag: true means the chip has never
        // been set, or lost backup power since it last was. It proves the
        // absence of a power-loss condition, not that the retained time is
        // actually correct - phrased that way deliberately, not as "proof
        // of a trusted time." A real unit showed lostPower()==false with a
        // 2000-00-00 calendar, which is why hasPlausibleCalendarTime() is
        // logged as its own separate line below rather than folded silently
        // into "valid".
        const bool lostPower = rtc.lostPower();
        Serial.print("[RTC] connected=true lostPower=");
        Serial.println(lostPower ? "true" : "false");

        const bool calendarPlausible = hasPlausibleCalendarTime();
        Serial.print("[RTC] calendarPlausible=");
        Serial.println(calendarPlausible ? "true" : "false");

        const bool valid = hasValidTime();
        Serial.print("[RTC] valid=");
        Serial.println(valid ? "true" : "false");

        syncSource = valid ? SyncSource::RTC_RETAINED : SyncSource::INVALID;

        DateTime now = rtc.now();
        if (valid)
        {
            Serial.print("[RTC] epoch=");
            Serial.println(getEpochTime());

            char localTime[20];
            snprintf(localTime, sizeof(localTime), "%04u-%02u-%02u %02u:%02u:%02u",
                      now.year(), now.month(), now.day(),
                      now.hour(), now.minute(), now.second());
            Serial.print("[RTC] local=");
            Serial.println(localTime);
        }
        else if (!lostPower && !calendarPlausible)
        {
            // The specific case this task exists to fix: chip reports no
            // power-loss condition, but the retained calendar is not
            // usable. Printed once at boot so it is unambiguous from the
            // "lostPower==true" invalid case (also possible, logged as
            // above with no further detail needed).
            char localTime[24];
            snprintf(localTime, sizeof(localTime), "%04u-%02u-%02u %02u:%02u:%02u",
                      now.year(), now.month(), now.day(),
                      now.hour(), now.minute(), now.second());
            Serial.print("[RTC] invalid calendar: ");
            Serial.println(localTime);
        }
    }
    else
    {
        Serial.println(
            "DS3231 NOT FOUND");
        syncSource = SyncSource::INVALID;
    }
}

void RTCManager::update()
{
    // Bounded, backoff-gated NTP recovery. Only ever considered while the
    // RTC is actually invalid - a DS3231 that already has a plausible
    // retained time is never rewritten here (see the resync-policy
    // reasoning in the RTC finalization report: invalid-on-boot recovery
    // only, no periodic drift-correction resync).
    if (!connected) return;
    if (hasValidTime()) return;

    // WiFiManager remains the sole owner of the radio connection - this
    // only reads its already-established state and never calls any
    // WiFi.begin()/reconnect()/disconnect() itself.
    if (!wifiManager.isConnected()) return;

    const unsigned long now = millis();
    if (lastNtpAttemptAt != 0 && now - lastNtpAttemptAt < NTP_RETRY_INTERVAL_MS) return;

    lastNtpAttemptAt = now;
    attemptNetworkTimeSync();
}

void RTCManager::attemptNetworkTimeSync()
{
    Serial.println("[RTC] Network time synchronization requested");

    // GMT offset only, no DST - see the header for why UTC+8/Asia-Manila is
    // this firmware's chosen convention. configTime() starts the SNTP
    // client; getLocalTime() below performs one bounded wait for it to
    // actually lock onto a server rather than blocking indefinitely.
    configTime(TIMEZONE_OFFSET_SECONDS, 0, "pool.ntp.org", "time.google.com");

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, NTP_SYNC_TIMEOUT_MS))
    {
        Serial.println("[RTC] Network time synchronization failed: timeout");
        return;
    }

    Serial.println("[RTC] Network time acquired");

    // Reject an obviously-wrong value rather than adjusting the DS3231 to
    // something that could be worse than simply staying invalid - e.g. the
    // C library's time struct defaulting to an early epoch before SNTP has
    // genuinely locked on.
    const int year = timeinfo.tm_year + 1900;
    if (year < 2025)
    {
        Serial.println("[RTC] Network time synchronization failed: implausible date received");
        return;
    }

    rtc.adjust(DateTime(
        (uint16_t)year,
        (uint8_t)(timeinfo.tm_mon + 1),
        (uint8_t)timeinfo.tm_mday,
        (uint8_t)timeinfo.tm_hour,
        (uint8_t)timeinfo.tm_min,
        (uint8_t)timeinfo.tm_sec));

    // Verify the write actually landed: adjust() clears the DS3231's
    // oscillator-stop flag as part of a successful write, so lostPower()
    // reading true here means the I2C write itself did not take.
    if (rtc.lostPower())
    {
        Serial.println("[RTC] Network time synchronization failed: DS3231 did not accept the adjustment");
        return;
    }

    syncSource = SyncSource::NTP;
    Serial.println("[RTC] DS3231 adjusted successfully");

    DateTime now = rtc.now();
    char localTime[20];
    snprintf(localTime, sizeof(localTime), "%04u-%02u-%02u %02u:%02u:%02u",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());
    Serial.print("[RTC] local=");
    Serial.println(localTime);
}

const char* RTCManager::getSyncSourceName()
{
    // Always re-derived from hasValidTime() (the single source of truth for
    // both the lostPower and calendar-plausibility checks) rather than
    // reproducing either check here - so a DS3231 that reports lost power,
    // or one whose calendar has become implausible, is never reported as
    // "NTP"/"RTC_RETAINED" after the fact.
    if (!hasValidTime()) return "INVALID";
    return syncSource == SyncSource::NTP ? "NTP" : "RTC_RETAINED";
}

bool RTCManager::isConnected()
{
    return connected;
}

bool RTCManager::hasPlausibleCalendarTime()
{
    if (!connected) return false;
    DateTime now = rtc.now();
    return isPlausibleCalendar(now.year(), now.month(), now.day(),
                                now.hour(), now.minute(), now.second());
}

bool RTCManager::hasValidTime()
{
    if (!connected) return false;
    if (rtc.lostPower()) return false;
    return hasPlausibleCalendarTime();
}

uint32_t RTCManager::getEpochTime()
{
    if (!hasValidTime()) return 0;

    DateTime now = rtc.now();

    // CONFIRMED BUG FIX (RTC epoch semantics audit): verified directly
    // against RTClib.cpp's actual DateTime::unixtime() implementation -
    // it applies no timezone conversion whatsoever, it just packs the
    // stored Y/M/D/H/M/S fields as though they were UTC. Since those
    // fields are this firmware's Asia/Manila LOCAL time (UTC+08:00, no
    // DST), the raw value is 8 hours AHEAD of the true UTC instant. This
    // is the single point where that correction is applied - every other
    // getter in this class deliberately returns the unconverted local
    // fields, since local-hour consumers (grow-light scheduling) need
    // exactly those.
    const uint32_t localCalendarEpoch = now.unixtime();

    // Guards unsigned underflow. Cannot actually occur in practice - RTClib
    // enforces a year-2000 floor, so unixtime() can never be smaller than
    // ~31 years worth of seconds, let alone smaller than 8 hours - but
    // checked explicitly rather than relying on that indirectly.
    if (localCalendarEpoch < (uint32_t)TIMEZONE_OFFSET_SECONDS)
    {
        return 0;
    }

    return localCalendarEpoch - (uint32_t)TIMEZONE_OFFSET_SECONDS;
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

