
#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <RTClib.h>

// ============================================================================
// Time contract (audited against the actual installed RTClib source, not
// assumed - DateTime::unixtime() applies zero timezone conversion; it just
// mechanically packs whatever Y/M/D/H/M/S is stored as if those fields were
// UTC):
//
//   getHour()/getMinute()/getSecond()/getYear()/getMonth()/getDay()
//     = the DS3231's raw stored calendar fields, which this firmware's
//       chosen convention defines as Asia/Manila LOCAL civil time
//       (UTC+08:00, no DST). Grow-light scheduling and any other
//       local-wall-clock consumer must use these.
//
//   getEpochTime()
//     = the TRUE absolute UTC Unix epoch for that same instant - i.e. the
//       DS3231's local reading corrected by the fixed -8h offset. Every
//       consumer that stores or transmits a timestamp meant to represent
//       "when this actually happened" (fogging/notification event history,
//       harvest-due comparison against a Firestore-sourced UTC epoch,
//       /status/rtc's epochUtc field) must use this, never unixtime() or
//       the local field getters, or the stored time will read 8 hours in
//       the future relative to the real instant.
// ============================================================================
class RTCManager
{
public:

    void begin();

    void update();

    // Local (Asia/Manila) calendar fields - see the contract above.
    uint8_t getHour();

    uint8_t getMinute();

    uint8_t getSecond();

    // Sets the DS3231 to the given LOCAL (Asia/Manila) calendar fields -
    // matches what getHour()/getYear()/etc. read back, and what
    // RTCManager::attemptNetworkTimeSync() writes after NTP.
    void setDateTime(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second);
    bool isConnected();

    // CONFIRMED LIVE-HARDWARE BUG FIX: hasValidTime() previously relied only
    // on rtc.lostPower()==false, which does NOT prove the stored calendar is
    // meaningful - a DS3231 that has held continuous backup power since
    // manufacture/last assembly but was never actually programmed via
    // rtc.adjust() can report lostPower()==false while its date/time
    // registers still hold their power-on-reset/uninitialized values (BCD
    // registers defaulting to 0 decode as month=0, day=0, year=2000 in
    // RTClib - OSF only flags a detected oscillator stop, not "never set").
    // That is exactly what a real unit showed: connected=true,
    // lostPower=false, calendar=2000-00-00. hasValidTime() now additionally
    // requires the stored calendar to be semantically plausible - see
    // hasPlausibleCalendarTime() below. No caller needs to reproduce these
    // checks itself; they all already just call hasValidTime().
    bool hasValidTime();

    // The calendar-plausibility half of hasValidTime(), exposed separately
    // only so /status/rtc and boot diagnostics can report *which* condition
    // failed (lostPower vs. an implausible calendar) - never call this
    // instead of hasValidTime() for control-flow decisions elsewhere.
    // Requires: year in [2025, 2100] (a Basilience-specific plausibility
    // floor, not a generic calendar-math limit - rejects a chip's
    // uninitialized/reset default the same way it would reject any other
    // implausibly old value), month 1-12, day valid for that specific
    // month/year (leap years handled), hour 0-23, minute/second 0-59.
    bool hasPlausibleCalendarTime();

    // TRUE ABSOLUTE UTC Unix epoch seconds - see the class-level contract
    // above. Only meaningful when hasValidTime() is true; 0 otherwise.
    // Callers needing a timestamp for anything that leaves this device
    // (event history, schedule comparisons against cloud-sourced UTC
    // epochs) must use this, never assemble one from the local getters
    // below or call DateTime::unixtime() directly.
    uint32_t getEpochTime();

    // Local (Asia/Manila) calendar fields - see the contract above.
    uint16_t getYear();

uint8_t getMonth();

uint8_t getDay();

    // Diagnostic-only: which of "RTC_RETAINED" / "NTP" / "INVALID"
    // currently explains hasValidTime()'s answer. Never used for control
    // flow - purely so /status/rtc can report where the DS3231's time
    // actually came from this boot.
    const char* getSyncSourceName();


private:

    RTC_DS3231 rtc;
        bool connected = false;

    // ---- Bounded NTP recovery (see RTCManager.cpp for full rationale) ----
    // Only ever runs when hasValidTime() is false and Wi-Fi is already
    // connected (WiFiManager remains the sole radio owner - this class only
    // reads its connection state, never calls any WiFi.* connection API).
    enum class SyncSource : uint8_t { INVALID, RTC_RETAINED, NTP };
    SyncSource syncSource = SyncSource::INVALID;
    unsigned long lastNtpAttemptAt = 0;

    void attemptNetworkTimeSync();

    // Bounded wait for getLocalTime() - never blocks indefinitely.
    static constexpr uint32_t NTP_SYNC_TIMEOUT_MS = 8000UL;
    // Backoff between retries after a failed attempt, so a persistently
    // unreachable NTP server cannot turn this into a per-loop cost.
    static constexpr unsigned long NTP_RETRY_INTERVAL_MS = 5UL * 60UL * 1000UL;
    // Asia/Manila, UTC+08:00, no DST (the Philippines does not observe
    // DST). This is Basilience's chosen RTC convention - see
    // RTCManager.cpp for why: grow-light scheduling and every other
    // RTC-derived field this firmware produces are already consumed as
    // local wall-clock hours, not UTC.
    static constexpr long TIMEZONE_OFFSET_SECONDS = 8L * 3600L;
};



#endif