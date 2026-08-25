#ifndef FOGGING_QUEUE_TYPES_H
#define FOGGING_QUEUE_TYPES_H

#include <Arduino.h>

// Capacity chosen for ~12-24h of realistic outage retention even under the
// most frequent automatic fog cycling this firmware runs (COLD_FOG_ON_TIME +
// COLD_FOG_OFF_TIME = 8 minutes/cycle = 2 events/8min): 384 events covers
// ~25.6h at that worst-case rate, ~32h at the normal 10-minute cycle. Not
// copied from NOTIFICATION_QUEUE_CAPACITY - fogging transitions are far more
// frequent than alert episodes, so this queue needs its own sizing.
constexpr uint16_t FOGGING_QUEUE_CAPACITY = 384;

enum class FoggingEventType : uint8_t
{
    OFF = 0,
    ON = 1
};

// Compact source code instead of the firmware's free-text ActuatorStatus
// source String - only the values ActuatorManager/AutomationManager actually
// produce for FOGGER, plus REBOOT_RECOVERY for the synthetic closeout event
// (see FoggingEventQueue::begin()). Mapped back to the exact literal strings
// the backend's isManual derivation already checks (source === "manual" ||
// source === "android") on replay, so that logic is untouched.
enum class FoggingSourceCode : uint8_t
{
    AUTOMATIC = 0,
    MANUAL = 1,
    ANDROID = 2,
    REBOOT_RECOVERY = 3
};

// AutomationManager's activeFogStrategy values ("", "hot", "cold") plus the
// one-time "startup" strategy used for the very first boot fog-on command.
enum class FoggingStrategyCode : uint8_t
{
    NONE = 0,   // activeFogStrategy == "" (normal cycle)
    HOT = 1,
    COLD = 2,
    STARTUP = 3
};

// Only the reasons ActuatorManager/AutomationManager actually attach to a
// FOGGER transition today, plus REBOOT_RECOVERY for the synthetic closeout.
// Deliberately not a free-text field - "compact reason code only if
// genuinely useful" per the task, and these are the only reasons observed.
enum class FoggingReasonCode : uint8_t
{
    NONE = 0,
    MANUAL_STOP = 1,
    SAFETY_TIMEOUT = 2,
    SAFETY_SUSPENDED = 3,
    REBOOT_RECOVERY = 4
};

// Canonical, offline-capable confirmed fogger ON/OFF transition. Fixed-size,
// POD-only (no String/pointers) so the whole durable queue is one NVS blob,
// same discipline as NotificationEvent - but deliberately NOT a copy of it:
// no title/message/SMS fields, no free-text eventId (the id is derived from
// bootId+sequence at use time instead of stored), because fogging events are
// far more frequent and none of that per-event text is needed downstream.
//
// Field order is largest-to-smallest to avoid compiler padding waste.
struct FoggingQueueEvent
{
    uint32_t occurredAtEpoch = 0;  // RTCManager::getEpochTime() at confirm time; meaningless if !timestampValid
    uint16_t sequence = 0;         // Per-boot monotonic counter; combined with bootId makes eventId globally unique
    uint16_t bootId = 0;           // Persisted counter, incremented once per boot (see FoggingEventQueue::begin())
    uint8_t eventType = 0;         // FoggingEventType
    uint8_t sourceCode = 0;        // FoggingSourceCode
    uint8_t strategyCode = 0;      // FoggingStrategyCode
    uint8_t reasonCode = 0;        // FoggingReasonCode
    bool timestampValid = false;   // RTCManager::hasValidTime() at confirm time
    bool inUse = false;
};

// Guards the sizing report (16 bytes x FOGGING_QUEUE_CAPACITY): catches any
// future field addition or compiler padding surprise at compile time rather
// than silently changing the NVS footprint/retention estimate.
static_assert(sizeof(FoggingQueueEvent) == 16, "FoggingQueueEvent size drifted from the reported 16 bytes");

inline const char* foggingSourceCodeString(FoggingSourceCode code)
{
    switch (code)
    {
        case FoggingSourceCode::AUTOMATIC: return "automatic";
        case FoggingSourceCode::MANUAL: return "manual";
        case FoggingSourceCode::ANDROID: return "android";
        case FoggingSourceCode::REBOOT_RECOVERY: return "automatic"; // recovery closeout is not a manual action
    }
    return "automatic";
}

inline const char* foggingStrategyCodeString(FoggingStrategyCode code)
{
    switch (code)
    {
        case FoggingStrategyCode::NONE: return "";
        case FoggingStrategyCode::HOT: return "hot";
        case FoggingStrategyCode::COLD: return "cold";
        case FoggingStrategyCode::STARTUP: return "startup";
    }
    return "";
}

inline const char* foggingReasonCodeString(FoggingReasonCode code)
{
    switch (code)
    {
        case FoggingReasonCode::NONE: return "";
        case FoggingReasonCode::MANUAL_STOP: return "Manual stop";
        case FoggingReasonCode::SAFETY_TIMEOUT: return "Safety limit: manual actuator running too long";
        case FoggingReasonCode::SAFETY_SUSPENDED: return "Suspended by safety condition";
        case FoggingReasonCode::REBOOT_RECOVERY: return "Reboot recovery: device restarted while fogger was on";
    }
    return "";
}

#endif
