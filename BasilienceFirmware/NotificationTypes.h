#ifndef NOTIFICATION_TYPES_H
#define NOTIFICATION_TYPES_H

#include <Arduino.h>

// Bounded capacities - see SmsRecipientCache/NotificationManager reports for
// rationale. Chosen from domain constraints (one Admin + a handful of
// Personnel per device; a modest backlog of offline events), not copied
// blindly from an example.
constexpr uint8_t MAX_SMS_RECIPIENTS = 10;
constexpr uint8_t NOTIFICATION_QUEUE_CAPACITY = 20;

enum class NotificationEventType : uint8_t
{
    LOW_WATER,
    HIGH_WATER_TEMP,
    HIGH_AIR_TEMP,
    SENSOR_FAULT,
    DEVICE_UNREACHABLE,
    HARVEST_DUE
};

// Prefixed (SEV_*) because Arduino.h's esp32-hal-gpio.h #defines bare LOW/HIGH
// as GPIO level macros (0x0/0x1), which textually collide with unprefixed
// enumerator names anywhere Arduino.h is also included.
enum class NotificationSeverity : uint8_t
{
    SEV_LOW,
    SEV_MEDIUM,
    SEV_HIGH,
    SEV_CRITICAL
};

enum class SmsDeliveryStatus : uint8_t
{
    PENDING,
    IN_PROGRESS,
    PARTIAL,
    DELIVERED,
    FAILED,
    DEFERRED
};

enum class CloudReplayStatus : uint8_t
{
    NOT_QUEUED,
    PENDING,
    IN_PROGRESS,
    ACKED
};

// Per-recipient SMS attempt outcome, indexed the same way as the recipient
// cache slot it was attempted against.
enum class RecipientSmsState : uint8_t
{
    NOT_ATTEMPTED,
    SENT,
    FAILED_NO_RETRY,
    FAILED_RETRY_EXHAUSTED
};

// Canonical, offline-capable firmware notification event. Fixed-size and
// POD-only (no String/pointers) so the whole durable queue can be persisted
// as one NVS blob and safely memcpy'd/reloaded across reboot.
//
// The SAME title/message here is used both for the SMS body and, on cloud
// replay, for the Firestore history record - wording is authored once at
// enqueue time and never regenerated.
struct NotificationEvent
{
    bool inUse = false;

    char eventId[40] = {0};
    NotificationEventType type = NotificationEventType::LOW_WATER;
    NotificationSeverity severity = NotificationSeverity::SEV_MEDIUM;
    char title[48] = {0};
    char message[160] = {0};

    // Unix epoch seconds from RTCManager::getEpochTime(), captured once at
    // enqueue time. timestampValid is false if the RTC had no trustworthy
    // time at that moment - occurredAtEpoch must not be treated as
    // authoritative in that case (see RTCManager::hasValidTime()).
    uint32_t occurredAtEpoch = 0;
    bool timestampValid = false;

    bool smsEligible = false;
    SmsDeliveryStatus smsStatus = SmsDeliveryStatus::PENDING;
    uint8_t recipientState[MAX_SMS_RECIPIENTS] = {0};      // RecipientSmsState values
    uint8_t recipientRetryCount[MAX_SMS_RECIPIENTS] = {0};

    bool queuedForCloud = false;
    CloudReplayStatus cloudStatus = CloudReplayStatus::NOT_QUEUED;

    // millis() at enqueue time - a same-boot-only fallback for giving up on
    // SMS delivery when occurredAtEpoch/timestampValid can't be trusted (no
    // RTC hardware present). Meaningless across a reboot (millis() resets to
    // 0), which is fine: a slot that was still waiting when the device
    // rebooted gives up on its very next tick post-reboot rather than
    // silently holding a queue slot forever - see NotificationManager.cpp's
    // reapSettledSlots()/startSmsFanOutIfIdle() for how this is used.
    unsigned long enqueuedAtMillis = 0;
};

inline const char* notificationEventTypeName(NotificationEventType type)
{
    switch (type)
    {
        case NotificationEventType::LOW_WATER: return "LOW_WATER";
        case NotificationEventType::HIGH_WATER_TEMP: return "HIGH_WATER_TEMP";
        case NotificationEventType::HIGH_AIR_TEMP: return "HIGH_AIR_TEMP";
        case NotificationEventType::SENSOR_FAULT: return "SENSOR_FAULT";
        case NotificationEventType::DEVICE_UNREACHABLE: return "DEVICE_UNREACHABLE";
        case NotificationEventType::HARVEST_DUE: return "HARVEST_DUE";
    }
    return "EVENT";
}

inline const char* notificationSeverityName(NotificationSeverity severity)
{
    switch (severity)
    {
        case NotificationSeverity::SEV_LOW: return "LOW";
        case NotificationSeverity::SEV_MEDIUM: return "MEDIUM";
        case NotificationSeverity::SEV_HIGH: return "HIGH";
        case NotificationSeverity::SEV_CRITICAL: return "CRITICAL";
    }
    return "MEDIUM";
}

#endif
