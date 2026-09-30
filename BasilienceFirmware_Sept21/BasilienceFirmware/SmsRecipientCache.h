#ifndef SMS_RECIPIENT_CACHE_H
#define SMS_RECIPIENT_CACHE_H

#include <Arduino.h>
#include <Preferences.h>
#include "NotificationTypes.h"

// Bounded, NVS-persisted cache of SMS-eligible recipient phone numbers,
// synced from the /devices/{deviceId}/smsRecipients RTDB projection while
// Firebase is reachable. SMS delivery (NotificationManager) reads recipients
// only through this class - it never talks to Firebase or NVS directly.
//
// Capacity: MAX_SMS_RECIPIENTS (10) - see NotificationTypes.h. Sized for one
// Admin plus a realistic Personnel roster on a single small-farm device, not
// an arbitrary example number.
class SmsRecipientCache
{
public:
    void begin();

    // Applies a freshly-read authoritative snapshot: each entry is
    // structurally validated (GsmManager::isValidCanonicalPhilippineMobile,
    // no carrier-prefix table) and deduplicated by canonical number. Only
    // persists to NVS if the resulting set actually differs from what's
    // cached, to avoid unnecessary flash wear. A failed RTDB read must never
    // call this - the caller is responsible for leaving the last known-good
    // cache untouched on failure. An authoritative EMPTY snapshot (0 entries
    // because no one is currently assigned/eligible) is a valid input and
    // clears the cache.
    void applySnapshot(const String canonicalPhones[], uint8_t count);

    uint8_t recipientCount() const;
    // Canonical phone at [0, recipientCount()), or "" if out of range.
    String recipientAt(uint8_t index) const;

private:
    static constexpr const char* NVS_NAMESPACE = "smsrecip";

    Preferences preferences;
    String phones[MAX_SMS_RECIPIENTS];
    uint8_t count = 0;

    void loadFromNvs();
    void saveToNvs();
};

#endif
