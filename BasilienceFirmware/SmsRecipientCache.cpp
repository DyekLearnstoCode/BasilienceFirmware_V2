#include "SmsRecipientCache.h"
#include "GsmManager.h"

void SmsRecipientCache::begin()
{
    loadFromNvs();
    Serial.print("[SMS] Recipient cache loaded from NVS: ");
    Serial.println(count);
}

void SmsRecipientCache::applySnapshot(const String canonicalPhones[], uint8_t incomingCount)
{
    String deduped[MAX_SMS_RECIPIENTS];
    uint8_t dedupedCount = 0;

    for (uint8_t i = 0; i < incomingCount; i++)
    {
        String phone = canonicalPhones[i];
        if (!GsmManager::isValidCanonicalPhilippineMobile(phone))
        {
            continue; // invalid/empty phone excluded, does not block the rest
        }

        bool duplicate = false;
        for (uint8_t j = 0; j < dedupedCount; j++)
        {
            if (deduped[j] == phone) { duplicate = true; break; }
        }
        if (duplicate) continue;

        if (dedupedCount >= MAX_SMS_RECIPIENTS)
        {
            Serial.print("[SMS] Recipient snapshot exceeds capacity (");
            Serial.print(MAX_SMS_RECIPIENTS);
            Serial.println("); extra recipients dropped rather than overwriting arbitrary slots");
            break;
        }
        deduped[dedupedCount++] = phone;
    }

    bool changed = dedupedCount != count;
    if (!changed)
    {
        for (uint8_t i = 0; i < count; i++)
        {
            if (phones[i] != deduped[i]) { changed = true; break; }
        }
    }

    if (!changed)
    {
        Serial.println("[SMS] Recipient cache unchanged");
        return;
    }

    for (uint8_t i = 0; i < dedupedCount; i++) phones[i] = deduped[i];
    count = dedupedCount;
    saveToNvs();

    Serial.print("[SMS] Recipients synced: ");
    Serial.println(count);
}

uint8_t SmsRecipientCache::recipientCount() const
{
    return count;
}

String SmsRecipientCache::recipientAt(uint8_t index) const
{
    if (index >= count) return "";
    return phones[index];
}

void SmsRecipientCache::loadFromNvs()
{
    preferences.begin(NVS_NAMESPACE, true);
    uint8_t storedCount = preferences.getUChar("count", 0);
    if (storedCount > MAX_SMS_RECIPIENTS) storedCount = MAX_SMS_RECIPIENTS;
    for (uint8_t i = 0; i < storedCount; i++)
    {
        char key[6];
        snprintf(key, sizeof(key), "p%u", i);
        phones[i] = preferences.getString(key, "");
    }
    preferences.end();
    count = storedCount;
}

void SmsRecipientCache::saveToNvs()
{
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putUChar("count", count);
    for (uint8_t i = 0; i < count; i++)
    {
        char key[6];
        snprintf(key, sizeof(key), "p%u", i);
        preferences.putString(key, phones[i]);
    }
    preferences.end();
}
