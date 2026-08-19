#include "HarvestScheduleCache.h"

void HarvestScheduleCache::begin()
{
    loadFromNvs();
    Serial.print("[HARVEST] Schedule loaded from NVS: active=");
    Serial.println(active ? "true" : "false");
}

void HarvestScheduleCache::applySnapshot(const String& newCycleId, int newCycleNumber,
                                          uint32_t newNextHarvestAtEpoch, bool newActive)
{
    bool changed = (newActive != active) || (newCycleId != cycleId) ||
                   (newCycleNumber != cycleNumber) || (newNextHarvestAtEpoch != nextHarvestAtEpoch);
    if (!changed)
    {
        Serial.println("[HARVEST] Schedule unchanged");
        return;
    }

    cycleId = newCycleId;
    cycleNumber = newCycleNumber;
    nextHarvestAtEpoch = newNextHarvestAtEpoch;
    active = newActive;
    saveToNvs();

    Serial.print("[HARVEST] Schedule synced: cycleId=");
    Serial.print(cycleId);
    Serial.print(" active=");
    Serial.println(active ? "true" : "false");
}

bool HarvestScheduleCache::isActive() const { return active; }
String HarvestScheduleCache::getCycleId() const { return cycleId; }
int HarvestScheduleCache::getCycleNumber() const { return cycleNumber; }
uint32_t HarvestScheduleCache::getNextHarvestAtEpoch() const { return nextHarvestAtEpoch; }

void HarvestScheduleCache::loadFromNvs()
{
    preferences.begin(NVS_NAMESPACE, true);
    active = preferences.getBool("active", false);
    cycleId = preferences.getString("cycleId", "");
    cycleNumber = preferences.getInt("cycleNum", 0);
    nextHarvestAtEpoch = preferences.getUInt("nextAt", 0);
    preferences.end();
}

void HarvestScheduleCache::saveToNvs()
{
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putBool("active", active);
    preferences.putString("cycleId", cycleId);
    preferences.putInt("cycleNum", cycleNumber);
    preferences.putUInt("nextAt", nextHarvestAtEpoch);
    preferences.end();
}
