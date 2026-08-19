#ifndef HARVEST_SCHEDULE_CACHE_H
#define HARVEST_SCHEDULE_CACHE_H

#include <Arduino.h>
#include <Preferences.h>

// NVS-persisted cache of the active cultivation cycle's harvest due date,
// synced from the /devices/{deviceId}/harvestSchedule RTDB projection while
// Firebase is reachable. NotificationManager reads only through this class
// to generate an offline-capable HARVEST_DUE event - the ESP never fetches
// or reconstructs cycle data from Firestore itself.
class HarvestScheduleCache
{
public:
    void begin();

    // Applies a freshly-read authoritative snapshot. A failed RTDB read must
    // never call this - the caller leaves the last known-good schedule as-is.
    // active=false (no active cycle, or the cycle completed/was deleted)
    // deactivates the cached schedule so a stale cycle can't keep firing
    // HARVEST_DUE after it no longer applies.
    void applySnapshot(const String& cycleId, int cycleNumber, uint32_t nextHarvestAtEpoch, bool active);

    bool isActive() const;
    String getCycleId() const;
    int getCycleNumber() const;
    uint32_t getNextHarvestAtEpoch() const;

private:
    static constexpr const char* NVS_NAMESPACE = "harvestsc";

    Preferences preferences;
    String cycleId;
    int cycleNumber = 0;
    uint32_t nextHarvestAtEpoch = 0;
    bool active = false;

    void loadFromNvs();
    void saveToNvs();
};

#endif
