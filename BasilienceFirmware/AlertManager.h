#ifndef ALERT_MANAGER_H
#define ALERT_MANAGER_H

#include <Arduino.h>

class AlertManager
{
public:
    void begin();

    void update();

    bool isDirty() const;

    void markSynced();

private:
    bool alertsDirty = true;

    // Consecutive-invalid-evaluation debounce for sensorFault: raising it
    // requires SENSOR_TRANSIENT_FAILURE_THRESHOLD consecutive invalid ticks,
    // but any single valid tick clears it immediately - so a transient
    // one-tick sensor hiccup never trips a false sensorFault, while a real
    // sustained failure still does after the same short threshold.
    uint8_t sensorFaultPendingCount = 0;

    // Same debounce shape as sensorFault above, applied per threshold alert
    // via setAlertDebounced() below: raising an alert requires
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD consecutive over-the-line
    // evaluations, but clearing is immediate on the first evaluation back on
    // the good side. This stops a single noisy reading (ADC jitter, a
    // momentary sensor hiccup) from flipping an alert on its own and
    // generating a fresh notification for what is really one ongoing
    // condition - without delaying recovery, or delaying an automation
    // trigger that reads the same flag (e.g. lowWater/ecLow/ecHigh).
    uint8_t lowWaterPendingCount = 0;
    uint8_t criticalLowWaterPendingCount = 0;
    uint8_t waterLevelLowPendingCount = 0;
    uint8_t waterLevelHighPendingCount = 0;
    uint8_t lowAirTemperaturePendingCount = 0;
    uint8_t highTemperaturePendingCount = 0;
    uint8_t humidityLowPendingCount = 0;
    uint8_t humidityHighPendingCount = 0;
    uint8_t waterTempOutOfRangePendingCount = 0;
    uint8_t waterTempLowPendingCount = 0;
    uint8_t phLowPendingCount = 0;
    uint8_t phHighPendingCount = 0;
    uint8_t ecLowPendingCount = 0;
    uint8_t ecHighPendingCount = 0;

    void setAlert(const char* name, bool& currentValue, bool nextValue);

    // Shared debounce for every threshold-crossing alert: nextValue must be
    // true for SENSOR_TRANSIENT_FAILURE_THRESHOLD consecutive calls before
    // the alert actually raises; a single false call clears both the alert
    // and the pending count immediately. pendingCount is per-alert state
    // owned by the caller, since alerts derived from the same sensor (e.g.
    // phLow/phHigh) must debounce independently of each other.
    void setAlertDebounced(const char* name, bool& currentValue,
                           bool nextValue, uint8_t& pendingCount);

    void updateLowWaterAlert();

    void updateTemperatureAlert();

    void updateHumidityAlert();

    void updateWaterTemperatureAlert();

    void updatePHAlert();

    void updateECAlert();

    void updateSensorFaultAlert();
};

#endif
