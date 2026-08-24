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

    void setAlert(const char* name, bool& currentValue, bool nextValue);

    void updateLowWaterAlert();

    void updateTemperatureAlert();

    void updateHumidityAlert();

    void updateWaterTemperatureAlert();

    void updatePHAlert();

    void updateECAlert();

    void updateSensorFaultAlert();
};

#endif
