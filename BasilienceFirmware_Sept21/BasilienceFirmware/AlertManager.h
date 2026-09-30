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
    // sustained failure still does after the same short threshold. Unlike
    // the threshold alerts below, sensorFault is not a Stage 2
    // sample-confirmed/hysteresis alert - it is unchanged from before.
    uint8_t sensorFaultPendingCount = 0;

    // Stage 2 (sensor architecture redesign, incident-style alerts):
    // per-parameter, per-direction confirmation counters applied via
    // setAlertDebounced() below. Each alert now needs TWO counters, not one:
    // *AbnormalPendingCount counts consecutive genuine new observations
    // agreeing the hysteresis-gated threshold is crossed (raises the alert
    // once it reaches SENSOR_TRANSIENT_FAILURE_THRESHOLD);
    // *RecoveryPendingCount counts consecutive genuine new observations
    // agreeing the hysteresis recovery margin has been crossed back (clears
    // the alert on the same threshold). Both sides require genuinely NEW
    // sensor samples (see newObservation()/SensorManager's *SampleVersion
    // getters) - a loop() tick that re-evaluates an unchanged reading
    // advances neither. This makes each alert represent one confirmed
    // abnormal episode with exactly one false->true and one true->false
    // notification edge, instead of clearing (or re-raising) on a single
    // reading right at either threshold.
    uint8_t lowWaterAbnormalPendingCount = 0;
    uint8_t lowWaterRecoveryPendingCount = 0;
    uint8_t criticalLowWaterAbnormalPendingCount = 0;
    uint8_t criticalLowWaterRecoveryPendingCount = 0;
    uint8_t waterLevelLowAbnormalPendingCount = 0;
    uint8_t waterLevelLowRecoveryPendingCount = 0;
    uint8_t waterLevelHighAbnormalPendingCount = 0;
    uint8_t waterLevelHighRecoveryPendingCount = 0;
    uint8_t lowAirTemperatureAbnormalPendingCount = 0;
    uint8_t lowAirTemperatureRecoveryPendingCount = 0;
    uint8_t highTemperatureAbnormalPendingCount = 0;
    uint8_t highTemperatureRecoveryPendingCount = 0;
    uint8_t humidityLowAbnormalPendingCount = 0;
    uint8_t humidityLowRecoveryPendingCount = 0;
    uint8_t humidityHighAbnormalPendingCount = 0;
    uint8_t humidityHighRecoveryPendingCount = 0;
    uint8_t waterTempOutOfRangeAbnormalPendingCount = 0;
    uint8_t waterTempOutOfRangeRecoveryPendingCount = 0;
    uint8_t waterTempLowAbnormalPendingCount = 0;
    uint8_t waterTempLowRecoveryPendingCount = 0;
    uint8_t phLowAbnormalPendingCount = 0;
    uint8_t phLowRecoveryPendingCount = 0;
    uint8_t phHighAbnormalPendingCount = 0;
    uint8_t phHighRecoveryPendingCount = 0;
    uint8_t ecLowAbnormalPendingCount = 0;
    uint8_t ecLowRecoveryPendingCount = 0;
    uint8_t ecHighAbnormalPendingCount = 0;
    uint8_t ecHighRecoveryPendingCount = 0;

    // Stage 2: last SensorManager sample-version processed per underlying
    // sensor (see newObservation() below) - one per sensor, shared across
    // that sensor's low/high alerts, mirroring SensorManager's own "one
    // version counter per sensor" ownership (a DHT read refreshes
    // temperature AND humidity together; an HC-SR04 read refreshes both
    // waterLevel-percentage and waterLevelCm together).
    uint32_t phLastProcessedVersion = 0;
    uint32_t ecLastProcessedVersion = 0;
    uint32_t dhtLastProcessedVersion = 0;
    uint32_t waterTempLastProcessedVersion = 0;
    uint32_t waterLevelLastProcessedVersion = 0;

    void setAlert(const char* name, bool& currentValue, bool nextValue);

    // Shared confirmation logic for every threshold-crossing alert
    // (Stage 2). nextAbnormal already reflects the hysteresis-gated
    // Schmitt-trigger check (aboveWithHysteresis()/belowWithHysteresis()),
    // using currentValue as the "currently active" side. While nextAbnormal
    // is true, only abnormalPendingCount advances (recoveryPendingCount is
    // held at 0); while false, only recoveryPendingCount advances
    // (abnormalPendingCount held at 0) - either counter only advances on a
    // call made with newObservation=true (see newObservation() below), and
    // the alert flips exactly once either counter reaches
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD. pendingCount pairs are per-alert
    // state owned by the caller, since alerts derived from the same sensor
    // (e.g. phLow/phHigh) must debounce independently of each other.
    void setAlertDebounced(const char* name, bool& currentValue, bool nextAbnormal,
                           uint8_t& abnormalPendingCount, uint8_t& recoveryPendingCount,
                           bool newSample);

    // True exactly once per genuinely new SensorManager observation for a
    // given sensor - compares currentVersion (one of SensorManager's
    // get*SampleVersion() accessors, incremented ONLY at that sensor's own
    // accepted-observation point, never merely on elapsed time or a
    // re-evaluated stale/failed/skipped read) against the caller-owned
    // lastProcessedVersion, consuming it (updating lastProcessedVersion) the
    // moment a change is seen - "current version == last processed -> do
    // nothing; current version != last processed -> process once".
    static bool newObservation(uint32_t currentVersion, uint32_t& lastProcessedVersion);

    // Schmitt-trigger threshold checks (Stage 2 hysteresis): while
    // currentlyActive is false, only the raw threshold trips them; once
    // active, they stay active until the value has recovered past
    // threshold +/- hysteresis, not merely back across threshold itself.
    // Returns false outright when !valid (mirrors the plain single-threshold
    // checks these replace - an invalid/unavailable reading can never raise
    // or hold an alert active).
    static bool aboveWithHysteresis(bool currentlyActive, float value,
                                    float threshold, float hysteresis, bool valid);
    static bool belowWithHysteresis(bool currentlyActive, float value,
                                    float threshold, float hysteresis, bool valid);

    void updateLowWaterAlert();

    // Returns whether a fresh DHT sample was available this call
    // (newObservation() gate) - passed to updateHumidityAlert() below since
    // one DHT read refreshes both temperature and humidity together.
    bool updateTemperatureAlert();

    void updateHumidityAlert(bool newSample);

    void updateWaterTemperatureAlert();

    void updatePHAlert();

    void updateECAlert();

    void updateSensorFaultAlert();
};

#endif
