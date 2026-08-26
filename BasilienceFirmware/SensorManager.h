#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

#include "AnalogSampler.h"

class SensorManager
{
public:
    SensorManager();

    void begin();

    void update();

private:
    // =====================================================
    // Hardware
    // =====================================================

    DHT dht;

    OneWire oneWire;

    DallasTemperature waterSensor;

    AnalogSampler ecSampler;

    AnalogSampler phSampler;

    bool sensorSourceReported = false;
    bool lastReportedMockSource = false;
    bool sensorSourceWaitingLogged = false;
    bool mockBootWaitHeldLogged = false;

    // Sensor-source persistence. The effective source (mock vs. physical) is
    // decided locally at boot from NVS so a cold boot with no Wi-Fi/Firebase
    // still reaches a definite source and local automation can run. Firebase
    // remains authoritative once reachable and reconciles this value.
    static constexpr const char* SOURCE_NVS_NAMESPACE = "sensorsrc";
    static constexpr const char* SOURCE_NVS_KEY = "mockEnabled";
    Preferences sourcePreferences;

    // Boot-restored mock source waiting for its first payload of this session.
    // Armed only by resolveLocalSensorSource(); a mock session enabled from
    // the app after boot never arms it.
    bool mockBootWaitingForPayload = false;
    unsigned long mockBootWaitStartedAt = 0;

    void resolveLocalSensorSource();
    void updateMockBootWait();

    // Water-temperature read scheduling and transient-failure tolerance.
    // physicalSensors.waterTemp only becomes NaN once a scheduled read has
    // failed WATER_TEMP_READ_INTERVAL_MS-spaced attempts consecutively for
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD times; lastValidWaterTemp is kept
    // separately so readEC() can still compensate using it even after that.
    unsigned long lastWaterTempReadTime = 0;
    uint8_t waterTempFailureStreak = 0;
    float lastValidWaterTemp = NAN;

    // DS18B20 enumeration state. 0 devices at boot is re-checked on the same
    // throttled WATER_TEMP_READ_INTERVAL_MS cadence readWaterTemperature()
    // already uses - no separate timer - so a probe that wasn't settled yet
    // at begin() is picked up automatically once it starts responding.
    // waterSensorAddress is cached once enumeration succeeds so normal reads
    // use DallasTemperature::getTempC(address) instead of re-walking the
    // OneWire bus search on every getTempCByIndex(0) call.
    uint8_t waterSensorDeviceCount = 0;
    DeviceAddress waterSensorAddress = {0};
    bool waterSensorAddressValid = false;

    enum class EcCompensationSource { LIVE, LAST_VALID, FALLBACK_DEFAULT };
    EcCompensationSource lastEcCompensationSource = EcCompensationSource::LIVE;

    // HC-SR04 read scheduling and transient-failure tolerance, mirroring the
    // water-temperature pattern above. physicalSensors.waterLevel/
    // waterLevelDistanceCm only become NaN once a scheduled read has failed
    // WATER_LEVEL_READ_INTERVAL_MS-spaced attempts consecutively for
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD times.
    unsigned long lastWaterLevelReadTime = 0;
    uint8_t waterLevelFailureStreak = 0;

    // =====================================================
    // Sensor Reading Functions
    // =====================================================

    float measureDistanceCM();

    void readDHT();

    void readWaterTemperature();

    void readWaterLevel();

    void readEC();

    void readPH();

    void applyEffectiveSensors();

public:
    // Called by FirebaseManager when the authoritative remote setting is read,
    // so the next offline boot starts from the same source. Writes only on an
    // actual change.
    void persistSensorSource(bool mockEnabled);

    // A complete, validated mock payload was parsed during THIS session, so a
    // boot-restored mock source is confirmed live and stops waiting.
    void notifyMockPayloadReceived();

    // The cloud explicitly turned mock mode off; any boot wait is moot.
    void cancelMockBootWait();

    // True only for a boot-restored mock session that hasn't received its
    // first fresh payload yet (armed by resolveLocalSensorSource(), cleared
    // by notifyMockPayloadReceived()/cancelMockBootWait()/the boot-wait
    // timeout). Lets other modules (AlertManager) recognize this as an
    // intentional, transient initialization state rather than a real fault -
    // does not itself change what applyEffectiveSensors() publishes.
    bool isMockBootWaiting() const { return mockBootWaitingForPayload; }
};

#endif
