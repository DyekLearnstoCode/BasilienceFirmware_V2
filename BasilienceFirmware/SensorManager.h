#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

    // Water-temperature read scheduling and transient-failure tolerance.
    // physicalSensors.waterTemp only becomes NaN once a scheduled read has
    // failed WATER_TEMP_READ_INTERVAL_MS-spaced attempts consecutively for
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD times; lastValidWaterTemp is kept
    // separately so readEC() can still compensate using it even after that.
    unsigned long lastWaterTempReadTime = 0;
    uint8_t waterTempFailureStreak = 0;
    float lastValidWaterTemp = NAN;

    enum class EcCompensationSource { LIVE, LAST_VALID, FALLBACK_DEFAULT };
    EcCompensationSource lastEcCompensationSource = EcCompensationSource::LIVE;

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
};

#endif
