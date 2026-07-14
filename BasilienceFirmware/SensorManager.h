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

    // =====================================================
    // Sensor Reading Functions
    // =====================================================

    float measureDistanceCM();

    void readDHT();

    void readWaterTemperature();

    void readWaterLevel();

    void readEC();

    void readPH();
};

#endif