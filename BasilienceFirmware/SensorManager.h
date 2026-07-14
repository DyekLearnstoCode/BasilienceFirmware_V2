#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class SensorManager
{
public:
    SensorManager();

    void begin();

    void update();

private:
    DHT dht;

    OneWire oneWire;

    DallasTemperature waterSensor;

    // =====================================================
    // EC Sampling Engine
    // =====================================================

    static const int EC_SAMPLE_COUNT = 60;

    int ecSamples[EC_SAMPLE_COUNT];

    int ecSampleIndex;

    bool ecBufferFilled;

    unsigned long lastECSample;

    void updateECSamples();

    int getMedianADC();
    float measureDistanceCM();

    // =====================================================

    void readDHT();

    void readWaterTemperature();

    void readEC();

    void readPH();

    void readWaterLevel();
    float measureDistanceCM();
};

#endif