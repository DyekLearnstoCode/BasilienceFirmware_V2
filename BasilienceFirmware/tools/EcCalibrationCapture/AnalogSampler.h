// Copied from BasilienceFirmware/AnalogSampler.h so this calibration-check
// sketch can build standalone (Arduino IDE only compiles files inside the
// sketch folder). Keep this in sync with the original if it ever changes.
#ifndef ANALOG_SAMPLER_H
#define ANALOG_SAMPLER_H

#include <Arduino.h>

class AnalogSampler
{
public:
    enum ReadMode
    {
        RAW_ADC,
        MILLIVOLTS
    };

    static const uint8_t MAX_SAMPLES = 60;

    AnalogSampler(
        uint8_t pin,
        uint8_t sampleCount,
        unsigned long sampleInterval,
        ReadMode mode);

    void begin();

    void update();

    bool ready() const;

    int median() const;

    float average() const;

    int minValue() const;

    int maxValue() const;

    int rawMedian() const;

private:
    uint8_t pin;

    uint8_t sampleCount;

    unsigned long sampleInterval;

    ReadMode mode;

    unsigned long lastSampleTime;

    int samples[MAX_SAMPLES];

    int rawSamples[MAX_SAMPLES];

    uint8_t sampleIndex;

    bool bufferFilled;
};

#endif
