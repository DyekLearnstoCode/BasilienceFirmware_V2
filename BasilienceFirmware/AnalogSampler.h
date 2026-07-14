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

private:
    uint8_t pin;

    uint8_t sampleCount;

    unsigned long sampleInterval;

    ReadMode mode;

    unsigned long lastSampleTime;

    int samples[MAX_SAMPLES];

    uint8_t sampleIndex;

    bool bufferFilled;
};

#endif