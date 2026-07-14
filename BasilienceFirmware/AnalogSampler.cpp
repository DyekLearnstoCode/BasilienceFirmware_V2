#include "AnalogSampler.h"

#include <cstring>

AnalogSampler::AnalogSampler(
    uint8_t pin,
    uint8_t sampleCount,
    unsigned long sampleInterval,
    ReadMode mode)
{
    this->pin = pin;

    this->mode = mode;

    if (sampleCount > MAX_SAMPLES)
        sampleCount = MAX_SAMPLES;

    this->sampleCount = sampleCount;

    this->sampleInterval = sampleInterval;

    lastSampleTime = 0;

    sampleIndex = 0;

    bufferFilled = false;
}

void AnalogSampler::begin()
{
    memset(
        samples,
        0,
        sizeof(samples));
}

void AnalogSampler::update()
{
    if (millis() - lastSampleTime < sampleInterval)
        return;

    lastSampleTime = millis();

    switch (mode)
    {
    case RAW_ADC:

        samples[sampleIndex] =
            analogRead(pin);

        break;

    case MILLIVOLTS:

        samples[sampleIndex] =
            analogReadMilliVolts(pin);

        break;
    }

    sampleIndex++;

    if (sampleIndex >= sampleCount)
    {
        sampleIndex = 0;

        bufferFilled = true;
    }
}

bool AnalogSampler::ready() const
{
    return bufferFilled;
}

int AnalogSampler::median() const
{
    if (!bufferFilled)
        return 0;

    int sorted[MAX_SAMPLES];

    memcpy(
        sorted,
        samples,
        sampleCount * sizeof(int));

    for (int i = 0; i < sampleCount - 1; i++)
    {
        for (int j = i + 1; j < sampleCount; j++)
        {
            if (sorted[j] < sorted[i])
            {
                int temp = sorted[i];

                sorted[i] = sorted[j];

                sorted[j] = temp;
            }
        }
    }

    return sorted[sampleCount / 2];
}

float AnalogSampler::average() const
{
    if (!bufferFilled)
        return 0;

    long sum = 0;

    for (int i = 0; i < sampleCount; i++)
    {
        sum += samples[i];
    }

    return (float)sum / sampleCount;
}