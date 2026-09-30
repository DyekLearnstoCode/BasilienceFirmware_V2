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
    memset(samples, 0, sizeof(samples));
    memset(rawSamples, 0, sizeof(rawSamples));

    sampleIndex = 0;
    bufferFilled = false;
    lastSampleTime = 0;
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

        // Same physical conversion this branch already performs - no extra
        // ADC transaction needed, just also keep it in rawSamples[] so
        // rawMedian() works uniformly regardless of mode (see its own
        // comment in AnalogSampler.h).
        rawSamples[sampleIndex] = samples[sampleIndex];

        break;

    case MILLIVOLTS:

        samples[sampleIndex] =
            analogReadMilliVolts(pin);

        // A second, separate ADC conversion from analogReadMilliVolts()'s
        // own internal one, microseconds apart on the same pin - negligible
        // for this purpose (rail-fault detection looks for a condition
        // sustained across many SECONDS of samples, not sub-millisecond
        // precision), but real: rawSamples[i] and samples[i] are not
        // guaranteed to be the exact same physical sample instant in this
        // mode. Avoids reaching into the ESP-IDF calibration internals
        // (esp_adc_cal_raw_to_voltage()) that analogReadMilliVolts() itself
        // uses, which would be a larger, riskier change for the same result.
        rawSamples[sampleIndex] =
            analogRead(pin);

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

int AnalogSampler::rawMedian() const
{
    if (!bufferFilled)
        return 0;

    int sorted[MAX_SAMPLES];

    memcpy(
        sorted,
        rawSamples,
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

int AnalogSampler::minValue() const
{
    if (!bufferFilled)
        return 0;

    int lo = samples[0];
    for (int i = 1; i < sampleCount; i++)
    {
        if (samples[i] < lo) lo = samples[i];
    }
    return lo;
}

int AnalogSampler::maxValue() const
{
    if (!bufferFilled)
        return 0;

    int hi = samples[0];
    for (int i = 1; i < sampleCount; i++)
    {
        if (samples[i] > hi) hi = samples[i];
    }
    return hi;
}