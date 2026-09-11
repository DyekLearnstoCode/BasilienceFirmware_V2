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

    // Diagnostic-only accessors over the current filled buffer (real-hardware
    // pH ADC audit) - the distribution width these expose is what median()
    // is chosen over average() to resist; not used by any filtering
    // decision itself. Return 0 if the buffer isn't filled yet, matching
    // median()/average()'s own not-ready convention.
    int minValue() const;

    int maxValue() const;

    // Raw 12-bit ADC count (analogRead(), 0-4095), captured UNCONDITIONALLY
    // alongside the mode-selected samples[] above regardless of this
    // instance's own ReadMode - i.e. a MILLIVOLTS-mode sampler (pH/EC's own
    // ecSampler/phSampler) still tracks raw counts too, at no extra
    // configuration cost. Added for rail-proximity hardware-fault detection
    // (see Config.h's PH_FAULT_RAW_*/EC_FAULT_RAW_* and SensorManager::
    // readPH()/readEC()): analogReadMilliVolts()'s calibrated ceiling varies
    // per-chip (eFuse calibration data), but the raw ADC's own saturation
    // point (0 or 4095 at 12-bit) is an architectural constant true on every
    // ESP32 unit regardless of calibration - a chip-independent corroborating
    // signal the calibrated millivolt reading alone cannot provide. Not used
    // by any EXISTING filtering/calibration decision - median()/average()
    // above are completely unaffected, still driven purely by ReadMode.
    int rawMedian() const;

private:
    uint8_t pin;

    uint8_t sampleCount;

    unsigned long sampleInterval;

    ReadMode mode;

    unsigned long lastSampleTime;

    int samples[MAX_SAMPLES];

    // See rawMedian()'s own comment - parallel to samples[] above, always
    // populated regardless of mode.
    int rawSamples[MAX_SAMPLES];

    uint8_t sampleIndex;

    bool bufferFilled;
};

#endif