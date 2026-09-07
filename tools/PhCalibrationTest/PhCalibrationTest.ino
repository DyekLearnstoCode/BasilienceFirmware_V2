// Standalone pH probe calibration/bench-test tool for the Basilience ESP32
// hardware. Reproduces the exact acquisition path used in production
// (SensorManager::readPH()): GPIO35, 12-bit ADC, ADC_11db attenuation,
// median of 51 millivolt samples taken every 20ms.
//
// Usage:
//   1. Flash this sketch alone (not the full firmware) and open the Serial
//      Monitor at 115200 baud, line ending "Newline".
//   2. Rinse the probe, dip it in a pH 6.86 buffer, wait for the live
//      reading to settle, then send "1".
//   3. Rinse, dip in a pH 4.01 buffer, wait to settle, then send "2".
//   4. Send "c" to compute and print the new PH_SLOPE / PH_OFFSET.
//   5. Optional: dip in a third buffer (e.g. pH 9.18), send "3", then "c"
//      again to see how far the 3rd point falls from the 2-point line
//      (a linearity check, not part of the fit).
//   6. Paste the printed constants into Calibration.h.
//
// Send "r" at any time to clear captured points and start over.

#include <Arduino.h>

constexpr uint8_t PH_SENSOR_PIN = 34;
constexpr uint8_t PH_SAMPLE_COUNT = 51;
constexpr unsigned long PH_SAMPLE_INTERVAL_MS = 20;

int samples[PH_SAMPLE_COUNT];
uint8_t sampleIndex = 0;
bool bufferFilled = false;
unsigned long lastSampleAt = 0;
int lastMedianMv = 0;

struct CalPoint
{
    bool captured = false;
    float ph = 0.0f;
    int mv = 0;
};

CalPoint point1, point2, point3;

enum class PendingInput
{
    NONE,
    PH_FOR_POINT1,
    PH_FOR_POINT2,
    PH_FOR_POINT3,
};

PendingInput pendingInput = PendingInput::NONE;

int sampleMedianMv()
{
    int sorted[PH_SAMPLE_COUNT];
    memcpy(sorted, samples, sizeof(samples));

    for (int i = 0; i < PH_SAMPLE_COUNT - 1; i++)
    {
        for (int j = i + 1; j < PH_SAMPLE_COUNT; j++)
        {
            if (sorted[j] < sorted[i])
            {
                int tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    return sorted[PH_SAMPLE_COUNT / 2];
}

void printMenu()
{
    Serial.println();
    Serial.println(F("=== Basilience pH Calibration Test ==="));
    Serial.println(F("1 = capture point 1   2 = capture point 2   3 = capture point 3 (optional check)"));
    Serial.println(F("c = compute calibration from captured points"));
    Serial.println(F("r = reset captured points"));
    Serial.println();
}

void capturePoint(CalPoint &point, const char *label)
{
    if (!bufferFilled)
    {
        Serial.println(F("No reading yet - wait a moment and try again."));
        return;
    }

    point.mv = lastMedianMv;
    point.captured = false; // set true once the pH value is entered

    Serial.print(F("Captured "));
    Serial.print(label);
    Serial.print(F(" raw = "));
    Serial.print(point.mv);
    Serial.println(F(" mV. Now type the buffer's actual pH value (e.g. 6.86) and press Enter:"));
}

void computeCalibration()
{
    if (!point1.captured || !point2.captured)
    {
        Serial.println(F("Need points 1 and 2 captured (with their pH values) before computing."));
        return;
    }

    if (point1.mv == point2.mv)
    {
        Serial.println(F("Point 1 and point 2 have identical mV readings - can't fit a line. Re-capture."));
        return;
    }

    float slope = (point2.ph - point1.ph) / (float)(point2.mv - point1.mv);
    float offset = point1.ph - slope * point1.mv;

    Serial.println();
    Serial.println(F("=== New calibration ==="));
    Serial.print(F("Point 1: "));
    Serial.print(point1.ph, 2);
    Serial.print(F(" pH -> "));
    Serial.print(point1.mv);
    Serial.println(F(" mV"));
    Serial.print(F("Point 2: "));
    Serial.print(point2.ph, 2);
    Serial.print(F(" pH -> "));
    Serial.print(point2.mv);
    Serial.println(F(" mV"));

    Serial.println();
    Serial.println(F("Paste into Calibration.h:"));
    Serial.print(F("constexpr float PH_SLOPE  = "));
    Serial.print(slope, 8);
    Serial.println(F("f;"));
    Serial.print(F("constexpr float PH_OFFSET = "));
    Serial.print(offset, 5);
    Serial.println(F("f;"));

    if (point3.captured)
    {
        float predicted = slope * point3.mv + offset;
        float error = predicted - point3.ph;

        Serial.println();
        Serial.println(F("=== Linearity check (point 3) ==="));
        Serial.print(F("Buffer pH = "));
        Serial.print(point3.ph, 2);
        Serial.print(F(", raw = "));
        Serial.print(point3.mv);
        Serial.println(F(" mV"));
        Serial.print(F("Predicted by 2-point fit = "));
        Serial.print(predicted, 3);
        Serial.print(F(", error = "));
        Serial.println(error, 3);
    }

    Serial.println();
}

void handleCommand(const String &cmd)
{
    if (cmd == "1")
    {
        capturePoint(point1, "point 1");
        pendingInput = PendingInput::PH_FOR_POINT1;
    }
    else if (cmd == "2")
    {
        capturePoint(point2, "point 2");
        pendingInput = PendingInput::PH_FOR_POINT2;
    }
    else if (cmd == "3")
    {
        capturePoint(point3, "point 3");
        pendingInput = PendingInput::PH_FOR_POINT3;
    }
    else if (cmd == "c")
    {
        computeCalibration();
    }
    else if (cmd == "r")
    {
        point1 = CalPoint();
        point2 = CalPoint();
        point3 = CalPoint();
        pendingInput = PendingInput::NONE;
        Serial.println(F("Cleared captured points."));
    }
    else if (pendingInput != PendingInput::NONE)
    {
        float ph = cmd.toFloat();

        CalPoint *target = nullptr;
        if (pendingInput == PendingInput::PH_FOR_POINT1) target = &point1;
        else if (pendingInput == PendingInput::PH_FOR_POINT2) target = &point2;
        else if (pendingInput == PendingInput::PH_FOR_POINT3) target = &point3;

        target->ph = ph;
        target->captured = true;
        pendingInput = PendingInput::NONE;

        Serial.print(F("Stored pH = "));
        Serial.println(ph, 2);
    }
    else
    {
        printMenu();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    printMenu();
}

void loop()
{
    if (millis() - lastSampleAt >= PH_SAMPLE_INTERVAL_MS)
    {
        lastSampleAt = millis();

        samples[sampleIndex] = analogReadMilliVolts(PH_SENSOR_PIN);
        sampleIndex++;

        if (sampleIndex >= PH_SAMPLE_COUNT)
        {
            sampleIndex = 0;
            bufferFilled = true;

            lastMedianMv = sampleMedianMv();

            Serial.print(F("mV="));
            Serial.println(lastMedianMv);
        }
    }

    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0)
        {
            handleCommand(cmd);
        }
    }
}
