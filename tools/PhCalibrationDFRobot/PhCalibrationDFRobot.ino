// pH probe bench tool using the DFRobot_PH library (vendored alongside
// this sketch, adapted - see DFRobot_PH.cpp's header for the three
// deviations from upstream). Calibration lives in the ESP32's
// flash-backed EEPROM, pre-seeded with Basilience's own already-verified
// two-point calibration (pH 6.86 -> 2562.40 mV, pH 4.01 -> 2931.53 mV,
// see BasilienceFirmware/Calibration.h) - so this reads correctly right
// after flashing, with NO new physical buffer dip required.
//
// Stability: every printed pH goes through the same stability window
// production firmware uses (SensorManager.cpp / Config.h -
// STABILITY_SAMPLE_WINDOW=10, STABILITY_SAMPLE_INTERVAL_MS=1000,
// PH_STABILITY_TOLERANCE=0.05) - 10 samples, about a second apart, all
// within 0.05 pH of each other, before a reading is marked [STABLE].
// Until then it's printed as [SETTLING] - a still-moving probe (or a
// trim pot being turned) will never falsely read as stable.
//
// Commands:
//   sethi <pH>   - store the CURRENT reading as the higher-pH point
//                  (only accepted while [STABLE]), e.g. "sethi 6.86"
//   setlo <pH>   - same, for the lower-pH point, e.g. "setlo 4.01"
//   enterph, calph, exitph - upstream DFRobot flow for redoing a
//                  calibration with true 7.0/4.0 buffers; note its
//                  buffer-recognition windows are tuned for DFRobot's own
//                  circuit and likely won't recognize this hardware's
//                  actual voltages (see DFRobot_PH.cpp) - prefer
//                  sethi/setlo instead.
//
// Acquisition matches production (SensorManager::readPH()): GPIO35,
// 12-bit ADC, ADC_11db attenuation, median of 51 millivolt samples taken
// every 20ms.

#include <Arduino.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"

constexpr uint8_t PH_SENSOR_PIN = 35;
constexpr uint8_t PH_SAMPLE_COUNT = 51;
constexpr unsigned long PH_SAMPLE_INTERVAL_MS = 20;
constexpr size_t EEPROM_SIZE = 32; // DFRobot_PH uses bytes 0-15

// Matches Config.h's own stability window exactly, so this tool's notion
// of "stable" agrees with production's.
constexpr uint8_t STABILITY_SAMPLE_WINDOW = 10;
constexpr unsigned long STABILITY_SAMPLE_INTERVAL_MS = 1000UL;
constexpr float PH_STABILITY_TOLERANCE = 0.05f;

int samples[PH_SAMPLE_COUNT];
uint8_t sampleIndex = 0;
bool bufferFilled = false;
unsigned long lastSampleAt = 0;
float lastMedianMv = 0.0f;

float phWindow[STABILITY_SAMPLE_WINDOW];
uint8_t phWindowCount = 0;
uint8_t phWindowNext = 0;
unsigned long lastWindowSampleAt = 0;
bool isStable = false;
float lastPh = 0.0f;

float temperature = 25.0; // no temperature probe on this bench setup
DFRobot_PH ph;

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

// Same rule as SensorManager::updateStabilityWindow(): once the window is
// full, all STABILITY_SAMPLE_WINDOW samples must sit within
// PH_STABILITY_TOLERANCE of each other (max-min check) for the reading to
// count as stable.
void updateStabilityWindow(float candidatePh)
{
    phWindow[phWindowNext] = candidatePh;
    phWindowNext = (phWindowNext + 1) % STABILITY_SAMPLE_WINDOW;
    if (phWindowCount < STABILITY_SAMPLE_WINDOW) phWindowCount++;

    if (phWindowCount < STABILITY_SAMPLE_WINDOW)
    {
        isStable = false;
        return;
    }

    float minPh = phWindow[0];
    float maxPh = phWindow[0];
    for (uint8_t i = 1; i < STABILITY_SAMPLE_WINDOW; i++)
    {
        if (phWindow[i] < minPh) minPh = phWindow[i];
        if (phWindow[i] > maxPh) maxPh = phWindow[i];
    }
    isStable = (maxPh - minPh) <= PH_STABILITY_TOLERANCE;
}

void handleCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;

    String upperCmd = cmd;
    upperCmd.toUpperCase();

    if (upperCmd.startsWith("SETHI ") || upperCmd.startsWith("SETLO "))
    {
        bool isHi = upperCmd.startsWith("SETHI ");
        float bufferPh = cmd.substring(6).toFloat();

        if (!bufferFilled)
        {
            Serial.println(F("No reading yet - wait a moment and try again."));
        }
        else if (!isStable)
        {
            Serial.println(F("Not [STABLE] yet - wait for the reading to settle before capturing."));
        }
        else if (bufferPh <= 0.0f || bufferPh >= 14.0f)
        {
            Serial.println(F("Enter a real buffer pH, e.g. \"sethi 6.86\"."));
        }
        else
        {
            ph.setCalibrationPoint(isHi, lastMedianMv, bufferPh);
        }
        return;
    }

    if (upperCmd == "CLEAR")
    {
        // Wipes the 16 bytes DFRobot_PH uses back to blank (0xFF), then
        // re-runs begin() so it re-seeds its compiled-in defaults - needed
        // because begin() only writes its defaults when EEPROM is already
        // blank; once anything has been saved (via setlo/sethi or calph),
        // reflashing alone won't undo it on this board.
        for (int i = 0; i < 16; i++) EEPROM.write(i, 0xFF);
        EEPROM.commit();
        ph.begin();
        Serial.println(F(">>>EEPROM cleared - reloaded compiled-in defaults<<<"));
        return;
    }

    // Anything else (enterph/calph/exitph/unrecognized) goes through the
    // upstream DFRobot command parser unchanged.
    ph.calibration(lastMedianMv, temperature, (char*)cmd.c_str());
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Must happen before ph.begin() - DFRobot_PH::begin() reads straight
    // from EEPROM.read()/EEPROM.write() without calling begin() itself.
    EEPROM.begin(EEPROM_SIZE);
    ph.begin();

    Serial.println();
    Serial.println(F("=== Basilience pH Bench Tool (DFRobot_PH / EEPROM) ==="));
    Serial.println(F("Pre-seeded with the existing verified calibration - reads correctly now."));
    Serial.println(F("Commands: sethi <pH>, setlo <pH>, clear | enterph, calph, exitph"));
    Serial.println();
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
            lastMedianMv = (float)sampleMedianMv();
        }
    }

    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    if (bufferFilled && millis() - lastWindowSampleAt >= STABILITY_SAMPLE_INTERVAL_MS)
    {
        lastWindowSampleAt = millis();
        lastPh = ph.readPH(lastMedianMv, temperature);
        updateStabilityWindow(lastPh);

        Serial.print(F("mV="));
        Serial.print(lastMedianMv, 0);
        Serial.print(F("  pH="));
        Serial.print(lastPh, 2);
        Serial.println(isStable ? F("  [STABLE]") : F("  [SETTLING]"));
    }
}
