// EC DFRobot-vs-Generic-TDS Diagnostic
//
// STANDALONE bench sketch. Not part of BasilienceFirmware, not #included by
// it, and does not touch any BasilienceFirmware source file. It exists to
// answer one question: when the EC probe sits in a fixed 1.4 mS/cm buffer,
// does the raw ADC voltage itself drift, and do independent EC/TDS formulas
// (that are NOT the Basilience two-point calibration) drift the same way?
// If they do, the drift is upstream (probe/wiring/ADC). If only Basilience's
// own EC conversion drifts, the problem is in that conversion, not the
// signal.
//
// Hardware config below (pin, ADC resolution/attenuation, millivolt read
// method) is copied from the real firmware so the comparison is on the same
// signal path:
//   - EC_PIN            : BasilienceFirmware/Config.h -> constexpr uint8_t EC_PIN = 34;
//   - ADC resolution     : BasilienceFirmware/SensorManager.cpp SensorManager::begin()
//                          -> analogReadResolution(12);
//   - ADC attenuation    : BasilienceFirmware/SensorManager.cpp SensorManager::begin()
//                          -> analogSetAttenuation(ADC_11db);
//   - mV read method     : BasilienceFirmware/AnalogSampler.* MILLIVOLTS mode
//                          -> analogReadMilliVolts(EC_PIN), the ESP32 core's
//                          own factory-calibrated ADC-to-mV conversion (this
//                          sketch does NOT use a manual *3.3/4095 formula).
//   - Water temp sensor  : BasilienceFirmware/Config.h -> constexpr uint8_t
//                          WATER_TEMP_PIN = 5; a DS18B20 on OneWire, read with
//                          the same OneWire + DallasTemperature libraries the
//                          firmware uses.
//
// This sketch deliberately does NOT use Basilience's own EC calibration
// (Calibration.h EC_CAL_1_VOLTAGE=2.443V/12.88mS, EC_CAL_2_VOLTAGE=2.205V/
// 1.40mS). Those anchors are the thing under test, not a tool to reuse here.
//
// ---------------------------------------------------------------------------
// A) DFRobot EC computation
// ---------------------------------------------------------------------------
// Uses DFRobot's own "DFRobot_EC" Arduino library (Library Manager: search
// "DFRobot_EC", by DFRobot) rather than a hand-copied version of their
// formula, so this is genuinely their normal/default sensor math, not a
// reimplementation that could silently drift from it.
//
// Library: DFRobot_EC (install via Arduino Library Manager)
//   https://github.com/DFRobot/DFRobot_EC
//
// The library stores its own calibration constant (K value) in EEPROM and
// ships with a factory-default K of 1.0 until you calibrate it. This sketch
// intentionally does NOT perform that calibration - ec.readEC(voltage, tempC)
// is called in its normal/default diagnostic form, uncalibrated, exactly as
// the library behaves out of the box. If you want to exercise the DFRobot
// library's OWN calibration flow (its "enterec"/"calec"/"exitec" Serial
// commands, unrelated to Basilience's EC_CAL_* constants), set
// ENABLE_DFROBOT_CALIBRATION_COMMANDS to 1 below - normally leave it 0 so the
// printed lines stay a clean one-line-per-second log.
//
// ---------------------------------------------------------------------------
// B) Generic TDS computation
// ---------------------------------------------------------------------------
// The commonly used analog-TDS-sensor formula (the one that ships in most
// generic "TDS Meter" example code), applied to the SAME voltage as (A),
// with its own simple linear temperature compensation to 25C. It does not
// share any code or state with the DFRobot EC computation above - they are
// two independent conversions of the same measured voltage.
//
// Neither (A) nor (B) touches BasilienceFirmware source. Nothing here is
// written back into Calibration.h or any firmware file.

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <DFRobot_EC.h>

// --- Hardware pins (copied from BasilienceFirmware/Config.h) ---
const int EC_PIN = 34;           // Config.h: constexpr uint8_t EC_PIN = 34;
const int WATER_TEMP_PIN = 5;    // Config.h: constexpr uint8_t WATER_TEMP_PIN = 5; (DS18B20 data pin)

// Set to 1 to enable the DFRobot_EC library's own interactive Serial
// calibration commands (its normal "ENTERC"/"CALEC:x.x"/"EXITEC" flow, sent
// over Serial Monitor). Leave at 0 for a clean, uncalibrated diagnostic log.
#define ENABLE_DFROBOT_CALIBRATION_COMMANDS 0

OneWire oneWire(WATER_TEMP_PIN);
DallasTemperature waterSensor(&oneWire);
DFRobot_EC ec;

const int SAMPLE_COUNT = 30;          // samples averaged/medianed per printed line
const unsigned long SAMPLE_SPACING_MS = 5;
const unsigned long PRINT_INTERVAL_MS = 1000; // ~1 reading/second, per spec

unsigned long lastPrintAt = 0;

void sortInts(int* arr, int n)
{
    for (int i = 1; i < n; i++)
    {
        int key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key)
        {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// Median of SAMPLE_COUNT raw 12-bit ADC counts (0-4095).
int sampleRawMedian()
{
    int samples[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        samples[i] = analogRead(EC_PIN);
        delay(SAMPLE_SPACING_MS);
    }
    sortInts(samples, SAMPLE_COUNT);
    return samples[SAMPLE_COUNT / 2];
}

// Median of SAMPLE_COUNT ESP32-calibrated millivolt readings.
int sampleMilliVoltsMedian()
{
    int samples[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        samples[i] = analogReadMilliVolts(EC_PIN);
        delay(SAMPLE_SPACING_MS);
    }
    sortInts(samples, SAMPLE_COUNT);
    return samples[SAMPLE_COUNT / 2];
}

void setup()
{
    Serial.begin(115200);
    delay(200);

    // Same ADC setup as SensorManager::begin() in the real firmware.
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    waterSensor.begin();

    EEPROM.begin(32); // DFRobot_EC library persists its K value here
    ec.begin();

    Serial.println("Basilience EC diagnostic: DFRobot EC vs generic TDS");
    Serial.println("Probe sits in a fixed reference solution; watch for drift over time.");
    Serial.println("---------------------------------------------------------------");
}

void loop()
{
#if ENABLE_DFROBOT_CALIBRATION_COMMANDS
    // Feeds DFRobot_EC's own calibration command parser if enabled above.
    // Requires a live voltage/temperature, so this call also happens inside
    // the timed block below.
#endif

    unsigned long now = millis();
    if (now - lastPrintAt < PRINT_INTERVAL_MS)
        return;
    lastPrintAt = now;

    int rawMedian = sampleRawMedian();
    int mvMedian = sampleMilliVoltsMedian();
    float voltage = mvMedian / 1000.0f;

    waterSensor.requestTemperatures();
    float tempC = waterSensor.getTempCByIndex(0);
    if (tempC == DEVICE_DISCONNECTED_C || isnan(tempC))
        tempC = 25.0f; // no DS18B20 wired up right now - fall back so EC/TDS math still runs

#if ENABLE_DFROBOT_CALIBRATION_COMMANDS
    ec.calibration(voltage, tempC); // reads/handles ENTERC/CALEC/EXITEC on Serial
#endif

    // A) DFRobot EC library's own default/uncalibrated computation.
    float dfrobotEcMScm = ec.readEC(voltage, tempC);

    // B) Generic analog-TDS formula, independent of (A), same voltage/temp.
    float compensationCoefficient = 1.0f + 0.02f * (tempC - 25.0f);
    float compensationVoltage = voltage / compensationCoefficient;
    float tdsPpm = (133.42f * compensationVoltage * compensationVoltage * compensationVoltage
                    - 255.86f * compensationVoltage * compensationVoltage
                    + 857.39f * compensationVoltage) * 0.5f;

    Serial.print("RAW=");
    Serial.print(rawMedian);
    Serial.print(" | mV=");
    Serial.print(mvMedian);
    Serial.print(" | V=");
    Serial.print(voltage, 4);
    Serial.print(" | Temp=");
    Serial.print(tempC, 1);
    Serial.print("C | DFRobot EC=");
    Serial.print(dfrobotEcMScm, 3);
    Serial.print(" mS/cm | Generic TDS=");
    Serial.print(tdsPpm, 0);
    Serial.println(" ppm");
}
