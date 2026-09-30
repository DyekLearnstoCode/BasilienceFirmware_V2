// EC Calibration Capture
//
// Interactive capture flow, same idea as the pH buffer-capture tool: dip
// the EC probe into a reference solution, watch the live mV reading settle
// on Serial, then type that solution's known EC value (in mS/cm) into
// Serial Monitor and press Enter. The sketch captures the current settled
// median mV reading and pairs it with the EC value you typed. Repeat for
// each solution, then read the captured pairs back here to fill in
// Calibration.h's EC_CAL_1_*/EC_CAL_2_* constants.
//
// Uses the same AnalogSampler class, pin, sample count/interval, and ADC
// setup as BasilienceFirmware's own SensorManager::readEC(), so captured
// values match what the real firmware would read for the same probe input.
//
// Serial Monitor must be set to send a newline ("Newline" or "Both NL & CR")
// for typed input to be recognized.
//
// Commands (type into Serial Monitor, press Enter):
//   <number>   e.g. "12.88" or "1.4" - captures the current settled mV
//              reading and labels it with this EC value (mS/cm). Refused if
//              the reading hasn't held steady yet (see STABILITY_* below) -
//              keep waiting and watch the "spread=" number in the live line
//              drop below the "stable" threshold.
//   f <number> forces a capture even while unstable (e.g. "f 12.88") - the
//              captured point is flagged so you know to treat it with less
//              confidence.
//   r          clears all captured points and starts over
//   l          lists all captured points again without capturing a new one
//   u          undoes (removes) the last captured point
//
// A capture isn't the end of the story: for a few seconds afterward the
// sketch keeps checking that the reading stays where it was captured. If it
// turns out it was only briefly calm and starts drifting again, you'll get
// a "WARNING: Point N drifted after capture" line - type 'u' to drop that
// point and wait for it to genuinely settle before recapturing.

#include "AnalogSampler.h"

constexpr uint8_t EC_PIN = 34;                    // matches Config.h EC_PIN
constexpr uint8_t EC_SAMPLE_COUNT = 61;           // matches Config.h EC_SAMPLE_COUNT (AnalogSampler clamps to its own 60-sample max internally)
constexpr unsigned long EC_SAMPLE_INTERVAL = 20;  // matches Config.h EC_SAMPLE_INTERVAL (ms)
constexpr unsigned long LIVE_PRINT_INTERVAL_MS = 1000;
constexpr uint8_t MAX_CAPTURES = 6;

// A capture is only accepted once the last STABILITY_WINDOW live readings
// (one per LIVE_PRINT_INTERVAL_MS, so this many seconds of history) have all
// stayed within STABILITY_THRESHOLD_MV of each other. Tune these if real
// probes settle noisier/quieter than this in practice.
constexpr uint8_t STABILITY_WINDOW = 5;
constexpr int STABILITY_THRESHOLD_MV = 5;

// After a capture is accepted, it keeps being watched for this many more
// live ticks (seconds). If the reading ever drifts more than
// STABILITY_THRESHOLD_MV away from the value that was actually captured
// during that window, the "stable" moment was a fluke, not real settling -
// the point gets flagged and you're told to undo ('u') and recapture.
constexpr uint8_t POST_CAPTURE_WATCH_TICKS = 5;

AnalogSampler ecSampler(EC_PIN, EC_SAMPLE_COUNT, EC_SAMPLE_INTERVAL, AnalogSampler::MILLIVOLTS);

unsigned long lastLivePrintAt = 0;
String inputLine = "";

struct CapturedPoint
{
    float ecValue;
    int medianMv;
    bool forced;
};

CapturedPoint captures[MAX_CAPTURES];
uint8_t captureCount = 0;

int recentMedians[STABILITY_WINDOW];
uint8_t recentCount = 0;
uint8_t recentIndex = 0;

// Post-capture verification state - watches the single most recent capture
// only (verifyPointIndex), since that's the one still unproven.
bool verifying = false;
uint8_t verifyPointIndex = 0;
int verifyTargetMv = 0;
uint8_t verifyTicksLeft = 0;

void recordRecentMedian(int medianMv)
{
    recentMedians[recentIndex] = medianMv;
    recentIndex = (recentIndex + 1) % STABILITY_WINDOW;
    if (recentCount < STABILITY_WINDOW) recentCount++;
}

// Returns whether the last STABILITY_WINDOW live readings agree closely
// enough to trust. outSpread is always set (0 if there isn't enough history
// yet) so callers can report it either way.
bool isStable(int &outSpread)
{
    if (recentCount < STABILITY_WINDOW)
    {
        outSpread = 0;
        return false;
    }

    int lo = recentMedians[0];
    int hi = recentMedians[0];
    for (uint8_t i = 1; i < STABILITY_WINDOW; i++)
    {
        if (recentMedians[i] < lo) lo = recentMedians[i];
        if (recentMedians[i] > hi) hi = recentMedians[i];
    }

    outSpread = hi - lo;
    return outSpread <= STABILITY_THRESHOLD_MV;
}

void printLiveStatus()
{
    if (!ecSampler.ready())
    {
        Serial.println("[EC-CAPTURE] warming up...");
        return;
    }

    const int medianMv = ecSampler.median();
    recordRecentMedian(medianMv);

    int spread;
    const bool stable = isStable(spread);

    Serial.print("[EC-CAPTURE] live mV=");
    Serial.print(medianMv);
    Serial.print(" min=");
    Serial.print(ecSampler.minValue());
    Serial.print(" max=");
    Serial.print(ecSampler.maxValue());
    Serial.print(" spread(last ");
    Serial.print(STABILITY_WINDOW);
    Serial.print("s)=");
    Serial.print(spread);
    Serial.print("mV ");
    Serial.println(stable
        ? "- STABLE, safe to type the EC value now"
        : "- not stable yet, keep waiting");

    checkPostCaptureVerification(medianMv);
}

// Confirms (or debunks) the most recent capture. A capture only passed the
// STABILITY_WINDOW check at the instant it was taken - this is what catches
// a value that looked flat for a moment and then kept moving, which the
// short window alone can't tell apart from genuine settling.
void checkPostCaptureVerification(int medianMv)
{
    if (!verifying) return;

    if (abs(medianMv - verifyTargetMv) > STABILITY_THRESHOLD_MV)
    {
        Serial.print("[EC-CAPTURE] WARNING: Point ");
        Serial.print(verifyPointIndex + 1);
        Serial.print(" drifted after capture (captured mV=");
        Serial.print(verifyTargetMv);
        Serial.print(", now mV=");
        Serial.print(medianMv);
        Serial.println(") - that was a brief calm spot, not real settling. Type 'u' to remove it and recapture once it genuinely holds.");
        verifying = false;
        return;
    }

    if (verifyTicksLeft > 0)
    {
        verifyTicksLeft--;
    }

    if (verifyTicksLeft == 0)
    {
        Serial.print("[EC-CAPTURE] Point ");
        Serial.print(verifyPointIndex + 1);
        Serial.println(" held steady - confirmed.");
        verifying = false;
    }
}

void listCaptures()
{
    if (captureCount == 0)
    {
        Serial.println("[EC-CAPTURE] no points captured yet");
        return;
    }

    Serial.println("[EC-CAPTURE] captured points so far:");
    for (uint8_t i = 0; i < captureCount; i++)
    {
        Serial.print("  Point ");
        Serial.print(i + 1);
        Serial.print(": EC=");
        Serial.print(captures[i].ecValue, 3);
        Serial.print(" mS/cm -> mV=");
        Serial.print(captures[i].medianMv);
        Serial.println(captures[i].forced ? "  (forced - was not stable, treat with less confidence)" : "");
    }
}

void handleCommand(String line)
{
    line.trim();
    if (line.length() == 0) return;

    if (line.equalsIgnoreCase("r"))
    {
        captureCount = 0;
        Serial.println("[EC-CAPTURE] cleared all captured points");
        return;
    }

    if (line.equalsIgnoreCase("l"))
    {
        listCaptures();
        return;
    }

    if (line.equalsIgnoreCase("u"))
    {
        if (captureCount == 0)
        {
            Serial.println("[EC-CAPTURE] nothing to undo");
            return;
        }
        captureCount--;
        if (verifying && verifyPointIndex == captureCount) verifying = false;
        Serial.print("[EC-CAPTURE] removed Point ");
        Serial.println(captureCount + 1);
        listCaptures();
        return;
    }

    // "f <value>" forces a capture past the stability gate below.
    bool forceCapture = false;
    if (line.length() > 1 && (line[0] == 'f' || line[0] == 'F') &&
        (line[1] == ' ' || line[1] == '\t'))
    {
        forceCapture = true;
        line = line.substring(2);
        line.trim();
    }

    // Otherwise, treat the line as the known EC value of the solution
    // currently in the probe.
    if (!ecSampler.ready())
    {
        Serial.println("[EC-CAPTURE] not ready yet - wait for the live reading to appear first");
        return;
    }

    const float ecValue = line.toFloat();
    if (ecValue <= 0.0f)
    {
        Serial.println("[EC-CAPTURE] couldn't read that as a positive EC value - type a number like 1.4 or 12.88, or 'r'/'l'/'f <value>'");
        return;
    }

    if (captureCount >= MAX_CAPTURES)
    {
        Serial.println("[EC-CAPTURE] capture list full - type 'r' to clear and start over");
        return;
    }

    int spread;
    const bool stable = isStable(spread);
    if (!stable && !forceCapture)
    {
        Serial.print("[EC-CAPTURE] not captured - reading isn't stable yet (spread=");
        Serial.print(spread);
        Serial.print("mV over the last ");
        Serial.print(STABILITY_WINDOW);
        Serial.println("s, need <= " + String(STABILITY_THRESHOLD_MV) + "mV). Keep waiting, or type 'f " + String(ecValue, 2) + "' to force it anyway.");
        return;
    }

    const int medianMv = ecSampler.median();
    captures[captureCount].ecValue = ecValue;
    captures[captureCount].medianMv = medianMv;
    captures[captureCount].forced = !stable;

    // Watch this point for POST_CAPTURE_WATCH_TICKS more seconds - see
    // checkPostCaptureVerification()'s own comment for why passing the
    // instantaneous stability check isn't proof enough on its own.
    verifying = true;
    verifyPointIndex = captureCount;
    verifyTargetMv = medianMv;
    verifyTicksLeft = POST_CAPTURE_WATCH_TICKS;

    captureCount++;

    Serial.print("[EC-CAPTURE] captured Point ");
    Serial.print(captureCount);
    Serial.print(": EC=");
    Serial.print(ecValue, 3);
    Serial.print(" mS/cm -> mV=");
    Serial.print(medianMv);
    Serial.println(!stable ? "  (forced - was not stable, treat with less confidence)" : "  (stable, verifying for a few more seconds...)");

    listCaptures();
}

void setup()
{
    Serial.begin(115200);

    // Same ADC setup as SensorManager::begin() - without this, the raw
    // counts/millivolt calibration curve won't match production.
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    ecSampler.begin();

    Serial.println("[EC-CAPTURE] Ready. Dip the probe in a reference solution, let it settle,");
    Serial.println("[EC-CAPTURE] then type that solution's known EC in mS/cm and press Enter.");
}

void loop()
{
    ecSampler.update();

    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n')
        {
            handleCommand(inputLine);
            inputLine = "";
        }
        else if (c != '\r')
        {
            inputLine += c;
        }
    }

    const unsigned long now = millis();
    if (now - lastLivePrintAt >= LIVE_PRINT_INTERVAL_MS)
    {
        lastLivePrintAt = now;
        printLiveStatus();
    }
}
