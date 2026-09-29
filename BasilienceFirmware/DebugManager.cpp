#include "DebugManager.h"
#include "ActuatorManager.h"

#include "Globals.h"
#include "Config.h"
#include "RTCManager.h"
#include <WiFi.h>

// ======================================================
// Serial Diagnostics / Observability pass - tuning constants
// ======================================================
// Local to this file: purely output-cadence/threshold values, never read by
// any control/automation code, so they live here rather than in Config.h
// alongside the actual control-timing constants.
namespace
{
    // Section 4: one compact grouped snapshot every ~5s, independent of the
    // older per-page DEBUG_INTERVAL cadence below.
    constexpr unsigned long STATUS_SNAPSHOT_INTERVAL_MS = 5000UL;

    // Section 6: loop-gap stall threshold. Chosen against this firmware's own
    // bounded worst cases rather than an arbitrary guess - a Firebase RTDB call
    // on an already-open session is capped at ~4000ms by
    // config.timeout.serverResponse (see FirebaseManager::begin() for exactly
    // what that setting does and does not cover; a call that must open a NEW
    // connection is not capped by it, which is why those are gated behind the
    // background preflight) and the DS18B20 conversion no longer blocks loop()
    // at all. 5000ms sits just above that single-call cap, so an ordinary,
    // already-expected worst-case Firebase timeout does NOT itself trigger a
    // STALL warning, while two such calls stacking in one iteration - or any
    // other unexpected block - does.
    constexpr unsigned long LOOP_STALL_THRESHOLD_MS = 5000UL;

    // Section 11: how often an unchanged, still-active safety block reprints
    // as a "still blocked" reminder, separate from an immediate reprint on
    // reason-change. One minute is frequent enough to notice during a bench/
    // field session without flooding a block that persists for a while (e.g.
    // a genuinely low reservoir).
    constexpr unsigned long SAFETY_BLOCK_REPEAT_INTERVAL_MS = 60000UL;

    // STATUS's own compact cooling-state label - systemState.coolingPulseState
    // is a plain public field, so this is a read-only mapping, not a new
    // AutomationManager accessor.
    const char* coolingPulseStateName(CoolingPulseState state)
    {
        switch (state)
        {
            case CoolingPulseState::IDLE: return "IDLE";
            case CoolingPulseState::FILL: return "FILL";
            case CoolingPulseState::COOL_SOAK: return "SOAK";
            case CoolingPulseState::FLUSH: return "FLUSH";
            default: return "?";
        }
    }
}

void DebugManager::begin()
{
    lastPrintTime = 0;

    currentPage = 0;

    lastStatusPrintAt = millis();
    lastLoopTickAt = millis();
}

// Minimal local/offline TEST_SMS trigger. No Serial command console/parser
// exists anywhere in this firmware, so this is the smallest possible input
// hook rather than a new console framework: accumulate one line, and if it
// is exactly "TEST_SMS" call the SAME NotificationManager::requestTestSms()
// the Firebase-triggered path (FirebaseManager::readTestSmsCommand())
// calls - there is exactly one TEST_SMS implementation regardless of which
// trigger fired it. Deliberately called from the very top of update(),
// ahead of every DEBUG_ENABLED/Serial-Monitor-Focus-Mode gate below - this
// is a command INPUT, not diagnostic output, and must work identically
// whether debug printing is on/off or a controller is isolated for bench
// testing. Fully local: Serial + the NVS-backed SmsRecipientCache +
// GsmManager, no Wi-Fi/Firebase involved.
void DebugManager::checkTestSmsCommand()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c == '\r') continue;

        if (c == '\n')
        {
            serialLineBuffer.trim();
            if (serialLineBuffer == "TEST_SMS")
            {
                Serial.println("[SMS-TEST] Serial TEST_SMS command received");
                notificationManager.requestTestSms();
            }
            // Serial Diagnostics / Observability pass (section 15): the
            // minimal runtime level switch, reusing this same input hook
            // rather than a second command parser. Output-only - changes
            // nothing about what the firmware does, only what reaches Serial.
            else if (serialLineBuffer == "LOG_ERROR")
            {
                logLevel = LogLevel::LEVEL_ERROR;
                Serial.println("[BOOT] Log level -> ERROR");
            }
            else if (serialLineBuffer == "LOG_NORMAL")
            {
                logLevel = LogLevel::LEVEL_NORMAL;
                Serial.println("[BOOT] Log level -> NORMAL");
            }
            else if (serialLineBuffer == "LOG_VERBOSE")
            {
                logLevel = LogLevel::LEVEL_VERBOSE;
                Serial.println("[BOOT] Log level -> VERBOSE");
            }
            serialLineBuffer = "";
            continue;
        }

        // Bounded so unrelated Serial noise (or a line with no newline) can
        // never grow this indefinitely.
        if (serialLineBuffer.length() < 32)
        {
            serialLineBuffer += c;
        }
    }
}

// ======================================================
// Serial Diagnostics / Observability pass
// ======================================================
// Standard line prefix (section 3): "[HH:MM:SS][CAT] " once RTC time is
// valid, "[000123456ms][CAT] " (9-digit zero-padded millis()) before that -
// e.g. immediately after boot, or on a unit with no RTC fitted at all. No
// trailing space/newline; callers finish the line themselves.
void DebugManager::printLogPrefix(const char* category) const
{
    if (rtcManager.hasValidTime())
    {
        char buf[12];
        snprintf(buf, sizeof(buf), "[%02u:%02u:%02u]",
                  (unsigned)rtcManager.getHour(),
                  (unsigned)rtcManager.getMinute(),
                  (unsigned)rtcManager.getSecond());
        Serial.print(buf);
    }
    else
    {
        char buf[14];
        snprintf(buf, sizeof(buf), "[%09lums]", millis());
        Serial.print(buf);
    }

    Serial.print("[");
    Serial.print(category);
    Serial.print("] ");
}

// Section 11: one rate-limited line per independent safety-block channel.
// Reprints on reason-change (including clearing, an empty reason) or once
// SAFETY_BLOCK_REPEAT_INTERVAL_MS has elapsed with the same reason still
// active - never on every re-evaluation of a persistently blocked condition.
void DebugManager::logSafetyBlock(SafetyLogChannel channel, const char* subsystem, const String& reason)
{
    SafetyBlockState& state = safetyBlockState[static_cast<uint8_t>(channel)];

    if (reason.length() == 0)
    {
        // Condition cleared - reset silently so the next real block prints
        // immediately rather than waiting out the repeat interval.
        state.lastReason = "";
        state.lastPrintedAt = 0;
        return;
    }

    const bool reasonChanged = reason != state.lastReason;
    const bool repeatDue = state.lastPrintedAt != 0 &&
        millis() - state.lastPrintedAt >= SAFETY_BLOCK_REPEAT_INTERVAL_MS;

    if (!reasonChanged && !repeatDue && state.lastPrintedAt != 0)
        return;

    if (!atLeast(LogLevel::LEVEL_NORMAL))
    {
        // Still track the reason/timing so a level change back to NORMAL
        // doesn't immediately reprint a block that has been active/unchanged
        // the whole time it was suppressed.
        state.lastReason = reason;
        state.lastPrintedAt = millis();
        return;
    }

    printLogPrefix("SAFE");
    Serial.print(subsystem);
    Serial.print(" BLOCKED | reason=");
    Serial.println(reason);

    state.lastReason = reason;
    state.lastPrintedAt = millis();
}

// Section 6/7 support: called once per loop() iteration, at the very top -
// see BasilienceFirmware.ino. gap is the time since the previous call, i.e.
// how long the PREVIOUS iteration actually took. A stall print fires
// immediately (not deferred to the next STATUS line) since that is the whole
// point of a stall warning; the running max for the current STATUS window is
// otherwise only consumed by printStatusSnapshot() below.
void DebugManager::recordLoopTick()
{
    const unsigned long now = millis();
    lastLoopGapMs = now - lastLoopTickAt;
    lastLoopTickAt = now;

    if (lastLoopGapMs > maxLoopGapSinceStatusMs)
        maxLoopGapSinceStatusMs = lastLoopGapMs;

    if (lastLoopGapMs >= LOOP_STALL_THRESHOLD_MS && atLeast(LogLevel::LEVEL_NORMAL))
    {
        printLogPrefix("PERF");
        Serial.print("LOOP STALL ");
        Serial.print(lastLoopGapMs);
        Serial.println("ms");
    }
}

// Section 6: Firebase call duration. The SLOW case (>= SLOW_FIREBASE_
// OPERATION_MS, unchanged threshold/behavior from before this pass) always
// shows at LEVEL_NORMAL+; a healthy duration is otherwise only shown at
// LEVEL_VERBOSE, so NORMAL never sees "every healthy Firebase transaction".
void DebugManager::logFirebaseDuration(const char* operation, unsigned long durationMs) const
{
    const bool slow = durationMs >= SLOW_FIREBASE_OPERATION_MS;

    if (!slow && !atLeast(LogLevel::LEVEL_VERBOSE))
        return;
    if (slow && !atLeast(LogLevel::LEVEL_NORMAL))
        return;

    printLogPrefix("PERF");
    Serial.print("Firebase ");
    Serial.print(operation);
    if (slow)
    {
        Serial.print(" SLOW");
    }
    Serial.print(" ");
    Serial.print(durationMs);
    Serial.println("ms");
}

// Section 4: the compact grouped snapshot. Internally throttled to
// STATUS_SNAPSHOT_INTERVAL_MS - callers need no own gate. Every value read
// here is an existing, already-computed field; nothing is recomputed or
// re-sampled just to populate this line. Never fabricates a value for an
// unavailable sensor - see the "--"/fault-flag handling below.
void DebugManager::printStatusSnapshot()
{
    if (millis() - lastStatusPrintAt < STATUS_SNAPSHOT_INTERVAL_MS)
        return;
    lastStatusPrintAt = millis();

    printLogPrefix("STATUS");
    Serial.println();

    // ---- NET ----
    Serial.print("NET   WiFi=");
    Serial.print(systemState.wifiConnected ? "UP" : "DOWN");
    Serial.print(" Firebase=");
    Serial.print(systemState.firebaseConnected ? "READY" : "UNAVAILABLE");
    if (systemState.wifiConnected)
    {
        Serial.print(" RSSI=");
        Serial.print(WiFi.RSSI());
        Serial.print("dBm");
    }
    Serial.println();

    // ---- CYCLE ----
    Serial.print("CYCLE Active=");
    Serial.print(harvestScheduleCache.isActive() ? "YES" : "NO");
    Serial.print(" Manual=");
    Serial.print(systemState.manualMode ? "YES" : "NO");
    Serial.print(" Mode=");
    Serial.println(getModeName(systemState.currentMode));

    // ---- SENS ----
    // Flags (section 4): S=stable, U=valid-but-not-yet-confirmed-stable,
    // X=unavailable/fault, V=valid (no stability concept implemented for
    // that parameter). Only ever derived from an existing firmware flag -
    // never invented.
    Serial.print("SENS  pH=");
    if (sensors.phFault || !isfinite(sensors.ph)) Serial.print("--[X]");
    else { Serial.print(sensors.ph, 2); Serial.print(sensorManager.isPhCurrentlyStable() ? "[S]" : "[U]"); }

    Serial.print(" EC=");
    if (sensors.ecFault || !isfinite(sensors.ec)) Serial.print("--[X]");
    else { Serial.print(sensors.ec, 2); Serial.print(sensorManager.isEcCurrentlyStable() ? "[S]" : "[U]"); }

    Serial.print(" WL=");
    if (!isfinite(sensors.waterLevelCm)) Serial.print("--[X]");
    else
    {
        Serial.print(sensors.waterLevelCm, 2); Serial.print("cm/");
        Serial.print((int)(sensors.waterLevel + 0.5f)); Serial.print("%[V]");
    }

    Serial.print(" WT=");
    if (!isfinite(sensors.waterTemp)) Serial.print("--[X]");
    else { Serial.print(sensors.waterTemp, 1); Serial.print("C[V]"); }

    Serial.print(" AT=");
    if (!sensors.dhtAvailable) Serial.print("--[X]");
    else { Serial.print(sensors.temperature, 1); Serial.print("C[V]"); }

    Serial.print(" RH=");
    if (!sensors.dhtAvailable) Serial.println("--[X]");
    else { Serial.print(sensors.humidity, 0); Serial.println("%[V]"); }

    // ---- AUTO ----
    Serial.print("AUTO  pH=");
    Serial.print(systemState.currentMode == DOSING_PH ? "DOSING" :
                  systemState.currentMode == STABILIZING_PH ? "STABILIZING" : "IDLE");
    Serial.print(" EC=");
    Serial.print(systemState.currentMode == DOSING_EC ? "DOSING" :
                  systemState.currentMode == STABILIZING_EC ? "STABILIZING" : "IDLE");
    Serial.print(" Refill=");
    Serial.print(systemState.currentMode == REFILLING ? "ACTIVE" : "IDLE");
    Serial.print(" Cool=");
    Serial.print(coolingPulseStateName(systemState.coolingPulseState));
    Serial.print(" Fog=");
    Serial.println(actuatorManager.isOn(FOGGER) ? "ON" : "OFF");

    // ---- ACT ----
    Serial.print("ACT   Fog=");
    Serial.print(actuatorManager.isOn(FOGGER) ? "ON" : "OFF");
    Serial.print(" Blower=");
    Serial.print(actuatorManager.isOn(BLOWER) ? "ON" : "OFF");
    Serial.print(" Canopy=");
    Serial.print(actuatorManager.getStatus(CANOPY_FAN).speed);
    Serial.print("% Circ=");
    Serial.print(actuatorManager.isOn(CIRCULATION_PUMP) ? "ON" : "OFF");
    Serial.print(" Peltier=");
    Serial.print(actuatorManager.isOn(PELTIER) ? "ON" : "OFF");
    Serial.print(" Valve=");
    Serial.println(actuatorManager.isOn(SOLENOID) ? "ON" : "OFF");

    Serial.print("      pHUp=");
    Serial.print(actuatorManager.isOn(PH_UP_PUMP) ? "ON" : "OFF");
    Serial.print(" pHDown=");
    Serial.print(actuatorManager.isOn(PH_DOWN_PUMP) ? "ON" : "OFF");
    Serial.print(" Grow=");
    Serial.print(actuatorManager.isOn(GROW_PUMP) ? "ON" : "OFF");
    Serial.print(" Bloom=");
    Serial.print(actuatorManager.isOn(BLOOM_PUMP) ? "ON" : "OFF");
    Serial.print(" Light=");
    Serial.println(actuatorManager.isOn(GROW_LIGHT) ? "ON" : "OFF");

    // ---- PERF ----
    Serial.print("PERF  LoopLast=");
    Serial.print(lastLoopGapMs);
    Serial.print("ms LoopMax5s=");
    Serial.print(maxLoopGapSinceStatusMs);
    Serial.println("ms");

    maxLoopGapSinceStatusMs = 0;
}

// Section 14: printed once from setup(), after local managers' begin() but
// before Wi-Fi/Firebase start - see BasilienceFirmware.ino. Deliberately
// small (one block, no per-setting dump).
void DebugManager::printBootSummary()
{
    Serial.println("================= BASILIENCE =================");
    // No firmware-version constant exists anywhere in this codebase, so this
    // says so rather than fabricating one.
    Serial.println("Firmware: (no version constant defined)");
    Serial.print("Cycle cached: ");
    Serial.println(harvestScheduleCache.isActive() ? "YES" : "NO");
    Serial.print("RTC valid: ");
    Serial.println(rtcManager.hasValidTime() ? "YES" : "NO");
    Serial.print("Settings source: ");
    Serial.println(firebaseManager.settingsRestoredFromNvs ? "NVS" : "defaults");
    // Fixed text, not read back from FirebaseManager's config object - the
    // response figure is the value R1 set in FirebaseManager::begin()
    // (config.timeout.serverResponse), and R2's setWaitForConversion(false).
    // The connect/handshake phases are deliberately NOT listed as 4000ms: the
    // library does not apply socketConnection to them (see the comment in
    // FirebaseManager::begin()).
    Serial.println("Firebase timeout: response=4000ms (new connections gated by network preflight)");
    Serial.println("DS18B20: async conversion");
    Serial.println("==============================================");
}

void DebugManager::update()
{
    checkTestSmsCommand();

    // Serial Diagnostics / Observability pass: the new compact instrumentation
    // (STATUS, and every event/transition/SAFE/NET/CMD log added at its own
    // call site elsewhere) is gated purely on the runtime log level, never
    // on DEBUG_ENABLED/Serial Monitor Focus Mode below - those remain
    // exactly what they were, now governing only the pre-existing
    // page-cycling dashboard.
    if (atLeast(LogLevel::LEVEL_NORMAL))
    {
        printStatusSnapshot();
    }

    // Consolidation: the pre-existing page-cycling dashboard
    // (printSystemStatus/printSensors/printAlerts/printActuators/printRTC)
    // is high-volume, raw, multi-line-per-page output - exactly what NORMAL
    // should avoid. Nothing below is removed (still useful for deep
    // debugging), it is demoted to LEVEL_VERBOSE so NORMAL (the default,
    // and what farm/data-gathering testing uses) isn't flooded by it
    // alongside the new STATUS line covering the same ground far more
    // compactly.
    if (!atLeast(LogLevel::LEVEL_VERBOSE))
        return;

    if (!DEBUG_ENABLED)
        return;

    // Serial Monitor Focus Mode: the periodic round-robin dashboards below
    // are exactly the kind of high-volume generic output an isolated
    // controller test does not want competing with its own focused event
    // logs (see shouldPrintDebug()'s own comment - SYSTEM is never true
    // while a controller is isolated). Suppressed entirely rather than
    // replaced with a mode-specific summary - the focused event logs added
    // at each controller's own log sites already serve that role. NONE
    // (normal/full-system operation) is completely unaffected.
    if (!shouldPrintDebug(DebugCategory::SYSTEM))
        return;

    if (millis() - lastPrintTime < DEBUG_INTERVAL)
        return;

    lastPrintTime = millis();

    switch (currentPage)
    {
        case 0:
            printSystemStatus();
            break;

        case 1:
            printSensors();
            break;

        case 2:
            printAlerts();
            break;

        case 3:
            printActuators();
            break;

        case 4:
            printRTC();
            break;
    }

    currentPage++;

    if (currentPage > 4)
        currentPage = 0;
}

void DebugManager::printHeader(const char *title)
{
    Serial.println();
    Serial.println("========================================");
    Serial.print(" BASILIENCE - ");
    Serial.println(title);
    Serial.println("========================================");
}

void DebugManager::printSeparator()
{
    Serial.println("----------------------------------------");
}

void DebugManager::printFloat(
    const char *label,
    float value,
    const char *unit,
    uint8_t decimals)
{
        Serial.print(label);
        Serial.print(" : ");
        Serial.print(value, decimals);

        if (unit != nullptr)
        {
            Serial.print(" ");
            Serial.print(unit);
        }

        Serial.println();
}



void DebugManager::printInteger(
    const char *label,
    int value,
    const char *unit)
{
    Serial.print(label);
    Serial.print(" : ");
    Serial.print(value);

    if (unit != nullptr)
    {
        Serial.print(" ");
        Serial.print(unit);
    }

    Serial.println();
}

void DebugManager::printBool(
    const char *label,
    bool value)
{
    Serial.print(label);
    Serial.print(" : ");
    Serial.println(value ? "ON" : "OFF");
}

void DebugManager::printSensors()
{
    printHeader("SENSOR DATA");
   

    Serial.println();

    printFloat(
        "Air Temperature",
        sensors.temperature,
        "C",
        2);

    printFloat(
        "Humidity",
        sensors.humidity,
        "%",
        2);

    printFloat(
        "Water Temperature",
        sensors.waterTemp,
        "C",
        2);

    printFloat(
        "Water Level",
        sensors.waterLevel,
        "%",
        1);

    printFloat(
        "Water Level Depth",
        sensors.waterLevelCm,
        "cm",
        2);

    printFloat(
        "Water Volume",
        sensors.waterVolumeLiters,
        "L",
        2);

    printFloat(
        "Water Level Distance",
        sensors.waterLevelDistanceCm,
        "cm",
        2);

    printInteger(
        "EC mV",
        sensors.ecRaw,
        nullptr);

    printFloat(
        "EC Voltage",
        sensors.ecVoltage,
        "V",
        3);

    printFloat(
        "EC",
        sensors.ec,
        "mS/cm",
        3);

    printFloat(
        "TDS",
        sensors.tds,
        "ppm",
        0);

    printInteger(
        "pH mV",
        sensors.phMilliVolts,
        "mV");

    printFloat(
        "pH",
        sensors.ph,
        nullptr,
        2);

    printSeparator();
}

void DebugManager::printActuators()
{
    printHeader("ACTUATOR STATES");

    printBool(
        "Fogger",
        actuatorManager.isOn(FOGGER));

    printBool(
        "Grow Light",
        actuatorManager.isOn(GROW_LIGHT));

    printBool(
        "Blower",
        actuatorManager.isOn(BLOWER));

    printBool(
        "Solenoid",
        actuatorManager.isOn(SOLENOID));

    printBool(
        "Grow Pump",
        actuatorManager.isOn(GROW_PUMP));

    printBool(
        "Bloom Pump",
        actuatorManager.isOn(BLOOM_PUMP));

    printBool(
        "pH Up Pump",
        actuatorManager.isOn(PH_UP_PUMP));

    printBool(
        "pH Down Pump",
        actuatorManager.isOn(PH_DOWN_PUMP));

    printBool(
        "Canopy Fan",
        actuatorManager.isOn(CANOPY_FAN));

    printBool(
        "Peltier",
        actuatorManager.isOn(PELTIER));

    // Speed (PWM duty, 0-100) only actually varies for the two PWM-capable
    // actuators - see ActuatorManager::isPwmActuator(). Printed here rather
    // than folded into the ON/OFF lines above so a commanded-but-unapplied
    // speed change is visible on its own.
    printInteger(
        "Canopy Fan Speed",
        actuatorManager.getStatus(CANOPY_FAN).speed,
        "%");

    printInteger(
        "Blower Speed",
        actuatorManager.getStatus(BLOWER).speed,
        "%");

    printSeparator();
}

const char* DebugManager::getModeName(
    SystemMode mode)
{
    switch(mode)
    {
        case SENSOR_STABILIZATION:
            return "SENSOR_STABILIZATION";

        case STARTUP:
            return "STARTUP";

        case NORMAL:
            return "NORMAL";

        case REFILLING:
            return "REFILLING";

        case DOSING_PH:
            return "DOSING_PH";

        case STABILIZING_PH:
            return "STABILIZING_PH";

        case DOSING_EC:
            return "DOSING_EC";

        case STABILIZING_EC:
            return "STABILIZING_EC";

        case SAFETY_LOCK:
            return "SAFETY_LOCK";

        default:
            return "UNKNOWN";
    }
}

void DebugManager::printSystemStatus()
{

    printHeader("SYSTEM STATUS");

    Serial.print("Mode            : ");
    Serial.println(
        getModeName(
            systemState.currentMode));

    printBool(
        "Manual Mode",
        systemState.manualMode);

    printBool(
    "WiFi Connected",
    systemState.wifiConnected);

    printBool(
    "Firebase Connected",
    systemState.firebaseConnected);

    printBool(
        "Reservoir Lock",
        systemState.reservoirLocked);

    printBool(
        "Fog Cycle",
        actuatorManager.isOn(FOGGER));

    Serial.print("pH Direction    : ");

    switch(systemState.phDirection)
    {
        case PH_NONE:
            Serial.println("NONE");
            break;

        case PH_UP:
            Serial.println("UP");
            break;

        case PH_DOWN:
            Serial.println("DOWN");
            break;
    }

    Serial.print("EC Dose Time    : ");
    Serial.print(systemState.ecDoseTime / 1000);
    Serial.println(" sec");

    Serial.print("PH Attempts     : ");
    Serial.println(systemState.phAttempts);

    Serial.print("EC Attempts     : ");
    Serial.println(systemState.ecAttempts);

    Serial.print("Dilution No-Rise Streak : ");
    Serial.println(systemState.ecDilutionNoRiseStreak);

    Serial.print("Refill No-Rise Streak   : ");
    Serial.println(systemState.refillNoRiseStreak);

    printBool(
    "Safety Lock",
    systemState.currentMode ==
    SAFETY_LOCK);


    Serial.print("Min PH          : ");
    Serial.println(systemState.minPH);

    Serial.print("Max PH          : ");
    Serial.println(systemState.maxPH);

    Serial.print("Min EC          : ");
    Serial.println(systemState.minEC);

    Serial.print("Light ON        : ");
    Serial.print(systemState.lightOnHour);
    Serial.print(":");
    Serial.println(systemState.lightOnMinute);

    Serial.print("Light OFF       : ");
    Serial.print(systemState.lightOffHour);
    Serial.print(":");
    Serial.println(systemState.lightOffMinute);

    Serial.print("RTC Time        : ");

    Serial.print(
        rtcManager.getHour());

    Serial.print(":");

    Serial.print(
        rtcManager.getMinute());

    Serial.print(":");

    Serial.println(
        rtcManager.getSecond());

        printSeparator();
    }

void DebugManager::printAlerts()
{
    printHeader("ALERT STATUS");

    printBool(
        "Low Water",
        alertState.lowWater);

    printBool(
        "EC Low",
        alertState.ecLow);

    printBool(
        "pH Out Of Range",
        alertState.phOutOfRange);

    printBool(
        "Water Temp OOR",
        alertState.waterTempOutOfRange);

    printBool(
        "High Air Temp",
        alertState.highTemperature);

    printBool(
        "Sensor Fault",
        alertState.sensorFault);

    printSeparator();
}

void DebugManager::printRTC()
{
    printHeader("RTC STATUS");

    Serial.print("Current Time : ");

    if(rtcManager.getHour() < 10)
        Serial.print("0");

    Serial.print(rtcManager.getHour());

    Serial.print(":");

    if(rtcManager.getMinute() < 10)
        Serial.print("0");

    Serial.print(rtcManager.getMinute());

    Serial.print(":");

    if(rtcManager.getSecond() < 10)
        Serial.print("0");

    Serial.println(rtcManager.getSecond());

    Serial.print("Light ON     : ");

    Serial.print(systemState.lightOnHour);

    Serial.print(":");

    Serial.println(systemState.lightOnMinute);

    Serial.print("Light OFF    : ");

    Serial.print(systemState.lightOffHour);

    Serial.print(":");

    Serial.println(systemState.lightOffMinute);

    printSeparator();
}

// ======================================================
// Serial Monitor Focus Mode
// ======================================================
// See DebugManager.h's own comments. All three methods read
// systemState.automationTestSubsystem fresh on every call - no cached/
// compile-time state - so they track a live mode change immediately.

bool DebugManager::shouldPrintDebug(DebugCategory category) const
{
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
    {
        // Serial Diagnostics / Observability pass (NORMAL cleanup): during
        // ordinary full-system operation (no isolated bench test), these
        // categories are raw/internal-sampling or development-oriented
        // detail - HC-SR04/water-level internals, DHT raw reads, pH/EC
        // ADC/stability sampling, startup-phase timers, and Canopy Fan's own
        // decision trace - not the new high-level NORMAL instrumentation
        // (STATUS/AUTO/ACT/CIRC/COOL/FOG/SAFE/CMD/PERF/NET), which is gated
        // directly by atLeast() at its own call sites and never routed
        // through this function. Every other category
        // (NETWORK/GSM/NOTIFICATION/LIGHT/SYSTEM) is unaffected.
        //
        // This branch alone is skipped entirely once a subsystem IS
        // isolated (below): a developer who explicitly isolated e.g.
        // COOLING for bench testing still sees its own diagnostics
        // regardless of log level - that isolation is itself the request
        // for detail, independent of LEVEL_NORMAL/VERBOSE.
        switch (category)
        {
            case DebugCategory::WATER:
            case DebugCategory::PH:
            case DebugCategory::EC:
            case DebugCategory::COOLING:
            case DebugCategory::DHT:
            case DebugCategory::STARTUP:
            case DebugCategory::CANOPY:
                return atLeast(LogLevel::LEVEL_VERBOSE);
            default:
                return true;
        }
    }

    switch (systemState.automationTestSubsystem)
    {
        case AutomationTestSubsystem::STARTUP:
            // Startup's own phase/timer diagnostics, plus water depth - the
            // pre-startup refill decision and accepted waterLevelCm are
            // explicitly in scope even though REFILL is not the isolated
            // controller. DHT deliberately excluded: startup fogging does
            // not consume it (SafetyManager::canFog()).
            return category == DebugCategory::STARTUP ||
                   category == DebugCategory::WATER;

        case AutomationTestSubsystem::REFILL:
            return category == DebugCategory::WATER;

        case AutomationTestSubsystem::PH:
            return category == DebugCategory::PH;

        case AutomationTestSubsystem::EC:
            return category == DebugCategory::EC;

        case AutomationTestSubsystem::COOLING:
            return category == DebugCategory::COOLING;

        case AutomationTestSubsystem::FOGGING:
            // DHT is optional for fogging (cadence selection only) but its
            // availability/stale status is explicitly requested - the raw
            // read diagnostics are already throttled to one line per 5s
            // (DHT_RAW_DIAGNOSTIC_INTERVAL_MS), not per-pH/EC-sample spam.
            return category == DebugCategory::FOGGING ||
                   category == DebugCategory::DHT;

        case AutomationTestSubsystem::CANOPY:
            return category == DebugCategory::CANOPY ||
                   category == DebugCategory::DHT;

        case AutomationTestSubsystem::GROW_LIGHT:
            return category == DebugCategory::LIGHT;

        default:
            return false;
    }
}

bool DebugManager::shouldPrintActuator(Actuator actuator) const
{
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
        return true;

    switch (systemState.automationTestSubsystem)
    {
        case AutomationTestSubsystem::STARTUP:
            return actuator == FOGGER || actuator == BLOWER || actuator == SOLENOID;

        case AutomationTestSubsystem::REFILL:
            return actuator == SOLENOID;

        case AutomationTestSubsystem::PH:
            return actuator == PH_UP_PUMP || actuator == PH_DOWN_PUMP ||
                   actuator == CIRCULATION_PUMP;

        case AutomationTestSubsystem::EC:
            // SOLENOID included - EC dilution actuates it (see
            // AutomationManager::handleDosingEC()'s EC_DILUTE branch).
            return actuator == GROW_PUMP || actuator == BLOOM_PUMP ||
                   actuator == CIRCULATION_PUMP || actuator == SOLENOID;

        case AutomationTestSubsystem::COOLING:
            return actuator == PELTIER || actuator == CIRCULATION_PUMP;

        case AutomationTestSubsystem::FOGGING:
            return actuator == FOGGER || actuator == BLOWER;

        case AutomationTestSubsystem::CANOPY:
            return actuator == CANOPY_FAN;

        case AutomationTestSubsystem::GROW_LIGHT:
            return actuator == GROW_LIGHT;

        default:
            return false;
    }
}

bool DebugManager::shouldPrintStateTransition(SystemMode fromMode, SystemMode toMode) const
{
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
        return true;

    // Always-critical / always-common, regardless of which controller (if
    // any) is isolated: a safety lock is a system-wide event by definition,
    // and NORMAL/SENSOR_STABILIZATION are the shared resting/boot states
    // every controller transitions through.
    if (fromMode == SAFETY_LOCK || toMode == SAFETY_LOCK ||
        fromMode == SENSOR_STABILIZATION || toMode == SENSOR_STABILIZATION ||
        toMode == NORMAL)
        return true;

    switch (systemState.automationTestSubsystem)
    {
        case AutomationTestSubsystem::STARTUP:
            return fromMode == STARTUP || toMode == STARTUP;

        case AutomationTestSubsystem::REFILL:
            return fromMode == REFILLING || toMode == REFILLING;

        case AutomationTestSubsystem::PH:
            return fromMode == DOSING_PH || toMode == DOSING_PH ||
                   fromMode == STABILIZING_PH || toMode == STABILIZING_PH;

        case AutomationTestSubsystem::EC:
            return fromMode == DOSING_EC || toMode == DOSING_EC ||
                   fromMode == STABILIZING_EC || toMode == STABILIZING_EC;

        default:
            // COOLING/FOGGING/CANOPY/GROW_LIGHT have no dedicated SystemMode
            // of their own - they run continuously inside NORMAL (already
            // covered by the toMode == NORMAL rule above), so there is no
            // additional state transition to show for them.
            return false;
    }
}

