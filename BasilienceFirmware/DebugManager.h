#ifndef DEBUG_MANAGER_H
#define DEBUG_MANAGER_H

#include <Types.h>
#include <Arduino.h>

// Serial Diagnostics / Observability pass. Purely output-side: nothing here
// decides any control/automation behavior, exactly like Serial Monitor Focus
// Mode below it. LEVEL_ERROR is a near-silent "faults only" floor,
// LEVEL_NORMAL (the default - what farm/data-gathering testing uses) adds
// the 5s STATUS snapshot, state transitions, safety blocks, network
// transitions, commands, performance stalls and fault/recovery events, and
// LEVEL_VERBOSE additionally restores the pre-existing page-cycling
// dashboard (printSystemStatus() etc.) and other raw/deep diagnostics that
// already existed before this pass.
enum class LogLevel : uint8_t
{
    LEVEL_ERROR = 0,
    LEVEL_NORMAL = 1,
    LEVEL_VERBOSE = 2
};

// Small set of independently rate-limited "why is this subsystem blocked"
// channels (section 11 of the observability pass) - one slot per subsystem
// that has its own automatic decision gate, so a persistent block condition
// on one subsystem never suppresses or is confused with another's.
enum class SafetyLogChannel : uint8_t
{
    PH = 0,
    EC,
    FOG,
    COOL,
    REFILL,
    COUNT
};

class DebugManager
{
public:
    void begin();

    void update();

    // ---- Serial Diagnostics / Observability pass ----
    // Runtime level, defaulting to NORMAL - see LogLevel's own comment.
    // Changed only via the Serial "LOG_ERROR"/"LOG_NORMAL"/"LOG_VERBOSE"
    // commands handled by checkTestSmsCommand()'s existing line parser.
    LogLevel getLogLevel() const { return logLevel; }
    bool atLeast(LogLevel minLevel) const { return logLevel >= minLevel; }

    // Prints "[HH:MM:SS][CAT] " (RTC valid) or "[000123456ms][CAT] " (RTC
    // invalid/unavailable) with NO trailing newline or space - the caller
    // finishes the line with ordinary Serial.print/println calls, matching
    // this codebase's chained-Serial.print style rather than a second
    // string-building convention. Always available regardless of level -
    // callers gate on atLeast()/getLogLevel() first.
    void printLogPrefix(const char* category) const;

    // One shared, rate-limited "subsystem blocked" line per channel (reason
    // changed, condition cleared, or REPEAT_INTERVAL elapsed - never spammed
    // every tick a still-blocked condition is re-evaluated). An empty reason
    // clears the channel's remembered state so the next real block reprints
    // immediately rather than waiting out the repeat interval.
    void logSafetyBlock(SafetyLogChannel channel, const char* subsystem, const String& reason);

    // Loop-gap (PERF) tracking. recordLoopTick() is called exactly once, at
    // the very top of loop(), before any other work - see
    // BasilienceFirmware.ino. Immediately prints a LOOP STALL warning when a
    // single gap is abnormally long (see the .cpp for the threshold's own
    // reasoning); otherwise only feeds the last/max values the 5s STATUS
    // line reports.
    void recordLoopTick();

    // Firebase call duration diagnostics (existing logFirebaseDuration()
    // callers in FirebaseManager.cpp are unchanged - this replaces its body).
    // Always shows an operation at/above SLOW_FIREBASE_OPERATION_MS (2000ms);
    // shows every operation's duration only at LEVEL_VERBOSE.
    void logFirebaseDuration(const char* operation, unsigned long durationMs) const;

    // Section 14: one small boot-configuration summary. Called once from
    // setup(), after every local manager's own begin() (so RTC/cache/settings
    // state actually reflects this boot) but before Wi-Fi/Firebase start.
    void printBootSummary();

    // Serial Monitor Focus Mode (see the automation resilience diagnostics
    // follow-up report). All three respond dynamically to the CURRENTLY
    // selected systemState.automationTestSubsystem every call - never a
    // compile-time flag - and always return true when it is NONE, so normal
    // full-system logging is unaffected. None of these decide any
    // control/automation behavior; they only decide what reaches Serial.
    //
    // shouldPrintDebug: is this diagnostic CATEGORY relevant to the
    // currently isolated controller (or is no controller isolated)?
    bool shouldPrintDebug(DebugCategory category) const;

    // shouldPrintActuator: is THIS actuator's state-transition log relevant
    // to the currently isolated controller?
    bool shouldPrintActuator(Actuator actuator) const;

    // shouldPrintStateTransition: is a state change between these two
    // SystemModes relevant to the currently isolated controller? SAFETY_LOCK
    // and the common NORMAL/SENSOR_STABILIZATION resting states always pass
    // through, regardless of which controller (if any) is isolated - see the
    // implementation's own comment.
    bool shouldPrintStateTransition(SystemMode fromMode, SystemMode toMode) const;

private:

    unsigned long lastPrintTime;

    uint8_t currentPage;

    // Minimal local/offline TEST_SMS trigger - see checkTestSmsCommand()'s
    // own comment in DebugManager.cpp. Bounded, single-line accumulator;
    // never grows past a few characters. Also now the input path for the
    // LOG_ERROR/LOG_NORMAL/LOG_VERBOSE level commands (same accumulator,
    // same reasoning - one minimal Serial input hook, not a second console).
    String serialLineBuffer;
    void checkTestSmsCommand();

    // ---- Serial Diagnostics / Observability pass ----
    LogLevel logLevel = LogLevel::LEVEL_NORMAL;

    // 5-second STATUS snapshot (section 4). Independent of DEBUG_ENABLED/
    // currentPage above - those gate the older page-cycling dashboard, now
    // demoted to LEVEL_VERBOSE only (see update()'s own comment).
    unsigned long lastStatusPrintAt = 0;
    void printStatusSnapshot();

    // Loop-gap (PERF) tracking - see recordLoopTick()'s header comment.
    unsigned long lastLoopTickAt = 0;
    unsigned long lastLoopGapMs = 0;
    unsigned long maxLoopGapSinceStatusMs = 0;

    // logSafetyBlock()'s per-channel remembered state.
    struct SafetyBlockState
    {
        String lastReason;
        unsigned long lastPrintedAt = 0;
    };
    SafetyBlockState safetyBlockState[static_cast<uint8_t>(SafetyLogChannel::COUNT)];

    void printSensors();
    void printActuators();
    void printHeader(const char* title);

    void printFloat(
        const char* label,
        float value,
        const char* unit,
        uint8_t decimals);

    void printInteger(
        const char* label,
        int value,
        const char* unit);

    void printBool(
        const char* label,
        bool value);

    void printSeparator();

    void printSystemStatus();

    void printAlerts();
    void printRTC();

    const char* getModeName(
    SystemMode mode);


};


#endif