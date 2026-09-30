#include "FirebaseManager.h"
#include "Globals.h"
#include "Arduino.h"
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <time.h>

namespace
{

constexpr unsigned long COMMAND_READ_INTERVAL  = 1500;
constexpr unsigned long COMMAND_FAILURE_BACKOFF_INTERVAL = 5000;
constexpr unsigned long MOCK_READ_INTERVAL     = 2000;
constexpr unsigned long SETTINGS_READ_INTERVAL = 60000;
constexpr unsigned long UPLOAD_INTERVAL        = 10000;
// Dedicated sensor-telemetry cadence (quick-response refinement task) -
// deliberately separate from UPLOAD_INTERVAL above, which is still used
// unchanged by writeStatus()/writeTelemetry() (status/telemetry are not
// latency-sensitive the way live pH/EC/water/temperature display is).
// Audited: UPLOAD_INTERVAL's only OTHER use tied to the sensor heartbeat
// itself (the isSensorUploadDue() gate and the debug/physicalSensors
// diagnostic's own "is the main heartbeat current" check) now uses this
// constant instead, so both track the same real cadence; every other
// UPLOAD_INTERVAL use is untouched.
//
// Uniform sensor snapshot task: this is now specifically the APP/CLOUD live
// snapshot interval, set to 5s per that task's explicit requirement -
// superseding the earlier as-fast-as-possible quick-response goal for this
// one cadence. Every value writeSensors() publishes already lands in one
// FirebaseJson object under one atomic writeJson() call with one shared
// "timestamp" field, so widening this interval is the only change needed
// to turn that into a real 5-second grouped snapshot. Deliberately NOT
// touching: each physical sensor's own read timer (DHT/DS18B20/water-level
// stay on their existing 5000ms cadence, EC/pH keep their continuous ADC
// sampling - Config.h), writeActuators() (fully independent, event-driven
// on isStatusDirty(), never gated by this constant), or anything
// AutomationManager reads directly from sensors/physicalSensors every
// loop() tick - none of those go through this path.
constexpr unsigned long SENSOR_UPLOAD_INTERVAL_MS = 5000;
constexpr unsigned long DEVICE_INFO_INTERVAL   = 15000;
constexpr unsigned long REALTIME_FALLBACK_INTERVAL = 60000;
constexpr unsigned long HEARTBEAT_SUCCESS_LOG_INTERVAL_MS = 60000;
constexpr unsigned long SENSOR_TEST_TIMEOUT_MS = 10UL * 60UL * 1000UL;
// Manual Mode can stay on for minutes at a time - this bounds how long the
// low-priority cloud-maintenance jobs (telemetry, device info, diagnostic
// sensors, SMS recipients, harvest schedule, notification/fogging ACK
// replay) can be deferred in a row while it's active, so queues/telemetry
// still get occasional service instead of going dark for the whole session.
constexpr unsigned long MANUAL_MODE_LOW_PRIORITY_GRACE_MS = 20000;
// Tighter window around an actually-observed fresh manual command: an
// expensive low-priority job must not START within this many ms of one,
// regardless of how long the broader MANUAL_MODE_LOW_PRIORITY_GRACE_MS
// window still has left to run - see update().
constexpr unsigned long MANUAL_COMMAND_ACTIVITY_WINDOW_MS = 5000;

// Consecutive transport-level failures before Firebase health leaves
// DEGRADED and enters COOLDOWN (no Firebase network calls at all).
constexpr uint8_t TRANSPORT_FAILURE_COOLDOWN_THRESHOLD = 3;
constexpr unsigned long COOLDOWN_INITIAL_MS = 15000UL;
constexpr unsigned long COOLDOWN_MAX_MS = 60000UL;

// First-connection retry cadence. A failed PREFLIGHT costs the main loop
// nothing (it runs in a background task), so it may retry sooner than a failed
// authentication attempt, which does call into the Firebase library. Both
// escalate on the same doubling schedule and share COOLDOWN_MAX_MS as the cap,
// and an attempt that reached the library never retries faster than
// COOLDOWN_INITIAL_MS.
constexpr unsigned long CLOUD_STARTUP_RETRY_INITIAL_MS = 5000UL;
// The background preflight bounds its own TCP/TLS work (two handshakes of at
// most 5s each, see below) but its DNS lookups are bounded only by lwIP's
// resolver retries, so this is generous. The main task only stops waiting for it
// at this point (the task itself is then left to finish and is reclaimed before
// another one starts), and waiting costs the main loop nothing.
constexpr unsigned long PREFLIGHT_DEADLINE_MS = 30000UL;
// How recent a passing preflight must be to vouch for an operation that opens
// a NEW connection at a moment of the library's choosing: a token refresh, or a
// cooldown recovery. Ordinary RTDB traffic does not need this - it rides on
// cloudPathVerified, which the first transport failure withdraws.
constexpr unsigned long CLOUD_PATH_FRESH_MS = 30000UL;
// The preflight probe uses EXACTLY the timeouts the patched Firebase library
// applies to its own connections (FirebaseManager.h checks the macros exist).
// A probe that was more patient than the library would pass on a slow link the
// library then gives up on, and the device would sit in a revoke/re-verify loop
// with a path that "looks fine". setHandshakeTimeout() takes whole seconds.
constexpr uint32_t PREFLIGHT_TLS_CONNECT_TIMEOUT_MS = BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS;
constexpr uint32_t PREFLIGHT_TLS_HANDSHAKE_TIMEOUT_S =
    (BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS + 999UL) / 1000UL;
// Post-auth initialization: pause before a failed step is retried. Doubles on
// each consecutive failure up to COOLDOWN_MAX_MS and resets on success, so a
// step the cloud keeps rejecting costs the main loop one short call per
// interval rather than one per tick.
constexpr unsigned long CLOUD_INIT_STEP_RETRY_MS = 2000UL;
// Cached-locally, low-priority background refreshes (SMS recipients) -
// within the task's suggested 60-120s range.
constexpr unsigned long LOW_PRIORITY_READ_INTERVAL_MS = 90000UL;
// The harvestSchedule projection carries the active-cycle flag that gates all
// cultivation automation, so a change to it is a control-state transition, not
// reporting metadata: an Admin creating or completing a cycle must not wait a
// low-priority rotation for the device to react. The payload is five small
// fields, and this cadence still only applies while Firebase is HEALTHY - the
// existing DEGRADED/COOLDOWN deferral is untouched.
constexpr unsigned long HARVEST_SCHEDULE_READ_INTERVAL_MS = 5000UL;
// Mirrors COMMAND_FAILURE_BACKOFF_INTERVAL's pattern, applied to the
// actuatorStatus cloud mirror so a failed write cannot retry on the very
// next loop() tick regardless of the broader health state.
constexpr unsigned long ACTUATOR_SYNC_FAILURE_BACKOFF_MS = 5000UL;
// A cloud outage at least this long (matches the app's "Offline" threshold)
// means any command found on reconnect was written while the device could not
// act on it and may be arbitrarily old - see currentCommandBaselined in
// FirebaseManager.h.
constexpr unsigned long COMMAND_REBASELINE_OUTAGE_MS = 30000UL;

// STA self-heal: a cloud outage this long with Wi-Fi still associated forces
// one ordinary saved-network reconnect; forced reconnects are at least
// STA_RECOVERY_COOLDOWN_MS apart. See checkStaRecovery().
constexpr unsigned long STA_RECOVERY_OUTAGE_MS = 30000UL;
constexpr unsigned long STA_RECOVERY_COOLDOWN_MS = 60000UL;
constexpr unsigned long DEV_COMMAND_CLEAR_RETRY_INTERVAL_MS = 5000UL;

//==================================================
// Firebase Operation Conversions
//==================================================

OperationType toOperationType(const String& value)
{
    if(value == "REFILL")
        return OperationType::REFILL;

    if(value == "PH_UP")
        return OperationType::PH_UP;

    if(value == "PH_DOWN")
        return OperationType::PH_DOWN;

    if(value == "EC_CORRECTION")
        return OperationType::EC_CORRECTION;

    if(value == "RESET_SAFETY")
        return OperationType::RESET_SAFETY;

    return OperationType::NONE;
}

OperationAction toOperationAction(const String& value)
{
    if(value == "START")
        return OperationAction::START;

    if(value == "STOP")
        return OperationAction::STOP;

    if(value == "ENABLE")
        return OperationAction::ENABLE;

    if(value == "DISABLE")
        return OperationAction::DISABLE;

    if(value == "EXECUTE")
        return OperationAction::EXECUTE;

    return OperationAction::NONE;
}

RequestState toRequestState(const String& value)
{
    if(value == "PENDING")
        return RequestState::PENDING;

    if(value == "ACCEPTED")
        return RequestState::ACCEPTED;

    if(value == "RUNNING")
        return RequestState::RUNNING;

    if(value == "COMPLETED")
        return RequestState::COMPLETED;

    if(value == "REJECTED")
        return RequestState::REJECTED;

    if(value == "FAILED")
        return RequestState::FAILED;

    return RequestState::IDLE;
}

RequestSource toRequestSource(const String& value)
{
    if(value == "MANUAL")
        return RequestSource::MANUAL;

    if(value == "AUTOMATIC")
        return RequestSource::AUTOMATIC;

    return RequestSource::NONE;
}

//==================================================
// Firebase Operation Serialization
//==================================================

const char* operationToString(OperationType operation)
{
    switch(operation)
    {
        case OperationType::REFILL:
            return "REFILL";

        case OperationType::PH_UP:
            return "PH_UP";

        case OperationType::PH_DOWN:
            return "PH_DOWN";

        case OperationType::EC_CORRECTION:
            return "EC_CORRECTION";

        case OperationType::RESET_SAFETY:
            return "RESET_SAFETY";

        default:
            return "NONE";
    }
}

const char* actionToString(OperationAction action)
{
    switch(action)
    {
        case OperationAction::START:
            return "START";

        case OperationAction::STOP:
            return "STOP";

        case OperationAction::ENABLE:
            return "ENABLE";

        case OperationAction::DISABLE:
            return "DISABLE";

        case OperationAction::EXECUTE:
            return "EXECUTE";

        default:
            return "NONE";
    }
}

const char* requestStateToString(RequestState state)
{
    switch(state)
    {
        case RequestState::PENDING:
            return "PENDING";

        case RequestState::ACCEPTED:
            return "ACCEPTED";

        case RequestState::RUNNING:
            return "RUNNING";

        case RequestState::COMPLETED:
            return "COMPLETED";

        case RequestState::REJECTED:
            return "REJECTED";

        case RequestState::FAILED:
            return "FAILED";

        default:
            return "IDLE";
    }
}

bool floatValuesDiffer(float left, float right)
{
    if(isnan(left) || isnan(right))
        return !(isnan(left) && isnan(right));

    return fabsf(left - right) > 0.001f;
}

} // namespace


//==================================================
// Initialization
//==================================================

void FirebaseManager::loadPersistedSettings()
{
    if (!preferences.begin("automation", true)) return;
    // Serial Diagnostics / Observability pass: read-only record of which
    // branch was taken, purely for the boot summary line ("Settings source:
    // NVS / defaults") - does not change which values are loaded below.
    settingsRestoredFromNvs = preferences.getBool("valid", false);
    if (preferences.getBool("valid", false))
    {
        systemState.lightOnHour = preferences.getUChar("lightOnH", systemState.lightOnHour);
        systemState.lightOnMinute = preferences.getUChar("lightOnM", systemState.lightOnMinute);
        systemState.lightOffHour = preferences.getUChar("lightOffH", systemState.lightOffHour);
        systemState.lightOffMinute = preferences.getUChar("lightOffM", systemState.lightOffMinute);
        systemState.minPH = preferences.getFloat("minPH", systemState.minPH);
        systemState.maxPH = preferences.getFloat("maxPH", systemState.maxPH);
        systemState.phTargetMin = preferences.getFloat("phTargetMin", systemState.phTargetMin);
        systemState.phTargetMax = preferences.getFloat("phTargetMax", systemState.phTargetMax);
        systemState.minEC = preferences.getFloat("minEC", systemState.minEC);
        systemState.maxEC = preferences.getFloat("maxEC", systemState.maxEC);
        systemState.ecTargetMin = preferences.getFloat("ecTargetMin", systemState.ecTargetMin);
        systemState.ecTargetMax = preferences.getFloat("ecTargetMax", systemState.ecTargetMax);
        systemState.refillStartLevel = preferences.getFloat("refillStart", systemState.refillStartLevel);
        systemState.refillStopLevel = preferences.getFloat("refillStop", systemState.refillStopLevel);
        systemState.refillStartLevelCm = preferences.getFloat("refillStartCm", systemState.refillStartLevelCm);
        systemState.refillStopLevelCm = preferences.getFloat("refillStopCm", systemState.refillStopLevelCm);
        systemState.criticalLowWaterCm = preferences.getFloat("critLowCm", systemState.criticalLowWaterCm);
        systemState.waterLevelEmptyDistanceCm = preferences.getFloat("wlEmptyCm", systemState.waterLevelEmptyDistanceCm);
        systemState.waterLevelFullDistanceCm = preferences.getFloat("wlFullCm", systemState.waterLevelFullDistanceCm);
        // Own NVS key, deliberately not derived from "wlEmptyCm" above - see
        // Types.h's sensorToBottomCm comment.
        systemState.sensorToBottomCm = preferences.getFloat("sensorBottomCm", systemState.sensorToBottomCm);
        systemState.highAirTemp = preferences.getFloat("highAir", systemState.highAirTemp);
        systemState.airTempRelease = preferences.getFloat("airRelease", systemState.airTempRelease);
        systemState.highHumidity = preferences.getFloat("highHumidity", systemState.highHumidity);
        systemState.humidityRelease = preferences.getFloat("humidityRel", systemState.humidityRelease);
        systemState.highWaterTemp = preferences.getFloat("highWater", systemState.highWaterTemp);
        systemState.coolerOffTemp = preferences.getFloat("coolerOff", systemState.coolerOffTemp);
        // Loaded BEFORE Firebase becomes available on this boot, per this
        // task's own requirement - falls back to the compiled default
        // (BLOWER_SPEED_DEFAULT_PERCENT) only if this key was never
        // persisted (first boot, or before this feature existed).
        systemState.blowerSpeedPercent = preferences.getUChar("blowerSpeed", systemState.blowerSpeedPercent);
        Serial.println("[SETTINGS] Restored persisted automation settings");
    }

    // Config/settings schema migration (see CONFIG_SCHEMA_VERSION in
    // Config.h) - runs regardless of the "valid" guard above, since even a
    // device that has never persisted anything else still needs its own
    // cfgVersion baseline established. An already-deployed device may have
    // just restored (or, for maxAirTemp, may be about to pull from
    // Firebase in readSettings() - it has no NVS entry of its own) the OLD
    // stale compiled default (maxAirTemp=28, blowerSpeedPercent=30) from
    // before this schema version - a stored value is indistinguishable at
    // the value level alone from a genuine admin choice, which is exactly
    // why this is gated by a persisted one-time version rather than a
    // "does it equal the old default" heuristic: once migrated, a future
    // admin setting either field back to today's old numbers is never
    // touched again. Corrected LOCALLY here unconditionally (cheap, safe,
    // works fully offline) - the Firebase side of this migration (pushing
    // the correction so it survives the next settings pull, and
    // persisting cfgVersion only once that push actually succeeds)
    // happens in readSettings() via systemState.configMigrationPending,
    // since it needs connectivity and this function must stay
    // offline-safe.
    if (preferences.getUChar("cfgVersion", 0) < CONFIG_SCHEMA_VERSION)
    {
        systemState.maxAirTemp = TARGET_MAX_AIR_TEMP;
        systemState.blowerSpeedPercent = BLOWER_SPEED_DEFAULT_PERCENT;
        systemState.configMigrationPending = true;
        Serial.println("[SETTINGS] Config migration pending: maxAirTemp->32C, blowerSpeedPercent->65% (corrected locally; Firebase reconciliation pending connectivity)");
    }

    preferences.end();
}

void FirebaseManager::persistSettings()
{
    if (!preferences.begin("automation", false)) return;
    preferences.putUChar("lightOnH", systemState.lightOnHour);
    preferences.putUChar("lightOnM", systemState.lightOnMinute);
    preferences.putUChar("lightOffH", systemState.lightOffHour);
    preferences.putUChar("lightOffM", systemState.lightOffMinute);
    preferences.putFloat("minPH", systemState.minPH);
    preferences.putFloat("maxPH", systemState.maxPH);
    preferences.putFloat("phTargetMin", systemState.phTargetMin);
    preferences.putFloat("phTargetMax", systemState.phTargetMax);
    preferences.putFloat("minEC", systemState.minEC);
    preferences.putFloat("maxEC", systemState.maxEC);
    preferences.putFloat("ecTargetMin", systemState.ecTargetMin);
    preferences.putFloat("ecTargetMax", systemState.ecTargetMax);
    preferences.putFloat("refillStart", systemState.refillStartLevel);
    preferences.putFloat("refillStop", systemState.refillStopLevel);
    preferences.putFloat("refillStartCm", systemState.refillStartLevelCm);
    preferences.putFloat("refillStopCm", systemState.refillStopLevelCm);
    preferences.putFloat("critLowCm", systemState.criticalLowWaterCm);
    preferences.putFloat("wlEmptyCm", systemState.waterLevelEmptyDistanceCm);
    preferences.putFloat("wlFullCm", systemState.waterLevelFullDistanceCm);
    preferences.putFloat("sensorBottomCm", systemState.sensorToBottomCm);
    preferences.putFloat("highAir", systemState.highAirTemp);
    preferences.putFloat("airRelease", systemState.airTempRelease);
    preferences.putFloat("highHumidity", systemState.highHumidity);
    preferences.putFloat("humidityRel", systemState.humidityRelease);
    preferences.putFloat("highWater", systemState.highWaterTemp);
    preferences.putFloat("coolerOff", systemState.coolerOffTemp);
    // Only ever reached via readSettings() after a value already passed
    // validation there, or via the compiled default already in
    // systemState - never writes an unvalidated Firebase value.
    preferences.putUChar("blowerSpeed", systemState.blowerSpeedPercent);
    preferences.putBool("valid", true);
    preferences.end();
}

void FirebaseManager::begin()
{
    if (wifiManager.consumeFirebaseResumePending())
    {
        Serial.println("[FIREBASE] Resuming after Wi-Fi reconnect");
    }


    config.api_key = API_KEY;

    config.database_url = DATABASE_URL;

    // What bounds a Firebase call's time on the main loop, traced against
    // Firebase Arduino Client Library v4.4.17 and ESP32 core 3.0.7. Each phase
    // has its own control, and only one of them is a FirebaseConfig setting:
    //
    //  - Server response: config.timeout.serverResponse (ms) IS honoured.
    //    FirebaseCore::reconnect() compares millis() against it on every
    //    wait-for-response loop, for both the RTDB client and the auth client.
    //    A silent server ends a request ~4s after the last byte.
    //  - TCP connect: BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS, set in the
    //    vendored library's WiFiClientImpl.h (patch 1/3, see BASILIENCE_PATCH.md
    //    in the library folder). Upstream is 30000ms and no setting reaches it.
    //  - TLS handshake: BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS, set in the
    //    vendored library's BSSL_SSL_Client.h (patches 2/3 and 3/3). Upstream is
    //    60000ms and the library never applies config.timeout.sslHandshake.
    //  - DNS: lwIP's own resolver retries (about 7s per configured DNS server).
    //    Nothing in the library or this firmware bounds it.
    //
    // Both patched values are the same for every connection the library opens,
    // RTDB and authentication/token refresh alike, because both create their
    // socket and TLS engine from the same two classes. FirebaseManager.h refuses
    // to compile against a copy of the library that lacks the patch.
    //
    // config.timeout.socketConnection is deliberately NOT set. Its documented
    // unit is milliseconds (1000-60000, default 10000), but FirebaseData::
    // setTimeout() hands it to a setter that multiplies by 1000, so any legal
    // value means 1000+ SECONDS - and only for the RTDB client, only when its
    // session is rebuilt, and the SSL engine resets it to 15000ms at the start
    // of every TLS connect anyway. It bounds nothing in any configuration, so
    // leaving it at the library default is the honest choice. (Previously set to
    // 4000 on the mistaken belief that it capped TCP connect.)
    config.timeout.serverResponse = 4000;

    loadDeviceId();
    loadActuatorCommandTimestamps();

    // ROOT CAUSE of the observed reconnect loop: true here lets the Firebase
    // client library independently call WiFi.reconnect() from inside
    // FirebaseCore::resumeNetwork() whenever ITS OWN networkReady() check
    // happens to read a momentary non-CONNECTED status during any RTDB
    // call - and WiFi.reconnect() (STAClass::reconnect() in the ESP32
    // core) unconditionally calls esp_wifi_disconnect() first if still
    // associated, forcibly dropping a connection that may not have
    // actually failed. That's a second, uncoordinated reconnect owner
    // fighting WiFiManager's own state machine, which already guarantees
    // exactly one association attempt in flight - it's why the same DHCP
    // lease kept getting reacquired with no "[WIFI] Connecting to..." log
    // line from WiFiManager: the library was reconnecting the radio
    // itself, outside WiFiManager entirely. false makes WiFiManager the
    // sole owner of Wi-Fi reconnection; the library now only observes
    // connectivity (failing/degrading Firebase operations when Wi-Fi is
    // actually down) instead of acting on it. reconnectWiFi() is
    // deprecated in this library version in favor of reconnectNetwork(),
    // used here instead. Same fix applied at the other call site in
    // beginFirebaseRecovery(). Set before the first authentication attempt
    // now (it used to follow a blocking one) so the library never gets the
    // chance to reconnect the radio itself during startup either.
    Firebase.reconnectNetwork(false);

    Serial.print("Loaded Device ID: [");
    Serial.print(deviceId);
    Serial.println("]");

    // If Wi-Fi was already down when update() first ran, update() prints this
    // same line itself on the DOWN->UP edge; printing it here too would
    // report one connection twice.
    if (wasWifiConnectedForLog)
    {
        logWifiConnectedDiagnostics();
    }

    // Arm the non-blocking first connection and return. Nothing below this
    // point in the boot sequence waits on the network any more: update()
    // drives PREFLIGHT -> AUTHENTICATING -> INIT_STEPS -> COMPLETE one bounded
    // step per loop() iteration (see the CloudStartupPhase comment), so local
    // sensing, safety, automation and actuator control keep their normal
    // cadence for as long as the cloud stays unreachable. Cloud readiness is
    // declared in completeCloudStartup(), never here.
    systemState.firebaseConnected = false;
    cloudPathVerified = false;
    cloudSessionValidated = false;
    cloudPathVerifiedAt = 0;
    cloudStartupPhase = CloudStartupPhase::PREFLIGHT;
    cloudInitStep = CloudInitStep::RESOLVE_DEVICE_ID;
    cloudStartupNextAttemptAt = 0;
    cloudStartupBackoffMs = 0;
    cloudStartupAttempt = 0;
    cloudInitRetryAt = 0;
    cloudInitRetryMs = 0;
    sensorTestBootClearDone = false;
    preflightRunning = false;
    authPhase = FirebaseAuthPhase::IDLE;

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.println("Firebase startup armed | local automation continues");
    }
}

//==================================================
// Non-blocking cloud startup
//==================================================

// One [NET] line per Wi-Fi association edge, never per tick.
void FirebaseManager::logWifiConnectedDiagnostics()
{
    if (!debugManager.atLeast(LogLevel::LEVEL_NORMAL)) return;

    debugManager.printLogPrefix("NET");
    Serial.print("WiFi CONNECTED | IP=");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI=");
    Serial.print(WiFi.RSSI());
    Serial.print("dBm Gateway=");
    Serial.print(WiFi.gatewayIP());
    Serial.print(" DNS=");
    Serial.print(WiFi.dnsIP(0));
    const IPAddress dns2 = WiFi.dnsIP(1);
    if (dns2 != IPAddress((uint32_t)0))
    {
        Serial.print(",");
        Serial.print(dns2);
    }
    Serial.println();
}

// Printed at the start of each connection attempt (attempts are at least
// CLOUD_STARTUP_RETRY_INITIAL_MS apart and back off from there). Both clocks
// are shown because they are independent: nothing in this firmware copies the
// DS3231 into the ESP32 system clock. That has no effect on TLS here - see
// evaluatePreflightResult() and the startup report - but it is the first thing
// to rule out when a connection fails, so it is always on record.
void FirebaseManager::logSystemTimeDiagnostics()
{
    if (!debugManager.atLeast(LogLevel::LEVEL_NORMAL)) return;

    const time_t systemEpoch = time(nullptr);
    const bool systemClockSet = systemEpoch > (time_t)FIREBASE_DEFAULT_TS;

    debugManager.printLogPrefix("NET");
    Serial.print("System time=");
    if (systemClockSet)
    {
        struct tm utc;
        gmtime_r(&systemEpoch, &utc);
        char stamp[24];
        snprintf(stamp, sizeof(stamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                 utc.tm_hour, utc.tm_min, utc.tm_sec);
        Serial.print(stamp);
    }
    else
    {
        Serial.print("UNSET");
    }
    Serial.print(" (epoch=");
    Serial.print((unsigned long)systemEpoch);
    Serial.print(") | RTC=");
    Serial.println(rtcManager.hasValidTime() ? "VALID" : "INVALID");
}

// Escalating gate on when the next preflight may start: 5s, 10s, 20s, 40s, then
// 60s. A failure that never reached the Firebase library (a failed preflight)
// may retry from CLOUD_STARTUP_RETRY_INITIAL_MS. One that did (authentication,
// a failed token refresh) waits at least COOLDOWN_INITIAL_MS, since each
// library attempt can occupy the main loop for a full transport timeout. The
// level is reset by any successful Firebase call and by Wi-Fi loss.
void FirebaseManager::bumpCloudRetryBackoff(bool libraryContacted, const char* reason)
{
    unsigned long next = (cloudStartupBackoffMs == 0)
        ? CLOUD_STARTUP_RETRY_INITIAL_MS
        : min(cloudStartupBackoffMs * 2, COOLDOWN_MAX_MS);
    if (libraryContacted && next < COOLDOWN_INITIAL_MS)
    {
        next = COOLDOWN_INITIAL_MS;
    }

    cloudStartupBackoffMs = next;
    cloudStartupNextAttemptAt = millis() + next;
    if (cloudStartupNextAttemptAt == 0) cloudStartupNextAttemptAt = 1;

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.print("Firebase retry in ");
        Serial.print(next / 1000UL);
        Serial.print("s | ");
        Serial.println(reason);
    }
}

// Startup only: the authentication attempt reached the library and failed.
void FirebaseManager::scheduleCloudStartupRetry(bool libraryContacted, const char* reason)
{
    cloudPathVerified = false;
    bumpCloudRetryBackoff(libraryContacted, reason);

    // Whatever this attempt left behind is discarded: the next one starts from
    // a fresh preflight, and startAuthAttempt() re-reads the persisted
    // credentials. NVS credentials are never touched here.
    cloudStartupPhase = CloudStartupPhase::PREFLIGHT;
    preflightRunning = false;
    authPhase = FirebaseAuthPhase::IDLE;
}

// Withdraws permission to call the Firebase library until a fresh preflight
// passes. Idempotent within one outage: only the call that actually flips
// cloudPathVerified counts against the backoff, so several failures inside one
// update() tick are one event.
//
// The FIRST revoke after a healthy period lets the next preflight start at
// once (a preflight costs the main loop nothing, and a single dropped request
// should not park the cloud for seconds). A revoke that follows a preflight
// which passed but was then contradicted by the very next library call is the
// "path looks fine, calls keep failing" pattern, and each repeat waits longer.
void FirebaseManager::revokeCloudPath(const char* reason, bool libraryContacted)
{
    if (!cloudPathVerified) return;
    cloudPathVerified = false;

    unsigned long delayMs = cloudStartupBackoffMs;
    if (libraryContacted && delayMs < COOLDOWN_INITIAL_MS)
    {
        delayMs = COOLDOWN_INITIAL_MS;
    }

    const unsigned long now = millis();
    cloudStartupNextAttemptAt = (delayMs == 0) ? 0 : now + delayMs;
    if (delayMs != 0 && cloudStartupNextAttemptAt == 0) cloudStartupNextAttemptAt = 1;
    cloudStartupBackoffMs = (cloudStartupBackoffMs == 0)
        ? CLOUD_STARTUP_RETRY_INITIAL_MS
        : min(cloudStartupBackoffMs * 2, COOLDOWN_MAX_MS);

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.print("Firebase path unverified | ");
        Serial.print(reason);
        Serial.print(" | next check in ");
        Serial.print(delayMs / 1000UL);
        Serial.println("s");
    }
}

bool FirebaseManager::cloudPathFresh() const
{
    return cloudPathVerified && (millis() - cloudPathVerifiedAt) <= CLOUD_PATH_FRESH_MS;
}

// Wi-Fi was lost (or the setup AP took the radio). Nothing learned about the
// path so far is trusted once the link comes back, and the outage was the
// network's, not the cloud's, so there is no backoff penalty: the next
// preflight starts as soon as the link is up. An unfinished first connection
// goes back to PREFLIGHT. A post-auth INIT_STEPS attempt keeps its progress
// (its steps resume), but is held off by cloudPathVerified like everything else.
void FirebaseManager::noteNetworkLost()
{
    noteCloudUnavailable();
    cloudPathVerified = false;
    preflightRunning = false;
    cloudStartupNextAttemptAt = 0;
    cloudStartupBackoffMs = 0;
    abortCloudStartupAttempt();
}

void FirebaseManager::abortCloudStartupAttempt()
{
    if (cloudStartupPhase != CloudStartupPhase::PREFLIGHT &&
        cloudStartupPhase != CloudStartupPhase::AUTHENTICATING)
    {
        return;
    }

    cloudStartupPhase = CloudStartupPhase::PREFLIGHT;
    preflightRunning = false;
    cloudPathVerified = false;
    cloudStartupNextAttemptAt = 0;
    cloudStartupBackoffMs = 0;
    authPhase = FirebaseAuthPhase::IDLE;
}

void FirebaseManager::noteProvisioningActive()
{
    // loop() skips update() while the setup AP owns the radio, so update()'s own
    // Wi-Fi-lost handling never runs for that outage. Do it here, including
    // starting the outage clock that decides the 30s command re-baseline.
    noteNetworkLost();
}

// One step of the background preflight, used by both the first connection and
// every later re-verification. Returns PASSED only after the task has finished
// and released its TLS clients, so nothing that follows can overlap it.
FirebaseManager::PreflightPoll FirebaseManager::pollPreflight()
{
    const unsigned long now = millis();

    if (!preflightRunning)
    {
        if (cloudStartupNextAttemptAt != 0 &&
            (long)(now - cloudStartupNextAttemptAt) < 0)
        {
            return PreflightPoll::PENDING;
        }

        cloudStartupAttempt++;
        logSystemTimeDiagnostics();

        if (!startPreflight())
        {
            if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
            {
                debugManager.printLogPrefix("NET");
                Serial.println("Firebase connection deferred | preflight unavailable (previous check still finishing, or out of memory)");
            }
            bumpCloudRetryBackoff(false, "preflight could not start");
            return PreflightPoll::FAILED;
        }
        return PreflightPoll::PENDING;
    }

    if (xSemaphoreTake(preflightDoneSemaphore, 0) == pdTRUE)
    {
        preflightTaskActive = false;
        preflightRunning = false;

        if (!evaluatePreflightResult())
        {
            bumpCloudRetryBackoff(false, "network not usable yet");
            return PreflightPoll::FAILED;
        }
        return PreflightPoll::PASSED;
    }

    // Stop waiting, but leave the task flagged active: startPreflight()
    // refuses to launch another until this one has signalled, so two never
    // write the shared result fields at once.
    if (now - preflightStartedAt >= PREFLIGHT_DEADLINE_MS)
    {
        preflightRunning = false;
        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.println("Firebase connection deferred | preflight timed out (DNS/TLS not answering)");
        }
        bumpCloudRetryBackoff(false, "preflight timed out");
        return PreflightPoll::FAILED;
    }
    return PreflightPoll::PENDING;
}

// While cloudPathVerified is false, this is the ONLY thing update() does about
// the cloud. It makes no Firebase library call.
void FirebaseManager::advanceRuntimePreflight()
{
    if (pollPreflight() != PreflightPoll::PASSED)
    {
        return;
    }

    cloudPathVerified = true;
    cloudPathVerifiedAt = millis();

    // A passing preflight proves the NETWORK path only. The library's own
    // session has not been shown to work yet: update() runs
    // validateCloudSession() before anything else is called or reported.
    cloudSessionValidated = false;

    // Any session that predates the failure or outage is dead or stale. Close
    // it now, so the first call afterwards opens a fresh connection on the path
    // that was just proven instead of writing into a half-open socket.
    fbdo.stopWiFiClient();
    fbdo.clear();

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.println("Firebase path verified | validating Firebase session");
    }
}

// Runs once per update() tick (never with any other Firebase call in the same
// tick) while the network path is verified but the library session is not yet
// proven. The operation is a single tiny read of /commands/mockSensors/enabled:
//  - Read only: it writes nothing, consumes no command, changes no setting and
//    starts no actuator. The value that comes back is deliberately discarded.
//  - It is a path this firmware already reads every few seconds
//    (readMockSensors()), so the database rules are known to allow it.
//  - It is a fresh library request on the session that advanceRuntimePreflight()
//    just closed, so it exercises exactly what has been failing: opening the
//    library's own TCP/TLS connection and completing an HTTP round trip.
// A missing node ("path not exist") or a value of another type is an answer
// from the server and proves the session just as well as a bool would. Anything
// else (timeouts, SSL errors, permission or authentication errors) does not.
void FirebaseManager::validateCloudSession()
{
    const unsigned long startedAt = millis();
    const bool read = Firebase.RTDB.getBool(
        &fbdo, deviceRoot() + "/commands/mockSensors/enabled");
    const unsigned long durationMs = millis() - startedAt;
    logFirebaseDuration("Session validation", durationMs);

    bool answered = read;
    String reason;
    if (!read)
    {
        reason = fbdo.errorReason();
        String lower = reason;
        lower.toLowerCase();
        answered = lower.indexOf("path not exist") >= 0 ||
                   lower.indexOf("data type mismatch") >= 0;
    }

    if (answered)
    {
        logSocketDiagnostics("successful-recovery");
        // The first real success of this recovery: this is where the health
        // streak clears (DEGRADED -> HEALTHY) and the retry backoff resets.
        recordFirebaseResult(true);
        cloudSessionValidated = true;

        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.print("Firebase session validated | ");
            Serial.print(durationMs);
            Serial.println("ms");
        }
        return;
    }

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.print("Firebase session validation failed | ");
        Serial.println(reason.isEmpty() ? String("no reason reported") : reason);
    }

    // The preflight passed and the library then contradicted it, at the cost
    // of a real (possibly multi-second) library call: withdraw the path first
    // with the library-contacted delay (at least COOLDOWN_INITIAL_MS, doubling
    // while it keeps happening), then do the health accounting. Its own revoke
    // is then a no-op, and the three-strikes COOLDOWN/recovery still applies.
    revokeCloudPath("session validation failed", true);
    recordFirebaseResult(false);
}

// Drives PREFLIGHT and AUTHENTICATING. Called once per update() tick while
// either is active; every call does a bounded amount of work on the main loop
// task and returns. The Firebase library is not touched at all until the
// preflight has passed.
void FirebaseManager::advanceCloudStartupConnect()
{
    if (cloudStartupPhase == CloudStartupPhase::PREFLIGHT)
    {
        if (pollPreflight() != PreflightPoll::PASSED)
        {
            return;
        }

        cloudPathVerified = true;
        cloudPathVerifiedAt = millis();
        cloudSessionValidated = false;

        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.print("Firebase auth attempt started | attempt #");
            Serial.println(cloudStartupAttempt);
        }
        bootstrapHttpResultCode = 0;
        startAuthAttempt();
        cloudStartupPhase = CloudStartupPhase::AUTHENTICATING;
        return;
    }

    // AUTHENTICATING: exactly the state machine runtime recovery uses.
    if (!pollAuthStateMachine())
    {
        return;
    }

    if (authPhase == FirebaseAuthPhase::SUCCESS)
    {
        if (!authSucceededViaLegacy)
        {
            Serial.println("[FIREBASE-AUTH] Secure device identity active");
            Serial.print("[FIREBASE-AUTH] uid=");
            Serial.println(deviceId);
        }
        else
        {
            Serial.println("[SECURITY] Legacy Firebase auth compatibility mode active");
        }

        authPhase = FirebaseAuthPhase::IDLE;
        cloudStartupPhase = CloudStartupPhase::INIT_STEPS;
        cloudInitStep = CloudInitStep::RESOLVE_DEVICE_ID;
        cloudInitRetryAt = 0;
        cloudInitRetryMs = 0;
        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.println("Firebase auth OK | initializing database");
        }
        return;
    }

    // FAILED. The preflight passed, so this is not simply "no network": say
    // what was actually observed instead of guessing.
    if (SECURE_DEVICE_AUTH_REQUIRED)
    {
        Serial.println("[FIREBASE-AUTH] Secure auth unavailable this attempt (no device secret provisioned, or bootstrap failed)");
        Serial.println("[FIREBASE-AUTH] Legacy anonymous auth is disabled (SECURE_DEVICE_AUTH_REQUIRED=true) - Firebase connectivity unavailable");
    }
    else
    {
        Serial.println("[FIREBASE-AUTH] All available authentication methods failed this attempt");
    }

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.print("Firebase auth failed | preflight passed, sign-in did not complete");
        if (bootstrapHttpResultCode != 0)
        {
            Serial.print(" | bootstrap HTTP=");
            Serial.print(bootstrapHttpResultCode);
            if (bootstrapHttpResultCode < 0) Serial.print(" (connect/TLS error)");
        }
        Serial.println();
    }

    // The library's own auth client may be holding a half-open session.
    fbdo.stopWiFiClient();
    fbdo.clear();

    scheduleCloudStartupRetry(true, "authentication failed");
}

// Launches the background preflight. Main task only. Returns false when a
// previous preflight task has not signalled yet (it is still running and owns
// the result fields) or a resource could not be allocated.
bool FirebaseManager::startPreflight()
{
    if (preflightDoneSemaphore == nullptr)
    {
        preflightDoneSemaphore = xSemaphoreCreateBinary();
        if (preflightDoneSemaphore == nullptr)
        {
            Serial.println("[NET] Unable to allocate preflight semaphore");
            return false;
        }
    }

    if (preflightTaskActive)
    {
        // A previous, abandoned task: reclaim it only once it has really
        // finished.
        if (xSemaphoreTake(preflightDoneSemaphore, 0) != pdTRUE)
        {
            return false;
        }
        preflightTaskActive = false;
    }
    xSemaphoreTake(preflightDoneSemaphore, 0);

    // Never hold two TLS contexts at once. An abandoned bootstrap task (one
    // WAIT_BOOTSTRAP_HTTP gave up on, or one whose auth attempt was cut short
    // by an outage) can still be inside its handshake, and a preflight
    // handshake needs tens of KB of its own. Reclaim it exactly the way
    // pollAuthStateMachine()'s drain does, and only once it has really
    // signalled. Nothing polls the auth state machine while a preflight is
    // being (re)started - update() is either in PREFLIGHT or holding for one -
    // so a leftover WAIT_BOOTSTRAP_HTTP is an attempt that was interrupted:
    // drop it to IDLE (a recovery in that state then reports "failed" and goes
    // back to cooldown) so the drain rule below can apply.
    if (bootstrapHttpTaskActive)
    {
        if (authPhase == FirebaseAuthPhase::WAIT_BOOTSTRAP_HTTP)
        {
            authPhase = FirebaseAuthPhase::IDLE;
        }
        if (bootstrapHttpDoneSemaphore == nullptr ||
            xSemaphoreTake(bootstrapHttpDoneSemaphore, 0) != pdTRUE)
        {
            return false;
        }
        bootstrapHttpTaskActive = false;
        bootstrapHttpResultToken = "";
        bootstrapHttpResultSuccess = false;
        bootstrapHttpResultDeviceId = "";
    }

    // The hosts the Firebase library will actually contact: "securetoken" is
    // the refresh-token grant and the first host TLS is proven against;
    // "identitytoolkit" is anonymous sign-up and custom-token exchange; the
    // RTDB host comes from DATABASE_URL and is the second host TLS is proven
    // against, because the database can be blocked while the auth host is not.
    // The bootstrap endpoint is only relevant to a device holding a
    // provisioned secret.
    uint8_t count = 0;
    preflightHosts[count++] = "securetoken.googleapis.com";
    preflightHosts[count++] = "identitytoolkit.googleapis.com";

    String rtdbHost = DATABASE_URL;
    const int schemeEnd = rtdbHost.indexOf("://");
    if (schemeEnd >= 0) rtdbHost = rtdbHost.substring(schemeEnd + 3);
    const int pathStart = rtdbHost.indexOf('/');
    if (pathStart >= 0) rtdbHost = rtdbHost.substring(0, pathStart);
    preflightHosts[count++] = rtdbHost;

    loadDeviceAuthCredentials();
    if (deviceAuthSecret.length() > 0)
    {
        String bootstrapHost = BOOTSTRAP_ENDPOINT_URL;
        const int bootstrapSchemeEnd = bootstrapHost.indexOf("://");
        if (bootstrapSchemeEnd >= 0) bootstrapHost = bootstrapHost.substring(bootstrapSchemeEnd + 3);
        const int bootstrapPathStart = bootstrapHost.indexOf('/');
        if (bootstrapPathStart >= 0) bootstrapHost = bootstrapHost.substring(0, bootstrapPathStart);
        preflightHosts[count++] = bootstrapHost;
    }
    preflightHostCount = count;
    logAuthDiagnostics("preflight");

    for (uint8_t i = 0; i < PREFLIGHT_HOST_COUNT; i++)
    {
        preflightDns[i] = -1;
        preflightDnsMs[i] = 0;
        preflightDnsIp[i][0] = '\0';
    }
    for (uint8_t i = 0; i < PREFLIGHT_TLS_TARGETS; i++)
    {
        preflightTls[i] = -1;
        preflightTlsError[i] = 0;
        preflightTlsMs[i] = 0;
    }

    // Same core as the caller, exactly as bootstrapSecureAuth() does, so the
    // two are always time-sliced and never truly concurrent.
    const BaseType_t targetCore = xPortGetCoreID();
    TaskHandle_t createdHandle = nullptr;
    // 12KB: the TLS handshake below runs on this task's stack (same floor the
    // bootstrap HTTPS task uses).
    const BaseType_t created = xTaskCreatePinnedToCore(
        &FirebaseManager::preflightTaskFn,
        "fbPreflight",
        12288,
        this,
        1,
        &createdHandle,
        targetCore);

    if (created != pdPASS || createdHandle == nullptr)
    {
        Serial.println("[NET] Unable to start preflight task");
        return false;
    }

    preflightTaskActive = true;
    preflightRunning = true;
    preflightStartedAt = millis();
    return true;
}

// Runs entirely off the main loop task. Touches only its own locals and the
// preflight* result fields, and writes those once, right before signalling.
// Never calls into the Firebase library and never prints (all [NET] output
// comes from evaluatePreflightResult() on the main task, in order).
void FirebaseManager::preflightTaskFn(void* arg)
{
    FirebaseManager* self = static_cast<FirebaseManager*>(arg);

    // Every object with a destructor lives inside this scope on purpose.
    // vTaskDelete() at the bottom never returns, so it never unwinds the
    // function: an object declared at function scope would have its destructor
    // skipped and its heap leaked on every single run. Closing the scope first
    // runs them all, the TLS client included, before the task is deleted.
    {
        const uint8_t count = self->preflightHostCount;
        String hosts[PREFLIGHT_HOST_COUNT];
        for (uint8_t i = 0; i < count; i++) hosts[i] = self->preflightHosts[i];

        int8_t dns[PREFLIGHT_HOST_COUNT];
        uint32_t dnsMs[PREFLIGHT_HOST_COUNT] = { 0 };
        char ips[PREFLIGHT_HOST_COUNT][40];
        for (uint8_t i = 0; i < PREFLIGHT_HOST_COUNT; i++)
        {
            dns[i] = -1;
            ips[i][0] = '\0';
        }

        // DNS for every host the library will need. The first failure stops the
        // rest: with a dead resolver each lookup would otherwise cost its full
        // resolver timeout in turn, and the answer is already known.
        bool dnsFailed = false;
        for (uint8_t i = 0; i < count && !dnsFailed; i++)
        {
            IPAddress ip;
            const unsigned long dnsStartedAt = millis();
            const int dnsResult = WiFi.hostByName(hosts[i].c_str(), ip);
            dnsMs[i] = millis() - dnsStartedAt;
            if (dnsResult == 1)
            {
                dns[i] = 1;
                strncpy(ips[i], ip.toString().c_str(), sizeof(ips[i]) - 1);
                ips[i][sizeof(ips[i]) - 1] = '\0';
            }
            else
            {
                dns[i] = 0;
                dnsFailed = true;
            }
        }

        // Real TLS handshakes, the same kind of connection the library is about
        // to make, with no data sent and no authentication: first the auth host
        // (hosts[0]), then the database host (hosts[2]) - the database can be
        // blocked while the auth host is not, and the reverse. The second is
        // only tried once the first has worked, since a path that cannot
        // complete one handshake will not complete the next and each failure
        // costs its full timeout. Insecure mode: this is a reachability test,
        // so it deliberately depends on neither the system clock nor a CA
        // bundle. Each handshake is bounded by the two timeouts set below, and
        // each client is a separate object from any the Firebase library owns,
        // created and destroyed inside this loop body.
        const uint8_t tlsHostIndex[PREFLIGHT_TLS_TARGETS] = { 0, 2 };
        int8_t tls[PREFLIGHT_TLS_TARGETS] = { -1, -1 };
        int tlsError[PREFLIGHT_TLS_TARGETS] = { 0, 0 };
        uint32_t tlsMs[PREFLIGHT_TLS_TARGETS] = { 0, 0 };

        const uint32_t heapBefore = ESP.getFreeHeap();
        const uint32_t maxBlockBefore = ESP.getMaxAllocHeap();

        for (uint8_t t = 0; t < PREFLIGHT_TLS_TARGETS; t++)
        {
            const uint8_t hostIndex = tlsHostIndex[t];
            if (hostIndex >= count || dns[hostIndex] != 1) break;
            if (t > 0 && tls[t - 1] != 1) break;

            NetworkClientSecure probe;
            probe.setInsecure();
            probe.setHandshakeTimeout(PREFLIGHT_TLS_HANDSHAKE_TIMEOUT_S);

            const unsigned long startedAt = millis();
            const int connected = probe.connect(hosts[hostIndex].c_str(), 443, PREFLIGHT_TLS_CONNECT_TIMEOUT_MS);
            tlsMs[t] = millis() - startedAt;
            if (connected)
            {
                tls[t] = 1;
            }
            else
            {
                tls[t] = 0;
                char errText[64];
                tlsError[t] = probe.lastError(errText, sizeof(errText));
            }
            probe.stop();
        }

        for (uint8_t i = 0; i < PREFLIGHT_HOST_COUNT; i++)
        {
            self->preflightDns[i] = dns[i];
            self->preflightDnsMs[i] = dnsMs[i];
            memcpy(self->preflightDnsIp[i], ips[i], sizeof(ips[i]));
        }
        for (uint8_t t = 0; t < PREFLIGHT_TLS_TARGETS; t++)
        {
            self->preflightTls[t] = tls[t];
            self->preflightTlsError[t] = tlsError[t];
            self->preflightTlsMs[t] = tlsMs[t];
        }
        self->preflightHeapBefore = heapBefore;
        self->preflightMaxBlockBefore = maxBlockBefore;
        // Read after the last probe has been stopped and destroyed, so this is
        // the heap as the handshakes leave it (it should equal heapBefore, give
        // or take small allocator noise: any lasting difference is a leak).
        self->preflightFreeHeap = ESP.getFreeHeap();
        self->preflightMaxBlock = ESP.getMaxAllocHeap();
    }

    xSemaphoreGive(self->preflightDoneSemaphore);
    vTaskDelete(nullptr);
}

// Prints the preflight outcome and decides whether the Firebase library may be
// contacted. DNS for the auth/RTDB hosts and a completed TLS handshake to BOTH
// the auth host and the database host are hard requirements. DNS for the
// bootstrap endpoint is reported but does not gate:
// if it is the only thing failing, the bootstrap task fails fast on its own
// background task and the state machine falls through to the next credential.
bool FirebaseManager::evaluatePreflightResult()
{
    const bool verbose = debugManager.atLeast(LogLevel::LEVEL_NORMAL);
    // Hosts [0..2] are the auth/RTDB hosts (always present); [3], when
    // present, is the bootstrap endpoint.
    constexpr uint8_t GATING_HOST_COUNT = 3;

    bool gatingDnsOk = true;
    const char* failedHost = nullptr;

    for (uint8_t i = 0; i < preflightHostCount; i++)
    {
        if (verbose)
        {
            debugManager.printLogPrefix("NET-DIAG");
            Serial.print(" DNS host=");
            Serial.print(preflightHosts[i]);
            Serial.print(" result=");
            if (preflightDns[i] == 1) Serial.print(preflightDnsIp[i]);
            else if (preflightDns[i] == 0) Serial.print("FAIL");
            else Serial.print("NOT_RUN");
            Serial.print(" elapsed=");
            Serial.print(preflightDnsMs[i]);
            Serial.println("ms");
        }
        if (preflightDns[i] == 0 && i < GATING_HOST_COUNT && gatingDnsOk)
        {
            gatingDnsOk = false;
            failedHost = preflightHosts[i].c_str();
        }
    }

    if (!gatingDnsOk)
    {
        if (verbose)
        {
            debugManager.printLogPrefix("NET");
            Serial.print("Firebase connection deferred | DNS failure (");
            Serial.print(failedHost);
            Serial.print(") | WiFi=UP gateway=");
            Serial.print(WiFi.gatewayIP());
            Serial.print(" dns=");
            Serial.println(WiFi.dnsIP(0));
        }
        return false;
    }

    // Both TLS targets must have completed a handshake. [0] is the auth host,
    // [1] the database host (preflightHosts[2]); a target that was not tried
    // (because the one before it failed) is reported by that earlier failure.
    static const uint8_t tlsHostIndex[PREFLIGHT_TLS_TARGETS] = { 0, 2 };
    bool tlsOk = true;
    int firstTlsError = 0;
    const char* firstTlsHost = nullptr;

    for (uint8_t t = 0; t < PREFLIGHT_TLS_TARGETS; t++)
    {
        if (preflightTls[t] == -1)
        {
            tlsOk = false;
            continue;
        }

        const bool ok = preflightTls[t] == 1;
        if (verbose)
        {
            debugManager.printLogPrefix("NET-DIAG");
            Serial.print(" TLS host=");
            Serial.print(preflightHosts[tlsHostIndex[t]]);
            Serial.print(" result=");
            Serial.print(ok ? "OK" : "FAIL");
            Serial.print(" elapsed=");
            Serial.print(preflightTlsMs[t]);
            Serial.print("ms client_error=");
            Serial.print(preflightTlsError[t]);
            Serial.println(" tcp_stage=not_exposed_by_NetworkClientSecure");
        }
        if (!ok && tlsOk)
        {
            firstTlsError = preflightTlsError[t];
            firstTlsHost = preflightHosts[tlsHostIndex[t]].c_str();
        }
        if (!ok) tlsOk = false;
    }

    if (verbose)
    {
        // Heap around the handshakes: "before" is free/largest block just
        // ahead of the first one, "after" is the same once the last one has
        // been torn down. They should match, and a failure with a small
        // "before" points at memory rather than the network.
        debugManager.printLogPrefix("NET");
        Serial.print("heap before=");
        Serial.print(preflightHeapBefore);
        Serial.print("/");
        Serial.print(preflightMaxBlockBefore);
        Serial.print(" after=");
        Serial.print(preflightFreeHeap);
        Serial.print("/");
        Serial.println(preflightMaxBlock);
    }

    if (tlsOk)
    {
        return true;
    }

    if (verbose)
    {
        debugManager.printLogPrefix("NET");
        Serial.print("Firebase connection deferred | TLS/connect error=");
        Serial.print(firstTlsError);
        if (firstTlsHost != nullptr)
        {
            Serial.print(" (");
            Serial.print(firstTlsHost);
            Serial.print(")");
        }
        Serial.println();
    }
    return false;
}

// Post-auth database initialization, advanced one step per update() tick. This
// is everything begin() used to do synchronously after authenticating. Each
// step is at most a handful of RTDB calls, so a slow cloud can cost the main
// loop one call's worth of time per tick, never the whole sequence at once.
void FirebaseManager::advanceCloudInit()
{
    const unsigned long now = millis();
    if (cloudInitRetryAt != 0 && (long)(now - cloudInitRetryAt) < 0)
    {
        return;
    }
    cloudInitRetryAt = 0;

    const CloudInitStep stepBeforeRun = cloudInitStep;
    if (runCloudInitStep())
    {
        cloudInitRetryMs = 0;
        if (cloudInitStep == CloudInitStep::FINISH)
        {
            completeCloudStartup();
        }
        return;
    }

    // Transport failures inside a step already feed recordFirebaseResult(), so
    // three of them in a row put the connection into the existing
    // COOLDOWN/recovery cycle (which sits above this in update()). What is
    // left here is the retry pacing for a step the cloud keeps refusing for
    // any other reason. Authentication is not repeated for that: the session
    // is fine, and repeating an anonymous sign-up would only create accounts.
    cloudInitRetryMs = (cloudInitRetryMs == 0)
        ? CLOUD_INIT_STEP_RETRY_MS
        : min(cloudInitRetryMs * 2, COOLDOWN_MAX_MS);
    cloudInitRetryAt = millis() + cloudInitRetryMs;
    if (cloudInitRetryAt == 0) cloudInitRetryAt = 1;

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        // Same order as CloudInitStep.
        static const char* const stepNames[] = {
            "resolve-device-id", "prime-commands", "seed-status", "seed-settings",
            "seed-commands", "seed-operations", "clear-sensor-test",
            "clear-dev-flags", "prime-commands-final", "read-settings", "finish"
        };
        debugManager.printLogPrefix("NET");
        Serial.print("Firebase init step failed | step=");
        Serial.print(stepNames[static_cast<uint8_t>(stepBeforeRun)]);
        Serial.print(" | retry in ");
        Serial.print(cloudInitRetryMs / 1000UL);
        Serial.println("s");
    }
}

// Executes the current step. Returns true when the step is done (and advances
// cloudInitStep); false when it must be retried. Must not be entered unless
// Firebase.ready() was true this tick - update() guarantees that.
bool FirebaseManager::runCloudInitStep()
{
    switch (cloudInitStep)
    {
        case CloudInitStep::RESOLVE_DEVICE_ID:
            if (deviceId.isEmpty() && !provisionDevice())
            {
                return false;
            }
            cloudInitStep = CloudInitStep::PRIME_COMMANDS_EARLY;
            return true;

        case CloudInitStep::PRIME_COMMANDS_EARLY:
            // Establish the existing RTDB actuator-command snapshot as a consumed
            // baseline before publishing this boot's first heartbeat. Commands
            // written while the device was offline must never execute as fresh
            // hardware requests. Best effort here: a brand-new device has no
            // /commands node until SEED_COMMANDS_CURRENT creates it, and
            // PRIME_COMMANDS_FINAL below does not let startup complete until the
            // baseline really exists.
            if (!actuatorCommandsPrimed)
            {
                primeActuatorCommands();
            }
            cloudInitStep = CloudInitStep::SEED_STATUS;
            return true;

        case CloudInitStep::SEED_STATUS:
            seedStatusNode();
            cloudInitStep = CloudInitStep::SEED_SETTINGS;
            return true;

        case CloudInitStep::SEED_SETTINGS:
            if (!seedSettingsNode()) return false;
            cloudInitStep = CloudInitStep::SEED_COMMANDS_CURRENT;
            return true;

        case CloudInitStep::SEED_COMMANDS_CURRENT:
            if (!seedCommandsCurrentNode()) return false;
            cloudInitStep = CloudInitStep::SEED_OPERATIONS_CURRENT;
            return true;

        case CloudInitStep::SEED_OPERATIONS_CURRENT:
            if (!seedOperationsCurrentNode()) return false;
            cloudInitStep = CloudInitStep::CLEAR_SENSOR_TEST;
            return true;

        case CloudInitStep::CLEAR_SENSOR_TEST:
        {
            // Diagnostic mode is deliberately non-persistent. A reboot always
            // clears both the retained command and its acknowledgement before
            // normal control.
            if (!sensorTestBootClearDone)
            {
                systemState.sensorTestEnabled = false;
                systemState.sensorTestStartTime = 0;

                // Second write only after the first succeeded: on a dead link
                // that is one blocked call per tick instead of two.
                const bool commandCleared = Firebase.RTDB.setBool(
                    &fbdo, deviceRoot() + "/commands/sensorTest/enabled", false);
                recordFirebaseResult(commandCleared);
                if (!commandCleared)
                {
                    return false;
                }
                const bool statusCleared = Firebase.RTDB.setBool(
                    &fbdo, deviceRoot() + "/status/sensorTest", false);
                recordFirebaseResult(statusCleared);
                if (!statusCleared)
                {
                    return false;
                }
                sensorTestBootClearDone = true;
            }
            cloudInitStep = CloudInitStep::CLEAR_DEV_FLAGS;
            return true;
        }

        case CloudInitStep::CLEAR_DEV_FLAGS:
            // Same non-persistence rule for the other developer/test flags: they
            // are level flags left sitting in RTDB, so without this a stale
            // automationTestMode / ignoreWaterLevelAutomation / mockSensors value
            // would re-arm itself on every reboot. Until this succeeds their
            // readers ignore enabled=true (see devCommandsCleared).
            if (!devCommandsCleared)
            {
                clearDevCommandsAtBoot();
                if (!devCommandsCleared) return false;
            }
            cloudInitStep = CloudInitStep::PRIME_COMMANDS_FINAL;
            return true;

        case CloudInitStep::PRIME_COMMANDS_FINAL:
            // A new device may not have had a /commands node during the earlier
            // baseline attempt. SEED_COMMANDS_CURRENT creates the canonical
            // command container, so the baseline must exist before startup can
            // complete.
            if (!actuatorCommandsPrimed)
            {
                primeActuatorCommands();
                if (!actuatorCommandsPrimed) return false;
            }
            cloudInitStep = CloudInitStep::READ_SETTINGS;
            return true;

        case CloudInitStep::READ_SETTINGS:
            readSettings();
            if (!lastSettingsReadOk) return false;
            cloudInitStep = CloudInitStep::FINISH;
            return true;

        case CloudInitStep::FINISH:
            return true;
    }
    return true;
}

// Reached only when every INIT step has succeeded. This - not Wi-Fi being
// connected, and not authentication alone - is what "cloud ready" means.
void FirebaseManager::completeCloudStartup()
{
    // noteCloudAvailable() may have re-armed the reconnect baselines if the
    // link dropped for 30s+ while INIT_STEPS was in progress (see
    // COMMAND_REBASELINE_OUTAGE_MS). Never declare ready over a cleared
    // baseline: go back and redo exactly the steps that were undone.
    if (!devCommandsCleared)
    {
        cloudInitStep = CloudInitStep::CLEAR_DEV_FLAGS;
        return;
    }
    if (!actuatorCommandsPrimed)
    {
        cloudInitStep = CloudInitStep::PRIME_COMMANDS_FINAL;
        return;
    }

    // A transport failure in the last step withdraws the path without failing
    // the step. Never declare ready over a withdrawn path: wait here (the next
    // tick's hold runs the preflight, then this runs again).
    if (!cloudPathVerified)
    {
        return;
    }

    syncRTC();

    systemState.settingsLoaded = true;

    systemState.syncRTC = true;

    cloudStartupPhase = CloudStartupPhase::COMPLETE;
    cloudStartupBackoffMs = 0;
    // The initialization steps that just finished (settings read, seeding,
    // command baselines) were real library operations on this verified path,
    // so the session is already proven: no separate validation read is needed
    // for the first READY of a boot.
    cloudSessionValidated = true;
    // The normal update() path would otherwise announce this same transition
    // a second time as "Firebase RESTORED".
    wasFirebaseConnected = true;
    systemState.firebaseConnected = true;

    Serial.println("Firebase Started");
    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.println("Firebase READY");
    }
}

// Writes enabled=false to each developer/test command node. Idempotent and
// safe to call repeatedly: it does nothing once every write has succeeded. The
// payload fields under each node are left alone - only the level flag the
// device reads is cleared, so the app can simply set it true again on purpose.
void FirebaseManager::clearDevCommandsAtBoot()
{
    lastDevCommandClearAttemptAt = millis();
    if (devCommandsCleared) return;

    // Section 12: retried from update() on failure (see the F2 fix), so this
    // "requested" line can print more than once for the same overall clear -
    // that is the correct, honest picture of a retry actually happening, not
    // noise.
    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("CMD");
        Serial.println("DEV FLAGS CLEAR requested");
    }

    static const char* const clearPaths[] =
    {
        "/commands/automationTestMode/enabled",
        "/commands/ignoreWaterLevelAutomation/enabled",
        "/commands/mockSensors/enabled"
    };

    bool allCleared = true;
    for (const char* path : clearPaths)
    {
        const bool ok = Firebase.RTDB.setBool(&fbdo, deviceRoot() + path, false);
        recordFirebaseResult(ok);
        if (!ok)
        {
            allCleared = false;
            Serial.print("[DEV-CLEAR] failed to clear ");
            Serial.print(path);
            Serial.print(": ");
            Serial.println(fbdo.errorReason());

            // A transport failure means the remaining writes would block for
            // the same timeout each and fail the same way; stop, and let the
            // normal retry redo the whole (idempotent) clear. An
            // application-level refusal on one node still lets the others be
            // cleared, exactly as before.
            if (isTransportFailureReason(fbdo.errorReason()))
            {
                break;
            }
        }
    }

    if (allCleared)
    {
        devCommandsCleared = true;
        Serial.println("[DEV-CLEAR] developer test flags cleared at boot");
        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("CMD");
            Serial.println("DEV FLAGS CLEAR complete");
        }
    }
}

void FirebaseManager::noteCloudUnavailable()
{
    if (cloudUnavailableSince == 0)
    {
        cloudUnavailableSince = millis();
        if (cloudUnavailableSince == 0) cloudUnavailableSince = 1;
    }
}

// Called once the cloud is usable again. A short blip keeps normal command
// handling; a real outage re-baselines both channels so anything written
// while the device was unreachable is consumed as already-handled rather
// than run late. Re-arming actuatorCommandsPrimed reuses the existing
// "consumed as reconnect baseline" pass in readActuatorCommands().
//
// The developer/test flags (automationTestMode/ignoreWaterLevelAutomation/
// mockSensors) get the same treatment: devCommandsCleared being true only
// means "cleared as of the outage that just ended", not "safe forever". A
// value written to one of those nodes while the device was unreachable is
// sitting in RTDB the moment the cloud comes back, and every reader already
// distrusts enabled=true until devCommandsCleared is true - so clearing it
// here forces clearDevCommandsAtBoot() to run again (via its existing
// retry path in update()) before any of the three flags are honoured
// post-reconnect.
void FirebaseManager::noteCloudAvailable()
{
    if (cloudUnavailableSince == 0) return;

    const unsigned long outageMs = millis() - cloudUnavailableSince;
    cloudUnavailableSince = 0;

    if (outageMs < COMMAND_REBASELINE_OUTAGE_MS) return;

    currentCommandBaselined = false;
    actuatorCommandsPrimed = false;

    // CONFIRMED BUG FIX (automation test mode vs reconnect dev-flag clear):
    // devCommandsCleared=false forces clearDevCommandsAtBoot() to re-run,
    // which WRITES enabled=false to /commands/automationTestMode (and the
    // other two dev/test nodes) - correct for its original purpose (a stale
    // enabled=true left over from BEFORE this boot; see
    // applyAutomationTestModeCommand()'s own devCommandsCleared gate), but
    // indistinguishable at the RTDB level from an EXPLICITLY active local
    // test session that was simply riding out a transient outage. Re-arming
    // it unconditionally here made that same clear fire on every qualifying
    // RECONNECT too, not just a boot, silently switching off an active PH/
    // EC/etc. isolated test mid-run merely because Firebase dropped for 30s.
    // Skipped only while a test is actually active locally - the moment it
    // ends (naturally, or because the app genuinely disabled it; the
    // ordinary per-tick command read in applyAutomationTestModeCommand()
    // always reflects whatever is CURRENTLY in RTDB, regardless of this
    // flag, since devCommandsCleared stays true across this outage in that
    // case), the next outage clears exactly as before.
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
    {
        devCommandsCleared = false;
    }

    // Section 5/12: the standard-format pair, replacing the previous single
    // combined [COMMAND] line - same trigger (outageMs >= COMMAND_REBASELINE_
    // OUTAGE_MS), same behavior; only the wording/format changed.
    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("CMD");
        Serial.println("Re-baselining command channels");
        if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
        {
            debugManager.printLogPrefix("CMD");
            Serial.println("Re-clearing developer flags");
        }
    }
}

// Ends the current escalation stage: the associated-outage clock restarts and
// the next suppression may log again. Deliberately leaves lastStaRecoveryAt
// alone - the 60s cooldown between forced reconnects is absolute.
void FirebaseManager::resetStaRecoveryStage()
{
    staRecoveryOutageSince = 0;
    staRecoverySuppressedLogged = false;
}

// Called once per update() tick, only after the Wi-Fi-connected gate (so Wi-Fi
// is associated) and only once the cloud startup has been armed. Reads the
// READY flag as last computed (systemState.firebaseConnected - authenticated,
// initialized AND session-validated); it makes no Firebase call itself and
// changes nothing about how preflight/validation/READY work. If the cloud has
// stayed not-READY continuously for STA_RECOVERY_OUTAGE_MS while associated,
// and the previous forced reconnect was at least STA_RECOVERY_COOLDOWN_MS ago,
// it asks WiFiManager for one ordinary saved-network reconnect. Everything
// after that - link drop, reassociation, fresh preflight, session validation,
// READY - is the existing path, unchanged. Returns true only on the tick a
// reconnect was actually requested.
bool FirebaseManager::checkStaRecovery()
{
    if (systemState.firebaseConnected)
    {
        resetStaRecoveryStage();
        return false;
    }

    const unsigned long now = millis();
    if (staRecoveryOutageSince == 0)
    {
        staRecoveryOutageSince = now;
        if (staRecoveryOutageSince == 0) staRecoveryOutageSince = 1;
        return false;
    }

    if (now - staRecoveryOutageSince < STA_RECOVERY_OUTAGE_MS) return false;

    if (lastStaRecoveryAt != 0 && now - lastStaRecoveryAt < STA_RECOVERY_COOLDOWN_MS)
    {
        if (!staRecoverySuppressedLogged && debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.println("STA recovery reconnect suppressed | cooldown");
        }
        staRecoverySuppressedLogged = true;
        return false;
    }

    if (!wifiManager.requestReconnect()) return false;

    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.println("Cloud outage >=30s while WiFi associated -> forcing STA reconnect");
    }

    lastStaRecoveryAt = now;
    if (lastStaRecoveryAt == 0) lastStaRecoveryAt = 1;
    resetStaRecoveryStage();
    return true;
}

// The database seeding that used to be one initializeDatabase() call is split
// into four independent steps so runCloudInitStep() can run one per update()
// tick. Their bodies are otherwise unchanged, with one deliberate difference:
// each read that decides "does this node exist yet" now retries instead of
// seeding when the read failed for a transport reason. Seeding on any failed
// read used to be safe only because begin() ran when the link was known good;
// with the connection now established in the background, a dropped read must
// not be mistaken for a missing node and overwrite the admin's /settings with
// this device's defaults.
void FirebaseManager::seedStatusNode()
{
    Serial.println("Firebase RTDB onDisconnect rules registered.");

    // Presence is backend-owned. Only a successfully received sensor heartbeat
    // may update status/online; Firebase initialization alone is insufficient.
    FirebaseJson connectivityJson;
    connectivityJson.set("provisioning", false);
    updateJson(deviceRoot() + "/status", connectivityJson);
}

bool FirebaseManager::seedSettingsNode()
{
    FirebaseJson json;

    //--------------------------------------------------
    // Settings
    //--------------------------------------------------

    const bool settingsRead = Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/settings");
    recordFirebaseResult(settingsRead);
    if(!settingsRead && isTransportFailureReason(fbdo.errorReason()))
    {
        return false;
    }

    if(!settingsRead)
    {
        json.clear();

        json.set("lightOnHour",
            systemState.lightOnHour);

        json.set("lightOnMinute",
            systemState.lightOnMinute);

        json.set("lightOffHour",
            systemState.lightOffHour);

        json.set("lightOffMinute",
            systemState.lightOffMinute);

        json.set("minPH",
            systemState.minPH);

        json.set("maxPH",
            systemState.maxPH);

        json.set("phTargetMin", systemState.phTargetMin);
        json.set("phTargetMax", systemState.phTargetMax);

        json.set("minEC",
            systemState.minEC);

        json.set("maxEC", systemState.maxEC);
        json.set("ecTargetMin", systemState.ecTargetMin);
        json.set("ecTargetMax", systemState.ecTargetMax);

        json.set("refillStartLevel",
            systemState.refillStartLevel);

        json.set("refillStopLevel",
            systemState.refillStopLevel);

        // Water-depth model (AUTHORITATIVE for control) - see Config.h's
        // "Water Reservoir Geometry" section.
        json.set("refillStartLevelCm",
            systemState.refillStartLevelCm);

        json.set("refillStopLevelCm",
            systemState.refillStopLevelCm);

        json.set("criticalLowWaterCm",
            systemState.criticalLowWaterCm);

        json.set("waterLevelEmptyDistanceCm",
            systemState.waterLevelEmptyDistanceCm);

        json.set("waterLevelFullDistanceCm",
            systemState.waterLevelFullDistanceCm);

        json.set("sensorToBottomCm",
            systemState.sensorToBottomCm);

        json.set("highWaterTemp",
            systemState.highWaterTemp);

        json.set("coolerOffTemp",
            systemState.coolerOffTemp);

        // Only reached when /settings does not exist at all yet (a brand
        // new/never-provisioned device) - never re-seeded on every boot of
        // an already-provisioned one. See BLOWER_SPEED_DEFAULT_PERCENT's
        // own comment for why 50% is a fallback-only value.
        json.set("blowerSpeedPercent",
            systemState.blowerSpeedPercent);

        json.set("highAirTemp",
            systemState.highAirTemp);

        json.set("airTempRelease", systemState.airTempRelease);
        json.set("highHumidity", systemState.highHumidity);
        json.set("humidityRelease", systemState.humidityRelease);

        // Target (acceptable) ranges - what "in range" means for Monitoring,
        // alerts and Reports. Separate from the control thresholds above.
        json.set("minAirTemp", systemState.minAirTemp);
        json.set("maxAirTemp", systemState.maxAirTemp);
        json.set("minHumidity", systemState.minHumidity);
        json.set("maxHumidity", systemState.maxHumidity);
        json.set("minWaterTemp", systemState.minWaterTemp);
        json.set("maxWaterTemp", systemState.maxWaterTemp);
        json.set("minWaterLevel", systemState.minWaterLevel);
        json.set("maxWaterLevel", systemState.maxWaterLevel);

        writeJson(
            deviceRoot() + "/settings",
            json);
    }
    else
    {
        FirebaseJsonData highAirData;
        if (!fbdo.jsonObject().get(highAirData, "highAirTemp") || !highAirData.success)
        {
            FirebaseJson missingSetting;
            missingSetting.set("highAirTemp", systemState.highAirTemp);
            if (updateJson(deviceRoot() + "/settings", missingSetting))
            {
                Serial.println("[SETTINGS] Seeded missing highAirTemp");
            }
            else if (isTransportFailureReason(fbdo.errorReason()))
            {
                // Link gone mid-step: stop here instead of issuing the
                // remaining writes into it. Everything seeded so far is
                // idempotent, so the retried step simply skips it.
                return false;
            }
        }

        // An already-provisioned device predates the target-range fields, so
        // seed any that are absent rather than leaving them unset. Existing
        // values are never overwritten.
        FirebaseJsonData rangeProbe;
        FirebaseJson missingRanges;
        bool seededRange = false;
        const char* rangeKeys[8] = {
            "minAirTemp", "maxAirTemp", "minHumidity", "maxHumidity",
            "minWaterTemp", "maxWaterTemp", "minWaterLevel", "maxWaterLevel"
        };
        const float rangeValues[8] = {
            systemState.minAirTemp, systemState.maxAirTemp,
            systemState.minHumidity, systemState.maxHumidity,
            systemState.minWaterTemp, systemState.maxWaterTemp,
            systemState.minWaterLevel, systemState.maxWaterLevel
        };
        for (uint8_t i = 0; i < 8; i++)
        {
            if (!fbdo.jsonObject().get(rangeProbe, rangeKeys[i]) || !rangeProbe.success)
            {
                missingRanges.set(rangeKeys[i], rangeValues[i]);
                seededRange = true;
            }
        }
        if (seededRange)
        {
            if (updateJson(deviceRoot() + "/settings", missingRanges))
            {
                Serial.println("[SETTINGS] Seeded missing target ranges");
            }
            else if (isTransportFailureReason(fbdo.errorReason()))
            {
                return false;
            }
        }

        FirebaseJsonData settingData;
        FirebaseJson missingSettings;
        bool hasMissingSettings = false;
#define SEED_SETTING(name, value) \
        if (!fbdo.jsonObject().get(settingData, name) || !settingData.success) { \
            missingSettings.set(name, value); \
            hasMissingSettings = true; \
        }
        SEED_SETTING("phTargetMin", systemState.phTargetMin);
        SEED_SETTING("phTargetMax", systemState.phTargetMax);
        SEED_SETTING("maxEC", systemState.maxEC);
        SEED_SETTING("ecTargetMin", systemState.ecTargetMin);
        SEED_SETTING("ecTargetMax", systemState.ecTargetMax);
        SEED_SETTING("airTempRelease", systemState.airTempRelease);
        SEED_SETTING("highHumidity", systemState.highHumidity);
        SEED_SETTING("humidityRelease", systemState.humidityRelease);
#undef SEED_SETTING
        if (hasMissingSettings &&
            !updateJson(deviceRoot() + "/settings", missingSettings) &&
            isTransportFailureReason(fbdo.errorReason()))
        {
            return false;
        }
    }

    return true;
}

bool FirebaseManager::seedCommandsCurrentNode()
{
    FirebaseJson json;

    //--------------------------------------------------
    // Commands
    //--------------------------------------------------

    const bool currentRead = Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/commands/current");
    recordFirebaseResult(currentRead);
    if(!currentRead && isTransportFailureReason(fbdo.errorReason()))
    {
        return false;
    }

    if(!currentRead)
    {
        json.clear();

        json.set("requestId", 0);
        json.set("operation", "NONE");
        json.set("action", "NONE");
        json.set("requestTimestamp", 0);
        json.set("protocolVersion", 1);

        writeJson(
            deviceRoot() + "/commands/current",
            json);
    }

    return true;
}

bool FirebaseManager::seedOperationsCurrentNode()
{
    FirebaseJson json;

    //--------------------------------------------------
    // Current Operation
    //--------------------------------------------------

    const bool operationRead = Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/operations/current");
    recordFirebaseResult(operationRead);
    if(!operationRead && isTransportFailureReason(fbdo.errorReason()))
    {
        return false;
    }

    if(!operationRead)
    {
        json.clear();

        json.set("requestId", 0);
        json.set("operation", "NONE");
        json.set("action", "NONE");
        json.set("state", "IDLE");
        json.set("reason", "");
        json.set("requestTimestamp", 0);
        json.set("acceptedTimestamp", 0);
        json.set("startedTimestamp", 0);
        json.set("completedTimestamp", 0);
        json.set("lastUpdatedTimestamp", 0);
        json.set("protocolVersion", 1);

        writeJson(
            deviceRoot() + "/operations/current",
            json);
    }

    return true;
}

// RTC: there is no seeding step. The removed code used to write this device's
// own (possibly post-power-loss, meaningless) DS3231 reading to
// /devices/{deviceId}/rtc whenever that node was absent - and syncRTC() then
// read that SAME node back and called rtc.adjust() on it. Nothing else in this
// system (no Cloud Function, no Android screen) ever wrote a genuinely
// trustworthy value there, so the whole thing was a circular echo that could
// silently clear the DS3231's lostPower flag on garbage data. RTC status is now
// published read-only to /devices/{deviceId}/status/rtc by writeStatus()
// instead.

//==================================================
// Main Update
//==================================================

void FirebaseManager::update()
{
    cloudCallsThisTick = 0;

    // The diagnostic timeout is local safety state and must remain enforceable
    // even when cloud communication is unavailable.
    enforceSensorTestTimeout();

    // Capture completion-time effective readings before any connectivity guard.
    // Local control may continue normally while this immutable cloud snapshot
    // waits for Firebase to become available.
    captureAutomaticTerminalSnapshot();

    // Automatic pH, EC, and refill requests intentionally retain their existing
    // ACCEPTED -> RUNNING -> actuator-handler lifecycle. Protect the two local
    // control passes after acceptance from low-priority cloud work so that
    // noncritical Firebase latency cannot unnecessarily delay physical startup.
    const bool deferLowPriorityForControlResponse =
        shouldDeferOptionalJobsForControlResponse();

    // This guard must precede Firebase.ready(), heartbeat, actuator/alert
    // publication, and every RTDB call. An automatic request needs two local
    // passes to advance ACCEPTED -> RUNNING -> its actuator handler; allowing
    // even a "high priority" synchronous cloud call here can insert the same
    // multi-second delay before the actuator is physically requested.
    if(deferLowPriorityForControlResponse)
    {
        return;
    }

    //--------------------------------------------------
    // Connection Status
    //--------------------------------------------------

    // This guard must run before Firebase.ready() because that call can initiate
    // SSL/reconnect work and starve the local AP HTTP server.
    if (wifiManager.isProvisioningMode())
    {
        if (!suspendedForProvisioning)
        {
            Serial.println("[FIREBASE] Suspended during provisioning mode");
            suspendedForProvisioning = true;
        }
        systemState.firebaseConnected = false;
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        wasFirebaseConnected = false;
        noteNetworkLost();
        resetStaRecoveryStage();
        return;
    }

    if (!wifiManager.isConnected())
    {
        // Section 5: printed once on the actual UP->DOWN edge, never every
        // tick Wi-Fi stays down.
        if (wasWifiConnectedForLog && debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.println("WiFi LOST");
            debugManager.printLogPrefix("NET");
            Serial.println("Local automation continues");
        }
        wasWifiConnectedForLog = false;

        systemState.firebaseConnected = false;
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        wasFirebaseConnected = false;
        noteNetworkLost();
        // A real Wi-Fi loss (or the forced reconnect's own link drop) is the
        // existing WiFiManager path's to handle; it never counts toward the
        // associated-outage clock.
        resetStaRecoveryStage();
        return;
    }

    // Section 5: printed once on the actual DOWN->UP edge. Carries the same
    // address/gateway/DNS detail as the first connection of the boot, so a
    // later "connected but no cloud" report always shows what the link handed
    // the device.
    if (!wasWifiConnectedForLog)
    {
        logWifiConnectedDiagnostics();
    }
    wasWifiConnectedForLog = true;

    // First-connection gate. Until the cloud has authenticated, the Firebase
    // library must not be touched from here: Firebase.ready() alone can start
    // a token refresh over TLS on this task. PREFLIGHT and AUTHENTICATING are
    // advanced one bounded step per tick by advanceCloudStartupConnect(), and
    // this function returns immediately afterwards, so loop() keeps servicing
    // sensors, automation, safety, actuators and diagnostics at full rate.
    if (cloudStartupPhase == CloudStartupPhase::NOT_STARTED)
    {
        return;
    }

    // Wi-Fi is associated and the cloud startup is armed: escalate to one
    // ordinary STA reconnect if the cloud has stayed not-READY for 30s. On the
    // tick it fires, make no further cloud call on the link it just dropped.
    if (checkStaRecovery())
    {
        return;
    }

    if (cloudStartupPhase == CloudStartupPhase::PREFLIGHT ||
        cloudStartupPhase == CloudStartupPhase::AUTHENTICATING)
    {
        systemState.firebaseConnected = false;
        wasFirebaseConnected = false;
        noteCloudUnavailable();
        advanceCloudStartupConnect();
        return;
    }

    // Path gate for everything after authentication (INIT_STEPS and COMPLETE).
    // While cloudPathVerified is false the library is not called at all: no
    // Firebase.ready() (which can start a token refresh over a brand-new TLS
    // connection), no RTDB call, no recovery attempt. The only work is the
    // background preflight, which never touches the library, so this tick
    // returns at once and loop() keeps its full rate. This is what makes a
    // dead WAN, a dead resolver or a blocked host cost the main loop at most
    // the ONE library call that first noticed it - not one call per tick, and
    // not the 3-strikes-then-cooldown-then-recovery cycle.
    if (!cloudPathVerified)
    {
        if (wasFirebaseConnected && debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.println("Firebase UNAVAILABLE | WiFi=UP, checking path before any cloud call");
        }
        systemState.firebaseConnected = false;
        wasFirebaseConnected = false;
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        noteCloudUnavailable();
        advanceRuntimePreflight();
        return;
    }

    // A token refresh opens a brand-new TLS connection at a moment the library
    // picks, from inside Firebase.ready(). Never let that happen on the strength
    // of a path check that is old: ask for a fresh one first. (isTokenExpired()
    // is a pure time comparison, no network.)
    const bool tokenRefreshDue = Firebase.isTokenExpired();
    if (tokenRefreshDue && !cloudPathFresh())
    {
        revokeCloudPath("token refresh due, path check is stale", false);
        return;
    }

    // Network path verified, Firebase library session NOT yet proven (see
    // cloudSessionValidated). Until validateCloudSession() succeeds this is not
    // READY, no presence is asserted, and nothing but that one validation read
    // is called: no provisioning write, no command read, no upload.
    const bool sessionUnproven =
        cloudStartupPhase == CloudStartupPhase::COMPLETE && !cloudSessionValidated;
    if (sessionUnproven)
    {
        systemState.firebaseConnected = false;
        wasFirebaseConnected = false;
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        noteCloudUnavailable();
    }

    if (suspendedForProvisioning && !sessionUnproven)
    {
        // Confirmed live bug this fixes: /status/provisioning was set true
        // when entering provisioning (see the startProvisioning command
        // handler below) but was never cleared back to false anywhere in
        // this firmware. DeviceConnectionManager.resolveState() on the app
        // side treats provisioning==true as an unconditional "always show
        // Reconnecting," with no time bound of its own - so any device
        // that had EVER gone through Wi-Fi Configuration/AP mode once
        // would show Reconnecting in the app permanently, even while
        // fully online. Written here (not unconditionally alongside the
        // in-memory flag below) so a failed write leaves
        // suspendedForProvisioning true and this retries on the very next
        // tick, mirroring the existing retry-by-not-advancing-state
        // pattern the startProvisioning command handler already uses for
        // its own "set true" write.
        const unsigned long provisioningClearStartedAt = millis();
        const bool provisioningCleared = Firebase.RTDB.setBool(
            &fbdo, deviceRoot() + "/status/provisioning", false);
        logFirebaseDuration("Provisioning state clear", millis() - provisioningClearStartedAt);
        recordFirebaseResult(provisioningCleared);
        if (!provisioningCleared)
        {
            Serial.println("[FIREBASE] Unable to clear provisioning state; will retry next tick");
            return;
        }
        Serial.println("[FIREBASE] Resuming after Wi-Fi reconnect");
        suspendedForProvisioning = false;
    }

    // "Connected" for the rest of the firmware (notification routing, STATUS,
    // heartbeat state) means the cloud is READY: authenticated AND every
    // INIT_STEPS step done. A valid token alone, mid-initialization, is not.
    systemState.firebaseConnected =
        Firebase.ready() && cloudStartupPhase == CloudStartupPhase::COMPLETE &&
        cloudSessionValidated;
    if (systemState.firebaseConnected && !wasFirebaseConnected)
    {
        // Section 5: reformatted to the standard [NET] line, with the outage
        // duration - computed from cloudUnavailableSince BEFORE
        // noteCloudAvailable() (called further below, once reached) clears
        // it, so this always has a real duration to report, short or long.
        // Replaces the previous plain "[FIREBASE] Reconnected" line rather
        // than adding a second one for the same edge.
        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("NET");
            Serial.print("Firebase RESTORED");
            if (cloudUnavailableSince != 0)
            {
                Serial.print(" | outage=");
                Serial.print((millis() - cloudUnavailableSince) / 1000UL);
                Serial.print("s");
            }
            Serial.println();
        }
    }
    else if (!systemState.firebaseConnected && wasFirebaseConnected &&
             debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("NET");
        Serial.println("Firebase UNAVAILABLE | WiFi=UP");
    }
    wasFirebaseConnected = systemState.firebaseConnected;

    if(!Firebase.ready())
    {
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        noteCloudUnavailable();
        // Not ready with a verified path means the library itself just failed
        // (a token refresh that did not complete, or one it declined to retry
        // yet). Do not poll it again every tick: re-verify first, and wait at
        // least COOLDOWN_INITIAL_MS before the next attempt.
        revokeCloudPath("Firebase.ready() is false", true);
        return;
    }

    //--------------------------------------------------
    // Firebase transport health
    //--------------------------------------------------

    // COOLDOWN means repeated transport failures already confirmed the
    // connection is broken - retrying heartbeat/actuator/command calls
    // here would just block for the same timeout again for nothing. No
    // Firebase network call happens this cycle except, once the backoff
    // window has elapsed, exactly one controlled recovery attempt. Local
    // automation, safety, actuators, GSM, and the NVS notification queue
    // are entirely unaffected - they already ran before this function was
    // ever called (see loop()).
    //
    // Critical verification report, Priority 1: recovery itself is now
    // non-blocking. beginFirebaseRecovery() only kicks off the auth state
    // machine and returns immediately; RECOVERING is a real, possibly
    // multi-tick state now (it used to be set and resolved within one
    // synchronous call), polled one bounded step at a time by
    // pollFirebaseRecovery() below on every subsequent update() call until
    // it concludes into HEALTHY or back into COOLDOWN.
    if (firebaseHealth == FirebaseHealthState::COOLDOWN)
    {
        noteCloudUnavailable();
        if (millis() - cooldownStartedAt >= cooldownDurationMs)
        {
            // Recovery re-authenticates, i.e. opens new TLS connections to the
            // auth host. It runs only on the back of a path check that passed
            // moments ago: if the last one has aged out while the cooldown
            // waited, withdraw the permission and let the (free) background
            // preflight redo it first. The cooldown timer has already elapsed,
            // so recovery starts on the first tick after that check passes.
            if (cloudPathFresh())
            {
                beginFirebaseRecovery();
            }
            else
            {
                revokeCloudPath("cooldown over, path check is stale", false);
            }
        }
        return;
    }

    if (firebaseHealth == FirebaseHealthState::RECOVERING)
    {
        noteCloudUnavailable();
        pollFirebaseRecovery();
        return;
    }

    // Session validation. After COOLDOWN/RECOVERING (handled above, so no
    // library call is made during a cooldown wait) and before anything that
    // treats the cloud as usable. Exactly one library operation this tick: if
    // a token refresh was due, Firebase.ready() above already spent this
    // tick's blocking budget on it, so validation starts on the next tick.
    if (sessionUnproven)
    {
        if (!tokenRefreshDue)
        {
            validateCloudSession();
        }
        return;
    }

    // Reaching here means the cloud is usable again. If it was gone long
    // enough, re-baseline both command channels before either is read below.
    noteCloudAvailable();

    // Post-auth initialization (device id, command baseline, database seeding,
    // stale developer-flag clear, first settings read). Runs one step per tick
    // and holds every normal cloud job - heartbeat, command reads, uploads -
    // off until it has finished, so no command can be read, and no dev flag
    // honoured, before its baseline/clear has been established. Placed after
    // noteCloudAvailable() on purpose: any re-baseline that call triggers
    // (an outage of 30s or more) happens BEFORE the steps below (re)establish
    // the baselines.
    if (cloudStartupPhase == CloudStartupPhase::INIT_STEPS)
    {
        advanceCloudInit();
        return;
    }

    // Developer/test flags must be confirmed cleared before their readers are
    // trusted - see devCommandsCleared. The INIT_STEPS clear normally does this
    // already; this is the retry path after a later outage re-arms it.
    if (!devCommandsCleared && !deviceId.isEmpty() &&
        millis() - lastDevCommandClearAttemptAt >= DEV_COMMAND_CLEAR_RETRY_INTERVAL_MS)
    {
        clearDevCommandsAtBoot();
        return;
    }

    // DEGRADED (1-2 transport failures, below the COOLDOWN threshold) keeps
    // essential ops (heartbeat, actuator sync, command reads) on their
    // normal cadence but suppresses low-priority/optional work below,
    // reusing the same deferLowPriorityJobs mechanism already used to
    // protect automatic-operation response latency.
    const bool deferLowPriorityForHealth =
        firebaseHealth != FirebaseHealthState::HEALTHY;

    // /sensors is the authoritative presence heartbeat. When due, it owns this
    // Firebase opportunity and no optional cloud job is allowed to run first.
    // The forced heartbeat owed after a published alert transition (see below)
    // goes first, as its own tick.
    if (alertHeartbeatPending)
    {
        alertHeartbeatPending = false;
        writeSensors(true);
        return;
    }

    if (isSensorUploadDue())
    {
        // ONE library transaction this tick. If it failed, that also withdrew
        // cloudPathVerified, so the tick which discovers a dead path spends
        // its time on that one failed call and nothing else. The actuator
        // publication that used to follow it here runs on the next tick.
        writeSensors();
        return;
    }

    // HIGH PRIORITY: manual actuator control response. Checked directly on
    // every update() call (not via the optional-job round-robin below) so a
    // pending app command is never left waiting behind a low-priority
    // job's rotation slot - only its own existing
    // COMMAND_READ_INTERVAL/backoff statics still govern how often either
    // actually performs a Firebase call, so this changes ordering/latency
    // only, not call frequency. The independent per-actuator esp_timer
    // deadline (see ActuatorManager) is what protects physical timing
    // during a stall; this is a responsiveness improvement on top of that,
    // not a second safety mechanism. Preserves the pre-existing
    // sensorTestEnabled suppression that used to be enforced by these two
    // jobs' skip-list entries in the round-robin below (job == 2/3 there)
    // - normal cultivation commands stay suppressed during the physical
    // sensor diagnostic mode, unchanged.
    if (!systemState.sensorTestEnabled)
    {
        readActuatorCommands();
        if (cloudTickBudgetSpent()) return;
        readCommands();
        if (cloudTickBudgetSpent()) return;
    }

    // Preserve event-driven actuator publication immediately behind heartbeat
    // and command reads.
    writeActuators();
    if (cloudTickBudgetSpent()) return;

    // A slow actuator-status transition may itself consume the remaining
    // heartbeat window. Re-check before starting any optional job.
    if (isSensorUploadDue())
    {
        writeSensors();
        return;
    }

    // Alert transitions are event-driven and run directly behind heartbeat and
    // actuator status instead of waiting for the optional-job cursor. Developer
    // Sensor Test keeps its existing alert/history suppression contract.
    const bool alertWasDirty =
        !systemState.sensorTestEnabled && alertManager.isDirty();
    if (alertWasDirty && writeAlerts())
    {
        alertHeartbeatPending = true;
    }
    if (cloudTickBudgetSpent()) return;

    // A slow alert update can consume the remaining heartbeat window.
    if (isSensorUploadDue())
    {
        writeSensors();
        return;
    }

    // Reduces the chance a known-slow, low-priority job (fogging/
    // notification ACK poll especially - the field-observed source of the
    // longest stalls) STARTS close to an actual manual command. Gated on
    // lastManualCommandActivityAt (set only when a fresh command is
    // actually observed) rather than bare manualMode: manualMode can stay
    // on for minutes with no command in flight, which was too coarse a
    // signal - a low-priority job starting seconds before an eventual tap
    // was still exposed to the same 10-70s stall risk. This is
    // deliberately narrower than deferLowPriorityJobs below (which also
    // covers readSettings/writeStatus for the control-response/health
    // cases): those two stay on their normal cadence here because they
    // carry safety-relevant state (safetyLock, reservoirLocked, target
    // ranges) Android's manual-control UI depends on. Bounded by
    // MANUAL_MODE_LOW_PRIORITY_GRACE_MS so a flurry of commands spaced
    // under 5s apart still can't suppress these jobs indefinitely - see
    // runOneOptionalFirebaseJob().
    const bool recentManualCommandActivity =
        millis() - lastManualCommandActivityAt < MANUAL_COMMAND_ACTIVITY_WINDOW_MS;
    const bool deferLowPriorityForManualInteraction =
        recentManualCommandActivity &&
        (millis() - lastLowPriorityCloudJobAt < MANUAL_MODE_LOW_PRIORITY_GRACE_MS);

    // Every remaining synchronous read/write is distributed across subsequent
    // loop iterations so slow requests cannot accumulate in one update.
    runOneOptionalFirebaseJob(
        systemState.sensorTestEnabled,
        alertWasDirty || deferLowPriorityForControlResponse || deferLowPriorityForHealth,
        deferLowPriorityForManualInteraction);
}

bool FirebaseManager::isSensorUploadDue() const
{
    return millis() - lastSensorUploadAttempt >= SENSOR_UPLOAD_INTERVAL_MS;
}

bool FirebaseManager::shouldDeferOptionalJobsForControlResponse()
{
    const OperationRequest& request = systemState.operationRequest;
    const bool responseSensitiveOperation =
        request.operation == OperationType::PH_UP ||
        request.operation == OperationType::PH_DOWN ||
        request.operation == OperationType::EC_CORRECTION ||
        request.operation == OperationType::REFILL;

    if (request.source == RequestSource::AUTOMATIC &&
        responseSensitiveOperation &&
        request.state == RequestState::ACCEPTED &&
        request.requestId != lastProtectedAutomaticRequestId)
    {
        lastProtectedAutomaticRequestId = request.requestId;
        automaticControlPassesRemaining = 2;
    }

    if (automaticControlPassesRemaining == 0)
    {
        return false;
    }

    automaticControlPassesRemaining--;
    return true;
}

void FirebaseManager::runOneOptionalFirebaseJob(
    bool sensorTestMode,
    bool deferLowPriorityJobs,
    bool deferLowPriorityForManualInteraction)
{
    // Actuator/operation command reads (formerly jobs 2/3 here) moved to
    // their own always-checked fast path directly in update(), immediately
    // behind heartbeat - see the comment there. Every remaining job below
    // shifted down by 2 accordingly; this list is otherwise unchanged.
    constexpr uint8_t OPTIONAL_JOB_COUNT = 15;

    for (uint8_t checked = 0; checked < OPTIONAL_JOB_COUNT; checked++)
    {
        const uint8_t job = optionalFirebaseJobCursor;
        optionalFirebaseJobCursor = (optionalFirebaseJobCursor + 1) % OPTIONAL_JOB_COUNT;

        // Normal cultivation commands/alerts remain suppressed during the
        // existing physical sensor diagnostic mode. Command reads (formerly
        // job 2/3) are now suppressed at their new call site in update()
        // instead of here.
        if (sensorTestMode &&
            (job == 0 || job == 1 || job == 4))
        {
            continue;
        }
        if (!sensorTestMode && job == 8)
        {
            continue;
        }

        // A new alert transition must not sit behind low-priority
        // synchronization. Advance the cursor past these jobs now; they
        // remain eligible on later non-urgent rotations and therefore
        // cannot be permanently starved. Recipient/harvest-schedule sync
        // and notification/fogging replay (9-12) are likewise low-priority
        // background work, same treatment as 4-8 - sensors/safety/
        // actuator sync/commands/heartbeats must never be starved by
        // history replay.
        if (deferLowPriorityJobs &&
            (job == 4 || job == 5 || job == 6 || job == 7 || job == 8 ||
             job == 9 || job == 10 || job == 11 || job == 12 || job == 14))
        {
            continue;
        }

        // Narrower manual-interaction deferral: telemetry (6), device info
        // (7), diagnostic sensors (8), SMS recipients (9), notification
        // (11), fogging (12), and the Test SMS command read (14) - the
        // known-slow, purely-optional jobs. readSettings (4) and
        // writeStatus (5) are deliberately exempt (see update()). Harvest
        // schedule (10) is exempt only once a cached active schedule
        // already exists - a device with none yet must still be able to
        // learn of one while Manual Mode happens to be on.
        if (deferLowPriorityForManualInteraction &&
            (job == 6 || job == 7 || job == 8 || job == 9 || job == 11 || job == 12 || job == 14 ||
             (job == 10 && harvestScheduleCache.isActive())))
        {
            continue;
        }

        // This job is about to actually run (survived every skip check
        // above) - reset the grace-window clock so the NEXT manual-mode
        // check starts a fresh bounded deferral instead of compounding.
        if (job == 6 || job == 7 || job == 8 || job == 9 || job == 10 ||
            job == 11 || job == 12 || job == 14)
        {
            lastLowPriorityCloudJobAt = millis();
        }

        switch (job)
        {
            case 0:
                // Dirty transitions bypass the cursor above. This slot preserves
                // the existing 60-second full fallback and initial gated publish.
                if (!alertManager.isDirty()) writeAlerts();
                return;

            case 1:
                syncOperationState();
                return;

            case 2:
                readSensorTestCommand();
                return;

            case 3:
                readMockSensors();
                return;

            case 4:
                if (millis() - lastSettingsRead >= SETTINGS_READ_INTERVAL)
                {
                    lastSettingsRead = millis();
                    readSettings();
                }
                return;

            case 5:
                writeStatus();
                return;

            case 6:
                writeTelemetry();
                return;

            case 7:
                writeDeviceInfo();
                return;

            case 8:
                writeDiagnosticSensors();
                return;

            case 9:
                readSmsRecipients();
                return;

            case 10:
                readHarvestSchedule();
                return;

            case 11:
                replayQueuedNotification();
                return;

            case 12:
                replayQueuedFoggingEvent();
                return;

            case 13:
                readWaterLevelOverrideCommand();
                return;

            case 14:
                readTestSmsCommand();
                return;
        }
    }
}

void FirebaseManager::syncOperationState()
{
    OperationRequest& request = systemState.operationRequest;
    const RequestState state = request.state;
    const bool terminal = state == RequestState::COMPLETED ||
        state == RequestState::FAILED || state == RequestState::REJECTED;
    const bool orderedAutomaticCompletion =
        state == RequestState::COMPLETED &&
        request.source == RequestSource::AUTOMATIC &&
        (request.operation == OperationType::PH_UP ||
         request.operation == OperationType::PH_DOWN ||
         request.operation == OperationType::EC_CORRECTION ||
         request.operation == OperationType::REFILL);

    if (state == lastPublishedOperationState && !terminal)
    {
        return;
    }

    if (state != lastPublishedOperationState)
    {
        if (orderedAutomaticCompletion)
        {
            // The capture runs before connectivity checks, so this remains the
            // exact effective snapshot observed when the local operation ended.
            if (!automaticTerminalSyncPending ||
                automaticTerminalRequestId != request.requestId)
            {
                captureAutomaticTerminalSnapshot();
            }

            if (!automaticTerminalSensorUploaded)
            {
                if (!writeSensors(true, &automaticTerminalSensors))
                {
                    if (!automaticTerminalSensorUploadFailureLogged)
                    {
                        Serial.print("[LATENCY] sensorSnapshotFailed t=");
                        Serial.print(millis());
                        Serial.print(" requestId=");
                        Serial.println(request.requestId);
                        automaticTerminalSensorUploadFailureLogged = true;
                    }
                    return;
                }

                automaticTerminalSensorUploaded = true;
                automaticTerminalSensorUploadFailureLogged = false;
                automaticTerminalSnapshotUploadedAt = millis();
                if (debugManager.shouldPrintDebug(DebugCategory::NETWORK))
                {
                    Serial.println("[OP-SYNC] Sensor snapshot uploaded");

                    Serial.print("[LATENCY] sensorSnapshotUploaded t=");
                    Serial.print(automaticTerminalSnapshotUploadedAt);
                    Serial.print(" requestId=");
                    Serial.println(request.requestId);
                    Serial.print("[LATENCY] localComplete -> sensorSnapshotUploaded = ");
                    Serial.print(automaticTerminalSnapshotUploadedAt - request.completedTimestamp);
                    Serial.println(" ms");
                }
            }
        }

        if (!writeCurrentOperation())
        {
            // A later heartbeat could overwrite /sensors before this terminal
            // retry. Require the frozen completion snapshot immediately before
            // every new COMPLETED publication attempt.
            if (orderedAutomaticCompletion)
            {
                automaticTerminalSensorUploaded = false;
            }

            if (!operationPublishFailureLogged)
            {
                Serial.print("[OP-SYNC] ");
                Serial.print(requestStateToString(state));
                Serial.println(" publish failed - retry pending");
                operationPublishFailureLogged = true;

                if (orderedAutomaticCompletion)
                {
                    Serial.print("[LATENCY] completedPublishFailed t=");
                    Serial.print(millis());
                    Serial.print(" requestId=");
                    Serial.println(request.requestId);
                }
            }
            return;
        }

        // Publication bookkeeping advances only after Firebase acknowledges the
        // write. A failure leaves the same request and timestamps retryable.
        lastPublishedOperationState = state;
        operationPublishFailureLogged = false;

        if (terminal)
        {
            const bool dbgNetwork = debugManager.shouldPrintDebug(DebugCategory::NETWORK);
            if (dbgNetwork)
            {
                Serial.print("[OP-SYNC] ");
                Serial.print(requestStateToString(state));
                Serial.print(" published requestId=");
                Serial.print(request.requestId);
                Serial.print(" operation=");
                Serial.println(operationToString(request.operation));
            }

            if (orderedAutomaticCompletion)
            {
                if (dbgNetwork)
                {
                    const unsigned long publishedAt = millis();
                    Serial.print("[LATENCY] completedPublished t=");
                    Serial.print(publishedAt);
                    Serial.print(" requestId=");
                    Serial.println(request.requestId);
                    Serial.print("[LATENCY] sensorSnapshotUploaded -> completedPublished = ");
                    Serial.print(publishedAt - automaticTerminalSnapshotUploadedAt);
                    Serial.println(" ms");
                    Serial.print("[LATENCY] localComplete -> completedPublished = ");
                    Serial.print(publishedAt - request.completedTimestamp);
                    Serial.println(" ms");
                }

                if (request.operation == OperationType::PH_UP ||
                    request.operation == OperationType::PH_DOWN ||
                    request.operation == OperationType::EC_CORRECTION)
                {
                    systemState.chemistryFoggingHoldActive = false;
                    // Relevant to PH/EC (unblocks fogging eligibility after a
                    // chemistry correction) and to FOGGING (its own optional
                    // dependency) - not folded into dbgNetwork above.
                    if (debugManager.shouldPrintDebug(DebugCategory::PH) ||
                        debugManager.shouldPrintDebug(DebugCategory::EC) ||
                        debugManager.shouldPrintDebug(DebugCategory::FOGGING))
                    {
                        Serial.println("[CHEMISTRY] Lifecycle published - fogging eligible");
                    }
                }
            }
        }
    }

    if (!terminal)
    {
        return;
    }

    // Archive failures are retried while the already-published terminal state
    // remains intact. Reset is permitted only after both durable writes succeed.
    if (archiveCurrentOperation())
    {
        const uint16_t archivedRequestId = request.requestId;
        resetCurrentOperation();

        if (automaticTerminalSyncPending &&
            automaticTerminalRequestId == archivedRequestId)
        {
            automaticTerminalSyncPending = false;
            automaticTerminalSensorUploaded = false;
            automaticTerminalRequestId = 0;
        }
    }
}

void FirebaseManager::captureAutomaticTerminalSnapshot()
{
    const OperationRequest& request = systemState.operationRequest;
    const bool requiresSensorOrdering =
        request.state == RequestState::COMPLETED &&
        request.source == RequestSource::AUTOMATIC &&
        (request.operation == OperationType::PH_UP ||
         request.operation == OperationType::PH_DOWN ||
         request.operation == OperationType::EC_CORRECTION ||
         request.operation == OperationType::REFILL);

    if (!requiresSensorOrdering ||
        (automaticTerminalSyncPending &&
         automaticTerminalRequestId == request.requestId))
    {
        return;
    }

    automaticTerminalSyncPending = true;
    automaticTerminalSensorUploaded = false;
    automaticTerminalSensorUploadFailureLogged = false;
    automaticTerminalRequestId = request.requestId;
    automaticTerminalSensors = sensors;

    Serial.print("[OP-SYNC] COMPLETED pending requestId=");
    Serial.print(request.requestId);
    Serial.print(" operation=");
    Serial.println(operationToString(request.operation));
}

//==================================================
// Settings Synchronization
//==================================================

void FirebaseManager::readSettings()
{
    const unsigned long startedAt = millis();
    const bool succeeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/settings");
    logFirebaseDuration("Settings read", millis() - startedAt);
    recordFirebaseResult(succeeded);
    lastSettingsReadOk = succeeded;
    if(!succeeded)
    {
        return;
    }

    FirebaseJsonData data;

    //--------------------------------------------------
    // Grow Light
    //--------------------------------------------------

    if (fbdo.jsonObject().get(data, "lightOnHour") && data.intValue >= 0 && data.intValue <= 23)
        systemState.lightOnHour = static_cast<uint8_t>(data.intValue);

    if (fbdo.jsonObject().get(data, "lightOnMinute") && data.intValue >= 0 && data.intValue <= 59)
        systemState.lightOnMinute = static_cast<uint8_t>(data.intValue);

    if (fbdo.jsonObject().get(data, "lightOffHour") && data.intValue >= 0 && data.intValue <= 23)
        systemState.lightOffHour = static_cast<uint8_t>(data.intValue);

    if (fbdo.jsonObject().get(data, "lightOffMinute") && data.intValue >= 0 && data.intValue <= 59)
        systemState.lightOffMinute = static_cast<uint8_t>(data.intValue);

    //--------------------------------------------------
    // pH
    //--------------------------------------------------

    float incomingMinPH = systemState.minPH;
    float incomingMaxPH = systemState.maxPH;

    if (fbdo.jsonObject().get(data, "minPH"))
        incomingMinPH = data.floatValue;

    if (fbdo.jsonObject().get(data, "maxPH"))
        incomingMaxPH = data.floatValue;

    // Validate pH bounds
    if (incomingMinPH >= 0.0f && incomingMaxPH <= 14.0f && incomingMaxPH > incomingMinPH)
    {
        systemState.minPH = incomingMinPH;
        systemState.maxPH = incomingMaxPH;
    }

    float incomingPHTargetMin = systemState.phTargetMin;
    float incomingPHTargetMax = systemState.phTargetMax;
    if (fbdo.jsonObject().get(data, "phTargetMin")) incomingPHTargetMin = data.floatValue;
    if (fbdo.jsonObject().get(data, "phTargetMax")) incomingPHTargetMax = data.floatValue;
    if (incomingPHTargetMin >= systemState.minPH &&
        incomingPHTargetMax <= systemState.maxPH &&
        incomingPHTargetMax > incomingPHTargetMin)
    {
        systemState.phTargetMin = incomingPHTargetMin;
        systemState.phTargetMax = incomingPHTargetMax;
    }

    //--------------------------------------------------
    // EC
    //--------------------------------------------------

    float incomingMinEC = systemState.minEC;
    float incomingMaxEC = systemState.maxEC;
    float incomingECTargetMin = systemState.ecTargetMin;
    float incomingECTargetMax = systemState.ecTargetMax;
    if (fbdo.jsonObject().get(data, "minEC")) incomingMinEC = data.floatValue;
    if (fbdo.jsonObject().get(data, "maxEC")) incomingMaxEC = data.floatValue;
    if (fbdo.jsonObject().get(data, "ecTargetMin")) incomingECTargetMin = data.floatValue;
    if (fbdo.jsonObject().get(data, "ecTargetMax")) incomingECTargetMax = data.floatValue;
    if (incomingMinEC > 0.0f && incomingMaxEC > incomingMinEC &&
        incomingECTargetMin >= incomingMinEC &&
        incomingECTargetMax <= incomingMaxEC &&
        incomingECTargetMax > incomingECTargetMin)
    {
        systemState.minEC = incomingMinEC;
        systemState.maxEC = incomingMaxEC;
        systemState.ecTargetMin = incomingECTargetMin;
        systemState.ecTargetMax = incomingECTargetMax;
    }

    //--------------------------------------------------
    // Target (acceptable) ranges
    //
    // A missing field keeps the current/compiled value - an old device whose
    // settings document predates these fields keeps working on defaults. A
    // pair is applied only when it is physically sensible AND min < max, so a
    // malformed or inverted remote edit is rejected and the last valid range
    // survives.
    //--------------------------------------------------

    applyTargetRange("minAirTemp", "maxAirTemp",
        systemState.minAirTemp, systemState.maxAirTemp, -40.0f, 80.0f);
    applyTargetRange("minHumidity", "maxHumidity",
        systemState.minHumidity, systemState.maxHumidity, 0.0f, 100.0f);
    applyTargetRange("minWaterTemp", "maxWaterTemp",
        systemState.minWaterTemp, systemState.maxWaterTemp, 0.0f, 100.0f);
    applyTargetRange("minWaterLevel", "maxWaterLevel",
        systemState.minWaterLevel, systemState.maxWaterLevel, 0.0f, 100.0f);

    //--------------------------------------------------
    // Reservoir
    //--------------------------------------------------

    float incomingRefillStart = systemState.refillStartLevel;
    float incomingRefillStop = systemState.refillStopLevel;

    bool hasRefillStart = fbdo.jsonObject().get(data, "refillStartLevel");
    if (hasRefillStart)
        incomingRefillStart = data.floatValue;

    bool hasRefillStop = fbdo.jsonObject().get(data, "refillStopLevel");
    if (hasRefillStop)
        incomingRefillStop = data.floatValue;

    if (hasRefillStart || hasRefillStop)
    {
        const bool validStart = incomingRefillStart >= 0.0f && incomingRefillStart <= 100.0f;
        const bool validStop = incomingRefillStop >= 0.0f && incomingRefillStop <= 100.0f;
        const bool validOrder = incomingRefillStart < incomingRefillStop;

        if (validStart && validStop && validOrder)
        {
            const bool changed =
                floatValuesDiffer(systemState.refillStartLevel, incomingRefillStart) ||
                floatValuesDiffer(systemState.refillStopLevel, incomingRefillStop);

            systemState.refillStartLevel = incomingRefillStart;
            systemState.refillStopLevel = incomingRefillStop;

            if (!refillSettingsInitialized || changed)
            {
                Serial.println("[SETTINGS] Refill thresholds updated");
                Serial.print("[SETTINGS] refillStartLevel=");
                Serial.println(systemState.refillStartLevel, 2);
                Serial.print("[SETTINGS] refillStopLevel=");
                Serial.println(systemState.refillStopLevel, 2);
            }

            refillSettingsInitialized = true;
            refillRejectionLogged = false;
        }
        else
        {
            const bool newRejection =
                !refillRejectionLogged ||
                floatValuesDiffer(lastRejectedRefillStart, incomingRefillStart) ||
                floatValuesDiffer(lastRejectedRefillStop, incomingRefillStop);

            if (newRejection)
            {
                Serial.println("[SETTINGS] Refill threshold update rejected");
                Serial.print("[SETTINGS] reason=");
                if (!validStart)
                    Serial.println("refillStartLevel must be between 0 and 100");
                else if (!validStop)
                    Serial.println("refillStopLevel must be at most 100");
                else
                    Serial.println("refillStartLevel must be less than refillStopLevel");
                Serial.print("[SETTINGS] keeping start=");
                Serial.println(systemState.refillStartLevel, 2);
                Serial.print("[SETTINGS] keeping stop=");
                Serial.println(systemState.refillStopLevel, 2);
            }

            refillRejectionLogged = true;
            lastRejectedRefillStart = incomingRefillStart;
            lastRejectedRefillStop = incomingRefillStop;
        }
    }

    //--------------------------------------------------
    // Reservoir - water-depth model (AUTHORITATIVE for control; see
    // Config.h's "Water Reservoir Geometry" section). Legacy percentage
    // pair above is no longer read by any control path.
    //--------------------------------------------------

    float incomingRefillStartCm = systemState.refillStartLevelCm;
    float incomingRefillStopCm = systemState.refillStopLevelCm;

    bool hasRefillStartCm = fbdo.jsonObject().get(data, "refillStartLevelCm");
    if (hasRefillStartCm)
        incomingRefillStartCm = data.floatValue;

    bool hasRefillStopCm = fbdo.jsonObject().get(data, "refillStopLevelCm");
    if (hasRefillStopCm)
        incomingRefillStopCm = data.floatValue;

    if (hasRefillStartCm || hasRefillStopCm)
    {
        const bool validStartCm = incomingRefillStartCm >= 0.0f && incomingRefillStartCm <= MAX_WORKING_WATER_CM;
        const bool validStopCm = incomingRefillStopCm >= 0.0f && incomingRefillStopCm <= MAX_WORKING_WATER_CM;
        const bool validOrderCm = incomingRefillStartCm < incomingRefillStopCm;

        if (validStartCm && validStopCm && validOrderCm)
        {
            const bool changedCm =
                floatValuesDiffer(systemState.refillStartLevelCm, incomingRefillStartCm) ||
                floatValuesDiffer(systemState.refillStopLevelCm, incomingRefillStopCm);

            systemState.refillStartLevelCm = incomingRefillStartCm;
            systemState.refillStopLevelCm = incomingRefillStopCm;

            if (!refillLevelCmSettingsInitialized || changedCm)
            {
                Serial.println("[SETTINGS] Refill depth thresholds updated");
                Serial.print("[SETTINGS] refillStartLevelCm=");
                Serial.println(systemState.refillStartLevelCm, 2);
                Serial.print("[SETTINGS] refillStopLevelCm=");
                Serial.println(systemState.refillStopLevelCm, 2);
            }

            refillLevelCmSettingsInitialized = true;
            refillLevelCmRejectionLogged = false;
        }
        else
        {
            const bool newRejectionCm =
                !refillLevelCmRejectionLogged ||
                floatValuesDiffer(lastRejectedRefillStartCm, incomingRefillStartCm) ||
                floatValuesDiffer(lastRejectedRefillStopCm, incomingRefillStopCm);

            if (newRejectionCm)
            {
                Serial.println("[SETTINGS] Refill depth threshold update rejected");
                Serial.print("[SETTINGS] reason=");
                if (!validStartCm)
                    Serial.println("refillStartLevelCm must be between 0 and MAX_WORKING_WATER_CM");
                else if (!validStopCm)
                    Serial.println("refillStopLevelCm must be at most MAX_WORKING_WATER_CM");
                else
                    Serial.println("refillStartLevelCm must be less than refillStopLevelCm");
                Serial.print("[SETTINGS] keeping startCm=");
                Serial.println(systemState.refillStartLevelCm, 2);
                Serial.print("[SETTINGS] keeping stopCm=");
                Serial.println(systemState.refillStopLevelCm, 2);
            }

            refillLevelCmRejectionLogged = true;
            lastRejectedRefillStartCm = incomingRefillStartCm;
            lastRejectedRefillStopCm = incomingRefillStopCm;
        }
    }

    if (fbdo.jsonObject().get(data, "criticalLowWaterCm"))
    {
        const float incomingCriticalLowCm = data.floatValue;
        if (incomingCriticalLowCm >= 0.0f && incomingCriticalLowCm <= MAX_WORKING_WATER_CM)
        {
            if (floatValuesDiffer(systemState.criticalLowWaterCm, incomingCriticalLowCm))
            {
                systemState.criticalLowWaterCm = incomingCriticalLowCm;
                Serial.print("[SETTINGS] criticalLowWaterCm=");
                Serial.println(systemState.criticalLowWaterCm, 2);
            }
        }
        else
        {
            Serial.println("[SETTINGS] criticalLowWaterCm update rejected: must be between 0 and MAX_WORKING_WATER_CM");
        }
    }

    //--------------------------------------------------
    // Water level sensor calibration (empty/full distance, cm)
    //--------------------------------------------------

    float incomingWaterLevelEmptyCm = systemState.waterLevelEmptyDistanceCm;
    float incomingWaterLevelFullCm = systemState.waterLevelFullDistanceCm;

    bool hasWaterLevelEmptyCm = fbdo.jsonObject().get(data, "waterLevelEmptyDistanceCm");
    if (hasWaterLevelEmptyCm)
        incomingWaterLevelEmptyCm = data.floatValue;

    bool hasWaterLevelFullCm = fbdo.jsonObject().get(data, "waterLevelFullDistanceCm");
    if (hasWaterLevelFullCm)
        incomingWaterLevelFullCm = data.floatValue;

    if (hasWaterLevelEmptyCm || hasWaterLevelFullCm)
    {
        // Both must be positive (physical distances), and full-tank distance
        // must be strictly less than empty-tank distance - the sensor sits
        // above the water, so a fuller tank is always a shorter distance.
        const bool validEmpty = incomingWaterLevelEmptyCm > 0.0f;
        const bool validFull = incomingWaterLevelFullCm > 0.0f;
        const bool validOrder = incomingWaterLevelFullCm < incomingWaterLevelEmptyCm;

        if (validEmpty && validFull && validOrder)
        {
            const bool changed =
                floatValuesDiffer(systemState.waterLevelEmptyDistanceCm, incomingWaterLevelEmptyCm) ||
                floatValuesDiffer(systemState.waterLevelFullDistanceCm, incomingWaterLevelFullCm);

            systemState.waterLevelEmptyDistanceCm = incomingWaterLevelEmptyCm;
            systemState.waterLevelFullDistanceCm = incomingWaterLevelFullCm;

            if (!waterLevelCalibrationInitialized || changed)
            {
                Serial.println("[SETTINGS] Water level calibration updated");
                Serial.print("[SETTINGS] waterLevelEmptyDistanceCm=");
                Serial.println(systemState.waterLevelEmptyDistanceCm, 2);
                Serial.print("[SETTINGS] waterLevelFullDistanceCm=");
                Serial.println(systemState.waterLevelFullDistanceCm, 2);
            }

            waterLevelCalibrationInitialized = true;
            waterLevelCalibrationRejectionLogged = false;
        }
        else
        {
            const bool newRejection =
                !waterLevelCalibrationRejectionLogged ||
                floatValuesDiffer(lastRejectedWaterLevelEmptyCm, incomingWaterLevelEmptyCm) ||
                floatValuesDiffer(lastRejectedWaterLevelFullCm, incomingWaterLevelFullCm);

            if (newRejection)
            {
                Serial.println("[SETTINGS] Water level calibration update rejected");
                Serial.print("[SETTINGS] reason=");
                if (!validEmpty)
                    Serial.println("waterLevelEmptyDistanceCm must be greater than 0");
                else if (!validFull)
                    Serial.println("waterLevelFullDistanceCm must be greater than 0");
                else
                    Serial.println("waterLevelFullDistanceCm must be less than waterLevelEmptyDistanceCm");
                Serial.print("[SETTINGS] keeping empty=");
                Serial.println(systemState.waterLevelEmptyDistanceCm, 2);
                Serial.print("[SETTINGS] keeping full=");
                Serial.println(systemState.waterLevelFullDistanceCm, 2);
            }

            waterLevelCalibrationRejectionLogged = true;
            lastRejectedWaterLevelEmptyCm = incomingWaterLevelEmptyCm;
            lastRejectedWaterLevelFullCm = incomingWaterLevelFullCm;
        }
    }

    //--------------------------------------------------
    // Sensor-to-bottom calibration (authoritative - see Types.h's
    // sensorToBottomCm comment). Deliberately a SEPARATE field/key from
    // waterLevelEmptyDistanceCm above, never falling back to it, so a
    // stale legacy value cannot silently resurface here.
    //--------------------------------------------------

    bool hasSensorToBottomCm = fbdo.jsonObject().get(data, "sensorToBottomCm");
    if (hasSensorToBottomCm)
    {
        const float incomingSensorToBottomCm = data.floatValue;

        if (incomingSensorToBottomCm > 0.0f)
        {
            const bool changed = floatValuesDiffer(systemState.sensorToBottomCm, incomingSensorToBottomCm);

            systemState.sensorToBottomCm = incomingSensorToBottomCm;

            if (!sensorToBottomCalibrationInitialized || changed)
            {
                Serial.print("[SETTINGS] sensorToBottomCm=");
                Serial.println(systemState.sensorToBottomCm, 2);
            }

            sensorToBottomCalibrationInitialized = true;
        }
        else
        {
            Serial.println("[SETTINGS] sensorToBottomCm update rejected: must be greater than 0");
        }
    }

    //--------------------------------------------------
    // Air temperature alert threshold
    //--------------------------------------------------

    if (fbdo.jsonObject().get(data, "highAirTemp") && data.success)
    {
        const float incomingHighAir = data.floatValue;
        const bool numericHighAir =
            data.typeNum == FirebaseJson::JSON_FLOAT ||
            data.typeNum == FirebaseJson::JSON_DOUBLE ||
            data.typeNum == FirebaseJson::JSON_INT;
        const bool validHighAir =
            numericHighAir &&
            isfinite(incomingHighAir) &&
            incomingHighAir >= -40.0f &&
            incomingHighAir <= 80.0f;

        if (validHighAir)
        {
            const bool changed =
                floatValuesDiffer(systemState.highAirTemp, incomingHighAir);

            systemState.highAirTemp = incomingHighAir;

            if (!highAirTempSettingsInitialized || changed)
            {
                Serial.print("[SETTINGS] highAirTemp=");
                Serial.println(systemState.highAirTemp, 2);
            }

            highAirTempSettingsInitialized = true;
            highAirTempRejectionLogged = false;
        }
        else
        {
            const bool newRejection =
                !highAirTempRejectionLogged ||
                floatValuesDiffer(lastRejectedHighAirTemp, incomingHighAir);

            if (newRejection)
            {
                Serial.print("[SETTINGS] highAirTemp rejected: ");
                Serial.print(incomingHighAir, 2);
                Serial.println(" (valid DHT22 range is -40.00 to 80.00 C)");
                Serial.print("[SETTINGS] keeping highAirTemp=");
                Serial.println(systemState.highAirTemp, 2);
            }

            highAirTempRejectionLogged = true;
            lastRejectedHighAirTemp = incomingHighAir;
        }
    }

    float incomingAirRelease = systemState.airTempRelease;
    float incomingHighHumidity = systemState.highHumidity;
    float incomingHumidityRelease = systemState.humidityRelease;
    if (fbdo.jsonObject().get(data, "airTempRelease")) incomingAirRelease = data.floatValue;
    if (incomingAirRelease >= -40.0f && incomingAirRelease < systemState.highAirTemp)
        systemState.airTempRelease = incomingAirRelease;
    if (fbdo.jsonObject().get(data, "highHumidity")) incomingHighHumidity = data.floatValue;
    if (fbdo.jsonObject().get(data, "humidityRelease")) incomingHumidityRelease = data.floatValue;
    if (incomingHumidityRelease >= 0.0f && incomingHighHumidity <= 100.0f &&
        incomingHumidityRelease < incomingHighHumidity)
    {
        systemState.highHumidity = incomingHighHumidity;
        systemState.humidityRelease = incomingHumidityRelease;
    }

    //--------------------------------------------------
    // Water cooling
    //--------------------------------------------------

    float incomingHighWater = systemState.highWaterTemp;
    float incomingCoolerOff = systemState.coolerOffTemp;

    if (fbdo.jsonObject().get(data, "highWaterTemp"))
        incomingHighWater = data.floatValue;

    if (fbdo.jsonObject().get(data, "coolerOffTemp"))
        incomingCoolerOff = data.floatValue;

    if (incomingHighWater > 0.0f && incomingHighWater <= 100.0f &&
        incomingCoolerOff >= 0.0f && incomingHighWater > incomingCoolerOff)
    {
        systemState.highWaterTemp = incomingHighWater;
        systemState.coolerOffTemp = incomingCoolerOff;
    }

    //--------------------------------------------------
    // Automatic fogging blower speed
    //--------------------------------------------------

    float incomingBlowerSpeed = systemState.blowerSpeedPercent;
    if (fbdo.jsonObject().get(data, "blowerSpeedPercent"))
        incomingBlowerSpeed = data.floatValue;

    if (incomingBlowerSpeed >= BLOWER_SPEED_MIN_PERCENT &&
        incomingBlowerSpeed <= BLOWER_SPEED_MAX_PERCENT)
    {
        systemState.blowerSpeedPercent = (uint8_t)incomingBlowerSpeed;
        blowerSpeedRejectionLogged = false;
    }
    else
    {
        const bool newRejection =
            !blowerSpeedRejectionLogged ||
            floatValuesDiffer(lastRejectedBlowerSpeed, incomingBlowerSpeed);

        if (newRejection)
        {
            Serial.print("[SETTINGS] blowerSpeedPercent rejected: ");
            Serial.print(incomingBlowerSpeed, 1);
            Serial.print(" (valid range is ");
            Serial.print(BLOWER_SPEED_MIN_PERCENT);
            Serial.print(" to ");
            Serial.print(BLOWER_SPEED_MAX_PERCENT);
            Serial.println(")");
            Serial.print("[SETTINGS] keeping blowerSpeedPercent=");
            Serial.println(systemState.blowerSpeedPercent);
        }

        blowerSpeedRejectionLogged = true;
        lastRejectedBlowerSpeed = incomingBlowerSpeed;
    }

    // Config/settings schema migration, Firebase side (see
    // CONFIG_SCHEMA_VERSION in Config.h and loadPersistedSettings()'s
    // matching comment for the local/offline side of this same migration).
    // The ordinary pull logic above (minAirTemp/maxAirTemp via
    // applyTargetRange(), blowerSpeedPercent just above) may have just
    // re-applied whatever STALE value an already-deployed device's
    // Firebase /settings node still holds from before this schema version
    // (maxAirTemp=28, blowerSpeedPercent=30) - deliberately overridden
    // here, AFTER that pull, so the correction always wins over a stale
    // pulled value this tick. Pushed to Firebase (not merely corrected
    // in-memory) so the NEXT sync pulls the corrected value instead of
    // reverting to the stale one - a genuine PUSH, the one exception to
    // this function's otherwise pull-only behavior, and it only ever runs
    // while configMigrationPending is true. cfgVersion is persisted to NVS
    // ONLY once this push actually succeeds - if Firebase is unreachable,
    // configMigrationPending simply stays true and this block retries on
    // the next successful sync; a device that never regains connectivity
    // keeps running correctly on the values
    // loadPersistedSettings() already corrected locally, indefinitely, it
    // just never marks the migration formally complete.
    if (systemState.configMigrationPending)
    {
        systemState.maxAirTemp = TARGET_MAX_AIR_TEMP;
        systemState.blowerSpeedPercent = BLOWER_SPEED_DEFAULT_PERCENT;

        FirebaseJson migrationJson;
        migrationJson.set("maxAirTemp", systemState.maxAirTemp);
        migrationJson.set("blowerSpeedPercent", systemState.blowerSpeedPercent);

        if (updateJson(deviceRoot() + "/settings", migrationJson))
        {
            systemState.configMigrationPending = false;

            if (preferences.begin("automation", false))
            {
                preferences.putUChar("cfgVersion", CONFIG_SCHEMA_VERSION);
                preferences.end();
            }

            Serial.println("[SETTINGS] Config migration complete: maxAirTemp=32C, blowerSpeedPercent=65% pushed to Firebase");
        }
        else
        {
            Serial.println("[SETTINGS] Config migration Firebase push failed - local values already corrected, will retry next sync");
        }
    }

    // Only validated/accepted runtime values are persisted.
    persistSettings();
}


bool FirebaseManager::readHardwareStaMac(uint8_t out[6])
{
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK)
    {
        Serial.println("[IDENTITY] ERROR: Unable to resolve hardware Wi-Fi MAC");
        return false;
    }

    bool allZero = true;
    for (int i = 0; i < 6; i++)
    {
        if (mac[i] != 0) { allZero = false; break; }
    }
    if (allZero)
    {
        Serial.println("[IDENTITY] ERROR: Unable to resolve hardware Wi-Fi MAC");
        return false;
    }

    memcpy(out, mac, 6);
    return true;
}

String FirebaseManager::getMacAddress()
{
    uint8_t mac[6];
    if (!readHardwareStaMac(mac)) return "";
    char buf[13];
    snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

String FirebaseManager::getFormattedMacAddress()
{
    uint8_t mac[6];
    if (!readHardwareStaMac(mac)) return "";
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

bool FirebaseManager::provisionDevice()
{
    String mac = getMacAddress();
    if (mac.isEmpty())
    {
        // Never derive a provisioning lookup from a zero/unresolvable MAC.
        Serial.println("[IDENTITY] ERROR: Provisioning deferred - hardware MAC unavailable");
        return false;
    }
    String path = "/provisioning/" + mac + "/deviceToken";

    Serial.println("Checking provisioning...");

    // Uses the member fbdo (this used to build a throwaway local one) so the
    // outcome can go through recordFirebaseResult() like every other RTDB call:
    // a dead link then counts toward COOLDOWN instead of being retried blindly.
    const bool lookedUp = Firebase.RTDB.getString(&fbdo, path);
    recordFirebaseResult(lookedUp);

    if (lookedUp)
    {
        const String resolved = fbdo.stringData();

        if (!resolved.isEmpty())
        {
            saveDeviceId(resolved);

            Serial.print("Provisioned Device ID: ");
            Serial.println(deviceId);
            return true;
        }
    }
    else
    {
        Serial.print("Provisioning lookup failed: ");
        Serial.println(fbdo.errorReason());
    }
    return false;
}

//==================================================
// Secure Device Auth
//==================================================

// Decides the first reachable phase from whichever credentials are
// currently persisted. Performs no network call itself.
void FirebaseManager::startAuthAttempt()
{
    loadDeviceAuthCredentials();
    authSucceededViaLegacy = false;

    if (deviceAuthRefreshToken.length() > 0)
    {
        authPhase = FirebaseAuthPhase::TRY_REFRESH_TOKEN;
    }
    else if (deviceAuthSecret.length() > 0)
    {
        authPhase = FirebaseAuthPhase::TRY_DEVICE_SECRET;
    }
    else if (!SECURE_DEVICE_AUTH_REQUIRED)
    {
        authPhase = FirebaseAuthPhase::TRY_LEGACY_SIGNUP;
    }
    else
    {
        authPhase = FirebaseAuthPhase::FAILED;
    }
}

// Advances the auth state machine by exactly one bounded step. Every WAIT_*
// state does a single, immediate Firebase.ready() check against a millis()
// deadline - never a loop, never a delay - so the 10-second bound each one
// enforces is spread across many FirebaseManager::update() calls instead of
// being spent inside one blocking call.
bool FirebaseManager::pollAuthStateMachine()
{
    // Opportunistic drain, every call, regardless of phase: reclaims a
    // bootstrap-HTTP task's completion signal even after WAIT_BOOTSTRAP_HTTP
    // has already given up on it (its own defensive timeout below). This is
    // the ONLY place bootstrapHttpTaskActive is cleared outside that
    // state's own normal success path, and it's what makes a later
    // bootstrapSecureAuth() call safe: it refuses to start a second task
    // (see its own bootstrapHttpTaskActive guard) until the OLD task is
    // CONFIRMED finished - via this drain actually observing its semaphore
    // give - never merely "we stopped waiting for it." Without this, a
    // still-running abandoned task and a freshly-started one could both
    // write bootstrapHttpResult*/bootstrapHttpMac/bootstrapHttpSecret at
    // the same time, exactly the shared-state race this design must not
    // introduce.
    if (bootstrapHttpTaskActive && authPhase != FirebaseAuthPhase::WAIT_BOOTSTRAP_HTTP &&
        bootstrapHttpDoneSemaphore != nullptr &&
        xSemaphoreTake(bootstrapHttpDoneSemaphore, 0) == pdTRUE)
    {
        bootstrapHttpTaskActive = false;
        bootstrapHttpResultToken = "";
        bootstrapHttpResultSuccess = false;
        bootstrapHttpResultDeviceId = "";
    }

    switch (authPhase)
    {
        case FirebaseAuthPhase::IDLE:
        case FirebaseAuthPhase::SUCCESS:
        case FirebaseAuthPhase::FAILED:
            return true;

        case FirebaseAuthPhase::TRY_REFRESH_TOKEN:
        {
            Serial.println("[SECURITY] Stored refresh token found");
            // A string that is not shaped like a JWT (header.payload.
            // signature) is auto-detected by this library as a bare refresh
            // token and triggers a refresh-grant sign-in directly against
            // Google's securetoken endpoint - confirmed against
            // FirebaseCore.cpp's own signer logic, not assumed from
            // documentation alone. No bootstrap call is made on this path.
            // restoreFromRefreshToken() only kicks this off now - it no
            // longer waits for the result itself.
            restoreFromRefreshToken(deviceAuthRefreshToken);
            authPhaseStartedAt = millis();
            authPhase = FirebaseAuthPhase::WAIT_REFRESH_READY;
            return false;
        }

        case FirebaseAuthPhase::WAIT_REFRESH_READY:
            if (Firebase.ready())
            {
                Serial.println("[SECURITY] Refresh-token authentication succeeded");
                onRefreshTokenAuthSucceeded();
                authPhase = FirebaseAuthPhase::SUCCESS;
                return true;
            }
            if (millis() - authPhaseStartedAt >= 10000UL)
            {
                Serial.println("[SECURITY] Refresh-token authentication failed");
                if (deviceAuthSecret.length() > 0)
                {
                    authPhase = FirebaseAuthPhase::TRY_DEVICE_SECRET;
                }
                else if (!SECURE_DEVICE_AUTH_REQUIRED)
                {
                    authPhase = FirebaseAuthPhase::TRY_LEGACY_SIGNUP;
                }
                else
                {
                    authPhase = FirebaseAuthPhase::FAILED;
                    return true;
                }
            }
            return false;

        case FirebaseAuthPhase::TRY_DEVICE_SECRET:
        {
            Serial.println("[SECURITY] Stored device secret found");
            // bootstrapSecureAuth() only resolves the MAC and starts the
            // background HTTP task now (see its own comment) - it does not
            // block. WAIT_BOOTSTRAP_HTTP below polls for that task's result.
            const bool kickedOff = bootstrapSecureAuth(deviceAuthSecret);
            if (!kickedOff)
            {
                Serial.println("[FIREBASE-AUTH] Bootstrap failed");
                authPhase = (!SECURE_DEVICE_AUTH_REQUIRED)
                    ? FirebaseAuthPhase::TRY_LEGACY_SIGNUP
                    : FirebaseAuthPhase::FAILED;
                return authPhase == FirebaseAuthPhase::FAILED;
            }
            authPhaseStartedAt = millis();
            authPhase = FirebaseAuthPhase::WAIT_BOOTSTRAP_HTTP;
            return false;
        }

        case FirebaseAuthPhase::WAIT_BOOTSTRAP_HTTP:
        {
            // Non-blocking check (0 tick timeout - a pure poll, never a
            // wait): true only once bootstrapHttpTaskFn() has already
            // called xSemaphoreGive() and is on its way to self-deleting.
            if (xSemaphoreTake(bootstrapHttpDoneSemaphore, 0) == pdTRUE)
            {
                bootstrapHttpTaskActive = false;

                if (!bootstrapHttpResultSuccess || bootstrapHttpResultToken.isEmpty())
                {
                    bootstrapHttpResultToken = "";
                    authPhase = (!SECURE_DEVICE_AUTH_REQUIRED)
                        ? FirebaseAuthPhase::TRY_LEGACY_SIGNUP
                        : FirebaseAuthPhase::FAILED;
                    return authPhase == FirebaseAuthPhase::FAILED;
                }

                if (deviceId.isEmpty() && !bootstrapHttpResultDeviceId.isEmpty())
                {
                    saveDeviceId(bootstrapHttpResultDeviceId);
                }

                // Kick-off only from here - Firebase.ready() confirmation is
                // WAIT_BOOTSTRAP_READY below, polled non-blockingly. This is
                // the only place the minted token is used, and only in the
                // main loop task.
                Firebase.setCustomToken(&config, bootstrapHttpResultToken);
                bootstrapHttpResultToken = "";
                Firebase.begin(&config, &auth);

                authPhaseStartedAt = millis();
                authPhase = FirebaseAuthPhase::WAIT_BOOTSTRAP_READY;
                return false;
            }

            // Defensive outer bound only - the task's own 5s HTTPClient
            // timeout should always resolve first. Guards against a
            // genuinely stuck task (e.g. a lower-level lwIP/mbedTLS hang
            // outside HTTPClient's own timeout) rather than waiting forever.
            // Deliberately does NOT clear bootstrapHttpTaskActive here -
            // the task may still be genuinely running, and clearing it now
            // would let a later bootstrapSecureAuth() call start a second
            // task while this one could still be mid-write to the shared
            // result fields. It's abandoned (this auth attempt moves on
            // without it) but not forgotten: the top-of-function drain
            // above safely reclaims bootstrapHttpTaskActive, only once
            // this task is CONFIRMED finished.
            if (millis() - authPhaseStartedAt >= 8000UL)
            {
                Serial.println("[FIREBASE-AUTH] Bootstrap task did not complete in time; abandoning it");
                authPhase = (!SECURE_DEVICE_AUTH_REQUIRED)
                    ? FirebaseAuthPhase::TRY_LEGACY_SIGNUP
                    : FirebaseAuthPhase::FAILED;
                return authPhase == FirebaseAuthPhase::FAILED;
            }
            return false;
        }

        case FirebaseAuthPhase::WAIT_BOOTSTRAP_READY:
            if (Firebase.ready())
            {
                Serial.println("[SECURITY] Firebase custom-token authentication succeeded");
                onBootstrapAuthSucceeded();
                authPhase = FirebaseAuthPhase::SUCCESS;
                return true;
            }
            if (millis() - authPhaseStartedAt >= 10000UL)
            {
                Serial.println("[FIREBASE-AUTH] Sign-in with minted token did not complete");
                authPhase = (!SECURE_DEVICE_AUTH_REQUIRED)
                    ? FirebaseAuthPhase::TRY_LEGACY_SIGNUP
                    : FirebaseAuthPhase::FAILED;
                return authPhase == FirebaseAuthPhase::FAILED;
            }
            return false;

        case FirebaseAuthPhase::TRY_LEGACY_SIGNUP:
            Serial.println("[SECURITY] Legacy Firebase auth compatibility mode active");
            if (Firebase.signUp(&config, &auth, "", ""))
            {
                Serial.println("Firebase SignUp OK");
            }
            else
            {
                Serial.print("Firebase SignUp Failed: ");
                Serial.println(config.signer.signupError.message.c_str());
            }
            Firebase.begin(&config, &auth);
            authPhaseStartedAt = millis();
            authPhase = FirebaseAuthPhase::WAIT_LEGACY_READY;
            return false;

        case FirebaseAuthPhase::WAIT_LEGACY_READY:
            if (Firebase.ready())
            {
                authSucceededViaLegacy = true;
                authPhase = FirebaseAuthPhase::SUCCESS;
                return true;
            }
            if (millis() - authPhaseStartedAt >= 10000UL)
            {
                authPhase = FirebaseAuthPhase::FAILED;
                return true;
            }
            return false;
    }
    return true;
}

void FirebaseManager::onRefreshTokenAuthSucceeded()
{
    // The refresh-grant response can rotate the refresh token, not just the
    // short-lived ID token. Re-persisting here (in addition to the bootstrap
    // path) ensures NVS always holds whatever token the library is currently
    // using, instead of a possibly-superseded one from a prior boot.
    const char* rotatedRefreshToken = Firebase.getRefreshToken();
    if (rotatedRefreshToken != nullptr && strlen(rotatedRefreshToken) > 0
        && deviceAuthRefreshToken != rotatedRefreshToken)
    {
        saveRefreshToken(String(rotatedRefreshToken));
        Serial.println("[SECURITY] Refresh token persisted");
    }
}

void FirebaseManager::onBootstrapAuthSucceeded()
{
    const char* newRefreshToken = Firebase.getRefreshToken();
    if (newRefreshToken != nullptr && strlen(newRefreshToken) > 0)
    {
        saveRefreshToken(String(newRefreshToken));
        Serial.println("[SECURITY] Refresh token persisted");
    }
}

bool FirebaseManager::restoreFromRefreshToken(const String& refreshToken)
{
    // Kick-off only (critical verification report, Priority 1) - this used
    // to also block here waiting for Firebase.ready(); that wait is now
    // pollAuthStateMachine()'s WAIT_REFRESH_READY state, polled non-
    // blockingly once per FirebaseManager::update() call instead.
    Firebase.setCustomToken(&config, refreshToken);
    Firebase.begin(&config, &auth);
    return true;
}

bool FirebaseManager::bootstrapSecureAuth(const String& secret)
{
    // Wire format sent to BOOTSTRAP_ENDPOINT_URL below is unchanged
    // (colon-separated, e.g. "AA:BB:CC:DD:EE:FF") - only the underlying
    // source is now the hardware-level read, not WiFi.macAddress(). Fast,
    // synchronous, no network - safe to resolve here in the main task.
    String mac = getFormattedMacAddress();
    if (mac.isEmpty())
    {
        // begin() only runs once Wi-Fi is connected, by which point
        // WiFi.mode(WIFI_STA) has long been set and the MAC is stable - this
        // remains a defensive guard, not an expected path here.
        Serial.println("[FIREBASE-AUTH] MAC address not yet valid; deferring bootstrap");
        return false;
    }

    if (bootstrapHttpTaskActive)
    {
        // Defensive only - the auth state machine never re-enters
        // TRY_DEVICE_SECRET while a previous attempt's task could still be
        // in flight, but refuse to double-launch rather than assume that.
        Serial.println("[FIREBASE-AUTH] Bootstrap task already in flight; not starting another");
        return false;
    }

    if (bootstrapHttpDoneSemaphore == nullptr)
    {
        bootstrapHttpDoneSemaphore = xSemaphoreCreateBinary();
        if (bootstrapHttpDoneSemaphore == nullptr)
        {
            Serial.println("[FIREBASE-AUTH] Unable to allocate bootstrap semaphore");
            return false;
        }
    }
    // Binary semaphore starts "empty" after creation, but clear defensively
    // in case a previous attempt's give was never consumed (there should
    // never be one outstanding here, since bootstrapHttpTaskActive already
    // guards re-entry - this is a zero-cost safety net, not a workaround).
    xSemaphoreTake(bootstrapHttpDoneSemaphore, 0);

    bootstrapHttpMac = mac;
    bootstrapHttpSecret = secret;
    bootstrapHttpResultSuccess = false;
    bootstrapHttpResultToken = "";
    bootstrapHttpResultDeviceId = "";

    // Pin to whichever core is calling this (the main loop task) rather than
    // assuming ARDUINO_RUNNING_CORE, so the two are always time-sliced on
    // the same core and never truly concurrent - see the header comment on
    // bootstrapHttpDoneSemaphore for why that matters here.
    const BaseType_t targetCore = xPortGetCoreID();
    TaskHandle_t createdHandle = nullptr;
    // 12KB: mbedTLS's TLS handshake buffers need more than the default 4KB
    // Arduino loop-task-sized stack would give a new task; 12KB is the
    // commonly-recommended floor for an HTTPS-over-TLS task on this
    // platform, with margin above the deepest call chain here (HTTPClient ->
    // NetworkClientSecure -> mbedTLS, plus FirebaseJson parsing).
    const BaseType_t created = xTaskCreatePinnedToCore(
        &FirebaseManager::bootstrapHttpTaskFn,
        "fbBootstrapHttp",
        12288,
        this,
        1,
        &createdHandle,
        targetCore);

    if (created != pdPASS || createdHandle == nullptr)
    {
        Serial.println("[FIREBASE-AUTH] Unable to start bootstrap HTTP task");
        bootstrapHttpSecret = "";
        return false;
    }

    bootstrapHttpTaskActive = true;
    Serial.println("[SECURITY] Requesting device bootstrap token (background task)");
    return true;
}

// Runs entirely off the main loop task - see the header comment on
// bootstrapHttpDoneSemaphore for the exact thread-safety boundary this
// function must never cross (no Firebase.*, no fbdo, no config/auth, no
// AutomationManager/ActuatorManager/SystemState access). Logic here is
// otherwise unchanged from the original inline implementation.
void FirebaseManager::bootstrapHttpTaskFn(void* arg)
{
    FirebaseManager* self = static_cast<FirebaseManager*>(arg);

    // Everything with a destructor below (the Strings, the TLS client, the
    // HTTP client, the JSON objects) lives inside this scope on purpose - it is
    // left unindented to keep this change small. vTaskDelete() at the bottom
    // never returns, so it never unwinds the function: anything declared at
    // function scope would have its destructor skipped and its heap leaked on
    // every run, including the ~1KB custom token. Closing the scope before the
    // give/delete runs every destructor first. The handoff writes stay inside
    // it, still strictly before the semaphore give.
    {
    // Local copies only - never touches any field other than the
    // designated bootstrapHttp*/bootstrapHttpResult* handoff fields below,
    // and only writes those once, right before signaling done.
    const String mac = self->bootstrapHttpMac;
    const String secret = self->bootstrapHttpSecret;

    bool success = false;
    int lastHttpCode = 0;
    String resultToken;
    String resultDeviceId;

    NetworkClientSecure secureClient;
    secureClient.setCACert(BOOTSTRAP_CA_CERT);

    HTTPClient http;
    // Still a single bounded blocking call - the stock Arduino HTTPClient
    // has no non-blocking POST - but it now blocks only THIS task, never the
    // main loop task, so 5s here no longer has any bearing on
    // FirebaseManager::update()'s own timing.
    http.setTimeout(5000);
    if (!http.begin(secureClient, BOOTSTRAP_ENDPOINT_URL))
    {
        Serial.println("[FIREBASE-AUTH] Unable to open bootstrap connection");
        lastHttpCode = -1;
    }
    else
    {
        http.addHeader("Content-Type", "application/json");

        FirebaseJson payload;
        payload.set("mac", mac);
        payload.set("deviceSecret", secret);
        String body;
        payload.toString(body);

        int httpCode = http.POST(body);
        lastHttpCode = httpCode;
        // The secret existed only in `payload`/`body`/the local `secret`
        // copy above, all local to this task - cleared immediately after
        // send; never logged, never echoed anywhere.
        body = "";
        payload.clear();

        if (httpCode != 200)
        {
            Serial.print("[FIREBASE-AUTH] Bootstrap rejected, HTTP ");
            Serial.println(httpCode);
            http.end();
        }
        else
        {
            String response = http.getString();
            http.end();

            FirebaseJson responseJson;
            responseJson.setJsonData(response);
            FirebaseJsonData field;

            String customToken;
            if (responseJson.get(field, "customToken")) customToken = field.stringValue;

            // deviceId is not secret (it is already the Firestore claim code
            // shown to Admins during claiming) - returned alongside the
            // token purely so a first-time device that has not yet
            // persisted a deviceId can learn the server-resolved one
            // without a separate /provisioning read.
            String resolvedDeviceId;
            if (responseJson.get(field, "deviceId")) resolvedDeviceId = field.stringValue;

            response = "";

            if (customToken.isEmpty())
            {
                Serial.println("[FIREBASE-AUTH] Bootstrap response missing token");
            }
            else
            {
                Serial.println("[SECURITY] Bootstrap succeeded");
                success = true;
                resultToken = customToken;
                resultDeviceId = resolvedDeviceId;
                customToken = "";
            }
        }
    }

    // Single-writer handoff: this task writes these fields exactly once,
    // here, before signaling - the main task must not read them until it
    // has observed the semaphore.
    self->bootstrapHttpResultSuccess = success;
    self->bootstrapHttpResultToken = resultToken;
    self->bootstrapHttpResultDeviceId = resultDeviceId;
    self->bootstrapHttpResultCode = lastHttpCode;
    self->bootstrapHttpSecret = "";
    }

    xSemaphoreGive(self->bootstrapHttpDoneSemaphore);
    vTaskDelete(nullptr);
}

void FirebaseManager::loadDeviceAuthCredentials()
{
    preferences.begin("device_auth", true);
    deviceAuthSecret = preferences.getString("device_secret", "");
    deviceAuthRefreshToken = preferences.getString("refresh_token", "");
    preferences.end();
}

void FirebaseManager::saveRefreshToken(const String& token)
{
    preferences.begin("device_auth", false);
    preferences.putString("refresh_token", token);
    preferences.end();
    deviceAuthRefreshToken = token;
}

void FirebaseManager::saveDeviceSecretFromProvisioning(const String& secret)
{
    preferences.begin("device_auth", false);
    preferences.putString("device_secret", secret);
    // A freshly-injected secret invalidates whatever refresh token (if any)
    // belonged to the previous credential generation - force a fresh
    // bootstrap on next boot rather than risk mixing old/new identity state.
    preferences.remove("refresh_token");
    preferences.end();
    deviceAuthSecret = secret;
    deviceAuthRefreshToken = "";
    Serial.println("[FIREBASE-AUTH] Device secret received via local provisioning - will bootstrap on next boot");
}

//==================================================
// Firebase transport health (timeout cascade / backoff / recovery)
//==================================================

bool FirebaseManager::isTransportFailureReason(const String& reason) const
{
    if (reason.isEmpty()) return false;

    String lower = reason;
    lower.toLowerCase();

    // Grounded in the exact strings FB_Const.h's errorReason() can return
    // (verified against the vendored library source, not guessed):
    // "response payload read timed out", "connection refused", "send
    // request failed", "not connected", "connection lost", "no http
    // server", "response read failed.", "upload timed out", "upload data
    // sent error", "incomplete SSL client data", "request timed out",
    // "gateway timeout", "bad gateway", "service unavailable", "internal
    // server error". Deliberately excludes permission/shape/application-
    // level strings like "bad request", "unauthorized", "forbidden", "not
    // found", "path not exist", "data type mismatch" - those never touch
    // firebaseHealth.
    static const char* transportMarkers[] = {
        "timed out",
        "timeout",
        "connection refused",
        "connection lost",
        "not connected",
        "no http server",
        "response read failed",
        "send request failed",
        "incomplete ssl",
        "upload data sent error",
        "bad gateway",
        "service unavailable",
        "internal server error"
    };

    for (size_t i = 0; i < sizeof(transportMarkers) / sizeof(transportMarkers[0]); i++)
    {
        if (lower.indexOf(transportMarkers[i]) >= 0) return true;
    }
    return false;
}

void FirebaseManager::recordFirebaseResult(bool success)
{
    // Deliberately separate from consecutiveSensorUploadFailures (the
    // existing presence/heartbeat counter): that one only tracks
    // writeSensors() specifically and drives its own local "[PRESENCE]
    // ..." logging, and device-offline detection is entirely backend-owned
    // (Cloud Functions evaluating lastServerSeen staleness) rather than
    // client-declared - so it's left completely untouched here. This
    // streak tracks every RTDB call site instead, purely to gate this
    // client's own retry/backoff behavior, and never writes any RTDB path
    // or triggers any notification itself.
    //
    // Every RTDB call reports here, so this is also where the per-tick call
    // budget is counted (see cloudCallsThisTick).
    if (cloudCallsThisTick < 255) cloudCallsThisTick++;

    if (success)
    {
        if (firebaseHealth == FirebaseHealthState::DEGRADED)
        {
            Serial.println("[FIREBASE-HEALTH] DEGRADED -> HEALTHY");
        }
        transportFailureStreak = 0;
        firebaseHealth = FirebaseHealthState::HEALTHY;
        // A real Firebase transaction just worked, so whatever made earlier
        // checks fail is over: the next outage starts its backoff from scratch.
        cloudStartupBackoffMs = 0;
        return;
    }

    // COOLDOWN/RECOVERING already reflect a confirmed-unhealthy transport;
    // a single call's outcome while in those states cannot un-confirm it
    // (that is what the bounded recovery attempt is for), and no further
    // Firebase calls should even occur while COOLDOWN is active.
    if (firebaseHealth == FirebaseHealthState::COOLDOWN ||
        firebaseHealth == FirebaseHealthState::RECOVERING)
    {
        return;
    }

    String reason = fbdo.errorReason();
    Serial.print("[FIREBASE-DIAG] operation failure path=");
    Serial.print(fbdo.dataPath());
    Serial.print(" http_code=");
    Serial.print(fbdo.httpCode());
    Serial.print(" error_code=");
    Serial.print(fbdo.errorCode());
    Serial.print(" tcp_connected=");
    Serial.print((unsigned)fbdo.tcpClient.connected());
    Serial.print(" firebase_error=");
    Serial.println(reason);
    logAuthDiagnostics("firebase-operation-failure");

    if (!isTransportFailureReason(reason))
    {
        // Application-level failure (permission denied, missing optional
        // path, malformed data, rejected operation command, etc.) - does
        // not indicate a broken connection, so it does not move health.
        return;
    }

    const bool firstTransportFailure = transportFailureStreak == 0;
    transportFailureStreak++;
    if (firstTransportFailure) logSocketDiagnostics("first-failure");
    Serial.print("[FIREBASE-HEALTH] transport failure #");
    Serial.print(transportFailureStreak);
    Serial.print(": ");
    Serial.print(reason);
    // Which request failed and the radio/heap state at that moment, so the
    // first failure of a run can be matched to the call that preceded it.
    Serial.print(" | path=");
    Serial.print(fbdo.dataPath());
    Serial.print(" rssi=");
    Serial.print(WiFi.RSSI());
    Serial.print("dBm heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print("/");
    Serial.println(ESP.getMaxAllocHeap());

    // The FIRST transport failure is enough to stop calling the library: it
    // has just told us the path is not usable, so the calls that would follow
    // are exactly the "repeated calls into a connection already known to be
    // down" this must avoid. update() now holds every library call until a
    // background preflight proves the path again. (The 3-failure COOLDOWN
    // below still applies once calls do resume and keep failing.)
    revokeCloudPath("transport failure", false);

    if (firebaseHealth == FirebaseHealthState::HEALTHY)
    {
        firebaseHealth = FirebaseHealthState::DEGRADED;
        Serial.println("[FIREBASE-HEALTH] HEALTHY -> DEGRADED");
    }

    if (transportFailureStreak >= TRANSPORT_FAILURE_COOLDOWN_THRESHOLD)
    {
        enterFirebaseCooldown();
    }
}

void FirebaseManager::enterFirebaseCooldown()
{
    firebaseHealth = FirebaseHealthState::COOLDOWN;
    cooldownStartedAt = millis();

    if (cooldownDurationMs == 0)
    {
        cooldownDurationMs = COOLDOWN_INITIAL_MS;
    }
    else
    {
        cooldownDurationMs = min(cooldownDurationMs * 2, COOLDOWN_MAX_MS);
        Serial.print("[FIREBASE-HEALTH] Backoff increased to ");
        Serial.print(cooldownDurationMs);
        Serial.println(" ms");
    }

    Serial.print("[FIREBASE-HEALTH] Entering cooldown ");
    Serial.print(cooldownDurationMs);
    Serial.println(" ms");
    // Logged once per cooldown entry, not per skipped loop() iteration -
    // update() otherwise returns silently on every pass while COOLDOWN
    // holds, which could be many times per second.
    Serial.println("[FIREBASE-HEALTH] Skipping low-priority sync during cooldown");
}

// Critical verification report, Priority 1. Called exactly once, from
// update(), the instant COOLDOWN's backoff window elapses. Tears down the
// possibly-stuck transport session and kicks off the auth state machine,
// then returns immediately - no network wait happens here. firebaseHealth
// is set to RECOVERING immediately so update()'s dispatch starts polling it
// on the very next call.
void FirebaseManager::beginFirebaseRecovery()
{
    firebaseHealth = FirebaseHealthState::RECOVERING;
    Serial.println("[FIREBASE-HEALTH] Recovery attempt");
    logAuthDiagnostics("recovery");

    // Close/release the possibly-stuck internal SSL client before
    // re-establishing a session - fbdo.stopWiFiClient() is the verified
    // public API for this (Firebase.h's own reset(FirebaseConfig*) was
    // considered and rejected: its doc explicitly says it resets stored
    // auth credentials, which would violate "restore auth state without
    // losing credentials").
    fbdo.stopWiFiClient();
    fbdo.clear();
    logSocketDiagnostics("before-retry");

    // Same reasoning as FirebaseManager::begin(): WiFiManager, not this
    // library, owns Wi-Fi reconnection - this only permits the library to
    // resume its own RTDB/auth transport once Wi-Fi is already back.
    Firebase.reconnectNetwork(false);

    // Re-runs exactly the same auth flow begin() uses at boot: secure
    // identity first (refresh token, then secret bootstrap - both read the
    // same persisted NVS credentials, untouched by recovery), falling back
    // to legacy anonymous auth only in migration compatibility mode. No
    // credentials are cleared or regenerated by this path. Non-blocking:
    // this only decides the first phase, it performs no network call itself.
    startAuthAttempt();
}

// Called from update() every tick while firebaseHealth stays RECOVERING.
// Advances the same state machine begin() drives, one bounded step per
// call - see pollAuthStateMachine()'s own comment for exactly how small
// each step is.
void FirebaseManager::pollFirebaseRecovery()
{
    if (!pollAuthStateMachine())
    {
        return;
    }

    if (authPhase == FirebaseAuthPhase::SUCCESS)
    {
        Serial.println("[FIREBASE-HEALTH] Recovery succeeded");
        firebaseHealth = FirebaseHealthState::HEALTHY;
        transportFailureStreak = 0;
        cooldownDurationMs = 0;
    }
    else
    {
        Serial.println("[FIREBASE-HEALTH] Recovery failed");
        enterFirebaseCooldown();
    }

    authPhase = FirebaseAuthPhase::IDLE;
}

//==================================================
// Time Synchronization
//==================================================
// PREVIOUSLY: this read /devices/{deviceId}/rtc back from RTDB and called
// rtc.adjust() on whatever it found there. That node was, in turn, only
// ever seeded by this SAME device's own DS3231 reading (see the removed
// block that used to be in initializeDatabase()) - there is no NTP client, no Android
// screen, and no Cloud Function anywhere in this system that ever writes a
// genuinely trustworthy time to that path. So the "sync" was a circular
// echo of the device's own clock, and worse: rtc.adjust() unconditionally
// clears the DS3231's lostPower flag, so a device that booted with a
// lost/garbage time would get that garbage "confirmed" as valid on the
// very next boot, permanently hiding the fact it was never actually
// correct.
//
// NOW: actual recovery (bounded SNTP, gated on Wi-Fi already being
// connected) lives in RTCManager::update(), driven independently of
// Firebase's own lifecycle - RTC validity has nothing to do with whether
// Firebase happens to be connected, only with Wi-Fi and the DS3231 itself.
// This function, called once from completeCloudStartup(), is left as a one-time status
// report at Firebase-boot time, not a second place that writes the clock -
// there's exactly one writer of rtc.adjust() now (RTCManager).
void FirebaseManager::syncRTC()
{
    Serial.println("[RTC] synchronization requested");

    if (rtcManager.hasValidTime())
    {
        // lostPower()==false only proves the DS3231 isn't reporting a
        // power-loss/oscillator-stop condition - it is not proof the
        // retained value is correct, so this is deliberately not phrased
        // as "trusted."
        Serial.println("[RTC] Existing RTC time retained; no power-loss condition reported");
        return;
    }

    Serial.println("[RTC] synchronization failed: DS3231 reports power loss; "
                    "awaiting network time recovery once Wi-Fi is available");
}

//==================================================
// Operation Protocol
//==================================================

void FirebaseManager::readCommands()
{
    static unsigned long lastCommandRead = 0;
    static unsigned long lastCommandFailure = 0;
    static bool commandBackoffActive = false;

    if (commandBackoffActive &&
        millis() - lastCommandFailure < COMMAND_FAILURE_BACKOFF_INTERVAL)
    {
        return;
    }

    if(millis() - lastCommandRead < COMMAND_READ_INTERVAL)
    {
        return;
    }

    lastCommandRead = millis();


    const unsigned long startedAt = millis();
    const bool succeeded = Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/commands/current");
    logFirebaseDuration("Operation command read", millis() - startedAt);
    recordFirebaseResult(succeeded);
    if(!succeeded)
    {
        commandBackoffActive = true;
        lastCommandFailure = millis();
        return;
    }
    commandBackoffActive = false;

    FirebaseJsonData data;

    uint16_t requestId = 0;
    uint32_t requestTimestamp = 0;
    uint32_t protocolVersion = 0;

    String operationString;
    String actionString;

    fbdo.jsonObject().get(data, "requestId");
    requestId = data.intValue;

    fbdo.jsonObject().get(data, "operation");
    operationString = data.stringValue;

    fbdo.jsonObject().get(data, "action");
    actionString = data.stringValue;

    fbdo.jsonObject().get(data, "requestTimestamp");
    requestTimestamp = data.intValue;

    fbdo.jsonObject().get(data, "protocolVersion");
    protocolVersion = data.intValue;

    //--------------------------------------------------
    // Duplicate Request
    //--------------------------------------------------
    // Checked before any validation/rejection path so a request that fails
    // protocol/operation/action parsing (e.g. a malformed document) is still
    // deduplicated by its requestId instead of being reprocessed - and
    // re-rejected - on every poll.

    // Baseline pass: the document that is already sitting in /commands/current
    // when the device boots (or comes back from a long outage) was written
    // before this device could act on it, and this document is never deleted.
    // Record it as handled instead of executing it. Only a request that shows
    // up AFTER this pass is treated as a live command.
    if(!currentCommandBaselined)
    {
        currentCommandBaselined = true;
        systemState.lastProcessedRequestId = requestId;
        lastProcessedRequestTimestamp = requestTimestamp;
        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("CMD");
            Serial.print("BASELINE | ignored stale current command id=");
            Serial.print(requestId);
            Serial.print(" ts=");
            Serial.println(requestTimestamp);
        }
        return;
    }

    if(isDuplicateRequest(requestId, requestTimestamp))
    {
        return;
    }

    // A fresh, non-duplicate /commands/current request: every write to this
    // path today originates from the app (REFILL/RESET_SAFETY/pH-EC trigger
    // buttons), so reaching here - independent of whatever validation
    // happens below - is genuine manual interaction.
    lastManualCommandActivityAt = millis();

    // Section 12: command receipt, before whatever validation/ownership
    // below decides to accept or reject it.
    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("CMD");
        Serial.print("RX ");
        Serial.print(operationString);
        Serial.print(" | id=");
        Serial.print(requestId);
        Serial.print(" ts=");
        Serial.println(requestTimestamp);
    }

    //--------------------------------------------------
    // Lifecycle Ownership
    //--------------------------------------------------
    // systemState.operationRequest belongs to exactly one request - manual or
    // automatic - until it has been published and archived back to IDLE. An
    // incoming command must never overwrite that request while it still owns
    // the lifecycle object, so it is deferred and re-evaluated on a later
    // poll instead of being validated/rejected now.

    if(isOperationLifecycleOwned())
    {
        if(lastDeferredCommandRequestId != requestId)
        {
            Serial.print("[COMMAND] Deferred requestId=");
            Serial.print(requestId);
            Serial.println(": operation lifecycle busy");
            lastDeferredCommandRequestId = requestId;
        }

        return;
    }

    // From here every path below either rejects or accepts this request, and
    // both record its requestId as the last processed one (see
    // rejectOperationRequest()/createOperationRequest()). The timestamp is the
    // other half of the dedupe key, so record it at the same point.
    lastProcessedRequestTimestamp = requestTimestamp;

    //--------------------------------------------------
    // Protocol Validation
    //--------------------------------------------------

    if(protocolVersion != 1)
    {
        rejectOperationRequest(
            requestId,
            "Unsupported protocol version.");

        return;
    }

    if(requestTimestamp == 0)
    {
        rejectOperationRequest(
            requestId,
            "Invalid request timestamp.");

        return;
    }

    OperationType operation =
        toOperationType(
            operationString);

    if(operation == OperationType::NONE)
    {
        rejectOperationRequest(
            requestId,
            "Invalid operation.");

        return;
    }

    OperationAction action =
        toOperationAction(
            actionString);

    if(action == OperationAction::NONE)
    {
        rejectOperationRequest(
            requestId,
            "Invalid action.");

        return;
    }

    //--------------------------------------------------
    // Runtime Validation
    //--------------------------------------------------

    String reason;

    if(!validateOperationRequest(
        operation,
        action,
        reason))
    {
        rejectOperationRequest(
            requestId,
            reason.c_str());

        return;
    }

    //--------------------------------------------------
    // Accept Request
    //--------------------------------------------------

    automationManager.createOperationRequest(
    requestId,
    operation,
    action,
    RequestSource::MANUAL);
}


void FirebaseManager::readActuatorCommands()
{
    static unsigned long lastCommandRead = 0;
    static unsigned long lastCommandFailure = 0;
    static bool commandBackoffActive = false;

    if (commandBackoffActive &&
        millis() - lastCommandFailure < COMMAND_FAILURE_BACKOFF_INTERVAL)
    {
        return;
    }
    if(millis() - lastCommandRead < COMMAND_READ_INTERVAL) return;
    lastCommandRead = millis();

    const unsigned long startedAt = millis();
    const bool succeeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands");
    logFirebaseDuration("Actuator command read", millis() - startedAt);
    recordFirebaseResult(succeeded);
    if(!succeeded)
    {
        commandBackoffActive = true;
        lastCommandFailure = millis();
        return;
    }
    commandBackoffActive = false;

    FirebaseJson& snapshot = fbdo.jsonObject();
    FirebaseJsonData jsonData;

    // Check for manualMode flag
    if (snapshot.get(jsonData, "manualMode"))
    {
        const bool newManualMode = jsonData.boolValue;
        if (newManualMode != systemState.manualMode)
        {
            Serial.println(newManualMode ? "[MANUAL] Manual Mode enabled" : "[MANUAL] Manual Mode disabled");
            // Enabling Manual Mode is itself a legitimate manual interaction
            // and starts the 15-minute inactivity lease even if no actuator
            // command follows immediately - see
            // ActuatorManager::update()'s expiry check and
            // MANUAL_MODE_INACTIVITY_TIMEOUT_MS in Config.h. Refreshing on
            // the disable edge too is harmless (the expiry check only runs
            // while manualMode is true).
            lastManualCommandActivityAt = millis();
        }
        systemState.manualMode = newManualMode;
    }
    // Raw observation from THIS poll's snapshot only - deliberately not
    // "was a request ever seen," since the whole point of
    // provisioningCommandPendingDelete below is to distinguish "still there"
    // from "gone now" on each successive poll.
    const bool startProvisioningObserved =
        snapshot.get(jsonData, "startProvisioning") && jsonData.success && jsonData.boolValue;
    const bool automationTestModeChanged = applyAutomationTestModeCommand(snapshot);

    const bool dispatchCommands = actuatorCommandsPrimed;
    consumeActuatorCommandSnapshot(snapshot, dispatchCommands);
    if (!actuatorCommandsPrimed)
    {
        actuatorCommandsPrimed = true;
        Serial.println("[MANUAL] Existing actuator commands consumed as reconnect baseline");
        if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        {
            debugManager.printLogPrefix("CMD");
            Serial.println("BASELINE actuator commands after reconnect");
        }
    }

    if(automationTestModeChanged)
    {
        setAutomationTestMode(systemState.automationTestSubsystem, true);
    }

    // Developer-only manual provisioning trigger.
    // Wi-Fi credentials are intentionally not accepted through Firebase/RTDB.
    //
    // commands/startProvisioning is meant to be consumed exactly once.
    // Previously the delete's result was never checked, so a failed/unacked
    // delete left the stale `true` in RTDB to be re-read - and re-acted on -
    // by the very next 1.5s poll, by the first poll after this device
    // reconnected on the new network once setup finished, or after any
    // reboot. The two branches below turn that into an explicit handshake:
    // AP mode is only ever entered once the command is confirmed consumed,
    // either just now or on a later poll - never on an unconfirmed delete.
    if (provisioningCommandPendingDelete)
    {
        if (startProvisioningObserved)
        {
            Serial.println("[PROVISION] command delete unconfirmed; retrying");
            const bool deleted = Firebase.RTDB.deleteNode(
                &fbdo, deviceRoot() + "/commands/startProvisioning");
            recordFirebaseResult(deleted);
            if (!deleted)
            {
                // Still ambiguous - leave provisioningCommandPendingDelete set
                // and try again on the next successful poll.
                return;
            }
            Serial.println("[PROVISION] command delete confirmed");
        }
        else
        {
            // The node is gone now, even though the delete that (supposedly)
            // removed it was never confirmed - the earlier attempt must have
            // actually succeeded server-side despite the lost/failed ack.
            Serial.println("[PROVISION] command absent after pending delete; treating as consumed");
        }

        provisioningCommandPendingDelete = false;
        beginManualProvisioningAfterCommandConsumed();
        return;
    }

    if (startProvisioningObserved)
    {
        Serial.println("[PROVISION] start command received");
        const unsigned long provisioningWriteStartedAt = millis();
        const bool provisioningPublished = Firebase.RTDB.setBool(
            &fbdo, deviceRoot() + "/status/provisioning", true);
        logFirebaseDuration("Provisioning state write", millis() - provisioningWriteStartedAt);
        recordFirebaseResult(provisioningPublished);
        if (!provisioningPublished)
        {
            Serial.println("[WIFI] Unable to publish provisioning state before cloud suspension");
            // Retain the command so the next poll starts this handshake fresh.
            // Entering AP mode before backend grace exists would create a
            // false offline event.
            return;
        }

        const bool deleted = Firebase.RTDB.deleteNode(
            &fbdo, deviceRoot() + "/commands/startProvisioning");
        recordFirebaseResult(deleted);
        if (!deleted)
        {
            Serial.println("[PROVISION] command delete unconfirmed; retrying");
            provisioningCommandPendingDelete = true;
            return;
        }
        Serial.println("[PROVISION] command delete confirmed");

        beginManualProvisioningAfterCommandConsumed();
    }
}

void FirebaseManager::beginManualProvisioningAfterCommandConsumed()
{
    if (!suspendedForProvisioning)
    {
        suspendedForProvisioning = true;
    }
    systemState.firebaseConnected = false;
    wasFirebaseConnected = false;
    Serial.println("[PROVISION] entering manual AP mode");
    wifiManager.startManualProvisioning();
}

void FirebaseManager::primeActuatorCommands()
{
    const bool baselineRead = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands");
    recordFirebaseResult(baselineRead);
    if (!baselineRead)
    {
        Serial.println("[MANUAL] Command baseline deferred until Firebase is readable");
        return;
    }

    FirebaseJson& snapshot = fbdo.jsonObject();
    const bool automationTestModeChanged = applyAutomationTestModeCommand(snapshot);
    consumeActuatorCommandSnapshot(snapshot, false);
    if(automationTestModeChanged)
    {
        setAutomationTestMode(systemState.automationTestSubsystem, true);
    }
    actuatorCommandsPrimed = true;
    Serial.println("[MANUAL] Existing actuator commands consumed as boot baseline");
    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("CMD");
        Serial.println("BASELINE actuator commands after boot");
    }
}

bool FirebaseManager::applyAutomationTestModeCommand(FirebaseJson& snapshot)
{
    FirebaseJsonData data;

    // Developer-only Grow Light mock time - read unconditionally, ahead of
    // the enabled/subsystem branches below, so a value stays captured in
    // systemState even while a different (or no) Automation Test Mode is
    // currently selected. Safe either way:
    // AutomationManager::growLightMockTimeActive() re-checks
    // automationTestSubsystem == GROW_LIGHT live on every use, so a stored
    // value never takes effect on its own - the app isn't required to
    // delete it when switching test modes away from Grow Light. Minutes
    // are clamped, not rejected, since this is a developer convenience
    // field, not a production safety setting.
    if(snapshot.get(data, "automationTestMode/mockGrowLightTimeEnabled") && data.success)
    {
        systemState.mockGrowLightTimeEnabled = data.boolValue;
    }
    if(snapshot.get(data, "automationTestMode/mockGrowLightMinutes") && data.success)
    {
        int minutes = data.intValue;
        if(minutes < 0) minutes = 0;
        if(minutes > 1439) minutes = 1439;
        systemState.mockGrowLightMinutes = (uint16_t)minutes;
    }

    // A stale enabled=true left in RTDB from before this boot must never
    // re-isolate the automation - see devCommandsCleared.
    if(!devCommandsCleared)
    {
        return false;
    }

    if(!snapshot.get(data, "automationTestMode/enabled") || !data.success)
    {
        return false;
    }

    const AutomationTestSubsystem previous = systemState.automationTestSubsystem;
    const bool enabled = data.boolValue;
    if(!enabled)
    {
        setAutomationTestMode(AutomationTestSubsystem::NONE, false);
        return previous != systemState.automationTestSubsystem;
    }

    if(!snapshot.get(data, "automationTestMode/subsystem") || !data.success)
    {
        Serial.println("[AUTO-TEST] rejected command: enabled mode has no subsystem");
        return false;
    }

    String value = data.stringValue;
    value.trim();
    value.toUpperCase();

    AutomationTestSubsystem subsystem = AutomationTestSubsystem::NONE;
    bool valid = true;
    if(value == "STARTUP") subsystem = AutomationTestSubsystem::STARTUP;
    else if(value == "REFILL") subsystem = AutomationTestSubsystem::REFILL;
    else if(value == "PH") subsystem = AutomationTestSubsystem::PH;
    else if(value == "EC") subsystem = AutomationTestSubsystem::EC;
    else if(value == "COOLING") subsystem = AutomationTestSubsystem::COOLING;
    else if(value == "FOGGING") subsystem = AutomationTestSubsystem::FOGGING;
    else if(value == "CANOPY") subsystem = AutomationTestSubsystem::CANOPY;
    else if(value == "GROW_LIGHT") subsystem = AutomationTestSubsystem::GROW_LIGHT;
    else valid = false;

    if(!valid)
    {
        Serial.print("[AUTO-TEST] rejected unknown subsystem: ");
        Serial.println(value);
        return false;
    }

    setAutomationTestMode(subsystem, false);
    return previous != systemState.automationTestSubsystem;
}

void FirebaseManager::setAutomationTestMode(
    AutomationTestSubsystem subsystem,
    bool publishAcknowledgement)
{
    if(subsystem != systemState.automationTestSubsystem)
    {
        // Always printed regardless of the previous/new focus filter (see
        // DebugManager's Serial Monitor Focus Mode) - this IS the entry/exit
        // announcement the focus filter itself keys off of, so it can never
        // be suppressed by it.
        Serial.print("[AUTO-TEST] ");
        Serial.print(automationTestSubsystemName(systemState.automationTestSubsystem));
        Serial.print(" -> ");
        Serial.println(automationTestSubsystemName(subsystem));

        systemState.automationTestSubsystem = subsystem;
    }

    if(publishAcknowledgement && WiFi.status() == WL_CONNECTED && Firebase.ready())
    {
        FirebaseJson acknowledgement;
        acknowledgement.set("enabled", subsystem != AutomationTestSubsystem::NONE);
        acknowledgement.set("subsystem", automationTestSubsystemName(subsystem));
        updateJson(deviceRoot() + "/status/automationTestMode", acknowledgement);
    }
}

void FirebaseManager::consumeActuatorCommandSnapshot(FirebaseJson& snapshot, bool dispatchCommands)
{
    bool hasCommand[ACTUATOR_COUNT] = { false };
    bool states[ACTUATOR_COUNT] = { false };
    String sources[ACTUATOR_COUNT];
    uint64_t timestamps[ACTUATOR_COUNT] = { 0 };
    uint8_t speeds[ACTUATOR_COUNT];
    // Explicit one-shot override intent (see Types.h ActuatorCommand). Absent
    // on any command that predates this field or wasn't a confirmed override -
    // defaults to false, i.e. normal soft-rule enforcement, exactly like a
    // missing "speed" already defaults to 100 below.
    bool overrides[ACTUATOR_COUNT] = { false };
    FirebaseJsonData data;

    // Parse the complete snapshot first. deleteNode() reuses fbdo and would
    // otherwise invalidate snapshot while the remaining actuators are parsed.
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        const String name = getActuatorName((Actuator)i);
        speeds[i] = 100;

        if (snapshot.get(data, name + "/state") && data.success)
        {
            hasCommand[i] = true;
            states[i] = data.boolValue;
        }
        if (snapshot.get(data, name + "/source") && data.success)
        {
            sources[i] = data.stringValue;
        }
        if (snapshot.get(data, name + "/timestamp") && data.success)
        {
            const double rawTimestamp = data.doubleValue;
            if (isfinite(rawTimestamp) && rawTimestamp > 0)
            {
                timestamps[i] = static_cast<uint64_t>(rawTimestamp);
            }
        }
        if (snapshot.get(data, name + "/speed") && data.success)
        {
            speeds[i] = static_cast<uint8_t>(constrain(data.intValue, 0, 100));
        }
        if (snapshot.get(data, name + "/overrideRequested") && data.success)
        {
            overrides[i] = data.boolValue;
        }
    }

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        if (!hasCommand[i]) continue;

        const Actuator actuator = static_cast<Actuator>(i);
        const String commandPath = deviceRoot() + "/commands/" + getActuatorName(actuator);
        const uint64_t previousTimestamp = lastActuatorCommandTimestamps[i];
        // "Not the one already handled", not "newer than the last one". The
        // timestamp is the sender's own wall clock, so an ordering
        // comparison let one phone with a fast clock push the persisted
        // watermark ahead and silently drop every later command from a
        // correctly-timed phone (each also being deleted below). Replay
        // protection is unchanged: a command re-read after a failed
        // delete carries the identical timestamp, still matching the
        // persisted watermark.
        const bool isNew = timestamps[i] != previousTimestamp;

        if (dispatchCommands && sources[i] == "manual")
        {
            Serial.print("[MANUAL] command key=");
            Serial.print(getActuatorName(actuator));
            Serial.print(" state=");
            Serial.print(states[i] ? 1 : 0);
            Serial.print(" timestamp=");
            Serial.println((unsigned long long)timestamps[i]);
            Serial.print("[MANUAL] previousTimestamp=");
            Serial.println((unsigned long long)previousTimestamp);
            Serial.print("[MANUAL] freshness=");
            Serial.println(isNew ? "fresh" : "stale");
        }

        if (isNew)
        {
            lastActuatorCommandTimestamps[i] = timestamps[i];
            saveActuatorCommandTimestamp(actuator, timestamps[i]);
            // /commands/{actuator} is ADMIN-write-only (database.rules.json)
            // and only ever written by the app, so any fresh write observed
            // here - regardless of source string or dispatchCommands (the
            // reconnect baseline pass) - is genuine manual interaction.
            if (dispatchCommands) lastManualCommandActivityAt = millis();
        }

        if (!dispatchCommands)
        {
            Serial.print("[MANUAL] Baseline ignored: ");
            Serial.println(getActuatorName(actuator));
        }
        else if (timestamps[i] == 0)
        {
            Serial.print("[MANUAL] Rejected command without valid timestamp: ");
            Serial.println(getActuatorName(actuator));
        }
        else if (sources[i] == "android" && !states[i] && isNew)
        {
            // Disabling Android manual mode emits cleanup OFF events. Treat
            // them as manual stops so ActuatorManager can stop only actuators
            // that are actually manual-owned; automatic actuators are immune.
            actuatorManager.requestCommand(
                actuator,
                false,
                "manual",
                static_cast<double>(timestamps[i]),
                speeds[i]);
        }
        else if (sources[i] != "manual")
        {
            Serial.print("[MANUAL] Ignored unsupported actuator command source: ");
            Serial.println(sources[i]);
        }
        else if (isNew)
        {
            actuatorManager.requestCommand(
                actuator,
                states[i],
                sources[i],
                static_cast<double>(timestamps[i]),
                speeds[i],
                "",
                "",
                overrides[i]);
        }

        // Commands are events, not desired-state storage. Removing the event
        // after consumption prevents reconnect/reboot replay; the persisted
        // watermark remains the fallback if this deletion fails.
        const bool commandRemoved = Firebase.RTDB.deleteNode(&fbdo, commandPath);
        recordFirebaseResult(commandRemoved);
        if (!commandRemoved)
        {
            Serial.print("[MANUAL] Command cleanup failed: ");
            Serial.println(getActuatorName(actuator));
        }
    }
}

//==================================================
// Operation Protocol Helpers
//==================================================    
bool FirebaseManager::hasActiveOperation() const
{
    switch(systemState.operationRequest.state)
    {
        case RequestState::IDLE:

        case RequestState::COMPLETED:

        case RequestState::FAILED:

        case RequestState::REJECTED:

            return false;

        default:

            return true;
    }
}

// Unlike hasActiveOperation() (ACCEPTED/RUNNING only), this also covers
// terminal states that syncOperationState() has not yet published/archived.
// systemState.operationRequest belongs to exactly one request - manual or
// automatic - until resetCurrentOperation() returns it to IDLE, so command
// intake must treat any non-IDLE state as owned and defer instead of
// overwriting it.
bool FirebaseManager::isOperationLifecycleOwned() const
{
    return systemState.operationRequest.state !=
        RequestState::IDLE;
}

// A request is a duplicate only when BOTH its requestId and its
// requestTimestamp match the last processed one. requestId alone isn't
// unique across senders (two phones can generate the same id), and
// treating that as a duplicate silently dropped a valid command from the
// second phone. A genuine re-read of the same document still matches on
// both fields, so the persistent-document replay protection is unchanged.
bool FirebaseManager::isDuplicateRequest(uint16_t requestId, uint32_t requestTimestamp) const
{
    return requestId == systemState.lastProcessedRequestId &&
           requestTimestamp == lastProcessedRequestTimestamp;
}

bool FirebaseManager::validateOperationRequest(
    OperationType operation,
    OperationAction action,
    String& reason)
{
    //--------------------------------------------------
    // Operation
    //--------------------------------------------------

    if(operation ==
       OperationType::NONE)
    {
        reason =
            "Invalid operation";

        return false;
    }

    //--------------------------------------------------
    // Action
    //--------------------------------------------------

    if(action ==
       OperationAction::NONE)
    {
        reason =
            "Invalid action";

        return false;
    }

    //--------------------------------------------------
    // Existing Operation
    //--------------------------------------------------

    if(hasActiveOperation())
    {
        reason =
            "Operation already active";

        return false;
    }

    // Validate the requested direction/need here as well as at the actuator
    // layer. Operation requests are an API and must not be less strict than the
    // direct manual actuator path.
    if (operation == OperationType::REFILL)
    {
        const SafetyResult safety = safetyManager.canRefill();
        if (safety != SafetyResult::SAFE)
        {
            reason = safetyManager.getSafetyReason(safety);
            return false;
        }
        if (sensors.waterLevelCm > systemState.refillStartLevelCm)
        {
            reason = "Refill rejected: Water level is above the refill start threshold.";
            return false;
        }
    }
    else if (operation == OperationType::PH_UP || operation == OperationType::PH_DOWN)
    {
        const SafetyResult safety = safetyManager.canDosePH();
        if (safety != SafetyResult::SAFE)
        {
            reason = safetyManager.getSafetyReason(safety);
            return false;
        }
        if (operation == OperationType::PH_UP && sensors.ph >= systemState.minPH)
        {
            reason = "pH Up rejected: Current pH does not require an increase.";
            return false;
        }
        if (operation == OperationType::PH_DOWN && sensors.ph <= systemState.maxPH)
        {
            reason = "pH Down rejected: Current pH does not require a decrease.";
            return false;
        }
    }
    else if (operation == OperationType::EC_CORRECTION)
    {
        const bool low = sensors.ec < systemState.minEC;
        const bool high = sensors.ec > systemState.maxEC;
        const SafetyResult safety = low
            ? safetyManager.canDoseEC()
            : safetyManager.canDiluteEC();
        if (safety != SafetyResult::SAFE)
        {
            reason = safetyManager.getSafetyReason(safety);
            return false;
        }
        if (!low && !high)
        {
            reason = "EC correction rejected: Current EC is within the acceptable range.";
            return false;
        }
    }
    else if (operation == OperationType::RESET_SAFETY)
    {
        if (!systemState.safetyLock && !systemState.phSubsystemLocked &&
            !systemState.ecSubsystemLocked && !systemState.refillSubsystemLocked &&
            !systemState.coolingSubsystemLocked)
        {
            reason = "No safety subsystem is locked.";
            return false;
        }
    }

    return true;
}

void FirebaseManager::rejectOperationRequest(
    uint16_t requestId,
    const char* reason)
{
    // Section 12: single chokepoint for every /commands/current rejection in
    // this file, so one call site covers all of them. The operation name
    // itself is not available here (some callers reject before it is even
    // parsed, e.g. an unsupported protocol version) - id + reason is what
    // every caller can always provide.
    if (debugManager.atLeast(LogLevel::LEVEL_NORMAL))
    {
        debugManager.printLogPrefix("CMD");
        Serial.print("REJECT id=");
        Serial.print(requestId);
        Serial.print(" | reason=");
        Serial.println(reason);
    }

    OperationRequest& request =
        systemState.operationRequest;

    //--------------------------------------------------
    // Identity
    //--------------------------------------------------

    request.requestId = requestId;
    request.operation = OperationType::NONE;
    request.action = OperationAction::NONE;
    request.source = RequestSource::MANUAL;

    //--------------------------------------------------
    // State
    //--------------------------------------------------

    request.state = RequestState::REJECTED;

    strncpy(
        request.reason,
        reason,
        sizeof(request.reason) - 1);

    request.reason[sizeof(request.reason) - 1] = '\0';

    //--------------------------------------------------
    // Timestamps
    //--------------------------------------------------

    unsigned long now = millis();

    request.requestTimestamp = now;
    request.acceptedTimestamp = 0;
    request.startedTimestamp = 0;
    request.completedTimestamp = now;
    request.lastUpdatedTimestamp = now;

    //--------------------------------------------------
    // Bookkeeping
    //--------------------------------------------------

    systemState.lastProcessedRequestId = requestId;
}


bool FirebaseManager::writeCurrentOperation()
{

    FirebaseJson json;

    OperationRequest& request =
        systemState.operationRequest;

    //--------------------------------------------------
    // Identity
    //--------------------------------------------------

    json.set(
        "requestId",
        request.requestId);

    json.set(
        "operation",
        operationToString(request.operation));

    json.set(
        "action",
        actionToString(request.action));

    json.set(
        "source",
        request.source == RequestSource::MANUAL ?
        "MANUAL" :
        "AUTOMATIC");

    //--------------------------------------------------
    // State
    //--------------------------------------------------

    json.set(
        "state",
        requestStateToString(request.state));

    json.set(
        "reason",
        request.reason);

    //--------------------------------------------------
    // Timestamps
    //--------------------------------------------------

    json.set(
        "requestTimestamp",
        request.requestTimestamp);

    json.set(
        "acceptedTimestamp",
        request.acceptedTimestamp);

    json.set(
        "startedTimestamp",
        request.startedTimestamp);

    json.set(
        "completedTimestamp",
        request.completedTimestamp);

    json.set(
        "lastUpdatedTimestamp",
        request.lastUpdatedTimestamp);

    //--------------------------------------------------
    // Protocol
    //--------------------------------------------------

    json.set(
        "protocolVersion",
        1);

    return writeJson(
        deviceRoot() + "/operations/current",
        json);
}

bool FirebaseManager::archiveCurrentOperation()
{
    OperationRequest& request =
        systemState.operationRequest;

    FirebaseJson json;

    //--------------------------------------------------
    // Identity
    //--------------------------------------------------

    json.set(
        "requestId",
        request.requestId);

    json.set(
        "operation",
        operationToString(request.operation));

    json.set(
        "action",
        actionToString(request.action));

    json.set(
        "source",
        request.source == RequestSource::MANUAL ?
        "MANUAL" :
        "AUTOMATIC");

    //--------------------------------------------------
    // Result
    //--------------------------------------------------

    json.set(
        "state",
        requestStateToString(request.state));

    json.set(
        "reason",
        request.reason);

    //--------------------------------------------------
    // Timeline
    //--------------------------------------------------

    json.set(
        "requestTimestamp",
        request.requestTimestamp);

    json.set(
        "acceptedTimestamp",
        request.acceptedTimestamp);

    json.set(
        "startedTimestamp",
        request.startedTimestamp);

    json.set(
        "completedTimestamp",
        request.completedTimestamp);

    json.set(
        "lastUpdatedTimestamp",
        request.lastUpdatedTimestamp);

    //--------------------------------------------------
    // Protocol
    //--------------------------------------------------

    json.set(
        "protocolVersion",
        1);

    String path =
        deviceRoot() +
        "/operations/history/" +
        String(request.requestId);

    return writeJson(
        path,
        json);
}

void FirebaseManager::updateOperationState(
    RequestState state,
    const char* reason)
{
    OperationRequest& request =
        systemState.operationRequest;

    unsigned long now = millis();

    request.state = state;
    request.lastUpdatedTimestamp = now;

    if(reason != nullptr)
    {
        strncpy(
            request.reason,
            reason,
            sizeof(request.reason) - 1);

        request.reason[
            sizeof(request.reason) - 1] = '\0';
    }

    if(state == RequestState::RUNNING &&
       request.startedTimestamp == 0)
    {
        request.startedTimestamp = now;
    }

    if(state == RequestState::COMPLETED ||
       state == RequestState::FAILED ||
       state == RequestState::REJECTED)
    {
        request.completedTimestamp = now;
    }
}

void FirebaseManager::resetCurrentOperation()
{
    OperationRequest& request =
        systemState.operationRequest;

    //--------------------------------------------------
    // Identity
    //--------------------------------------------------

    request.requestId = 0;
    request.operation = OperationType::NONE;
    request.action = OperationAction::NONE;
    request.source = RequestSource::NONE;

    //--------------------------------------------------
    // State
    //--------------------------------------------------

    request.state = RequestState::IDLE;

    request.reason[0] = '\0';

    //--------------------------------------------------
    // Timestamps
    //--------------------------------------------------

    request.requestTimestamp = 0;
    request.acceptedTimestamp = 0;
    request.startedTimestamp = 0;
    request.completedTimestamp = 0;
    request.lastUpdatedTimestamp = 0;
}

//==================================================
// Device Uploads
//==================================================

bool FirebaseManager::writeSensors(bool force, const SensorData* snapshot)
{
    if(!force && !isSensorUploadDue())
    {
        return true;
    }
    // A failed write is still one heartbeat attempt. Keep retries on the normal
    // SENSOR_UPLOAD_INTERVAL_MS (1s) cadence instead of hammering RTDB on
    // every loop iteration.
    if (!force) lastSensorUploadAttempt = millis();

    FirebaseJson json;
    const SensorData& publishedSensors = snapshot ? *snapshot : sensors;

    // CONFIRMED BUG FIX (effective sensor data vs Firebase sensor sync):
    // phLastPublishedValue/ecLastPublishedValue/ecLastPublishedTds below
    // exist to survive a momentary invalid READING from the same live
    // source (see their own comment), not a SOURCE change. Without this,
    // a stale mock payload falling back to PHYSICAL (or entering PH
    // INVALID_TEST_HOLD) left the old mock ph/ec sitting in these caches
    // forever, so /sensors kept reporting it as current even though
    // automation had already correctly treated that reading as invalid.
    // Cleared exactly once, on the falling edge out of mock (detected via
    // the existing isUsingEffectiveMockSensors() signal - no new sensor
    // logic), so a genuine transient invalid PHYSICAL reading still keeps
    // its own last-known-good value exactly as before this fix.
    const bool sourceIsMockNow = sensorManager.isUsingEffectiveMockSensors();
    if (lastPublishedSourceWasMock && !sourceIsMockNow)
    {
        systemState.phLastPublishedValue = NAN;
        systemState.ecLastPublishedValue = NAN;
        systemState.ecLastPublishedTds = NAN;
    }
    lastPublishedSourceWasMock = sourceIsMockNow;

    //--------------------------------------------------
    // Environment
    //--------------------------------------------------

    if (!isnan(publishedSensors.temperature)) json.set("airTemperature", publishedSensors.temperature);
    if (!isnan(publishedSensors.humidity)) json.set("humidity", publishedSensors.humidity);
    // Health/staleness pair for the two fields above - see the automation
    // resilience pass report. airTemperature/humidity may now be a held
    // last-good value rather than a fresh sample; dhtStale is what lets the
    // app show "LAST KNOWN / STALE" instead of misrepresenting it as a
    // current measurement. Always published (booleans have no NaN state).
    json.set("dhtAvailable", publishedSensors.dhtAvailable);
    json.set("dhtStale", publishedSensors.dhtStale);

    //--------------------------------------------------
    // Reservoir
    //--------------------------------------------------

    if (!isnan(publishedSensors.waterTemp)) json.set("waterTemperature", publishedSensors.waterTemp);
    // waterLevel: derived 0-100 working percentage (water-depth model - see
    // Config.h's "Water Reservoir Geometry"). waterLevelCm/waterVolumeLiters
    // are the new authoritative depth/volume fields; waterLevelDistanceCm is
    // the raw HC-SR04 distance, published here (not diagnostics-only
    // anymore) so Android can show it without the Sensor Test flow.
    if (isfinite(publishedSensors.waterLevel)) json.set("waterLevel", publishedSensors.waterLevel);
    if (isfinite(publishedSensors.waterLevelCm)) json.set("waterLevelCm", publishedSensors.waterLevelCm);
    if (isfinite(publishedSensors.waterVolumeLiters)) json.set("waterVolumeLiters", publishedSensors.waterVolumeLiters);
    if (isfinite(publishedSensors.waterLevelDistanceCm)) json.set("waterLevelDistanceCm", publishedSensors.waterLevelDistanceCm);
    // Always published (boolean, no NaN state) - same shape as dhtAvailable/
    // dhtStale, so the app can eventually show "reading held (fogger
    // running)" instead of silently displaying a frozen number as if it
    // were live.
    json.set("waterLevelHeldForFogger", publishedSensors.waterLevelHeldForFogger);

    //--------------------------------------------------
    // Nutrient
    //--------------------------------------------------

    // Telemetry/control separation (Stage 1 of the sensor architecture
    // redesign): Firebase now always publishes the CURRENT filtered
    // sensors.ph, including while a pH correction is
    // DOSING_PH/STABILIZING_PH - it no longer holds/freezes at a
    // checkpoint value just because a correction is active. Automation's
    // own stability/confirmation logic (sensorManager.isPhCurrentlyStable(),
    // systemState.phStableSince/phStableCheckpointPublished in
    // AutomationManager::handleStabilizingPH()) is completely independent
    // of this and is untouched by this file. phLastPublishedValue is kept
    // only as a NaN-fallback cache: writeSensors() overwrites the whole
    // /sensors node via setJSON() (not a merge), so a transient invalid
    // reading would otherwise DELETE the "ph" field instead of leaving the
    // last known-good value in place - same behavior as before this
    // change, just no longer gated on correction state.
    if (!isnan(publishedSensors.ph))
    {
        systemState.phLastPublishedValue = publishedSensors.ph;
    }
    if (!isnan(systemState.phLastPublishedValue)) json.set("ph", systemState.phLastPublishedValue);
    // pH hardware-fault state - same shape as dhtAvailable/dhtStale above:
    // always published (booleans, no NaN state), so Android can
    // distinguish "no reading has confirmed yet" from "a rail-proximity
    // hardware fault is confirmed" instead of both collapsing to the same
    // bare "--". See SensorManager::readPH()'s and Types.h's own comments
    // for the full detection design. Deliberately distinct from
    // phOutOfRange (AlertManager) - a fault is a hardware problem, an
    // out-of-range pH is a normal, dosing-correctable chemistry state.
    json.set("phFault", publishedSensors.phFault);
    json.set("phAvailable", publishedSensors.phAvailable);

    // tds is derived from the same EC reading, so it is cached/published in
    // lockstep with ec below. See the pH block's own comment above - same
    // Stage 1 change: always reflects the current filtered sensors.ec/tds,
    // no longer held during DOSING_EC/STABILIZING_EC. ecLastPublishedValue/
    // ecLastPublishedTds remain only as the NaN-fallback cache.
    if (!isnan(publishedSensors.ec)) systemState.ecLastPublishedValue = publishedSensors.ec;
    if (!isnan(publishedSensors.tds)) systemState.ecLastPublishedTds = publishedSensors.tds;
    if (!isnan(systemState.ecLastPublishedValue)) json.set("ec", systemState.ecLastPublishedValue);
    if (!isnan(systemState.ecLastPublishedTds)) json.set("tds", systemState.ecLastPublishedTds);
    // EC hardware-fault state - same shape/reasoning as phFault/phAvailable
    // above.
    json.set("ecFault", publishedSensors.ecFault);
    json.set("ecAvailable", publishedSensors.ecAvailable);
    // Quick-response refinement task: ph above is now the FAST TELEMETRY
    // value (the pH temporal step filter's own trusted candidate), not the
    // slower 10-sample automation-trust window's output. phConfirming lets
    // Android distinguish "this is the last trusted reading, a new one is
    // being confirmed" from a plain stale/unavailable state. Always
    // published (boolean, no NaN state) - false whenever ph is NaN too
    // (nothing has ever been confirmed at all yet, see Types.h's comment).
    json.set("phConfirming", publishedSensors.phConfirming);

    //--------------------------------------------------
    // Metadata
    //--------------------------------------------------

    // Coherent initial sensor snapshot (see Types.h's
    // sensorSnapshotBaselineAt comment). Written as nested fields of this
    // SAME json object, in the SAME single writeJson() call as every
    // sensor value above - Firebase RTDB applies one REST write atomically
    // regardless of how many nested keys it carries, so Android can never
    // observe sensorState.ready=true paired with a partially-written
    // sensor set, or vice versa.
    //
    // Deliberately NOT SENSOR_STABILIZATION_TIME (10s) - that's
    // AutomationManager's own boot-wait duration for a different purpose
    // (holding automatic refill/pH/EC/fog regulation off) and is far
    // longer than Monitoring UI readiness needs.
    //
    // sensorState.ready refinement (quick-response follow-up): readiness
    // requires every Monitoring-displayed sensor's state for THIS
    // acquisition session to be KNOWN - either a real value, or confirmed
    // unavailable (not just water level + pH telemetry, the original
    // narrower "fast sensor" set - EC/water temperature/DHT air
    // temperature+humidity are now included too, so the dashboard can no
    // longer reveal before they've had a chance to report anything at
    // all). A sensor is never required to be VALID, only for its state to
    // no longer be "haven't checked yet" - see each isXStateKnown()/
    // isPhEcAnalogSettling() accessor's own comment in SensorManager.h.
    // pH/EC's deliberate ~20s analog settle window
    // (PH_EC_ANALOG_SETTLE_TIME) itself counts as a known "not yet
    // available" state, not an unknown one, so it doesn't have to fully
    // elapse before readiness can fire - without that carve-out pH/EC
    // would force every physical fresh boot to the SENSOR_READY_MAX_MS
    // hard fallback below, exactly the "normally 1-2s" target this
    // refinement is meant to hit. Bounded by SENSOR_READY_MIN_MS (so
    // "ready" is never reported before even one real read cycle could
    // possibly have run) and SENSOR_READY_MAX_MS (so a genuinely
    // stuck/failed sensor still bounds readiness at ~3s rather than
    // blocking the dashboard indefinitely - it simply reveals as
    // "--"/stale on its own card, exactly as one that fails later would;
    // "usable snapshot" never means "every sensor is currently valid
    // forever").
    const unsigned long sensorSnapshotElapsed =
        millis() - systemState.sensorSnapshotBaselineAt;
    const bool allSensorsObserved =
        sensorManager.isWaterLevelStateKnown() &&
        (sensorManager.hasPhTelemetry() || sensorManager.isPhEcAnalogSettling()) &&
        sensorManager.isEcStateKnown() &&
        sensorManager.isWaterTempStateKnown() &&
        sensorManager.isDhtStateKnown();
    const bool sensorSnapshotReady =
        sensorSnapshotElapsed >= SENSOR_READY_MAX_MS ||
        (sensorSnapshotElapsed >= SENSOR_READY_MIN_MS && allSensorsObserved);
    json.set("sensorState/stabilizing", !sensorSnapshotReady);
    json.set("sensorState/ready", sensorSnapshotReady);
    // Device-uptime ms, matching the existing "timestamp" field's own
    // convention immediately below - not a second, differently-scaled time
    // source for Android to reconcile (see Part G of the task report -
    // Android does not, and must not, compare this against its own wall
    // clock; it exists for firmware-local/diagnostic reference only).
    json.set("sensorState/updatedAt", millis());

    json.set(
        "timestamp",
        millis());

    const unsigned long uploadStartedAt = millis();
    const bool uploadSucceeded = writeJson(deviceRoot() + "/sensors", json);
    const unsigned long uploadDuration = millis() - uploadStartedAt;
    logFirebaseDuration("Sensor upload", uploadDuration);

    if(uploadSucceeded)
    {
        lastSensorUploadAttempt = millis();
        lastSuccessfulSensorUpload = lastSensorUploadAttempt;

        const uint32_t recoveredFailures = consecutiveSensorUploadFailures;
        consecutiveSensorUploadFailures = 0;
        lastSensorUploadFailureReason = "";

        if (recoveredFailures > 0)
        {
            Serial.print("[PRESENCE] Heartbeat resumed after ");
            Serial.print(recoveredFailures);
            Serial.println(" failures");
            heartbeatResumePending = false;
        }
        else if (heartbeatResumePending)
        {
            Serial.println("[PRESENCE] Heartbeat resumed after connectivity loss");
            heartbeatResumePending = false;
        }
        else if ((!hasPublishedHeartbeat ||
                 millis() - lastHeartbeatSuccessLog >= HEARTBEAT_SUCCESS_LOG_INTERVAL_MS) &&
                 debugManager.shouldPrintDebug(DebugCategory::NETWORK))
        {
            Serial.println("[PRESENCE] Heartbeat uploaded");
            lastHeartbeatSuccessLog = millis();
        }
        hasPublishedHeartbeat = true;

        if (force && debugManager.shouldPrintDebug(DebugCategory::NETWORK))
        {
            Serial.print("[SENSOR-SYNC] waterLevel=");
            Serial.print(publishedSensors.waterLevel, 2);
            Serial.print(" ph=");
            // Reflects what was actually written to Firebase this call
            // (systemState.phLastPublishedValue/ecLastPublishedValue) -
            // since Stage 1, this always equals the live filtered sensors.ph
            // when valid, and only falls back to the last known-good value
            // on a transient NaN (see the pH block's own comment above).
            Serial.print(systemState.phLastPublishedValue, 2);
            Serial.print(" ec=");
            Serial.print(systemState.ecLastPublishedValue, 2);
            Serial.print(" t=");
            Serial.println(millis());
        }

        if (debugManager.shouldPrintDebug(DebugCategory::NETWORK))
        {
            Serial.println(
                "Sensors Uploaded");
        }
    }
    else
    {
        consecutiveSensorUploadFailures++;
        lastSensorUploadFailureReason = fbdo.errorReason();
        if (consecutiveSensorUploadFailures == 1 ||
            consecutiveSensorUploadFailures % 5 == 0)
        {
            Serial.println("[PRESENCE] Sensor upload failed");
            Serial.print("[PRESENCE] Consecutive failures: ");
            Serial.println(consecutiveSensorUploadFailures);
            if (!lastSensorUploadFailureReason.isEmpty())
            {
                Serial.print("[PRESENCE] Failure reason: ");
                Serial.println(lastSensorUploadFailureReason);
            }
        }
    }

    return uploadSucceeded;
}

void FirebaseManager::writeStatus()
{
    static unsigned long lastStatusUpload = 0;

    if(millis() - lastStatusUpload < UPLOAD_INTERVAL)
    {
        return;
    }

    lastStatusUpload = millis();

    FirebaseJson json;

    //--------------------------------------------------
    // System
    //--------------------------------------------------

    json.set(
        "currentMode",
        (int)systemState.currentMode);

    // Correction direction, mirrored for Android's manual-command advisor -
    // systemState.phDirection/ecDirection already drive DOSING_PH/DOSING_EC
    // internally (AutomationManager) but had no RTDB representation before
    // this. Lets the app tell "stabilizing after raising" from "...lowering"
    // without guessing from alert flags that can clear before stabilization ends.
    json.set(
        "phDirection",
        systemState.phDirection == PH_UP ? "up" :
        systemState.phDirection == PH_DOWN ? "down" : "none");

    json.set(
        "ecDirection",
        systemState.ecDirection == EC_RAISE ? "raise" :
        systemState.ecDirection == EC_DILUTE ? "dilute" : "none");

    // Lets the app tell "still in the silent settle window, reading not
    // trustworthy yet" from "settled and watching" during STABILIZING_PH/EC -
    // the exact same signal SafetyManager::canFog() already uses internally
    // for the same purpose. See phWatchPhaseActive's own comment in Types.h.
    json.set("phWatchPhaseActive", systemState.phWatchPhaseActive);
    json.set("ecWatchPhaseActive", systemState.ecWatchPhaseActive);

    // Countdown, in seconds, for the app's stabilizing-loader UI. Only
    // meaningful while currentMode is STABILIZING_PH/STABILIZING_EC/REFILLING
    // respectively - see each field's own comment in Types.h for what they
    // mean (and don't mean) outside that state.
    json.set("phStabilizeSecondsRemaining", systemState.phStabilizeSecondsRemaining);
    json.set("ecStabilizeSecondsRemaining", systemState.ecStabilizeSecondsRemaining);
    json.set("refillSecondsRemaining", systemState.refillSecondsRemaining);

    json.set(
        "manualMode",
        systemState.manualMode);

    json.set(
        "mockData",
        systemState.mockSensorsEnabled);
    json.set("mockDataDynamic",
        systemState.mockSensorsEnabled && systemState.mockSensorsDynamic);

    json.set("sensorTest", systemState.sensorTestEnabled);

    json.set("ignoreWaterLevelAutomation",
        systemState.ignoreWaterLevelAutomation);

    json.set("automationTestMode/enabled",
        systemState.automationTestSubsystem != AutomationTestSubsystem::NONE);
    json.set("automationTestMode/subsystem",
        automationTestSubsystemName(systemState.automationTestSubsystem));

    //--------------------------------------------------
    // Safety
    //--------------------------------------------------

    json.set(
        "reservoirLocked",
        systemState.reservoirLocked);

    json.set(
        "safetyLock",
        systemState.safetyLock);

    json.set("phSubsystemLocked", systemState.phSubsystemLocked);
    json.set("ecSubsystemLocked", systemState.ecSubsystemLocked);
    json.set("refillSubsystemLocked", systemState.refillSubsystemLocked);
    json.set("coolingSubsystemLocked", systemState.coolingSubsystemLocked);

    //--------------------------------------------------
    // Connectivity
    //--------------------------------------------------

    json.set(
        "wifiConnected",
        systemState.wifiConnected);

    json.set(
        "firebaseConnected",
        systemState.firebaseConnected);

    //--------------------------------------------------
    // RTC - diagnostic-only status snapshot of this device's DS3231. Never
    // an authoritative "current time" for anything outside this device;
    // see syncRTC() for why no cloud value ever gets written back into it.
    // Piggybacks on this existing status upload's own cadence rather than
    // adding a separate Firebase job just for the clock.
    //--------------------------------------------------

    json.set("rtc/connected", rtcManager.isConnected());
    json.set("rtc/valid", rtcManager.hasValidTime());
    // "RTC_RETAINED" | "NTP" | "INVALID" - where this boot's time actually
    // came from. See RTCManager::getSyncSourceName().
    json.set("rtc/syncSource", rtcManager.getSyncSourceName());
    if (rtcManager.hasValidTime())
    {
        // epochUtc = true absolute UTC Unix epoch (RTCManager::getEpochTime()
        // corrects for the DS3231's local storage - see its contract in
        // RTCManager.h). year/month/day/hour/minute/second below are the
        // DS3231's raw stored fields, i.e. Asia/Manila LOCAL civil time
        // (UTC+08:00) - NOT UTC. Renamed from the previous "epoch" to
        // "epochUtc" to make this split unambiguous at the RTDB level, not
        // just in code comments.
        json.set("rtc/epochUtc", (double)rtcManager.getEpochTime());
        json.set("rtc/year", rtcManager.getYear());
        json.set("rtc/month", rtcManager.getMonth());
        json.set("rtc/day", rtcManager.getDay());
        json.set("rtc/hour", rtcManager.getHour());
        json.set("rtc/minute", rtcManager.getMinute());
        json.set("rtc/second", rtcManager.getSecond());
    }
    else
    {
        // No date/time fields when invalid - an absent field can't be
        // mistaken for a real (e.g. 1970/epoch-0) reading the way a
        // present-but-zero one could.
        json.set("rtc/epochUtc", 0);
    }
    // lastSyncAt was removed: it held millis() (device uptime), which is
    // neither a calendar time nor an actual last-successful-sync moment -
    // exactly the confusion the RTC finalization task flagged. /status
    // itself has no existing authoritative update timestamp to reuse, so
    // this is a straight removal rather than a rename to a field nothing
    // would consume.

    const unsigned long uploadStartedAt = millis();
    const bool uploadSucceeded = updateJson(deviceRoot() + "/status", json);
    logFirebaseDuration("Status upload", millis() - uploadStartedAt);
    if(uploadSucceeded && debugManager.shouldPrintDebug(DebugCategory::NETWORK))
    {
        Serial.println(
            "Status Uploaded");
    }
}

void FirebaseManager::writeTelemetry()
{
    static unsigned long lastTelemetryUpload = 0;

    if(millis() - lastTelemetryUpload < UPLOAD_INTERVAL)
    {
        return;
    }

    lastTelemetryUpload = millis();

    FirebaseJson json;

    //--------------------------------------------------
    // pH
    //--------------------------------------------------

    json.set(
        "phAttempts",
        systemState.phAttempts);

    json.set(
        "phDoseTime",
        systemState.phDoseTime);

    //--------------------------------------------------
    // EC
    //--------------------------------------------------

    json.set(
        "ecAttempts",
        systemState.ecAttempts);

    json.set(
        "ecDoseTime",
        systemState.ecDoseTime);

    const unsigned long uploadStartedAt = millis();
    const bool uploadSucceeded = writeJson(deviceRoot() + "/telemetry", json);
    logFirebaseDuration("Telemetry upload", millis() - uploadStartedAt);
    if(uploadSucceeded && debugManager.shouldPrintDebug(DebugCategory::NETWORK))
    {
        Serial.println(
            "Telemetry Uploaded");
    }
}

bool FirebaseManager::writeAlerts()
{
    const bool startupPhaseComplete =
        systemState.currentMode != SENSOR_STABILIZATION &&
        systemState.currentMode != STARTUP;
    const bool sensorFaultPublishingEligible =
        startupPhaseComplete &&
        systemState.sensorSourceResolved &&
        !systemState.mockApplyPending;
    const bool initialSensorFaultPublishDue =
        sensorFaultPublishingEligible &&
        !sensorFaultPublicationInitialized;
    const bool fullUploadDue =
        !alertCacheInitialized ||
        millis() - lastAlertFullUpload >= REALTIME_FALLBACK_INTERVAL;

    if (!alertManager.isDirty() && !fullUploadDue && !initialSensorFaultPublishDue)
    {
        return false;
    }

    FirebaseJson json;
    bool hasFields = false;
    bool hasTransition = false;
    bool sensorFaultIncluded = false;
    bool sensorFaultChanged = false;

#define ADD_ALERT_FIELD(fieldName) \
    do { \
        const bool fieldChanged = !alertCacheInitialized || \
            alertState.fieldName != lastPublishedAlerts.fieldName; \
        if (fullUploadDue || fieldChanged) { \
            json.set(#fieldName, alertState.fieldName); \
            hasFields = true; \
        } \
        if (fieldChanged) hasTransition = true; \
    } while (false)

    ADD_ALERT_FIELD(lowWater);
    ADD_ALERT_FIELD(criticalLowWater);
    ADD_ALERT_FIELD(waterLevelLow);
    ADD_ALERT_FIELD(waterLevelHigh);
    ADD_ALERT_FIELD(refillIneffective);
    ADD_ALERT_FIELD(ecLow);
    ADD_ALERT_FIELD(ecHigh);
    ADD_ALERT_FIELD(ecDilutionIneffective);
    ADD_ALERT_FIELD(phOutOfRange);
    ADD_ALERT_FIELD(phLow);
    ADD_ALERT_FIELD(phHigh);
    ADD_ALERT_FIELD(waterTempOutOfRange);
    ADD_ALERT_FIELD(waterTempLow);
    ADD_ALERT_FIELD(lowAirTemperature);
    ADD_ALERT_FIELD(highTemperature);
    ADD_ALERT_FIELD(humidityLow);
    ADD_ALERT_FIELD(humidityHigh);

    if (sensorFaultPublishingEligible)
    {
        sensorFaultChanged =
            !sensorFaultPublicationInitialized ||
            alertState.sensorFault != lastPublishedAlerts.sensorFault;

        if (fullUploadDue || sensorFaultChanged)
        {
            json.set("sensorFault", alertState.sensorFault);
            hasFields = true;
            sensorFaultIncluded = true;
        }

        if (sensorFaultChanged) hasTransition = true;
    }

#undef ADD_ALERT_FIELD

    if (!hasFields)
    {
        alertManager.markSynced();
        return false;
    }

    if (!updateJson(deviceRoot() + "/alerts", json))
    {
        return false;
    }

#define LOG_ALERT_TRANSITION(fieldName) \
    do { \
        if ((!alertCacheInitialized || \
            alertState.fieldName != lastPublishedAlerts.fieldName) && \
            debugManager.shouldPrintDebug(DebugCategory::NETWORK)) { \
            Serial.print("[ALERT-SYNC] " #fieldName "="); \
            Serial.print(alertState.fieldName ? "true" : "false"); \
            Serial.print(" t="); \
            Serial.println(millis()); \
        } \
    } while (false)

    LOG_ALERT_TRANSITION(lowWater);
    LOG_ALERT_TRANSITION(criticalLowWater);
    LOG_ALERT_TRANSITION(waterLevelLow);
    LOG_ALERT_TRANSITION(waterLevelHigh);
    LOG_ALERT_TRANSITION(refillIneffective);
    LOG_ALERT_TRANSITION(ecLow);
    LOG_ALERT_TRANSITION(ecHigh);
    LOG_ALERT_TRANSITION(ecDilutionIneffective);
    LOG_ALERT_TRANSITION(phOutOfRange);
    LOG_ALERT_TRANSITION(phLow);
    LOG_ALERT_TRANSITION(phHigh);
    LOG_ALERT_TRANSITION(waterTempOutOfRange);
    LOG_ALERT_TRANSITION(waterTempLow);
    LOG_ALERT_TRANSITION(lowAirTemperature);
    LOG_ALERT_TRANSITION(highTemperature);
    LOG_ALERT_TRANSITION(humidityLow);
    LOG_ALERT_TRANSITION(humidityHigh);

    if (sensorFaultIncluded && sensorFaultChanged && debugManager.shouldPrintDebug(DebugCategory::NETWORK))
    {
        Serial.print("[ALERT-SYNC] sensorFault=");
        Serial.print(alertState.sensorFault ? "true" : "false");
        Serial.print(" t=");
        Serial.println(millis());
    }

#undef LOG_ALERT_TRANSITION

    lastPublishedAlerts = alertState;
    if (sensorFaultIncluded)
    {
        sensorFaultPublicationInitialized = true;
    }
    alertCacheInitialized = true;
    if (fullUploadDue) lastAlertFullUpload = millis();
    alertManager.markSynced();
    return hasTransition;
}

void FirebaseManager::writeActuators()
{
    // This is called unconditionally on every update() pass (unlike the
    // round-robin optional jobs), so a failed write previously had nothing
    // stopping it from retrying - blocking for a full request timeout - on
    // literally the next loop() tick, forever, since actuatorManager stays
    // "dirty" until a write actually succeeds. This backoff mirrors
    // readCommands()'s existing COMMAND_FAILURE_BACKOFF_INTERVAL pattern.
    static unsigned long lastActuatorSyncFailureAt = 0;
    static bool actuatorSyncBackoffActive = false;

    if (actuatorSyncBackoffActive &&
        millis() - lastActuatorSyncFailureAt < ACTUATOR_SYNC_FAILURE_BACKOFF_MS)
    {
        return;
    }

    FirebaseJson json;
    const bool fullUploadDue =
        !actuatorCacheInitialized ||
        millis() - lastActuatorFullUpload >= REALTIME_FALLBACK_INTERVAL;

    if (!actuatorManager.isStatusDirty() && !fullUploadDue)
    {
        return;
    }

    bool hasFields = false;
    bool changed[ACTUATOR_COUNT] = { false };
    ActuatorStatus pendingStatus[ACTUATOR_COUNT];

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        Actuator a = (Actuator)i;
        ActuatorStatus current = actuatorManager.getStatus(a);
        pendingStatus[i] = current;

        changed[i] = !actuatorCacheInitialized ||
            current.state != lastPublishedActuators[i].state ||
            current.running != lastPublishedActuators[i].running ||
            current.speed != lastPublishedActuators[i].speed ||
            current.startedAt != lastPublishedActuators[i].startedAt ||
            current.source != lastPublishedActuators[i].source ||
            current.strategy != lastPublishedActuators[i].strategy ||
            current.reason != lastPublishedActuators[i].reason ||
            current.overrideActive != lastPublishedActuators[i].overrideActive;

        if (!fullUploadDue && !changed[i])
        {
            continue;
        }

        String name = getActuatorName(a);
        
        json.set(name + "/running", current.running);
        json.set(name + "/state", (int)current.state);
        json.set(name + "/speed", current.speed);
        json.set(name + "/startedAt", current.startedAt);
        json.set(name + "/source", current.source);
        json.set(name + "/strategy", current.strategy);
        json.set(name + "/reason", current.reason);
        json.set(name + "/overrideActive", current.overrideActive);

        // Single-producer compatibility marker (task-mandated): its presence
        // tells the legacy state-trigger Cloud Function (logFoggerActivity)
        // that THIS firmware already recorded the transition through its own
        // durable queue and replay path, so it must defer instead of writing
        // a second foggingLogs document for the same transition. Legacy
        // firmware never sets this field, so that Cloud Function's existing
        // behavior is unchanged for older devices.
        if (a == FOGGER)
        {
            json.set(name + "/lastFoggingEventId", foggingEventQueue.getLastEventId());
        }

        hasFields = true;
    }

    if (!hasFields)
    {
        actuatorManager.markStatusSynced();
        return;
    }

    if (updateJson(deviceRoot() + "/actuatorStatus", json))
    {
        actuatorSyncBackoffActive = false;

        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            if (changed[i] && debugManager.shouldPrintActuator((Actuator)i))
            {
                Serial.print("[ACTUATOR-SYNC] ");
                Serial.print(getActuatorName((Actuator)i));
                Serial.print(" running=");
                Serial.print(pendingStatus[i].running ? "true" : "false");
                Serial.print(" source=");
                Serial.print(pendingStatus[i].source);
                Serial.print(" t=");
                Serial.println(millis());
            }

            lastPublishedActuators[i] = pendingStatus[i];
        }

        actuatorCacheInitialized = true;
        if (fullUploadDue) lastActuatorFullUpload = millis();
        actuatorManager.markStatusSynced();
    }
    else
    {
        // Local actuator state is untouched and remains authoritative;
        // actuatorManager stays dirty (markStatusSynced() was not called),
        // so the same latest state is retried - not replayed history - once
        // the backoff (or a broader COOLDOWN) clears.
        actuatorSyncBackoffActive = true;
        lastActuatorSyncFailureAt = millis();
    }
}

void FirebaseManager::writeDeviceInfo()
{
    static unsigned long lastDeviceInfoUpload = 0;

    if(lastDeviceInfoUpload != 0 &&
       millis() - lastDeviceInfoUpload < DEVICE_INFO_INTERVAL)
    {
        return;
    }

    lastDeviceInfoUpload = millis();

    FirebaseJson json;

    //--------------------------------------------------
    // Device
    //--------------------------------------------------

    json.set(
        "deviceName",
        DEVICE_NAME);

    json.set(
        "firmwareVersion",
        FIRMWARE_VERSION);

    //--------------------------------------------------
    // Status
    //--------------------------------------------------

    json.set(
        "lastSeen",
        millis());

    if(writeJson(
        deviceRoot() + "/deviceInfo",
        json) && debugManager.shouldPrintDebug(DebugCategory::NETWORK))
    {
        Serial.println(
            "Device Info Uploaded");
    }
}

//==================================================
// Offline notification pipeline
//==================================================

void FirebaseManager::readSmsRecipients()
{
    // Cached in NVS via SmsRecipientCache and does not need frequent reads -
    // previously had no cadence gate at all here (only the round-robin
    // optional-job rotation limited it), which is exactly the "9595 ms every
    // rotation" pattern this task is fixing.
    static unsigned long lastSmsRecipientsRead = 0;
    if (millis() - lastSmsRecipientsRead < LOW_PRIORITY_READ_INTERVAL_MS)
    {
        return;
    }
    lastSmsRecipientsRead = millis();

    const unsigned long startedAt = millis();
    bool succeeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/smsRecipients");
    logFirebaseDuration("SMS recipients read", millis() - startedAt);
    recordFirebaseResult(succeeded);
    if (!succeeded)
    {
        // Failed read - the last known-good cache is left untouched.
        return;
    }

    FirebaseJson& snapshot = fbdo.jsonObject();
    String phones[MAX_SMS_RECIPIENTS];
    uint8_t count = 0;

    int type;
    String key, value;
    size_t len = snapshot.iteratorBegin();
    for (size_t i = 0; i < len; i++)
    {
        snapshot.iteratorGet(i, type, key, value);
        if (type != FirebaseJson::JSON_OBJECT) continue; // each child is {phone, enabled[, role]}

        FirebaseJson child(value);
        FirebaseJsonData field;

        bool enabled = true;
        if (child.get(field, "enabled")) enabled = field.boolValue;
        if (!enabled) continue;

        if (child.get(field, "phone") && count < MAX_SMS_RECIPIENTS)
        {
            phones[count++] = field.stringValue;
        }
    }
    snapshot.iteratorEnd();

    // A genuinely empty object (0 eligible children) is an authoritative
    // snapshot too - distinct from the failed-read early return above - and
    // SmsRecipientCache treats a 0-count call as a valid clear.
    smsRecipientCache.applySnapshot(phones, count);
}

// Applies one min/max target-range pair from the settings snapshot currently
// held in fbdo. Shared by all four ranges so they cannot drift apart in how
// they validate. Logs only when a value is actually rejected, so a healthy
// device stays quiet.
void FirebaseManager::applyTargetRange(const char* minKey, const char* maxKey,
                                       float& minTarget, float& maxTarget,
                                       float physicalMin, float physicalMax)
{
    FirebaseJsonData data;

    float incomingMin = minTarget;
    float incomingMax = maxTarget;

    const bool hasMin = fbdo.jsonObject().get(data, minKey) && data.success;
    if (hasMin) incomingMin = data.floatValue;

    const bool hasMax = fbdo.jsonObject().get(data, maxKey) && data.success;
    if (hasMax) incomingMax = data.floatValue;

    if (!hasMin && !hasMax) return; // nothing published yet - keep defaults

    const bool valid =
        isfinite(incomingMin) && isfinite(incomingMax) &&
        incomingMin >= physicalMin && incomingMax <= physicalMax &&
        incomingMax > incomingMin;

    if (!valid)
    {
        Serial.print("[SETTINGS] Rejected target range ");
        Serial.print(minKey);
        Serial.print("/");
        Serial.print(maxKey);
        Serial.println(" - keeping last valid values");
        return;
    }

    minTarget = incomingMin;
    maxTarget = incomingMax;
}

void FirebaseManager::readHarvestSchedule()
{
    // Cached locally like readSmsRecipients(), but on its own faster cadence:
    // this projection is the cultivation gate, not background metadata.
    static unsigned long lastHarvestScheduleRead = 0;
    if (millis() - lastHarvestScheduleRead < HARVEST_SCHEDULE_READ_INTERVAL_MS)
    {
        return;
    }
    lastHarvestScheduleRead = millis();

    const unsigned long startedAt = millis();
    bool succeeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/harvestSchedule");
    logFirebaseDuration("Harvest schedule read", millis() - startedAt);
    recordFirebaseResult(succeeded);
    if (!succeeded)
    {
        // Failed read - the last known-good schedule is left untouched.
        return;
    }

    FirebaseJson& snapshot = fbdo.jsonObject();
    FirebaseJsonData field;

    bool active = false;
    if (snapshot.get(field, "active")) active = field.boolValue;

    String cycleId;
    if (snapshot.get(field, "cycleId")) cycleId = field.stringValue;

    int cycleNumber = 0;
    if (snapshot.get(field, "cycleNumber")) cycleNumber = field.intValue;

    uint32_t nextHarvestAt = 0;
    if (snapshot.get(field, "nextHarvestAt")) nextHarvestAt = (uint32_t)field.intValue;

    // No active cycle (or the projection producer explicitly cleared it) is
    // an authoritative "nothing due" snapshot, not a failure.
    harvestScheduleCache.applySnapshot(cycleId, cycleNumber, nextHarvestAt, active);
}

void FirebaseManager::replayQueuedNotification()
{
    NotificationEvent event;
    if (!notificationManager.getNextCloudReplayEvent(event)) return;

    if (notificationManager.isCloudReplayStale(event.eventId, 5UL * 60UL * 1000UL))
    {
        // (Re)submit the full, idempotent event content. A resubmission
        // after a stale window (e.g. the Cloud Function missed it, or the
        // ESP rebooted mid-replay) simply overwrites the same node with the
        // same content - safe, since the destination write is idempotent by
        // eventId on the Cloud Function side.
        FirebaseJson payload;
        payload.set("type", notificationEventTypeName(event.type));
        payload.set("severity", notificationSeverityName(event.severity));
        payload.set("title", event.title);
        payload.set("message", event.message);
        payload.set("occurredAt", (int)event.occurredAtEpoch);
        payload.set("timestampValid", event.timestampValid);
        payload.set("smsFallbackUsed",
            event.smsStatus == SmsDeliveryStatus::DELIVERED || event.smsStatus == SmsDeliveryStatus::PARTIAL);

        const unsigned long startedAt = millis();
        bool ok = writeJson(deviceRoot() + "/notificationQueue/" + event.eventId, payload);
        logFirebaseDuration("Notification replay write", millis() - startedAt);
        if (ok) notificationManager.markCloudReplaySubmitted(event.eventId);
        return;
    }

    // Already submitted and still fresh - just poll for the Cloud
    // Function's ack rather than resubmitting every rotation.
    const unsigned long startedAt = millis();
    bool succeeded = Firebase.RTDB.getString(&fbdo, deviceRoot() + "/notificationQueue/" + event.eventId + "/status");
    logFirebaseDuration("Notification ack poll", millis() - startedAt);
    recordFirebaseResult(succeeded);
    if (succeeded && fbdo.stringData() == "acked")
    {
        notificationManager.markCloudReplayAcked(event.eventId);
    }
}

// Append-only history replay for FoggingEventQueue - deliberately never
// touches actuatorStatus/fogger (see task report: replaying historical
// ON/OFF through the current-state node would let a stale replay flip what
// Monitoring/live UI read as the fogger's PRESENT state).
void FirebaseManager::replayQueuedFoggingEvent()
{
    FoggingQueueEvent event;
    String eventId;
    if (!foggingEventQueue.getNextReplayEvent(event, eventId)) return;

    if (foggingEventQueue.isReplayStale(eventId, 5UL * 60UL * 1000UL))
    {
        // (Re)submit the full, idempotent event content - safe to resend
        // identical content on retry/reboot since the Cloud Function side is
        // idempotent by this same eventId.
        FirebaseJson payload;
        payload.set("event", event.eventType == (uint8_t)FoggingEventType::ON ? "ON" : "OFF");
        payload.set("occurredAt", (int)event.occurredAtEpoch);
        payload.set("timestampValid", event.timestampValid);
        payload.set("source", foggingSourceCodeString((FoggingSourceCode)event.sourceCode));
        payload.set("strategy", foggingStrategyCodeString((FoggingStrategyCode)event.strategyCode));
        payload.set("reason", foggingReasonCodeString((FoggingReasonCode)event.reasonCode));

        const unsigned long startedAt = millis();
        bool ok = writeJson(deviceRoot() + "/foggingEventQueue/" + eventId, payload);
        logFirebaseDuration("Fogging event replay write", millis() - startedAt);
        if (ok) foggingEventQueue.markReplaySubmitted(eventId);
        return;
    }

    // Already submitted and still fresh - just poll for the Cloud
    // Function's ack rather than resubmitting every rotation.
    const unsigned long startedAt = millis();
    bool succeeded = Firebase.RTDB.getString(&fbdo, deviceRoot() + "/foggingEventQueue/" + eventId + "/status");
    logFirebaseDuration("Fogging event ack poll", millis() - startedAt);
    recordFirebaseResult(succeeded);
    if (succeeded && fbdo.stringData() == "acked")
    {
        foggingEventQueue.markReplayAcked(eventId);
    }
}

//==================================================
// Utilities
//==================================================

bool FirebaseManager::writeJson(
    const String& path,
    FirebaseJson& json)
{
    // A failure earlier in this same tick withdrew permission to call the
    // library. Report "not written" without calling it: every caller already
    // retries a false return later, and none of this path's writes is worth a
    // second blocking call into a connection that was just found dead.
    if (!cloudPathVerified) return false;

    bool success =
        Firebase.RTDB.setJSON(
            &fbdo,
            path,
            &json);

    recordFirebaseResult(success);

    if(!success)
    {
        Serial.print("Firebase Write Failed: ");
        Serial.println(path);
        Serial.println(fbdo.errorReason());
    }

    return success;
}

bool FirebaseManager::updateJson(
    const String& path,
    FirebaseJson& json)
{
    // See writeJson(): same reasoning.
    if (!cloudPathVerified) return false;

    bool success =
        Firebase.RTDB.updateNode(
            &fbdo,
            path,
            &json);

    recordFirebaseResult(success);

    if(!success)
    {
        Serial.print("Firebase Update Failed: ");
        Serial.println(path);
        Serial.println(fbdo.errorReason());
    }

    return success;
}

// Serial Diagnostics / Observability pass: delegates to the shared
// DebugManager implementation (standard [HH:MM:SS][PERF] prefix, SLOW vs.
// LEVEL_VERBOSE-only healthy-duration split) so every one of this file's
// existing logFirebaseDuration(...) call sites - readCommands(),
// readActuatorCommands(), sensor/status/telemetry uploads, etc. - is upgraded
// without needing to touch each call site individually. Threshold and which
// operations get timed are unchanged.
void FirebaseManager::logAuthDiagnostics(const char* reason) const
{
    const firebase_auth_token_status status = config.signer.tokens.status;
    const firebase_auth_token_type type = config.signer.tokens.token_type;
    const time_t now = time(nullptr);
    const uint32_t expires = config.signer.tokens.expires;
    const uint32_t refreshLead = config.signer.preRefreshSeconds;
    bool refreshDueKnown = type == token_type_legacy_token ||
                           (type != token_type_undefined &&
                            (expires == 0 || now > (time_t)FIREBASE_DEFAULT_TS));
    bool refreshDue = type != token_type_legacy_token &&
                      type != token_type_undefined && expires == 0;
    if (type != token_type_legacy_token && expires > 0 &&
        now > (time_t)FIREBASE_DEFAULT_TS)
    {
        const uint32_t boundedLead = refreshLead > expires ? expires : refreshLead;
        refreshDue = now >= (time_t)(expires - boundedLead);
    }

    Serial.printf("[AUTH-DIAG] reason=%s deviceCloudReady=%u tokenStatus=%u tokenType=%u refreshDue=",
                  reason ? reason : "unknown",
                  systemState.firebaseConnected ? 1U : 0U,
                  (unsigned)status, (unsigned)type);
    if (!refreshDueKnown) Serial.print("UNKNOWN");
    else Serial.print(refreshDue ? "1" : "0");
    Serial.printf(" refreshActive=%u tokenLastRequestAgeMs=%lu\n",
                  status == token_status_on_refresh ? 1U : 0U,
                  (unsigned long)(millis() - config.signer.tokens.last_millis));
}

void FirebaseManager::logSocketDiagnostics(const char* point)
{
    Serial.printf("[SOCKET-DIAG] point=%s tcp_connected=%u\n",
                  point ? point : "unknown",
                  (unsigned)fbdo.tcpClient.connected());
}

void FirebaseManager::logFirebaseDuration(
    const char* operation,
    unsigned long durationMs)
{
    debugManager.logFirebaseDuration(operation, durationMs);
    if (durationMs < 2000UL || !debugManager.atLeast(LogLevel::LEVEL_NORMAL))
        return;

    logAuthDiagnostics("slow-operation");
    const int httpCode = fbdo.httpCode();
    const int errorCode = fbdo.errorCode();
    const bool completed = errorCode == 0 && httpCode >= 200 && httpCode < 300;
    Serial.printf("[FIREBASE-DIAG] slow operation=%s elapsed=%lums result=%s http_code=%d error=%d tcp_connected=%u available=not_sampled connection_origin=UNKNOWN token_status=%u token_type=%u refresh_active_at_end=%u refresh_completed_during=UNKNOWN\n",
                  operation, durationMs, completed ? "OK" : "CHECK",
                  httpCode, errorCode, (unsigned)fbdo.tcpClient.connected(),
                  (unsigned)config.signer.tokens.status, (unsigned)config.signer.tokens.token_type,
                  config.signer.tokens.status == token_status_on_refresh ? 1U : 0U);
}

void FirebaseManager::loadDeviceId()
{
    preferences.begin("device", false);

    deviceId = preferences.getString("device_id", "");

    preferences.end();
}

bool FirebaseManager::saveDeviceId(const String& id)
{
    preferences.begin("device", false);

    preferences.putString("device_id", id);

    preferences.end();

    deviceId = id;

    return true;
}

void FirebaseManager::loadActuatorCommandTimestamps()
{
    if (!preferences.begin("manual_cmd", true))
    {
        Serial.println("[MANUAL] Unable to open command watermark storage");
        return;
    }

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        const String key = "c" + String(i);
        lastActuatorCommandTimestamps[i] = preferences.getULong64(key.c_str(), 0);
    }
    preferences.end();
}

void FirebaseManager::saveActuatorCommandTimestamp(Actuator actuator, uint64_t timestamp)
{
    if (!preferences.begin("manual_cmd", false))
    {
        Serial.println("[MANUAL] Unable to persist command watermark");
        return;
    }

    const String key = "c" + String(static_cast<int>(actuator));

    // Inspect the key's ALREADY-STORED type before writing anything. NVS
    // fixes a key's value type the first time it is created and rejects
    // every later write of a different type to the same key
    // (ESP_ERR_NVS_TYPE_MISMATCH), independent of free space - PT_INVALID
    // here means the key does not exist yet, anything else is whatever type
    // it was actually created with. This distinguishes the two cases that
    // are safe to (re)create - missing, or confirmed wrong-typed - from a
    // key that is ALREADY the correct PT_U64: a write failure on that one is
    // some other, unrelated NVS condition, and must never be treated as a
    // reason to delete an already-valid replay-protection watermark.
    const PreferenceType existingType = preferences.getType(key.c_str());

    if (existingType != PT_U64 && existingType != PT_INVALID)
    {
        // Confirmed legacy/wrong-typed key - one-time migration. remove()
        // deletes the wrong-typed entry (the same call already used for
        // "refresh_token"/"resumeFirebase" elsewhere in this firmware),
        // then the write below creates it fresh under the correct type.
        preferences.remove(key.c_str());

        if (preferences.putULong64(key.c_str(), timestamp) == 0)
        {
            Serial.print("[MANUAL] Failed to persist command watermark: ");
            Serial.println(getActuatorName(actuator));
        }
        else
        {
            Serial.print("[MANUAL] Command watermark key migrated: ");
            Serial.println(getActuatorName(actuator));
        }
    }
    else if (preferences.putULong64(key.c_str(), timestamp) == 0)
    {
        // Key is either missing (PT_INVALID, nothing to preserve) or
        // already correctly typed PT_U64 (a valid persisted watermark
        // exists) - either way this is an ordinary write attempt, not a
        // migration, and on failure the existing entry (if any) is left
        // exactly as it was. Removing it here would erase real replay
        // protection over what could be a transient/unrelated NVS failure.
        Serial.print("[MANUAL] Failed to persist command watermark: ");
        Serial.println(getActuatorName(actuator));
    }

    preferences.end();
}

const String& FirebaseManager::getDeviceId() const
{
    return deviceId;
}

String FirebaseManager::deviceRoot() const
{
    return "/devices/" + deviceId;
}



//==================================================
// Remote Mocking
//==================================================

void FirebaseManager::syncMockSensors()
{
    if (wifiManager.isProvisioningMode() || WiFi.status() != WL_CONNECTED || !Firebase.ready())
        return;

    readMockSensors();
}

void FirebaseManager::readMockSensors()
{
    static unsigned long lastMockRead = 0;
    static unsigned long lastMockReadFailure = 0;
    static bool mockReadBackoffActive = false;

    if (mockReadBackoffActive &&
        millis() - lastMockReadFailure < COMMAND_FAILURE_BACKOFF_INTERVAL)
    {
        return;
    }
    if(millis() - lastMockRead < MOCK_READ_INTERVAL) return;
    lastMockRead = millis();

    const bool mockReadSucceeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands/mockSensors");
    recordFirebaseResult(mockReadSucceeded);
    if(!mockReadSucceeded) {
        // A transient read failure must not silently change sensor authority.
        mockReadBackoffActive = true;
        lastMockReadFailure = millis();
        return;
    }
    mockReadBackoffActive = false;

    FirebaseJsonData data;
    FirebaseJson& json = fbdo.jsonObject();

    json.get(data, "enabled");
    // enabled=true is honoured only once this boot's stale-flag clear has been
    // confirmed - see devCommandsCleared.
    const bool nextEnabled = data.success && data.boolValue && devCommandsCleared;

    // Backward compatible: every existing/static payload omits this field
    // and therefore remains deterministic static mock data.
    json.get(data, "dynamic");
    const bool nextDynamic = data.success && data.boolValue;

    // A successful read with mock mode disabled resolves the effective source
    // to physical sensors even if optional mock payload fields are incomplete.
    // Persisting here (not only in the change branch below) means a malformed
    // mock payload can never leave the stored source pointing at MOCK.
    if (!nextEnabled)
    {
        systemState.sensorSourceResolved = true;
        sensorManager.persistSensorSource(false);

        // The cloud has explicitly turned mock mode off, so a boot-restored
        // mock source no longer has anything to wait for.
        sensorManager.cancelMockBootWait();
    }

    SensorData nextBase = systemState.mockSensorBases;

    // Air Temperature
    json.get(data, "airTemperature");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextBase.temperature = data.to<float>();
    else 
        nextBase.temperature = NAN;

    // Humidity
    json.get(data, "humidity");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextBase.humidity = data.to<float>();
    else 
        nextBase.humidity = NAN;

    // Water Temperature
    json.get(data, "waterTemperature");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextBase.waterTemp = data.to<float>();
    else 
        nextBase.waterTemp = NAN;

    // Water Level
    json.get(data, "waterLevel");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextBase.waterLevel = data.to<float>();
    else
        nextBase.waterLevel = NAN;

    // Water Level Depth (cm) - AUTHORITATIVE for refill/low-water control
    // (see Config.h's "Water Reservoir Geometry"). A tester injecting
    // waterLevelCm directly takes precedence; otherwise it's derived from
    // the percentage above so existing percentage-only mock payloads keep
    // driving refill/low-water control exactly as before, just
    // recalibrated to the new working-depth scale.
    // Capture get()'s own return value, not just data.success:
    // FirebaseJsonBase::mGet() only clears/repopulates `data` when the key
    // IS found - when "waterLevelCm" is absent (deleted, e.g. by the app's
    // null-override-clear), `data` is left completely untouched from the
    // "waterLevel" get() immediately above, so data.success was still true
    // and this branch silently reused the PERCENTAGE value as if it were
    // the cm override. Confirmed live bug: a mock waterLevel of 20%/33%/75%
    // with no cm override sent was read back as a literal 20/33/75cm depth
    // (should be 1.2/1.98/4.5cm), falsely tripping the reservoir-full
    // EC-dilution block.
    bool waterLevelCmFound = json.get(data, "waterLevelCm");
    if (waterLevelCmFound && data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
    {
        nextBase.waterLevelCm = data.to<float>();
    }
    else if (isfinite(nextBase.waterLevel))
    {
        nextBase.waterLevelCm = (nextBase.waterLevel / 100.0f) * MAX_WORKING_WATER_CM;
    }
    else
    {
        nextBase.waterLevelCm = NAN;
    }

    nextBase.waterVolumeLiters = isfinite(nextBase.waterLevelCm)
        ? nextBase.waterLevelCm * RESERVOIR_LENGTH_CM * RESERVOIR_WIDTH_CM / 1000.0f
        : NAN;

    // pH. Unlike the environmental fields above, a value is also rejected
    // for being out of the 0-14 domain, not just absent/wrong-typed - but
    // either failure only NaNs this one field (same as every field above)
    // instead of aborting the whole read. This used to `return` here, which
    // meant a payload that mocked only e.g. water level while leaving pH/EC
    // blank (the app's DevOptionsFragment.pushMockValues() omits a field
    // entirely from the write when its input box is empty) silently dropped
    // every other field too, not just pH.
    json.get(data, "ph");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
    {
        const float parsedPh = data.to<float>();
        if (isfinite(parsedPh) && parsedPh >= 0.0f && parsedPh <= 14.0f)
        {
            nextBase.ph = parsedPh;
        }
        else
        {
            Serial.print("[MOCK] pH rejected: ");
            Serial.println(parsedPh, 2);
            nextBase.ph = NAN;
        }
    }
    else
    {
        nextBase.ph = NAN;
    }

    // EC - same treatment as pH above.
    json.get(data, "ec");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
    {
        const float parsedEc = data.to<float>();
        if (isfinite(parsedEc) && parsedEc >= 0.0f)
        {
            nextBase.ec = parsedEc;
        }
        else
        {
            Serial.print("[MOCK] EC rejected: ");
            Serial.println(parsedEc, 2);
            nextBase.ec = NAN;
        }
    }
    else
    {
        nextBase.ec = NAN;
    }

    const bool enabledChanged = nextEnabled != systemState.mockSensorsEnabled;
    const bool dynamicChanged = nextDynamic != systemState.mockSensorsDynamic;
    const bool baseChanged =
        floatValuesDiffer(nextBase.temperature, systemState.mockSensorBases.temperature) ||
        floatValuesDiffer(nextBase.humidity, systemState.mockSensorBases.humidity) ||
        floatValuesDiffer(nextBase.waterTemp, systemState.mockSensorBases.waterTemp) ||
        floatValuesDiffer(nextBase.waterLevel, systemState.mockSensorBases.waterLevel) ||
        floatValuesDiffer(nextBase.ph, systemState.mockSensorBases.ph) ||
        floatValuesDiffer(nextBase.ec, systemState.mockSensorBases.ec);

    systemState.mockSensorBases = nextBase;
    systemState.mockSensorsDynamic = nextDynamic;
    // A new base or mode transition starts exactly at the configured values.
    // Static mode always mirrors the command payload. Dynamic mode alone may
    // mutate mockSensors between command polls.
    if (!nextEnabled || !nextDynamic || enabledChanged || dynamicChanged || baseChanged)
    {
        systemState.mockSensors = nextBase;
        systemState.mockDynamicUpdatedAt = millis();
    }
    systemState.sensorSourceResolved = true;

    // Reaching this point means the command payload itself was read
    // successfully (individual fields may still be NaN if the app left them
    // unset or out of range - see the per-field handling above), so this is
    // a fresh payload for THIS session - the only thing that confirms a
    // boot-restored mock source. A failed Firebase read returns earlier
    // (mockReadBackoffActive path above) and deliberately does not count.
    if (nextEnabled)
    {
        sensorManager.notifyMockPayloadReceived();
    }

    if (!enabledChanged && !(nextEnabled && (dynamicChanged || baseChanged)))
        return;

    if (enabledChanged)
    {
        // Firebase is authoritative once reachable. Record the new source so
        // the next offline boot starts from it rather than from a stale one.
        sensorManager.persistSensorSource(nextEnabled);

        Serial.print("[AUTOMATION] Sensor source reconciled from Firebase: ");
        Serial.println(nextEnabled ? "MOCK" : "PHYSICAL");
    }

    systemState.mockSensorsEnabled = nextEnabled;
    systemState.mockApplyPending = true;

    if (nextEnabled)
    {
        Serial.println("[MOCK] Command received");
        Serial.println("[MOCK] enabled=true");
        Serial.print("[MOCK] dynamic="); Serial.println(nextDynamic ? "true" : "false");
        Serial.print("[MOCK] Base pH="); Serial.println(nextBase.ph, 2);
        Serial.print("[MOCK] Base EC="); Serial.println(nextBase.ec, 2);
        Serial.print("[MOCK] Base AirTemp="); Serial.println(nextBase.temperature, 2);
        Serial.print("[MOCK] Base Humidity="); Serial.println(nextBase.humidity, 2);
        Serial.print("[MOCK] Base WaterTemp="); Serial.println(nextBase.waterTemp, 2);
        Serial.print("[MOCK] Base WaterLevel="); Serial.println(nextBase.waterLevel, 2);
    }
}

void FirebaseManager::syncSensorTest()
{
    enforceSensorTestTimeout();

    if (wifiManager.isProvisioningMode() || WiFi.status() != WL_CONNECTED || !Firebase.ready())
        return;

    readSensorTestCommand();
}

void FirebaseManager::enforceSensorTestTimeout()
{
    if (systemState.sensorTestEnabled &&
        millis() - systemState.sensorTestStartTime >= SENSOR_TEST_TIMEOUT_MS)
    {
        sensorTestCommandBlockedUntilFalse = true;
        setSensorTestEnabled(false, false);
    }
}

void FirebaseManager::readSensorTestCommand()
{
    static unsigned long lastSensorTestRead = 0;
    static unsigned long lastSensorTestReadFailure = 0;
    static bool sensorTestReadBackoffActive = false;

    if (sensorTestReadBackoffActive &&
        millis() - lastSensorTestReadFailure < COMMAND_FAILURE_BACKOFF_INTERVAL)
    {
        return;
    }
    if (millis() - lastSensorTestRead < 2000) return; // 2 seconds interval
    lastSensorTestRead = millis();

    FirebaseJsonData data;
    const bool sensorTestReadSucceeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands/sensorTest");
    recordFirebaseResult(sensorTestReadSucceeded);
    if (!sensorTestReadSucceeded)
    {
        sensorTestReadBackoffActive = true;
        lastSensorTestReadFailure = millis();
        return;
    }

    sensorTestReadBackoffActive = false;
    {
        FirebaseJson& json = fbdo.jsonObject();
        if (json.get(data, "enabled") && data.success)
        {
            const bool nextEnabled = data.boolValue;
            if (nextEnabled && sensorTestCommandBlockedUntilFalse)
            {
                recordFirebaseResult(
                    Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/commands/sensorTest/enabled", false));
                return;
            }

            if (!nextEnabled) sensorTestCommandBlockedUntilFalse = false;
            setSensorTestEnabled(nextEnabled);
        }
    }
}

void FirebaseManager::setSensorTestEnabled(bool enabled, bool publishAcknowledgement)
{
    if (enabled == systemState.sensorTestEnabled) return;

    systemState.sensorTestEnabled = enabled;
    if (enabled)
    {
        systemState.sensorTestStartTime = millis();
        Serial.println("[DEV TEST] Physical sensor test ENABLED");
        recordFirebaseResult(
            Firebase.RTDB.deleteNode(&fbdo, deviceRoot() + "/debug/physicalSensors"));
        if (hasActiveOperation())
        {
            updateOperationState(RequestState::FAILED, "Cancelled: physical sensor test enabled");
            writeCurrentOperation();
            archiveCurrentOperation();
            resetCurrentOperation();
            lastPublishedOperationState = RequestState::IDLE;
        }
        actuatorManager.turnOffAll("Sensor test enabled");
    }
    else
    {
        systemState.sensorTestStartTime = 0;
        Serial.println("[DEV TEST] Physical sensor test DISABLED");
        actuatorManager.turnOffAll("Sensor test disabled");
        automationManager.begin();
    }

    if (publishAcknowledgement &&
        WiFi.status() == WL_CONNECTED && Firebase.ready())
    {
        recordFirebaseResult(
            Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/status/sensorTest", enabled));
    }
}

void FirebaseManager::readWaterLevelOverrideCommand()
{
    static unsigned long lastRead = 0;
    static unsigned long lastReadFailure = 0;
    static bool readBackoffActive = false;

    if (readBackoffActive &&
        millis() - lastReadFailure < COMMAND_FAILURE_BACKOFF_INTERVAL)
    {
        return;
    }
    if (millis() - lastRead < 2000) return; // 2 seconds interval
    lastRead = millis();

    FirebaseJsonData data;
    const bool readSucceeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands/ignoreWaterLevelAutomation");
    recordFirebaseResult(readSucceeded);
    if (!readSucceeded)
    {
        readBackoffActive = true;
        lastReadFailure = millis();
        return;
    }

    readBackoffActive = false;
    {
        FirebaseJson& json = fbdo.jsonObject();
        if (json.get(data, "enabled") && data.success)
        {
            // enabled=true is honoured only once this boot's stale-flag clear
            // has been confirmed - see devCommandsCleared. Turning the
            // override OFF is always allowed.
            setIgnoreWaterLevelAutomation(data.boolValue && devCommandsCleared);
        }
    }
}

// Applying the flag is deliberately just the systemState write + logging +
// acknowledgement here - the actual bypass behavior lives entirely in
// AutomationManager (handleNormal() refuses to start a new automatic
// refill while this is set; handleRefilling() exits an already-running
// automatic one on the very next tick). Driving both off the same
// persistent flag, re-checked every tick, is simpler and more robust than
// a one-shot side effect here trying to reach into AutomationManager's
// state machine - it self-corrects regardless of exactly when this read
// lands relative to the automation loop, and needs no special-casing for
// which state happens to be active when the flag flips.
void FirebaseManager::setIgnoreWaterLevelAutomation(bool enabled, bool publishAcknowledgement)
{
    if (enabled == systemState.ignoreWaterLevelAutomation) return;

    systemState.ignoreWaterLevelAutomation = enabled;

    Serial.print("[DEV WATER] ignoreWaterLevelAutomation=");
    Serial.println(enabled ? "true" : "false");

    if (enabled)
    {
        Serial.println("[DEV WATER] automatic refill bypass active");
    }
    else
    {
        Serial.println("[DEV WATER] normal refill automation restored");
    }

    if (publishAcknowledgement &&
        WiFi.status() == WL_CONNECTED && Firebase.ready())
    {
        recordFirebaseResult(
            Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/status/ignoreWaterLevelAutomation", enabled));
    }
}

// One-shot admin Test SMS request - /commands/testSms/{timestamp},
// mirroring the per-actuator freshness+delete pattern in
// consumeActuatorCommandSnapshot() rather than the persisted-flag pattern
// above (a single fire-once request, not a standing mode).
// lastTestSmsCommandTimestamp only lives in RAM (a duplicate test SMS
// after a reboot mid-request is harmless - no actuator, no dose, no
// automation is involved), and the command node is deleted immediately
// after being handed to NotificationManager so a later poll or a
// reconnect never replays it within the same boot.
void FirebaseManager::readTestSmsCommand()
{
    static unsigned long lastRead = 0;
    static unsigned long lastReadFailure = 0;
    static bool readBackoffActive = false;

    if (readBackoffActive &&
        millis() - lastReadFailure < COMMAND_FAILURE_BACKOFF_INTERVAL)
    {
        return;
    }
    if (millis() - lastRead < 2000) return; // 2 seconds interval
    lastRead = millis();

    const bool readSucceeded = Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands/testSms");
    recordFirebaseResult(readSucceeded);
    if (!readSucceeded)
    {
        readBackoffActive = true;
        lastReadFailure = millis();
        return;
    }
    readBackoffActive = false;

    FirebaseJsonData data;
    FirebaseJson& json = fbdo.jsonObject();
    if (!json.get(data, "timestamp") || !data.success) return;

    const double rawTimestamp = data.doubleValue;
    if (!isfinite(rawTimestamp) || rawTimestamp <= 0) return;
    const uint64_t timestamp = static_cast<uint64_t>(rawTimestamp);

    // Same equality-based dedup shape as isDuplicateRequest(), scoped to this
    // one command only - a repeated read of the same still-present node
    // (before the delete below lands, or a request that failed to delete)
    // must not re-fire the SMS pipeline.
    if (timestamp <= lastTestSmsCommandTimestamp) return;
    lastTestSmsCommandTimestamp = timestamp;

    notificationManager.requestTestSms();

    recordFirebaseResult(
        Firebase.RTDB.deleteNode(&fbdo, deviceRoot() + "/commands/testSms"));
}

void FirebaseManager::writeDiagnosticSensors()
{
    constexpr unsigned long DIAGNOSTIC_SERIAL_INTERVAL_MS = 1500;
    constexpr unsigned long DIAGNOSTIC_UPLOAD_INTERVAL_MS = 5000;
    constexpr unsigned long DIAGNOSTIC_FAILURE_BACKOFF_MS = 10000;

    static unsigned long lastDiagnosticSerialLog = 0;
    static unsigned long lastDiagnosticUploadAttempt = 0;
    static unsigned long lastDiagnosticFailure = 0;
    static bool diagnosticBackoffActive = false;

    const unsigned long now = millis();
    if (now - lastDiagnosticSerialLog < DIAGNOSTIC_SERIAL_INTERVAL_MS)
    {
        return;
    }
    lastDiagnosticSerialLog = now;

    FirebaseJson json;

    // Print and set values
    Serial.println("--- [DEV TEST] Live Readings ---");

    // pH
    if (!isnan(physicalSensors.ph) && isfinite(physicalSensors.ph) && physicalSensors.ph >= 0.0f && physicalSensors.ph <= 14.0f)
    {
        json.set("ph", physicalSensors.ph);
        Serial.print("[DEV TEST] pH="); Serial.println(physicalSensors.ph, 2);
    }
    else
    {
        Serial.println("[DEV TEST] pH=INVALID");
    }

    // EC
    if (!isnan(physicalSensors.ec) && isfinite(physicalSensors.ec) && physicalSensors.ec >= 0.0f)
    {
        json.set("ec", physicalSensors.ec);
        Serial.print("[DEV TEST] EC="); Serial.println(physicalSensors.ec, 2);
    }
    else
    {
        Serial.println("[DEV TEST] EC=INVALID");
    }

    // Voltage behind the EC reading above - diagnostic only, useful for a
    // developer inspecting the probe's actual analog signal from the app
    // without a serial cable. EC calibration itself isn't adjustable here -
    // the accepted calibration (Calibration.h) is unchanged.
    //
    // "ecRaw" keeps its pre-existing field/key name, but as of the EC
    // calibration redesign it holds the same calibrated millivolt reading
    // as ecVoltage (in mV rather than V) - no longer a raw 0-4095 ADC
    // count. No current app code reads this field, so nothing consumes the
    // old meaning today.
    if (isfinite(physicalSensors.ecVoltage))
    {
        json.set("ecVoltage", physicalSensors.ecVoltage);
    }
    json.set("ecRaw", physicalSensors.ecRaw);

    // Air Temperature
    if (!isnan(physicalSensors.temperature) && isfinite(physicalSensors.temperature) && physicalSensors.temperature >= -40.0f && physicalSensors.temperature <= 100.0f)
    {
        json.set("airTemperature", physicalSensors.temperature);
        Serial.print("[DEV TEST] AirTemp="); Serial.println(physicalSensors.temperature, 2);
    }
    else
    {
        Serial.println("[DEV TEST] AirTemp=INVALID");
    }

    // Humidity
    if (!isnan(physicalSensors.humidity) && isfinite(physicalSensors.humidity) && physicalSensors.humidity >= 0.0f && physicalSensors.humidity <= 100.0f)
    {
        json.set("humidity", physicalSensors.humidity);
        Serial.print("[DEV TEST] Humidity="); Serial.println(physicalSensors.humidity, 2);
    }
    else
    {
        Serial.println("[DEV TEST] Humidity=INVALID");
    }

    // Water Temperature
    if (!isnan(physicalSensors.waterTemp) && isfinite(physicalSensors.waterTemp) && physicalSensors.waterTemp >= 0.0f && physicalSensors.waterTemp <= 100.0f)
    {
        json.set("waterTemperature", physicalSensors.waterTemp);
        Serial.print("[DEV TEST] WaterTemp="); Serial.println(physicalSensors.waterTemp, 2);
    }
    else
    {
        Serial.println("[DEV TEST] WaterTemp=INVALID");
    }

    // Water Level
    if (!isnan(physicalSensors.waterLevel) && isfinite(physicalSensors.waterLevel) && physicalSensors.waterLevel >= 0.0f && physicalSensors.waterLevel <= 100.0f)
    {
        json.set("waterLevel", physicalSensors.waterLevel);
        Serial.print("[DEV TEST] WaterLevel="); Serial.println(physicalSensors.waterLevel, 2);
    }
    else
    {
        Serial.println("[DEV TEST] WaterLevel=INVALID");
    }

    // Water Level Distance (raw HC-SR04 reading, diagnostics only)
    if (!isnan(physicalSensors.waterLevelDistanceCm) && isfinite(physicalSensors.waterLevelDistanceCm))
    {
        json.set("waterLevelDistanceCm", physicalSensors.waterLevelDistanceCm);
        Serial.print("[DEV TEST] WaterLevelDistanceCm="); Serial.println(physicalSensors.waterLevelDistanceCm, 2);
    }
    else
    {
        Serial.println("[DEV TEST] WaterLevelDistanceCm=INVALID");
    }

    // Water Depth / Volume (water-depth model)
    if (isfinite(physicalSensors.waterLevelCm))
    {
        json.set("waterLevelCm", physicalSensors.waterLevelCm);
        Serial.print("[DEV TEST] WaterLevelCm="); Serial.println(physicalSensors.waterLevelCm, 2);
    }
    else
    {
        Serial.println("[DEV TEST] WaterLevelCm=INVALID");
    }
    if (isfinite(physicalSensors.waterVolumeLiters))
    {
        json.set("waterVolumeLiters", physicalSensors.waterVolumeLiters);
        Serial.print("[DEV TEST] WaterVolumeLiters="); Serial.println(physicalSensors.waterVolumeLiters, 2);
    }

    if (now - lastDiagnosticUploadAttempt < DIAGNOSTIC_UPLOAD_INTERVAL_MS)
    {
        return;
    }

    if (diagnosticBackoffActive &&
        now - lastDiagnosticFailure < DIAGNOSTIC_FAILURE_BACKOFF_MS)
    {
        return;
    }

    // The normal /sensors write is the authoritative presence heartbeat. It is
    // called before this low-priority diagnostic path, and diagnostics are
    // skipped whenever that heartbeat has not succeeded or is due/overdue.
    if (!hasPublishedHeartbeat ||
        now - lastSuccessfulSensorUpload >= SENSOR_UPLOAD_INTERVAL_MS)
    {
        return;
    }

    lastDiagnosticUploadAttempt = now;
    json.set("timestamp", now);
    if (writeJson(deviceRoot() + "/debug/physicalSensors", json))
    {
        diagnosticBackoffActive = false;
    }
    else
    {
        diagnosticBackoffActive = true;
        lastDiagnosticFailure = millis();
        Serial.println("[DEV TEST] Diagnostic upload failed");
        Serial.println("[DEV TEST] Firebase diagnostic backoff active");
    }
}
