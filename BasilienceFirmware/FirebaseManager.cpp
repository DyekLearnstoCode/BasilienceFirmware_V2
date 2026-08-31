#include "FirebaseManager.h"
#include "Globals.h"
#include "Arduino.h"
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <esp_mac.h>

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
constexpr unsigned long SENSOR_UPLOAD_INTERVAL_MS = 1000;
constexpr unsigned long DEVICE_INFO_INTERVAL   = 15000;
constexpr unsigned long REALTIME_FALLBACK_INTERVAL = 60000;
constexpr unsigned long SLOW_FIREBASE_OPERATION_MS = 2000;
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

    loadDeviceId();
    loadActuatorCommandTimestamps();

    // Secure device identity (uid = deviceId) is tried first. Only when
    // neither a refresh token nor a bootstrap secret is available yet does
    // this fall back to legacy anonymous auth - and only while
    // SECURE_DEVICE_AUTH_REQUIRED is false, so already-fielded devices
    // (including the current test unit, pending its one-time secret
    // injection) are never locked out by this change alone.
    bool authenticated = trySecureAuthentication();

    if (authenticated)
    {
        Serial.println("[FIREBASE-AUTH] Secure device identity active");
        Serial.print("[FIREBASE-AUTH] uid=");
        Serial.println(deviceId);
    }
    else if (SECURE_DEVICE_AUTH_REQUIRED)
    {
        Serial.println("[FIREBASE-AUTH] Secure auth unavailable this boot (no device secret provisioned, or bootstrap failed)");
        Serial.println("[FIREBASE-AUTH] Legacy anonymous auth is disabled (SECURE_DEVICE_AUTH_REQUIRED=true) - Firebase connectivity unavailable this boot");
        systemState.firebaseConnected = false;
        // Local automation/safety/actuator/GSM/notification control is
        // untouched by this return - none of it lives in this class or
        // depends on Firebase having authenticated.
        return;
    }
    else
    {
        Serial.println("[SECURITY] Legacy Firebase auth compatibility mode active");
        if (Firebase.signUp(
                &config,
                &auth,
                "",
                ""))
        {
            Serial.println("Firebase SignUp OK");
        }
        else
        {
            Serial.print("Firebase SignUp Failed: ");
            Serial.println(config.signer.signupError.message.c_str());
        }

        Firebase.begin(
            &config,
            &auth);
    }

    // ROOT CAUSE of the observed reconnect loop (see task report): true here
    // lets the Firebase client library independently call WiFi.reconnect()
    // from inside FirebaseCore::resumeNetwork() whenever ITS OWN
    // networkReady() check happens to read a momentary non-CONNECTED status
    // during any RTDB call - and WiFi.reconnect() (STAClass::reconnect() in
    // the ESP32 core) unconditionally calls esp_wifi_disconnect() first if
    // still associated, forcibly dropping a connection that may not have
    // actually failed. That is a second, uncoordinated reconnect owner
    // fighting WiFiManager's own state machine, which already guarantees
    // exactly one association attempt in flight - it is why the same DHCP
    // lease kept getting reacquired with no "[WIFI] Connecting to..." log
    // line from WiFiManager: the library was reconnecting the radio itself,
    // outside WiFiManager entirely. false makes WiFiManager the sole owner
    // of Wi-Fi reconnection; the library now only observes connectivity
    // (failing/degrading Firebase operations when Wi-Fi is actually down)
    // instead of acting on it. reconnectWiFi() is deprecated in this
    // library version in favor of reconnectNetwork(), used here instead.
    // Same fix applied at the other call site in attemptFirebaseRecovery().
    Firebase.reconnectNetwork(false);

Serial.print("Loaded Device ID: [");
Serial.print(deviceId);
Serial.println("]");

    if (deviceId.isEmpty())
    {
        provisionDevice();

        if (deviceId.isEmpty())
        {
            Serial.println("Provisioning failed.");
            return;
        }
    }

    systemState.firebaseConnected = true;

    // Establish the existing RTDB actuator-command snapshot as a consumed
    // baseline before publishing this boot's first heartbeat. Commands written while
    // the device was offline must never execute as fresh hardware requests.
    primeActuatorCommands();

    initializeDatabase();

    // Diagnostic mode is deliberately non-persistent. A reboot always clears
    // both the retained command and its acknowledgement before normal control.
    systemState.sensorTestEnabled = false;
    systemState.sensorTestStartTime = 0;
    Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/commands/sensorTest/enabled", false);
    Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/status/sensorTest", false);

    // A new device may not have had a /commands node during the pre-online
    // baseline attempt. initializeDatabase() creates the canonical command
    // container, so finish priming before begin() returns to the main loop.
    if (!actuatorCommandsPrimed)
    {
        primeActuatorCommands();
    }

    readSettings();

    syncRTC();

    systemState.settingsLoaded = true;

    systemState.syncRTC = true;

    Serial.println("Firebase Started");
}
void FirebaseManager::initializeDatabase()
{

    Serial.println("Firebase RTDB onDisconnect rules registered.");

    // Presence is backend-owned. Only a successfully received sensor heartbeat
    // may update status/online; Firebase initialization alone is insufficient.
    FirebaseJson connectivityJson;
    connectivityJson.set("provisioning", false);
    updateJson(deviceRoot() + "/status", connectivityJson);

    FirebaseJson json;

    //--------------------------------------------------
    // Settings
    //--------------------------------------------------

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/settings"))
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
        if (seededRange && updateJson(deviceRoot() + "/settings", missingRanges))
        {
            Serial.println("[SETTINGS] Seeded missing target ranges");
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
        if (hasMissingSettings)
        {
            updateJson(deviceRoot() + "/settings", missingSettings);
        }
    }

    //--------------------------------------------------
    // Commands
    //--------------------------------------------------

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/commands/current"))
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

    //--------------------------------------------------
    // Current Operation
    //--------------------------------------------------

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/operations/current"))
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

    // RTC: no seeding block here anymore. The removed code used to write
    // this device's own (possibly post-power-loss, meaningless) DS3231
    // reading to /devices/{deviceId}/rtc whenever that node was absent -
    // and syncRTC() below then read that SAME node back and called
    // rtc.adjust() on it. Nothing else in this system (confirmed: no
    // Cloud Function, no Android screen) ever wrote a genuinely trustworthy
    // value there, so the whole thing was a circular echo that could
    // silently clear the DS3231's lostPower flag on garbage data - see the
    // RTC report for the full trace. RTC status is now published read-only
    // to /devices/{deviceId}/status/rtc by writeStatus() instead.
}

//==================================================
// Main Update
//==================================================

void FirebaseManager::update()
{
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
        return;
    }

    if (!wifiManager.isConnected())
    {
        systemState.firebaseConnected = false;
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        wasFirebaseConnected = false;
        return;
    }

    if (suspendedForProvisioning)
    {
        // Confirmed live bug this fixes: /status/provisioning was set true
        // when entering provisioning (see the startProvisioning command
        // handler below) but was never cleared back to false anywhere in
        // this firmware. DeviceConnectionManager.resolveState() on the app
        // side treats provisioning==true as an unconditional "always show
        // Reconnecting," with no time bound of its own - so any device that
        // had EVER gone through Wi-Fi Configuration/AP mode once would show
        // Reconnecting in the app permanently, even while fully online.
        // Written here (not unconditionally alongside the in-memory flag
        // below) so a failed write leaves suspendedForProvisioning true and
        // this retries on the very next tick, mirroring the existing
        // retry-by-not-advancing-state pattern the startProvisioning command
        // handler already uses for its own "set true" write.
        const unsigned long provisioningClearStartedAt = millis();
        const bool provisioningCleared = Firebase.RTDB.setBool(
            &fbdo, deviceRoot() + "/status/provisioning", false);
        logFirebaseDuration("Provisioning state clear", millis() - provisioningClearStartedAt);
        if (!provisioningCleared)
        {
            Serial.println("[FIREBASE] Unable to clear provisioning state; will retry next tick");
            return;
        }
        Serial.println("[FIREBASE] Resuming after Wi-Fi reconnect");
        suspendedForProvisioning = false;
    }

    systemState.firebaseConnected = Firebase.ready();
    if (systemState.firebaseConnected && !wasFirebaseConnected)
    {
        Serial.println("[FIREBASE] Reconnected");
    }
    wasFirebaseConnected = systemState.firebaseConnected;

    if(!Firebase.ready())
    {
        if (hasPublishedHeartbeat) heartbeatResumePending = true;
        return;
    }

    //--------------------------------------------------
    // Firebase transport health
    //--------------------------------------------------

    // COOLDOWN means repeated transport failures already confirmed the
    // connection is broken - retrying heartbeat/actuator/command calls here
    // would just block for the same timeout again for nothing. No Firebase
    // network call happens this cycle except, once the backoff window has
    // elapsed, exactly one controlled recovery attempt. Local automation,
    // safety, actuators, GSM, and the NVS notification queue are entirely
    // unaffected - they already ran before this function was ever called
    // (see loop(), and Part 12 of the report this task produces).
    if (firebaseHealth == FirebaseHealthState::COOLDOWN)
    {
        if (millis() - cooldownStartedAt >= cooldownDurationMs)
        {
            attemptFirebaseRecovery();
        }
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
    if (isSensorUploadDue())
    {
        writeSensors();
        writeActuators();
        return;
    }

    // HIGH PRIORITY: manual actuator control response. Checked directly on
    // every update() call (not via the optional-job round-robin below) so a
    // pending app command is never left waiting behind a low-priority job's
    // rotation slot - only its own existing COMMAND_READ_INTERVAL/backoff
    // statics still govern how often either actually performs a Firebase
    // call, so this changes ordering/latency only, not call frequency. The
    // independent per-actuator esp_timer deadline (see ActuatorManager) is
    // what protects physical timing during a stall; this is a responsiveness
    // improvement on top of that, not a second safety mechanism.
    // Preserves the pre-existing sensorTestEnabled suppression that used to
    // be enforced by these two jobs' skip-list entries in the round-robin
    // below (job == 2/3 there) - normal cultivation commands stay suppressed
    // during the physical sensor diagnostic mode, unchanged.
    if (!systemState.sensorTestEnabled)
    {
        readActuatorCommands();
        readCommands();
    }

    // Preserve event-driven actuator publication immediately behind heartbeat
    // and command reads.
    writeActuators();

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
        writeSensors(true);
    }

    // A slow alert update can consume the remaining heartbeat window.
    if (isSensorUploadDue())
    {
        writeSensors();
        return;
    }

    // Reduces the chance a known-slow, low-priority job (fogging/notification
    // ACK poll especially - the field-observed source of the longest stalls)
    // STARTS close to an actual manual command. Gated on
    // lastManualCommandActivityAt (set only when a fresh command is actually
    // observed - see consumeActuatorCommandSnapshot()/readCommands()) rather
    // than bare manualMode: manualMode can stay on for minutes with no
    // command in flight, which was too coarse a signal - a low-priority job
    // starting seconds before an eventual tap was still exposed to the same
    // 10-70s stall risk. This is deliberately narrower than
    // deferLowPriorityJobs below (which also covers readSettings/writeStatus
    // for the control-response/health cases): those two stay on their normal
    // cadence here because they carry safety-relevant state (safetyLock,
    // reservoirLocked, target ranges) Android's manual-control UI depends on.
    // Bounded by MANUAL_MODE_LOW_PRIORITY_GRACE_MS so a flurry of commands
    // spaced under 5s apart still can't suppress these jobs indefinitely -
    // see runOneOptionalFirebaseJob().
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
    constexpr uint8_t OPTIONAL_JOB_COUNT = 14;

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

        // A new alert transition must not sit behind low-priority synchronization.
        // Advance the cursor past these jobs now; they remain eligible on later
        // non-urgent rotations and therefore cannot be permanently starved.
        // Recipient/harvest-schedule sync and notification/fogging replay
        // (9-12) are likewise low-priority background work, same treatment
        // as 4-8 - sensors/safety/actuator sync/commands/heartbeats must
        // never be starved by history replay.
        if (deferLowPriorityJobs &&
            (job == 4 || job == 5 || job == 6 || job == 7 || job == 8 ||
             job == 9 || job == 10 || job == 11 || job == 12))
        {
            continue;
        }

        // Narrower manual-interaction deferral: telemetry (6), device info
        // (7), diagnostic sensors (8), SMS recipients (9), notification (11)
        // and fogging (12) ACK replay - the known-slow, purely-optional jobs.
        // readSettings (4) and writeStatus (5) are deliberately exempt (see
        // update()). Harvest schedule (10) is exempt only once a cached
        // active schedule already exists - a device with none yet must still
        // be able to learn of one while Manual Mode happens to be on.
        if (deferLowPriorityForManualInteraction &&
            (job == 6 || job == 7 || job == 8 || job == 9 || job == 11 || job == 12 ||
             (job == 10 && harvestScheduleCache.isActive())))
        {
            continue;
        }

        // This job is about to actually run (survived every skip check
        // above) - reset the grace-window clock so the NEXT manual-mode
        // check starts a fresh bounded deferral instead of compounding.
        if (job == 6 || job == 7 || job == 8 || job == 9 || job == 10 ||
            job == 11 || job == 12)
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
    // Sensor-to-bottom calibration (authoritative - see the automation
    // resilience pass report and Types.h's sensorToBottomCm comment).
    // Deliberately a SEPARATE field/key from waterLevelEmptyDistanceCm above,
    // never falling back to it, so a stale legacy value cannot silently
    // resurface here.
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

void FirebaseManager::provisionDevice()
{
    String mac = getMacAddress();
    if (mac.isEmpty())
    {
        // Never derive a provisioning lookup from a zero/unresolvable MAC.
        Serial.println("[IDENTITY] ERROR: Provisioning deferred - hardware MAC unavailable");
        return;
    }
    String path = "/provisioning/" + mac + "/deviceToken";

    Serial.println("Checking provisioning...");

    FirebaseData fbdo;

    if (Firebase.RTDB.getString(&fbdo, path))
    {
        deviceId = fbdo.stringData();

        if (!deviceId.isEmpty())
        {
            saveDeviceId(deviceId);

            Serial.print("Provisioned Device ID: ");
            Serial.println(deviceId);
        }
    }
    else
    {
        Serial.print("Provisioning lookup failed: ");
        Serial.println(fbdo.errorReason());
    }
}

//==================================================
// Secure Device Auth
//==================================================

bool FirebaseManager::trySecureAuthentication()
{
    loadDeviceAuthCredentials();

    if (deviceAuthRefreshToken.length() > 0)
    {
        Serial.println("[SECURITY] Stored refresh token found");
        if (restoreFromRefreshToken(deviceAuthRefreshToken))
        {
            Serial.println("[SECURITY] Refresh-token authentication succeeded");
            return true;
        }
        Serial.println("[SECURITY] Refresh-token authentication failed");
    }

    if (deviceAuthSecret.length() > 0)
    {
        Serial.println("[SECURITY] Stored device secret found");
        if (bootstrapSecureAuth(deviceAuthSecret))
        {
            return true;
        }
        Serial.println("[FIREBASE-AUTH] Bootstrap failed");
    }

    return false;
}

bool FirebaseManager::restoreFromRefreshToken(const String& refreshToken)
{
    // A string that is not shaped like a JWT (header.payload.signature) is
    // auto-detected by this library as a bare refresh token and triggers a
    // refresh-grant sign-in directly against Google's securetoken endpoint -
    // confirmed against FirebaseCore.cpp's own signer logic, not assumed
    // from documentation alone. No bootstrap call is made on this path.
    Firebase.setCustomToken(&config, refreshToken);
    Firebase.begin(&config, &auth);

    // Bounded wait, consistent with this same begin() sequence's existing
    // tolerance for a one-time blocking network step at Wi-Fi-connect time
    // (the anonymous signUp() this replaces already blocked synchronously
    // here) - not a new blocking pattern, and not part of the per-iteration
    // main loop this firmware keeps non-blocking elsewhere.
    unsigned long startedAt = millis();
    while (!Firebase.ready() && millis() - startedAt < 10000UL)
    {
        delay(100);
    }

    if (!Firebase.ready())
    {
        return false;
    }

    // The refresh-grant response can rotate the refresh token, not just the
    // short-lived ID token. Re-persisting here (in addition to the bootstrap
    // path) ensures NVS always holds whatever token the library is currently
    // using, instead of a possibly-superseded one from a prior boot.
    const char* rotatedRefreshToken = Firebase.getRefreshToken();
    if (rotatedRefreshToken != nullptr && strlen(rotatedRefreshToken) > 0
        && refreshToken != rotatedRefreshToken)
    {
        saveRefreshToken(String(rotatedRefreshToken));
        Serial.println("[SECURITY] Refresh token persisted");
    }

    return true;
}

bool FirebaseManager::bootstrapSecureAuth(const String& secret)
{
    // Wire format sent to BOOTSTRAP_ENDPOINT_URL below is unchanged
    // (colon-separated, e.g. "AA:BB:CC:DD:EE:FF") - only the underlying
    // source is now the hardware-level read, not WiFi.macAddress().
    String mac = getFormattedMacAddress();
    if (mac.isEmpty())
    {
        // begin() only runs once Wi-Fi is connected, by which point
        // WiFi.mode(WIFI_STA) has long been set and the MAC is stable - this
        // remains a defensive guard, not an expected path here.
        Serial.println("[FIREBASE-AUTH] MAC address not yet valid; deferring bootstrap");
        return false;
    }

    NetworkClientSecure secureClient;
    secureClient.setCACert(BOOTSTRAP_CA_CERT);

    HTTPClient http;
    http.setTimeout(15000);
    if (!http.begin(secureClient, BOOTSTRAP_ENDPOINT_URL))
    {
        Serial.println("[FIREBASE-AUTH] Unable to open bootstrap connection");
        return false;
    }
    http.addHeader("Content-Type", "application/json");

    FirebaseJson payload;
    payload.set("mac", mac);
    payload.set("deviceSecret", secret);
    String body;
    payload.toString(body);

    Serial.println("[SECURITY] Requesting device bootstrap token");
    int httpCode = http.POST(body);
    // The secret existed only in `payload`/`body`, local to this function -
    // cleared immediately after send; never logged, never echoed anywhere.
    body = "";
    payload.clear();

    if (httpCode != 200)
    {
        Serial.print("[FIREBASE-AUTH] Bootstrap rejected, HTTP ");
        Serial.println(httpCode);
        http.end();
        return false;
    }

    String response = http.getString();
    http.end();

    FirebaseJson responseJson;
    responseJson.setJsonData(response);
    FirebaseJsonData field;

    String customToken;
    if (responseJson.get(field, "customToken")) customToken = field.stringValue;

    // deviceId is not secret (it is already the Firestore claim code shown
    // to Admins during claiming) - returned alongside the token purely so a
    // first-time device that has not yet persisted a deviceId can learn the
    // server-resolved one without a separate /provisioning read.
    String resolvedDeviceId;
    if (responseJson.get(field, "deviceId")) resolvedDeviceId = field.stringValue;

    response = "";

    if (customToken.isEmpty())
    {
        Serial.println("[FIREBASE-AUTH] Bootstrap response missing token");
        return false;
    }
    Serial.println("[SECURITY] Bootstrap succeeded");

    if (deviceId.isEmpty() && !resolvedDeviceId.isEmpty())
    {
        saveDeviceId(resolvedDeviceId);
    }

    Firebase.setCustomToken(&config, customToken);
    customToken = "";
    Firebase.begin(&config, &auth);

    unsigned long startedAt = millis();
    while (!Firebase.ready() && millis() - startedAt < 10000UL)
    {
        delay(100);
    }

    if (!Firebase.ready())
    {
        Serial.println("[FIREBASE-AUTH] Sign-in with minted token did not complete");
        return false;
    }
    Serial.println("[SECURITY] Firebase custom-token authentication succeeded");

    const char* newRefreshToken = Firebase.getRefreshToken();
    if (newRefreshToken != nullptr && strlen(newRefreshToken) > 0)
    {
        saveRefreshToken(String(newRefreshToken));
        Serial.println("[SECURITY] Refresh token persisted");
    }

    return true;
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
    // "response payload read timed out", "connection refused",
    // "send request failed", "not connected", "connection lost",
    // "no http server", "response read failed.", "upload timed out",
    // "upload data sent error", "incomplete SSL client data",
    // "request timed out", "gateway timeout", "bad gateway",
    // "service unavailable", "internal server error". Deliberately excludes
    // permission/shape/application-level strings like "bad request",
    // "unauthorized", "forbidden", "not found", "path not exist", "data
    // type mismatch" - those never touch firebaseHealth.
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
    // writeSensors() specifically and drives its own local "[PRESENCE] ..."
    // logging, and device-offline detection is entirely backend-owned
    // (Cloud Functions evaluating lastServerSeen staleness) rather than
    // client-declared - so it is left completely untouched here. This
    // streak tracks every RTDB call site instead, purely to gate this
    // client's own retry/backoff behavior, and never writes any RTDB path
    // or triggers any notification itself.
    if (success)
    {
        if (firebaseHealth == FirebaseHealthState::DEGRADED)
        {
            Serial.println("[FIREBASE-HEALTH] DEGRADED -> HEALTHY");
        }
        transportFailureStreak = 0;
        firebaseHealth = FirebaseHealthState::HEALTHY;
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
    if (!isTransportFailureReason(reason))
    {
        // Application-level failure (permission denied, missing optional
        // path, malformed data, rejected operation command, etc.) - does
        // not indicate a broken connection, so it does not move health.
        return;
    }

    transportFailureStreak++;
    Serial.print("[FIREBASE-HEALTH] transport failure #");
    Serial.print(transportFailureStreak);
    Serial.print(": ");
    Serial.println(reason);

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

bool FirebaseManager::attemptFirebaseRecovery()
{
    firebaseHealth = FirebaseHealthState::RECOVERING;
    Serial.println("[FIREBASE-HEALTH] Recovery attempt");

    // Close/release the possibly-stuck internal SSL client before
    // re-establishing a session - fbdo.stopWiFiClient() is the verified
    // public API for this (Firebase.h's own reset(FirebaseConfig*) was
    // considered and rejected: its doc explicitly says it resets stored
    // auth credentials, which would violate "restore auth state without
    // losing credentials").
    fbdo.stopWiFiClient();
    fbdo.clear();

    // Re-run exactly the same auth flow begin() uses at boot: secure
    // identity first (refresh token, then secret bootstrap - both read the
    // same persisted NVS credentials, untouched by recovery), falling back
    // to legacy anonymous auth only in migration compatibility mode. No
    // credentials are cleared or regenerated by this path.
    bool authenticated = trySecureAuthentication();
    if (!authenticated && !SECURE_DEVICE_AUTH_REQUIRED)
    {
        if (Firebase.signUp(&config, &auth, "", ""))
        {
            Firebase.begin(&config, &auth);
        }
    }

    // Same reasoning as FirebaseManager::begin() above: WiFiManager, not
    // this library, owns Wi-Fi reconnection.
    Firebase.reconnectNetwork(false);

    unsigned long startedAt = millis();
    while (!Firebase.ready() && millis() - startedAt < 10000UL)
    {
        delay(100);
    }

    if (Firebase.ready())
    {
        Serial.println("[FIREBASE-HEALTH] Recovery succeeded");
        firebaseHealth = FirebaseHealthState::HEALTHY;
        transportFailureStreak = 0;
        cooldownDurationMs = 0;
        return true;
    }

    Serial.println("[FIREBASE-HEALTH] Recovery failed");
    enterFirebaseCooldown();
    return false;
}

//==================================================
// Time Synchronization
//==================================================
// PREVIOUSLY: this read /devices/{deviceId}/rtc back from RTDB and called
// rtc.adjust() on whatever it found there. That node was, in turn, only
// ever seeded by this SAME device's own DS3231 reading (see the removed
// block in initializeDatabase()) - there is no NTP client, no Android
// screen, and no Cloud Function anywhere in this system that ever writes a
// genuinely trustworthy time to that path (confirmed by inspecting both).
// So the "sync" was a circular echo of the device's own clock, and worse:
// rtc.adjust() unconditionally clears the DS3231's lostPower flag, so a
// device that booted with a lost/garbage time would get that garbage
// "confirmed" as valid on the very next boot, permanently hiding the fact
// it was never actually correct.
//
// NOW: actual recovery (bounded SNTP, gated on Wi-Fi already being
// connected) lives in RTCManager::update(), driven independently of
// Firebase's own lifecycle - RTC validity has nothing to do with whether
// Firebase happens to be connected, only with Wi-Fi and the DS3231 itself.
// This function, called once from begin(), is left as a one-time status
// report at Firebase-boot time, not a second place that writes the clock -
// there is exactly one writer of rtc.adjust() now (RTCManager).
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

    if(isDuplicateRequest(requestId))
    {
        return;
    }

    // A fresh, non-duplicate /commands/current request: every write to this
    // path today originates from the app (REFILL/RESET_SAFETY/pH-EC trigger
    // buttons), so reaching here - independent of whatever validation
    // happens below - is genuine manual interaction.
    lastManualCommandActivityAt = millis();

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
        }
        systemState.manualMode = newManualMode;
    }
    const bool startProvisioningRequested =
        snapshot.get(jsonData, "startProvisioning") && jsonData.success && jsonData.boolValue;
    const bool automationTestModeChanged = applyAutomationTestModeCommand(snapshot);

    const bool dispatchCommands = actuatorCommandsPrimed;
    consumeActuatorCommandSnapshot(snapshot, dispatchCommands);
    if (!actuatorCommandsPrimed)
    {
        actuatorCommandsPrimed = true;
        Serial.println("[MANUAL] Existing actuator commands consumed as reconnect baseline");
    }

    if(automationTestModeChanged)
    {
        setAutomationTestMode(systemState.automationTestSubsystem, true);
    }

    // Developer-only manual provisioning trigger.
    // Wi-Fi credentials are intentionally not accepted through Firebase/RTDB.
    if (startProvisioningRequested)
    {
        Serial.println("Manual provisioning/AP mode command received.");
        const unsigned long provisioningWriteStartedAt = millis();
        const bool provisioningPublished = Firebase.RTDB.setBool(
            &fbdo, deviceRoot() + "/status/provisioning", true);
        logFirebaseDuration("Provisioning state write", millis() - provisioningWriteStartedAt);
        if (!provisioningPublished)
        {
            Serial.println("[WIFI] Unable to publish provisioning state before cloud suspension");
            // Retain the command so the next poll can retry. Entering AP mode
            // before backend grace exists would create a false offline event.
            return;
        }
        Firebase.RTDB.deleteNode(&fbdo, deviceRoot() + "/commands/startProvisioning");
        if (!suspendedForProvisioning)
        {
            suspendedForProvisioning = true;
        }
        systemState.firebaseConnected = false;
        wasFirebaseConnected = false;
        wifiManager.startManualProvisioning();
    }
}

void FirebaseManager::primeActuatorCommands()
{
    if (!Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands"))
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
}

bool FirebaseManager::applyAutomationTestModeCommand(FirebaseJson& snapshot)
{
    FirebaseJsonData data;

    // Developer-only Grow Light mock time - read unconditionally, ahead of
    // the enabled/subsystem branches below, so a value stays captured in
    // systemState even while a different (or no) Automation Test Mode is
    // currently selected. Safe either way: AutomationManager::
    // growLightMockTimeActive() re-checks automationTestSubsystem ==
    // GROW_LIGHT live on every use, so a stored value never takes effect on
    // its own - the app is not required to delete it when switching test
    // modes away from Grow Light. Minutes are clamped, not rejected, since
    // this is a developer convenience field, not a production safety
    // setting.
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
        const bool isNew = timestamps[i] > previousTimestamp;

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
        if (!Firebase.RTDB.deleteNode(&fbdo, commandPath))
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

bool FirebaseManager::isDuplicateRequest(uint16_t requestId) const
{
    return requestId ==
           systemState.lastProcessedRequestId;
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

    if (!isnan(publishedSensors.ec)) json.set("ec", publishedSensors.ec);
    if (!isnan(publishedSensors.tds)) json.set("tds", publishedSensors.tds);
    if (!isnan(publishedSensors.ph)) json.set("ph", publishedSensors.ph);
    // Quick-response refinement task: ph above is now the FAST TELEMETRY
    // value (the pH temporal step filter's own trusted candidate), not the
    // slower 10-sample automation-trust window's output - see
    // SensorManager::applyEffectiveSensors()'s own comment. phConfirming
    // lets Android distinguish "this is the last trusted reading, a new one
    // is being confirmed" from a plain stale/unavailable state. Always
    // published (boolean, no NaN state) - false whenever ph is NaN too
    // (nothing has ever been confirmed at all yet, see Types.h's comment).
    json.set("phConfirming", publishedSensors.phConfirming);

    //--------------------------------------------------
    // Metadata
    //--------------------------------------------------

    // Coherent initial sensor snapshot (see the quick-response refinement
    // task report and Types.h's sensorSnapshotBaselineAt comment). Written
    // as nested fields of this SAME json object, in the SAME single
    // writeJson() call as every sensor value above - Firebase RTDB applies
    // one REST write atomically regardless of how many nested keys it
    // carries, so Android can never observe sensorState.ready=true paired
    // with a partially-written sensor set, or vice versa.
    //
    // Deliberately NOT SENSOR_STABILIZATION_TIME (10s) - that is
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
    // longer reveal before they have had a chance to report anything at
    // all, which previously let them flash "--" individually right after
    // reveal). A sensor is never required to be VALID, only for its state
    // to no longer be "haven't checked yet" - see each isXStateKnown()/
    // isPhEcAnalogSettling() accessor's own comment in SensorManager.h.
    // pH/EC's deliberate ~20s analog settle window (PH_EC_ANALOG_SETTLE_TIME)
    // itself counts as a known "not yet available" state, not an unknown
    // one, so it does not have to fully elapse before readiness can fire -
    // without that carve-out pH/EC would force every physical fresh boot
    // to the SENSOR_READY_MAX_MS hard fallback below, which is exactly the
    // "normally 1-2s" target this refinement is meant to hit. Bounded by
    // SENSOR_READY_MIN_MS (so "ready" is never reported before even one
    // real read cycle could possibly have run) and SENSOR_READY_MAX_MS
    // (so a genuinely stuck/failed sensor still bounds readiness at ~3s
    // rather than blocking the dashboard indefinitely - it simply reveals
    // as "--"/stale on its own card, exactly as one that fails later
    // would; "usable snapshot" never means "every sensor is currently
    // valid forever" - see Part D/F of the task report).
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
            Serial.print(publishedSensors.ph, 2);
            Serial.print(" ec=");
            Serial.print(publishedSensors.ec, 2);
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
        // just in code comments; safe to rename because this whole /status/rtc
        // structure was only added in the immediately prior task and has no
        // existing Android/Cloud Function reader yet (confirmed by search).
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
    ADD_ALERT_FIELD(ecLow);
    ADD_ALERT_FIELD(ecHigh);
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
    LOG_ALERT_TRANSITION(ecLow);
    LOG_ALERT_TRANSITION(ecHigh);
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

void FirebaseManager::logFirebaseDuration(
    const char* operation,
    unsigned long durationMs) const
{
    if (durationMs < SLOW_FIREBASE_OPERATION_MS) return;

    Serial.print("[FIREBASE] Slow operation: ");
    Serial.print(operation);
    Serial.print(" took ");
    Serial.print(durationMs);
    Serial.println(" ms");
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
    if (preferences.putULong64(key.c_str(), timestamp) == 0)
    {
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
    const bool nextEnabled = data.success && data.boolValue;

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
    // waterLevelCm directly takes precedence; otherwise it is derived from
    // the percentage above so existing percentage-only mock payloads keep
    // driving refill/low-water control exactly as before, just recalibrated
    // to the new working-depth scale.
    json.get(data, "waterLevelCm");
    if (data.success &&
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

    // pH
    const bool phExtracted = json.get(data, "ph") && data.success;
    if (!phExtracted ||
        (data.typeNum != FirebaseJson::JSON_FLOAT &&
         data.typeNum != FirebaseJson::JSON_DOUBLE &&
         data.typeNum != FirebaseJson::JSON_INT))
    {
        Serial.println("[MOCK] pH parse failed");
        return;
    }

    const float parsedPh = data.to<float>();
    if (!isfinite(parsedPh) || parsedPh < 0.0f || parsedPh > 14.0f)
    {
        Serial.print("[MOCK] pH rejected: ");
        Serial.println(parsedPh, 2);
        return;
    }

    nextBase.ph = parsedPh;

    // EC
    const bool ecExtracted = json.get(data, "ec") && data.success;
    if (!ecExtracted ||
        (data.typeNum != FirebaseJson::JSON_FLOAT &&
         data.typeNum != FirebaseJson::JSON_DOUBLE &&
         data.typeNum != FirebaseJson::JSON_INT))
    {
        Serial.println("[MOCK] EC parse failed");
        return;
    }

    const float parsedEc = data.to<float>();
    if (!isfinite(parsedEc) || parsedEc < 0.0f)
    {
        Serial.print("[MOCK] EC rejected: ");
        Serial.println(parsedEc, 2);
        return;
    }

    nextBase.ec = parsedEc;

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

    // Reaching this point means every field parsed and validated, so this is a
    // fresh valid payload for THIS session - the only thing that confirms a
    // boot-restored mock source. Malformed payloads return earlier and
    // deliberately do not count.
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
                Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/commands/sensorTest/enabled", false);
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
        Firebase.RTDB.deleteNode(&fbdo, deviceRoot() + "/debug/physicalSensors");
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
        Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/status/sensorTest", enabled);
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
            setIgnoreWaterLevelAutomation(data.boolValue);
        }
    }
}

// Applying the flag is deliberately just the systemState write + logging +
// acknowledgement here - the actual bypass behavior lives entirely in
// AutomationManager (handleNormal() refuses to start a new automatic refill
// while this is set; handleRefilling() exits an
// already-running automatic one on the very next tick). Driving both off the
// same persistent flag, re-checked every tick, is simpler and more robust
// than a one-shot side effect here trying to reach into AutomationManager's
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
        Firebase.RTDB.setBool(&fbdo, deviceRoot() + "/status/ignoreWaterLevelAutomation", enabled);
    }
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

    // Raw ADC/voltage behind the EC reading above - diagnostic only, useful
    // for a developer inspecting the probe's actual analog signal from the
    // app without a serial cable. EC calibration itself is not adjustable
    // here - the accepted calibration (Calibration.h) is unchanged.
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
