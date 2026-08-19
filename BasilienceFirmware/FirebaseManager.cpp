#include "FirebaseManager.h"
#include "Globals.h"
#include "Arduino.h"
#include <HTTPClient.h>
#include <NetworkClientSecure.h>

namespace
{

constexpr unsigned long COMMAND_READ_INTERVAL  = 1500;
constexpr unsigned long COMMAND_FAILURE_BACKOFF_INTERVAL = 5000;
constexpr unsigned long MOCK_READ_INTERVAL     = 2000;
constexpr unsigned long SETTINGS_READ_INTERVAL = 60000;
constexpr unsigned long UPLOAD_INTERVAL        = 10000;
constexpr unsigned long DEVICE_INFO_INTERVAL   = 15000;
constexpr unsigned long REALTIME_FALLBACK_INTERVAL = 60000;
constexpr unsigned long SLOW_FIREBASE_OPERATION_MS = 2000;
constexpr unsigned long HEARTBEAT_SUCCESS_LOG_INTERVAL_MS = 60000;
constexpr unsigned long SENSOR_TEST_TIMEOUT_MS = 10UL * 60UL * 1000UL;

// Consecutive transport-level failures before Firebase health leaves
// DEGRADED and enters COOLDOWN (no Firebase network calls at all).
constexpr uint8_t TRANSPORT_FAILURE_COOLDOWN_THRESHOLD = 3;
constexpr unsigned long COOLDOWN_INITIAL_MS = 15000UL;
constexpr unsigned long COOLDOWN_MAX_MS = 60000UL;
// Cached-locally, low-priority background refreshes (SMS recipients,
// harvest schedule) - within the task's suggested 60-120s range.
constexpr unsigned long LOW_PRIORITY_READ_INTERVAL_MS = 90000UL;
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
        systemState.highAirTemp = preferences.getFloat("highAir", systemState.highAirTemp);
        systemState.airTempRelease = preferences.getFloat("airRelease", systemState.airTempRelease);
        systemState.highHumidity = preferences.getFloat("highHumidity", systemState.highHumidity);
        systemState.humidityRelease = preferences.getFloat("humidityRel", systemState.humidityRelease);
        systemState.highWaterTemp = preferences.getFloat("highWater", systemState.highWaterTemp);
        systemState.coolerOffTemp = preferences.getFloat("coolerOff", systemState.coolerOffTemp);
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
    preferences.putFloat("highAir", systemState.highAirTemp);
    preferences.putFloat("airRelease", systemState.airTempRelease);
    preferences.putFloat("highHumidity", systemState.highHumidity);
    preferences.putFloat("humidityRel", systemState.humidityRelease);
    preferences.putFloat("highWater", systemState.highWaterTemp);
    preferences.putFloat("coolerOff", systemState.coolerOffTemp);
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

    Firebase.reconnectWiFi(true);

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

        json.set("highWaterTemp",
            systemState.highWaterTemp);

        json.set("coolerOffTemp",
            systemState.coolerOffTemp);

        json.set("highAirTemp",
            systemState.highAirTemp);

        json.set("airTempRelease", systemState.airTempRelease);
        json.set("highHumidity", systemState.highHumidity);
        json.set("humidityRelease", systemState.humidityRelease);

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

    //--------------------------------------------------
    // RTC
    //--------------------------------------------------

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/rtc"))
    {
        json.clear();

        json.set(
            "month",
            rtcManager.getMonth());

        json.set(
            "day",
            rtcManager.getDay());

        json.set(
            "year",
            rtcManager.getYear());

        json.set(
            "hour",
            rtcManager.getHour());

        json.set(
            "minute",
            rtcManager.getMinute());

        json.set(
            "second",
            rtcManager.getSecond());

        writeJson(
            deviceRoot() + "/rtc",
            json);
    }
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

    // Preserve event-driven actuator publication immediately behind heartbeat.
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

    // Every remaining synchronous read/write is distributed across subsequent
    // loop iterations so slow requests cannot accumulate in one update.
    runOneOptionalFirebaseJob(
        systemState.sensorTestEnabled,
        alertWasDirty || deferLowPriorityForControlResponse || deferLowPriorityForHealth);
}

bool FirebaseManager::isSensorUploadDue() const
{
    return millis() - lastSensorUploadAttempt >= UPLOAD_INTERVAL;
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
    bool deferLowPriorityJobs)
{
    constexpr uint8_t OPTIONAL_JOB_COUNT = 14;

    for (uint8_t checked = 0; checked < OPTIONAL_JOB_COUNT; checked++)
    {
        const uint8_t job = optionalFirebaseJobCursor;
        optionalFirebaseJobCursor = (optionalFirebaseJobCursor + 1) % OPTIONAL_JOB_COUNT;

        // Normal cultivation commands/alerts remain suppressed during the
        // existing physical sensor diagnostic mode.
        if (sensorTestMode &&
            (job == 0 || job == 1 || job == 2 || job == 3 || job == 6))
        {
            continue;
        }
        if (!sensorTestMode && job == 10)
        {
            continue;
        }

        // A new alert transition must not sit behind low-priority synchronization.
        // Advance the cursor past these jobs now; they remain eligible on later
        // non-urgent rotations and therefore cannot be permanently starved.
        // Recipient/harvest-schedule sync and notification replay (11-13) are
        // likewise low-priority background work, same treatment as 6-10.
        if (deferLowPriorityJobs &&
            (job == 6 || job == 7 || job == 8 || job == 9 || job == 10 ||
             job == 11 || job == 12 || job == 13))
        {
            continue;
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
                readCommands();
                return;

            case 3:
                readActuatorCommands();
                return;

            case 4:
                readSensorTestCommand();
                return;

            case 5:
                readMockSensors();
                return;

            case 6:
                if (millis() - lastSettingsRead >= SETTINGS_READ_INTERVAL)
                {
                    lastSettingsRead = millis();
                    readSettings();
                }
                return;

            case 7:
                writeStatus();
                return;

            case 8:
                writeTelemetry();
                return;

            case 9:
                writeDeviceInfo();
                return;

            case 10:
                writeDiagnosticSensors();
                return;

            case 11:
                readSmsRecipients();
                return;

            case 12:
                readHarvestSchedule();
                return;

            case 13:
                replayQueuedNotification();
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
            Serial.print("[OP-SYNC] ");
            Serial.print(requestStateToString(state));
            Serial.print(" published requestId=");
            Serial.print(request.requestId);
            Serial.print(" operation=");
            Serial.println(operationToString(request.operation));

            if (orderedAutomaticCompletion)
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

                if (request.operation == OperationType::PH_UP ||
                    request.operation == OperationType::PH_DOWN ||
                    request.operation == OperationType::EC_CORRECTION)
                {
                    systemState.chemistryFoggingHoldActive = false;
                    Serial.println("[CHEMISTRY] Lifecycle published - fogging eligible");
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

    // Only validated/accepted runtime values are persisted.
    persistSettings();
}


String FirebaseManager::getMacAddress()
{
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    mac.toUpperCase();
    return mac;
}

void FirebaseManager::provisionDevice()
{
    String mac = getMacAddress();
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
    String mac = WiFi.macAddress();
    if (mac.isEmpty() || mac == "00:00:00:00:00:00")
    {
        // begin() only runs once Wi-Fi is connected, by which point
        // WiFi.mode(WIFI_STA) has long been set and the MAC is stable - this
        // is a defensive guard against the well-known all-zero transient
        // value, not an expected path here.
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

    Firebase.reconnectWiFi(true);

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
void FirebaseManager::syncRTC()
{

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/rtc"))
    {
        Serial.println(
            "RTC SYNC FAILED");

        return;
    }

    FirebaseJsonData data;

    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;

    fbdo.jsonObject().get(data, "year");
    year = data.intValue;

    fbdo.jsonObject().get(data, "month");
    month = data.intValue;

    fbdo.jsonObject().get(data, "day");
    day = data.intValue;

    fbdo.jsonObject().get(data, "hour");
    hour = data.intValue;

    fbdo.jsonObject().get(data, "minute");
    minute = data.intValue;

    fbdo.jsonObject().get(data, "second");
    second = data.intValue;

    rtcManager.setDateTime(
        year,
        month,
        day,
        hour,
        minute,
        second);

    Serial.println(
        "RTC SYNC SUCCESS");
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
        systemState.manualMode = jsonData.boolValue;
    }
    const bool startProvisioningRequested =
        snapshot.get(jsonData, "startProvisioning") && jsonData.success && jsonData.boolValue;

    const bool dispatchCommands = actuatorCommandsPrimed;
    consumeActuatorCommandSnapshot(snapshot, dispatchCommands);
    if (!actuatorCommandsPrimed)
    {
        actuatorCommandsPrimed = true;
        Serial.println("[MANUAL] Existing actuator commands consumed as reconnect baseline");
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
    consumeActuatorCommandSnapshot(snapshot, false);
    actuatorCommandsPrimed = true;
    Serial.println("[MANUAL] Existing actuator commands consumed as boot baseline");
}

void FirebaseManager::consumeActuatorCommandSnapshot(FirebaseJson& snapshot, bool dispatchCommands)
{
    bool hasCommand[ACTUATOR_COUNT] = { false };
    bool states[ACTUATOR_COUNT] = { false };
    String sources[ACTUATOR_COUNT];
    uint64_t timestamps[ACTUATOR_COUNT] = { 0 };
    uint8_t speeds[ACTUATOR_COUNT];
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
    }

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        if (!hasCommand[i]) continue;

        const Actuator actuator = static_cast<Actuator>(i);
        const String commandPath = deviceRoot() + "/commands/" + getActuatorName(actuator);
        const bool isNew = timestamps[i] > lastActuatorCommandTimestamps[i];

        if (isNew)
        {
            lastActuatorCommandTimestamps[i] = timestamps[i];
            saveActuatorCommandTimestamp(actuator, timestamps[i]);
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
                speeds[i]);
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
        if (sensors.waterLevel >= systemState.refillStartLevel)
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
    // 10-second cadence instead of hammering RTDB on every loop iteration.
    if (!force) lastSensorUploadAttempt = millis();

    FirebaseJson json;
    const SensorData& publishedSensors = snapshot ? *snapshot : sensors;

    //--------------------------------------------------
    // Environment
    //--------------------------------------------------

    if (!isnan(publishedSensors.temperature)) json.set("airTemperature", publishedSensors.temperature);
    if (!isnan(publishedSensors.humidity)) json.set("humidity", publishedSensors.humidity);

    //--------------------------------------------------
    // Reservoir
    //--------------------------------------------------

    if (!isnan(publishedSensors.waterTemp)) json.set("waterTemperature", publishedSensors.waterTemp);
    if (isfinite(publishedSensors.waterLevel)) json.set("waterLevel", publishedSensors.waterLevel);

    //--------------------------------------------------
    // Nutrient
    //--------------------------------------------------

    if (!isnan(publishedSensors.ec)) json.set("ec", publishedSensors.ec);
    if (!isnan(publishedSensors.tds)) json.set("tds", publishedSensors.tds);
    if (!isnan(publishedSensors.ph)) json.set("ph", publishedSensors.ph);

    //--------------------------------------------------
    // Metadata
    //--------------------------------------------------

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
        else if (!hasPublishedHeartbeat ||
                 millis() - lastHeartbeatSuccessLog >= HEARTBEAT_SUCCESS_LOG_INTERVAL_MS)
        {
            Serial.println("[PRESENCE] Heartbeat uploaded");
            lastHeartbeatSuccessLog = millis();
        }
        hasPublishedHeartbeat = true;

        if (force)
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

        Serial.println(
            "Sensors Uploaded");
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

    json.set(
        "manualMode",
        systemState.manualMode);

    json.set(
        "mockData",
        systemState.mockSensorsEnabled);

    json.set("sensorTest", systemState.sensorTestEnabled);

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

    const unsigned long uploadStartedAt = millis();
    const bool uploadSucceeded = updateJson(deviceRoot() + "/status", json);
    logFirebaseDuration("Status upload", millis() - uploadStartedAt);
    if(uploadSucceeded)
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
    if(uploadSucceeded)
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
    ADD_ALERT_FIELD(ecLow);
    ADD_ALERT_FIELD(ecHigh);
    ADD_ALERT_FIELD(phOutOfRange);
    ADD_ALERT_FIELD(phLow);
    ADD_ALERT_FIELD(phHigh);
    ADD_ALERT_FIELD(waterTempOutOfRange);
    ADD_ALERT_FIELD(lowAirTemperature);
    ADD_ALERT_FIELD(highTemperature);

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
        if (!alertCacheInitialized || \
            alertState.fieldName != lastPublishedAlerts.fieldName) { \
            Serial.print("[ALERT-SYNC] " #fieldName "="); \
            Serial.print(alertState.fieldName ? "true" : "false"); \
            Serial.print(" t="); \
            Serial.println(millis()); \
        } \
    } while (false)

    LOG_ALERT_TRANSITION(lowWater);
    LOG_ALERT_TRANSITION(ecLow);
    LOG_ALERT_TRANSITION(ecHigh);
    LOG_ALERT_TRANSITION(phOutOfRange);
    LOG_ALERT_TRANSITION(phLow);
    LOG_ALERT_TRANSITION(phHigh);
    LOG_ALERT_TRANSITION(waterTempOutOfRange);
    LOG_ALERT_TRANSITION(lowAirTemperature);
    LOG_ALERT_TRANSITION(highTemperature);

    if (sensorFaultIncluded && sensorFaultChanged)
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
            current.reason != lastPublishedActuators[i].reason;

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
            if (changed[i])
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
        json))
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

void FirebaseManager::readHarvestSchedule()
{
    // Same reasoning as readSmsRecipients(): cached locally, no need for a
    // reduced polling cadence tied to the round-robin rotation alone.
    static unsigned long lastHarvestScheduleRead = 0;
    if (millis() - lastHarvestScheduleRead < LOW_PRIORITY_READ_INTERVAL_MS)
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

    // A successful read with mock mode disabled resolves the effective source
    // to physical sensors even if optional mock payload fields are incomplete.
    if (!nextEnabled)
    {
        systemState.sensorSourceResolved = true;
    }

    SensorData nextMock = systemState.mockSensors;

    // Air Temperature
    json.get(data, "airTemperature");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextMock.temperature = data.to<float>();
    else 
        nextMock.temperature = NAN;

    // Humidity
    json.get(data, "humidity");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextMock.humidity = data.to<float>();
    else 
        nextMock.humidity = NAN;

    // Water Temperature
    json.get(data, "waterTemperature");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextMock.waterTemp = data.to<float>();
    else 
        nextMock.waterTemp = NAN;

    // Water Level
    json.get(data, "waterLevel");
    if (data.success &&
        (data.typeNum == FirebaseJson::JSON_FLOAT ||
         data.typeNum == FirebaseJson::JSON_DOUBLE ||
         data.typeNum == FirebaseJson::JSON_INT))
        nextMock.waterLevel = data.to<float>();
    else 
        nextMock.waterLevel = NAN;

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

    nextMock.ph = parsedPh;

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

    nextMock.ec = parsedEc;

    const bool enabledChanged = nextEnabled != systemState.mockSensorsEnabled;
    const bool payloadChanged =
        floatValuesDiffer(nextMock.temperature, systemState.mockSensors.temperature) ||
        floatValuesDiffer(nextMock.humidity, systemState.mockSensors.humidity) ||
        floatValuesDiffer(nextMock.waterTemp, systemState.mockSensors.waterTemp) ||
        floatValuesDiffer(nextMock.waterLevel, systemState.mockSensors.waterLevel) ||
        floatValuesDiffer(nextMock.ph, systemState.mockSensors.ph) ||
        floatValuesDiffer(nextMock.ec, systemState.mockSensors.ec);

    systemState.mockSensors = nextMock;
    systemState.sensorSourceResolved = true;

    if (!enabledChanged && !(nextEnabled && payloadChanged))
        return;

    systemState.mockSensorsEnabled = nextEnabled;
    systemState.mockApplyPending = true;

    if (nextEnabled)
    {
        Serial.println("[MOCK] Command received");
        Serial.println("[MOCK] enabled=true");
        Serial.print("[MOCK] pH="); Serial.println(nextMock.ph, 2);
        Serial.print("[MOCK] EC="); Serial.println(nextMock.ec, 2);
        Serial.print("[MOCK] AirTemp="); Serial.println(nextMock.temperature, 2);
        Serial.print("[MOCK] Humidity="); Serial.println(nextMock.humidity, 2);
        Serial.print("[MOCK] WaterTemp="); Serial.println(nextMock.waterTemp, 2);
        Serial.print("[MOCK] WaterLevel="); Serial.println(nextMock.waterLevel, 2);
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
        now - lastSuccessfulSensorUpload >= UPLOAD_INTERVAL)
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
