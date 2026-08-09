#include "FirebaseManager.h"
#include "Globals.h"
#include "Arduino.h"

namespace
{

constexpr unsigned long COMMAND_READ_INTERVAL  = 1500;
constexpr unsigned long MOCK_READ_INTERVAL     = 2000;
constexpr unsigned long SETTINGS_READ_INTERVAL = 60000;
constexpr unsigned long UPLOAD_INTERVAL        = 10000;
constexpr unsigned long DEVICE_INFO_INTERVAL   = 15000;
constexpr unsigned long REALTIME_FALLBACK_INTERVAL = 60000;

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

void FirebaseManager::begin()
{
    if (wifiManager.consumeFirebaseResumePending())
    {
        Serial.println("[FIREBASE] Resuming after Wi-Fi reconnect");
    }


    config.api_key = API_KEY;

    config.database_url = DATABASE_URL;

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

    Firebase.reconnectWiFi(true);

        loadDeviceId();
        loadActuatorCommandTimestamps();

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
    // baseline before publishing this boot as online. Commands written while
    // the device was offline must never execute as fresh hardware requests.
    primeActuatorCommands();

    initializeDatabase();

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

    systemState.currentMode = NORMAL;

    Serial.println("Firebase Started");
}
void FirebaseManager::initializeDatabase()
{

    Serial.println("Firebase RTDB onDisconnect rules registered.");

    // Mark online state immediately on both status and deviceInfo nodes
    FirebaseJson onlineJson;
    onlineJson.set("online", true);
    updateJson(deviceRoot() + "/status", onlineJson);
    updateJson(deviceRoot() + "/deviceInfo", onlineJson);

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

        json.set("minEC",
            systemState.minEC);

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
        wasFirebaseConnected = false;
        return;
    }

    if (!wifiManager.isConnected())
    {
        systemState.firebaseConnected = false;
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
        return;
    }

    //--------------------------------------------------
    // Realtime state transitions take priority over periodic traffic.
    //--------------------------------------------------

    const bool alertTransitionPublished = writeAlerts();
    writeActuators();

    // Keep the sensor snapshot consistent with a newly published alert edge.
    if (alertTransitionPublished)
    {
        writeSensors(true);
    }

    //--------------------------------------------------
    // Settings Synchronization
    //--------------------------------------------------

    if(millis() - lastSettingsRead >=
       SETTINGS_READ_INTERVAL)
    {
        lastSettingsRead =
            millis();

        readSettings();
    }

    //--------------------------------------------------
    // Command Processing
    //--------------------------------------------------

    readCommands();
    readActuatorCommands();

    // A manual command may have entered AP mode during readActuatorCommands().
    // Stop this update before any upload or additional Firebase operation.
    if (wifiManager.isProvisioningMode())
    {
        if (!suspendedForProvisioning)
        {
            Serial.println("[FIREBASE] Suspended during provisioning mode");
            suspendedForProvisioning = true;
        }
        systemState.firebaseConnected = false;
        wasFirebaseConnected = false;
        return;
    }

    //--------------------------------------------------
    // Device Uploads
    //--------------------------------------------------

    writeDeviceInfo();

    writeSensors();

    writeStatus();

    writeTelemetry();

    //--------------------------------------------------
    // Operation State Synchronization
    //--------------------------------------------------

    if(systemState.operationRequest.state ==
       lastPublishedOperationState)
    {
        return;
    }

    writeCurrentOperation();

    lastPublishedOperationState =
        systemState.operationRequest.state;

    RequestState state =
        systemState.operationRequest.state;

    if(state != RequestState::COMPLETED &&
       state != RequestState::FAILED &&
       state != RequestState::REJECTED)
    {
        return;
    }

    if(archiveCurrentOperation())
    {
        resetCurrentOperation();
    }
}

//==================================================
// Settings Synchronization
//==================================================

void FirebaseManager::readSettings()
{
    if(!Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/settings"))
    {
        return;
    }

    FirebaseJsonData data;

    //--------------------------------------------------
    // Grow Light
    //--------------------------------------------------

    if (fbdo.jsonObject().get(data, "lightOnHour"))
        systemState.lightOnHour = data.intValue;

    if (fbdo.jsonObject().get(data, "lightOnMinute"))
        systemState.lightOnMinute = data.intValue;

    if (fbdo.jsonObject().get(data, "lightOffHour"))
        systemState.lightOffHour = data.intValue;

    if (fbdo.jsonObject().get(data, "lightOffMinute"))
        systemState.lightOffMinute = data.intValue;

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
    if (incomingMinPH > 0.0f && incomingMaxPH > incomingMinPH)
    {
        systemState.minPH = incomingMinPH;
        systemState.maxPH = incomingMaxPH;
    }

    //--------------------------------------------------
    // EC
    //--------------------------------------------------

    if (fbdo.jsonObject().get(data, "minEC"))
    {
        if (data.floatValue > 0.0f)
            systemState.minEC = data.floatValue;
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
        const bool validStart = incomingRefillStart > 0.0f;
        const bool validStop = incomingRefillStop <= 100.0f;
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
                    Serial.println("refillStartLevel must be greater than 0");
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

    //--------------------------------------------------
    // Water cooling
    //--------------------------------------------------

    float incomingHighWater = systemState.highWaterTemp;
    float incomingCoolerOff = systemState.coolerOffTemp;

    if (fbdo.jsonObject().get(data, "highWaterTemp"))
        incomingHighWater = data.floatValue;

    if (fbdo.jsonObject().get(data, "coolerOffTemp"))
        incomingCoolerOff = data.floatValue;

    if (incomingHighWater > 0.0f && incomingHighWater > incomingCoolerOff)
    {
        systemState.highWaterTemp = incomingHighWater;
        systemState.coolerOffTemp = incomingCoolerOff;
    }
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

    if(millis() - lastCommandRead < COMMAND_READ_INTERVAL)
    {
        return;
    }

    lastCommandRead = millis();


    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/commands/current"))
    {
        return;
    }

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
    // Duplicate Request
    //--------------------------------------------------

    if(isDuplicateRequest(requestId))
    {
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
    if(millis() - lastCommandRead < COMMAND_READ_INTERVAL) return;
    lastCommandRead = millis();

    if(!Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands")) return;

    FirebaseJson& snapshot = fbdo.jsonObject();
    FirebaseJsonData jsonData;

    // Check for manualMode flag
    if (snapshot.get(jsonData, "manualMode"))
    {
        systemState.manualMode = jsonData.boolValue;
    }
    const bool startProvisioningRequested =
        snapshot.get(jsonData, "startProvisioning") && jsonData.success;

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
        Firebase.RTDB.deleteNode(&fbdo, deviceRoot() + "/commands/startProvisioning");
        if (!suspendedForProvisioning)
        {
            suspendedForProvisioning = true;
        }
        systemState.firebaseConnected = false;
        wasFirebaseConnected = false;
        wifiManager.startAP();
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

void FirebaseManager::writeSensors(bool force)
{
    static unsigned long lastSensorUpload = 0;

    if(!force && millis() - lastSensorUpload < UPLOAD_INTERVAL)
    {
        return;
    }

    FirebaseJson json;

    //--------------------------------------------------
    // Environment
    //--------------------------------------------------

    if (!isnan(sensors.temperature)) json.set("airTemperature", sensors.temperature);
    if (!isnan(sensors.humidity)) json.set("humidity", sensors.humidity);

    //--------------------------------------------------
    // Reservoir
    //--------------------------------------------------

    if (!isnan(sensors.waterTemp)) json.set("waterTemperature", sensors.waterTemp);
    if (isfinite(sensors.waterLevel)) json.set("waterLevel", sensors.waterLevel);

    //--------------------------------------------------
    // Nutrient
    //--------------------------------------------------

    if (!isnan(sensors.ec)) json.set("ec", sensors.ec);
    if (!isnan(sensors.tds)) json.set("tds", sensors.tds);
    if (!isnan(sensors.ph)) json.set("ph", sensors.ph);

    //--------------------------------------------------
    // Metadata
    //--------------------------------------------------

    json.set(
        "timestamp",
        millis());

    if(writeJson(
        deviceRoot() + "/sensors",
        json))
    {
        lastSensorUpload = millis();

        if (force)
        {
            Serial.print("[SENSOR-SYNC] waterLevel=");
            Serial.print(sensors.waterLevel, 2);
            Serial.print(" ph=");
            Serial.print(sensors.ph, 2);
            Serial.print(" ec=");
            Serial.print(sensors.ec, 2);
            Serial.print(" t=");
            Serial.println(millis());
        }

        Serial.println(
            "Sensors Uploaded");
    }
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

    //--------------------------------------------------
    // Safety
    //--------------------------------------------------

    json.set(
        "reservoirLocked",
        systemState.reservoirLocked);

    json.set(
        "safetyLock",
        systemState.safetyLock);

    //--------------------------------------------------
    // Connectivity
    //--------------------------------------------------

    json.set(
        "wifiConnected",
        systemState.wifiConnected);

    json.set(
        "firebaseConnected",
        systemState.firebaseConnected);

    if(updateJson(
        deviceRoot() + "/status",
        json))
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

    if(writeJson(
        deviceRoot() + "/telemetry",
        json))
    {
        Serial.println(
            "Telemetry Uploaded");
    }
}

bool FirebaseManager::writeAlerts()
{
    const bool fullUploadDue =
        !alertCacheInitialized ||
        millis() - lastAlertFullUpload >= REALTIME_FALLBACK_INTERVAL;

    if (!alertManager.isDirty() && !fullUploadDue)
    {
        return false;
    }

    FirebaseJson json;
    bool hasFields = false;
    bool hasTransition = false;

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
    ADD_ALERT_FIELD(phOutOfRange);
    ADD_ALERT_FIELD(waterTempOutOfRange);
    ADD_ALERT_FIELD(highTemperature);
    ADD_ALERT_FIELD(sensorFault);

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
    LOG_ALERT_TRANSITION(phOutOfRange);
    LOG_ALERT_TRANSITION(waterTempOutOfRange);
    LOG_ALERT_TRANSITION(highTemperature);
    LOG_ALERT_TRANSITION(sensorFault);

#undef LOG_ALERT_TRANSITION

    lastPublishedAlerts = alertState;
    alertCacheInitialized = true;
    if (fullUploadDue) lastAlertFullUpload = millis();
    alertManager.markSynced();
    return hasTransition;
}

void FirebaseManager::writeActuators()
{
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
        "online",
        systemState.firebaseConnected);

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

    if(!success)
    {
        Serial.print("Firebase Update Failed: ");
        Serial.println(path);
        Serial.println(fbdo.errorReason());
    }

    return success;
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
    if(millis() - lastMockRead < MOCK_READ_INTERVAL) return;
    lastMockRead = millis();

    if(!Firebase.RTDB.getJSON(&fbdo, deviceRoot() + "/commands/mockSensors")) {
        // A transient read failure must not silently change sensor authority.
        return;
    }

    FirebaseJsonData data;
    FirebaseJson& json = fbdo.jsonObject();

    json.get(data, "enabled");
    const bool nextEnabled = data.success && data.boolValue;
    SensorData nextMock = systemState.mockSensors;

    // Air Temperature
    json.get(data, "airTemperature");
    if (data.success && (data.typeNum == FirebaseJson::JSON_DOUBLE || data.typeNum == FirebaseJson::JSON_INT)) 
        nextMock.temperature = data.floatValue;
    else 
        nextMock.temperature = NAN;

    // Humidity
    json.get(data, "humidity");
    if (data.success && (data.typeNum == FirebaseJson::JSON_DOUBLE || data.typeNum == FirebaseJson::JSON_INT)) 
        nextMock.humidity = data.floatValue;
    else 
        nextMock.humidity = NAN;

    // Water Temperature
    json.get(data, "waterTemperature");
    if (data.success && (data.typeNum == FirebaseJson::JSON_DOUBLE || data.typeNum == FirebaseJson::JSON_INT)) 
        nextMock.waterTemp = data.floatValue;
    else 
        nextMock.waterTemp = NAN;

    // Water Level
    json.get(data, "waterLevel");
    if (data.success && (data.typeNum == FirebaseJson::JSON_DOUBLE || data.typeNum == FirebaseJson::JSON_INT)) 
        nextMock.waterLevel = data.floatValue;
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
