#include "FirebaseManager.h"
#include "Globals.h"
#include "Arduino.h"

namespace
{

constexpr unsigned long COMMAND_READ_INTERVAL  = 5000;
constexpr unsigned long SETTINGS_READ_INTERVAL = 60000;
constexpr unsigned long UPLOAD_INTERVAL        = 10000;
constexpr unsigned long DEVICE_INFO_INTERVAL   = 60000;

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

} // namespace


//==================================================
// Initialization
//==================================================

void FirebaseManager::begin()
{

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

    initializeDatabase();

    readSettings();

    syncRTC();

    systemState.settingsLoaded = true;

    systemState.syncRTC = true;

    systemState.currentMode = NORMAL;

    Serial.println("Firebase Started");
}
void FirebaseManager::initializeDatabase()
{

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

        writeJson(
            deviceRoot() + "/settings",
            json);
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

    systemState.firebaseConnected =
        Firebase.ready();

    if(!Firebase.ready())
    {
        return;
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

    //--------------------------------------------------
    // Device Uploads
    //--------------------------------------------------

    writeDeviceInfo();

    writeSensors();

    writeStatus();

    writeTelemetry();

    writeAlerts();

    writeActuators();

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

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        deviceRoot() + "/settings"))
    {
        return;
    }

    FirebaseJsonData data;

    //--------------------------------------------------
    // Grow Light
    //--------------------------------------------------

    fbdo.jsonObject().get(data, "lightOnHour");
    systemState.lightOnHour = data.intValue;

    fbdo.jsonObject().get(data, "lightOnMinute");
    systemState.lightOnMinute = data.intValue;

    fbdo.jsonObject().get(data, "lightOffHour");
    systemState.lightOffHour = data.intValue;

    fbdo.jsonObject().get(data, "lightOffMinute");
    systemState.lightOffMinute = data.intValue;

    //--------------------------------------------------
    // pH
    //--------------------------------------------------

    fbdo.jsonObject().get(data, "minPH");
    systemState.minPH = data.floatValue;

    fbdo.jsonObject().get(data, "maxPH");
    systemState.maxPH = data.floatValue;

    //--------------------------------------------------
    // EC
    //--------------------------------------------------

    fbdo.jsonObject().get(data, "minEC");
    systemState.minEC = data.floatValue;

    //--------------------------------------------------
    // Reservoir
    //--------------------------------------------------

    fbdo.jsonObject().get(data, "refillStartLevel");
    systemState.refillStartLevel = data.floatValue;

    fbdo.jsonObject().get(data, "refillStopLevel");
    systemState.refillStopLevel = data.floatValue;

    //--------------------------------------------------
    // Cooling
    //--------------------------------------------------

    fbdo.jsonObject().get(data, "highWaterTemp");
    systemState.highWaterTemp = data.floatValue;

    fbdo.jsonObject().get(data, "coolerOffTemp");
    systemState.coolerOffTemp = data.floatValue;
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
    
    FirebaseJsonData jsonData;
    
    // Check for manualMode flag
    fbdo.jsonObject().get(jsonData, "manualMode");
    if (jsonData.success)
    {
        systemState.manualMode = jsonData.boolValue;
    }

    
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        Actuator a = (Actuator)i;
        String name = getActuatorName(a);
        
        bool hasCommand = false;
        bool state = false;
        String source = "";
        double timestamp = 0;

        fbdo.jsonObject().get(jsonData, name + "/state");
        if (jsonData.success)
        {
            state = jsonData.boolValue;
            hasCommand = true;
        }

        fbdo.jsonObject().get(jsonData, name + "/source");
        if (jsonData.success)
        {
            source = jsonData.stringValue;
        }

        fbdo.jsonObject().get(jsonData, name + "/timestamp");
        if (jsonData.success)
        {
            timestamp = jsonData.doubleValue;
        }

        uint8_t speed = 100;
        fbdo.jsonObject().get(jsonData, name + "/speed");
        if (jsonData.success)
        {
            speed = jsonData.intValue;
        }

        if (hasCommand)
        {
            static double lastTimestamps[ACTUATOR_COUNT] = {0};
            if (timestamp > lastTimestamps[i])
            {
                lastTimestamps[i] = timestamp;
                actuatorManager.requestCommand(a, state, source, timestamp, speed);
            }
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

void FirebaseManager::writeSensors()
{
    static unsigned long lastSensorUpload = 0;

    if(millis() - lastSensorUpload < UPLOAD_INTERVAL)
    {
        return;
    }

    lastSensorUpload = millis();

    FirebaseJson json;

    //--------------------------------------------------
    // Environment
    //--------------------------------------------------

    json.set(
        "airTemperature",
        sensors.temperature);

    json.set(
        "humidity",
        sensors.humidity);

    //--------------------------------------------------
    // Reservoir
    //--------------------------------------------------

    json.set(
        "waterTemperature",
        sensors.waterTemp);

    json.set(
        "waterLevel",
        sensors.waterLevel);

    //--------------------------------------------------
    // Nutrient
    //--------------------------------------------------

    json.set(
        "ec",
        sensors.ec);

    json.set(
        "tds",
        sensors.tds);

    json.set(
        "ph",
        sensors.ph);

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

    if(writeJson(
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

void FirebaseManager::writeAlerts()
{
    static unsigned long lastAlertUpload = 0;

    if(millis() - lastAlertUpload < UPLOAD_INTERVAL)
    {
        return;
    }

    lastAlertUpload = millis();

    FirebaseJson json;

    //--------------------------------------------------
    // Water
    //--------------------------------------------------

    json.set(
        "lowWater",
        alertState.lowWater);

    //--------------------------------------------------
    // Nutrient
    //--------------------------------------------------

    json.set(
        "ecLow",
        alertState.ecLow);

    json.set(
        "phOutOfRange",
        alertState.phOutOfRange);

    //--------------------------------------------------
    // Temperature
    //--------------------------------------------------

    json.set(
        "waterTempOutOfRange",
        alertState.waterTempOutOfRange);

    json.set(
        "highTemperature",
        alertState.highTemperature);

    //--------------------------------------------------
    // System
    //--------------------------------------------------

    json.set(
        "sensorFault",
        alertState.sensorFault);

    if(writeJson(
        deviceRoot() + "/alerts",
        json))
    {
        Serial.println(
            "Alerts Uploaded");
    }
}

void FirebaseManager::writeActuators()
{
    FirebaseJson json;
    bool changed = false;

    static ActuatorStatus lastStatus[ACTUATOR_COUNT];

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        Actuator a = (Actuator)i;
        ActuatorStatus current = actuatorManager.getStatus(a);

        if (current.state != lastStatus[i].state || 
            current.running != lastStatus[i].running || 
            current.speed != lastStatus[i].speed ||
            current.reason != lastStatus[i].reason)
        {
            changed = true;
            lastStatus[i] = current;
        }

        String name = getActuatorName(a);
        
        json.set(name + "/running", current.running);
        json.set(name + "/state", (int)current.state);
        json.set(name + "/speed", current.speed);
        json.set(name + "/startedAt", current.startedAt);
        json.set(name + "/source", current.source);
        json.set(name + "/reason", current.reason);
    }

    if (!changed)
    {
        return;
    }

    if (writeJson(deviceRoot() + "/actuatorStatus", json))
    {
        Serial.println("Actuator Status Uploaded");
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

const String& FirebaseManager::getDeviceId() const
{
    return deviceId;
}

String FirebaseManager::deviceRoot() const
{
    return "/devices/" + deviceId;
}

