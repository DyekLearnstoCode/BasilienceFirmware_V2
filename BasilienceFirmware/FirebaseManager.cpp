
#include <Firebase_ESP_Client.h>
#include <WiFi.h>
#include "Config.h"
#include "Globals.h"

namespace
{

    OperationType toOperationType(const String& value)
    {
        if (value == "PH_UP")
            return OperationType::PH_UP;

        if (value == "PH_DOWN")
            return OperationType::PH_DOWN;

        if (value == "GROW_PUMP")
            return OperationType::GROW_PUMP;

        if (value == "BLOOM_PUMP")
            return OperationType::BLOOM_PUMP;

        if (value == "REFILL")
            return OperationType::REFILL;

        if (value == "FOGGER")
            return OperationType::FOGGER;

        if (value == "CANOPY_FAN")
            return OperationType::CANOPY_FAN;

        if (value == "PELTIER")
            return OperationType::PELTIER;

        if (value == "GROW_LIGHT")
            return OperationType::GROW_LIGHT;

        if (value == "RESTART_ESP")
            return OperationType::RESTART_ESP;

        if (value == "RESET_SAFETY")
            return OperationType::RESET_SAFETY;

        return OperationType::NONE;
    }

    OperationAction toOperationAction(const String& value)
    {
        if (value == "START")
            return OperationAction::START;

        if (value == "STOP")
            return OperationAction::STOP;

        if (value == "ENABLE")
            return OperationAction::ENABLE;

        if (value == "DISABLE")
            return OperationAction::DISABLE;

        if (value == "EXECUTE")
            return OperationAction::EXECUTE;

        return OperationAction::NONE;
    }

    RequestState toRequestState(const String& value)
    {
        if (value == "ACCEPTED")
            return RequestState::ACCEPTED;

        if (value == "PENDING")
            return RequestState::PENDING;

        if (value == "RUNNING")
            return RequestState::RUNNING;

        if (value == "COMPLETED")
            return RequestState::COMPLETED;

        if (value == "REJECTED")
            return RequestState::REJECTED;

        if (value == "FAILED")
            return RequestState::FAILED;

        return RequestState::IDLE;
    }

    RequestSource toRequestSource(const String& value)
    {
        if (value == "MANUAL")
            return RequestSource::MANUAL;

        if (value == "AUTOMATIC")
            return RequestSource::AUTOMATIC;

        return RequestSource::NONE;
    }

    const char* operationToString(OperationType operation)
        {
            switch(operation)
            {
                case OperationType::PH_UP:
                    return "PH_UP";

                case OperationType::PH_DOWN:
                    return "PH_DOWN";

                case OperationType::GROW_PUMP:
                    return "GROW_PUMP";

                case OperationType::BLOOM_PUMP:
                    return "BLOOM_PUMP";

                case OperationType::REFILL:
                    return "REFILL";

                case OperationType::FOGGER:
                    return "FOGGER";

                case OperationType::CANOPY_FAN:
                    return "CANOPY_FAN";

                case OperationType::PELTIER:
                    return "PELTIER";

                case OperationType::GROW_LIGHT:
                    return "GROW_LIGHT";

                case OperationType::RESTART_ESP:
                    return "RESTART_ESP";

                case OperationType::RESET_SAFETY:
                    return "RESET_SAFETY";

                default:
                    return "NONE";
            }
        }

        const char* actionToString(
            OperationAction action)
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

        const char* requestStateToString(
            RequestState state)
        {
            switch(state)
            {
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

                case RequestState::PENDING:
                    return "PENDING";

                default:
                    return "IDLE";
            }
        }

}

bool FirebaseManager::hasActiveOperation() const
{
    switch (systemState.operationRequest.state)
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

void FirebaseManager::createOperationRequest(
    uint16_t requestId,
    OperationType operation,
    OperationAction action,
    RequestSource source)
    {
        OperationRequest& request =
            systemState.operationRequest;

        request.requestId =
            requestId;

        request.operation =
            operation;

        request.action =
            action;

        request.source =
            source;

        request.state =
            RequestState::ACCEPTED;

        request.reason[0] =
            '\0';

        unsigned long now = millis();

        request.requestTimestamp = now;
        request.acceptedTimestamp = now;
        request.startedTimestamp = 0;
        request.completedTimestamp = 0;
        request.lastUpdatedTimestamp = now;

        systemState.lastProcessedRequestId =
            requestId;
}

bool FirebaseManager::validateOperationRequest(OperationType operation,OperationAction action, String& reason)
{
        if(operation ==
        OperationType::NONE)
        {
            reason =
                "Invalid operation";

            return false;
        }

        if(action ==
        OperationAction::NONE)
        {
            reason =
                "Invalid action";

            return false;
        }

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
    String root =
        "/devices/" +
        String(DEVICE_ID);

    FirebaseJson operationJson;

    operationJson.set(
        "requestId",
        requestId);

    operationJson.set(
        "operation",
        "NONE");

    operationJson.set(
        "action",
        "NONE");

    operationJson.set(
        "state",
        "REJECTED");

    operationJson.set(
        "reason",
        reason);

    operationJson.set(
        "requestTimestamp",
        0);

    operationJson.set(
        "acceptedTimestamp",
        0);

    operationJson.set(
        "startedTimestamp",
        0);

    operationJson.set(
        "completedTimestamp",
        0);

    operationJson.set(
        "lastUpdatedTimestamp",
        millis());

    operationJson.set(
        "protocolVersion",
        1);

    writeJson(
        root + "/operations/current",
        operationJson);
}

void FirebaseManager::begin()
{
    Serial.println();
    Serial.println("Connecting WiFi...");

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD);

    while(WiFi.status() !=
          WL_CONNECTED)
    {
        delay(500);

        Serial.print(".");
    }

    Serial.println();
    Serial.println(
        "WiFi Connected");

    systemState.wifiConnected =
    true;

    Serial.print(
        "IP Address: ");

    Serial.println(
        WiFi.localIP());

    config.api_key =
    API_KEY;

    config.database_url =
        DATABASE_URL;

    if (Firebase.signUp(
            &config,
            &auth,
            "",
            ""))
    {
        Serial.println(
            "Firebase SignUp OK");
    }
    else
    {
        Serial.print(
            "Firebase SignUp Failed: ");

        Serial.println(
            config.signer.signupError.message.c_str());
    }

    Firebase.begin(
        &config,
        &auth);

    Firebase.reconnectWiFi(
        true);

    systemState.firebaseConnected =
    true;

    initializeDatabase();

    Serial.println(
        "Firebase Started");

        
}

void FirebaseManager::initializeDatabase()
{
    String root =
        "/devices/" +
        String(DEVICE_ID);

    FirebaseJson json;
    FirebaseJsonData data;

    // =====================================================
    // Settings
    // =====================================================
    if (!Firebase.RTDB.getJSON(
            &fbdo,
            root + "/settings"))
    {
        json.clear();

        json.set("lightOnHour", systemState.lightOnHour);
        json.set("lightOnMinute", systemState.lightOnMinute);
        json.set("lightOffHour", systemState.lightOffHour);
        json.set("lightOffMinute", systemState.lightOffMinute);

        json.set("minPH", systemState.minPH);
        json.set("maxPH", systemState.maxPH);
        json.set("minEC", systemState.minEC);

        json.set(
            "refillStartLevel",
            systemState.refillStartLevel);

        json.set(
            "refillStopLevel",
            systemState.refillStopLevel);

        json.set(
            "highWaterTemp",
            systemState.highWaterTemp);

        json.set(
            "coolerOffTemp",
            systemState.coolerOffTemp);

        writeJson(
        root + "/settings",
        json);
    }

    // =====================================================
    // Commands
    // =====================================================
    if (!Firebase.RTDB.getJSON(
            &fbdo,
            root + "/commands/current"))
    {
        json.clear();

        json.set("requestId", 0);
        json.set("operation", "NONE");
        json.set("action", "NONE");
        json.set("requestTimestamp", 0);
        json.set("protocolVersion", 1);

        writeJson(
            root + "/commands/current",
            json);
    }

    // =====================================================
    // Operations
    // =====================================================
    if (!Firebase.RTDB.getJSON(
            &fbdo,
            root + "/operations/current"))
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
            root + "/operations/current",
            json);
    }

    // =====================================================
    // RTC
    // =====================================================
    if (!Firebase.RTDB.getJSON(
            &fbdo,
            root + "/rtc"))
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
            root + "/rtc",
            json);
    }
}

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
        Serial.print(
            "Firebase Write Failed: ");

        Serial.println(
            fbdo.errorReason());
    }

    return success;
}

bool FirebaseManager::writeCurrentOperation()
{
    String root =
        "/devices/" +
        String(DEVICE_ID);

    OperationRequest& request =
        systemState.operationRequest;

    FirebaseJson operationJson;

    operationJson.set(
        "requestId",
        request.requestId);

    operationJson.set(
        "operation",
        operationToString(
            request.operation));

    operationJson.set(
        "action",
        actionToString(
            request.action));

    operationJson.set(
        "state",
        requestStateToString(
            request.state));

    operationJson.set(
        "reason",
        request.reason);

    operationJson.set(
        "requestTimestamp",
        request.requestTimestamp);

    operationJson.set(
        "acceptedTimestamp",
        request.acceptedTimestamp);

    operationJson.set(
        "startedTimestamp",
        request.startedTimestamp);

    operationJson.set(
        "completedTimestamp",
        request.completedTimestamp);

    operationJson.set(
        "lastUpdatedTimestamp",
        request.lastUpdatedTimestamp);

    operationJson.set(
        "protocolVersion",
        1);

    return writeJson(
    root + "/operations/current",
    operationJson);
}

bool FirebaseManager::archiveCurrentOperation()
{
    OperationRequest& request =
        systemState.operationRequest;

    FirebaseJson operationJson;

    operationJson.set(
        "requestId",
        request.requestId);

    operationJson.set(
        "operation",
        operationToString(
            request.operation));

    operationJson.set(
        "action",
        actionToString(
            request.action));

    operationJson.set(
        "state",
        requestStateToString(
            request.state));

    operationJson.set(
        "reason",
        request.reason);

    operationJson.set(
        "requestTimestamp",
        request.requestTimestamp);

    operationJson.set(
        "acceptedTimestamp",
        request.acceptedTimestamp);

    operationJson.set(
        "startedTimestamp",
        request.startedTimestamp);

    operationJson.set(
        "completedTimestamp",
        request.completedTimestamp);

    operationJson.set(
        "lastUpdatedTimestamp",
        request.lastUpdatedTimestamp);

    operationJson.set(
        "protocolVersion",
        1);

    String path =
        "/devices/" +
        String(DEVICE_ID) +
        "/operations/history/" +
        String(request.requestId);

    return writeJson(
        path,
        operationJson);
}

void FirebaseManager::resetCurrentOperation()
{
    uint32_t lastProcessedId =
        systemState.lastProcessedRequestId;

    systemState.operationRequest =
        OperationRequest{};

    systemState.operationRequest.state =
        RequestState::IDLE;

    systemState.lastProcessedRequestId =
        lastProcessedId;

    lastPublishedOperationState =
        RequestState::IDLE;
}

void FirebaseManager::updateOperationState(
    RequestState newState)
{
    OperationRequest& request =
        systemState.operationRequest;

    unsigned long now =
        millis();

    request.state =
        newState;

    switch(newState)
    {
        case RequestState::ACCEPTED:
            request.acceptedTimestamp =
                now;
            break;

        case RequestState::RUNNING:
            request.startedTimestamp =
                now;
            break;

        case RequestState::COMPLETED:
            request.completedTimestamp =
                now;
            break;

        default:
            break;
    }

    request.lastUpdatedTimestamp =
        now;
}

void FirebaseManager::update()
{
    systemState.firebaseConnected =
        Firebase.ready();

    if (!Firebase.ready())
        return;

    if (millis() - lastSettingsRead >= 60000)
    {
        lastSettingsRead = millis();

        readSettings();
    }

    readCommands();

    writeDeviceInfo();

    writeSensors();

    writeStatus();

    writeTelemetry();

    writeAlerts();

    writeActuators();

    // ====================================
    // Publish Operation State Changes
    // ====================================

    if(systemState.operationRequest.state !=
    lastPublishedOperationState)
    {
        writeCurrentOperation();

        lastPublishedOperationState =
            systemState.operationRequest.state;

        RequestState state =
            systemState.operationRequest.state;

        if(state == RequestState::COMPLETED ||
        state == RequestState::FAILED ||
        state == RequestState::REJECTED)
        {
            if(archiveCurrentOperation())
            {
                resetCurrentOperation();
            }
        }
    }
}

void FirebaseManager::readSettings()
{
     FirebaseJsonData data;
     
    String root =
        "/devices/" +
        String(DEVICE_ID);

    String path =
        root +
        "/settings";

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        path))
    {
        return;
    }

    fbdo.jsonObject().get(
        data,
        "lightOnHour");

    systemState.lightOnHour =
        data.intValue;

    fbdo.jsonObject().get(
        data,
        "lightOnMinute");

    systemState.lightOnMinute =
        data.intValue;

    fbdo.jsonObject().get(
        data,
        "lightOffHour");

    systemState.lightOffHour =
        data.intValue;

    fbdo.jsonObject().get(
        data,
        "lightOffMinute");

    systemState.lightOffMinute =
        data.intValue;

    fbdo.jsonObject().get(
        data,
        "minPH");

    systemState.minPH =
        data.floatValue;

    fbdo.jsonObject().get(
        data,
        "maxPH");

    systemState.maxPH =
        data.floatValue;

    fbdo.jsonObject().get(
        data,
        "minEC");

    systemState.minEC =
        data.floatValue;

    fbdo.jsonObject().get(
    data,
    "refillStartLevel");

    systemState.refillStartLevel =
        data.floatValue;

    fbdo.jsonObject().get(
        data,
        "refillStopLevel");

    systemState.refillStopLevel =
        data.floatValue;

    fbdo.jsonObject().get(
        data,
        "highWaterTemp");

    systemState.highWaterTemp =
        data.floatValue;

    fbdo.jsonObject().get(
        data,
        "coolerOffTemp");

    systemState.coolerOffTemp =
        data.floatValue;
}

void FirebaseManager::readCommands()
{
    static unsigned long lastCommandRead = 0;

    if (millis() - lastCommandRead < 5000)
        return;

    lastCommandRead = millis();

    String root =
        "/devices/" +
        String(DEVICE_ID);

    String path =
        root +
        "/commands/current";

    if (!Firebase.RTDB.getJSON(
            &fbdo,
            path))
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

    if (protocolVersion != 1)
    {
        rejectOperationRequest(
            requestId,
            "Unsupported protocol version.");
        return;
    }

    if (requestTimestamp == 0)
    {
        rejectOperationRequest(
            requestId,
            "Invalid request timestamp.");
        return;
    }

    OperationType operation =
        toOperationType(operationString);

    if (operation == OperationType::NONE)
    {
        rejectOperationRequest(
            requestId,
            "Invalid operation.");
        return;
    }

    OperationAction action =
        toOperationAction(actionString);

    if (action == OperationAction::NONE)
    {
        rejectOperationRequest(
            requestId,
            "Invalid action.");
        return;
    }

    if (isDuplicateRequest(requestId))
    {
        return;
    }

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

    createOperationRequest(
        requestId,
        operation,
        action,
        RequestSource::MANUAL);
}

void FirebaseManager::syncRTC()
{

     String root =
        "/devices/" +
        String(DEVICE_ID);

    String path =
        root +
        "/rtc";

    FirebaseJsonData data;

    if(!Firebase.RTDB.getJSON(
        &fbdo,
        path))
    {
        Serial.println(
            "RTC SYNC FAILED");

        return;
    }

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    fbdo.jsonObject().get(
        data,
        "year");
    year = data.intValue;

    fbdo.jsonObject().get(
        data,
        "month");
    month = data.intValue;

    fbdo.jsonObject().get(
        data,
        "day");
    day = data.intValue;

    fbdo.jsonObject().get(
        data,
        "hour");
    hour = data.intValue;

    fbdo.jsonObject().get(
        data,
        "minute");
    minute = data.intValue;

    fbdo.jsonObject().get(
        data,
        "second");
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

void FirebaseManager::writeSensors()
{
    static unsigned long lastSensorUpload = 0;

    if(millis() - lastSensorUpload < 10000)
        return;

    lastSensorUpload = millis();

    FirebaseJson sensorJson;

    sensorJson.set(
        "airTemperature",
        sensors.temperature);

    sensorJson.set(
        "humidity",
        sensors.humidity);

    sensorJson.set(
        "waterTemperature",
        sensors.waterTemp);

    sensorJson.set(
        "waterLevel",
        sensors.waterLevel);

    sensorJson.set(
        "ec",
        sensors.ec);

    sensorJson.set(
        "tds",
        sensors.tds);

    sensorJson.set(
        "ph",
        sensors.ph);

    sensorJson.set(
    "timestamp",
    millis());

    String root =
    "/devices/" +
    String(DEVICE_ID);

    writeJson(
        root + "/sensors",
        sensorJson);
        {
            Serial.println(
                "Sensors Uploaded");
        }
}

void FirebaseManager::writeStatus()
{
    static unsigned long lastStatusUpload = 0;

    if(millis() - lastStatusUpload < 10000)
        return;

    lastStatusUpload = millis();

    FirebaseJson statusJson;

    statusJson.set(
    "currentMode",
    (int)systemState.currentMode);

    statusJson.set(
        "manualMode",
        systemState.manualMode);

    statusJson.set(
        "reservoirLocked",
        systemState.reservoirLocked);

    statusJson.set(
    "safetyLock",
    systemState.safetyLock);

    statusJson.set(
        "wifiConnected",
        systemState.wifiConnected);

    statusJson.set(
        "firebaseConnected",
        systemState.firebaseConnected);

    String root =
    "/devices/" +
    String(DEVICE_ID);

    writeJson(
        root + "/status",
        statusJson);
        {
            Serial.println(
                "Status Uploaded");
        }
}

void FirebaseManager::writeTelemetry()
{
    static unsigned long
        lastTelemetryUpload = 0;

    if(millis() -
       lastTelemetryUpload <
       10000)
    {
        return;
    }

    lastTelemetryUpload =
        millis();

    String root =
        "/devices/" +
        String(DEVICE_ID);

    FirebaseJson telemetryJson;

    telemetryJson.set(
        "phAttempts",
        systemState.phAttempts);

    telemetryJson.set(
        "ecAttempts",
        systemState.ecAttempts);

    telemetryJson.set(
        "phDoseTime",
        systemState.phDoseTime);

    telemetryJson.set(
        "ecDoseTime",
        systemState.ecDoseTime);

    writeJson(
        root + "/telemetry",
        telemetryJson);
}

void FirebaseManager::writeAlerts()
{
    String root =
    "/devices/" +
    String(DEVICE_ID);

    static unsigned long lastAlertUpload = 0;

    if(millis() - lastAlertUpload < 10000)
        return;

    lastAlertUpload = millis();

    FirebaseJson alertJson;

    alertJson.set(
        "lowWater",
        alertState.lowWater);

    alertJson.set(
        "ecLow",
        alertState.ecLow);

    alertJson.set(
        "phOutOfRange",
        alertState.phOutOfRange);

    alertJson.set(
        "waterTempOutOfRange",
        alertState.waterTempOutOfRange);

    alertJson.set(
        "highTemperature",
        alertState.highTemperature);

    alertJson.set(
        "sensorFault",
        alertState.sensorFault);

        if(writeJson(
        root + "/alerts",
        alertJson))
    {
        Serial.println(
            "Alerts Uploaded");
    }
}

void FirebaseManager::writeActuators()
{
    String root =
    "/devices/" +
    String(DEVICE_ID);
    
    ActuatorTelemetry current;

    current.fogger =
        actuatorManager.isOn(
            FOGGER);

    current.growLight =
        actuatorManager.isOn(
            GROW_LIGHT);

    current.blower =
        actuatorManager.isOn(
            BLOWER);

    current.solenoid =
        actuatorManager.isOn(
            SOLENOID);

    current.growPump =
        actuatorManager.isOn(
            GROW_PUMP);

    current.bloomPump =
        actuatorManager.isOn(
            BLOOM_PUMP);

    current.phUpPump =
        actuatorManager.isOn(
            PH_UP_PUMP);

    current.phDownPump =
        actuatorManager.isOn(
            PH_DOWN_PUMP);

    current.peltier =
        actuatorManager.isOn(
            PELTIER);

    bool changed = false;

    if(current.fogger !=
       lastActuatorState.fogger)
        changed = true;

    if(current.growLight !=
       lastActuatorState.growLight)
        changed = true;

    if(current.blower !=
       lastActuatorState.blower)
        changed = true;

    if(current.solenoid !=
       lastActuatorState.solenoid)
        changed = true;

    if(current.growPump !=
       lastActuatorState.growPump)
        changed = true;

    if(current.bloomPump !=
       lastActuatorState.bloomPump)
        changed = true;

    if(current.phUpPump !=
       lastActuatorState.phUpPump)
        changed = true;

    if(current.phDownPump !=
       lastActuatorState.phDownPump)
        changed = true;

    if(current.peltier !=
       lastActuatorState.peltier)
        changed = true;

    if(!changed)
        return;

    FirebaseJson actuatorJson;

    actuatorJson.set(
        "fogger",
        current.fogger);

    actuatorJson.set(
        "growLight",
        current.growLight);

    actuatorJson.set(
        "blower",
        current.blower);

    actuatorJson.set(
        "solenoid",
        current.solenoid);

    actuatorJson.set(
        "growPump",
        current.growPump);

    actuatorJson.set(
        "bloomPump",
        current.bloomPump);

    actuatorJson.set(
        "phUpPump",
        current.phUpPump);

    actuatorJson.set(
        "phDownPump",
        current.phDownPump);

    actuatorJson.set(
        "peltier",
        current.peltier);

    if(writeJson(
        root + "/actuators",
        actuatorJson))
    {
        lastActuatorState =
            current;

        Serial.println(
            "Actuator State Changed");
    }
}

void FirebaseManager::writeDeviceInfo()
{
    static unsigned long
    lastDeviceInfoUpload = 0;

    if(lastDeviceInfoUpload != 0 &&
    millis() -
    lastDeviceInfoUpload <
    60000)
    {
        return;
    }

    lastDeviceInfoUpload =
        millis();

    String root =
        "/devices/" +
        String(DEVICE_ID);

    FirebaseJson infoJson;

    infoJson.set(
        "deviceName",
        DEVICE_NAME);

    infoJson.set(
        "firmwareVersion",
        FIRMWARE_VERSION);

    infoJson.set(
    "online",
    systemState.firebaseConnected);

    infoJson.set(
        "lastSeen",
        millis());

    writeJson(
        root + "/deviceInfo",
        infoJson);
}