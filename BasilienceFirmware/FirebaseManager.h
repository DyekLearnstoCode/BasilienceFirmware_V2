#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>

class FirebaseManager
{
public:

    void begin();

    void update();

private:

    void initializeDatabase();

    FirebaseData fbdo;

    FirebaseAuth auth;

    FirebaseConfig config;

    FirebaseJson json;

    unsigned long lastSettingsRead = 0;

    void readSettings();

    void syncRTC();

    void readCommands();

    bool hasActiveOperation() const;

    bool isDuplicateRequest(
        uint16_t requestId) const;

    bool validateOperationRequest(
        OperationType operation,
        OperationAction action,
        String& reason);

    void createOperationRequest(
        uint16_t requestId,
        OperationType operation,
        OperationAction action,
        RequestSource source);

    void rejectOperationRequest(
        uint16_t requestId,
        const char* reason);

    bool writeCurrentOperation();
    bool archiveCurrentOperation();
    
    bool writeJson(
    const String& path,
    FirebaseJson& json);

    RequestState lastPublishedOperationState =
        RequestState::IDLE;

    void writeSensors();

    void writeStatus();

    void writeTelemetry();

    void writeDeviceInfo();

    void writeAlerts();

    void writeActuators();
    void resetCurrentOperation();
    void updateOperationState(
    RequestState newState);

    

    ActuatorTelemetry lastActuatorState;
};

#endif