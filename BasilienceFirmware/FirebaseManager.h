#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>

#include "Types.h"

class FirebaseManager
{
public:

    void begin();

    void update();

private:

    //==================================================
    // Firebase
    //==================================================

    
    FirebaseData fbdo;

    FirebaseAuth auth;

    FirebaseConfig config;

    FirebaseJson json;

    //==================================================
    // Runtime
    //==================================================

    unsigned long lastSettingsRead = 0;

    RequestState lastPublishedOperationState =
        RequestState::IDLE;

    ActuatorTelemetry lastActuatorState;

    //==================================================
    // Initialization
    //==================================================

    void initializeDatabase();

    bool writeJson(
        const String& path,
        FirebaseJson& json);
    
    String deviceRoot() const;

    //==================================================
    // Synchronization
    //==================================================

    void readSettings();

    void syncRTC();

    void readCommands();

    //==================================================
    // Operation Protocol
    //==================================================

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

    void resetCurrentOperation();

    void updateOperationState(
    RequestState state,
    const char* reason = nullptr);

    //==================================================
    // Uploads
    //==================================================

    void writeSensors();

    void writeStatus();

    void writeTelemetry();

    void writeAlerts();

    void writeActuators();

    void writeDeviceInfo();
};

#endif