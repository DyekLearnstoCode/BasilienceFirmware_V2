#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>

#include "Types.h"

class FirebaseManager
{
public:

    void begin();

    void loadPersistedSettings();

    void update();

    void syncMockSensors();

    void syncSensorTest();

private:

    //==================================================
    // Firebase
    //==================================================

    
    FirebaseData fbdo;

    FirebaseAuth auth;

    FirebaseConfig config;

    FirebaseJson json;
    Preferences preferences;
    String deviceId;

    //==================================================
    // Runtime
    //==================================================

    unsigned long lastSettingsRead = 0;
    unsigned long lastSensorUploadAttempt = 0;
    uint8_t optionalFirebaseJobCursor = 0;
    uint16_t lastProtectedAutomaticRequestId = 0;
    uint8_t automaticControlPassesRemaining = 0;

    bool wasFirebaseConnected = false;
    bool suspendedForProvisioning = false;
    bool hasPublishedHeartbeat = false;
    bool heartbeatResumePending = false;
    unsigned long lastSuccessfulSensorUpload = 0;
    unsigned long lastHeartbeatSuccessLog = 0;
    uint32_t consecutiveSensorUploadFailures = 0;
    String lastSensorUploadFailureReason;
    bool refillSettingsInitialized = false;
    bool refillRejectionLogged = false;
    float lastRejectedRefillStart = NAN;
    float lastRejectedRefillStop = NAN;
    bool highAirTempSettingsInitialized = false;
    bool highAirTempRejectionLogged = false;
    float lastRejectedHighAirTemp = NAN;

    unsigned long lastAlertFullUpload = 0;
    unsigned long lastActuatorFullUpload = 0;
    bool alertCacheInitialized = false;
    bool sensorFaultPublicationInitialized = false;
    bool actuatorCacheInitialized = false;
    AlertState lastPublishedAlerts;
    ActuatorStatus lastPublishedActuators[ACTUATOR_COUNT];
    uint64_t lastActuatorCommandTimestamps[ACTUATOR_COUNT] = { 0 };
    bool actuatorCommandsPrimed = false;
    bool sensorTestCommandBlockedUntilFalse = false;

    RequestState lastPublishedOperationState =
        RequestState::IDLE;
    bool automaticTerminalSyncPending = false;
    bool automaticTerminalSensorUploaded = false;
    bool operationPublishFailureLogged = false;
    uint16_t automaticTerminalRequestId = 0;
    SensorData automaticTerminalSensors;
    uint16_t lastDeferredCommandRequestId = 0;
    unsigned long automaticTerminalSnapshotUploadedAt = 0;
    bool automaticTerminalSensorUploadFailureLogged = false;

    //==================================================
    // Initialization
    //==================================================

    void initializeDatabase();

    bool writeJson(
        const String& path,
        FirebaseJson& json);

    bool updateJson(
        const String& path,
        FirebaseJson& json);

    void logFirebaseDuration(const char* operation, unsigned long durationMs) const;
    bool isSensorUploadDue() const;
    bool shouldDeferOptionalJobsForControlResponse();
    void runOneOptionalFirebaseJob(bool sensorTestMode, bool deferLowPriorityJobs);
    void enforceSensorTestTimeout();
    void captureAutomaticTerminalSnapshot();
    void syncOperationState();
    
    String deviceRoot() const;
    void loadDeviceId();
    bool saveDeviceId(const String& id);
    const String& getDeviceId() const;

    //==================================================
    // Synchronization
    //==================================================

    void readSettings();

    void persistSettings();

    void syncRTC();

    void readCommands();
    void readActuatorCommands();
    void primeActuatorCommands();
    void consumeActuatorCommandSnapshot(FirebaseJson& snapshot, bool dispatchCommands);
    void loadActuatorCommandTimestamps();
    void saveActuatorCommandTimestamp(Actuator actuator, uint64_t timestamp);
    void readMockSensors();
    void readSensorTestCommand();
    void setSensorTestEnabled(bool enabled, bool publishAcknowledgement = true);

    void provisionDevice();
    String getMacAddress();

    //==================================================
    // Operation Protocol
    //==================================================

    bool hasActiveOperation() const;

    bool isOperationLifecycleOwned() const;

    bool isDuplicateRequest(
        uint16_t requestId) const;

    bool validateOperationRequest(
        OperationType operation,
        OperationAction action,
        String& reason);

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

    bool writeSensors(bool force = false, const SensorData* snapshot = nullptr);

    void writeStatus();

    void writeTelemetry();

    bool writeAlerts();

    void writeActuators();

    void writeDeviceInfo();

    void writeDiagnosticSensors();
};

#endif
