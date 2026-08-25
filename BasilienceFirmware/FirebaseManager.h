#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>

#include "Types.h"

class FirebaseManager
{
public:

    // Client-side view of transport health, used to bound cloud-side retry
    // behavior without ever gating local automation/safety/actuator control.
    // HEALTHY: normal cadence. DEGRADED: essential ops only (heartbeat,
    // actuator sync, command reads), low-priority reads/writes deferred.
    // COOLDOWN: no Firebase network calls at all until the backoff window
    // expires. RECOVERING: one bounded reconnect attempt, then HEALTHY or
    // back to COOLDOWN with increased backoff.
    enum class FirebaseHealthState
    {
        HEALTHY,
        DEGRADED,
        COOLDOWN,
        RECOVERING
    };

    void begin();

    void loadPersistedSettings();

    void update();

    void syncMockSensors();

    void syncSensorTest();

    // Called by WiFiManager's local-AP provisioning HTTP server when a
    // device secret is injected during an explicit provisioning/migration
    // session (see WiFiManager::setupAPServer()'s /secure-provision route).
    // Persists it to the device_auth NVS namespace only - never echoed back,
    // never logged, never written to RTDB. Takes effect on the next boot's
    // bootstrap attempt (does not itself trigger an immediate re-auth).
    void saveDeviceSecretFromProvisioning(const String& secret);

    // Human-readable "AA:BB:CC:DD:EE:FF" form of the hardware STA MAC, read
    // via esp_read_mac(ESP_MAC_WIFI_STA) - valid even before WiFi.mode() has
    // ever been called (e.g. first-boot AP-only provisioning, where
    // WiFi.macAddress() is known to read back all-zero). Returns "" if the
    // hardware MAC could not be resolved; never returns an all-zero MAC.
    String getFormattedMacAddress();

    // Normalized "AABBCCDDEEFF" (uppercase, no separators) form of the same
    // hardware STA MAC - unchanged output format from before this fix, only
    // the source underneath changed. Returns "" if the hardware MAC could
    // not be resolved; never returns an all-zero MAC. This is the form used
    // for the Firebase /provisioning/{mac}/deviceToken lookup.
    String getMacAddress();

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

    // device_auth NVS namespace contents, loaded once per boot by
    // loadDeviceAuthCredentials(). Never logged, never written to RTDB.
    String deviceAuthSecret;
    String deviceAuthRefreshToken;

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

    // Transport-level connection health, separate from the presence/heartbeat
    // failure counter above - see recordFirebaseResult()'s comment for why
    // the two are intentionally never merged.
    FirebaseHealthState firebaseHealth = FirebaseHealthState::HEALTHY;
    uint8_t transportFailureStreak = 0;
    unsigned long cooldownStartedAt = 0;
    unsigned long cooldownDurationMs = 0;

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

    // Reads the 6 raw STA MAC bytes via esp_read_mac(ESP_MAC_WIFI_STA),
    // which works regardless of WiFi.mode()/WiFi.begin() state - unlike
    // WiFi.macAddress(), which reads back all-zero whenever the STA
    // interface has never been brought up (confirmed root cause: a device
    // with no saved credentials boots straight into WiFi.mode(WIFI_AP)
    // provisioning, so the STA netif is never started and
    // WiFi.macAddress() has nothing to report). Logs "[IDENTITY] ERROR:
    // Unable to resolve hardware Wi-Fi MAC" and returns false (out left
    // untouched) if the read fails or comes back all-zero.
    bool readHardwareStaMac(uint8_t out[6]);

    //==================================================
    // Secure Device Auth (bootstrap + refresh-token identity)
    //==================================================

    // Orchestrates the boot-time auth flow: refresh-token restore, then
    // secret-based bootstrap, in that order. Returns false if neither
    // credential is present/works - callers must not treat that as fatal,
    // only as "secure identity unavailable this boot."
    bool trySecureAuthentication();

    // Restores a previously-established identity from a persisted refresh
    // token (Firebase.setCustomToken() auto-detects a non-JWT-shaped string
    // as a refresh token and performs a refresh-grant sign-in directly - see
    // FirebaseCore.cpp's own signer logic - no bootstrap call needed).
    bool restoreFromRefreshToken(const String& refreshToken);

    // Calls the HTTPS bootstrap endpoint with {mac, deviceSecret}, exchanges
    // the returned custom token for a full Firebase identity, and persists
    // the resulting refresh token. The secret is never logged and is only
    // ever held in local variables that go out of scope when this returns.
    bool bootstrapSecureAuth(const String& secret);

    void loadDeviceAuthCredentials();
    void saveRefreshToken(const String& token);

    //==================================================
    // Firebase transport health (timeout cascade / backoff / recovery)
    //==================================================

    // Called after every real Firebase.RTDB.* call (directly, or via
    // writeJson()/updateJson()) with that call's outcome. On failure, reads
    // fbdo.errorReason() once to classify it as transport-level or
    // application-level (see isTransportFailureReason()) - only transport
    // failures move firebaseHealth. A success always resets the streak and
    // returns health to HEALTHY.
    void recordFirebaseResult(bool success);

    // True for transport/network-style failures only (timeouts, connection
    // refused/lost/reset, SSL failures, "no http server", 5xx gateway
    // errors) - grounded in the exact strings FB_Const.h's errorReason()
    // can return, not guessed. False for application-level outcomes such as
    // permission denied, a missing optional path, malformed data, or a
    // rejected operation command, none of which indicate a broken
    // connection.
    bool isTransportFailureReason(const String& reason) const;

    // Enters (or re-enters with escalated backoff) COOLDOWN. Backoff starts
    // at 15s, doubles on each subsequent cooldown entry, capped at 60s.
    void enterFirebaseCooldown();

    // One bounded, controlled reconnect attempt: re-runs the same
    // trySecureAuthentication()/legacy-fallback flow begin() already uses
    // (auth state and NVS credentials are untouched - only the transport
    // session is torn down and re-established), then polls Firebase.ready()
    // with the same 10s bound used everywhere else in this class. Returns
    // true and moves to HEALTHY on success; returns false and re-enters
    // COOLDOWN with escalated backoff on failure.
    bool attemptFirebaseRecovery();

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

    //==================================================
    // Offline notification pipeline (recipients / harvest schedule / replay)
    //==================================================

    // Reads /devices/{deviceId}/smsRecipients and hands the result to
    // smsRecipientCache.applySnapshot(). A failed read returns without
    // calling applySnapshot() at all, so the last known-good cache is never
    // erased by a transient RTDB error.
    void readSmsRecipients();

    // Reads /devices/{deviceId}/harvestSchedule and hands the result to
    // harvestScheduleCache.applySnapshot(). Same failed-read contract as
    // readSmsRecipients().
    // Applies one validated min/max target-range pair from the settings
    // snapshot. Missing keys keep the current value; an inverted or
    // out-of-bounds pair is rejected wholesale.
    void applyTargetRange(const char* minKey, const char* maxKey,
                          float& minTarget, float& maxTarget,
                          float physicalMin, float physicalMax);

    void readHarvestSchedule();

    // Advances cloud replay of the notificationManager's durable queue by
    // exactly one step (one (re)submission or one ack poll) per call - never
    // more than one event in flight at a time.
    void replayQueuedNotification();

    // Same one-event-in-flight, one-step-per-call shape as
    // replayQueuedNotification() above, for FoggingEventQueue instead of
    // NotificationManager. Writes to the append-only
    // devices/{deviceId}/foggingEventQueue path, never to actuatorStatus -
    // see the task report for why those must stay separate.
    void replayQueuedFoggingEvent();
};

#endif
