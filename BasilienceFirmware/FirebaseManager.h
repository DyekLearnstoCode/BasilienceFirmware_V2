#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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
    // Timestamp of the last time any low-priority cloud-maintenance job
    // (telemetry, device info, diagnostic sensors, SMS recipients, harvest
    // schedule, notification/fogging ACK replay) actually ran - shared
    // across all of them by design (the smallest mechanism that still
    // guarantees SOME of them get serviced periodically). Used only to give
    // manual-interaction deferral a bounded grace window instead of an
    // unbounded suppression - see shouldDeferLowPriorityForManualInteraction().
    unsigned long lastLowPriorityCloudJobAt = 0;
    // Timestamp of the last fresh manual actuator or operation command
    // firmware actually observed (a new write under /commands/{actuator} or
    // a new, non-duplicate /commands/current request) - set in
    // consumeActuatorCommandSnapshot() and readCommands() respectively. The
    // tighter, event-driven half of the manual-interaction defer signal: see
    // update()'s deferLowPriorityForManualInteraction.
    unsigned long lastManualCommandActivityAt = 0;

public:
    // Read-only accessor for ActuatorManager::update()'s Manual Mode
    // inactivity-expiry check, which must run every loop() tick regardless
    // of Firebase connectivity - see that check's own comment for why it
    // cannot live inside FirebaseManager::update() (skipped during
    // provisioning / before Wi-Fi connects).
    unsigned long lastManualActivityMillis() const { return lastManualCommandActivityAt; }

private:
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
    bool refillLevelCmSettingsInitialized = false;
    bool refillLevelCmRejectionLogged = false;
    float lastRejectedRefillStartCm = NAN;
    float lastRejectedRefillStopCm = NAN;
    bool waterLevelCalibrationInitialized = false;
    bool waterLevelCalibrationRejectionLogged = false;
    float lastRejectedWaterLevelEmptyCm = NAN;
    float lastRejectedWaterLevelFullCm = NAN;
    // sensorToBottomCm - the authoritative calibration field (see Types.h's
    // matching comment and the automation resilience pass report), tracked
    // independently of the legacy waterLevelCalibrationInitialized above.
    bool sensorToBottomCalibrationInitialized = false;
    bool highAirTempSettingsInitialized = false;
    bool highAirTempRejectionLogged = false;
    float lastRejectedHighAirTemp = NAN;
    bool blowerSpeedRejectionLogged = false;
    float lastRejectedBlowerSpeed = NAN;

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
    void runOneOptionalFirebaseJob(bool sensorTestMode, bool deferLowPriorityJobs,
                                   bool deferLowPriorityForManualInteraction);
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
    bool applyAutomationTestModeCommand(FirebaseJson& snapshot);
    void setAutomationTestMode(AutomationTestSubsystem subsystem,
                               bool publishAcknowledgement = true);
    void consumeActuatorCommandSnapshot(FirebaseJson& snapshot, bool dispatchCommands);
    void loadActuatorCommandTimestamps();
    void saveActuatorCommandTimestamp(Actuator actuator, uint64_t timestamp);
    void readMockSensors();
    void readSensorTestCommand();
    void setSensorTestEnabled(bool enabled, bool publishAcknowledgement = true);

    // Developer testing override that bypasses ONLY the automatic
    // water-level/refill gate (AutomationManager's handleNormal(),
    // handleRefilling()). Mirrors
    // readSensorTestCommand()/setSensorTestEnabled()'s shape exactly - see
    // those for the established command/status RTDB pattern this follows.
    void readWaterLevelOverrideCommand();
    void setIgnoreWaterLevelAutomation(bool enabled, bool publishAcknowledgement = true);

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

    // Non-blocking auth state machine (critical verification report,
    // Priority 1). Both begin() (boot) and beginFirebaseRecovery()/
    // pollFirebaseRecovery() (runtime, reachable at any time - including
    // mid-dose) drive the SAME
    // underlying credential order (refresh token -> device-secret bootstrap
    // -> legacy anonymous fallback) through this one implementation, so
    // there is exactly one place that decides how a device authenticates.
    // See the .cpp for the full per-state rationale and the one remaining
    // bounded exception (the device-secret bootstrap's HTTPS POST).
    enum class FirebaseAuthPhase : uint8_t
    {
        IDLE,
        TRY_REFRESH_TOKEN,
        WAIT_REFRESH_READY,
        TRY_DEVICE_SECRET,
        // Waiting on the background bootstrap-HTTP task (see
        // bootstrapHttpTaskFn()) - distinct from WAIT_BOOTSTRAP_READY below,
        // which waits on Firebase.ready() itself once the token has already
        // been handed to the library.
        WAIT_BOOTSTRAP_HTTP,
        WAIT_BOOTSTRAP_READY,
        TRY_LEGACY_SIGNUP,
        WAIT_LEGACY_READY,
        SUCCESS,
        FAILED
    };

    FirebaseAuthPhase authPhase = FirebaseAuthPhase::IDLE;
    unsigned long authPhaseStartedAt = 0;
    // True only while the phase currently in SUCCESS/FAILED was reached via
    // TRY_LEGACY_SIGNUP - lets callers log "legacy" vs. "secure device
    // identity" accurately without re-deriving it from authPhase after it
    // may already have been reset to IDLE for the next attempt.
    bool authSucceededViaLegacy = false;

    // Resets the state machine to the first reachable phase for a fresh
    // attempt, based on which credentials are currently persisted. Does not
    // block and does not itself perform any network call.
    void startAuthAttempt();

    // Advances the state machine by exactly one bounded step and returns
    // immediately once that step is done - never loops, never delays. The
    // one exception is the TRY_DEVICE_SECRET step's HTTPS POST, a single
    // bounded (shortened-timeout) blocking call - see its own comment in the
    // .cpp for why a full non-blocking HTTP client is out of scope for this
    // pass. Returns true once the attempt has concluded (authPhase is
    // SUCCESS or FAILED this call); false while still in flight.
    bool pollAuthStateMachine();

    // Orchestrates the boot-time auth flow synchronously: at this point in
    // the boot sequence (called only from begin(), before any growth cycle
    // or dosing can possibly be in progress - AutomationManager always
    // starts in SENSOR_STABILIZATION) a bounded local poll loop is
    // acceptable, so this is the one place this class still blocks its
    // caller - every RUNTIME recovery attempt (beginFirebaseRecovery()/
    // pollFirebaseRecovery(), reachable at any time, including mid-dose)
    // drives the identical underlying state machine non-blockingly instead,
    // one step per FirebaseManager::update() call. Returns false if no
    // credential worked.
    bool trySecureAuthentication();

    // Restores a previously-established identity from a persisted refresh
    // token (Firebase.setCustomToken() auto-detects a non-JWT-shaped string
    // as a refresh token and performs a refresh-grant sign-in directly - see
    // FirebaseCore.cpp's own signer logic - no bootstrap call needed). Kicks
    // off Firebase.begin() and returns immediately; WAIT_REFRESH_READY polls
    // Firebase.ready() non-blockingly for the actual result.
    bool restoreFromRefreshToken(const String& refreshToken);

    // Resolves the device MAC (fast, synchronous, no network) and hands the
    // {mac, secret} pair off to a dedicated background FreeRTOS task (see
    // bootstrapHttpTaskFn()) that performs the actual HTTPS POST - this
    // function itself returns immediately, never blocking the caller.
    // Returns false only if the MAC isn't resolvable yet or the background
    // task could not be created (e.g. out of heap); WAIT_BOOTSTRAP_HTTP polls
    // for the task's result non-blockingly. The secret is copied into
    // bootstrapHttpSecret for the task's own use and cleared the moment the
    // task is done with it; never logged, never echoed anywhere.
    bool bootstrapSecureAuth(const String& secret);

    // Background bootstrap-HTTP task infrastructure. The ONLY genuinely
    // blocking primitive left in the auth path (HTTPClient::POST() - the
    // stock Arduino HTTPClient has no non-blocking POST) now runs on this
    // dedicated, short-lived task instead of inline in
    // FirebaseManager::update(), so the main loop task is never blocked by
    // it, even for the shortened 5s timeout. Thread-safety boundary,
    // deliberately narrow: the task touches ONLY its own local
    // NetworkClientSecure/HTTPClient/FirebaseJson objects and the plain
    // result fields below - it NEVER calls into the Firebase library (no
    // Firebase.*, no fbdo, no config/auth access) and never touches any
    // AutomationManager/ActuatorManager/SystemState field. Every actual
    // Firebase library call (Firebase.setCustomToken()/Firebase.begin()/
    // Firebase.ready()) still happens only in the main loop task, exactly as
    // before - so no Firebase library call is ever made from more than one
    // task. The task is pinned to the same core the main loop task is
    // running on (captured at kickoff via xPortGetCoreID(), not assumed) so
    // the two are always time-sliced, never truly concurrent, which removes
    // any cross-core cache-visibility question for the plain bool/String
    // handoff below - xSemaphoreGive()/xSemaphoreTake() still provide the
    // actual synchronization guarantee regardless.
    SemaphoreHandle_t bootstrapHttpDoneSemaphore = nullptr;
    // Set true (main task) the instant the task is created; set false (main
    // task only, never by the task itself) once WAIT_BOOTSTRAP_HTTP has
    // consumed its result - guards against ever starting a second task while
    // one is still in flight. Not touched by the task.
    bool bootstrapHttpTaskActive = false;
    // Written only by the main task before creating the background task;
    // read only by the task itself (which makes its own local copies before
    // any Firebase/library call could plausibly reenter this class).
    String bootstrapHttpMac;
    String bootstrapHttpSecret;
    // Written only by the task, only before it calls xSemaphoreGive(); read
    // only by the main task, only after xSemaphoreTake() succeeds - a strict
    // single-writer-then-signal, single-reader-after-signal handoff.
    bool bootstrapHttpResultSuccess = false;
    String bootstrapHttpResultToken;
    String bootstrapHttpResultDeviceId;

    // Task entry point. Static (FreeRTOS task functions cannot be non-static
    // member functions); `arg` is the owning FirebaseManager instance,
    // passed explicitly at creation. Self-deletes (vTaskDelete(nullptr)) as
    // its last action after giving bootstrapHttpDoneSemaphore.
    static void bootstrapHttpTaskFn(void* arg);

    // Persists a rotated/new refresh token once WAIT_REFRESH_READY/
    // WAIT_BOOTSTRAP_READY confirms Firebase.ready() - split out of
    // restoreFromRefreshToken()/bootstrapSecureAuth() themselves since that
    // confirmation no longer happens synchronously inside either of them.
    void onRefreshTokenAuthSucceeded();
    void onBootstrapAuthSucceeded();

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

    // Non-blocking recovery (critical verification report, Priority 1).
    // beginFirebaseRecovery() tears down the possibly-stuck transport
    // session and kicks off the SAME auth state machine begin() uses (auth
    // state and NVS credentials are untouched), then returns immediately -
    // called once, from update(), the instant COOLDOWN's backoff window
    // elapses. pollFirebaseRecovery() advances that attempt by one bounded
    // step per subsequent update() call while firebaseHealth stays
    // RECOVERING, and moves to HEALTHY on success or back to COOLDOWN with
    // escalated backoff on failure once the state machine concludes.
    void beginFirebaseRecovery();
    void pollFirebaseRecovery();

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
