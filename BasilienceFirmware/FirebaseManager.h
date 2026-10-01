#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "Types.h"

// This firmware must be built against the Basilience-patched copy of the
// Firebase Arduino Client Library that lives in this repository
// (libraries/Firebase_Arduino_Client_Library_for_ESP8266_and_ESP32, see
// BASILIENCE_PATCH.md in that folder). Upstream 4.4.17 lets a single connection
// attempt block loop() for 30s (TCP connect) plus 60s (TLS handshake); the patch
// replaces those two defaults with the values below and defines these macros.
// If either macro is missing, the compiler picked up an unpatched copy (for
// example the arduino-cli user-library folder instead of this repo's), and
// building anyway would silently bring the long stalls back.
#if !defined(BASILIENCE_FIREBASE_TCP_CONNECT_TIMEOUT_MS) || \
    !defined(BASILIENCE_FIREBASE_TLS_HANDSHAKE_TIMEOUT_MS)
#error "Unpatched Firebase Arduino Client Library. Build with the copy in <repo>/libraries (see BASILIENCE_PATCH.md), or apply that patch to the library you are building against."
#endif

// REVISED 2026-10-01: the former build-time ceiling (connect + handshake <=
// 5000 ms) is gone. It forced a 2000 ms TCP connect, which fails every time
// the first SYN is lost (lwIP retransmits it after 3000 ms), and that - not the
// cloud - was what kept Firebase down. Current values: connect 4000 ms,
// handshake 5000 ms; see BASILIENCE_PATCH.md for the derivation.

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

    // Arms cloud bring-up and returns immediately - it performs no network
    // I/O and never waits on Wi-Fi, DNS, TLS or authentication. Called once,
    // from loop(), the first time Wi-Fi is connected. The whole first
    // connection (network preflight -> authentication -> post-auth database
    // initialization) is then advanced a bounded step at a time by update(),
    // so local sensing, safety, automation and actuator control keep running
    // on every loop() iteration while the cloud is unreachable. The cloud is
    // "ready" only once update() has completed every step below, not merely
    // because Wi-Fi is connected.
    void begin();

    // loop() does not call update() while the setup AP owns the radio, so
    // update()'s own Wi-Fi-lost handling never runs for that outage. This is
    // called each provisioning-mode iteration to do it instead: withdraw the
    // permission to call the Firebase library (so the first cloud call after
    // the network returns is preceded by a fresh preflight), start the outage
    // clock that decides the 30s command re-baseline, and discard an unfinished
    // first-connection attempt so the next one starts from a fresh preflight.
    // Cheap and idempotent.
    void noteProvisioningActive();

    void loadPersistedSettings();

    void update();

    // Serial Diagnostics / Observability pass: set by loadPersistedSettings()
    // before Wi-Fi/Firebase ever start, purely for the boot summary line -
    // read-only elsewhere, never consulted by any control/settings logic.
    bool settingsRestoredFromNvs = false;

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
    // CONFIRMED BUG FIX (effective sensor data vs Firebase sensor sync):
    // tracks the mock/physical edge across writeSensors() calls - see its
    // own comment where this is read.
    bool lastPublishedSourceWasMock = false;
    uint8_t optionalFirebaseJobCursor = 0;
    // Timestamp of the last time any low-priority cloud-maintenance job
    // (telemetry, device info, diagnostic sensors, SMS recipients, harvest
    // schedule, notification/fogging ACK replay) actually ran - shared
    // across all of them by design (the smallest mechanism that still
    // guarantees SOME of them get serviced periodically). Used only to give
    // manual-interaction deferral a bounded grace window instead of an
    // unbounded suppression - see
    // shouldDeferLowPriorityForManualInteraction().
    unsigned long lastLowPriorityCloudJobAt = 0;
    // Timestamp of the last fresh manual actuator or operation command
    // firmware actually observed (a new write under /commands/{actuator} or
    // a new, non-duplicate /commands/current request) - set in
    // consumeActuatorCommandSnapshot() and readCommands() respectively. The
    // tighter, event-driven half of the manual-interaction defer signal:
    // see update()'s deferLowPriorityForManualInteraction.
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

    // Serial Diagnostics / Observability pass (section 5): edge-tracking only
    // for the [NET] WiFi UP/LOST transition lines - never read by any
    // control/reconnect logic, which already uses wifiManager.isConnected()
    // directly.
    bool wasWifiConnectedForLog = true;
    bool suspendedForProvisioning = false;
    // Set when commands/startProvisioning was observed true but deleteNode()
    // could not confirm the node was actually removed. While true, the next
    // successful commands poll re-checks the raw snapshot instead of trusting
    // the earlier ambiguous result: still-true retries the delete, absent is
    // treated as the earlier delete having taken effect after all. Either way
    // manual provisioning is only entered once the command is confirmed gone,
    // so a delete that failed to ack can never replay AP mode after a
    // reconnect or reboot. See beginManualProvisioningAfterCommandConsumed().
    bool provisioningCommandPendingDelete = false;
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

    // Second half of the /commands/current dedupe key (see
    // isDuplicateRequest()): two senders can legitimately reuse the same
    // requestId, but not the same requestId AND requestTimestamp.
    uint32_t lastProcessedRequestTimestamp = 0;

    // /commands/current is a persistent document, not an event queue, and
    // the firmware never deletes it. The first successful read after boot,
    // and the first read after a cloud outage of
    // COMMAND_REBASELINE_OUTAGE_MS or more, is therefore recorded as
    // already-handled instead of executed - otherwise a reboot (RAM
    // watermark lost) or a command written while the device was
    // unreachable would run, possibly hours late, on reconnect.
    bool currentCommandBaselined = false;

    // millis() of the first update() tick of the current cloud outage, or 0
    // while the cloud is reachable. See noteCloudUnavailable()/
    // noteCloudAvailable().
    unsigned long cloudUnavailableSince = 0;
    void noteCloudUnavailable();
    void noteCloudAvailable();

    // STA self-heal escalation (see checkStaRecovery()). Separate from
    // cloudUnavailableSince above on purpose: this clock only counts time the
    // cloud has been not READY WHILE Wi-Fi is associated, and restarts from
    // zero on any Wi-Fi loss (including the forced reconnect itself), so a
    // real Wi-Fi outage never counts toward it and each forced reconnect
    // needs its own fresh 30s of associated outage. lastStaRecoveryAt is the
    // absolute 60s cooldown and is never reset by recovery. 0 = not running /
    // never forced.
    unsigned long staRecoveryOutageSince = 0;
    unsigned long lastStaRecoveryAt = 0;
    bool staRecoverySuppressedLogged = false;
    bool checkStaRecovery();
    void resetStaRecoveryStage();

    // True when the latest background diagnostics of the current outage found
    // DNS not answering over the Wi-Fi link. It is the only evidence that lets
    // checkStaRecovery() bounce the radio; cleared when the cloud is READY or
    // Wi-Fi is lost. Diagnostics never gate any Firebase library call.
    bool diagDnsFailed = false;


    // Developer/test command nodes (commands/automationTestMode,
    // commands/ignoreWaterLevelAutomation, commands/mockSensors) are level
    // flags that stay in RTDB. They must never survive a device reboot, so
    // the CLEAR_DEV_FLAGS startup step writes enabled=false to all three, and
    // cloud startup cannot reach COMPLETE until that has succeeded; until
    // every write has succeeded, their readers ignore an enabled=true - a
    // failed clear can never re-arm a stale flag. Retried from update().
    //
    // Also cleared to false by noteCloudAvailable() after a reconnect from
    // an outage of COMMAND_REBASELINE_OUTAGE_MS or more: a value written to
    // one of these nodes while the device was offline is otherwise honoured
    // the instant the cloud comes back, since this flag was already true
    // from boot. This forces the same boot-time re-clear to run again first.
    bool devCommandsCleared = false;
    unsigned long lastDevCommandClearAttemptAt = 0;
    void clearDevCommandsAtBoot();

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
    // Non-blocking cloud startup (first Firebase connection)
    //==================================================

    // WAITING         - the first connection (or a retry) is waiting only for
    //                   its backoff to elapse. Nothing has to pass first: the
    //                   Firebase library does its own DNS/TCP/TLS.
    // AUTHENTICATING  - the existing auth state machine (startAuthAttempt() /
    //                   pollAuthStateMachine()), one bounded step per update().
    // INIT_STEPS      - post-auth database initialization, one small step per
    //                   update() (see CloudInitStep).
    // COMPLETE        - normal cloud sync (the rest of update()).
    //
    // A separate DNS/TCP/TLS probe exists (see "Network diagnostics" below) but
    // it only produces log evidence after a real Firebase failure; it never
    // allows or blocks a library call.

    enum class CloudStartupPhase : uint8_t
    {
        NOT_STARTED,
        WAITING,
        AUTHENTICATING,
        INIT_STEPS,
        COMPLETE
    };

    // Everything begin() used to do synchronously after authentication,
    // split so each update() tick performs at most one step.
    enum class CloudInitStep : uint8_t
    {
        RESOLVE_DEVICE_ID,
        PRIME_COMMANDS_EARLY,
        SEED_STATUS,
        SEED_SETTINGS,
        SEED_COMMANDS_CURRENT,
        SEED_OPERATIONS_CURRENT,
        CLEAR_SENSOR_TEST,
        CLEAR_DEV_FLAGS,
        PRIME_COMMANDS_FINAL,
        READ_SETTINGS,
        FINISH
    };

    CloudStartupPhase cloudStartupPhase = CloudStartupPhase::NOT_STARTED;
    CloudInitStep cloudInitStep = CloudInitStep::RESOLVE_DEVICE_ID;

    // 0 means "attempt as soon as possible". millis() of the earliest moment
    // the next startup attempt may start.
    unsigned long cloudStartupNextAttemptAt = 0;
    unsigned long cloudStartupBackoffMs = 0;
    uint16_t cloudStartupAttempt = 0;
    unsigned long cloudInitRetryAt = 0;
    unsigned long cloudInitRetryMs = 0;
    bool sensorTestBootClearDone = false;
    // Set by readSettings() so startup can tell a real read from a failed one
    // without changing readSettings()' many early returns.
    bool lastSettingsReadOk = false;


    // One blocking library transaction per update() tick in normal operation.
    // Every RTDB call reports its result to recordFirebaseResult(), which counts
    // it here; update() stops handing out further cloud work once one has run
    // (or the transport went into cooldown). Before this, a single healthy tick could run
    // command reads, an actuator write, an alert write, a heartbeat and an
    // optional job back to back, so on a slow link their waits stacked into one
    // multi-second stall. Nothing is dropped: whatever did not run is still due
    // on the next tick, and every function keeps its own cadence gate.
    uint8_t cloudCallsThisTick = 0;
    bool cloudTickBudgetSpent() const { return cloudCallsThisTick >= 1; }
    // An alert transition writes /alerts and then owes /sensors one forced
    // refresh. That second call now runs at the head of the next tick instead
    // of in the same one.
    bool alertHeartbeatPending = false;

    void advanceCloudStartupConnect();
    void advanceCloudInit();
    bool runCloudInitStep();
    // Startup only: an authentication attempt failed. Returns to WAITING.
    void scheduleCloudStartupRetry(bool libraryContacted, const char* reason);
    // Escalating gate on when the next startup attempt may start. Changes no phase.
    void bumpCloudRetryBackoff(bool libraryContacted, const char* reason);
    void abortCloudStartupAttempt();
    void completeCloudStartup();

    // Wi-Fi is gone (or the setup AP owns the radio): forget any backoff (the
    // outage was the network's) and restart an unfinished first connection.
    void noteNetworkLost();
    // Firebase.ready() was false while the cloud should have been usable (the
    // library's token refresh failed): counts as a full failed attempt and goes
    // straight to COOLDOWN.
    void noteLibraryNotReady();
    // Starts a background DNS/TCP/TLS probe for the log. Gates nothing.
    void launchDiagnostics(const char* reason);
    // Collects a finished probe. Gates nothing.
    void pollDiagnostics();


    // Connectivity diagnostics ([NET] lines). Each prints once per event, never
    // per loop tick.
    void logWifiConnectedDiagnostics();
    void logSystemTimeDiagnostics();

    // Network diagnostics (background task; the "preflight" names are kept from
    // when this gated Firebase, it is evidence-only now). Same thread-safety shape as the
    // bootstrap HTTP task below: the task touches ONLY its own local network
    // objects and the plain result fields here - never the Firebase library,
    // never SystemState or any other manager - writes them once, then gives
    // preflightDoneSemaphore. The main task reads them only after taking it.
    static constexpr uint8_t PREFLIGHT_HOST_COUNT = 4;
    SemaphoreHandle_t preflightDoneSemaphore = nullptr;
    // True from task creation until the main task has observed the task's
    // semaphore give. Guards against ever starting a second preflight task
    // while an abandoned one could still be writing the result fields.
    bool preflightTaskActive = false;
    // True while a diagnostics run is waiting on a result.
    bool preflightRunning = false;
    unsigned long preflightStartedAt = 0;
    uint8_t preflightHostCount = 0;
    // Written by the main task before creating the task; read by the task.
    String preflightHosts[PREFLIGHT_HOST_COUNT];
    // Written by the task only, before it signals done.
    int8_t preflightDns[PREFLIGHT_HOST_COUNT];       // 1 ok, 0 failed, -1 not attempted
    char preflightDnsIp[PREFLIGHT_HOST_COUNT][40];
    // Real TLS handshakes, no data sent: [0] = the auth host (preflightHosts[0]),
    // [1] = the database host (preflightHosts[2]). The second is only tried
    // once the first has succeeded.
    static constexpr uint8_t PREFLIGHT_TLS_TARGETS = 2;
    int8_t preflightTls[PREFLIGHT_TLS_TARGETS];      // 1 ok, 0 failed, -1 not attempted
    int preflightTlsError[PREFLIGHT_TLS_TARGETS];
    uint32_t preflightTlsMs[PREFLIGHT_TLS_TARGETS];
    // Heap seen just before the first handshake and after the last one has been
    // torn down (the difference is what a handshake costs, and that it is
    // returned).
    uint32_t preflightHeapBefore = 0;
    uint32_t preflightMaxBlockBefore = 0;
    uint32_t preflightFreeHeap = 0;
    uint32_t preflightMaxBlock = 0;

    bool startPreflight();
    bool evaluatePreflightResult();
    static void preflightTaskFn(void* arg);

    void seedStatusNode();
    // The seed steps return false only when the step must be retried (a
    // transport failure that leaves it unknown whether the node exists). An
    // application-level "path does not exist" is the normal first-boot case
    // and is what triggers seeding.
    bool seedSettingsNode();
    bool seedCommandsCurrentNode();
    bool seedOperationsCurrentNode();

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
    // Shared tail for both the fresh-request and pending-retry paths in
    // readActuatorCommands()'s startProvisioning handling - only ever called
    // once the command node is confirmed consumed. See
    // provisioningCommandPendingDelete's comment for why that confirmation
    // matters.
    void beginManualProvisioningAfterCommandConsumed();
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

    // One-shot admin Test SMS trigger - reads /commands/testSms/{timestamp},
    // hands off to notificationManager.requestTestSms() (the real SMS
    // pipeline: durable queue -> recipient fan-out -> GsmManager), then
    // deletes the command node so a later poll/reconnect never replays it.
    // Mirrors consumeActuatorCommandSnapshot()'s delete-after-consume
    // pattern rather than a persisted enabled/disabled flag, since this is
    // a one-shot request, not a standing mode.
    void readTestSmsCommand();
    uint64_t lastTestSmsCommandTimestamp = 0;

    // Returns true once deviceId is known (already persisted, or just looked
    // up); false when the lookup could not complete and should be retried.
    bool provisionDevice();

    // Reads the 6 raw STA MAC bytes via esp_read_mac(ESP_MAC_WIFI_STA),
    // which works regardless of WiFi.mode()/WiFi.begin() state - unlike
    // WiFi.macAddress(), which reads back all-zero whenever the STA
    // interface has never been brought up (a device with no saved
    // credentials boots straight into WiFi.mode(WIFI_AP) provisioning, so
    // the STA netif is never started and WiFi.macAddress() has nothing to
    // report). Logs "[IDENTITY] ERROR: Unable to resolve hardware Wi-Fi
    // MAC" and returns false (out left untouched) if the read fails or
    // comes back all-zero.
    bool readHardwareStaMac(uint8_t out[6]);

    //==================================================
    // Secure Device Auth (bootstrap + refresh-token identity)
    //==================================================

    // Non-blocking auth state machine (critical verification report,
    // Priority 1). Both the first connection (advanceCloudStartupConnect(),
    // driven from update()) and beginFirebaseRecovery()/
    // pollFirebaseRecovery() (runtime, reachable at any time - including
    // mid-dose) drive the SAME underlying credential order (refresh token
    // -> device-secret bootstrap -> legacy anonymous fallback) through this
    // one implementation, so there is exactly one place that decides how a
    // device authenticates. See the .cpp for the full per-state rationale
    // and the one remaining bounded exception (the device-secret
    // bootstrap's HTTPS POST).
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
    // bounded (shortened-timeout) blocking call - see its own comment in
    // the .cpp for why a full non-blocking HTTP client is out of scope for
    // this pass. Returns true once the attempt has concluded (authPhase is
    // SUCCESS or FAILED this call); false while still in flight.
    bool pollAuthStateMachine();

    // Restores a previously-established identity from a persisted refresh
    // token (Firebase.setCustomToken() auto-detects a non-JWT-shaped string
    // as a refresh token and performs a refresh-grant sign-in directly - see
    // FirebaseCore.cpp's own signer logic - no bootstrap call needed). Kicks
    // off Firebase.begin() and returns immediately; WAIT_REFRESH_READY
    // polls Firebase.ready() non-blockingly for the actual result.
    bool restoreFromRefreshToken(const String& refreshToken);

    // Resolves the device MAC (fast, synchronous, no network) and hands the
    // {mac, secret} pair off to a dedicated background FreeRTOS task (see
    // bootstrapHttpTaskFn()) that performs the actual HTTPS POST - this
    // function itself returns immediately, never blocking the caller.
    // Returns false only if the MAC isn't resolvable yet or the background
    // task could not be created (e.g. out of heap); WAIT_BOOTSTRAP_HTTP
    // polls for the task's result non-blockingly. The secret is copied into
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
    // Firebase.ready()) still happens only in the main loop task, exactly
    // as before - so no Firebase library call is ever made from more than
    // one task. The task is pinned to the same core the main loop task
    // runs on (captured at kickoff via xPortGetCoreID(), not assumed) so
    // the two are always time-sliced, never truly concurrent, removing any
    // cross-core cache-visibility question for the plain bool/String
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
    // Last HTTP status the task saw (-1 = it never connected). Same
    // single-writer-then-signal rule as the fields above; used only to
    // classify a failed attempt in the [NET] log, never for control flow.
    int bootstrapHttpResultCode = 0;

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
    // errors) - grounded in the exact strings FB_Const.h's errorReason() can
    // return, not guessed. False for application-level outcomes such as
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
    // state and NVS credentials untouched), then returns immediately -
    // called once, from update(), the instant COOLDOWN's backoff window
    // elapses. pollFirebaseRecovery() advances that attempt by one bounded
    // step per subsequent update() call while firebaseHealth stays
    // RECOVERING, moving to HEALTHY on success or back to COOLDOWN with
    // escalated backoff on failure once the state machine concludes.
    void beginFirebaseRecovery();
    void pollFirebaseRecovery();

    //==================================================
    // Operation Protocol
    //==================================================

    bool hasActiveOperation() const;

    bool isOperationLifecycleOwned() const;

    bool isDuplicateRequest(
        uint16_t requestId,
        uint32_t requestTimestamp) const;

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
