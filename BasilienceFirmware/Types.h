#pragma once

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include "Config.h"

//==================================================
// Sensor Data
//==================================================

struct SensorData
{
    // Air temperature/humidity - see the automation resilience pass report.
    // Once a valid pair has ever been read, these fields become the
    // "lastGoodAirTemperature"/"lastGoodHumidity" the app must be able to
    // display continuously: SensorManager::readDHT() keeps holding them at
    // that last accepted value through a confirmed-unavailable streak
    // instead of collapsing to NaN, exactly like waterLevelCm/waterTemp
    // already never invent a fake reading but do hold the last real one.
    // dhtAvailable/dhtStale (below) are what distinguish "this number is a
    // fresh measurement" from "this number is a stale hold-over" - a control/
    // alert consumer must gate on dhtAvailable, never on isfinite() alone,
    // since isfinite() no longer implies freshness.
    float temperature = NAN;
    float humidity    = NAN;
    float waterTemp   = NAN;

    // True exactly when the most recent DHT22 acquisition is a fresh,
    // trusted measurement (not within a confirmed-unavailable streak - see
    // readDHT()'s SENSOR_TRANSIENT_FAILURE_THRESHOLD debounce). False while
    // confirmed unavailable OR before any reading has ever succeeded since
    // boot.
    bool dhtAvailable = false;

    // True only when dhtAvailable is false AND temperature/humidity hold a
    // genuine last-good value from before the outage (i.e. NOT the "never
    // had a valid reading since boot" case, which must display as plainly
    // unavailable rather than inventing a fake stale reading - see
    // temperature/humidity's own comment above). isfinite(temperature) is
    // the tell: dhtStale is true only when there is something real to be
    // stale.
    bool dhtStale = false;

    // Electrical Conductivity
    float ec        = NAN;
    float tds       = NAN;
    float ecVoltage = NAN;
    int   ecRaw     = 0;

    // True once SensorManager::readEC()'s rail-proximity fault detector
    // (Config.h's EC_FAULT_* constants) has CONFIRMED the raw signal is
    // persistently pinned at/near the ADC's own physical rail - conservative,
    // debounced evidence of a probable hardware fault (disconnected/
    // unpowered/shorted module), never a single abnormal sample and never a
    // chemistry-range judgment (see EC_FAULT_RAIL_LOW_MV/HIGH_MV's own
    // comment). While true, ec/tds above are forced to NaN and the EC
    // stability window is held reset - never conflate this with ec merely
    // being outside minEC/maxEC, which is a normal, valid, in-range-signal
    // condition automation is expected to correct via dosing.
    bool ecFault = false;

    // True exactly when ec above is a currently-trusted, non-NaN reading
    // (mirrors isfinite(ec) - published explicitly for the same reason
    // dhtAvailable is, rather than making every consumer isfinite()-check
    // itself). False while ecFault is true, while the post-(re)connect
    // PH_EC_ANALOG_SETTLE_TIME window is running, or before the stability
    // window has ever confirmed a first reading.
    bool ecAvailable = false;

    // pH
    float ph          = NAN;
    int   phMilliVolts = 0;

    // True once SensorManager::readPH()'s rail-proximity fault detector
    // (Config.h's PH_FAULT_* constants) has CONFIRMED the raw millivolt
    // signal is persistently pinned at/near the ADC's own physical rail or
    // at/beyond the pH transmitter's own documented 0-3.0V output spec -
    // conservative, debounced evidence of a probable hardware fault
    // (disconnected/unpowered/shorted probe or a dead transmitter output),
    // never a single abnormal sample and never a chemistry-range judgment.
    // While true, ph above is forced to NaN and the pH stability window/step
    // filter baseline are held reset, so recovery must fully re-earn trust
    // through the normal pipeline (see PH_FAULT_RECOVERY_COUNT's own
    // comment) - never conflate this with ph merely being outside minPH/
    // maxPH, which is a normal, valid, in-range-signal condition automation
    // is expected to correct via dosing.
    bool phFault = false;

    // True exactly when ph above is a currently-trusted, non-NaN reading -
    // same reasoning as ecAvailable above.
    bool phAvailable = false;

    // True while the pH temporal step filter has an unconfirmed
    // confirmation streak in progress - either the very first baseline
    // (boot/reacquisition) or a later large jump (quick-response
    // refinement task; see SensorManager::isPhConfirming()). ph above still
    // holds the last TRUSTED value the whole time (never NaN'd out just
    // because a new candidate is being confirmed, never replaced by the
    // unconfirmed candidate itself) - this flag is what lets a consumer
    // (Android) show "confirming a new reading" instead of silently
    // presenting a possibly-stale value as fully current. Purely a
    // display/status signal - automatic dosing is independently blocked by
    // the existing stability-window gate regardless of this flag.
    bool phConfirming = false;

    // Water Level (water-depth model - see Config.h's "Water Reservoir
    // Geometry" section and SensorManager::readWaterLevel())
    float waterLevel = 0; // Derived 0-100 working percentage. 0 is valid (empty tank) — kept as-is

    // Actual measured water DEPTH in centimeters. AUTHORITATIVE for
    // automation - refill/low-water control compares this directly, never
    // the derived percentage. Clamped only at the lower bound (0) - a
    // genuine overfill above MAX_WORKING_WATER_CM (6.0) is reported as its
    // real depth, never capped; only waterLevel (the derived percentage
    // below) clamps at 100%. NaN exactly when waterLevel is NaN (both set
    // together in readWaterLevel()).
    float waterLevelCm = NAN;

    // Estimated reservoir volume in liters, derived from waterLevelCm and
    // the fixed reservoir base area. Diagnostics/reporting only - no control
    // path consumes this.
    float waterVolumeLiters = NAN;

    // Refill threshold confirmation (see the automation resilience pass
    // follow-up report). waterLevelCm is already the step-filtered
    // accepted control value, but a small transient (e.g. one bad reading
    // 0.20cm off) can still pass that filter's WATER_LEVEL_STEP_ACCEPT_CM
    // band and momentarily cross REFILL_START_CM/REFILL_STOP_CM on its own.
    // These flags require WATER_LEVEL_STEP_CONFIRM_COUNT consecutive
    // ACCEPTED readings (SensorManager::readWaterLevel(),
    // WATER_LEVEL_READ_INTERVAL_MS = 5s apart, not the ~300ms an earlier
    // version of this comment incorrectly stated) on
    // the correct side of the threshold before the crossing is trusted -
    // REFILL START/COMPLETE must read these, never waterLevelCm compared
    // against the threshold directly.
    bool refillStartConfirmed = false;
    bool refillStopConfirmed = false;

    // Diagnostics only: raw HC-SR04 measured distance in centimeters, before
    // depth conversion. Never consumed by automation/alerts/safety.
    float waterLevelDistanceCm = NAN;

    // True while the fogger is running, during which SensorManager::
    // readWaterLevel() skips taking new readings entirely rather than
    // trusting an ultrasonic echo disturbed by the fogger's own mist and
    // surface turbulence in the same reservoir. waterLevelCm/waterLevel/
    // waterVolumeLiters are left untouched while this is true, so they keep
    // reporting the last accepted reading from before the fogger started.
    bool waterLevelHeldForFogger = false;
};



//==================================================
// System Modes
//==================================================

enum SystemMode
{
    SENSOR_STABILIZATION,

    STARTUP,

    NORMAL,

    REFILLING,

    DOSING_PH,
    STABILIZING_PH,

    DOSING_EC,
    STABILIZING_EC,

    SAFETY_LOCK
};

// Pulse-cooling state machine (replaces continuous-circulation cooling -
// see AutomationManager::updateCoolingPulseStateMachine()). Deliberately NOT
// a SystemMode value: cooling has always run independently of the reservoir-
// wide FSM above (coolingDemandActive/manualCoolingDemandActive), and staying
// that way keeps this entirely out of pH/EC/refill's currentMode-based mutual
// exclusion instead of risking new interactions with it. Read directly from
// systemState (not just AutomationManager-private) because
// ActuatorManager::validateCommand()'s PELTIER case and the automatic
// independent-deadline tables both need to know the current state too.
enum class CoolingPulseState : uint8_t
{
    IDLE,
    FILL,
    COOL_SOAK,
    FLUSH
};

// Internal FILL/FLUSH sequencing sub-phase - never read outside
// AutomationManager, purely local bookkeeping for the confirm-then-time-
// then-confirm sequence each of those two states runs through. Declared
// here (not nested in AutomationManager) only so it can sit next to
// CoolingPulseState; nothing outside AutomationManager.cpp touches it.
enum class CoolingPulsePhase : uint8_t
{
    NONE,
    WAIT_CIRCULATION_ON,
    TIMED_RUN,
    WAIT_CIRCULATION_OFF,
    WAIT_PELTIER_OFF
};

// Temporary developer-only controller isolation. NONE is the existing full
// system; every other value permits exactly one automatic controller plus
// its explicit support dependencies. Sensor acquisition/publication and
// actuator-level safety/manual arbitration are intentionally outside this
// enum and continue normally in every mode.
enum class AutomationTestSubsystem : uint8_t
{
    NONE,
    STARTUP,
    REFILL,
    PH,
    EC,
    COOLING,
    FOGGING,
    CANOPY,
    GROW_LIGHT
};

inline const char* automationTestSubsystemName(AutomationTestSubsystem subsystem)
{
    switch(subsystem)
    {
        case AutomationTestSubsystem::STARTUP: return "STARTUP";
        case AutomationTestSubsystem::REFILL: return "REFILL";
        case AutomationTestSubsystem::PH: return "PH";
        case AutomationTestSubsystem::EC: return "EC";
        case AutomationTestSubsystem::COOLING: return "COOLING";
        case AutomationTestSubsystem::FOGGING: return "FOGGING";
        case AutomationTestSubsystem::CANOPY: return "CANOPY";
        case AutomationTestSubsystem::GROW_LIGHT: return "GROW_LIGHT";
        case AutomationTestSubsystem::NONE:
        default: return "OFF";
    }
}

// Serial Monitor Focus Mode (see DebugManager::shouldPrintDebug()) - groups
// the codebase's existing Serial diagnostic tags by the subsystem they
// describe, so a single AutomationTestSubsystem selection can decide which
// categories stay visible without touching every individual print call
// site. Purely a Serial-output classification - has no effect on any
// control/automation decision.
enum class DebugCategory : uint8_t
{
    WATER,        // HC-SR04 acquisition, water step filter, refill control
    PH,           // pH ADC/stability/dosing diagnostics
    EC,           // EC ADC/stability/dosing diagnostics
    COOLING,      // DS18B20/cooling decision diagnostics
    FOGGING,      // fog cadence/cycle diagnostics
    CANOPY,       // canopy climate decision diagnostics
    DHT,          // DHT raw/health diagnostics - shared by CANOPY and FOGGING
    LIGHT,        // grow-light schedule/RTC diagnostics
    STARTUP,      // startup-phase-specific diagnostics
    NETWORK,      // WiFi/Firebase/upload/sync chatter
    GSM,          // GSM/SMS chatter
    NOTIFICATION, // notification/fogging-event queue chatter
    SYSTEM        // periodic dashboards, settings/mock/manual, misc
};

enum class SafetyResult
{
    SAFE,

    LOW_WATER,

    RESERVOIR_LOCK,

    SAFETY_LOCKED,

    SENSOR_FAULT,

    HIGH_WATER_TEMP,

    INVALID_PH,

    INVALID_EC,

    SUBSYSTEM_LOCKED,

    RESERVOIR_FULL
};


//==================================================
// Actuators
//==================================================

enum Actuator
{
    // SSR
    FOGGER,
    GROW_LIGHT,

    // MOSFET
    BLOWER,
    SOLENOID,

    // Dosing Pumps
    GROW_PUMP,
    BLOOM_PUMP,
    PH_UP_PUMP,
    PH_DOWN_PUMP,

    // Temperature
    CANOPY_FAN,
    PELTIER,
    CIRCULATION_PUMP,

    ACTUATOR_COUNT
};

inline const char* getActuatorName(Actuator a)
{
    switch(a)
    {
        case FOGGER: return "fogger";
        case GROW_LIGHT: return "growLight";
        case BLOWER: return "blower";
        case SOLENOID: return "solenoid";
        case GROW_PUMP: return "growPump";
        case BLOOM_PUMP: return "bloomPump";
        case PH_UP_PUMP: return "phUpPump";
        case PH_DOWN_PUMP: return "phDownPump";
        case CANOPY_FAN: return "canopyFan";
        case PELTIER: return "peltier";
        case CIRCULATION_PUMP: return "circulationPump";
        default: return "unknown";
    }
}

enum class ActuatorCommandState
{
    OFF,
    COMMAND_RECEIVED,
    VALIDATING,
    REJECTED,
    ACTIVATING,
    RUNNING,
    STOPPING
};

struct ActuatorCommand
{
    bool isPending = false;
    bool targetState = false;
    uint8_t speed = 100;
    String source = "";
    String strategy = "";
    String reason = "";
    uint32_t timestamp = 0;
    // One-shot manual-override intent for THIS command only (see
    // ActuatorManager::validateCommand). Never persists past the command it
    // arrived on - a later command with this unset/false returns to normal
    // soft-rule enforcement.
    bool overrideRequested = false;
    // Waives BLOWER's automatic "fogger must be RUNNING" gate for this
    // command - set by AutomationManager's deliberate post-fogger purge
    // window (BLOWER_PURGE_MS), where the fogger has already stopped on
    // purpose. See ActuatorManager::validateCommand's BLOWER case.
    bool bypassAutoFoggerGate = false;
};

struct ActuatorStatus
{
    ActuatorCommandState state = ActuatorCommandState::OFF;
    bool running = false;
    uint8_t speed = 100;
    uint32_t startedAt = 0;
    String source = "";
    String strategy = "";
    String reason = "";
    // Mirrors the overrideRequested carried by the command currently
    // occupying this actuator, so validateCommand's continuous RUNNING-state
    // re-check (which reads status, not the transient command) can honor the
    // same override for the actuator's whole run, and so actuatorStatus can
    // publish it for the app to display.
    bool overrideActive = false;
    // Mirrors bypassAutoFoggerGate the same way overrideActive mirrors
    // overrideRequested, so the continuous RUNNING-state re-check sees it
    // for as long as this command's purge window lasts.
    bool bypassAutoFoggerGate = false;
    // True for exactly one tick after ActuatorManager's independent esp_timer
    // deadline (not AutomationManager's own dose/refill-duration timer)
    // force-stopped this actuator - distinct from a normal on-time stop, so
    // AutomationManager can tell "I stopped this myself" apart from
    // "something already forced this off, possibly while loop() was
    // stalled" and reconcile its own state machine (DOSING->STABILIZING,
    // REFILLING RUNNING->SETTLING) instead of re-issuing a now-physically-
    // meaningless command or silently believing the operation is still in
    // progress. Set only by ActuatorManager::update()'s existing deadline-
    // expiry reconciliation block; cleared automatically the next time a
    // fresh command is accepted for this actuator.
    bool forcedOffByDeadline = false;
};

//==================================================
// pH
//==================================================

enum PHDirection
{
    PH_NONE,
    PH_UP,
    PH_DOWN
};

enum ECDirection
{
    EC_NONE,
    EC_RAISE,
    EC_DILUTE
};


//==================================================
// Operation Protocol
//==================================================

enum class OperationType
{
    NONE,

    REFILL,

    PH_UP,
    PH_DOWN,

    EC_CORRECTION,

    RESET_SAFETY
};

enum class OperationAction
{
    NONE,

    START,
    STOP,

    ENABLE,
    DISABLE,

    EXECUTE
};

enum class RequestSource
{
    NONE,
    MANUAL,
    AUTOMATIC
};

enum class RequestState
{
    IDLE,
    PENDING,
    ACCEPTED,
    RUNNING,
    COMPLETED,
    REJECTED,
    FAILED
};

//==================================================
// Operation Request
//==================================================

struct OperationRequest
{
    uint16_t requestId = 0;

    OperationType operation = OperationType::NONE;
    RequestSource source = RequestSource::NONE;
    OperationAction action = OperationAction::NONE;
    RequestState state = RequestState::IDLE;


    char reason[64] = "";

    unsigned long requestTimestamp = 0;
    unsigned long acceptedTimestamp = 0;
    unsigned long startedTimestamp = 0;
    unsigned long completedTimestamp = 0;
    unsigned long lastUpdatedTimestamp = 0;
    
};


enum class OperationSource
{
    NONE,
    MANUAL,
    AUTOMATIC
};

//==================================================
// Runtime System State
//==================================================

enum class CorrectionMode
{
    NONE,
    AUTOMATIC,
    MANUAL
};
    
struct SystemState
{
    //==================================================
    // Runtime
    //==================================================

    bool manualMode = false;

    bool settingsLoaded = false;

    bool wifiConnected = false;

    bool firebaseConnected = false;

    bool syncRTC = false;

    bool safetyLock = false;

    bool phSubsystemLocked = false;
    bool ecSubsystemLocked = false;
    bool refillSubsystemLocked = false;
    bool coolingSubsystemLocked = false;

    // Pulse-cooling task: current phase of the FILL/COOL_SOAK/FLUSH cycle.
    // Owned/advanced by AutomationManager::updateCoolingPulseStateMachine();
    // read (never written) by ActuatorManager for the one narrow COOL_SOAK
    // Peltier-without-circulation exception and its independent deadline.
    CoolingPulseState coolingPulseState = CoolingPulseState::IDLE;

    bool reservoirLocked = false;

    bool resetSafetyLock = false;

    // Developer Mode physical sensor diagnostics
    bool sensorTestEnabled = false;
    unsigned long sensorTestStartTime = 0;

    // Developer testing override: bypasses ONLY the automatic
    // water-level/refill gate (AutomationManager's handleNormal(),
    // handleRefilling()) so pH/EC/fogging/
    // cooling automation can be exercised on hardware sitting below
    // criticalLowWaterCm. The water-level sensor itself, its alerts, and
    // manual refill are all untouched - see FirebaseManager::
    // setIgnoreWaterLevelAutomation()/readWaterLevelOverrideCommand().
    // Deliberately not restored from NVS: resets to false on every reboot,
    // same as sensorTestEnabled above, so a developer testing session never
    // silently survives a power cycle.
    bool ignoreWaterLevelAutomation = false;

    // Temporary developer-only automation isolation. FirebaseManager applies
    // commands/automationTestMode and echoes this confirmed value under
    // status/automationTestMode. It is deliberately not persisted in NVS.
    AutomationTestSubsystem automationTestSubsystem =
        AutomationTestSubsystem::NONE;

    // Remote Mocking
    bool mockSensorsEnabled = false;
    bool mockSensorsDynamic = false;
    // The RTDB command values are authoritative bases. mockSensors is the
    // current effective dataset and equals these bases in static mode.
    SensorData mockSensorBases;
    SensorData mockSensors;
    unsigned long mockDynamicUpdatedAt = 0;
    bool mockApplyPending = false;

    // True once the effective sensor source (mock vs. physical) has been
    // confirmed at least once from Firebase since boot. Shared between
    // FirebaseManager (which resolves it) and SensorManager/AlertManager
    // (which must not act on physical readings until it is true) so a
    // reboot/brownout can't let temporary physical readings drive automation
    // before a previously-enabled mock mode is restored.
    bool sensorSourceResolved = false;

    // Coherent-snapshot readiness baseline (see the real-time sensor
    // presentation task report) - millis() of the most recent boot or
    // mock/physical source transition. 0 at true boot (millis() is already
    // ~0 then, so "elapsed since 0" naturally satisfies
    // SENSOR_SNAPSHOT_READY_DELAY_MS shortly after real power-on with no
    // special-case needed). Reset by SensorManager::applyEffectiveSensors()
    // on a genuine source transition, mirroring how phStabilityWindow/
    // ecStabilityWindow/the pH temporal filter are reset at the same
    // instant - published as /sensors/sensorState/{stabilizing,ready} by
    // FirebaseManager::writeSensors() so Android knows when a coherent
    // initial snapshot exists, without requiring every sensor to be fully
    // settled (see SENSOR_SNAPSHOT_READY_DELAY_MS's own comment).
    unsigned long sensorSnapshotBaselineAt = 0;

    // Armed by AutomationManager::completeCurrentOperation() the instant an
    // automatic PH_UP/PH_DOWN/EC_CORRECTION reaches COMPLETED locally.
    // Released by FirebaseManager as soon as that COMPLETED state is
    // confirmed published to RTDB, or by AutomationManager after a bounded
    // local grace period if Firebase cannot be reached - so Fogger/Blower
    // resume waits for the cloud when possible but is never blocked by it.
    bool chemistryFoggingHoldActive = false;
    unsigned long chemistryFoggingHoldStartTime = 0;

    //==================================================
    // Operations
    //==================================================

    OperationRequest operationRequest;

    uint16_t lastProcessedRequestId = 0;

    //==================================================
    // State Machine
    //==================================================

    SystemMode currentMode =
        SENSOR_STABILIZATION;

    unsigned long stateStartTime = 0;

    unsigned long dosingStartTime = 0;

    CorrectionMode correctionMode =
        CorrectionMode::NONE;

    bool firstCorrectionCycle = true;

    //==================================================
    // pH/EC correction redesign - quiet monitoring + 4-minute budget
    //==================================================

    // Whole-correction time-budget anchor, shared by pH and EC since
    // currentMode only ever runs one of DOSING_PH/STABILIZING_PH/DOSING_EC/
    // STABILIZING_EC at a time - the two can never be active concurrently.
    // 0 = no correction currently running (same sentinel convention as
    // lastPhDoseEndedAt below). Set once when a correction first triggers
    // (processPHCorrection()/processECCorrection()), left untouched by an
    // internal redose, cleared on completion or failure. See
    // PH_EC_CORRECTION_STALL_TIMEOUT_MS (Config.h).
    unsigned long correctionCycleStartAt = 0;

    // True only while handleStabilizingPH()/handleStabilizingEC() are past
    // their initial silent settle window (PH_STABILIZATION_TIME/
    // EC_STABILIZATION_TIME) and purely watching, not actively dosing and
    // not waiting for a fresh dose to mix in yet. Lets
    // SafetyManager::canFog() allow fogging to resume during that window -
    // canFog()'s own pH/EC range checks already confirm the reading is
    // actually safe; this only says "and it is current enough to trust."
    // Set fresh every tick (true inside the watch block, false via
    // changeState() on entering DOSING_PH/STABILIZING_PH) rather than
    // relying on scattered resets, so it can never linger stale.
    bool phWatchPhaseActive = false;
    bool ecWatchPhaseActive = false;

    //==================================================
    // pH
    //==================================================

    PHDirection phDirection =
        PH_NONE;

    unsigned long phDoseTime = 0;

    uint8_t phAttempts = 0;

    // Trend tracking for handleStabilizingPH() - lets it tell "still
    // improving on its own," "stalled," and "reversing" apart instead of
    // blindly redosing on a timer. phLastTrendImproving gates whether a
    // NEW redose fires while the correction is still under budget (a
    // reversal redoses immediately; "improving"/no-progress do not). It no
    // longer has any bearing on the PH_EC_CORRECTION_STALL_TIMEOUT_MS
    // deadline verdict itself (correction-budget limbo fix) - that deadline
    // is a hard episode ceiling regardless of this flag's value; see
    // Config.h's own comment on PH_EC_CORRECTION_STALL_TIMEOUT_MS.
    float phTrendReferenceValue = NAN;
    unsigned long phLastTrendCheckAt = 0;
    bool phLastTrendImproving = true;

    // First-checkpoint / stable-hold-for-publish bookkeeping - see
    // PH_EC_STABLE_HOLD_FOR_PUBLISH_MS (Config.h).
    bool phFirstCheckpointPublished = false;
    unsigned long phStableSince = 0;
    bool phStableCheckpointPublished = false;

    // One-shot: AutomationManager sets this true for exactly the tick a
    // checkpoint should be published; FirebaseManager consumes and clears
    // it in the same write that includes the value.
    bool phPublishPending = false;

    // FirebaseManager::writeSensors() writes the /sensors node with a full
    // setJSON() overwrite, not a merge - omitting "ph" from that JSON would
    // DELETE the field from the database, not freeze it. This holds the
    // last value actually published (either normal unheld publishing, or
    // one of the two correction checkpoints) so a held write can keep
    // republishing something instead of wiping the field.
    float phLastPublishedValue = NAN;

    // millis() a pH-Up/pH-Down pump last actually finished running, from
    // EITHER source (manual or automatic) - see
    // ActuatorManager::update()'s STOPPING case, the single funnel every
    // pump-off transition passes through. 0 means "never dosed this boot,"
    // a sentinel rather than a real timestamp so the cooldown below never
    // blocks the very first correction. Consumed by
    // AutomationManager::canStartNewPHCorrection() as PH_DOSE_COOLDOWN - a
    // real minimum wait, not just the stability window's own "has the
    // reading stopped moving" check, since a probe can settle into a
    // misleadingly steady reading before the dosed chemical has actually
    // finished mixing into the reservoir.
    unsigned long lastPhDoseEndedAt = 0;

    float minPH = MIN_PH;

    float maxPH = MAX_PH;

    float phTargetMin = PH_TARGET_MIN;

    float phTargetMax = PH_TARGET_MAX;

    //==================================================
    // EC
    //==================================================

    unsigned long ecDoseTime = 0;

    uint8_t ecAttempts = 0;

    // Mirrors the phTrendReferenceValue/phLastTrendCheckAt/
    // phLastTrendImproving trio above, for handleStabilizingEC().
    float ecTrendReferenceValue = NAN;
    unsigned long ecLastTrendCheckAt = 0;
    bool ecLastTrendImproving = true;

    // Mirrors the ph* checkpoint/publish bookkeeping above, for EC.
    bool ecFirstCheckpointPublished = false;
    unsigned long ecStableSince = 0;
    bool ecStableCheckpointPublished = false;
    bool ecPublishPending = false;

    // See phLastPublishedValue's own comment above. ecLastPublishedTds
    // mirrors ecLastPublishedValue for the derived tds field, held/
    // published in lockstep with ec rather than tracked independently.
    float ecLastPublishedValue = NAN;
    float ecLastPublishedTds = NAN;

    // Same purpose/sentinel convention as lastPhDoseEndedAt above, for the
    // Grow/Bloom pumps - EC_DOSE_COOLDOWN is enforced from this in
    // AutomationManager::canStartNewECCorrection().
    unsigned long lastEcDoseEndedAt = 0;

    float minEC = MIN_EC;

    float maxEC = MAX_EC;

    float ecTargetMin = EC_TARGET_MIN;

    float ecTargetMax = EC_TARGET_MAX;

    ECDirection ecDirection = EC_NONE;

    //==================================================
    // Grow Light Schedule
    //==================================================

    uint8_t lightOnHour = 6;
    uint8_t lightOnMinute = 0;

    uint8_t lightOffHour = 22;
    uint8_t lightOffMinute = 0;

    // Developer-only mock "current time" for deterministically testing Grow
    // Light scheduling while the physical RTC is invalid/unavailable. Never
    // written to the DS3231 and never influences anything outside
    // AutomationManager::getCurrentMinutes()/updateGrowLightSchedule() - see
    // their own comments for the strict activation condition (both
    // automationTestSubsystem == GROW_LIGHT and this flag must be true).
    // Minutes since midnight, clamped to 0..1439 on receipt from Firebase.
    bool mockGrowLightTimeEnabled = false;
    uint16_t mockGrowLightMinutes = 0;

    //==================================================
    // Reservoir
    //==================================================

    // LEGACY percentage-based control thresholds - no longer read by any
    // control path (see Config.h's matching comment). Kept only so existing
    // NVS/RTDB data is not silently discarded.
    float refillStartLevel =
        REFILL_START_LEVEL;

    float refillStopLevel =
        REFILL_STOP_LEVEL;

    // AUTHORITATIVE water-depth control thresholds (centimeters) - see
    // Config.h's "Water Reservoir Geometry" section for the full design.
    // Settable via /settings (FirebaseManager::readSettings()), same pattern
    // as the legacy percentage fields above.
    float criticalLowWaterCm =
        CRITICAL_LOW_WATER_CM;

    float refillStartLevelCm =
        REFILL_START_CM;

    float refillStopLevelCm =
        REFILL_STOP_CM;

    // LEGACY - see the automation resilience pass report. No longer
    // consumed by readWaterLevel(); superseded by sensorToBottomCm below.
    // Left in place, unmodified, so existing NVS/RTDB data and any external
    // reader of /settings/waterLevelEmptyDistanceCm is not silently broken.
    // Deliberately NOT reused as the authoritative field: a stale persisted
    // value under this same key (NVS "wlEmptyCm" / RTDB
    // waterLevelEmptyDistanceCm) is exactly what let runtime keep computing
    // depth against 30.00cm long after the compiled default was corrected to
    // 28.67cm - reusing this key would let that same stale value silently
    // survive again.
    float waterLevelEmptyDistanceCm =
        WATER_LEVEL_EMPTY_DISTANCE_CM;

    // AUTHORITATIVE ultrasonic sensor-to-reservoir-bottom distance (cm) -
    // see Config.h's WATER_LEVEL_EMPTY_DISTANCE_CM for why this needs to be
    // configurable rather than a fixed constant (sensor mounting height
    // varies per installation). Its own NVS key ("sensorBottomCm") and its
    // own RTDB /settings field ("sensorToBottomCm"), independent of the
    // legacy waterLevelEmptyDistanceCm above - see this field's own comment.
    float sensorToBottomCm =
        WATER_LEVEL_EMPTY_DISTANCE_CM;

    // LEGACY - no longer consumed by the water-depth/percent/liters
    // conversion (see Config.h's matching comment).
    float waterLevelFullDistanceCm =
        WATER_LEVEL_FULL_DISTANCE_CM;

    //==================================================
    // Target (acceptable) ranges
    //
    // What "in range" means for Monitoring, alerts and Reports. Distinct from
    // the actuator control/hysteresis thresholds further below - see Config.h.
    // pH and EC already have their canonical pairs above (minPH/maxPH,
    // minEC/maxEC) and are not duplicated here.
    //==================================================

    float minAirTemp = TARGET_MIN_AIR_TEMP;
    float maxAirTemp = TARGET_MAX_AIR_TEMP;

    float minHumidity = TARGET_MIN_HUMIDITY;
    float maxHumidity = TARGET_MAX_HUMIDITY;

    float minWaterTemp = TARGET_MIN_WATER_TEMP;
    float maxWaterTemp = TARGET_MAX_WATER_TEMP;

    float minWaterLevel = TARGET_MIN_WATER_LEVEL;
    float maxWaterLevel = TARGET_MAX_WATER_LEVEL;

    //==================================================
    // Temperature
    //==================================================

    float highAirTemp =
        HIGH_AIR_TEMP;

    float airTempRelease =
        AIR_TEMP_RELEASE;

    // Canopy Fan's own cold-side control pair - see LOW_AIR_TEMP/
    // COLD_AIR_RELEASE's own comment in Config.h.
    float lowAirTemp =
        LOW_AIR_TEMP;

    float coldAirRelease =
        COLD_AIR_RELEASE;

    float highHumidity =
        HIGH_HUMIDITY;

    float humidityRelease =
        HUMIDITY_RELEASE;

    float highWaterTemp =
        HIGH_WATER_TEMP;

    float coolerOffTemp =
        COOLER_OFF_TEMP;

    float hotFogTemperature =
        HOT_FOG_TEMPERATURE;

    float coldFogTemperature =
        COLD_FOG_TEMPERATURE;

    // Automatic Blower PWM speed used while the Fogger/Blower pair is
    // automatically ON (real-hardware Canopy/Blower PWM follow-up) - see
    // AutomationManager::processFogCycle() and Config.h's
    // BLOWER_SPEED_DEFAULT_PERCENT/MIN/MAX for the full contract. Explicit
    // manual Blower speed commands are unaffected by this field entirely.
    uint8_t blowerSpeedPercent = BLOWER_SPEED_DEFAULT_PERCENT;

    // Config/settings schema migration bookkeeping - see CONFIG_SCHEMA_VERSION
    // in Config.h. True from boot (set in FirebaseManager::
    // loadPersistedSettings() whenever the persisted NVS "cfgVersion" is
    // behind CONFIG_SCHEMA_VERSION) until this device's corrected
    // maxAirTemp/blowerSpeedPercent values have been successfully PUSHED to
    // Firebase at least once (FirebaseManager::readSettings()), at which
    // point cfgVersion is persisted and this never fires again. Local
    // values are corrected immediately regardless of this flag - it only
    // gates the one-time Firebase-side reconciliation, which needs
    // connectivity this field's own local correction does not.
    bool configMigrationPending = false;
};

//==================================================
// Alerts
//==================================================

struct AlertState
{
    bool lowWater = false;

    // Severity escalation ON TOP of lowWater above (see the static automation
    // integration audit) - waterLevelCm <= criticalLowWaterCm (1.0cm), a
    // stricter bar than lowWater's refillStartLevelCm (2.0cm). Purely a
    // notification/status severity signal: the <=2.0cm operational block
    // (pH/EC dosing, fogging, cooling) is already fully enforced by lowWater
    // via SafetyManager/ActuatorManager, so this flag drives no additional
    // actuator gating of its own.
    bool criticalLowWater = false;

    bool highTemperature = false;

    bool lowAirTemperature = false;

    bool ecLow = false;

    bool ecHigh = false;

    bool phOutOfRange = false;

    bool phLow = false;

    bool phHigh = false;

    // Above maxWaterTemp. Name kept for compatibility with the existing Cloud
    // Function producer and the Android readers that already consume it.
    bool waterTempOutOfRange = false;
    // Below minWaterTemp - new directional counterpart.
    bool waterTempLow = false;

    // Humidity had no alert flags at all before target ranges existed.
    bool humidityLow = false;
    bool humidityHigh = false;

    // Target-range classification for water level. Deliberately separate from
    // lowWater above, which is the CONTROL signal that triggers automatic
    // refill and must keep following refillStartLevelCm.
    bool waterLevelLow = false;
    bool waterLevelHigh = false;

    bool sensorFault = false;
};

//==================================================
// Actuator Runtime
//==================================================

struct ActuatorState
{
    bool states[ACTUATOR_COUNT] = { false };
};

//==================================================
// Telemetry
//==================================================

struct ActuatorTelemetry
{
    bool fogger = false;

    bool growLight = false;

    bool blower = false;

    bool solenoid = false;

    bool growPump = false;

    bool bloomPump = false;

    bool phUpPump = false;

    bool phDownPump = false;

    bool peltier = false;
};


#endif
