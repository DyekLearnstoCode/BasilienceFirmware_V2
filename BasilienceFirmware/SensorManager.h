#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H
#include "Types.h"
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

#include "AnalogSampler.h"
#include "Config.h"

// Sliding-window stability gate for pH/EC - see
// SensorManager::updateStabilityWindow() and applyEffectiveSensors(), and
// STABILITY_SAMPLE_WINDOW/STABILITY_SAMPLE_INTERVAL_MS/PH_EC_STABLE_TIMEOUT_MS
// in Config.h for the cadence/tolerance/staleness design. One instance per
// sensor (SensorManager's phStabilityWindow/ecStabilityWindow).
struct StabilityWindow
{
    float samples[STABILITY_SAMPLE_WINDOW] = {NAN};
    uint8_t count = 0;              // valid samples currently held (<= STABILITY_SAMPLE_WINDOW)
    uint8_t next = 0;                // next ring-buffer slot to write
    unsigned long lastSampleAt = 0;  // throttles candidate intake to ~STABILITY_SAMPLE_INTERVAL_MS
    float lastStable = NAN;          // last accepted representative value - kept through a temporary-unstable or stale period as diagnostic history AND as what sensors.ph/ec keeps displaying/publishing (see applyEffectiveSensors())
    unsigned long lastStableAt = 0;  // millis() of that acceptance; staleness is measured from this
    bool hasStable = false;          // true once ANY stable value has ever been accepted (permanent - not cleared by a later stale timeout)
    // True exactly when the MOST RECENT window evaluation passed the
    // tolerance check - distinct from hasStable/lastStable, which persist
    // through a temporary instability so Firebase/display keep showing the
    // old value. This is what gates a NEW chemical correction from starting
    // (see AutomationManager::canStartNewPHCorrection()/
    // canStartNewECCorrection()) - a retained lastStable value is fine to
    // display, but not sufficient evidence to act on while the live signal
    // hasn't reconfirmed it.
    bool currentlyStable = true;
    bool staleLogged = false;        // edge-detection for the "stale" transition log only

    // Throttle for the periodic [xx-STABLE] diagnostic in
    // updateStabilityWindow() (real-hardware pre-integration follow-up -
    // Part A). The existing "unstable; keeping last=" log only fires ONCE,
    // on the stable->unstable transition edge - if the window then never
    // agrees again, there is no further evidence of why on the bench. This
    // timestamp throttles a periodic candidate/min/max/range/tolerance
    // dump that fires regardless of stable/unstable outcome, independent of
    // the transition-edge logs above.
    unsigned long lastDiagnosticAt = 0;
};

class SensorManager
{
public:
    SensorManager();

    void begin();

    void update();

private:
    // =====================================================
    // Hardware
    // =====================================================

    DHT dht;

    OneWire oneWire;

    DallasTemperature waterSensor;

    AnalogSampler ecSampler;

    AnalogSampler phSampler;

    // Throttle for readPH()'s [PH-ADC] diagnostic - see
    // PH_ADC_DIAGNOSTIC_INTERVAL_MS's own comment.
    unsigned long lastPhAdcDiagnosticAt = 0;

    bool sensorSourceReported = false;
    bool lastReportedMockSource = false;
    bool sensorSourceWaitingLogged = false;
    bool mockBootWaitHeldLogged = false;

    // Sensor-source persistence. The effective source (mock vs. physical) is
    // decided locally at boot from NVS so a cold boot with no Wi-Fi/Firebase
    // still reaches a definite source and local automation can run. Firebase
    // remains authoritative once reachable and reconciles this value.
    static constexpr const char* SOURCE_NVS_NAMESPACE = "sensorsrc";
    static constexpr const char* SOURCE_NVS_KEY = "mockEnabled";
    Preferences sourcePreferences;

    // Boot-restored mock source waiting for its first payload of this session.
    // Armed only by resolveLocalSensorSource(); a mock session enabled from
    // the app after boot never arms it.
    bool mockBootWaitingForPayload = false;
    unsigned long mockBootWaitStartedAt = 0;

    // Timestamp of the most recent moment physical sensors became the active
    // source (cold boot direct to physical, mock boot-wait timeout, or the
    // cloud turning mock off). applyEffectiveSensors() holds sensors.ph/ec
    // NaN for PH_EC_ANALOG_SETTLE_TIME after this, since the analog probes
    // haven't electrically settled yet even though physicalSensors already
    // has a finite (but still drifting) reading.
    unsigned long physicalPhEcSettledAt = 0;

    void resolveLocalSensorSource();
    void updateMockBootWait();

    // Water-temperature read scheduling and transient-failure tolerance.
    // physicalSensors.waterTemp only becomes NaN once a scheduled read has
    // failed WATER_TEMP_READ_INTERVAL_MS-spaced attempts consecutively for
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD times; lastValidWaterTemp is kept
    // separately so readEC() can still compensate using it even after that.
    unsigned long lastWaterTempReadTime = 0;
    uint8_t waterTempFailureStreak = 0;
    float lastValidWaterTemp = NAN;

    // Light exponential smoothing over accepted raw DS18B20 readings - see
    // readWaterTemperature()'s own comment for why (real-hardware
    // pre-integration Part D: occasional per-sample flicker with no
    // averaging at all before this). NaN means "no filtered value yet";
    // reset (not blended) on a confirmed-unavailable -> recovered
    // transition so recovery never blends against a stale pre-outage value.
    float waterTempFiltered = NAN;

    // DS18B20 enumeration state. 0 devices at boot is re-checked on the same
    // throttled WATER_TEMP_READ_INTERVAL_MS cadence readWaterTemperature()
    // already uses - no separate timer - so a probe that wasn't settled yet
    // at begin() is picked up automatically once it starts responding.
    // waterSensorAddress is cached once enumeration succeeds so normal reads
    // use DallasTemperature::getTempC(address) instead of re-walking the
    // OneWire bus search on every getTempCByIndex(0) call.
    uint8_t waterSensorDeviceCount = 0;
    DeviceAddress waterSensorAddress = {0};
    bool waterSensorAddressValid = false;

    enum class EcCompensationSource { LIVE, LAST_VALID, FALLBACK_DEFAULT };
    EcCompensationSource lastEcCompensationSource = EcCompensationSource::LIVE;

    // Authoritative pH/EC stability state - see StabilityWindow's own
    // comment and applyEffectiveSensors()'s use of these to build
    // sensors.ph/sensors.ec.
    StabilityWindow phStabilityWindow;
    StabilityWindow ecStabilityWindow;
    void updateStabilityWindow(StabilityWindow& window, float candidate, float tolerance, const char* logTag, DebugCategory category);
    void resetStabilityWindow(StabilityWindow& window);

    // pH temporal step filter - see Config.h's PH_STEP_ACCEPT_DELTA/
    // PH_STEP_CONFIRM_TOLERANCE/PH_STEP_CONFIRM_COUNT and
    // applyEffectiveSensors()'s own comment. Mirrors the HC-SR04 water-depth
    // step filter's design (lastAcceptedWaterDepthCm/waterLevelStepCandidateCm/
    // waterLevelStepCandidateCount above). lastAcceptedPhCandidate is the
    // last TRUSTED candidate - this IS the FAST TELEMETRY value published as
    // sensors.ph (quick-response refinement task), completely independent of
    // whether phStabilityWindow below has itself converged; NAN means no
    // trusted baseline yet (boot, or the filter has never confirmed a first
    // reading - never published as a fabricated number, see
    // applyEffectiveSensors()'s own comment). phStepCandidate/
    // phStepCandidateCount track an in-progress confirmation streak (used
    // for BOTH initial-baseline establishment and a later large-jump
    // confirmation - structurally identical, same as the water filter's own
    // reacquisition-vs-jump reuse); phStepCandidate is NAN exactly when no
    // streak is in progress, which isPhCurrentlyStable()/isPhConfirming()
    // below also read directly.
    float lastAcceptedPhCandidate = NAN;
    float phStepCandidate = NAN;
    uint8_t phStepCandidateCount = 0;

    // millis() lastAcceptedPhCandidate was last (re)established - the
    // TELEMETRY side's own freshness clock, independent of
    // phStabilityWindow.lastStableAt (the AUTOMATION-TRUST side's). See
    // applyEffectiveSensors()'s own comment for why telemetry needs its own
    // staleness check rather than sharing the window's.
    unsigned long lastAcceptedPhCandidateAt = 0;
    // Edge-detection for the telemetry-side "[PH-FILTER] telemetry stale"
    // log, mirroring StabilityWindow::staleLogged's own pattern.
    bool phTelemetryStaleLogged = false;

    // Throttles the step filter's own evaluation (state advancement AND its
    // [PH-FILTER] diagnostics) to PH_STEP_SAMPLE_INTERVAL_MS (quick-response
    // refinement task - deliberately faster than STABILITY_SAMPLE_INTERVAL_MS,
    // which the automation-trust window below still uses unchanged). readPH()
    // recomputes physicalSensors.ph on every loop() tick from a
    // continuously-updating median, so without this throttle "3 consecutive
    // candidates" could be satisfied by evaluating the SAME unchanged median
    // dozens of times within milliseconds, which is not 3 genuinely distinct
    // observations (the same bug class already fixed for the water-refill
    // threshold confirmation counters - see readWaterLevel()'s own comment).
    unsigned long lastPhStepEvalAt = 0;

    // HC-SR04 read scheduling and transient-failure tolerance, mirroring the
    // water-temperature pattern above. physicalSensors.waterLevel/
    // waterLevelDistanceCm only become NaN once a scheduled read has failed
    // WATER_LEVEL_READ_INTERVAL_MS-spaced attempts consecutively for
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD times.
    unsigned long lastWaterLevelReadTime = 0;
    uint8_t waterLevelFailureStreak = 0;

    // Median-of-5 filter over the raw distance samples (widened from
    // median-of-3 - real-hardware pre-integration Part E: ~1cm of
    // sample-to-sample HC-SR04 jitter was still passing through a 3-sample
    // median under stationary water). The refill solenoid's flow also
    // ripples the reservoir surface enough to shift a single echo by
    // several cm, which - fed straight to the alert layer - flapped
    // waterLevelLow on and off within one debounce window. A true median
    // (not an average) rejects an outlier sample instead of smoothing it
    // away, which would lag a real level change; cleared on
    // confirmed-unavailable so a recovery doesn't median against stale
    // pre-outage readings.
    float waterLevelDistanceHistory[5] = {NAN, NAN, NAN, NAN, NAN};
    uint8_t waterLevelHistoryIndex = 0;
    uint8_t waterLevelHistoryCount = 0;

    // Second-stage temporal plausibility filter over the median-of-5 output -
    // see Config.h's WATER_LEVEL_STEP_ACCEPT_CM/WATER_LEVEL_STEP_CONFIRM_*
    // and readWaterLevel()'s own comment. lastAcceptedWaterDepthCm is the
    // control-authoritative accepted value (what physicalSensors.waterLevelCm
    // is actually set from); NAN means "no accepted depth yet" (boot, or just
    // recovered from a confirmed sensor outage), which accepts the very next
    // candidate immediately rather than waiting on a confirmation streak
    // against nothing. waterLevelStepCandidateCm/waterLevelStepCandidateCount
    // track an in-progress large-jump confirmation streak; reset to
    // NAN/0 whenever a candidate does not agree with the pending one.
    float lastAcceptedWaterDepthCm = NAN;
    float waterLevelStepCandidateCm = NAN;
    uint8_t waterLevelStepCandidateCount = 0;

    // Refill threshold confirmation - see Types.h's refillStartConfirmed/
    // refillStopConfirmed and readWaterLevel()'s own comment. Counts
    // consecutive ACCEPTED readings (not loop() ticks) on the correct side
    // of refillStartLevelCm/refillStopLevelCm; reset to 0 the instant an
    // accepted reading falls back on the wrong side, and also on a confirmed
    // sensor outage (nothing to confirm while unavailable).
    uint8_t refillStartConfirmCount = 0;
    uint8_t refillStopConfirmCount = 0;

    // Edge-triggered Serial logging state for the two counters above -
    // separate from the counters themselves, which stay exactly as they are
    // (control-authoritative, consumed by refillStartConfirmed/
    // refillStopConfirmed regardless of what has or hasn't been printed).
    // Tracks the last COUNT VALUE actually printed, reset to 0 whenever the
    // real counter itself resets to 0, so "N/3" only prints on a genuine
    // 0->1->2->3 progression - never repeated once already at 3/3 - and a
    // later fresh episode (after the real counter resets) prints its own
    // fresh 1/3, 2/3, 3/3 sequence again.
    uint8_t refillStartConfirmLoggedCount = 0;
    uint8_t refillStopConfirmLoggedCount = 0;

    // Throttle for readWaterLevel()'s [WATER] depth/percent/volume
    // diagnostic - readWaterLevel() itself runs every
    // WATER_LEVEL_READ_INTERVAL_MS (300ms), far more often than this log
    // line needs to print.
    unsigned long lastWaterLevelDiagnosticAt = 0;

    // DHT22 read scheduling and transient-failure tolerance, mirroring the
    // water-temperature/water-level pattern above. physicalSensors.humidity/
    // temperature only become NaN once a scheduled read has failed
    // DHT_READ_INTERVAL_MS-spaced attempts consecutively for
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD times.
    unsigned long lastDhtReadTime = 0;
    uint8_t dhtFailureStreak = 0;
    // Consecutive good reads accumulated while confirmed-unavailable, before
    // recovery is trusted - see readDHT()'s symmetric recovery debounce.
    uint8_t dhtRecoveryStreak = 0;

    // Light exponential smoothing over accepted raw DHT22 readings - see
    // readDHT()'s own comment (real-hardware pre-integration Part C: the
    // DHT22's normal ~0.1-0.5C/~1-2%RH sample-to-sample noise was being
    // published completely raw, with no filtering at all). NaN means "no
    // filtered value yet"; reset (not blended) on a confirmed-unavailable ->
    // recovered transition so recovery never blends against a stale
    // pre-outage value.
    float dhtTemperatureFiltered = NAN;
    float dhtHumidityFiltered = NAN;

    // Throttle for readDHT()'s [DHT-RAW] diagnostic's VALID case - see
    // DHT_RAW_DIAGNOSTIC_INTERVAL_MS's own comment.
    unsigned long lastDhtRawDiagnosticAt = 0;

    // =====================================================
    // Sensor Reading Functions
    // =====================================================

    float measureDistanceCM();

    void readDHT();

    void readWaterTemperature();

    void readWaterLevel();

    void readEC();

    void readPH();

    void applyEffectiveSensors();

public:
    // Called by FirebaseManager when the authoritative remote setting is read,
    // so the next offline boot starts from the same source. Writes only on an
    // actual change.
    void persistSensorSource(bool mockEnabled);

    // A complete, validated mock payload was parsed during THIS session, so a
    // boot-restored mock source is confirmed live and stops waiting.
    void notifyMockPayloadReceived();

    // The cloud explicitly turned mock mode off; any boot wait is moot.
    void cancelMockBootWait();

    // True only for a boot-restored mock session that hasn't received its
    // first fresh payload yet (armed by resolveLocalSensorSource(), cleared
    // by notifyMockPayloadReceived()/cancelMockBootWait()/the boot-wait
    // timeout). Lets other modules (AlertManager) recognize this as an
    // intentional, transient initialization state rather than a real fault -
    // does not itself change what applyEffectiveSensors() publishes.
    bool isMockBootWaiting() const { return mockBootWaitingForPayload; }

    // True exactly when the most recent pH/EC stability-window evaluation
    // passed its tolerance check - see StabilityWindow::currentlyStable's own
    // comment. AutomationManager gates every NEW pH/EC correction start on
    // this (not just on sensors.ph/ec being non-NaN), so a retry after
    // dosing waits for a freshly reconfirmed reading rather than acting on
    // the pre-dose value Firebase/display are still (correctly) showing.
    //
    // pH also requires the temporal step filter above to have NO unconfirmed
    // jump in progress (phStepCandidate is NAN exactly when no confirmation
    // streak is active) - phStabilityWindow.currentlyStable alone is not
    // enough, since that window only ever sees candidates the step filter
    // has already trusted; while a jump is pending, the window keeps
    // reporting whatever it last decided (unchanged - it received no new
    // sample), so this composes in the step filter's own pending state
    // directly so a large unconfirmed jump can never be reported as
    // "currently stable enough to dose from".
    bool isPhCurrentlyStable() const { return phStabilityWindow.currentlyStable && isnan(phStepCandidate); }
    bool isEcCurrentlyStable() const { return ecStabilityWindow.currentlyStable; }

    // Quick-response refinement task: FAST TELEMETRY accessors, independent
    // of phStabilityWindow/isPhCurrentlyStable() above (the SLOW,
    // automation-trust side). hasPhTelemetry() is true once the temporal
    // step filter has ever confirmed a baseline - used by
    // FirebaseManager::writeSensors() as one of the two "fast sensor
    // settled" readiness signals (see Config.h's SENSOR_READY_MIN_MS/
    // SENSOR_READY_MAX_MS). isPhConfirming() is true while a jump is
    // pending (the same phStepCandidate state isPhCurrentlyStable() already
    // reads) - published so Android can show "pH is being confirmed"
    // instead of silently freezing on the old value with no explanation.
    bool hasPhTelemetry() const { return !isnan(lastAcceptedPhCandidate); }
    bool isPhConfirming() const { return !isnan(phStepCandidate); }

    // True while physical pH/EC readings are still within their deliberate
    // post-(re)connect analog settle window (PH_EC_ANALOG_SETTLE_TIME) -
    // sensors.ph/ec are intentionally held NaN this whole time (see
    // applyEffectiveSensors()), which for readiness purposes IS a known,
    // explicit "not yet available" state, not an unknown one - so pH/EC
    // readiness does not have to wait out the full ~20s settle window
    // before being considered observed, matching every other sensor's
    // valid-OR-explicitly-unavailable rule (sensorState.ready refinement,
    // see FirebaseManager::writeSensors()). Once settled, hasPhTelemetry()/
    // isEcStateKnown() below take over as the real observed signal.
    bool isPhEcAnalogSettling() const { return millis() - physicalPhEcSettledAt < PH_EC_ANALOG_SETTLE_TIME; }

    // sensorState.ready refinement: each of these is true once the given
    // Monitoring sensor's state for THIS acquisition session is KNOWN -
    // either a real reading has been produced, or
    // SENSOR_TRANSIENT_FAILURE_THRESHOLD consecutive failed reads have
    // confirmed it unavailable (the SAME debounce readDHT()/
    // readWaterTemperature()/readWaterLevel() already use to decide
    // dhtAvailable/dhtStale/NaN themselves - not a new, separate fault
    // concept). EC has no distinct failure path in the current firmware
    // (readEC() always produces a candidate once sampled), so its own
    // "known" state is simply having a first candidate, or the shared
    // pH/EC analog settle window above. None of these require a
    // genuinely failing sensor to become valid before readiness fires -
    // see FirebaseManager::writeSensors().
    bool isDhtStateKnown() const;
    bool isWaterTempStateKnown() const;
    bool isWaterLevelStateKnown() const;
    bool isEcStateKnown() const;
};

#endif
