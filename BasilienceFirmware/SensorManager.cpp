#include "SensorManager.h"

#include "Globals.h"
#include "Config.h"
#include "Calibration.h"

SensorManager::SensorManager()

    :

      dht(DHT_PIN, DHTTYPE),

      oneWire(WATER_TEMP_PIN),

      waterSensor(&oneWire),

      ecSampler(
          EC_PIN,
          EC_SAMPLE_COUNT,
          EC_SAMPLE_INTERVAL,
          AnalogSampler::RAW_ADC),

      phSampler(
          PH_SENSOR_PIN,
          PH_SAMPLE_COUNT,
          PH_SAMPLE_INTERVAL,
          AnalogSampler::MILLIVOLTS)

{
}

void SensorManager::begin()
{
    dht.begin();

    waterSensor.begin();

    Serial.print("[DS18B20] GPIO: ");
    Serial.println(WATER_TEMP_PIN);

    waterSensorDeviceCount = waterSensor.getDeviceCount();
    Serial.print("[DS18B20] Devices found: ");
    Serial.println(waterSensorDeviceCount);

    if (waterSensorDeviceCount > 0)
    {
        waterSensorAddressValid = waterSensor.getAddress(waterSensorAddress, 0);
        if (!waterSensorAddressValid)
        {
            Serial.println("[DS18B20] Device present but address could not be retrieved - falling back to index-based read");
        }
    }
    // A count of 0 here is not fatal - readWaterTemperature() re-enumerates
    // on its own throttled cadence and recovers automatically if the probe
    // wasn't settled yet at this point in boot (see its own comment).

    analogReadResolution(12);

    analogSetAttenuation(ADC_11db);

    pinMode(TRIG_PIN, OUTPUT);

    pinMode(ECHO_PIN, INPUT);

    ecSampler.begin();

    phSampler.begin();

    // SensorData's default waterLevel=0 is deliberately a valid real-world
    // reading (empty tank), unlike every other field here which defaults to
    // NaN - so it's the one field where "never sampled yet" and "genuinely
    // measured empty" are otherwise indistinguishable. Before readWaterLevel()
    // has ever completed a successful measurement (or reached its own
    // confirmed-unavailable threshold), physicalSensors.waterLevel must not
    // read as a plausible 0% to validWaterLevel()/isfinite() and unlock
    // refill/fog/dosing/cooling on a placeholder. This does not touch what a
    // REAL measured 0% means afterward - readWaterLevel()'s success path
    // unconditionally overwrites this with the actual computed percentage.
    physicalSensors.waterLevel = NAN;

    resolveLocalSensorSource();
}

// Decides the effective sensor source locally, at boot, without any network.
//
// Firebase used to be the only thing that could ever set sensorSourceResolved,
// so a unit that cold-booted with no Wi-Fi held every effective reading at NaN
// forever and local automation never engaged. The mock flag is now persisted
// in NVS, which means the same integrity guarantee (a previously-enabled mock
// session must not be silently replaced by physical readings after a reboot)
// can be honoured from local storage instead of from the cloud.
//
// PHYSICAL is the safe default: an unknown or never-written flag resolves to
// real sensors, never to simulated ones.
void SensorManager::resolveLocalSensorSource()
{
    bool mockEnabled = false;

    if (sourcePreferences.begin(SOURCE_NVS_NAMESPACE, true))
    {
        mockEnabled = sourcePreferences.getBool(SOURCE_NVS_KEY, false);
        sourcePreferences.end();
    }

    systemState.mockSensorsEnabled = mockEnabled;
    systemState.sensorSourceResolved = true;

    // Prime the change-detection used by applyEffectiveSensors() so the source
    // is announced exactly once here rather than again on the first update().
    sensorSourceReported = true;
    lastReportedMockSource = mockEnabled;

    if (mockEnabled)
    {
        // Mock readings are never persisted, so a mock session that survives a
        // reboot starts with no values, and physical readings must never
        // backfill mock mode. Rather than idle forever if the payload never
        // arrives, arm a bounded wait that reverts to physical sensors.
        mockBootWaitingForPayload = true;
        mockBootWaitStartedAt = millis();

        Serial.println("[AUTOMATION] Sensor source=MOCK (persisted)");
        Serial.println("[AUTOMATION] Waiting for fresh mock payload...");
    }
    else
    {
        physicalPhEcSettledAt = millis();

        Serial.println("[AUTOMATION] Sensor source=PHYSICAL (local)");
    }
}

void SensorManager::persistSensorSource(bool mockEnabled)
{
    if (!sourcePreferences.begin(SOURCE_NVS_NAMESPACE, false)) return;

    // Write only on a real change - this runs from the periodic mock read.
    if (sourcePreferences.getBool(SOURCE_NVS_KEY, false) != mockEnabled)
    {
        sourcePreferences.putBool(SOURCE_NVS_KEY, mockEnabled);
    }
    sourcePreferences.end();
}

void SensorManager::update()
{
    ecSampler.update();

    phSampler.update();

    readDHT();

    readWaterTemperature();

    readWaterLevel();

    readEC();

    readPH();

    applyEffectiveSensors();
}

// Bounded recovery for a boot-restored mock source. Runs every tick and is
// independent of connectivity, so it still fires with no Wi-Fi at all.
void SensorManager::updateMockBootWait()
{
    if (!mockBootWaitingForPayload) return;

    // Unsigned subtraction - safe across the millis() rollover.
    if (millis() - mockBootWaitStartedAt < MOCK_BOOT_PAYLOAD_TIMEOUT) return;

    mockBootWaitingForPayload = false;

    systemState.mockSensorsEnabled = false;
    physicalPhEcSettledAt = millis();
    persistSensorSource(false);

    // Drop the empty mock dataset so nothing stale can be read back if mock
    // mode is later re-enabled before a payload arrives.
    systemState.mockSensors = SensorData();
    systemState.mockSensors.waterLevel = NAN;

    Serial.println("[AUTOMATION] Mock payload timeout - reverting to PHYSICAL sensors");
}

void SensorManager::notifyMockPayloadReceived()
{
    if (!mockBootWaitingForPayload) return;

    mockBootWaitingForPayload = false;
    Serial.println("[AUTOMATION] Fresh mock payload received - remaining in MOCK mode");
}

void SensorManager::cancelMockBootWait()
{
    mockBootWaitingForPayload = false;
}

// sensorState.ready refinement - see the declarations' own comment in
// SensorManager.h. Read physicalSensors directly (not sensors/publishedSensors)
// since physical sampling runs unconditionally every tick regardless of mock
// mode, so a real hardware fault is still confirmed here even while mock data
// is what's actually being published.
bool SensorManager::isDhtStateKnown() const
{
    return physicalSensors.dhtAvailable || dhtFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD;
}

bool SensorManager::isWaterTempStateKnown() const
{
    return !isnan(physicalSensors.waterTemp) || waterTempFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD;
}

bool SensorManager::isWaterLevelStateKnown() const
{
    return isfinite(physicalSensors.waterLevelCm) || waterLevelFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD;
}

bool SensorManager::isEcStateKnown() const
{
    return isfinite(physicalSensors.ec) || isPhEcAnalogSettling();
}

// Feeds one new already-filtered pH/EC candidate (physicalSensors.ph/.ec)
// into its sliding stability window, rate-limited to one sample roughly
// every STABILITY_SAMPLE_INTERVAL_MS (see the constant's own comment for
// why - candidate values change every loop() tick but are heavily
// autocorrelated faster than that). Once the window holds
// STABILITY_SAMPLE_WINDOW samples, accepts a new window.lastStable/updates
// window.currentlyStable only when they all agree within `tolerance`;
// otherwise leaves lastStable in place (still what sensors.ph/ec serve) but
// clears currentlyStable (what gates a NEW correction - see
// AutomationManager::canStartNewPHCorrection()/canStartNewECCorrection()).
// Logs only on a genuine transition, never every sample.
void SensorManager::updateStabilityWindow(StabilityWindow& window, float candidate, float tolerance, const char* logTag, DebugCategory category)
{
    if (!isfinite(candidate)) return;

    // Serial Monitor Focus Mode: shared by pH and EC, so the caller supplies
    // which category this particular window belongs to - see
    // DebugManager::shouldPrintDebug()'s own comment. Purely a print gate;
    // every window/stability-state update below is unconditional.
    const bool dbg = debugManager.shouldPrintDebug(category);

    unsigned long now = millis();
    if (window.lastSampleAt != 0 && now - window.lastSampleAt < STABILITY_SAMPLE_INTERVAL_MS)
    {
        return;
    }
    window.lastSampleAt = now;

    window.samples[window.next] = candidate;
    window.next = (window.next + 1) % STABILITY_SAMPLE_WINDOW;
    if (window.count < STABILITY_SAMPLE_WINDOW) window.count++;

    if (window.count < STABILITY_SAMPLE_WINDOW)
    {
        return; // still filling the window for the first time
    }

    float lo = window.samples[0];
    float hi = window.samples[0];
    float sum = 0.0f;
    for (uint8_t i = 0; i < STABILITY_SAMPLE_WINDOW; i++)
    {
        const float s = window.samples[i];
        if (s < lo) lo = s;
        if (s > hi) hi = s;
        sum += s;
    }
    const float range = hi - lo;

    // Periodic evidence dump, independent of the transition-edge logs below
    // (which only fire once, on stable<->unstable changes) - see
    // StabilityWindow::lastDiagnosticAt's own comment. This is the only way
    // to see WHY a window stuck unstable stays unstable on real hardware.
    if (dbg && (window.lastDiagnosticAt == 0 ||
        now - window.lastDiagnosticAt >= STABILITY_DIAGNOSTIC_INTERVAL_MS))
    {
        window.lastDiagnosticAt = now;
        Serial.print(logTag);
        Serial.print(" candidate="); Serial.print(candidate, 3);
        Serial.print(" samples="); Serial.print(window.count);
        Serial.print(" min="); Serial.print(lo, 3);
        Serial.print(" max="); Serial.print(hi, 3);
        Serial.print(" range="); Serial.print(range, 3);
        Serial.print(" tolerance="); Serial.print(tolerance, 3);
        Serial.print(" currentlyStable="); Serial.print(window.currentlyStable ? "true" : "false");
        Serial.print(" hasStable="); Serial.print(window.hasStable ? "true" : "false");
        Serial.print(" lastStable=");
        if (window.hasStable) Serial.print(window.lastStable, 3);
        else Serial.print("none");
        Serial.print(" ageMs=");
        Serial.println(window.hasStable ? (now - window.lastStableAt) : 0UL);
    }

    if (range <= tolerance)
    {
        const float representative = sum / STABILITY_SAMPLE_WINDOW;
        const bool changed = !window.hasStable || fabsf(representative - window.lastStable) > 0.0001f;
        const bool wasUnstable = !window.currentlyStable;

        window.lastStable = representative;
        window.lastStableAt = now;
        window.hasStable = true;

        if (dbg && (changed || wasUnstable))
        {
            Serial.print(logTag);
            Serial.print(" accepted value=");
            Serial.print(representative, 2);
            Serial.print(" range=");
            Serial.println(range, 3);
        }
        if (dbg && wasUnstable)
        {
            Serial.print(logTag);
            Serial.print(" new stable value accepted=");
            Serial.print(representative, 2);
            Serial.println("; regulation resumed");
        }
        window.currentlyStable = true;
        window.staleLogged = false;
    }
    else
    {
        if (window.currentlyStable && dbg)
        {
            Serial.print(logTag);
            Serial.print(" unstable; keeping last=");
            if (window.hasStable) Serial.println(window.lastStable, 2);
            else Serial.println("none");

            Serial.print(logTag);
            Serial.println(" current window unstable; new correction paused");
        }
        window.currentlyStable = false;
    }
}

// Discards buffered (not-yet-confirmed) candidate samples so a resumed
// physical stream (after mock mode, or the post-reconnect settle window)
// requires a full fresh confirmation rather than evaluating stale
// pre-interruption samples mixed with new ones. lastStable/hasStable are
// also cleared: a reading from before the interruption is not something to
// keep serving as "current" once physical sensing resumes - see
// applyEffectiveSensors()'s call site for exactly when this fires.
void SensorManager::resetStabilityWindow(StabilityWindow& window)
{
    window.count = 0;
    window.next = 0;
    window.lastSampleAt = 0;
    window.lastStable = NAN;
    window.lastStableAt = 0;
    window.hasStable = false;
    window.currentlyStable = true;
    window.staleLogged = false;
}

void SensorManager::applyEffectiveSensors()
{
    // Evaluated before the source is selected below, so the tick that times
    // out already publishes physical readings rather than waiting one more.
    updateMockBootWait();

    // Mock mode's enabled/disabled state lives only in Firebase and is not
    // restored locally at boot (systemState.mockSensorsEnabled defaults to
    // false), so right after a reboot/brownout we don't yet know whether a
    // previously-enabled mock session should still own control. Until
    // FirebaseManager confirms the source at least once, hold every
    // effective reading explicitly invalid instead of defaulting to physical
    // - a plausible-looking physical reading at this point (e.g. an
    // unsettled water level) must never trigger automation/alerts.
    // Defensive only. resolveLocalSensorSource() resolves the source during
    // begin(), before the first update(), so this no longer gates a cold boot
    // on Firebase; it remains as a guard against any future path that clears
    // the flag.
    if (!systemState.sensorSourceResolved)
    {
        sensors = SensorData();
        sensors.waterLevel = NAN;

        if (!sensorSourceWaitingLogged)
        {
            Serial.println("[AUTOMATION] Sensor source=WAITING");
            sensorSourceWaitingLogged = true;
        }
        return;
    }

    // A persisted-MOCK boot has no mock readings of its own yet - mock values
    // are never persisted, so systemState.mockSensors is still the plain
    // default-constructed SensorData() at this point. That default is a
    // legitimate "no data" placeholder everywhere except waterLevel, which
    // defaults to 0 because 0 IS a valid real-sensor reading (empty tank -
    // see SensorData's own comment). Left alone here, that placeholder 0
    // reads as a genuine "tank empty" to every isfinite()-based safety check
    // (validWaterLevel() et al.), which is exactly what let automatic REFILL
    // fire the solenoid off boot-restored mock state before any payload had
    // ever arrived. Hold every effective field explicitly invalid - same
    // technique as the sensorSourceResolved guard above - until either a
    // fresh, validated payload arrives (notifyMockPayloadReceived(), which
    // FirebaseManager::readMockSensors() only calls after pH/EC and the rest
    // have all parsed successfully) or updateMockBootWait() above reverts to
    // PHYSICAL after its own timeout. Local safety shutdowns are untouched:
    // they command actuators OFF unconditionally and never gate on `sensors`
    // validity, so this only withholds permission to act, never the ability
    // to stand down. RTC/cycle restoration and grow-light scheduling are
    // unaffected too - grow light is driven purely by RTC time, not by any
    // field in `sensors`.
    if (systemState.mockSensorsEnabled && mockBootWaitingForPayload)
    {
        sensors = SensorData();
        sensors.waterLevel = NAN;

        if (!mockBootWaitHeldLogged)
        {
            Serial.println("[AUTOMATION] Mock boot wait - holding automation safe until fresh payload arrives");
            mockBootWaitHeldLogged = true;
        }
        return;
    }
    mockBootWaitHeldLogged = false;

    // Automation, alerts, safety, and Firebase publication all consume this one
    // effective dataset. Physical sampling remains active in physicalSensors
    // regardless of mock mode (Developer Sensor Test reads it directly), but
    // it must never backfill a field the mock command left unset. Mock mode
    // is meant to be a fully controlled simulated environment - a field the
    // mock payload didn't include stays invalid (NaN) rather than silently
    // reverting to a real, possibly noisy physical reading.
    if (systemState.mockSensorsEnabled)
    {
        sensors = systemState.mockSensors;
        sensors.tds = isnan(sensors.ec) ? NAN : sensors.ec * 500.0f;
        // Mock is a fully controlled dataset - a field the payload didn't
        // set stays NaN (same rule as every other mock field, see this
        // block's own comment below), and there is no "stale" concept for a
        // value the developer is directly supplying: it is either present
        // (available) or absent (unavailable), never a held-over reading.
        sensors.dhtAvailable = isfinite(sensors.temperature) && isfinite(sensors.humidity);
        sensors.dhtStale = false;
        // Same reasoning for the refill threshold confirmation (resilience
        // pass follow-up): mock is a fully controlled value, not a real
        // HC-SR04 stream to reacquire/confirm against, so it is trusted
        // directly rather than waiting on 3 mock ticks.
        sensors.refillStartConfirmed = isfinite(sensors.waterLevelCm) &&
            sensors.waterLevelCm <= systemState.refillStartLevelCm;
        sensors.refillStopConfirmed = isfinite(sensors.waterLevelCm) &&
            sensors.waterLevelCm >= systemState.refillStopLevelCm;
        // Same reasoning for phConfirming (quick-response refinement task) -
        // a mock pH value is never mid-confirmation, it is simply present
        // or absent (NaN), same as every other mock field.
        sensors.phConfirming = false;

        if (systemState.mockApplyPending)
        {
            Serial.println("[MOCK] Applied to effective firmware SensorData");
            Serial.print("[MOCK] Effective pH=");
            Serial.println(sensors.ph, 2);
            Serial.print("[MOCK] Effective EC=");
            Serial.println(sensors.ec, 2);
            systemState.mockApplyPending = false;
        }
    }
    else
    {
        // The cloud/local switch away from mock happens between one tick's
        // applyEffectiveSensors() call and the next - lastReportedMockSource
        // still holds last tick's value here, so this fires exactly once on
        // the transition, before it's overwritten below.
        if (lastReportedMockSource)
        {
            physicalPhEcSettledAt = millis();

            // A stability window accepted while sourcing mock/pre-interruption
            // data must not be trusted as "current" the instant physical
            // sensing resumes - see resetStabilityWindow()'s own comment.
            resetStabilityWindow(phStabilityWindow);
            resetStabilityWindow(ecStabilityWindow);

            // Same reasoning for the pH temporal step filter's own trusted
            // baseline (see lastAcceptedPhCandidate's own comment) - a
            // candidate accepted while sourcing mock data must not be
            // compared against as though it were the last real physical
            // reading; NAN makes the filter re-establish a fresh baseline
            // the same way it does at boot.
            lastAcceptedPhCandidate = NAN;
            lastAcceptedPhCandidateAt = 0;
            phTelemetryStaleLogged = false;
            phStepCandidate = NAN;
            phStepCandidateCount = 0;
            lastPhStepEvalAt = 0;

            // Coherent-snapshot readiness (see Types.h's
            // sensorSnapshotBaselineAt comment) - a source transition is a
            // genuine restart of the data stream, same as boot, so Android
            // should see /sensorState/stabilizing again rather than keep
            // treating an old physical-source snapshot as still current.
            systemState.sensorSnapshotBaselineAt = millis();
        }

        sensors = physicalSensors;

        // pH/EC probes need time to electrically settle after physical
        // sensors (re)become the active source - see PH_EC_ANALOG_SETTLE_TIME.
        // Held NaN rather than published: a real-looking but still-drifting
        // reading here would otherwise trip alerts and trigger dosing against
        // a value the probe hasn't finished producing yet. The stability
        // windows are deliberately not fed during this window either -
        // electrically-unsettled candidates are not worth accumulating.
        if (millis() - physicalPhEcSettledAt < PH_EC_ANALOG_SETTLE_TIME)
        {
            sensors.ph = NAN;
            sensors.ec = NAN;
            sensors.tds = NAN;
        }
        else
        {
            // sensors.ph/sensors.ec become authoritative ONLY here: the last
            // accepted stable value from a sliding window of already-filtered
            // candidates (physicalSensors.ph/.ec), never the instantaneous
            // reading directly - see updateStabilityWindow(). This is the one
            // dataset AutomationManager/AlertManager/SafetyManager/Firebase
            // publication all read; none of them see a raw candidate.
            //
            // A stale accepted value (no new confirmation within
            // PH_EC_STABLE_TIMEOUT_MS) is not kept forever - it reverts to
            // NaN, which the existing validPH()/validEC() SENSOR_FAULT path
            // (SafetyManager.cpp) already treats exactly like any other
            // invalid reading: abort/lock any in-progress correction, block a
            // new one from starting. hasStable/lastStable themselves are left
            // untouched by staleness (diagnostic history only), so this is a
            // read-time check, not a mutation.
            // pH validity/stability hardening: updateStabilityWindow() itself
            // only ever rejects a non-finite candidate (isfinite() check) -
            // it has no notion of what range is physically meaningful for
            // the sensor it's filtering, since it's shared with EC (a
            // different domain entirely, deliberately untouched here). A
            // physical pH candidate that is finite but outside 0.0-14.0
            // (e.g. from a probe reading near 0V/floating below its normal
            // operating range) would otherwise still be accepted into the
            // window and could become lastStable if it held steady for
            // STABILITY_SAMPLE_WINDOW samples - the confirmed root cause of
            // an observed lastStable=24.158. Rejected here (substituted with
            // NaN, which the window already correctly ignores) rather than
            // clamped, so an implausible reading is discarded, not silently
            // reinterpreted as 0 or 14. EC's own call is unmodified.
            const bool phDomainValid = isfinite(physicalSensors.ph) &&
                physicalSensors.ph >= 0.0f && physicalSensors.ph <= 14.0f;

            // pH temporal step filter (reservoir electrical-noise
            // hardening) - see Config.h's PH_STEP_* and
            // lastAcceptedPhCandidate's own comment for the full design.
            // Only a candidate this gate actually TRUSTS is ever offered to
            // phStabilityWindow below; that window is otherwise completely
            // untouched and still independently decides whether the
            // trusted stream itself is stable enough to become
            // authoritative FOR AUTOMATION (it does NOT replace the
            // window). lastAcceptedPhCandidate itself is the FAST TELEMETRY
            // value (quick-response refinement task) - published as
            // sensors.ph further below independent of whether the window
            // has converged. Throttled to PH_STEP_SAMPLE_INTERVAL_MS (not
            // STABILITY_SAMPLE_INTERVAL_MS - that slower cadence is now
            // only the window's own), so a "3 consecutive candidates"
            // streak means 3 genuinely distinct ~300ms-apart observations,
            // not 3 re-evaluations of one unchanged median within
            // milliseconds.
            const unsigned long nowForPhStep = millis();
            const bool phStepDue = lastPhStepEvalAt == 0 ||
                nowForPhStep - lastPhStepEvalAt >= PH_STEP_SAMPLE_INTERVAL_MS;

            if (phStepDue)
            {
                lastPhStepEvalAt = nowForPhStep;

                float phCandidateForWindow = NAN;
                const bool dbgPh = debugManager.shouldPrintDebug(DebugCategory::PH);

                if (phDomainValid)
                {
                    if (isnan(lastAcceptedPhCandidate))
                    {
                        // No trusted baseline yet (boot, or the filter has
                        // never confirmed a first reading) - require
                        // PH_STEP_CONFIRM_COUNT agreeing candidates before
                        // establishing one, exactly like a large jump below;
                        // never trust the very first reading blindly.
                        const bool agrees = !isnan(phStepCandidate) &&
                            fabsf(physicalSensors.ph - phStepCandidate) <= PH_STEP_CONFIRM_TOLERANCE;

                        if (agrees) phStepCandidateCount++;
                        else { phStepCandidate = physicalSensors.ph; phStepCandidateCount = 1; }

                        if (dbgPh)
                        {
                            Serial.print("[PH-FILTER] baseline candidate=");
                            Serial.print(physicalSensors.ph, 2);
                            Serial.print(" streak=");
                            Serial.print(phStepCandidateCount);
                            Serial.print("/");
                            Serial.println(PH_STEP_CONFIRM_COUNT);
                        }

                        if (phStepCandidateCount >= PH_STEP_CONFIRM_COUNT)
                        {
                            lastAcceptedPhCandidate = physicalSensors.ph;
                            lastAcceptedPhCandidateAt = nowForPhStep;
                            phStepCandidate = NAN;
                            phStepCandidateCount = 0;
                            phCandidateForWindow = lastAcceptedPhCandidate;

                            if (dbgPh)
                            {
                                Serial.print("[PH-FILTER] baseline established pH=");
                                Serial.println(lastAcceptedPhCandidate, 2);
                            }
                        }
                    }
                    else if (fabsf(physicalSensors.ph - lastAcceptedPhCandidate) <= PH_TELEMETRY_DEADBAND)
                    {
                        // Within the display deadband of the current
                        // trusted anchor - ordinary measurement noise.
                        // Ratcheting fix: this branch used to immediately
                        // move the anchor to physicalSensors.ph here (see
                        // PH_TELEMETRY_DEADBAND's own comment for why that
                        // let noise walk the anchor over many ticks) - it
                        // no longer touches lastAcceptedPhCandidate at all.
                        // Any in-progress confirmation streak is abandoned:
                        // the signal returned to the anchor, so whatever it
                        // was drifting toward is no longer worth confirming.
                        phStepCandidate = NAN;
                        phStepCandidateCount = 0;
                    }
                    else
                    {
                        // Beyond the deadband - held at the last trusted
                        // anchor (nothing new offered to the window this
                        // tick) until PH_STEP_CONFIRM_COUNT consecutive
                        // candidates mutually agree WITH EACH OTHER. The
                        // anchor itself is never touched until that streak
                        // completes below, so an intermediate observation
                        // can no longer become the new comparison baseline
                        // for the next tick (the ratcheting bug).
                        const bool agrees = !isnan(phStepCandidate) &&
                            fabsf(physicalSensors.ph - phStepCandidate) <= PH_STEP_CONFIRM_TOLERANCE;

                        if (agrees) phStepCandidateCount++;
                        else { phStepCandidate = physicalSensors.ph; phStepCandidateCount = 1; }

                        if (dbgPh)
                        {
                            Serial.print("[PH-FILTER] jump candidate=");
                            Serial.print(physicalSensors.ph, 2);
                            Serial.print(" from=");
                            Serial.print(lastAcceptedPhCandidate, 2);
                            Serial.print(" streak=");
                            Serial.print(phStepCandidateCount);
                            Serial.print("/");
                            Serial.println(PH_STEP_CONFIRM_COUNT);
                        }

                        if (phStepCandidateCount >= PH_STEP_CONFIRM_COUNT)
                        {
                            lastAcceptedPhCandidate = physicalSensors.ph;
                            lastAcceptedPhCandidateAt = nowForPhStep;
                            phStepCandidate = NAN;
                            phStepCandidateCount = 0;
                            phCandidateForWindow = lastAcceptedPhCandidate;

                            if (dbgPh)
                            {
                                Serial.print("[PH-FILTER] new level accepted pH=");
                                Serial.println(lastAcceptedPhCandidate, 2);
                            }
                        }
                    }
                }

                updateStabilityWindow(phStabilityWindow, phCandidateForWindow, PH_STABILITY_TOLERANCE, "[PH-STABLE]", DebugCategory::PH);
            }
            updateStabilityWindow(ecStabilityWindow, physicalSensors.ec, EC_STABILITY_TOLERANCE, "[EC-STABLE]", DebugCategory::EC);

            const unsigned long now = millis();
            const bool phStale = phStabilityWindow.hasStable &&
                (now - phStabilityWindow.lastStableAt > PH_EC_STABLE_TIMEOUT_MS);
            const bool ecStale = ecStabilityWindow.hasStable &&
                (now - ecStabilityWindow.lastStableAt > PH_EC_STABLE_TIMEOUT_MS);

            if (phStale && !phStabilityWindow.staleLogged)
            {
                if (debugManager.shouldPrintDebug(DebugCategory::PH))
                {
                    Serial.print("[PH-STABLE] stale; no confirmed reading for ");
                    Serial.print((now - phStabilityWindow.lastStableAt) / 1000UL);
                    Serial.println("s - marking unavailable");
                }
                phStabilityWindow.staleLogged = true;
            }
            if (ecStale && !ecStabilityWindow.staleLogged)
            {
                if (debugManager.shouldPrintDebug(DebugCategory::EC))
                {
                    Serial.print("[EC-STABLE] stale; no confirmed reading for ");
                    Serial.print((now - ecStabilityWindow.lastStableAt) / 1000UL);
                    Serial.println("s - marking unavailable");
                }
                ecStabilityWindow.staleLogged = true;
            }

            // Defense in depth (pH validity/stability hardening): even with
            // the entry-point range guard above, phStabilityWindow.lastStable
            // still gates AUTOMATION TRUST (isPhCurrentlyStable() ->
            // canStartNewPHCorrection()) via hasStable/currentlyStable - if
            // it is ever found outside the physically valid 0.0-14.0 pH
            // domain (should no longer be reachable via the entry guard, but
            // never trusted implicitly), the stability state is
            // invalidated/reset rather than exposed, same as a genuinely
            // disconnected/unstable probe.
            if (phStabilityWindow.hasStable &&
                (!isfinite(phStabilityWindow.lastStable) ||
                 phStabilityWindow.lastStable < 0.0f || phStabilityWindow.lastStable > 14.0f))
            {
                if (debugManager.shouldPrintDebug(DebugCategory::PH))
                {
                    Serial.print("[PH-STABLE] invalid lastStable=");
                    Serial.print(phStabilityWindow.lastStable, 3);
                    Serial.println(" outside 0.0-14.0 - rejecting and resetting stability state");
                }
                resetStabilityWindow(phStabilityWindow);
            }

            // Same defense in depth for the FAST TELEMETRY side (quick-
            // response refinement task) - lastAcceptedPhCandidate is what
            // sensors.ph is about to be published from below, independent
            // of the window above.
            if (!isnan(lastAcceptedPhCandidate) &&
                (!isfinite(lastAcceptedPhCandidate) ||
                 lastAcceptedPhCandidate < 0.0f || lastAcceptedPhCandidate > 14.0f))
            {
                if (debugManager.shouldPrintDebug(DebugCategory::PH))
                {
                    Serial.print("[PH-FILTER] invalid lastAcceptedPhCandidate=");
                    Serial.print(lastAcceptedPhCandidate, 3);
                    Serial.println(" outside 0.0-14.0 - rejecting and re-establishing baseline");
                }
                lastAcceptedPhCandidate = NAN;
                lastAcceptedPhCandidateAt = 0;
                phStepCandidate = NAN;
                phStepCandidateCount = 0;
            }

            // FAST TELEMETRY (quick-response refinement task): sensors.ph is
            // published from the temporal step filter's own trusted
            // candidate directly - NOT phStabilityWindow.lastStable - so
            // Firebase/Android see a confirmed pH within
            // PH_STEP_CONFIRM_COUNT x PH_STEP_SAMPLE_INTERVAL_MS
            // (~0.75-1.2s) of a genuine change, without waiting for the
            // slower 10-sample automation-trust window to also converge.
            // Still never fabricated: NaN until the filter has confirmed a
            // first baseline (see lastAcceptedPhCandidate's own comment),
            // and held at the LAST trusted value (not the new unconfirmed
            // one, not NaN) for the entire duration a jump is pending - see
            // phCandidateForWindow above, which is only ever set to a NEW
            // value once the filter itself accepts one. Its own freshness
            // clock (lastAcceptedPhCandidateAt) reuses the same
            // PH_EC_STABLE_TIMEOUT_MS bound the window uses, so a telemetry
            // value that stops reconfirming for that long still reverts to
            // NaN rather than displaying an arbitrarily old reading as
            // current - the same safety property the window's own staleness
            // check already provided.
            const bool phTelemetryStale = !isnan(lastAcceptedPhCandidate) &&
                (now - lastAcceptedPhCandidateAt > PH_EC_STABLE_TIMEOUT_MS);
            if (phTelemetryStale && !phTelemetryStaleLogged)
            {
                if (debugManager.shouldPrintDebug(DebugCategory::PH))
                {
                    Serial.print("[PH-FILTER] telemetry stale; no confirmed reading for ");
                    Serial.print((now - lastAcceptedPhCandidateAt) / 1000UL);
                    Serial.println("s - marking unavailable");
                }
                phTelemetryStaleLogged = true;
            }
            else if (!phTelemetryStale)
            {
                phTelemetryStaleLogged = false;
            }

            sensors.ph = (!isnan(lastAcceptedPhCandidate) && !phTelemetryStale)
                ? lastAcceptedPhCandidate : NAN;
            // Published alongside sensors.ph (quick-response refinement
            // task) - see Types.h's own comment. Reflects the SAME
            // phStepCandidate state isPhCurrentlyStable() already reads for
            // the automation-trust side, just exposed for display too.
            sensors.phConfirming = !isnan(phStepCandidate);
            sensors.ec = (ecStabilityWindow.hasStable && !ecStale) ? ecStabilityWindow.lastStable : NAN;
            // TDS is unaffected by pH/EC stability gating - it already has no
            // water-temperature dependency (kept from the prior pass) and is
            // published from physicalSensors.tds via the `sensors =
            // physicalSensors` copy above; stabilizing it was not requested.
        }

        if (systemState.mockApplyPending)
        {
            Serial.println("[MOCK] Mock sensor mode DISABLED");
            Serial.println("[MOCK] Physical sensors restored as automation source");
            Serial.print("[PHYSICAL] pH="); Serial.println(sensors.ph, 2);
            Serial.print("[PHYSICAL] EC="); Serial.println(sensors.ec, 2);
            Serial.print("[PHYSICAL] AirTemp="); Serial.println(sensors.temperature, 2);
            Serial.print("[PHYSICAL] Humidity="); Serial.println(sensors.humidity, 2);
            Serial.print("[PHYSICAL] WaterTemp="); Serial.println(sensors.waterTemp, 2);
            Serial.print("[PHYSICAL] WaterLevel="); Serial.println(sensors.waterLevel, 2);
            systemState.mockApplyPending = false;
        }
    }

    if (!sensorSourceReported || lastReportedMockSource != systemState.mockSensorsEnabled)
    {
        Serial.print("[AUTOMATION] Sensor source=");
        Serial.println(systemState.mockSensorsEnabled ? "MOCK" : "PHYSICAL");
        sensorSourceReported = true;
        lastReportedMockSource = systemState.mockSensorsEnabled;
    }
}

float SensorManager::measureDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);

    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);

    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    long duration =
        pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0)
        return -1;

    return duration * 0.0343f / 2.0f;
}

void SensorManager::readDHT()
{
    // The DHT22 needs real settling time between samples; re-reading on every
    // loop iteration is far faster than the sensor can actually answer and is
    // a common cause of spurious checksum/timeout failures unrelated to the
    // sensor or wiring actually failing. Not due yet simply means the last
    // values are kept - they must never be invalidated merely because a new
    // read isn't scheduled.
    if (millis() - lastDhtReadTime < DHT_READ_INTERVAL_MS)
    {
        return;
    }
    lastDhtReadTime = millis();

    float humidity =
        dht.readHumidity();

    float temperature =
        dht.readTemperature();

    // Raw SENSOR VALIDITY (the physical DHT22 measurement range) - never an
    // agronomic/automation threshold; 28C or 10C are both physically valid
    // readings whatever the cultivation target is. Confirmed bug this
    // fixes: the previous check here was isnan()-only, so an
    // impossible-but-non-NaN raw sample (587.96C, 1734.28% observed on real
    // hardware) still counted as a good read and was fed straight into
    // dhtTemperatureFiltered/dhtHumidityFiltered below, producing exactly
    // the decaying-toward-reality EMA pattern (80 -> 64 -> 52 -> 44...)
    // reported from the bench. Paired acquisition (see this task's own
    // "Important Pair Semantics"): temperature and humidity are one DHT22
    // reading, so either channel out of range invalidates the whole cycle,
    // exactly like the isnan()-only check already treated them as one unit.
    const bool temperatureFinite = isfinite(temperature);
    const bool humidityFinite = isfinite(humidity);
    const bool temperatureInRange = temperatureFinite &&
        temperature >= DHT22_MIN_TEMP_C && temperature <= DHT22_MAX_TEMP_C;
    const bool humidityInRange = humidityFinite &&
        humidity >= DHT22_MIN_HUMIDITY_PCT && humidity <= DHT22_MAX_HUMIDITY_PCT;
    const bool readOk = temperatureInRange && humidityInRange;

    // Serial Monitor Focus Mode: DHT diagnostics are shown for CANOPY and
    // FOGGING (DHT-dependent/optional-cadence controllers), suppressed
    // everywhere else - see DebugManager::shouldPrintDebug()'s own comment.
    // Purely a print gate; every failure/recovery streak and physicalSensors
    // update below is unconditional.
    const bool dbgDht = debugManager.shouldPrintDebug(DebugCategory::DHT);

    // [DHT-RAW] diagnostic: an invalid sample is always printed immediately
    // (already naturally rate-limited to once per DHT_READ_INTERVAL_MS, and
    // each one is exactly the evidence this exists to capture); a valid
    // sample is throttled so this cannot spam every successful read.
    if (!readOk)
    {
        if (dbgDht)
        {
            const char* reason = (!temperatureFinite || !humidityFinite)
                ? "non_finite"
                : (!temperatureInRange ? "temperature_out_of_range" : "humidity_out_of_range");
            Serial.print("[DHT-RAW] temp=");
            Serial.print(temperature, 2);
            Serial.print(" humidity=");
            Serial.print(humidity, 2);
            Serial.print(" valid=false reason=");
            Serial.println(reason);
        }
    }
    else if (dbgDht)
    {
        const unsigned long nowForDiag = millis();
        if (lastDhtRawDiagnosticAt == 0 ||
            nowForDiag - lastDhtRawDiagnosticAt >= DHT_RAW_DIAGNOSTIC_INTERVAL_MS)
        {
            lastDhtRawDiagnosticAt = nowForDiag;
            Serial.print("[DHT-RAW] temp=");
            Serial.print(temperature, 2);
            Serial.print(" humidity=");
            Serial.print(humidity, 2);
            Serial.println(" valid=true");
        }
    }

    const bool confirmedUnavailable = dhtFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD;

    if (!readOk)
    {
        // Any failed read cancels a recovery attempt in progress - a
        // marginal sensor must complete a clean run of good reads to be
        // trusted again, not just outnumber its bad ones.
        dhtRecoveryStreak = 0;

        if (dhtFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            dhtFailureStreak++;

            if (dhtFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
            {
                if (dbgDht)
                {
                    Serial.print("[DHT22] transient failure ");
                    Serial.print(dhtFailureStreak);
                    Serial.print("/");
                    Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
                }
            }
            else
            {
                // Automation resilience pass: confirmed-unavailable no
                // longer discards physicalSensors.temperature/humidity - the
                // reservoir electrical environment makes this a recurring,
                // expected state, and NaN-ing out an otherwise-good last
                // reading needlessly threw away display continuity and (via
                // canFog()/canopy) blocked unrelated automation. The reading
                // is held at its last accepted value; dhtAvailable/dhtStale
                // are the explicit signal that it is no longer fresh -
                // isfinite(temperature) alone no longer means "current".
                physicalSensors.dhtAvailable = false;
                physicalSensors.dhtStale = isfinite(physicalSensors.temperature);
                // Reset (not blended) so a later recovery starts fresh
                // rather than smoothing its first accepted reading against
                // a stale pre-outage value - see the members' own comment.
                dhtHumidityFiltered = NAN;
                dhtTemperatureFiltered = NAN;

                if (dbgDht)
                {
                    Serial.print("[DHT] unavailable; retaining lastGood temp=");
                    Serial.print(physicalSensors.temperature, 1);
                    Serial.print(" humidity=");
                    Serial.print(physicalSensors.humidity, 1);
                    Serial.print(" stale=");
                    Serial.println(physicalSensors.dhtStale ? "true" : "false");
                }
            }
        }
        // Already confirmed unavailable - stays NaN, no repeated logging.
        return;
    }

    if (!confirmedUnavailable)
    {
        // Not yet confirmed unavailable (0 to THRESHOLD-1 consecutive
        // failures) - a good read simply resumes normal operation right
        // away, same as before the recovery debounce below existed.
        dhtFailureStreak = 0;

        // Exponential smoothing over the raw reading - see
        // dhtTemperatureFiltered/dhtHumidityFiltered's own comment. isnan()
        // on the first accepted sample (or the first after a reset above)
        // seeds the filter directly rather than blending against NaN.
        dhtHumidityFiltered = isnan(dhtHumidityFiltered)
            ? humidity
            : dhtHumidityFiltered + DHT_SMOOTHING_ALPHA * (humidity - dhtHumidityFiltered);
        dhtTemperatureFiltered = isnan(dhtTemperatureFiltered)
            ? temperature
            : dhtTemperatureFiltered + DHT_SMOOTHING_ALPHA * (temperature - dhtTemperatureFiltered);

        physicalSensors.humidity = dhtHumidityFiltered;
        physicalSensors.temperature = dhtTemperatureFiltered;
        physicalSensors.dhtAvailable = true;
        physicalSensors.dhtStale = false;
        return;
    }

    // Confirmed unavailable: require a sustained run of good reads, not
    // just one, before trusting the sensor again and clearing sensorFault.
    // A marginal/flickering DHT22 was recovering on a single isolated good
    // read and immediately failing again, flipping alertState.sensorFault
    // false/true on every cycle and re-firing the Sensor Fault push
    // notification each time - this makes recovery symmetric with the
    // existing failure debounce above instead of instant.
    dhtRecoveryStreak++;

    if (dhtRecoveryStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
    {
        if (dbgDht)
        {
            Serial.print("[DHT22] recovery ");
            Serial.print(dhtRecoveryStreak);
            Serial.print("/");
            Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
        }
        return;
    }

    if (dbgDht)
    {
        Serial.println("[DHT22] recovered");
    }
    dhtFailureStreak = 0;
    dhtRecoveryStreak = 0;

    // Filter was reset to NaN on the earlier confirmed-unavailable
    // transition, so this seeds it fresh from the just-recovered reading
    // rather than blending against a stale pre-outage value.
    dhtHumidityFiltered = humidity;
    dhtTemperatureFiltered = temperature;

    physicalSensors.humidity =
        dhtHumidityFiltered;

    physicalSensors.temperature =
        dhtTemperatureFiltered;

    physicalSensors.dhtAvailable = true;
    physicalSensors.dhtStale = false;
}

void SensorManager::readWaterTemperature()
{
    // The DS18B20 conversion is a blocking OneWire transaction; running it
    // every loop iteration both stalls loop() and increases how often it can
    // collide with other blocking work (e.g. Firebase calls). Not due yet
    // simply means physicalSensors.waterTemp keeps its last value - it must
    // never be invalidated merely because a new read isn't scheduled.
    if (millis() - lastWaterTempReadTime < WATER_TEMP_READ_INTERVAL_MS)
    {
        return;
    }
    lastWaterTempReadTime = millis();

    // Serial Monitor Focus Mode: DS18B20/water-temperature diagnostics are
    // the COOLING controller's own sensor input - see
    // DebugManager::shouldPrintDebug()'s own comment. Purely a print gate;
    // every failure/recovery streak and physicalSensors update below is
    // unconditional.
    const bool dbgCooling = debugManager.shouldPrintDebug(DebugCategory::COOLING);

    if (waterSensorDeviceCount == 0)
    {
        // Re-enumerate on the same throttled cadence as the read itself - no
        // new timer, no delay(). Recovers automatically if the probe wasn't
        // settled/responding yet at begin() (e.g. long cable run, power-up
        // settling) and starts answering later.
        waterSensor.begin();
        waterSensorDeviceCount = waterSensor.getDeviceCount();

        if (waterSensorDeviceCount > 0)
        {
            if (dbgCooling)
            {
                Serial.print("[DS18B20] Device found on retry - count: ");
                Serial.println(waterSensorDeviceCount);
            }
            waterSensorAddressValid = waterSensor.getAddress(waterSensorAddress, 0);
        }
    }

    float temp = NAN;
    if (waterSensorDeviceCount > 0)
    {
        waterSensor.requestTemperatures();
        temp = waterSensorAddressValid
            ? waterSensor.getTempC(waterSensorAddress)
            : waterSensor.getTempCByIndex(0);
    }

    // DS18B20 commonly returns exactly 85.00C as a power-on/default
    // conversion result rather than a real reading (its scratchpad reset
    // value) - a small float tolerance avoids an unsafe exact comparison
    // while still only catching that specific default, not a genuine
    // ~85C reading (implausible for a hydroponic reservoir regardless).
    const bool powerOnDefault = fabsf(temp - 85.0f) < 0.01f;

    if (dbgCooling)
    {
        if (waterSensorDeviceCount == 0)
        {
            Serial.println("[DS18B20] Raw: no device enumerated");
        }
        else if (temp == DEVICE_DISCONNECTED_C)
        {
            Serial.println("[DS18B20] Raw: -127.00 C (DEVICE_DISCONNECTED_C)");
        }
        else if (powerOnDefault)
        {
            Serial.print("[DS18B20] Raw: "); Serial.print(temp, 2); Serial.println(" C (power-on/default)");
        }
        else if (!isfinite(temp))
        {
            Serial.println("[DS18B20] Raw: invalid (non-finite)");
        }
        else
        {
            Serial.print("[DS18B20] Raw: "); Serial.print(temp, 2); Serial.println(" C");
        }
    }

    const bool invalid =
        waterSensorDeviceCount == 0 ||
        temp == DEVICE_DISCONNECTED_C ||
        !isfinite(temp) ||
        powerOnDefault;

    if (invalid)
    {
        if (waterTempFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            waterTempFailureStreak++;

            if (waterTempFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
            {
                if (dbgCooling)
                {
                    Serial.print("[DS18B20] transient failure ");
                    Serial.print(waterTempFailureStreak);
                    Serial.print("/");
                    Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
                }
            }
            else
            {
                if (dbgCooling)
                {
                    Serial.println("[DS18B20] confirmed unavailable");
                }
                physicalSensors.waterTemp = NAN;
                // Reset (not blended) so a later recovery starts fresh
                // rather than smoothing its first accepted reading against
                // a stale pre-outage value - see waterTempFiltered's own
                // comment. A physically disconnected probe still reports
                // NaN immediately above; this only affects the filtered
                // value fed back in once real readings resume.
                waterTempFiltered = NAN;
            }
        }
        // Already confirmed unavailable - stays NaN, no repeated logging.
        return;
    }

    if (waterTempFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD && dbgCooling)
    {
        Serial.print("[DS18B20] recovered: ");
        Serial.print(temp, 2);
        Serial.println(" C");
    }

    waterTempFailureStreak = 0;

    // Exponential smoothing over the raw reading - see waterTempFiltered's
    // own comment. isnan() on the first accepted sample (or the first after
    // a reset above) seeds the filter directly rather than blending
    // against NaN.
    waterTempFiltered = isnan(waterTempFiltered)
        ? temp
        : waterTempFiltered + WATER_TEMP_SMOOTHING_ALPHA * (temp - waterTempFiltered);

    lastValidWaterTemp = waterTempFiltered;
    physicalSensors.waterTemp = waterTempFiltered;
}

void SensorManager::readWaterLevel()
{
    // The fogger and the water-level sensor share the same reservoir, and
    // the fogger's mist and surface turbulence are a real source of bad
    // ultrasonic echoes, not just ordinary sensor noise the median/step
    // filters below are built to reject. Rather than let a disturbed
    // reading fight its way through those filters, skip sampling entirely
    // while the fogger is on and hold the last accepted values - the
    // interval timer is deliberately not advanced here either, so the very
    // first tick after the fogger turns off takes a fresh reading
    // immediately rather than waiting out the rest of WATER_LEVEL_READ_
    // INTERVAL_MS.
    if (actuatorManager.isOn(FOGGER))
    {
        physicalSensors.waterLevelHeldForFogger = true;
        return;
    }
    physicalSensors.waterLevelHeldForFogger = false;

    // The HC-SR04 trigger/echo cycle needs real settling time; re-triggering
    // on every loop iteration is a common cause of spurious pulseIn()
    // timeouts unrelated to the sensor or wiring actually failing. Not due
    // yet simply means the last values are kept - they must never be
    // invalidated merely because a new read isn't scheduled.
    if (millis() - lastWaterLevelReadTime < WATER_LEVEL_READ_INTERVAL_MS)
    {
        return;
    }
    lastWaterLevelReadTime = millis();

    // Serial Monitor Focus Mode: water/refill diagnostics - see
    // DebugManager::shouldPrintDebug()'s own comment (visible for REFILL and
    // STARTUP). Purely a print gate; every failure streak, accepted-value,
    // and refill-confirmation update below is unconditional.
    const bool dbgWater = debugManager.shouldPrintDebug(DebugCategory::WATER);

    float distance =
        measureDistanceCM();

    if (distance < 0 || !isfinite(distance))
    {
        if (waterLevelFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            waterLevelFailureStreak++;

            if (waterLevelFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
            {
                if (dbgWater)
                {
                    Serial.print("[SENSOR] Water level transient read failure ");
                    Serial.print(waterLevelFailureStreak);
                    Serial.print("/");
                    Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
                }
            }
            else
            {
                if (dbgWater)
                {
                    Serial.println("[SENSOR] Water level confirmed unavailable");
                }
                physicalSensors.waterLevel = NAN;
                physicalSensors.waterLevelCm = NAN;
                physicalSensors.waterVolumeLiters = NAN;
                physicalSensors.waterLevelDistanceCm = NAN;
                physicalSensors.refillStartConfirmed = false;
                physicalSensors.refillStopConfirmed = false;
                waterLevelHistoryCount = 0;
                // A genuine sensor outage (timeout/negative pulseIn) is
                // category A - invalid/unavailable - not the plausible-but-
                // false echo category B the step filter exists for. Reset the
                // accepted baseline so recovery must go through the same
                // reacquisition sequence as boot (see readWaterLevel()'s own
                // comment on the resilience follow-up pass) rather than
                // trusting the first post-outage reading immediately.
                lastAcceptedWaterDepthCm = NAN;
                waterLevelStepCandidateCm = NAN;
                waterLevelStepCandidateCount = 0;
                // Nothing is confirmed while the sensor is unavailable.
                refillStartConfirmCount = 0;
                refillStopConfirmCount = 0;
            }
        }
        // Already confirmed unavailable - stays NaN, no repeated logging.
        return;
    }

    if (waterLevelFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD && dbgWater)
    {
        Serial.println("[SENSOR] Water level recovered");
    }
    waterLevelFailureStreak = 0;

    waterLevelDistanceHistory[waterLevelHistoryIndex] = distance;
    waterLevelHistoryIndex = (waterLevelHistoryIndex + 1) % 5;
    if (waterLevelHistoryCount < 5)
    {
        waterLevelHistoryCount++;
    }

    float filteredDistance = distance;
    if (waterLevelHistoryCount == 5)
    {
        // True median of 5 (not an average - see the member's own comment
        // on why smoothing/averaging was deliberately avoided). A small
        // insertion sort on a local copy; the sum-minus-min-minus-max trick
        // the previous 3-sample version used only works for exactly 3
        // values.
        float sorted[5];
        for (uint8_t i = 0; i < 5; i++) sorted[i] = waterLevelDistanceHistory[i];
        for (uint8_t i = 1; i < 5; i++)
        {
            const float key = sorted[i];
            int8_t j = i - 1;
            while (j >= 0 && sorted[j] > key)
            {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }
        filteredDistance = sorted[2];
    }

    physicalSensors.waterLevelDistanceCm = filteredDistance;

    // Water-depth model (see Config.h's "Water Reservoir Geometry" section
    // for the full design and the authoritative depth/percent/liters
    // reference table). sensorToBottomCm is the one remaining configurable
    // calibration input - the sensor-to-reservoir-bottom distance, which
    // genuinely varies with mounting height per installation - not a fixed
    // constant like the reservoir's own dimensions below it. Deliberately
    // NOT the legacy waterLevelEmptyDistanceCm field - see the automation
    // resilience pass report and systemState.sensorToBottomCm's own comment
    // in Types.h for why that field can carry a stale persisted value.
    const float sensorToBottomCm = systemState.sensorToBottomCm;

    // Only the LOWER bound is clamped here: a negative depth is a sensor/
    // geometry artifact, never physically real. The upper bound is
    // deliberately NOT clamped to MAX_WORKING_WATER_CM - a genuine overfill
    // (depth above the normal working level) must still report its real
    // measured depth, e.g. 7.0cm stays 7.0cm; see the static automation
    // integration audit. Only the DERIVED working percentage below clamps
    // at 100%. This is the CANDIDATE depth for the step-confirmation filter
    // below - not yet the accepted control value.
    float candidateDepthCm = sensorToBottomCm - filteredDistance;
    if (candidateDepthCm < 0.0f)
    {
        candidateDepthCm = 0.0f;
    }

    // Temporal plausibility filter (see Config.h's WATER_LEVEL_STEP_* and
    // this task's automation resilience pass report). The median-of-5 above
    // already rejects a single outlier echo, but a short RUN of consecutive
    // bad echoes can shift the median itself - the observed
    // 4.03->1.70->4.03cm pattern with no real water movement. A candidate
    // within WATER_LEVEL_STEP_ACCEPT_CM of the last accepted depth is normal
    // sensor noise/real gradual change and is accepted immediately. A larger
    // jump is held as a pending "step candidate" and the previous accepted
    // depth is kept as the control value until WATER_LEVEL_STEP_CONFIRM_COUNT
    // consecutive candidates mutually agree within
    // WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM - only then does the new level
    // become authoritative. A lone false echo never accumulates enough
    // agreeing candidates and is permanently rejected; a genuine drain/fill
    // still confirms within a few ~300ms read cycles.
    if (isnan(lastAcceptedWaterDepthCm))
    {
        // Reacquisition after boot or a confirmed sensor outage (resilience
        // pass follow-up): there is no accepted baseline to compare a small
        // change against, but the very first post-outage candidate must
        // still not be trusted blindly - it can just as easily be a single
        // bad echo as any other reading. Reuse the same step-candidate
        // agreement mechanism as a large jump below, gated on "no baseline
        // yet" instead of "large delta from baseline": require
        // WATER_LEVEL_STEP_CONFIRM_COUNT consecutive candidates that
        // mutually agree within WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM before
        // establishing a new baseline. Until then, physicalSensors.
        // waterLevelCm/waterLevel/waterVolumeLiters are left untouched
        // (still NaN from the outage, or NaN from boot) - never fabricated
        // from an unconfirmed candidate, and never fabricated as 0cm - and
        // the refill threshold counters below are never reached, so a
        // refill can neither start nor complete from reacquisition data.
        const bool agreesWithPending = !isnan(waterLevelStepCandidateCm) &&
            fabsf(candidateDepthCm - waterLevelStepCandidateCm) <= WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM;

        if (agreesWithPending)
        {
            waterLevelStepCandidateCount++;
        }
        else
        {
            waterLevelStepCandidateCm = candidateDepthCm;
            waterLevelStepCandidateCount = 1;
        }

        if (dbgWater)
        {
            Serial.print("[WATER-FILTER] reacquiring candidate=");
            Serial.print(candidateDepthCm, 2);
            Serial.print(" streak=");
            Serial.print(waterLevelStepCandidateCount);
            Serial.print("/");
            Serial.println(WATER_LEVEL_STEP_CONFIRM_COUNT);
        }

        if (waterLevelStepCandidateCount >= WATER_LEVEL_STEP_CONFIRM_COUNT)
        {
            lastAcceptedWaterDepthCm = candidateDepthCm;
            waterLevelStepCandidateCm = NAN;
            waterLevelStepCandidateCount = 0;

            physicalSensors.waterLevelCm = lastAcceptedWaterDepthCm;
            physicalSensors.waterLevel =
                constrain((lastAcceptedWaterDepthCm / MAX_WORKING_WATER_CM) * 100.0f, 0.0f, 100.0f);
            physicalSensors.waterVolumeLiters =
                lastAcceptedWaterDepthCm * RESERVOIR_LENGTH_CM * RESERVOIR_WIDTH_CM / 1000.0f;

            if (dbgWater)
            {
                Serial.print("[WATER-FILTER] baseline established depth=");
                Serial.println(lastAcceptedWaterDepthCm, 2);
            }
        }
        return;
    }

    float acceptedDepthCm;

    if (fabsf(candidateDepthCm - lastAcceptedWaterDepthCm) <= WATER_LEVEL_STEP_ACCEPT_CM)
    {
        acceptedDepthCm = candidateDepthCm;
        waterLevelStepCandidateCm = NAN;
        waterLevelStepCandidateCount = 0;
    }
    else
    {
        acceptedDepthCm = lastAcceptedWaterDepthCm;

        const bool agreesWithPending = !isnan(waterLevelStepCandidateCm) &&
            fabsf(candidateDepthCm - waterLevelStepCandidateCm) <= WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM;

        if (agreesWithPending)
        {
            waterLevelStepCandidateCount++;
        }
        else
        {
            waterLevelStepCandidateCm = candidateDepthCm;
            waterLevelStepCandidateCount = 1;
        }

        if (dbgWater)
        {
            Serial.print("[WATER-FILTER] rawDepth=");
            Serial.print(candidateDepthCm, 2);
            Serial.print(" accepted=");
            Serial.print(lastAcceptedWaterDepthCm, 2);
            Serial.print(" delta=");
            Serial.print(fabsf(candidateDepthCm - lastAcceptedWaterDepthCm), 2);
            Serial.print(" candidate=");
            Serial.print(waterLevelStepCandidateCount);
            Serial.print("/");
            Serial.print(WATER_LEVEL_STEP_CONFIRM_COUNT);
        }

        if (waterLevelStepCandidateCount >= WATER_LEVEL_STEP_CONFIRM_COUNT)
        {
            acceptedDepthCm = candidateDepthCm;
            waterLevelStepCandidateCm = NAN;
            waterLevelStepCandidateCount = 0;
            if (dbgWater)
            {
                Serial.print(" -> ACCEPT ");
                Serial.println(acceptedDepthCm, 2);
            }
        }
        else if (dbgWater)
        {
            Serial.println(" -> REJECT/HOLD");
        }
    }

    lastAcceptedWaterDepthCm = acceptedDepthCm;

    physicalSensors.waterLevelCm = acceptedDepthCm;

    physicalSensors.waterLevel =
        constrain((acceptedDepthCm / MAX_WORKING_WATER_CM) * 100.0f, 0.0f, 100.0f);

    physicalSensors.waterVolumeLiters =
        acceptedDepthCm * RESERVOIR_LENGTH_CM * RESERVOIR_WIDTH_CM / 1000.0f;

    // Refill threshold confirmation (resilience pass follow-up): the step
    // filter above already protects against a LARGE jump, but a small
    // transient within WATER_LEVEL_STEP_ACCEPT_CM (e.g. one bad reading
    // 0.20cm off) still becomes the accepted value immediately and can, on
    // its own, momentarily cross REFILL_START_CM/REFILL_STOP_CM. Counted
    // once per ACCEPTED reading here (~300ms cadence), not once per loop()
    // tick, so 3 consecutive counts genuinely means 3 distinct HC-SR04
    // reads agreeing, not 3 fast re-evaluations of one unchanged value.
    if (acceptedDepthCm <= systemState.refillStartLevelCm)
    {
        if (refillStartConfirmCount < WATER_LEVEL_STEP_CONFIRM_COUNT) refillStartConfirmCount++;
    }
    else
    {
        refillStartConfirmCount = 0;
    }

    if (acceptedDepthCm >= systemState.refillStopLevelCm)
    {
        if (refillStopConfirmCount < WATER_LEVEL_STEP_CONFIRM_COUNT) refillStopConfirmCount++;
    }
    else
    {
        refillStopConfirmCount = 0;
    }

    physicalSensors.refillStartConfirmed = refillStartConfirmCount >= WATER_LEVEL_STEP_CONFIRM_COUNT;
    physicalSensors.refillStopConfirmed = refillStopConfirmCount >= WATER_LEVEL_STEP_CONFIRM_COUNT;

    // Edge-triggered - see refillStartConfirmLoggedCount's own comment.
    // Prints only on a genuine count change (0->1->2->3), never repeated
    // while already confirmed at 3/3; the underlying counters/confirmed
    // flags above are unaffected either way. Reset bookkeeping runs
    // unconditionally (cheap state, not itself a print) so it stays correct
    // even if focus filtering toggles mid-episode.
    if (refillStartConfirmCount == 0)
    {
        refillStartConfirmLoggedCount = 0;
    }
    else if (refillStartConfirmCount != refillStartConfirmLoggedCount)
    {
        refillStartConfirmLoggedCount = refillStartConfirmCount;
        if (dbgWater)
        {
            Serial.print("[REFILL] start confirmation ");
            Serial.print(refillStartConfirmCount);
            Serial.print("/");
            Serial.println(WATER_LEVEL_STEP_CONFIRM_COUNT);
        }
    }

    if (refillStopConfirmCount == 0)
    {
        refillStopConfirmLoggedCount = 0;
    }
    else if (refillStopConfirmCount != refillStopConfirmLoggedCount)
    {
        refillStopConfirmLoggedCount = refillStopConfirmCount;
        if (dbgWater)
        {
            Serial.print("[REFILL] stop confirmation ");
            Serial.print(refillStopConfirmCount);
            Serial.print("/");
            Serial.println(WATER_LEVEL_STEP_CONFIRM_COUNT);
        }
    }

    // Throttled diagnostic - readWaterLevel() itself runs every 300ms
    // (WATER_LEVEL_READ_INTERVAL_MS), far more often than this needs to
    // print. See DHT_RAW_DIAGNOSTIC_INTERVAL_MS for the same pattern. Only
    // the accepted-value summary is throttled - the [WATER-FILTER] reject/
    // accept lines above already only print on a large-jump candidate tick,
    // which is inherently rare, so they are never subject to this throttle.
    const unsigned long nowForWaterDiag = millis();
    if (dbgWater && (lastWaterLevelDiagnosticAt == 0 ||
        nowForWaterDiag - lastWaterLevelDiagnosticAt >= DHT_RAW_DIAGNOSTIC_INTERVAL_MS))
    {
        lastWaterLevelDiagnosticAt = nowForWaterDiag;
        Serial.print("[WATER] distance=");
        Serial.print(filteredDistance, 2);
        Serial.print("cm depth=");
        Serial.print(acceptedDepthCm, 2);
        Serial.print("cm percent=");
        Serial.print(physicalSensors.waterLevel, 1);
        Serial.print(" volume=");
        Serial.print(physicalSensors.waterVolumeLiters, 2);
        Serial.println("L");
    }
}

void SensorManager::readEC()
{
    if (!ecSampler.ready())
        return;

    float adc = ecSampler.median();

    physicalSensors.ecRaw = (int)adc;

    float voltage =
        (adc * ADC_REFERENCE) /
        ADC_RESOLUTION;

    physicalSensors.ecVoltage = voltage;

    // A NaN water temperature must never reach the compensation formula - it
    // would make EC itself go NaN even though the EC sensor is fine. Fall
    // back to the last known-good reading, and only to a fixed default if
    // none has ever been captured this session.
    float compensationTemp;
    EcCompensationSource compensationSource;

    if (isfinite(physicalSensors.waterTemp))
    {
        compensationTemp = physicalSensors.waterTemp;
        compensationSource = EcCompensationSource::LIVE;
    }
    else if (isfinite(lastValidWaterTemp))
    {
        compensationTemp = lastValidWaterTemp;
        compensationSource = EcCompensationSource::LAST_VALID;
    }
    else
    {
        compensationTemp = 25.0f;
        compensationSource = EcCompensationSource::FALLBACK_DEFAULT;
    }

    if (compensationSource != lastEcCompensationSource)
    {
        if (debugManager.shouldPrintDebug(DebugCategory::EC))
        {
            if (compensationSource == EcCompensationSource::LAST_VALID)
            {
                Serial.println("[SENSOR] EC compensation using last valid water temperature");
            }
            else if (compensationSource == EcCompensationSource::FALLBACK_DEFAULT)
            {
                Serial.println("[SENSOR] EC compensation using 25C fallback");
            }
        }
        lastEcCompensationSource = compensationSource;
    }

    float compensationCoefficient =
        1.0f +
        0.02f *
            (compensationTemp - 25.0f);

    float compensationVoltage =
        voltage /
        compensationCoefficient;

    // Feeds physicalSensors.ec (candidate input to the stability filter -
    // see applyEffectiveSensors()) - temperature-compensated exactly as
    // before. Deliberately untouched by the TDS-only requirement below: EC
    // keeps its existing compensation, only TDS loses its own. Calibration
    // (EC_FACTOR, Calibration.h) is unchanged - already accepted, not
    // touched by this pass.
    float compensatedTdsPoly =
        (133.42f * compensationVoltage * compensationVoltage * compensationVoltage -
         255.86f * compensationVoltage * compensationVoltage +
         857.39f * compensationVoltage) *
        0.5f;

    float ec =
        (compensatedTdsPoly / 500.0f) *
        EC_FACTOR;

    physicalSensors.ec = ec;

    // TDS must NOT depend on Water Temperature (explicit requirement, kept
    // from the prior pass) - runs the exact same polynomial against the raw
    // (uncompensated) voltage instead of compensationVoltage, so it no
    // longer inherits the temperature adjustment ec above still applies.
    // physicalSensors.tds was always algebraically (poly-result * EC_FACTOR)
    // here - the /500 that built `ec` and the *500 that used to rebuild
    // `tds` from it cancelled exactly - so this reuses that same existing
    // conversion factor (no new one invented), just computed straight from
    // raw voltage.
    float rawTdsPoly =
        (133.42f * voltage * voltage * voltage -
         255.86f * voltage * voltage +
         857.39f * voltage) *
        0.5f;

    physicalSensors.tds = rawTdsPoly * EC_FACTOR;
}

void SensorManager::readPH()
{
    if (!phSampler.ready())
        return;

    // Real-hardware pH ADC acquisition audit: a physically flat ~1mV source
    // (confirmed with a multimeter directly on GPIO35) was still producing
    // ~10mV of movement in analogReadMilliVolts() - a known ESP32 SAR-ADC
    // noise/nonlinearity characteristic in the upper part of the ADC_11db
    // range (~2.4-2.6V is exactly where this signal sits; no lower
    // attenuation setting can represent this voltage without saturating,
    // so ADC_11db is already the only viable choice here - not changed).
    // median() (already used for EC's own ecSampler, unchanged) discards a
    // single spiked sample entirely instead of being pulled by it like the
    // previous average() was - the same PH_SAMPLE_COUNT/PH_SAMPLE_INTERVAL
    // acquisition window, just a more robust summary statistic over it.
    //
    // Finalized pH acquisition architecture (real-hardware pH bench audit -
    // temporary PH_MEDIAN_ONLY_DIAGNOSTIC confirmed the direct median alone
    // already produces sensible pH values, e.g. ~4.00-4.08 in a pH 4.01
    // buffer): the median feeds PH_SLOPE/PH_OFFSET directly, with no
    // further EMA smoothing stage - the pH-specific EMA that used to sit
    // here has been removed. physicalSensors.ph is the CANDIDATE fed to the
    // existing 10-sample stability window in applyEffectiveSensors(); that
    // window (unchanged: 0.05 tolerance, 3-minute freshness) remains the
    // sole authority for whether a reading is trustworthy enough to publish
    // as sensors.ph or act on for dosing.
    const int medianMv =
        phSampler.median();

    physicalSensors.phMilliVolts = medianMv;

    physicalSensors.ph =
        PH_SLOPE *
            medianMv +
        PH_OFFSET;

    // Throttled diagnostic - readPH() itself runs every loop() tick, far
    // more often than this needs to print. See DHT_RAW_DIAGNOSTIC_INTERVAL_MS
    // for the same pattern.
    const unsigned long now = millis();
    if (debugManager.shouldPrintDebug(DebugCategory::PH) &&
        (lastPhAdcDiagnosticAt == 0 ||
        now - lastPhAdcDiagnosticAt >= PH_ADC_DIAGNOSTIC_INTERVAL_MS))
    {
        lastPhAdcDiagnosticAt = now;
        const int rawMin = phSampler.minValue();
        const int rawMax = phSampler.maxValue();
        Serial.print("[PH-ADC] rawMin=");
        Serial.print(rawMin);
        Serial.print(" rawMax=");
        Serial.print(rawMax);
        Serial.print(" rawRange=");
        Serial.print(rawMax - rawMin);
        Serial.print(" rawMedian=");
        Serial.print(medianMv);
        Serial.print(" candidatePH=");
        Serial.println(physicalSensors.ph, 3);
    }
}
