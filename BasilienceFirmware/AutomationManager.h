#ifndef AUTOMATION_MANAGER_H
#define AUTOMATION_MANAGER_H

#include "Types.h"

class AutomationManager
{
public:
    void begin();
    void update();
    void setManualCoolingDemand(bool active);

    // Admin's explicit judgment that the current water level is acceptable -
    // called by ActuatorManager::requestCommand() when a manual OFF for
    // SOLENOID arrives while an automatic refill is in progress. Ends the
    // refill early, the same way handleRefilling() itself would once
    // sensors.waterLevel reaches refillStopLevel; a no-op if REFILLING isn't
    // actually the current mode.
    void stopRefillManually();

    // Read-only view of the circulation demand mask maintained by
    // updateCooling(). ActuatorManager consults these before allowing a manual
    // OFF so a user cannot stop circulation that an automatic operation is
    // relying on. This is the same mask that drives the pump and the existing
    // "[CIRCULATION] Demand added/removed" diagnostics - not a second copy of
    // the demand conditions.
    bool isCirculationRequired() const;
    const char* circulationRequirementReason() const;

        void createOperationRequest(
        uint16_t requestId,
        OperationType operation,
        OperationAction action,
        RequestSource source);

private:
    uint16_t generateAutoRequestId();

    //==================================================
    // Temporary Automation Test Mode
    //==================================================

    AutomationTestSubsystem lastAutomationTestSubsystem =
        AutomationTestSubsystem::NONE;
    bool automationTestModeInitialized = false;

    bool automationAllowed(AutomationTestSubsystem subsystem) const;
    AutomationTestSubsystem subsystemForOperation(OperationType operation) const;
    void reconcileAutomationTestMode();
    void stopPausedAutomaticControllers();
    void cancelPausedAutomaticOperation();

    //==================================================
    // Cultivation cycle gate
    //
    // harvestScheduleCache.isActive() is the authoritative flag - it is the
    // NVS-persisted mirror of the Firestore active cycle and already survives
    // an offline reboot. The members below are only a previous-state detector
    // so transitions can be logged and acted on once, never every loop.
    //==================================================

    bool cultivationActive = false;
    bool cultivationStateInitialized = false;

    void updateCultivationGate();
    void handleCultivationPaused();
    void stopCultivationChemistry();

    //==================================================
    // Startup
    //==================================================

    enum StartupPhase
    {
        STARTUP_FOG_ON,
        STARTUP_FOG_OFF
    };

    StartupPhase startupPhase;

    bool fogCycleOn;
    unsigned long fogTimerStart;
    String activeFogStrategy;

    bool refillDiagnosticsInitialized = false;
    bool lastRefillMockSource = false;
    float lastRefillWaterLevel = NAN;  // centimeters (waterLevelCm)
    float lastRefillStartLevel = NAN;  // centimeters (refillStartLevelCm)
    float lastRefillStopLevel = NAN;   // centimeters (refillStopLevelCm)

    // NaN = no manual acceptance in effect - the ordinary refillStartLevelCm
    // threshold governs. Set by stopRefillManually() to the water depth (cm)
    // at the moment the admin accepted it; the low-water auto-refill trigger
    // then only fires again once the depth drops below THIS, not merely for
    // remaining under refillStartLevelCm, so an admin's explicit "current
    // level is enough" isn't undone within the same tick by the very
    // condition it was meant to override. Cleared back to NaN the instant a
    // real refill is in progress (handleRefilling() itself, whichever path
    // started it), since any active refill supersedes a prior acceptance.
    float manualRefillAcceptedLevel = NAN;

    // One-shot guard so handleRefilling()'s developer-override exit logs
    // "[DEV WATER] exiting automatic REFILLING..." once per bypass, not
    // every tick the override stays enabled with water still low.
    bool devWaterOverrideExitLogged = false;
    bool shouldAutoRefill() const;

    enum class AutomaticRefillPhase : uint8_t
    {
        RUNNING,
        SETTLING
    };

    AutomaticRefillPhase automaticRefillPhase = AutomaticRefillPhase::RUNNING;
    uint8_t automaticRefillAttempt = 0;
    unsigned long automaticRefillPhaseStartedAt = 0;
    float automaticRefillAttemptStartLevel = NAN;

    void resetAutomaticRefillAttempts();
    bool handleBoundedAutomaticRefill();
    void completeRefillSuccess();
    int8_t lastWaterTemperatureBand = -2;
    bool coolingDemandActive = false;
    bool manualCoolingDemandActive = false;
    bool circulationDiagnosticsInitialized = false;
    uint8_t lastCirculationDemandMask = 0;

    // Bits of lastCirculationDemandMask. Defined here rather than inside
    // updateCooling() so the demand mask has exactly one definition shared by
    // the producer and the read-only accessors above.
    static constexpr uint8_t DEMAND_PELTIER = 0x01;
    static constexpr uint8_t DEMAND_PH = 0x02;
    static constexpr uint8_t DEMAND_EC = 0x04;
    bool lastCirculationRunning = false;
    bool lastPeltierRunning = false;
    ActuatorCommandState lastCirculationState = ActuatorCommandState::OFF;
    bool highAirDemandActive = false;
    bool highHumidityDemandActive = false;
    bool lowAirDemandActive = false;

    // Last AUTOMATIC canopy fan speed actually commanded from a fresh DHT
    // reading (handleCanopyClimate()) - see the automation resilience pass
    // report. Retained (not reset to 100%) whenever DHT becomes unavailable,
    // so canopy ownership does not abruptly jump; both handleCanopyClimate()
    // and the idle handleCultivationPaused() fallback consume this. 50% is
    // the deliberate boot-time default (no valid DHT reading has ever
    // existed yet), matching BLOWER_SPEED_DEFAULT_PERCENT-style fallbacks
    // elsewhere in this codebase - PWM COMMAND only, never measured RPM.
    uint8_t lastAutomaticCanopySpeed = 50;

    // 0 = circulation not yet confirmed running for the current
    // STABILIZING_PH episode. Set once, in updateCooling(), the first tick
    // CIRCULATION_PUMP reports confirmed RUNNING while PH_STABILIZATION
    // demand is active - handleStabilizingPH() measures its 10-second wait
    // from this, not from stateStartTime, so the interval reflects actual
    // pump-on time rather than the state-entry tick (which can precede
    // confirmed circulation by up to ~1s of actuator ramp-up). Reset back to
    // 0 by changeState() on every fresh entry into STABILIZING_PH, including
    // a retry, so each stabilization episode gets its own full 10s.
    unsigned long phStabilizationCirculationConfirmedAt = 0;

    // Same anchor, same reasoning, for STABILIZING_EC - see
    // phStabilizationCirculationConfirmedAt's own comment above.
    unsigned long ecStabilizationCirculationConfirmedAt = 0;

    //==================================================
    // Serial Monitor Focus Mode - compact per-controller dependency
    // summaries (see DebugManager::shouldPrintDebug()'s own comment).
    // Diagnostics only - no automation/safety decision reads these.
    //==================================================

    void logPHInputSummary();
    void logECInputSummary();
    void logCoolingInputSummary();
    void logFogInputSummary(const char* cadenceLabel);

    // Edge-triggered decision-outcome line for processPHCorrection()/
    // processECCorrection() - prints only when the line's own content
    // differs from the last one printed (covers both a changed reading and
    // a changed decision type), never every tick.
    void logPHDecisionLine(const String& line);
    void logECDecisionLine(const String& line);

    //==================================================
    // Core
    //==================================================

    void processCurrentState();

    void changeState(SystemMode newMode);

    void validateSystem();

    void completeCurrentOperation();

    void failCurrentOperation(
        const String& reason);

    void syncRTCFromFirebase();

    const char* getStateName(
        SystemMode mode);

    //==================================================
    // Manual Operations
    //==================================================

    void processOperationRequest();

    void processRefillOperation();

    void processPHUpOperation();

    void processPHDownOperation();

    void processResetSafetyOperation();
    void processECCorrectionOperation();

    bool abortCurrentOperation(
    SafetyResult result);

    bool abortCurrentOperation(
    const String& reason);

    void failCurrentSubsystem(const String& reason);

    //==================================================
    // Automatic Operations
    //==================================================

    void handleNormal();

    bool validateNormalOperation();

    void suspendAutomaticRootFogging(const String& reason);

    void updateCooling();

    String getCirculationReason(uint8_t demandMask) const;

    void handleCanopyClimate();

    bool processRefillRequest();

    bool processPHCorrection();

    bool processECCorrection();

    // Gate for starting a NEW pH/EC correction (initial trigger, a manual
    // operation's fresh start, or a stabilization retry) - distinct from
    // sensors.ph/ec being non-NaN. Requires the sensor's CURRENT stability
    // window to have just reconfirmed the reading (SensorManager::
    // isPhCurrentlyStable()/isEcCurrentlyStable()), not merely that a
    // pre-dose/pre-disturbance value is still being retained for display.
    // Does not gate a correction already RUNNING/STABILIZING - see each call
    // site for exactly what it blocks.
    bool canStartNewPHCorrection() const;
    bool canStartNewECCorrection() const;

    // Real-hardware pre-integration follow-up, Part E: while isolated
    // Automation Test Mode is selected, explains on the Serial log exactly
    // why the selected controller is (or isn't) currently blocked from
    // starting a correction - see its own comment for the exact reason
    // codes. Read-only/diagnostic only; makes no automation decisions and
    // changes no state other than its own internal log throttle.
    void logAutomationTestBlockReason();

    void processFogCycle();

    //==================================================
    // Scheduling
    //==================================================

    void updateGrowLightSchedule();

    bool isWithinSchedule(
        uint8_t startHour,
        uint8_t startMinute,
        uint8_t endHour,
        uint8_t endMinute) const;

    int getCurrentMinutes() const;

    // Strict activation condition for the developer-only Grow Light mock
    // time (Types.h's mockGrowLightMinutes/mockGrowLightTimeEnabled): both
    // automationTestSubsystem == GROW_LIGHT and the flag must be true. A
    // single source of truth shared by updateGrowLightSchedule()'s RTC-
    // validity gate and getCurrentMinutes()'s time source, so the two can
    // never disagree about whether mock time is in effect.
    bool growLightMockTimeActive() const;

    //==================================================
    // State Handlers
    //==================================================

    void handleSensorStabilization();

    void handleStartup();

    void handleRefilling();

    void handleDosingPH();

    void handleStabilizingPH();

    void handleDosingEC();

    void handleStabilizingEC();

    void handleSafetyLock();
};

#endif
