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
    float lastRefillWaterLevel = NAN;
    float lastRefillStartLevel = NAN;
    float lastRefillStopLevel = NAN;

    // NaN = no manual acceptance in effect - the ordinary refillStartLevel
    // threshold governs. Set by stopRefillManually() to the water level at
    // the moment the admin accepted it; the low-water auto-refill trigger
    // then only fires again once the level drops below THIS, not merely for
    // remaining under refillStartLevel, so an admin's explicit "current
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

    bool processReadyLocalRegulation();

    void suspendAutomaticRootFogging(const String& reason);

    void updateCooling();

    String getCirculationReason(uint8_t demandMask) const;

    void handleCanopyClimate();

    bool processRefillRequest();

    bool processPHCorrection();

    bool processECCorrection();

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
