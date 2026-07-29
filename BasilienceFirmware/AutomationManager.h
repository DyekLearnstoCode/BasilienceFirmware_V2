#ifndef AUTOMATION_MANAGER_H
#define AUTOMATION_MANAGER_H

#include "Types.h"

class AutomationManager
{
public:
    void begin();
    void update();

private:

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

    //==================================================
    // Automatic Operations
    //==================================================

    void handleNormal();

    bool validateNormalOperation();

    void updateCooling();

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