#ifndef AUTOMATION_MANAGER_H
#define AUTOMATION_MANAGER_H

#include "Types.h"

class AutomationManager {
public:
  void begin();
  void update();

private:
  enum StartupPhase {
    STARTUP_FOG_ON,
    STARTUP_FOG_OFF
  };

  StartupPhase startupPhase;

  bool fogCycleOn;

  void changeState(SystemMode newMode);

  void handleSensorStabilization();

  void handleStartup();

  void handleNormal();
  void updateGrowLightSchedule();

  const char* getStateName(SystemMode mode);

  void validateSystem();

  void updateAlerts();

  void handleRefilling();
  void handleSafetyLock();

  void handleDosingPH();

  void handleStabilizingPH();

  void handleDosingEC();

  void handleStabilizingEC();

  void completeCurrentOperation();

  void syncRTCFromFirebase();
};

#endif