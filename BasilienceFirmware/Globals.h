#ifndef GLOBALS_H
#define GLOBALS_H

#include "SensorManager.h"
#include "FirebaseManager.h"
#include "AutomationManager.h"
#include "AlertManager.h"
#include "StartupManager.h"
#include "MixingManager.h"
#include "SafetyManager.h"
#include "DebugManager.h"
#include "Types.h"
#include "ActuatorManager.h"

// ======================================================
// Shared Data
// ======================================================

extern SensorData sensors;
extern SystemState systemState;
extern AlertState alertState;
extern ActuatorManager actuatorManager;
extern SensorManager sensorManager;
extern FirebaseManager firebaseManager;
extern AutomationManager automationManager;
extern AlertManager alertManager;
extern StartupManager startupManager;
extern MixingManager mixingManager;
extern SafetyManager safetyManager;
extern DebugManager debugManager;


#endif