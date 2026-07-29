#ifndef GLOBALS_H
#define GLOBALS_H

#include "Types.h"
#include "RTCManager.h"

#include "ActuatorManager.h"
#include "SensorManager.h"
#include "FirebaseManager.h"
#include "AutomationManager.h"
#include "DebugManager.h"
#include "AlertManager.h"
#include "SafetyManager.h"

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
extern DebugManager debugManager;
extern RTCManager rtcManager;
extern AlertManager alertManager;
extern SafetyManager safetyManager;

#endif