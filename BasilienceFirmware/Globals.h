#ifndef GLOBALS_H
#define GLOBALS_H

#include "Types.h"

#include "ActuatorManager.h"
#include "SensorManager.h"
#include "FirebaseManager.h"
#include "AutomationManager.h"
#include "DebugManager.h"

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


#endif