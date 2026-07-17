#include "Globals.h"

// ======================================================
// Shared Data
// ======================================================

SensorData sensors;

SystemState systemState;

AlertState alertState;

// ======================================================
// Managers
// ======================================================

ActuatorManager actuatorManager;
SensorManager sensorManager;
FirebaseManager firebaseManager;
AutomationManager automationManager;
AlertManager alertManager;
StartupManager startupManager;
MixingManager mixingManager;
SafetyManager safetyManager;
DebugManager debugManager;