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
DebugManager debugManager;