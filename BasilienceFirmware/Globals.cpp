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

RTCManager rtcManager;
AlertManager alertManager;
SafetyManager safetyManager;
WiFiManager wifiManager;