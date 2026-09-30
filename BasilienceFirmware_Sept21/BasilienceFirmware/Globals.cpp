#include "Globals.h"

// ======================================================
// Shared Data
// ======================================================

SensorData sensors;
SensorData physicalSensors;

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
GsmManager gsmManager;
SmsRecipientCache smsRecipientCache;
HarvestScheduleCache harvestScheduleCache;
NotificationManager notificationManager;
FoggingEventQueue foggingEventQueue;
