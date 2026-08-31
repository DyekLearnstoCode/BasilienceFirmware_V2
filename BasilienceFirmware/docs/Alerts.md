# Basilience Alerts Guide

Based strictly on how the code in `AlertManager.cpp`, `FirebaseManager.cpp` (firmware) and `functions/index.js` (Cloud Functions, in the `Basilience` app repo) is written, here is the exact set of alerts/notifications the system can generate, what triggers each one, and where it comes from.

### 1. Parameter / threshold alerts
Computed every tick in `AlertManager.cpp` from live sensor readings against configured target ranges, published to `/devices/{deviceId}/alerts` by `FirebaseManager::writeAlerts()`, and turned into a push notification + Firestore history record by `onAlertUpdated` in `functions/index.js` the moment a flag transitions from false to true.

| Alert | RTDB flag | Trigger |
|---|---|---|
| Water level is low | `lowWater` | Reservoir depth at/below the refill trigger (`refillStartLevelCm`) - the operational threshold that also opens the solenoid |
| Critical low water level | `criticalLowWater` | Depth falls further, past `criticalLowWaterCm` - a severity escalation on top of `lowWater`, no new actuator behavior |
| Nutrient level is too low / too high | `ecLow` / `ecHigh` | EC below/above `minEC`/`maxEC` |
| pH is too low / too high | `phLow` / `phHigh` | pH below/above `minPH`/`maxPH` |
| High Water Temperature | `waterTempOutOfRange` | Water temp above `maxWaterTemp` (high side only) |
| Low Water Temperature | `waterTempLow` | Water temp below `minWaterTemp` |
| Low / High Air Temperature | `lowAirTemperature` / `highTemperature` | Air temp outside `minAirTemp`/`maxAirTemp` |
| Low / High Humidity | `humidityLow` / `humidityHigh` | Humidity outside `minHumidity`/`maxHumidity` |
| Low / High Water Level | `waterLevelLow` / `waterLevelHigh` | Depth outside the target-range percentage (distinct from the operational `lowWater`/`criticalLowWater` pair above) |
| Sensor Fault | `sensorFault` | A reservoir/root-zone sensor (water temp, pH, EC, or water level) is reading invalid/unavailable for `SENSOR_TRANSIENT_FAILURE_THRESHOLD` consecutive ticks |

**Notes:**
- Every alert above (except `sensorFault`) is debounced: it must hold true for `SENSOR_TRANSIENT_FAILURE_THRESHOLD` consecutive evaluations before it raises, but clears immediately on the first good reading - see `AlertManager::setAlertDebounced()`.
- `waterLevelLow` is deliberately suppressed by `onAlertUpdated` whenever `lowWater` is already active, so a single falling-water event doesn't generate two notifications.
- `phOutOfRange` also exists as an RTDB flag (`AlertManager::updatePHAlert()`, `phLow || phHigh`) but does **not** generate its own notification - it's superseded by `phLow`/`phHigh`, which already cover it.
- `sensorFault` is only eligible to publish/notify once startup has completed and the sensor source (physical vs. mock) has resolved - see `FirebaseManager::writeAlerts()`'s `sensorFaultPublishingEligible` gate.

### 2. Hardware / safety lockout alerts
Fired from `/devices/{deviceId}/status` fields via `onStatusUpdated` in `functions/index.js` when automation could not self-correct and a subsystem has stopped, requiring manual attention.

| Alert | Status flag |
|---|---|
| Safety Lock Activated | `safetyLock` |
| pH correction needs attention | `phSubsystemLocked` |
| Nutrient correction needs attention | `ecSubsystemLocked` |
| Reservoir refill needs attention | `refillSubsystemLocked` |
| Cooling System Stopped | `coolingSubsystemLocked` |

### 3. Recovery / success notifications
Sent once automation successfully resolves a condition - the positive counterpart to the alerts above.

- **"pH is back to normal"** - `PH_UP`/`PH_DOWN` operation completes successfully (`automationSuccessContent()`).
- **"Nutrient level is back to normal"** - `EC_CORRECTION` operation completes successfully.
- **"Reservoir refilled"** - `REFILL` operation completes successfully.
- **"Water temperature is back to normal"** - observed from the Peltier actuator's own RUNNING→OFF transition once `waterTemp <= coolerOffTemp`, since cooling completion happens well after `waterTempOutOfRange` has already cleared and can't be observed from the alert itself (`emitWaterTemperatureCoolingSuccess()`).

### 4. Connectivity alerts
Driven by device presence/heartbeat, independent of any sensor reading - `handleDeviceConnectivityTransition()` in `functions/index.js`.

- **"Basilience Device Unreachable"** - no heartbeat received for `OFFLINE_TIMEOUT_MS` (40s). Message: *"Basilience cannot communicate with the device. Check its power or network connection. Local automation may still be running if the device has power."*
- **"Basilience Device Back Online"** - device reconnects and cloud monitoring resumes.

This path is intentionally suppressed for up to `PROVISIONING_GRACE_MS` (10 minutes) while `/status/provisioning` is true, so a normal Wi-Fi reconfiguration session doesn't trigger a false "unreachable" alert.

### 5. Harvest reminders
Evaluated hourly by the `evaluateHarvestReminders` scheduled function against each device's active cycle, in the app's Philippine operating timezone.

- **"Harvest Scheduled Tomorrow"** - the cycle's `nextHarvestDate` is tomorrow.
- **"Harvest Ready Today"** - the cycle's `nextHarvestDate` is today.

### 6. Local/offline SMS fallback types
Firmware keeps a separate, deliberately smaller catalog in `NotificationTypes.h` (`NotificationEventType` enum) for the SMS path used when the device cannot reach the cloud at all - only the highest-priority conditions, not the full list above:

`LOW_WATER`, `HIGH_WATER_TEMP`, `HIGH_AIR_TEMP`, `SENSOR_FAULT`, `DEVICE_UNREACHABLE`, `HARVEST_DUE`

Each carries a severity (`SEV_LOW`/`SEV_MEDIUM`/`SEV_HIGH`/`SEV_CRITICAL`) and is queued durably (`NotificationEvent`, persisted as one NVS blob so it survives a reboot) until it can be delivered by SMS and/or replayed to the cloud once connectivity returns.
