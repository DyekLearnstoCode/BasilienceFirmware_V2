# Basilience Actuator Guide

Based strictly on how the code in `AutomationManager.cpp` and `ActuatorManager.cpp` is written, here is the exact role and behavior of each actuator in the firmware:

### 1. Fogger (`FOGGER`) & Blower (`BLOWER`)
These two are heavily tied together in the aeroponics misting cycle.
*   **Behavior**: During the startup and running cycles (e.g., `STARTUP_FOG_ON`), the firmware turns **both** the Fogger and the Blower ON at the exact same time. They remain on for a duration defined by `STARTUP_ON_TIME`. Afterward, they both turn off and wait for the off-cycle duration to pass before repeating. 
*   **Safety**: The `ActuatorManager` actively prevents the Fogger from turning on if `alertState.lowWater` is true, stopping it to prevent the ultrasonic discs from burning out when dry.

### 2. Grow Pump (`GROW_PUMP`) & Bloom Pump (`BLOOM_PUMP`)
These control your nutrient dosing.
*   **Behavior**: When the firmware detects that the reservoir's Electrical Conductivity (EC) is too low, it enters the dosing state (`handleDosingEC()`). The code turns **both** the Grow Pump and the Bloom Pump ON simultaneously. 
*   **Duration**: They run for the duration of `systemState.ecDoseTime`. Once the time is up, it stops both pumps and enters a mixing phase to let the nutrients distribute before checking the EC again.

### 3. pH Up Pump (`PH_UP_PUMP`) & pH Down Pump (`PH_DOWN_PUMP`)
These regulate the acidity of the water.
*   **Behavior**: In `handlepHCorrection()`, the firmware checks if `sensors.ph > systemState.highPH` or if it's `< systemState.lowPH`. It turns on the corresponding pump (never both) for a duration of `systemState.phDoseTime`.
*   **Safety**: `ActuatorManager` has a hard-coded conflict rule that forcibly rejects any command to turn on `PH_UP_PUMP` if `PH_DOWN_PUMP` is already running (and vice versa) to prevent accidental chemical reactions or infinite loops.

### 4. Water Solenoid (`SOLENOID`)
This is the automated refill valve.
*   **Behavior**: When `alertState.lowWater` triggers, `handleRefilling()` kicks in. It turns the Solenoid ON to open the water line.
*   **Duration**: It keeps the Solenoid open until `sensors.waterLevel >= systemState.refillStopLevel`, at which point it turns it off.
*   **Safety**: If `safetyManager.canRefill()` returns an unsafe status (e.g., a leak is detected), it instantly aborts the refill operation.

### 5. Peltier (`PELTIER`)
This chills the reservoir water.
*   **Behavior**: In `handleTemperatureControl()`, if `sensors.waterTemp > systemState.highWaterTemp`, it turns the Peltier ON. It remains ON until the temperature drops below `systemState.coolerOffTemp`, providing a hysteresis loop so the cooler isn't rapidly toggling on and off.

### 6. Grow Light (`GROW_LIGHT`)
Controls the lighting cycle.
*   **Behavior**: In `handleLighting()`, it uses the Real-Time Clock (RTC) to check the current hour and minute against `systemState.lightOnHour/Minute` and `lightOffHour/Minute`. It toggles the light ON or OFF precisely when the scheduled time hits.

### 7. Canopy Fan (`CANOPY_FAN`)
*   **Behavior**: Recently swapped in to replace the Water Heater. It blows air across the plant canopy to strengthen stems and disperse heat. *(Trigger logic currently in development)*.
