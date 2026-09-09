# Basilience Firmware Guide

This document explains the Basilience firmware in plain English for a developer or maintainer who did not write it. It covers what the system does, how it is put together, and what you would need to know to safely change it.

Two other documents already exist in this `docs/` folder: `Actuators.md`, `Alerts.md`, and `StateMachine.md`. Those are shorter, engineer-facing references. This guide is a wider walkthrough meant to get a new person oriented in the whole codebase. Where `StateMachine.md` describes an older design (for example a 20 minute startup fog and a water heater that no longer exist), this guide describes what the current code actually does, since the code is always the source of truth.

## 1. What this firmware is and what it runs on

Basilience is a controller for a fogponics growing system for basil. Fogponics means the plant roots hang in open air inside a chamber, and a fogger mists nutrient solution onto them instead of the roots sitting in a pool of water like normal hydroponics.

The firmware runs on an ESP32 microcontroller. It is written as an Arduino sketch (`BasilienceFirmware.ino`) plus a set of C++ classes, one per subsystem, called "managers" in this codebase.

The ESP32 is physically wired to:

- **DHT22 sensor** (pin 4): air temperature and humidity inside the grow chamber.
- **DS18B20 sensor** (pin 5, a one-wire digital sensor): reservoir water temperature.
- **pH probe** (analog pin 35) and **EC probe** (analog pin 34, EC means electrical conductivity, which is how the firmware estimates nutrient concentration in the water).
- **HC-SR04 ultrasonic sensor** (trigger pin 18, echo pin 19): measures water depth in the reservoir by timing an echo, the same way a bat or submarine sonar works.
- **DS3231 real-time clock module** (I2C pins 21 and 22): keeps the current date and time even when the ESP32 is powered off, using its own small battery.
- **SIM800L GSM module** (UART pins 36 RX and 23 TX, wired at a fixed 9600 baud): lets the device send text messages over the cellular network when Wi-Fi and the internet are not available.
- **Fogger** (pin 26) and **grow light** (pin 25): both are switched through solid-state relays (SSRs), which are electronic on/off switches with no moving parts.
- **Blower fan** (pin 27) and **water solenoid valve** (pin 15): switched through a MOSFET driver board. The blower's speed can be varied, not just on/off, using PWM (explained below).
- **Grow pump and bloom pump** (pins 32 and 33): peristaltic pumps that add the two parts of a nutrient formula to raise EC.
- **pH Up pump and pH Down pump** (pins 14 and 12): peristaltic pumps that dose pH-adjusting solution.
- **Canopy fan** (pin 17): blows air across the plants and also runs on variable speed PWM.
- **Peltier cooler** (pin 16) and **circulation pump** (pin 13): the Peltier is a solid-state device that cools the reservoir water when it gets too hot, and the circulation pump moves water past it (and generally keeps the reservoir mixed) whenever cooling or a chemical correction is happening.

The reservoir itself has fixed dimensions the firmware knows about (52 cm long, 34 cm wide, with a 6 cm "working" depth that counts as 100 percent full). These numbers, along with the ultrasonic sensor's mounting height, are what let the firmware turn a raw distance reading into a depth, a percentage, and a volume in liters.

The device talks to the outside world over Wi-Fi to a Google Firebase Realtime Database, and there is a companion Android app that reads and writes that same database. When Wi-Fi or the internet is not available, the firmware falls back to sending SMS text messages through the SIM800L module for the most important alerts.

## 2. How the main loop works

Arduino sketches have two functions: `setup()`, which runs once at power-on, and `loop()`, which repeats forever. Basilience's `loop()` (in `BasilienceFirmware.ino`) calls `update()` on each manager in turn, once per pass, and then starts the next pass immediately.

The most important rule in this codebase is that **nothing in `loop()` ever calls `delay()`**. A call to `delay()` freezes the entire chip for that amount of time, which would mean the reservoir could overheat, run dry, or the fogger could run unattended while the code was "asleep." Instead, every manager that needs to wait for something (a sensor to settle, a pump to finish a dose, a timeout to expire) checks the built-in `millis()` clock (milliseconds since boot) against a timestamp it saved earlier, and only acts once enough time has passed. If it is not time yet, the function simply returns immediately and lets the next manager run.

This pattern is sometimes called a "non-blocking state machine." A state machine is just a system that is always in exactly one named condition (a "state") at a time, and moves between states based on rules. Because none of these states ever block the loop, many things can appear to happen "at the same time": the fogger can be mid-cycle, the pH probe can be mid-stabilization-check, and the Wi-Fi connection can be mid-reconnect, all in the same fraction of a second of wall-clock time, because each one only needs a tiny sliver of CPU time on every pass through `loop()`.

The order of operations in `loop()` matters and is deliberate:

1. `wifiManager.update()` runs first, and checks whether the device is currently in "provisioning mode" (broadcasting its own Wi-Fi setup network instead of being connected to one).
2. `sensorManager.update()` and `rtcManager.update()` always run, so fresh readings and the clock are available to everything else.
3. `automationManager.update()` and `safetyManager.update()` run next, unless a developer diagnostic called Sensor Test is active (explained in section 12).
4. `actuatorManager.update()` applies whatever automation or manual commands were just decided, actually turning pins on or off.
5. `gsmManager.update()` and `notificationManager.update()` run regardless of Wi-Fi state, since SMS delivery must keep working even while the device is offline or in the middle of Wi-Fi setup.
6. If the device is in provisioning mode, the loop stops here for this pass, so the local Wi-Fi setup web server stays responsive.
7. Otherwise, `firebaseManager.update()` runs, which is where sensor readings get uploaded, settings get downloaded, and app commands get read.

Local growing control (sensing, safety, and actuators) always happens before anything cloud-related, and it never depends on the cloud being reachable.

## 3. Sensors

`SensorManager` is responsible for reading every physical sensor and turning the raw signal into a trustworthy value. A few of its ideas repeat across sensors:

- **Reading on a schedule, not every pass.** Each sensor has a minimum interval between reads (`DHT_READ_INTERVAL_MS`, `WATER_TEMP_READ_INTERVAL_MS`, `WATER_LEVEL_READ_INTERVAL_MS` in `Config.h`, all around 5 seconds). Reading faster than the sensor can physically respond just produces noise or errors, so the code waits.
- **Holding the last good value instead of blanking out.** If a sensor fails to give a valid reading a few times in a row (the threshold is `SENSOR_TRANSIENT_FAILURE_THRESHOLD`, currently 3), the firmware marks it "unavailable" rather than immediately reporting garbage. Until that threshold is hit, a single bad read is treated as noise and the previous reading is kept.
- **Smoothing.** Air temperature, humidity, and water temperature are smoothed with an exponential moving average, a simple filter where each new reading nudges a running average a little instead of replacing it outright, so a single noisy sample cannot cause a visible jump.
- **pH gets extra filtering.** Because the pH probe is sensitive to electrical noise from the reservoir, a raw pH candidate has to first pass a "step filter" (several readings in a row have to agree before a big jump is trusted) and then a "stability window" (10 samples about a second apart all have to agree within a small tolerance) before it is considered safe to act on. This is why the pH shown on the app can occasionally show "confirming a new reading" instead of updating instantly.
- **Water level gets a similar two-stage filter**: a median of the last 5 readings, then a step-confirmation check, before a new depth is trusted enough to trigger a refill or clear a low-water alert. The ultrasonic sensor is also skipped entirely while the fogger is running, because the mist disturbs the echo.

**Physical vs. mock sensor mode.** The app can tell the device to ignore its real sensors and use developer-supplied numbers instead ("mock sensors"), useful for testing automation logic on a bench without real plants and probes. Whether mock mode is on is remembered in the ESP32's flash memory (NVS, described more below) so it survives a reboot, but the actual mock numbers themselves are never saved, only the on/off flag. If the device reboots while mock mode was on and no fresh mock data arrives within two minutes (`MOCK_BOOT_PAYLOAD_TIMEOUT`), it gives up waiting and falls back to real sensors, so a forgotten mock session can never hold the system hostage. There is also a "dynamic mock" option that makes the mock numbers wander randomly within a small range every 10 seconds, to simulate a live, slightly-changing environment.

## 4. Automatic control

`AutomationManager` decides when to turn things on or off. It only runs at all while a "growth cycle" is active. A growth cycle is a cultivation run (planting to harvest) that the app creates and that the firmware learns about through `HarvestScheduleCache`. If no cycle is active, the firmware pauses almost everything (fogging, pH and EC dosing, refilling) and just keeps ventilation at a safe default, waiting for the next cycle to start.

The firmware distinguishes two different kinds of numbers that might sound similar:

- **Target ranges** (for example `minAirTemp`/`maxAirTemp`, `minPH`/`maxPH`) answer "is this reading acceptable for the crop." These are what alerts and reports compare against.
- **Control thresholds** (for example `highAirTemp`/`airTempRelease`, `highWaterTemp`/`coolerOffTemp`) answer "when should a fan or cooler switch on or off." These usually come in pairs with a gap between them, called hysteresis, so equipment does not rapidly flick on and off right at the boundary.

The system moves through a sequence of named states (see `Types.h`'s `SystemMode` enum):

- **SENSOR_STABILIZATION**: the very first state after boot or after a growth cycle starts. The firmware waits here until pH and EC readings have actually settled (or until a one-minute safety timeout), so it never bases its first decision on a reading that has not warmed up yet.
- **STARTUP**: a one-time acclimation sequence for newly transplanted seedlings. The fogger and blower run together for 90 seconds (`STARTUP_ON_TIME`), then the fogger turns off for 60 seconds (`STARTUP_OFF_TIME`) while the blower keeps running briefly to clear leftover mist, then the system moves to NORMAL.
- **NORMAL**: the main operating state. From here the firmware checks, every pass, whether a refill, a pH correction, or an EC correction is needed, and otherwise runs the ordinary fog cycle and keeps the canopy fan, grow light, and cooling running to their own rules.
- **REFILLING**: the water solenoid valve is open, adding water. See below for how this is bounded.
- **DOSING_PH / STABILIZING_PH** and **DOSING_EC / STABILIZING_EC**: a chemical correction is in progress. DOSING runs a pump for a fixed duration, then STABILIZING waits for the reservoir to mix and rechecks the reading before deciding whether another dose is needed.
- **SAFETY_LOCK**: every actuator is forced off and stays off until an administrator clears the lock (see section 6).

**Refilling.** When water drops to or below `refillStartLevelCm` (2 cm of depth by default), the firmware treats a refill as eligible. The real automatic refill logic runs the solenoid for a bounded 30 second burst, waits 10 seconds for the ultrasonic reading to settle (since flowing water gives a bad echo), then checks the depth again. This repeats for at most 3 attempts (`MAX_REFILL_ATTEMPTS`). If the reservoir still has not reached the stop level (`refillStopLevelCm`, 3 cm by default) after 3 attempts, the refill subsystem locks itself and waits for the level to recover on its own or for an administrator to intervene, rather than holding the solenoid open indefinitely.

**pH and EC correction.** When a reading is out of its safe range and has been confirmed stable, the firmware picks a direction (pH Up or pH Down, or for EC, "raise" using the grow and bloom pumps or "dilute" by adding water through the solenoid), runs the matching pump for a fixed dose time, then circulates for at least one minute before checking again. This can retry up to 3 times (`MAX_PH_ATTEMPTS` / `MAX_EC_ATTEMPTS`) before locking that subsystem and asking for manual attention. A short cooldown (`PH_DOSE_COOLDOWN` / `EC_DOSE_COOLDOWN`, one minute) also has to pass since the last dose before a brand new correction can start, because a probe can look "stable" before the just-dosed chemical has actually finished mixing in.

**Canopy climate and fogging cadence.** The canopy fan runs at 70 percent speed normally, 100 percent if the air gets too hot or too humid, and 50 percent if it gets too cold. The main fog cycle itself changes its on/off timing (not just its speed) depending on air temperature: a "hot" cadence runs the fogger longer per cycle, a "cold" cadence runs it for shorter bursts with longer rests, and "normal" is used the rest of the time (see `NORMAL_FOG_ON_TIME`, `HOT_FOG_ON_TIME`, `COLD_FOG_ON_TIME` and their matching off-times in `Config.h`).

**Developer Automation Test Mode.** There is also a built-in way for a developer to isolate a single controller (for example, only ever run the EC correction logic, with every other automatic actuator paused) to test one piece of the system at a time on real hardware. This is controlled from the app and never persists across a reboot, so it can never accidentally get left on in the field.

## 5. Actuators

An actuator here just means anything the firmware physically switches on or off: pumps, fans, valves, the fogger, and the grow light. `ActuatorManager` is the single place that ever actually changes a GPIO pin. Every command, whether it came from automation or from a person, goes through the same request-and-validate pipeline: a command is submitted, validated against a list of safety rules for that specific actuator, and only then applied to the pin.

The blower and canopy fan are the two actuators that support variable speed instead of just on/off, using PWM (pulse-width modulation, a technique that switches a pin on and off very fast so the average power delivered can be tuned, similar to how a dimmer switch works). Both run at a low 200 Hz switching frequency, chosen after bench testing showed the driver board's opto-isolated relays could not switch cleanly at the originally-used 5000 Hz (see the comment above `CANOPY_BLOWER_PWM_FREQUENCY_HZ` in `Config.h` for the full story, and `basilience_canopy_blower_pwm_fix.md` for the same finding recorded separately).

**Manual mode vs. automatic mode.** `systemState.manualMode` is a single flag that, when on, lets an administrator command an actuator directly from the app instead of automation deciding for it. Taking manual control of an actuator "holds" it: automation will not send it a new command while that hold is active, so a person's explicit choice is not immediately overwritten. That hold is released the moment the person sends an explicit OFF, when manual mode is turned off entirely, or when an independent hardware safety timer expires (see below). One actuator is a deliberate exception: the circulation pump. Because automatic cooling and pH/EC stabilization physically depend on the pump running, a manual attempt to turn it off is refused outright whenever an automatic operation still needs it.

Even in manual mode, hard safety rules are never skipped. For example, the two pH pumps can never run at the same time as each other, the fogger and dosing pumps are blocked while the reservoir is too low, and the Peltier cooler will not run automatically without the circulation pump also running (a manual command is a narrow exception to that specific rule, since a human operating the Peltier alone is assumed to know what they are doing).

Every actuator that can be commanded manually also has a hard, independent timeout enforced by its own microcontroller timer (not just the main loop), so that even if the main loop somehow stalled, a pump could never be left running forever. Dosing pumps cap out at `MANUAL_PUMP_RUNTIME` (5 seconds), and the solenoid, Peltier, and fogger cap out at `OPERATION_TIMEOUT_MS` (5 minutes) for a manual run.

For the exact behavior of each individual actuator, see `docs/Actuators.md`, which documents this in more detail from the code.

## 6. Safety system

A "safety lockout" is the firmware's way of saying "I cannot fix this myself, a human needs to look at it." There are two levels:

- **A subsystem lock** (`phSubsystemLocked`, `ecSubsystemLocked`, `refillSubsystemLocked`, `coolingSubsystemLocked`) stops just that one subsystem. For example, if pH correction fails 3 times in a row, `phSubsystemLocked` becomes true and pH dosing simply stops happening, while everything else (fogging, EC, cooling) keeps running normally.
- **A global safety lock** (`systemState.safetyLock`) forces every actuator off and freezes the whole system in the SAFETY_LOCK state. This only happens for something the firmware considers genuinely system-wide and serious, not a routine subsystem failure.

What triggers a lock:

- A sensor reading a subsystem depends on becomes invalid (`SafetyManager`'s validity checks require a value to be a real, in-range number for several ticks in a row, not just momentarily missing).
- pH or EC correction runs out of its attempt budget (3 tries) without reaching a safe range.
- An automatic refill runs out of its attempt budget (3 bounded 30-second bursts) without reaching the stop level.
- EC dilution is needed but the reservoir is already at its physical capacity (`RESERVOIR_FULL`), since adding more water is not possible.
- The reservoir runs low enough (at or below `refillStartLevelCm`) that dosing, fogging, and cooling are all blocked as an operational safety measure, even before any lock is set.

How a lock clears:

- Most subsystem locks clear themselves automatically once the underlying reading actually recovers into a safe range on its own (for example, pH drifting back inside `minPH`/`maxPH`). This is checked every pass in `handleNormal()`, not on a timer, so recovery is detected as soon as it happens.
- An administrator can also send an explicit "Reset Safety" request from the app. This is a deliberately weaker bar than full recovery: it only requires the relevant sensor to currently be reporting a real, physically valid number, not that the underlying problem is already fixed. The reasoning is that some locks (like 3 failed pH attempts) exist specifically because automation could not fix the problem on its own, so demanding it be already-fixed before allowing a reset would make the lock permanent.
- The global safety lock can only be cleared by Reset Safety, and only once every other safety condition it depends on (`canResetSafety()`) is also satisfied.

There is also a developer-only override, `ignoreWaterLevelAutomation`, that lets a bench tester run pH, EC, fogging, and cooling logic with the reservoir intentionally kept below the normal low-water threshold, for testing without needing a full reservoir. It never bypasses anything else, and it resets to off on every reboot so it can never be silently left on.

## 7. Alerts and notifications

Alerts are computed every pass by `AlertManager` by comparing live sensor readings against the configured target ranges (low water, pH out of range, EC out of range, temperature and humidity out of range, and so on). Alerts are debounced, meaning a reading has to stay on the wrong side of the line for a few consecutive checks before the alert actually raises, but it clears again the instant a single good reading comes in. This stops one noisy sample from firing (and un-firing) an alert repeatedly.

There are also hardware lockout alerts (the subsystem-locked flags from section 6) and recovery messages sent once a problem resolves on its own.

The full table of every alert, exactly what triggers it, and exactly which RTDB field it lives under is already documented in `docs/Alerts.md` and is not repeated here, since that document is kept as the authoritative reference and this guide would just drift out of sync with it otherwise.

## 8. Cloud connection

**Wi-Fi.** `WiFiManager` owns the radio and is the only class in this firmware allowed to call the low-level Wi-Fi connect/disconnect functions (this was a deliberate fix, since the ESP32's own library and the Firebase library both have their own auto-reconnect behavior that used to fight with this firmware's own retry logic). It reads saved credentials from flash storage and tries them. If that fails for about 20 seconds it retries every 5 seconds, and if the whole outage stretches on, it falls back to broadcasting its own setup Wi-Fi network (see section 10).

**Firebase.** `FirebaseManager` only starts once Wi-Fi is actually connected (it is never started while the device is in provisioning mode). Each device authenticates as its own identity in Firebase, using either a previously-saved refresh token or a one-time "device secret" exchanged with a cloud function for a token (see `SECURE_DEVICE_AUTH_REQUIRED` in `Config.h`), with an older, less secure anonymous-login fallback still available for devices that have not yet been migrated to the newer method.

**What goes up:** live sensor readings and a "device is alive" heartbeat (about once a second), the current automation state and safety flags, active alerts, the state of every actuator, and diagnostic telemetry like dose counts.

**What comes down:** configuration settings (target ranges, thresholds, grow light schedule, calibration numbers), manual actuator commands and manual operation requests (refill, pH correction, EC correction, reset safety) sent from the app, mock sensor payloads and developer test-mode selections, and the list of phone numbers eligible for SMS alerts.

**Offline behavior.** None of the local growing logic (sensing, automation, safety, actuators) depends on Firebase being reachable at all, by design. Settings loaded once are kept in flash (NVS) so a reboot with no internet still starts from the last known-good configuration. When the connection actually drops out, the firmware moves through a small set of health states (`FirebaseHealthState`): a few failures in a row degrade it to skipping only low-priority background work, and enough consecutive failures put it into a cooldown period with an increasing backoff delay before it tries to reconnect, so a broken connection cannot be hammered pointlessly or stall the loop.

## 9. SMS fallback

Text messages are a fallback channel, not a duplicate of the app's push notifications. `NotificationManager` only ever queues an SMS-worthy event when the device currently has no cloud connection at all (`wifiConnected && firebaseConnected` is false). If the device is online, the same alert already reaches the app through Firebase and a Cloud Function, and a text message on top of that would just be noise.

`NotificationManager` and `GsmManager` split the work: `GsmManager` is a low-level driver, it knows how to talk AT commands to the SIM800L modem, register on the cellular network, and send exactly one text to exactly one number at a time, with no idea of why. `NotificationManager` is the policy layer on top: it watches for alert transitions, decides which events deserve an SMS, keeps a durable queue of them (saved to flash so a reboot does not lose a pending alert), and sends one recipient at a time to everyone in the SMS recipient list, retrying failed sends once before giving up.

An SMS is only ever sent for a short, deliberately smaller list of the most urgent conditions: low water, high water temperature, high air temperature, a sensor fault, the device becoming completely unreachable, a harvest being due, and the device falling back into its own Wi-Fi setup mode. Ordinary in-range-but-notable events do not get a text.

A message looks like: `Basilience: <title> - <message>`, for example `Basilience: Low Reservoir - Water level dropped below the refill threshold.` It is kept under about 155 characters to fit in one SMS segment.

## 10. Wi-Fi setup and provisioning mode

The first time a device is powered on (or any time it loses its saved network and cannot get back on it), it broadcasts its own temporary Wi-Fi network named `Basilience-Setup`. Connecting a phone to that network and then sending the new home network's name and password to the device (the Android app does this over `POST /setup`) tells the device what to connect to. The device saves the new credentials to flash and tries them right away, while keeping `Basilience-Setup` running the whole time, so a phone still connected to it can ask `GET /status` for the real outcome ("connecting", "connected", or "connection_failed") instead of guessing. There is no reboot involved. If the new network fails to connect, the device automatically goes back to whatever credentials were working before, so a typo never strands the device on a broken network.

There are two flavors of this, both handled by `WiFiManager`:

- **Fallback provisioning** happens automatically and unattended, when the saved network stops working (for example the device is moved to a new location, or the router's password changed) and cannot be recovered within the recovery window. While in fallback mode, the device also keeps quietly retrying the old saved network in the background every 30 seconds, so if the outage was temporary it can silently reconnect and drop the setup network again without anyone having to do anything.
- **Manual provisioning** happens when a person deliberately asks the app to reconfigure Wi-Fi. The background retry described above is intentionally turned off for as long as a submitted set of credentials is still being tried, since both would otherwise fight over the same radio at once.

The setup network also has a `/secure-provision` endpoint used once, during initial device setup, to inject the device's one-time secret used for the secure Firebase authentication described in section 8.

## 11. Startup sequence

From power-on to normal operation, roughly in order:

1. **Local safety first.** `setup()` in `BasilienceFirmware.ino` loads persisted settings from flash, then brings up the actuator pins (forcing every output to a known-off state), the sensors, the RTC, alert tracking, safety checking, and automation, all before touching Wi-Fi or the internet at all. This ordering means the plant is being protected even if the device can never reach the network.
2. **Wi-Fi.** `wifiManager.begin()` either starts connecting to a saved network or, if none is saved, goes straight into provisioning mode.
3. **GSM.** `gsmManager.begin()` opens the SIM800L's UART and starts its own state machine (module check, SIM check, network registration), independent of Wi-Fi.
4. **Notification and fogging history caches** (`SmsRecipientCache`, `HarvestScheduleCache`, `NotificationManager`, `FoggingEventQueue`) load their persisted state from flash so a pending SMS or an in-progress cultivation cycle survives a reboot.
5. **`loop()` begins running immediately**, even before Wi-Fi has finished connecting, so sensing, safety, and automation start on the very first pass rather than waiting for the network.
6. **Firebase only starts once Wi-Fi actually connects.** If the device booted with no saved Wi-Fi at all, Firebase is never started until a network becomes available later.
7. Inside automation, the very first state is `SENSOR_STABILIZATION`, which waits for pH and EC readings to settle (or a one-minute cap) before moving on.
8. From there the system runs the one-time `STARTUP` acclimation fog sequence described in section 4, and then settles into `NORMAL`, where it stays for the rest of the growth cycle unless a refill, correction, or safety event pulls it into a different state temporarily.

## 12. Settings a developer would actually need to change

`Config.h` holds compile-time constants: hardware pin numbers, timing values, and default thresholds. Many of these are also mirrored into `systemState` as runtime defaults, which means an admin can later change them from the app without needing a new firmware build. The `Config.h` value only matters as the starting point and as the fallback if a stored setting is ever missing or invalid.

**Wi-Fi and Firebase identity** (top of `Config.h`): `WIFI_SSID`/`WIFI_PASSWORD` are a fallback network baked into the firmware, used only until real credentials are provisioned through the setup process in section 10. `API_KEY` and `DATABASE_URL` point at the specific Firebase project this firmware talks to. Changing these would point the device at a different database entirely.

**Pin assignments** (`SSR Outputs`, `MOSFET Outputs`, `Peristaltic Pumps`, `Temperature`, `Sensor Inputs`, `GSM / SIM800L` sections): these only need to change if the physical wiring changes, for example moving a wire to a different GPIO pin on a new board revision.

**Sensor read timing** (`WATER_TEMP_READ_INTERVAL_MS`, `WATER_LEVEL_READ_INTERVAL_MS`, `DHT_READ_INTERVAL_MS`): how often each sensor is actually re-sampled. Lowering these risks reintroducing the read errors these delays were added to fix. `SENSOR_TRANSIENT_FAILURE_THRESHOLD` controls how many bad reads in a row it takes before a sensor is considered genuinely broken rather than just noisy.

**Sensor thresholds and target ranges** (`MIN_HUMIDITY`/`MAX_HUMIDITY`, `MIN_PH`/`MAX_PH`, `PH_TARGET_MIN`/`PH_TARGET_MAX`, `MIN_EC`/`MAX_EC`, `EC_TARGET_MIN`/`EC_TARGET_MAX`, and the whole `Target (acceptable) ranges` block): these are the compiled defaults for what counts as safe and what counts as ideal for the crop. In practice these are meant to be tuned per crop or per grower from the app, not by editing this file, since they are mirrored into `systemState` and made editable at runtime.

**pH/EC stability filter tuning** (`STABILITY_SAMPLE_WINDOW`, `STABILITY_SAMPLE_INTERVAL_MS`, `PH_STABILITY_TOLERANCE`/`EC_STABILITY_TOLERANCE`, and the whole `pH Temporal Step Filter` block): these control how many agreeing readings it takes, and how tight the agreement has to be, before a pH or EC value is trusted enough to act on. Loosening these makes the system react faster but more likely to act on noise. Tightening them makes it slower but steadier. These should only be re-tuned against observed real noise on the bench, not guessed.

**Fog timing** (`STARTUP_ON_TIME`/`STARTUP_OFF_TIME`, `NORMAL_FOG_ON_TIME`/`NORMAL_FOG_OFF_TIME`, `HOT_FOG_ON_TIME`/`HOT_FOG_OFF_TIME`, `COLD_FOG_ON_TIME`/`COLD_FOG_OFF_TIME`, `BLOWER_PURGE_MS`): how long the fogger runs and rests in each temperature cadence, and how long the blower keeps running afterward to clear residual mist toward the roots.

**Blower and fan PWM** (`BLOWER_SPEED_DEFAULT_PERCENT`/`MIN`/`MAX`, `CANOPY_BLOWER_PWM_FREQUENCY_HZ`, `CANOPY_BLOWER_PWM_RESOLUTION_BITS`): the PWM frequency (200 Hz) is specific to the particular opto-isolated MOSFET driver board in use and was found by bench testing. Changing to a different fan driver board may require re-testing this frequency, since too high a frequency made the fans unable to run below full speed on this hardware (see the comment above `CANOPY_BLOWER_PWM_FREQUENCY_HZ`, and `basilience_canopy_blower_pwm_fix.md`).

**Dosing and correction timing** (`MIXING_DURATION`, `PH_DOSE_COOLDOWN`/`EC_DOSE_COOLDOWN`, `PH_STABILIZATION_TIME`/`EC_STABILIZATION_TIME`, `PH_DOSING_TIME`/`EC_DOSING_TIME`, `MAX_PH_ATTEMPTS`/`MAX_EC_ATTEMPTS`): how long each dose pulse runs, how long the system waits before trusting a post-dose reading, and how many attempts are allowed before giving up and locking the subsystem. These directly affect how aggressively the system chases pH and EC targets, so they should be changed conservatively.

**Water reservoir geometry** (`RESERVOIR_LENGTH_CM`, `RESERVOIR_WIDTH_CM`, `MAX_WORKING_WATER_CM`, `CRITICAL_LOW_WATER_CM`, `REFILL_START_CM`, `REFILL_STOP_CM`): the physical dimensions of the reservoir tank and the depth thresholds that trigger and stop an automatic refill. These are fixed for the current tank design. A different tank would need new values here. `WATER_LEVEL_EMPTY_DISTANCE_CM` is the compiled default for how far the ultrasonic sensor sits above an empty tank, but the actual value used at runtime is `systemState.sensorToBottomCm`, which is meant to be corrected per installation from the app if the sensor is mounted at a different height, without needing a reflash.

**Debug output** (`DEBUG_ENABLED`, `DEBUG_INTERVAL`): turns the periodic Serial Monitor status dashboard on or off, and how often it prints.

`Calibration.h` holds the two sensor calibration values that are specific to the individual probe hardware wired to this exact unit, not to the crop or the growing method:

- `EC_FACTOR`: a correction multiplier applied to the EC probe's raw reading, tuned against a known reference solution.
- `PH_SLOPE` and `PH_OFFSET`: the two numbers from a two-point pH probe calibration (dipping the probe in two buffer solutions of known pH and recording the voltage each one produced). If the physical pH probe is ever replaced, or if it needs a fresh calibration, these two numbers are what would change. The current values and the exact calibration voltages they came from are recorded in the file's own comments and in `basilience_ph_calibration.md`.

## 13. File-by-file reference

- **`BasilienceFirmware.ino`**: the Arduino entry point. Sets up every manager in a specific order at boot and defines the non-blocking main loop.
- **`Config.h`**: compile-time constants for pin numbers, timing, thresholds, and Wi-Fi/Firebase identity. See section 12.
- **`Calibration.h`**: the pH and EC probe calibration numbers specific to this unit's physical sensors.
- **`Types.h`**: every shared data structure and enum in the firmware, including `SensorData`, `SystemState`, `AlertState`, the `SystemMode` state list, and the `Actuator` list.
- **`Version.h`**: the device name and firmware version string.
- **`Globals.h` / `Globals.cpp`**: declares and creates the single shared instance of every manager and shared data structure (`sensors`, `systemState`, `alertState`, and so on) that the rest of the firmware reaches through `extern` references.
- **`SensorManager.h` / `.cpp`**: reads every physical sensor, applies filtering and failure debouncing, and resolves whether physical or mock sensor data is currently authoritative.
- **`AnalogSampler.h` / `.cpp`**: a small reusable helper that repeatedly samples one analog pin and can return the median or average, used underneath the pH and EC readings.
- **`ActuatorManager.h` / `.cpp`**: the only place that ever changes an actuator's physical pin state. Validates every command against safety rules and enforces independent hardware timeouts.
- **`AutomationManager.h` / `.cpp`**: the automatic control brain. Runs the state machine, decides when to fog, dose, refill, or cool, and owns the manual/automatic operation request lifecycle.
- **`SafetyManager.h` / `.cpp`**: answers "is it currently safe to do X" for every automatic and manual action, and handles clearing subsystem locks (Reset Safety).
- **`AlertManager.h` / `.cpp`**: compares live sensor readings against target ranges and raises or clears debounced alert flags.
- **`FirebaseManager.h` / `.cpp`**: everything to do with the Firebase Realtime Database: authentication, uploading sensor/status/alert data, downloading settings and commands, and device provisioning.
- **`WiFiManager.h` / `.cpp`**: owns the Wi-Fi radio, saved credentials, automatic reconnection, and the Wi-Fi setup access point used for provisioning.
- **`RTCManager.h` / `.cpp`**: reads and writes the DS3231 real-time clock, including a network-time (NTP) recovery path if the clock's stored time is invalid and Wi-Fi is available.
- **`GsmManager.h` / `.cpp`**: a low-level driver for the SIM800L GSM module. Knows how to register on the cellular network and send one SMS at a time, with no knowledge of notification policy.
- **`NotificationManager.h` / `.cpp`**: the notification policy layer. Watches for events worth notifying about, maintains the durable SMS queue, and drives the one-recipient-at-a-time SMS fan-out through `GsmManager`.
- **`NotificationTypes.h`**: the shared data structures and enums for the notification queue (event types, severities, delivery status).
- **`FoggingEventQueue.h` / `.cpp`**: a separate durable queue that records every confirmed fogger on/off transition, so a complete fogging history reaches the cloud even if the device was offline when it happened.
- **`FoggingQueueTypes.h`**: the compact data structure used by `FoggingEventQueue`.
- **`SmsRecipientCache.h` / `.cpp`**: a small persisted cache of which phone numbers are currently eligible to receive SMS alerts, synced down from Firebase.
- **`HarvestScheduleCache.h` / `.cpp`**: a small persisted cache of the active growth cycle and its next harvest date, synced down from Firebase. This is what `AutomationManager` checks to decide whether a growth cycle is active at all.
- **`DebugManager.h` / `.cpp`**: prints a rotating status dashboard to the Serial Monitor, and implements "Serial Monitor Focus Mode," which filters that output down to only the subsystem currently being isolated for testing.
- **`StartupManager.h` / `.cpp`**: an empty placeholder class. It is declared but never instantiated or called anywhere in the firmware. All real startup sequencing lives in `AutomationManager`'s `STARTUP` state instead.
- **`MixingManager.h` / `.cpp`**: also an empty placeholder class with no real logic. Mixing behavior (waiting after a dose before rechecking) is actually implemented inline inside `AutomationManager`'s stabilizing states.
- **`Utils.h` / `Utils.cpp`**: both files exist but are currently empty.
