#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H
#include <Arduino.h>
#include "esp_timer.h"
#include "Types.h"

class ActuatorManager
{
public:
    void begin();
    void update();
    void requestCommand(Actuator actuator, bool state, const String& source, double timestamp, uint8_t speed = 100, const String& strategy = "", const String& reason = "", bool overrideRequested = false, bool bypassAutoFoggerGate = false);

    void turnOn(Actuator actuator);

    void turnOff(Actuator actuator);

    void toggle(Actuator actuator);

    bool isOn(Actuator actuator) const;

    void turnOffAll(const String& reason = "");

    ActuatorStatus getStatus(Actuator actuator) const;

    bool isStatusDirty() const;

    void markStatusSynced();

private:
    bool statusDirty = true;

    bool actuatorStates[ACTUATOR_COUNT];
    ActuatorCommand commands[ACTUATOR_COUNT];
    ActuatorStatus statuses[ACTUATOR_COUNT];
    // True while manual ownership is holding an actuator (ON or OFF) against
    // routine automatic requests - see requestCommand()'s manual-source
    // handling and its "Ignore automatic schedule commands" guard. Cleared on
    // release (manual mode off, hold expiry, explicit re-command, rejection)
    // - see update()'s STOPPING handling and the independent-deadline path.
    bool manuallyOverridden[ACTUATOR_COUNT];
    // Edge-detection only, so the "[AUTO] ... skipped: manual ownership
    // active" log fires once per hold rather than every automation tick -
    // see requestCommand()'s manual-hold guard.
    bool automaticSkipLogged[ACTUATOR_COUNT];

    // Independent physical-safety deadline for a manual run of an actuator
    // that already carries a hard manual runtime cap (dosing pumps, solenoid,
    // peltier, fogger - see MANUAL_PUMP_RUNTIME/OPERATION_TIMEOUT_MS in
    // Config.h). Backed by one esp_timer per actuator so the GPIO gets forced
    // OFF on schedule even if loop() is stalled inside a long-blocking
    // Firebase call. Re-armed on every manual RUNNING start and disarmed on
    // every transition away from running, so a stale timer can never fire
    // into a later, unrelated command - see ActuatorManager.cpp.
    esp_timer_handle_t deadlineTimers[ACTUATOR_COUNT] = { nullptr };
    // Set ONLY by onDeadlineExpired() (the esp_timer service task context,
    // not loop()); cleared ONLY by update() on the next tick it observes
    // true. One flag per actuator, each with exactly one writer and one
    // reader, so a plain volatile flag needs no additional lock. The
    // callback touches nothing else - no Strings, no Firebase, no other
    // ActuatorManager state - see onDeadlineExpired().
    volatile bool deadlineExpired[ACTUATOR_COUNT] = { false };

    // Change-detection for the [CANOPY-PWM]/[BLOWER-PWM] diagnostics (real-
    // hardware PWM verification follow-up) - only CANOPY_FAN/BLOWER indices
    // are ever used (see isPwmActuator()), but sized per-actuator for a
    // direct index match with commands[]/statuses[]. -1 means "never
    // logged yet", so the first PWM application always logs once; set in
    // begin()'s existing per-actuator init loop rather than here, so this
    // doesn't need to know ACTUATOR_COUNT's exact value at declaration.
    int16_t lastLoggedPwmPercent[ACTUATOR_COUNT];

    uint8_t getPin(Actuator actuator) const;
    bool validateCommand(Actuator actuator, bool targetState, String& outReason, bool runningValidation = false);
    void armDeadline(Actuator actuator, unsigned long durationMs);
    void disarmDeadline(Actuator actuator);
    static void onDeadlineExpired(void* arg);
};

#endif
