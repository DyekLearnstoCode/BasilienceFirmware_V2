#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H
#include <Arduino.h>
#include "Types.h"

class ActuatorManager
{
public:
    void begin();
    void update();
    void requestCommand(Actuator actuator, bool state, const String& source, double timestamp, uint8_t speed = 100);

    void turnOn(Actuator actuator);

    void turnOff(Actuator actuator);

    void toggle(Actuator actuator);

    bool isOn(Actuator actuator) const;

    void turnOffAll();

    ActuatorStatus getStatus(Actuator actuator) const;

private:
    bool actuatorStates[ACTUATOR_COUNT];
    ActuatorCommand commands[ACTUATOR_COUNT];
    ActuatorStatus statuses[ACTUATOR_COUNT];

    uint8_t getPin(Actuator actuator) const;
    bool validateCommand(Actuator actuator, bool targetState, String& outReason);
};

#endif