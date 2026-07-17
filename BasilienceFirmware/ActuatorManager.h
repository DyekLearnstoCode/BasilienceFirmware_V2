#ifndef ACTUATOR_MANAGER_H
#define ACTUATOR_MANAGER_H
#include <Arduino.h>
#include "Types.h"

class ActuatorManager
{
public:
    void begin();

    void turnOn(Actuator actuator);

    void turnOff(Actuator actuator);

    void toggle(Actuator actuator);

    bool isOn(Actuator actuator) const;

    void turnOffAll();

private:
    bool actuatorStates[ACTUATOR_COUNT];

    uint8_t getPin(Actuator actuator) const;
};

#endif