#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H


enum class SafetyResult
{
    SAFE,

    LOW_WATER,

    SENSOR_FAULT,

    HIGH_WATER_TEMPERATURE,

    SAFETY_LOCK,

    RESERVOIR_LOCK
};

class SafetyManager
{
public:
    void begin();

    void update();

    SafetyResult canDosePH() const;

    SafetyResult canDoseEC() const;

    SafetyResult canRefill() const;

    SafetyResult canFog() const;

    SafetyResult canCool() const;
};



#endif