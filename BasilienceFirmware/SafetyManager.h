#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "Types.h"

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
    SafetyResult canResetSafety() const;

    const char* getSafetyReason(
        SafetyResult result) const;
};



#endif