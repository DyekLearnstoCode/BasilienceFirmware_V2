#ifndef AUTOMATION_MANAGER_H
#define AUTOMATION_MANAGER_H

#include "Types.h"

class AutomationManager
{
public:

    void begin();

    void update();

private:

    enum StartupPhase
    {
        FOGGING_PHASE,
        REST_PHASE
    };

    StartupPhase startupPhase;

    void handleStartup();
    void handleNormal();
    void changeState(SystemMode newMode);
    void handleRefilling();
    const char* getStateName(SystemMode mode);
};

#endif