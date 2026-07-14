#ifndef STARTUP_MANAGER_H
#define STARTUP_MANAGER_H

#include "Types.h" // or wherever StartupState is defined

class StartupManager
{
public:
    StartupManager();

    void begin();
    void update();

private:
    StartupState state;
    unsigned long startTime;
};

#endif