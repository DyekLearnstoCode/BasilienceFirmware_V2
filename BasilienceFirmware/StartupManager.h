#ifndef STARTUP_MANAGER_H
#define STARTUP_MANAGER_H

StartupManager::StartupManager()
{
    state = STARTUP_FOGGING;

    startTime = 0;
}

class StartupManager
{
public:
    void begin();
    void update();
};

private:

    StartupState state;

    unsigned long startTime;


#endif