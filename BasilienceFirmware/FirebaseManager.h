#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <FirebaseESP32.h>

class FirebaseManager
{
public:

    void begin();

    void update();

private:

    FirebaseData fbdo;

    FirebaseAuth auth;

    FirebaseConfig config;
};

#endif