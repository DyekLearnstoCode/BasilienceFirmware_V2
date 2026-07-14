#include "FirebaseManager.h"

#include "Config.h"

void FirebaseManager::begin()
{
    config.api_key = API_KEY;

    config.database_url = DATABASE_URL;

    Firebase.begin(&config, &auth);

    Firebase.reconnectWiFi(true);
}

void FirebaseManager::update()
{
}