#ifndef ALERT_MANAGER_H
#define ALERT_MANAGER_H


class AlertManager
{
public:
    void begin();

    void update();

private:
    void updateLowWaterAlert();

    void updateTemperatureAlert();

    void updateWaterTemperatureAlert();

    void updatePHAlert();

    void updateECAlert();

    void updateSensorFaultAlert();
};

#endif