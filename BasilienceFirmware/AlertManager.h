#ifndef ALERT_MANAGER_H
#define ALERT_MANAGER_H


class AlertManager
{
public:
    void begin();

    void update();

    bool isDirty() const;

    void markSynced();

private:
    bool alertsDirty = true;

    void setAlert(const char* name, bool& currentValue, bool nextValue);

    void updateLowWaterAlert();

    void updateTemperatureAlert();

    void updateWaterTemperatureAlert();

    void updatePHAlert();

    void updateECAlert();

    void updateSensorFaultAlert();
};

#endif
