#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include "ArmController.h"

class Drone {
public:
    Drone(DroneParams params);

    ArmController armN;
    ArmController armE;
    ArmController armS;
    ArmController armW;

    SerialUSB usbSerial;
    SerialUART radio;

    void setup();
    void main();

private:
    bool armed;
    bool EStopActive;
    uint32_t EStopTriggerTimeMS;
    uint32_t EStopResetTimeMS;
    uint32_t lastControlRecieveTimeMS;
};

struct DroneParams{
    int armNPWMPin;
    int armNHallPin;
    int armEPWMPin;
    int armEHallPin;
    int armSPWMPin;
    int armSHallPin;
    int armWPWMPin;
    int armWHallPin;
    SerialUSB serialParam;
    SerialUART radioParam;
    int txPin;
    int rxPin;
};

#endif