#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include "ArmController.h"
#include "Radio.h"

struct DroneParams{
    int armNPWMPin;
    int armNHallPin;

    int armEPWMPin;
    int armEHallPin;

    int armSPWMPin;
    int armSHallPin;
    
    int armWPWMPin;
    int armWHallPin;

    SerialUSB& serialParam;
    
    SerialUART& radioParam;
    int txPin;
    int rxPin;
};

class Drone {
public:
    Drone(DroneParams& params);

    ArmController armN;
    ArmController armE;
    ArmController armS;
    ArmController armW;

    Radio usbRadio;
    Radio uartRadio;

    void setup();
    void main();

private:
    bool armed;
    bool EStopActive;
    uint32_t EStopTriggerTimeMS = 0;
    uint32_t EStopResetTimeMS;

    uint32_t lastRadioRecieveTimeMS = 0;
    uint32_t lastUSBRecieveTimeMS = 0;

    uint32_t lastSentTelemetry = 0;
    
    bool sendTelemetry();

    void processIncommingFrame(Radio source, uint8_t type, uint8_t* payload, uint8_t len);
};


#endif