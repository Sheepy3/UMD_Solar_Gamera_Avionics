#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "ArmController.h"
#include "Radio.h"
#include "util.h"

// CRSF Address definitions
enum DestType : uint8_t {
    REMOTE_CONTROL = 0xEA,
    TRANSMITTER = 0xEE,
    GROUND_STATION = 0xC8,
    GPS = 0xC2 //Unused
};

// CRSF Protocol Constants
enum PayloadType : uint8_t {
    RC_CHANNELS_PACKED = 0x16,
    BATTERY_SENSOR = 0x08,
    ALTITUDE = 0x1E,
    LINK_STATISTICS = 0x14
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
    void triggerEStop();
    
private:
    uint32_t nowMS;

    bool armed;
    bool EStopActive;
    uint32_t EStopTriggerTimeMS;

    uint32_t lastUSBRecieveTimeMS = 0;
    uint32_t lastUARTRecieveTimeMS = 0;
    
    static const uint8_t TELEMETRY_FREQUENCY = 10;
    static const uint32_t TELEMETRY_DELAY = 1000L / TELEMETRY_FREQUENCY;
    static const uint32_t TIMEOUT_MS = 1000L;
    static const uint32_t ESTOP_LOCKOUT_MS = 10000L;

    uint32_t lastSentTelemetry = 0;
    
    void sendTelemetry();

    void processIncommingFrame(Radio& source, const uint8_t type, const uint8_t* payload, const uint8_t len);
};

#endif