#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "ArmController.h"
#include "LedController.h"
#include "Radio.h"
#include "util.h"

// true:  [DEBUG] CRSF_CHANNELS [132, 0, ...]
// false: [DEBUG] armed=false estop=true ...
static constexpr bool DEBUG_RAW_CRSF_CHANNELS = false;
static constexpr bool DEBUG_INCOMING_CRSF = false;

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
    LINK_STATISTICS = 0x14,
    FLIGHT_MODE = 0x21
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

    int statusLedPin;

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
    LedController statusLed;

    Radio usbRadio;
    Radio uartRadio;

    void setup();
    void main();
    void triggerEStop(const char* reason);
    
private:
    SerialUSB& debugSerial;

    uint32_t nowMS = 0;

    bool armed = false;
    bool EStopActive = false;
    uint32_t EStopTriggerTimeMS = 0;
    uint32_t lastEStopLogTimeMS = 0;
    const char* lastEStopReason = nullptr;

    uint32_t lastUSBRecieveTimeMS = 0;
    uint32_t lastUARTRecieveTimeMS = 0;
    uint16_t rawThrottleChannels[4] = {0, 0, 0, 0};
    
    // Keep the proof-of-concept rate modest so it also works with
    // conservative ELRS telemetry ratios.
    static const uint8_t TELEMETRY_FREQUENCY = 2;
    static const uint32_t TELEMETRY_DELAY = 1000L / TELEMETRY_FREQUENCY;
    static const uint32_t TIMEOUT_MS = 1000L;
    static const uint32_t ESTOP_LOCKOUT_MS = 10000L;

    uint32_t lastSentTelemetry = 0;
    uint16_t telemetrySequence = 0;
    uint32_t lastIncomingCRSFLogTimeMS = 0;

    void sendTelemetry();
    void printDebugTelemetry(const char* payload);
    void printEStopReason(const char* reason);
    void printIncomingCRSF(const char* sourceName, const uint32_t sourceDtMS, const uint8_t type, const uint8_t* payload, const uint8_t len, const uint16_t* channels, bool ignored);
    void printHexByte(uint8_t value);
    LedStatus getLedStatus() const;
    bool isThrottleActive() const;

    void processIncommingFrame(Radio& source, const uint8_t type, const uint8_t* payload, const uint8_t len);
};

#endif
