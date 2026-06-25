#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ArmController {
public:
    ArmController(uint8_t pwmPin, uint8_t hallPin);

    void setup();
    float getThrottle();
    void setThrottle(float inputThrottle); // 0.0 to 1.0
    void stop();

    float getRPM(uint8_t samples = COUNTS_PER_REVOLUTION);
    bool isStalled();

    void resetHallTimes();
    
private:
    static const uint8_t COUNTS_PER_REVOLUTION = 4;
    static const uint8_t BUFFER_MULT = 2;
    static const uint8_t BUFFER_SIZE = BUFFER_MULT * COUNTS_PER_REVOLUTION + 1;
    static const uint32_t STOP_TIMEOUT_MS = 3000UL;
    static const uint32_t STALL_TIMEOUT_MS = 3000UL;

    uint8_t pwmPin;
    uint8_t hallPin;
    
    // Volatiles for Interrupt
    volatile uint8_t numValidHallSamples;
    volatile uint32_t hallTimes[BUFFER_SIZE];
    volatile uint8_t hallIndex;
    volatile uint32_t lastPulseTime;

    float throttle;
    uint32_t lastZeroThrottleTimeMS;
    Servo escServo;

    void updateHallTimes();
    static void isrWrapper();
    static ArmController* instance;
};

#endif