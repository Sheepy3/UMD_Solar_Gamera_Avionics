#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ArmController {
public:
    static const uint8_t COUNTS_PER_REVOLUTION = 4;
    static const uint8_t BUFFER_MULT = 4;
    static const uint8_t BUFFER_SIZE = BUFFER_MULT * COUNTS_PER_REVOLUTION;

    ArmController(uint8_t pwmPin, uint8_t hallPin);

    void setup();
    void setThrottle(float throttle); // 0.0 to 1.0
    void stop();

    float getRPM(uint8_t samples = 10);
    void resetHallTimes();
    
private:
    uint8_t _pwmPin;
    uint8_t _hallPin;
    
    // Volatiles for Interrupt
    volatile uint8_t _validHallSamples;
    volatile uint32_t _hallTimes[BUFFER_SIZE];
    volatile uint8_t _hallIndex;
    volatile uint32_t _lastPulseTime;

    float _throttle;
    Servo _escServo;

    void updateHallTimes();
    static void isrWrapper();
    static ArmController* instance;
};

#endif