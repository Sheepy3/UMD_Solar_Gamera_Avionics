#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ArmController {
public:
    static const int COUNTS_PER_REVOLUTION = 4;
    static const int BUFFER_MULT = 4;
    static const int BUFFER_SIZE = BUFFER_MULT * COUNTS_PER_REVOLUTION;

    ArmController(int pwmPin, int hallPin);

    void setup();
    void setThrottle(float throttle); // 0.0 to 1.0
    void stop();

    float getRPM(int samples = 10);
    void resetHallTimes();
    
private:
    int _pwmPin;
    int _hallPin;
    
    // Volatiles for Interrupt
    volatile int _validHallSamples;
    volatile unsigned long _hallTimes[BUFFER_SIZE];
    volatile int _hallIndex;
    volatile unsigned long _lastPulseTime;

    float _throttle;
    Servo _escServo;

    void updateHallTimes();
    static void isrWrapper();
    static ArmController* instance;
};

#endif