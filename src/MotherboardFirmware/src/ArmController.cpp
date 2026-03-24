#include "ArmController.h"



ArmController* ArmController::instance = nullptr;

ArmController::ArmController(uint8_t pwmPin, uint8_t hallPin) 
    : pwmPin(pwmPin), hallPin(hallPin), numValidHallSamples(0), hallIndex(0), throttle(0) {}

void ArmController::setup() {
    resetHallTimes();

    pinMode(pwmPin, OUTPUT);
    pinMode(hallPin, INPUT_PULLUP);

    escServo.attach(pwmPin);
    escServo.write(0);

    instance = this;
    attachInterrupt(digitalPinToInterrupt(hallPin), ArmController::isrWrapper, RISING);
}

void ArmController::isrWrapper() {
    if (instance != nullptr) {
        instance->updateHallTimes();
    }
}

void ArmController::updateHallTimes() {
    uint32_t currentTime = micros();
    hallTimes[hallIndex] = currentTime;
    hallIndex = (hallIndex + 1) % BUFFER_SIZE;
    if (numValidHallSamples < BUFFER_SIZE) numValidHallSamples++;
    lastPulseTime = currentTime;
}

float ArmController::getThrottle(){
    return throttle;
}

void ArmController::setThrottle(float setThrottle) {
    if (throttle <= 0.01 && setThrottle > 0.01) {
        lastZeroThrottleTimeMS = millis();
    }

    setThrottle = constrain(setThrottle, 0.0, 1.0);
    throttle = setThrottle;

    uint8_t servoSetting = setThrottle * 180;
    escServo.write(servoSetting);
}

void ArmController::stop() {
    setThrottle(0.0);
}

float ArmController::getRPM(uint8_t samples) {
    uint32_t now = micros();
    
    noInterrupts();
    uint32_t lastPulse = lastPulseTime;
    uint8_t validCount = numValidHallSamples;
    uint8_t currentIndex = hallIndex;
    interrupts();

    // Check if motor is stopped
    if (validCount < 2 || (now - lastPulse > STOP_TIMEOUT_MICROS)) {
        return 0.0;
    }

    uint8_t count = samples;
    if (count >= validCount) count = validCount - 1;
    if (count >= BUFFER_SIZE) count = BUFFER_SIZE - 1;

    uint8_t latestIdx = (currentIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    uint8_t earliestIdx = (latestIdx - count + BUFFER_SIZE) % BUFFER_SIZE;
    
    noInterrupts();
    uint32_t endTime = hallTimes[latestIdx];
    uint32_t startTime = hallTimes[earliestIdx];
    interrupts();

    uint32_t timeSpan = endTime - startTime;
    if (timeSpan == 0) return 0.0;

    float rpm = (float)count * 6.0E7F / (float)timeSpan / COUNTS_PER_REVOLUTION;
    return rpm;
}

bool ArmController::isStalled() {
    if (throttle < 0.10f) {
        return false;
    }

    if (millis() - lastZeroThrottleTimeMS < 200) {
        return false;
    }

    noInterrupts();
    uint32_t last = lastPulseTime;
    interrupts();

    uint32_t now = micros();

    if (now - last > STALL_TIMEOUT_MICROS) {
        return true;
    }

    return false;
}

void ArmController::resetHallTimes() {
    noInterrupts();
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        hallTimes[i] = 0;
    }
    numValidHallSamples = 0;
    hallIndex = 0;
    lastPulseTime = 0;
    interrupts();
}