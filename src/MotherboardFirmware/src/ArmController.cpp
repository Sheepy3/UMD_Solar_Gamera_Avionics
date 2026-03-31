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
    uint32_t currentTime = millis();
    hallTimes[hallIndex] = currentTime;
    hallIndex = (hallIndex + 1) % BUFFER_SIZE;
    if (numValidHallSamples < BUFFER_SIZE) numValidHallSamples++;
    lastPulseTime = currentTime;
}

float ArmController::getThrottle(){
    return throttle;
}

void ArmController::setThrottle(float inputThrottle) {
    if (throttle <= 0.01 && inputThrottle > 0.01) {
        lastZeroThrottleTimeMS = millis();
    }

    inputThrottle = constrain(inputThrottle, 0.0, 1.0);

    throttle = inputThrottle;

    uint8_t servoSetting = throttle * 180;
    escServo.write(servoSetting);
}

void ArmController::stop() {
    setThrottle(0.0);
}

float ArmController::getRPM(uint8_t samples = COUNTS_PER_REVOLUTION) {
    uint32_t now = millis();
    
    noInterrupts();
    uint32_t lastPulse = lastPulseTime;
    uint8_t validCount = numValidHallSamples;
    uint8_t currentIndex = hallIndex;
    interrupts();

    // Check if motor is stopped
    if (validCount < 2 || (now - lastPulse > STOP_TIMEOUT_MS)) {
        return 0.0;
    }

    uint8_t count = min(samples, min(validCount - 1, BUFFER_SIZE - 1));

    uint8_t latestIdx = (currentIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    uint8_t earliestIdx = (latestIdx - count + BUFFER_SIZE) % BUFFER_SIZE;
    
    noInterrupts();
    uint32_t endTime = hallTimes[latestIdx];
    uint32_t startTime = hallTimes[earliestIdx];
    interrupts();

    uint32_t timeSpan = endTime - startTime;
    if (timeSpan == 0) return 0.0;

    float rpm = (float)count * 60.0f * 1000.0f / (float)timeSpan / COUNTS_PER_REVOLUTION;
    return rpm;
}

bool ArmController::isStalled() {
    if (throttle < 0.10f) {
        return false;
    }

    if (millis() - lastZeroThrottleTimeMS < 200UL) {
        return false;
    }

    noInterrupts();
    uint32_t last = lastPulseTime;
    interrupts();

    uint32_t now = millis();

    if (now - last > STALL_TIMEOUT_MS) {
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