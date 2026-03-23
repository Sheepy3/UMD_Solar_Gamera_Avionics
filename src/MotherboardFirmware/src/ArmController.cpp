#include "ArmController.h"

const uint32_t STOP_TIMEOUT_MICROS = 3000000UL;

ArmController* ArmController::instance = nullptr;

ArmController::ArmController(uint8_t pwmPin, uint8_t hallPin) 
    : _pwmPin(pwmPin), _hallPin(hallPin), _validHallSamples(0), _hallIndex(0), _throttle(0) {}

void ArmController::setup() {
    resetHallTimes();

    pinMode(_pwmPin, OUTPUT);
    pinMode(_hallPin, INPUT_PULLUP);

    _escServo.attach(_pwmPin);
    _escServo.write(0);

    instance = this;
    attachInterrupt(digitalPinToInterrupt(_hallPin), ArmController::isrWrapper, RISING);
}

void ArmController::isrWrapper() {
    if (instance != nullptr) {
        instance->updateHallTimes();
    }
}

void ArmController::updateHallTimes() {
    uint32_t currentTime = micros();
    _hallTimes[_hallIndex] = currentTime;
    _hallIndex = (_hallIndex + 1) % BUFFER_SIZE;
    if (_validHallSamples < BUFFER_SIZE) _validHallSamples++;
    _lastPulseTime = currentTime;
}

void ArmController::setThrottle(float throttle) {
    _throttle = constrain(throttle, 0.0, 1.0);

    uint8_t pulseWidth = _throttle * 180;
    _escServo.write(pulseWidth);
}

void ArmController::stop() {
    setThrottle(0.0);
}

float ArmController::getRPM(uint8_t samples) {
    uint32_t now = micros();
    
    noInterrupts();
    uint32_t lastPulse = _lastPulseTime;
    uint8_t validCount = _validHallSamples;
    uint8_t currentIndex = _hallIndex;
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
    uint32_t endTime = _hallTimes[latestIdx];
    uint32_t startTime = _hallTimes[earliestIdx];
    interrupts();

    uint32_t timeSpan = endTime - startTime;
    if (timeSpan == 0) return 0.0;

    float rpm = (float)count * 6.0E7F / (float)timeSpan / COUNTS_PER_REVOLUTION;
    return rpm;
}

void ArmController::resetHallTimes() {
    noInterrupts();
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        _hallTimes[i] = 0;
    }
    _validHallSamples = 0;
    _hallIndex = 0;
    _lastPulseTime = 0;
    interrupts();
}