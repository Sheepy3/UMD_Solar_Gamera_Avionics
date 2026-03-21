#include "ArmController.h"

const unsigned long STOP_TIMEOUT_MICROS = 3000000UL;

ArmController* ArmController::instance = nullptr;

ArmController::ArmController(int pwmPin, int hallPin) 
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
    unsigned long currentTime = micros();
    _hallTimes[_hallIndex] = currentTime;
    _hallIndex = (_hallIndex + 1) % 10;
    if (_validHallSamples < 10) _validHallSamples++;
    _lastPulseTime = currentTime;
}

void ArmController::setThrottle(float throttle) {
    _throttle = constrain(throttle, 0.0, 1.0);

    int pulseWidth = _throttle * 180;
    _escServo.write(pulseWidth);
}

void ArmController::stop() {
    setThrottle(0.0);
}

float ArmController::getRPM(int samples) {
    unsigned long now = micros();
    
    noInterrupts();
    unsigned long lastPulse = _lastPulseTime;
    int validCount = _validHallSamples;
    int currentIndex = _hallIndex;
    interrupts();

    // Check if motor is stopped
    if (validCount < 2 || (now - lastPulse > STOP_TIMEOUT_MICROS)) {
        return 0.0;
    }

    int count = samples;
    if (count >= validCount) count = validCount - 1;
    if (count >= BUFFER_SIZE) count = BUFFER_SIZE - 1;

    int latestIdx = (currentIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    int earliestIdx = (latestIdx - count + BUFFER_SIZE) % BUFFER_SIZE;
    
    noInterrupts();
    unsigned long endTime = _hallTimes[latestIdx];
    unsigned long startTime = _hallTimes[earliestIdx];
    interrupts();

    unsigned long timeSpan = endTime - startTime;
    if (timeSpan == 0) return 0.0;

    float rpm = (float)count * 6.0E7F / (float)timeSpan / COUNTS_PER_REVOLUTION;
    return rpm;
}

void ArmController::resetHallTimes() {
    noInterrupts();
    for (int i = 0; i < BUFFER_SIZE; i++) {
        _hallTimes[i] = 0;
    }
    _validHallSamples = 0;
    _hallIndex = 0;
    _lastPulseTime = 0;
    interrupts();
}