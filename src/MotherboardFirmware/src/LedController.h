#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

enum class LedStatus : uint8_t {
    Disarmed,
    Lockout,
    LockoutExitedUnarmed,
    Armed,
    ThrottleActive,
};

class LedController {
public:
    explicit LedController(uint8_t pin);

    void setup();
    void update(LedStatus status);

private:
    uint8_t pin;
    LedStatus lastStatus = LedStatus::Disarmed;
    uint32_t lastUpdateMS = 0;
    uint32_t startupTestEndMS = 0;
    bool initialized = false;
    bool forceRefresh = true;
    Adafruit_NeoPixel pixel;

    void show(uint8_t red, uint8_t green, uint8_t blue);
};

#endif
