#include "LedController.h"

#include <Adafruit_NeoPixel.h>

namespace {
constexpr uint8_t LED_COUNT = 1;
constexpr uint8_t LED_BRIGHTNESS = 63;
constexpr uint8_t STATUS_INTENSITY = 96;
constexpr uint8_t LOCKOUT_INTENSITY = 128;
constexpr uint32_t BLINK_PERIOD_MS = 500;
constexpr uint32_t STARTUP_TEST_MS = 750;
}

LedController::LedController(uint8_t pin)
    : pin(pin), pixel(LED_COUNT, pin, NEO_GRB + NEO_KHZ800) {}

void LedController::setup() {
    pixel.begin();
    pixel.setBrightness(LED_BRIGHTNESS);
    pixel.clear();
    pixel.show();
    initialized = true;
    startupTestEndMS = millis() + STARTUP_TEST_MS;
    show(STATUS_INTENSITY, STATUS_INTENSITY, STATUS_INTENSITY);
}

void LedController::update(LedStatus status) {
    if (!initialized) {
        return;
    }

    const uint32_t nowMS = millis();
    if (nowMS < startupTestEndMS) {
        return;
    }

    const bool statusChanged = forceRefresh || status != lastStatus;
    const bool blinkStateChanged = status == LedStatus::Lockout &&
        nowMS - lastUpdateMS >= BLINK_PERIOD_MS;

    if (!statusChanged && !blinkStateChanged) {
        return;
    }

    lastStatus = status;
    lastUpdateMS = nowMS;
    forceRefresh = false;

    switch (status) {
    case LedStatus::Disarmed:
        show(0, 0, STATUS_INTENSITY);       // Blue: normally disarmed.
        break;
    case LedStatus::Lockout:
        if ((nowMS / BLINK_PERIOD_MS) % 2 == 0) {
            show(LOCKOUT_INTENSITY, 0, 0);   // Blinking red: e-stop lockout in progress.
        } else {
            show(0, 0, 0);
        }
        break;
    case LedStatus::LockoutExitedUnarmed:
        show(LOCKOUT_INTENSITY, STATUS_INTENSITY / 2, 0); // Amber: lockout cleared, waiting to arm.
        break;
    case LedStatus::Armed:
        show(0, STATUS_INTENSITY, 0);       // Green: armed at zero throttle.
        break;
    case LedStatus::ThrottleActive:
        show(LOCKOUT_INTENSITY, 0, LOCKOUT_INTENSITY); // Magenta: armed and commanding motor PWM.
        break;
    }
}

void LedController::show(uint8_t red, uint8_t green, uint8_t blue) {
    pixel.setPixelColor(0, pixel.Color(red, green, blue));
    pixel.show();
}
