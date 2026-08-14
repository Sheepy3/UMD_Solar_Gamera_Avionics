#include <Arduino.h>
#include <Servo.h>

// This intentionally matches the known-good ESC sweep sketch, but runs on the
// Motherboard North arm PWM output (GPIO 20).
static constexpr uint8_t ESC_SIGNAL_PIN = 20;
static constexpr uint16_t ESC_MIN_PULSE_US = 1060;
static constexpr uint16_t ESC_MAX_PULSE_US = 2290;

Servo esc;
bool highThrottle = true;

void setThrottle(uint8_t throttle) {
  // Use the same API path as the known-good Arduino sketch.
  esc.write(throttle);

  const uint16_t pulseWidthUS = ESC_MIN_PULSE_US +
      static_cast<uint16_t>(throttle * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US) / 180UL);

  Serial.print("throttle=");
  Serial.print(throttle);
  Serial.print(" pulse_us=");
  Serial.println(pulseWidthUS);
}

void setup() {
  Serial.begin(115200);
  esc.attach(ESC_SIGNAL_PIN, ESC_MIN_PULSE_US, ESC_MAX_PULSE_US);

  // For ESC calibration, power the ESC only after this maximum-throttle signal
  // is present. Each serial character toggles between maximum and minimum.
  setThrottle(180);
  Serial.println("Holding maximum throttle. Send any character to toggle maximum/minimum throttle.");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  while (Serial.available() > 0) {
    Serial.read();
  }

  highThrottle = !highThrottle;
  if (highThrottle) {
    setThrottle(180);
    Serial.println("Maximum throttle set.");
  } else {
    setThrottle(0);
    Serial.println("Minimum throttle set.");
  }
}
