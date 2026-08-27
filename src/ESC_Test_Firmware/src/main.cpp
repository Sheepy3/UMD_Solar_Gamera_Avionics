#include <Arduino.h>

// Hall inputs used by MotherboardFirmware. East and west intentionally follow
// the firmware's corrected physical-arm mapping rather than the ECAD labels.
static constexpr uint8_t NORTH_HALL_PIN = 26;
static constexpr uint8_t EAST_HALL_PIN = 21;
static constexpr uint8_t SOUTH_HALL_PIN = 19;
static constexpr uint8_t WEST_HALL_PIN = 15;

static constexpr uint8_t SENSOR_COUNT = 4;
static constexpr uint8_t MAGNETS_PER_REVOLUTION = 1;
static constexpr uint32_t PRINT_INTERVAL_MS = 100;
static constexpr uint32_t MIN_LOW_PULSE_US = 5000;
static constexpr uint32_t MIN_STOP_TIMEOUT_US = 2000000;

uint32_t lastPrintTime = 0;
volatile uint32_t lowStartTimeUs[SENSOR_COUNT] = {};
volatile uint32_t lastPulseTimeUs[SENSOR_COUNT] = {};
volatile uint32_t pulsePeriodUs[SENSOR_COUNT] = {};
volatile uint32_t pulseCount[SENSOR_COUNT] = {};
volatile uint32_t rejectedGlitchCount[SENSOR_COUNT] = {};

void recordPinChange(uint8_t sensorIndex, uint8_t pin) {
  const uint32_t now = micros();

  if (digitalRead(pin) == LOW) {
    lowStartTimeUs[sensorIndex] = now;
    return;
  }

  const uint32_t lowStart = lowStartTimeUs[sensorIndex];
  if (lowStart == 0) {
    return;
  }
  lowStartTimeUs[sensorIndex] = 0;

  // Count only a complete magnet pulse. The captured output showed false LOW
  // pulses at roughly 1.4-2.8 ms intervals, so reject anything under 5 ms.
  if (now - lowStart < MIN_LOW_PULSE_US) {
    rejectedGlitchCount[sensorIndex]++;
    return;
  }

  const uint32_t previous = lastPulseTimeUs[sensorIndex];
  if (previous != 0) {
    pulsePeriodUs[sensorIndex] = lowStart - previous;
  }
  lastPulseTimeUs[sensorIndex] = lowStart;
  pulseCount[sensorIndex]++;
}

void northChangeISR() { recordPinChange(0, NORTH_HALL_PIN); }
void eastChangeISR() { recordPinChange(1, EAST_HALL_PIN); }
void southChangeISR() { recordPinChange(2, SOUTH_HALL_PIN); }
void westChangeISR() { recordPinChange(3, WEST_HALL_PIN); }

void printHallReading(const char* name, uint8_t pin, uint8_t sensorIndex) {
  noInterrupts();
  const uint32_t lastPulse = lastPulseTimeUs[sensorIndex];
  const uint32_t period = pulsePeriodUs[sensorIndex];
  const uint32_t count = pulseCount[sensorIndex];
  const uint32_t glitches = rejectedGlitchCount[sensorIndex];
  interrupts();

  Serial.print(name);
  Serial.print("=");

  if (period == 0) {
    Serial.print("WAIT");
  } else {
    const uint32_t timeSincePulse = micros() - lastPulse;
    const uint64_t adaptiveTimeout = static_cast<uint64_t>(period) * 3ULL;
    const uint32_t stopTimeout = adaptiveTimeout > MIN_STOP_TIMEOUT_US
        ? static_cast<uint32_t>(min(adaptiveTimeout, 0xFFFFFFFFULL))
        : MIN_STOP_TIMEOUT_US;

    if (timeSincePulse > stopTimeout) {
      Serial.print("0.0rpm");
    } else {
      const float rpm = 60000000.0f /
          (static_cast<float>(period) * MAGNETS_PER_REVOLUTION);
      Serial.print(rpm, 1);
      Serial.print("rpm");
    }
  }

  Serial.print(" raw=");
  Serial.print(digitalRead(pin) == HIGH ? "HIGH" : "LOW");
  Serial.print(" pulses=");
  Serial.print(count);
  Serial.print(" glitches=");
  Serial.print(glitches);
}

void setup() {
  Serial.begin(115200);

  // The A3144 output is open collector and requires a pull-up. Enabling the
  // Pico-side pull-up also prevents a weak/floating level-shifter output from
  // generating thousands of false edges.
  pinMode(NORTH_HALL_PIN, INPUT_PULLUP);
  pinMode(EAST_HALL_PIN, INPUT_PULLUP);
  pinMode(SOUTH_HALL_PIN, INPUT_PULLUP);
  pinMode(WEST_HALL_PIN, INPUT_PULLUP);

  // Watch both edges so a LOW must last long enough to be a real magnet pass
  // before it is included in the RPM calculation.
  attachInterrupt(digitalPinToInterrupt(NORTH_HALL_PIN), northChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EAST_HALL_PIN), eastChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SOUTH_HALL_PIN), southChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WEST_HALL_PIN), westChangeISR, CHANGE);

  Serial.println("Hall sensor RPM test");
  Serial.println("A3144: LOW = magnet detected, HIGH = no magnet");
  Serial.println("Pins: North=26 East=21 South=19 West=15");
  Serial.println("Assuming one magnet per shaft revolution; two passes are needed for the first RPM reading.");
  Serial.println("LOW pulses shorter than 5 ms are reported as glitches and excluded from RPM.");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintTime = now;

  printHallReading("North", NORTH_HALL_PIN, 0);
  Serial.print("  ");
  printHallReading("East", EAST_HALL_PIN, 1);
  Serial.print("  ");
  printHallReading("South", SOUTH_HALL_PIN, 2);
  Serial.print("  ");
  printHallReading("West", WEST_HALL_PIN, 3);
  Serial.println();
}
