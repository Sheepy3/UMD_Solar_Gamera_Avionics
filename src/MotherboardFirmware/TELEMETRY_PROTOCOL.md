# ELRS telemetry protocol

The motherboard sends project telemetry through CRSF Flight Mode (`0x21`) as
a length-delimited binary carrier. Flight Mode normally contains text, but the
current ELRS hardware path has been verified to return this binary payload,
including embedded zero bytes.

## Primary packet (`0x01`)

The primary packet is 18 complete CRSF bytes: address, length, CRSF type, a
14-byte project payload, and CRC-8/D5.

| Payload offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | Project packet type: `0x01` |
| 1 | 1 | Status flags |
| 2 | 2 | North RPM in unsigned millirpm, big-endian |
| 4 | 2 | East RPM in unsigned millirpm, big-endian |
| 6 | 2 | South RPM in unsigned millirpm, big-endian |
| 8 | 2 | West RPM in unsigned millirpm, big-endian |
| 10 | 4 | Motherboard uptime in milliseconds, big-endian |

Status bit 0 is armed, bit 1 is E-stop lockout, bit 2 is E-stop active, and
bits 3-7 are reserved. Millirpm encoding is `round(RPM * 1000)`, saturated to
the uint16 range; decoding is `value / 1000.0`. This represents 0-65.535 RPM
at 0.001 RPM resolution.

The timestamp is the Pico's unsigned 32-bit `millis()` value. It orders samples
and makes drops visible. It wraps after approximately 49.7 days; a decrease in
normal operation indicates a motherboard restart.

Packet type `0x00` is reserved. Additional telemetry schemas should receive a
new project packet type and remain no larger than 20 complete CRSF bytes under
the currently tested ELRS configuration.

## Transport settings

- Motherboard generation rate: `TELEMETRY_FREQUENCY` in `src/Drone.h`, 1 Hz.
- Pico-to-airborne-receiver UART: 460800 baud.
- Ground PC-to-transmitter UART: 921600 baud in `Clara_Stuff/main4.py`.
- Hardware testing delivered frames of 18, 19, and 20 complete bytes; frames
  of 21 bytes or larger were not returned with the tested ELRS settings.

Hall sensing remains disabled, so RPM fields currently report zero. The
primary packet transport, status flags, timestamps, embedded zero bytes, and
ground-side decoding have been validated on hardware.
