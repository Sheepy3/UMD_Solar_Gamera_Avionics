# ELRS telemetry proof of concept

## What was wrong

- The first motherboard PoC sent telemetry as CRSF `RC_CHANNELS_PACKED`
  (`0x16`) on `Serial1`. That type is RC control traffic in the opposite
  direction and was not returned through ELRS even after the ground-side
  receive path was proven operational.
- `main4.py` originally treated bytes returned by the ELRS transmitter as
  newline-terminated UTF-8 and its third-party CRSF parser only accepted a
  `0xC8` address. The module actually returns frames addressed as `0xEA`.
- Commit `b3c0511` removed the duplicate binary telemetry frame from the Pico
  USB port and replaced it with readable debug text. The ELRS UART send was not
  removed. USB debug text does not travel through the ELRS return link.
- Before the same commit, `lastSentTelemetry` was not advanced. That made the
  firmware transmit continuously after the first delay and could saturate the
  receiver-facing UART. It is fixed in the current firmware.
- The old packed-channel frame was 26 bytes and required fragmentation across
  rate-limited telemetry slots. The UQ8.8 proof frame is 34 bytes. The current
  controlled test temporarily sends an 18-byte frame at 1 Hz to determine
  whether frame size is the reason the larger payload is not returned.

## Proof-of-concept payload

The replacement uses standard CRSF `FLIGHT_MODE` telemetry (`0x21`) as a
conservative ASCII carrier. Its null-terminated payload is:

```text
SG2 + 26 hexadecimal characters + NUL
```

- `SG2`: Solar Gamera PoC signature and format version.
- Byte 0: status flags. Bit 0 is armed, bit 1 is E-stop lockout, bit 2 is
  E-stop active, and bits 3-7 are reserved.
- Bytes 1-8: North, East, South, and West RPM as big-endian UQ8.8 values.
- Bytes 9-12: big-endian unsigned 32-bit Pico uptime in milliseconds.

UQ8.8 decoding is `raw / 256.0`, providing approximately 0.00390625 RPM
resolution. The 13 binary bytes are hex encoded because flight mode is defined
as a null-terminated string. The result is a 30-byte payload and a 34-byte CRSF
frame. This remains a transport proof rather than a proposed final schema.

This PoC is merged with the newer firmware channel layout: motor commands use
CRSF channels 1-4 and the Clara command flag uses channel 6. The host regression
test verifies that `main4.py` produces exactly that wire order.

### Exact-size control experiment

With `TELEMETRY_COMPACT_SIZE_TEST` enabled, the firmware sends this payload:

```text
SG3 + 2 status hex characters + 8 timestamp hex characters + NUL
```

This is a 14-byte payload and an 18-byte complete CRSF frame, exactly matching
the size of the previously working SG1 proof. It intentionally omits all four
RPM values so packet length is the only meaningful transport variable. The
full SG2 encoder and decoder remain in place for re-enabling after the test.

Once this carrier is proven on hardware, standard sensor frames should carry
values with standard meanings and a documented project-specific extended frame
can carry arbitrary binary state.

## Wiring and configuration check

The motherboard header labels are from the receiver's perspective:

- header pin 1 / net `RX` / Pico GPIO16: Pico TX to ELRS receiver RX
- header pin 2 / net `TX` / Pico GPIO17: ELRS receiver TX to Pico RX
- header pin 3: +5 V
- header pin 4: ground

Because control reception already works, GPIO17 and the receiver TX path are
known good. Telemetry specifically depends on GPIO16, the receiver RX input,
and the receiver's telemetry configuration. Keep the receiver protocol set to
CRSF and make sure telemetry is not forced off. The firmware retains the
currently working UART rate of 460800 baud.

## Rate and baud settings

- Motherboard telemetry generation rate: `TELEMETRY_FREQUENCY` in `Drone.h`.
  It is currently held at 1 Hz for the exact-size control experiment. After
  transport is characterized, test 5 Hz and then 10 Hz while watching gaps
  and `Motherboard RX rate`; ELRS may overwrite an older queued flight-mode
  sample if this exceeds the configured RF telemetry capacity.
- Airborne Pico-to-receiver UART: `CRSF_BAUD` in `Drone.cpp`, currently 460800.
  The receiver UART setting must match it.
- Ground PC-to-transmitter-module UART: `BAUD` in `main4.py`, currently 921600.
- Controller RC frame request cadence: `SEND_RATE_MS` in `main4.py`, currently
  20 ms / 50 Hz. Tkinter scheduling currently produces a somewhat lower
  measured rate.
- PlatformIO firmware flashing speed is not explicitly overridden in
  `platformio.ini`; the selected Pico upload protocol uses its default. This is
  independent of telemetry throughput.

## End-to-end validation

1. Build and flash `src/MotherboardFirmware`.
2. Set `PORT` and `BAUD` in `src/Clara_Stuff/main4.py` for the ELRS transmitter
   module and run it normally.
3. Watch the once-per-second console diagnostics:
   - Pico USB `[DEBUG] CRSF_FLIGHT_MODE payload=SG3...`: the motherboard
     scheduler generated the new frame.
   - `Serial RX raw`: bytes physically returned to the PC serial port.
   - `CRSF RX rate` or `ELRS link`: the PC is receiving valid binary CRSF from
     the transmitter module.
   - `Motherboard RX: MB PoC ... rpm=[...] timestamp_ms=N ...`: the
     complete return path is working.
4. If the Pico timestamp changes but no motherboard frame reaches the PC, probe
   GPIO16/header pin 1. The size-test transmission begins
   `C8 10 21 53 47 33` and is 18 bytes long. No activity means a motherboard
   TX/pin issue; activity with no returned frame points to receiver
   wiring/configuration or the RF telemetry ratio.

Use the diagnostics in this order:

- `Serial RX raw: 0.0 bytes/s`: nothing is reaching the PC. Check the
  transmitter-breakout `Serial` connection and whether the USB UART supports
  the module's single-wire half-duplex return direction.
- Raw bytes present but wire CRC/discard counts rise: check 921600 baud,
  inversion, voltage level, and bus contention.
- TX echoes are present but non-echo `CRSF RX rate` remains zero: the adapter
  can hear its own uplink, but no response from the ELRS transmitter module is
  reaching the PC.
- Non-echo CRSF frames or link statistics arrive but motherboard telemetry does
  not: the ground-side return path works; focus on the airborne receiver RX,
  Pico GPIO16, and telemetry-ratio configuration.

With `main4.py` stopped so it releases the serial port, query the ELRS
transmitter module directly:

```powershell
cd src\Clara_Stuff
.\.venv\Scripts\python.exe elrs_config.py --port COM8 --baud 921600 --list
```

A device-info/parameter response proves the ground-side bidirectional path.
Verify that the listed `TLM Ratio`/telemetry-ratio setting is not `Off`. No
response means the problem is before the RF telemetry hop: UART wiring,
half-duplex direction control, baud, inversion, or transmitter-module power.

Run the host-side framing regression test with:

```powershell
cd src\Clara_Stuff
.\.venv\Scripts\python.exe -m unittest -v test_telemetry.py
```

For a production protocol, use standard CRSF sensor frame types where possible
and a documented extended frame for project-specific state. The flight-mode
string is intentionally a minimal transport proof compatible with the current
MVP controller.
