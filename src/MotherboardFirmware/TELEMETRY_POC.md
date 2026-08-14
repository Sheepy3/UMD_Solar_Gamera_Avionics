# ELRS telemetry proof of concept

## What was wrong

- The motherboard sends telemetry as a binary CRSF `RC_CHANNELS_PACKED`
  frame on `Serial1`. `main4.py` was treating all bytes returned by the ELRS
  transmitter as newline-terminated UTF-8, so it could never recognize that
  frame.
- Commit `b3c0511` removed the duplicate binary telemetry frame from the Pico
  USB port and replaced it with readable debug text. The ELRS UART send was not
  removed. USB debug text does not travel through the ELRS return link.
- Before the same commit, `lastSentTelemetry` was not advanced. That made the
  firmware transmit continuously after the first delay and could saturate the
  receiver-facing UART. It is fixed in the current firmware.
- The custom frame is 26 bytes and ELRS fragments it across rate-limited
  telemetry slots. Sending a fresh frame at 10 Hz can outpace conservative
  telemetry-ratio settings. The proof of concept uses 2 Hz.
- RPM values were passed through a helper intended for normalized `0..1`
  values, making almost every nonzero RPM saturate to 2047.

## Proof-of-concept payload

The frame remains CRSF type `0x16`, with 16 packed 11-bit fields:

| Channel | Meaning |
| --- | --- |
| 0 | status bit flags (`getArm`, `getEStop`) |
| 1-4 | North/East/South/West RPM, clamped to 0-2047 |
| 5-8 | North/East/South/West throttle, 0-2047 |
| 9 | E-stop lockout remaining, in 100 ms units |
| 10 | 11-bit telemetry sequence number |
| 11 | magic value `0x53A` |
| 12 | magic value `0x2C5` |
| 13 | reserved |
| 14-15 | low 22 bits of Pico uptime in milliseconds |

The magic values prevent `main4.py` from confusing control frames or other
CRSF traffic with motherboard telemetry.

This PoC is merged with the newer firmware channel layout: motor commands use
CRSF channels 1-4 and the Clara command flag uses channel 6. The host regression
test verifies that `main4.py` produces exactly that wire order.

`ENABLE_HALL_SENSORS` is currently `false`, so the proof-of-concept RPM fields
will remain zero until hall sensing is re-enabled; throttle, status, sequence,
lockout, and uptime still provide changing end-to-end validation data.

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

## End-to-end validation

1. Build and flash `src/MotherboardFirmware`.
2. Set `PORT` and `BAUD` in `src/Clara_Stuff/main4.py` for the ELRS transmitter
   module and run it normally.
3. Watch the once-per-second console diagnostics:
   - Pico USB `[DEBUG] ... seq=N`: the motherboard scheduler generated frames.
   - `CRSF RX rate` or `ELRS link`: the PC is receiving valid binary CRSF from
     the transmitter module.
   - `Motherboard RX: MB seq=N ...`: the complete return path is working.
4. If the Pico sequence changes but no motherboard frame reaches the PC, probe
   GPIO16/header pin 1. A valid transmission begins `C8 18 16` and is 26 bytes
   long. No activity means a motherboard TX/pin issue; activity there with no
   returned frame points to receiver wiring/configuration or the RF telemetry
   ratio.

Run the host-side framing regression test with:

```powershell
cd src\Clara_Stuff
.\.venv\Scripts\python.exe -m unittest -v test_telemetry.py
```

For a production protocol, replace the overloaded `0x16` frame with standard
CRSF sensor frame types where possible and use a documented extended frame for
project-specific state. This packed frame is intentionally a minimal proof of
concept compatible with the current MVP controller.
