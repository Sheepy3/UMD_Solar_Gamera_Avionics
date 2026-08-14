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
- The old packed-channel frame was 26 bytes and required more fragmentation
  across rate-limited telemetry slots. The replacement frame is 18 bytes and
  the proof of concept remains at 2 Hz.

## Proof-of-concept payload

The replacement uses standard CRSF `FLIGHT_MODE` telemetry (`0x21`) as a
conservative ASCII carrier. Its null-terminated payload is:

```text
SG1ssssffuuuu
```

- `SG1`: Solar Gamera PoC signature and format version.
- `ssss`: 16-bit sequence number in hexadecimal.
- `ff`: status flags in hexadecimal; bit 0 is armed and bit 1 is E-stop.
- `uuuu`: low 16 bits of Pico uptime in milliseconds, in hexadecimal.

For example, `SG1123401ABCD` means sequence `0x1234`, armed, not E-stopped,
and uptime-low `0xABCD`. This intentionally demonstrates transport of changing
application data while remaining a valid, ordinary receiver-to-transmitter
telemetry frame. It is not the proposed final production schema.

This PoC is merged with the newer firmware channel layout: motor commands use
CRSF channels 1-4 and the Clara command flag uses channel 6. The host regression
test verifies that `main4.py` produces exactly that wire order.

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

## End-to-end validation

1. Build and flash `src/MotherboardFirmware`.
2. Set `PORT` and `BAUD` in `src/Clara_Stuff/main4.py` for the ELRS transmitter
   module and run it normally.
3. Watch the once-per-second console diagnostics:
   - Pico USB `[DEBUG] CRSF_FLIGHT_MODE payload=SG1...`: the motherboard
     scheduler generated the new frame.
   - `Serial RX raw`: bytes physically returned to the PC serial port.
   - `CRSF RX rate` or `ELRS link`: the PC is receiving valid binary CRSF from
     the transmitter module.
   - `Motherboard RX: MB PoC seq=N ... carrier=CRSF flight mode 0x21`: the
     complete return path is working.
4. If the Pico sequence changes but no motherboard frame reaches the PC, probe
   GPIO16/header pin 1. The new transmission begins `C8 10 21 53 47 31` and is
   18 bytes long. No activity means a motherboard TX/pin issue; activity with no
   returned frame points to receiver wiring/configuration or the RF telemetry
   ratio.

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
