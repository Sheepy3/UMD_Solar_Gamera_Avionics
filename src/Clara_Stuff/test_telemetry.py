import unittest

from crsf_parser.handling import crsf_crc

from main4 import (
    FLAG_A,
    FLIGHT_MODE_FRAME_TYPE,
    PRIMARY_TELEMETRY_PACKET_TYPE,
    build_channels,
    build_crsf_frame,
    decode_primary_telemetry,
    extract_crsf_frames,
    format_primary_telemetry,
)


def build_firmware_frame(values):
    """Build a frame using the same little-endian 11-bit packing as util.h."""
    payload = bytearray()
    bits = 0
    bit_count = 0

    for value in values:
        bits |= (value & 0x7FF) << bit_count
        bit_count += 11

        while bit_count >= 8:
            payload.append(bits & 0xFF)
            bits >>= 8
            bit_count -= 8

    frame = bytearray([0xC8, 24, 0x16]) + payload + bytearray([0])
    frame[-1] = crsf_crc(frame[2:-1])
    return frame


def build_primary_telemetry_frame(status, rpm, timestamp_ms, address=0xC8):
    payload = bytearray([PRIMARY_TELEMETRY_PACKET_TYPE, status])
    for value in rpm:
        encoded = round(value * 1000)
        payload.extend(encoded.to_bytes(2, "big"))
    payload.extend(timestamp_ms.to_bytes(4, "big"))
    frame = bytearray(
        [address, len(payload) + 2, FLIGHT_MODE_FRAME_TYPE]
    )
    frame.extend(payload)
    frame.append(crsf_crc(frame[2:]))
    return frame


def unpack_firmware_payload(payload):
    """Decode payload bytes exactly like MotherboardFirmware/util.h."""
    channels = []
    bits = 0
    bit_count = 0

    for value in payload:
        bits |= value << bit_count
        bit_count += 8

        while bit_count >= 11 and len(channels) < 16:
            channels.append(bits & 0x7FF)
            bits >>= 11
            bit_count -= 11

    return channels


class MotherboardTelemetryTest(unittest.TestCase):
    def test_primary_telemetry_decodes_millirpm_and_zero_bytes(self):
        frame = build_primary_telemetry_frame(
            status=0x06,
            rpm=[0.0, 12.0, 12.345, 20.0],
            timestamp_ms=15000,
            address=0xEA,
        )

        self.assertEqual(len(frame), 18)
        self.assertEqual(frame[3], 0x01)
        self.assertIn(0, frame[3:-1])
        telemetry = decode_primary_telemetry(frame)
        self.assertEqual(telemetry["packet_type"], 0x01)
        self.assertFalse(telemetry["armed"])
        self.assertTrue(telemetry["estop_lockout"])
        self.assertTrue(telemetry["estop"])
        self.assertEqual(telemetry["rpm_millirpm"], [0, 12000, 12345, 20000])
        self.assertEqual(telemetry["rpm"], [0.0, 12.0, 12.345, 20.0])
        self.assertEqual(telemetry["timestamp_ms"], 15000)
        formatted = format_primary_telemetry(telemetry)
        self.assertIn("MB primary type=0x01", formatted)
        self.assertIn("rpm=[0.000,12.000,12.345,20.000]", formatted)

    def test_unrelated_flight_mode_is_not_primary_telemetry(self):
        payload = b"ACRO\0"
        frame = bytearray([0xEA, len(payload) + 2, FLIGHT_MODE_FRAME_TYPE])
        frame.extend(payload)
        frame.append(crsf_crc(frame[2:]))

        self.assertIsNone(decode_primary_telemetry(frame))

    def test_stream_framing_accepts_non_c8_crsf_address(self):
        values = [172] * 16
        frame = build_firmware_frame(values)
        frame[0] = 0xEA
        stream = bytearray(b"noise") + frame

        frames, discarded, crc_errors = extract_crsf_frames(stream)

        self.assertEqual(frames, [bytes(frame)])
        self.assertEqual(discarded, len(b"noise"))
        self.assertEqual(crc_errors, 0)
        self.assertEqual(stream, bytearray())

    def test_stream_framing_recovers_after_bad_crc(self):
        values = [172] * 16
        bad_frame = build_firmware_frame(values)
        bad_frame[-1] ^= 0xFF
        good_frame = build_firmware_frame(values)
        stream = bad_frame + good_frame

        frames, _, crc_errors = extract_crsf_frames(stream)

        self.assertEqual(frames, [bytes(good_frame)])
        self.assertGreaterEqual(crc_errors, 1)
        self.assertEqual(stream, bytearray())

    def test_controller_frame_matches_new_firmware_channel_mapping(self):
        requested = build_channels(FLAG_A, n=200, e=300, s=400, w=500)
        frame = build_crsf_frame(requested)
        firmware_channels = unpack_firmware_payload(frame[3:-1])

        self.assertEqual(firmware_channels[0:4], [200, 300, 400, 500])
        self.assertEqual(firmware_channels[5], FLAG_A)

if __name__ == "__main__":
    unittest.main()
