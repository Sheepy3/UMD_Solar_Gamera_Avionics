import unittest

from crsf_parser import CRSFParser, PacketValidationStatus
from crsf_parser.handling import crsf_crc

from main4 import (
    FLAG_A,
    FLIGHT_MODE_FRAME_TYPE,
    TELEMETRY_MAGIC_A,
    TELEMETRY_MAGIC_B,
    build_channels,
    build_crsf_frame,
    decode_flight_mode_poc,
    decode_motherboard_telemetry,
    extract_crsf_frames,
    format_flight_mode_poc,
    format_motherboard_telemetry,
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


def build_flight_mode_poc_frame(status, rpm, timestamp_ms, address=0xC8):
    data = bytearray([status])
    for value in rpm:
        encoded = round(value * 256)
        data.extend(encoded.to_bytes(2, "big"))
    data.extend(timestamp_ms.to_bytes(4, "big"))
    payload = b"SG2" + data.hex().upper().encode("ascii") + b"\0"
    frame = bytearray(
        [address, len(payload) + 2, FLIGHT_MODE_FRAME_TYPE]
    )
    frame.extend(payload)
    frame.append(crsf_crc(frame[2:]))
    return frame


def build_compact_flight_mode_poc_frame(
    status, timestamp_ms, address=0xC8
):
    payload = f"SG3{status:02X}{timestamp_ms:08X}".encode("ascii") + b"\0"
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
    def test_compact_poc_matches_working_frame_size_and_decodes(self):
        frame = build_compact_flight_mode_poc_frame(
            status=0x06,
            timestamp_ms=0x1234ABCD,
            address=0xEA,
        )

        self.assertEqual(len(frame), 18)
        self.assertEqual(frame[1], 0x10)
        telemetry = decode_flight_mode_poc(frame)
        self.assertFalse(telemetry["armed"])
        self.assertTrue(telemetry["estop_lockout"])
        self.assertTrue(telemetry["estop"])
        self.assertIsNone(telemetry["rpm"])
        self.assertEqual(telemetry["timestamp_ms"], 0x1234ABCD)
        formatted = format_flight_mode_poc(telemetry)
        self.assertIn("rpm=[omitted-size-test]", formatted)
        self.assertIn("18-byte size test", formatted)

    def test_standard_flight_mode_poc_decodes_after_elrs_address_change(self):
        frame = build_flight_mode_poc_frame(
            status=0x07,
            rpm=[0.0, 12.5, 37.25, 50.0],
            timestamp_ms=0x89ABCDEF,
            address=0xEA,
        )

        telemetry = decode_flight_mode_poc(frame)

        self.assertTrue(telemetry["armed"])
        self.assertTrue(telemetry["estop_lockout"])
        self.assertTrue(telemetry["estop"])
        self.assertEqual(telemetry["rpm"], [0.0, 12.5, 37.25, 50.0])
        self.assertEqual(telemetry["timestamp_ms"], 0x89ABCDEF)
        formatted = format_flight_mode_poc(telemetry)
        self.assertIn("rpm=[0.000,12.500,37.250,50.000]", formatted)
        self.assertIn("carrier=CRSF flight mode 0x21", formatted)

    def test_unrelated_flight_mode_is_not_motherboard_telemetry(self):
        payload = b"ACRO\0"
        frame = bytearray([0xEA, len(payload) + 2, FLIGHT_MODE_FRAME_TYPE])
        frame.extend(payload)
        frame.append(crsf_crc(frame[2:]))

        self.assertIsNone(decode_flight_mode_poc(frame))

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

    def test_firmware_frame_decodes_after_stream_noise(self):
        values = [
            0b11000100,
            100,
            200,
            300,
            400,
            0,
            512,
            1024,
            2047,
            9,
            42,
            TELEMETRY_MAGIC_A,
            TELEMETRY_MAGIC_B,
            0,
            1234,
            2,
        ]
        decoded = []

        def consume(frame, status):
            decoded.append((decode_motherboard_telemetry(frame), status))

        parser = CRSFParser(consume)
        stream = bytearray(b"serial noise") + build_firmware_frame(values)
        parser.parse_stream(stream)

        self.assertEqual(stream, bytearray())
        telemetry, status = decoded[0]
        self.assertEqual(status, PacketValidationStatus.VALID)
        self.assertEqual(telemetry["sequence"], 42)
        self.assertTrue(telemetry["armed"])
        self.assertTrue(telemetry["estop"])
        self.assertEqual(telemetry["rpm"], [100, 200, 300, 400])
        self.assertEqual(telemetry["uptime_ms"], (2 << 11) | 1234)
        self.assertIn("MB seq=42", format_motherboard_telemetry(telemetry))

    def test_regular_rc_frame_is_not_motherboard_telemetry(self):
        values = [172] * 16
        decoded = []
        parser = CRSFParser(
            lambda frame, status: decoded.append(
                decode_motherboard_telemetry(frame)
            )
        )

        stream = build_firmware_frame(values)
        parser.parse_stream(stream)

        self.assertEqual(decoded, [None])


if __name__ == "__main__":
    unittest.main()
