import unittest

from crsf_parser import CRSFParser, PacketValidationStatus
from crsf_parser.handling import crsf_crc

from main4 import (
    TELEMETRY_MAGIC_A,
    TELEMETRY_MAGIC_B,
    decode_motherboard_telemetry,
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


class MotherboardTelemetryTest(unittest.TestCase):
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
