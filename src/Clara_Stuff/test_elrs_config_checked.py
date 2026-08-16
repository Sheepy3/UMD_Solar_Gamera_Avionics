import unittest

from elrs_config import TYPE_PARAMETER_READ, TYPE_PARAMETER_SETTINGS_ENTRY
from elrs_config_checked import CheckedElrsConfigurator


class ScriptedConfigurator(CheckedElrsConfigurator):
    def __init__(self, field_data, split_at):
        self.device_id = 0xEE
        self.handset_id = 0xEA
        self.field_data = field_data
        self.split_at = split_at
        self.requested_chunk = 0

    def send(self, frame_type, payload=b""):
        self.assert_frame_type = frame_type
        self.requested_chunk = payload[3]

    def collect_frames(self, seconds):
        first = self.field_data[: self.split_at]
        second = self.field_data[self.split_at :]

        def response(remaining, data):
            payload = bytes(
                [self.handset_id, self.device_id, 2, remaining]
            ) + data
            return (0xEA, TYPE_PARAMETER_SETTINGS_ENTRY, payload)

        if self.requested_chunk == 0:
            return [response(1, first)]

        # Simulate a delayed duplicate of chunk zero arriving ahead of the
        # requested final chunk. The checked reader must reject it.
        return [response(1, first), response(0, second)]


class CheckedElrsConfiguratorTest(unittest.TestCase):
    def test_rejects_duplicate_chunk_and_decodes_current_selection(self):
        field_data = (
            bytes([0, 9])
            + b"Telem Ratio\0"
            + b"Std;Off;1:128;1:64;1:32;1:16;1:8;1:4;1:2;R\0"
            + bytes([7, 0, 9, 9])
            + b"\0"
        )
        configurator = ScriptedConfigurator(field_data, split_at=25)

        param = configurator.read_parameter(2, timeout=0.1)

        self.assertEqual(configurator.assert_frame_type, TYPE_PARAMETER_READ)
        self.assertIsNotNone(param)
        self.assertEqual(param.name, "Telem Ratio")
        self.assertEqual(param.value, 7)
        self.assertEqual(param.values[param.value], "1:4")


if __name__ == "__main__":
    unittest.main()
