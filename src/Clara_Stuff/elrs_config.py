import argparse
import time
from dataclasses import dataclass, field


ADDR_BROADCAST = 0x00
ADDR_HANDSET = 0xEA
ADDR_TX_MODULE = 0xEE

TYPE_DEVICE_PING = 0x28
TYPE_DEVICE_INFO = 0x29
TYPE_PARAMETER_SETTINGS_ENTRY = 0x2B
TYPE_PARAMETER_READ = 0x2C
TYPE_PARAMETER_WRITE = 0x2D

FIELD_TEXT_SELECTION = 9


@dataclass
class ParameterField:
    id: int
    parent: int | None = None
    type: int | None = None
    name: str = ""
    value: int | None = None
    values: list[str] = field(default_factory=list)
    unit: str = ""


@dataclass
class DeviceInfo:
    id: int
    name: str
    field_count: int
    is_elrs: bool


def crc8_d5(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(address: int, frame_type: int, payload: bytes = b"") -> bytes:
    body = bytes([frame_type]) + payload
    return bytes([address, len(body) + 1]) + body + bytes([crc8_d5(body)])


def parse_frames(buffer: bytearray) -> list[tuple[int, int, bytes]]:
    frames: list[tuple[int, int, bytes]] = []

    while len(buffer) >= 4:
        length = buffer[1]
        if length < 2 or length > 62:
            del buffer[0]
            continue

        frame_len = length + 2
        if len(buffer) < frame_len:
            break

        raw = bytes(buffer[:frame_len])
        del buffer[:frame_len]

        frame_type = raw[2]
        payload = raw[3:-1]
        expected_crc = raw[-1]
        if crc8_d5(raw[2:-1]) != expected_crc:
            continue

        frames.append((raw[0], frame_type, payload))

    return frames


def read_c_string(data: bytes, offset: int) -> tuple[str, int]:
    end = data.find(b"\x00", offset)
    if end < 0:
        return data[offset:].decode("latin1", errors="replace"), len(data)
    return data[offset:end].decode("latin1", errors="replace"), end + 1


def read_options(data: bytes, offset: int) -> tuple[list[str], int]:
    raw, next_offset = read_c_string(data, offset)
    return raw.split(";"), next_offset


def u32_be(data: bytes, offset: int) -> int:
    if offset + 4 > len(data):
        return 0
    return int.from_bytes(data[offset : offset + 4], "big")


class ElrsConfigurator:
    def __init__(self, serial_port, frame_address: int, device_id: int, handset_id: int):
        self.serial = serial_port
        self.frame_address = frame_address
        self.device_id = device_id
        self.handset_id = handset_id
        self.buffer = bytearray()

    def send(self, frame_type: int, payload: bytes = b"") -> None:
        self.serial.write(build_frame(self.frame_address, frame_type, payload))

    def collect_frames(self, seconds: float) -> list[tuple[int, int, bytes]]:
        deadline = time.monotonic() + seconds
        frames: list[tuple[int, int, bytes]] = []

        while time.monotonic() < deadline:
            waiting = self.serial.in_waiting
            if waiting:
                self.buffer.extend(self.serial.read(waiting))
                frames.extend(parse_frames(self.buffer))
            else:
                time.sleep(0.01)

        return frames

    def discover_tx(self, seconds: float = 3.0) -> DeviceInfo | None:
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            self.send(TYPE_DEVICE_PING, bytes([ADDR_BROADCAST, self.handset_id]))
            for _, frame_type, payload in self.collect_frames(0.25):
                if frame_type != TYPE_DEVICE_INFO or len(payload) < 3:
                    continue

                device_id = payload[1]
                name, offset = read_c_string(payload, 2)
                if offset + 12 >= len(payload):
                    continue

                field_count = payload[offset + 12]
                serial_number = u32_be(payload, offset)
                is_elrs = serial_number == 0x454C5253
                info = DeviceInfo(device_id, name, field_count, is_elrs)

                if device_id == self.device_id or is_elrs:
                    self.device_id = device_id
                    return info

        return None

    def read_parameter(self, field_id: int, timeout: float = 2.5) -> ParameterField | None:
        chunks = bytearray()
        chunk_index = 0
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            self.send(
                TYPE_PARAMETER_READ,
                bytes([self.device_id, self.handset_id, field_id, chunk_index]),
            )

            for _, frame_type, payload in self.collect_frames(0.12):
                if frame_type != TYPE_PARAMETER_SETTINGS_ENTRY or len(payload) < 5:
                    continue
                if payload[1] != self.device_id or payload[2] != field_id:
                    continue

                chunks_remaining = payload[3]
                chunks.extend(payload[4:])

                if chunks_remaining > 0:
                    chunk_index += 1
                    break

                return decode_parameter(field_id, bytes(chunks))

        return None

    def read_all_parameters(self, field_count: int) -> list[ParameterField]:
        fields: list[ParameterField] = []
        for field_id in range(1, field_count + 1):
            field = self.read_parameter(field_id)
            if field is not None and field.name:
                fields.append(field)
        return fields

    def write_parameter(self, param: ParameterField, value: int) -> None:
        payload = bytes([self.device_id, self.handset_id, param.id, value & 0xFF])
        self.send(TYPE_PARAMETER_WRITE, payload)


def decode_parameter(field_id: int, data: bytes) -> ParameterField | None:
    if len(data) < 3:
        return None

    parent = data[0] or None
    field_type = data[1] & 0x7F
    name, offset = read_c_string(data, 2)

    param = ParameterField(id=field_id, parent=parent, type=field_type, name=name)

    if field_type == FIELD_TEXT_SELECTION:
        param.values, offset = read_options(data, offset)
        if offset < len(data):
            param.value = data[offset]
        if offset + 4 < len(data):
            param.unit, _ = read_c_string(data, offset + 4)
    elif offset < len(data):
        param.value = data[offset]

    return param


def find_field(fields: list[ParameterField], wanted_name: str) -> ParameterField | None:
    wanted = wanted_name.casefold()
    for field in fields:
        if field.name.casefold() == wanted:
            return field
    for field in fields:
        if wanted in field.name.casefold():
            return field
    return None


def find_option(field: ParameterField, wanted_option: str) -> int | None:
    wanted = wanted_option.casefold()
    for index, option in enumerate(field.values):
        if option.casefold() == wanted:
            return index
    for index, option in enumerate(field.values):
        if wanted in option.casefold():
            return index
    return None


def print_fields(fields: list[ParameterField]) -> None:
    for param in fields:
        if param.type == FIELD_TEXT_SELECTION:
            current = ""
            if param.value is not None and param.value < len(param.values):
                current = f" current={param.values[param.value]!r}"
            options = ", ".join(repr(value) for value in param.values if value)
            print(f"{param.id:02d}: {param.name} [select]{current} options=[{options}]")
        else:
            print(f"{param.id:02d}: {param.name} type={param.type} value={param.value}")


def parse_int(text: str) -> int:
    return int(text, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Small PC-side version of the ELRS Lua parameter reader/writer."
    )
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--frame-address", type=parse_int, default=ADDR_TX_MODULE)
    parser.add_argument("--device-id", type=parse_int, default=ADDR_TX_MODULE)
    parser.add_argument("--handset-id", type=parse_int, default=ADDR_HANDSET)
    parser.add_argument("--list", action="store_true", help="List all readable TX parameters.")
    parser.add_argument("--set", nargs=2, metavar=("FIELD", "OPTION"), help="Set a text-select field by name.")
    parser.add_argument("--packet-rate", help="Shortcut for --set 'Packet Rate' OPTION.")
    parser.add_argument("--switch-mode", help="Shortcut for --set 'Switch Mode' OPTION.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not (args.list or args.set or args.packet_rate or args.switch_mode):
        args.list = True

    import serial

    with serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.5) as ser:
        configurator = ElrsConfigurator(
            ser,
            frame_address=args.frame_address,
            device_id=args.device_id,
            handset_id=args.handset_id,
        )

        ser.reset_input_buffer()
        info = configurator.discover_tx()
        if info is None:
            raise RuntimeError(
                "No ELRS TX device info response. Stop Clara/headless_tx, make sure the TX is on "
                "this port, and try --frame-address 0xC8 if 0xEE does not respond."
            )

        print(
            f"Found device 0x{info.id:02X}: {info.name} "
            f"fields={info.field_count} elrs={info.is_elrs}"
        )

        fields = configurator.read_all_parameters(info.field_count)

        if args.list:
            print_fields(fields)

        requested_sets: list[tuple[str, str]] = []
        if args.set:
            requested_sets.append((args.set[0], args.set[1]))
        if args.packet_rate:
            requested_sets.append(("Packet Rate", args.packet_rate))
        if args.switch_mode:
            requested_sets.append(("Switch Mode", args.switch_mode))

        for field_name, option_name in requested_sets:
            param = find_field(fields, field_name)
            if param is None:
                raise RuntimeError(f"Could not find field named {field_name!r}. Run --list to inspect names.")
            if param.type != FIELD_TEXT_SELECTION:
                raise RuntimeError(f"Field {param.name!r} is not a text-selection field.")

            option_index = find_option(param, option_name)
            if option_index is None:
                raise RuntimeError(
                    f"Could not find option {option_name!r} for {param.name!r}. "
                    f"Options: {param.values}"
                )

            configurator.write_parameter(param, option_index)
            print(f"Set {param.name!r} to {param.values[option_index]!r} ({option_index}).")
            time.sleep(0.25)


if __name__ == "__main__":
    main()
