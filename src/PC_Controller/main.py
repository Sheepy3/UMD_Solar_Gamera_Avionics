import time
from serial import Serial

from crsf_parser import CRSFParser, PacketValidationStatus
from crsf_parser.payloads import PacketsTypes
from crsf_parser.handling import crsf_build_frame

PORT = "COM3"       # <-- change
BAUD = 921600       # <-- your working baud

TX_HZ = 50          # 50 Hz is enough to keep link alive
TX_PERIOD = 1.0 / TX_HZ


def on_frame(frame, status: PacketValidationStatus) -> None:
    if status != PacketValidationStatus.VALID:
        return
    print(frame)


def main():
    parser = CRSFParser(on_frame)

    mid = 992
    channels = [mid] * 16

    with Serial(PORT, BAUD, timeout=0) as ser:
        buf = bytearray()
        next_tx = time.perf_counter()

        while True:
            # ---- RX: read and parse whatever came in ----
            data = ser.read(4096)
            if data:
                buf.extend(data)
                parser.parse_stream(buf)

            # ---- TX: send RC frames on schedule ----
            now = time.perf_counter()
            if now >= next_tx:
                frame = crsf_build_frame(
                    PacketsTypes.RC_CHANNELS_PACKED,
                    {"channels": channels},
                )
                ser.write(frame)
                next_tx += TX_PERIOD

            # avoid busy-spinning at 100% CPU
            time.sleep(0.001)


if __name__ == "__main__":
    main()
