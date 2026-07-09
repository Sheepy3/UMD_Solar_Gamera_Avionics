import argparse
import time
from typing import Sequence

from crsf_parser.handling import crsf_build_frame
from crsf_parser.payloads import PacketsTypes
from serial import Serial


NUM_CHANNELS = 16
CRSF_MIN = 0
CRSF_MAX = 2047

DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 921600
DEFAULT_HZ = 50.0

FLAGS = {
    "A": 172,   # arm
    "B": 992,   # e-stop
    "C": 1811,  # reset e-stop, disarmed
}


def clamp(value: int, low: int = CRSF_MIN, high: int = CRSF_MAX) -> int:
    return max(low, min(high, int(value)))


def build_channels(flag: int, throttles: Sequence[int]) -> list[int]:
    channels = [0] * NUM_CHANNELS
    channels[0] = clamp(flag)

    for index, throttle in enumerate(throttles[:4], start=1):
        channels[index] = clamp(throttle)

    return channels


def build_frame(channels: Sequence[int]) -> bytes:
    if len(channels) != NUM_CHANNELS:
        raise ValueError("Must be exactly 16 channels")

    safe_channels = [clamp(channel) for channel in channels]
    return crsf_build_frame(
        PacketsTypes.RC_CHANNELS_PACKED,
        {"channels": list(reversed(safe_channels))},
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a steady CRSF RC channel stream without the GUI."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Serial baud, default {DEFAULT_BAUD}")
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ, help=f"Transmit rate, default {DEFAULT_HZ}")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means run until Ctrl+C.")
    parser.add_argument(
        "--flag",
        choices=sorted(FLAGS),
        default="C",
        help="A=arm, B=e-stop, C=reset/disarmed. Default C.",
    )
    parser.add_argument("--north", type=int, default=0, help="North throttle channel, 0-2047")
    parser.add_argument("--east", type=int, default=0, help="East throttle channel, 0-2047")
    parser.add_argument("--south", type=int, default=0, help="South throttle channel, 0-2047")
    parser.add_argument("--west", type=int, default=0, help="West throttle channel, 0-2047")
    parser.add_argument("--status-every", type=float, default=1.0, help="Seconds between status prints. 0 disables.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.hz <= 0:
        raise ValueError("--hz must be greater than 0")

    flag_value = FLAGS[args.flag]
    channels = build_channels(flag_value, [args.north, args.east, args.south, args.west])
    frame = build_frame(channels)

    period = 1.0 / args.hz
    start = time.perf_counter()
    next_tx = start
    next_status = start + args.status_every if args.status_every > 0 else None
    sent = 0
    late_count = 0
    max_late_ms = 0.0

    print(
        f"Sending flag {args.flag}={flag_value} on {args.port} at {args.baud} baud, "
        f"{args.hz:g} Hz. Channels: {channels}"
    )

    try:
        with Serial(args.port, args.baud, timeout=0, write_timeout=0) as ser:
            while True:
                now = time.perf_counter()

                if args.duration > 0 and now - start >= args.duration:
                    break

                if now >= next_tx:
                    late_ms = (now - next_tx) * 1000.0
                    if late_ms > 1.0:
                        late_count += 1
                        max_late_ms = max(max_late_ms, late_ms)

                    ser.write(frame)
                    sent += 1
                    next_tx += period

                    if now - next_tx > period:
                        next_tx = now + period

                if next_status is not None and now >= next_status:
                    elapsed = now - start
                    actual_hz = sent / elapsed if elapsed > 0 else 0.0
                    print(
                        f"sent={sent} elapsed={elapsed:.1f}s actual_hz={actual_hz:.1f} "
                        f"late_writes={late_count} max_late_ms={max_late_ms:.1f}"
                    )
                    next_status += args.status_every

                sleep_time = next_tx - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(min(sleep_time, 0.001))

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        elapsed = time.perf_counter() - start
        actual_hz = sent / elapsed if elapsed > 0 else 0.0
        print(
            f"Final: sent={sent} elapsed={elapsed:.1f}s actual_hz={actual_hz:.1f} "
            f"late_writes={late_count} max_late_ms={max_late_ms:.1f}"
        )


if __name__ == "__main__":
    main()
