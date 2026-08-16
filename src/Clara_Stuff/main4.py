import tkinter as tk
from tkinter import ttk
import os
import sys

from crsf_parser import CRSFParser, PacketValidationStatus
from crsf_parser.payloads import PacketsTypes
from crsf_parser.handling import crsf_build_frame, crsf_crc

import serial
import time
import math


# -------- Packet Constants -------------
FLAG_A = 172    # arm
FLAG_B = 992   # e-stop
FLAG_C = 1811   # reset e-stop, disarmed
FLAG_D = 410   # reset e-stop and arm this not used

NUM_CHANNELS = 16

CRSF_MIN = 172
CRSF_MAX = 1811
CRSF_UNUSED_CH5 = CRSF_MIN

THROTTLE_N_CHANNEL = 0
THROTTLE_E_CHANNEL = 1
THROTTLE_S_CHANNEL = 2
THROTTLE_W_CHANNEL = 3
COMMAND_FLAG_CHANNEL = 5

PORT = "COM6"
BAUD = 921600

SEND_RATE_MS = 20  # 20 ms = 50 Hz

FLIGHT_MODE_FRAME_TYPE = 0x21
PRIMARY_TELEMETRY_PACKET_TYPE = 0x01
PRIMARY_TELEMETRY_FRAME_BYTES = 18

KNOWN_RETURN_TYPES = {
    PacketsTypes.LINK_STATISTICS,
}


# -------- Packet Functions -------------

def clamp(value, low=CRSF_MIN, high=CRSF_MAX):
    return max(low, min(high, int(value)))


def ramp_throttle(current_percent, target_percent, velocity_percent_per_second,
                  elapsed_seconds):
    """Move a throttle toward its target by one transmit tick.

    A zero velocity retains the controller's original immediate behavior.
    """
    if velocity_percent_per_second <= 0:
        return target_percent

    difference = target_percent - current_percent
    maximum_change = velocity_percent_per_second * max(0.0, elapsed_seconds)
    if abs(difference) <= maximum_change:
        return target_percent

    return current_percent + math.copysign(maximum_change, difference)


def build_channels(command_flag, n=0, e=0, s=0, w=0):
    channels = [0] * NUM_CHANNELS

    channels[THROTTLE_N_CHANNEL] = clamp(n)
    channels[THROTTLE_E_CHANNEL] = clamp(e)
    channels[THROTTLE_S_CHANNEL] = clamp(s)
    channels[THROTTLE_W_CHANNEL] = clamp(w)
    channels[4] = CRSF_UNUSED_CH5
    channels[COMMAND_FLAG_CHANNEL] = clamp(command_flag)

    return channels


def build_crsf_frame(channels):
    if len(channels) != NUM_CHANNELS:
        raise ValueError("Must be exactly 16 channels")

    channels = [clamp(ch) for ch in channels]

    return crsf_build_frame(
        PacketsTypes.RC_CHANNELS_PACKED,
        {"channels": list(reversed(channels))}
    )


def build_frame_A(n, e, s, w):
    channels = build_channels(FLAG_A, n, e, s, w)
    return build_crsf_frame(channels)


def build_frame_B():
    channels = build_channels(FLAG_B)
    return build_crsf_frame(channels)


def build_frame_C():
    channels = build_channels(FLAG_C)
    return build_crsf_frame(channels)


def build_frame_D():
    channels = build_channels(FLAG_D)
    return build_crsf_frame(channels)


def extract_crsf_frames(buffer):
    """Extract address-agnostic, CRC-valid CRSF frames from a byte stream.

    The old ``crsf-parser`` package hard-codes 0xC8 as the first byte. ELRS
    handset-side traffic can use a different device address, so framing must
    be validated by length and CRC before known payloads are handed to that
    package for decoding.
    """
    frames = []
    discarded_bytes = 0
    crc_errors = 0

    while len(buffer) >= 4:
        complete_frame = None
        earliest_partial = None

        # Search beyond a corrupt/noisy prefix instead of trusting the first
        # plausible length byte. A payload byte can otherwise masquerade as a
        # long frame and block parsing while a valid frame is already buffered.
        for start in range(len(buffer) - 3):
            frame_length = buffer[start + 1]
            if frame_length < 2 or frame_length > 62:
                continue

            total_length = frame_length + 2
            end = start + total_length
            if end > len(buffer):
                if earliest_partial is None:
                    earliest_partial = start
                continue

            candidate = bytes(buffer[start:end])
            if crsf_crc(candidate[2:-1]) == candidate[-1]:
                complete_frame = (start, end, candidate)
                break

            crc_errors += 1

        if complete_frame is not None:
            start, end, candidate = complete_frame
            discarded_bytes += start
            del buffer[:end]
            frames.append(candidate)
            continue

        if earliest_partial is not None:
            # Keep the earliest possible partial frame for the next serial
            # read, but discard any definite noise that precedes it.
            discarded_bytes += earliest_partial
            del buffer[:earliest_partial]
            break

        # No viable complete or partial header begins at the current byte.
        del buffer[0]
        discarded_bytes += 1

    return frames, discarded_bytes, crc_errors


def decode_primary_telemetry(raw_frame):
    """Decode the motherboard's 18-byte primary telemetry packet."""
    if (
        len(raw_frame) != PRIMARY_TELEMETRY_FRAME_BYTES
        or raw_frame[2] != FLIGHT_MODE_FRAME_TYPE
    ):
        return None

    payload = raw_frame[3:-1]
    if len(payload) != 14 or payload[0] != PRIMARY_TELEMETRY_PACKET_TYPE:
        return None

    status = payload[1]
    rpm_millirpm = [
        int.from_bytes(payload[offset:offset + 2], "big")
        for offset in range(2, 10, 2)
    ]
    return {
        "packet_type": payload[0],
        "armed": bool(status & 0x01),
        "estop_lockout": bool(status & 0x02),
        "estop": bool(status & 0x04),
        "status": status,
        "rpm_millirpm": rpm_millirpm,
        "rpm": [value / 1000.0 for value in rpm_millirpm],
        "timestamp_ms": int.from_bytes(payload[10:14], "big"),
        "raw_hex": payload.hex(),
    }


def format_primary_telemetry(telemetry):
    rpm = ",".join(f"{value:.3f}" for value in telemetry["rpm"])
    return (
        "MB primary "
        f"type=0x{telemetry['packet_type']:02X} "
        f"armed={str(telemetry['armed']).lower()} "
        f"estop_lockout={str(telemetry['estop_lockout']).lower()} "
        f"estop={str(telemetry['estop']).lower()} "
        f"rpm=[{rpm}] timestamp_ms={telemetry['timestamp_ms']}"
    )


# -------- GUI Application -------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Solar Gamera Controller")
        self.geometry("1000x1000")

        # Packet values:
        # internal index 0 = flag, 1-4 = North/East/South/West.
        # CRSF output maps throttles to channels 1-4 and the flag to channel 6.
        self.data_packet_list = [FLAG_B, 0, 0, 0, 0]

        self.serial_connected = False
        self.ser = None

        # ELRS returns binary CRSF telemetry from the motherboard. Keep a
        # persistent buffer because a frame may span several serial reads.
        self.rx_buffer = bytearray()
        self.crsf_parser = CRSFParser(self.on_crsf_frame)
        self.latest_rx_line = "No motherboard response yet"
        self.rx_count = 0
        self.raw_rx_byte_count = 0
        self.wire_frame_count = 0
        self.tx_echo_count = 0
        self.rx_start_time = time.time()
        self.latest_rx_rate = 0.0
        self.latest_raw_rx_byte_rate = 0.0
        self.latest_crsf_rx_rate = 0.0
        self.latest_tx_echo_rate = 0.0
        self.latest_raw_rx_hex = "none"
        self.latest_tx_frame = None
        self.wire_crc_errors = 0
        self.wire_discarded_bytes = 0
        self.latest_link_line = "No ELRS link statistics yet"

        self.is_armed = False
        self.is_estopped = True

        # A value of zero preserves the original immediate-throttle behavior.
        # Positive values ramp throttle changes in percentage points per second.
        self.throttle_velocity_percent_per_second = 0.0
        self.last_throttle_ramp_time = time.monotonic()

        self.setup_layout()

        # Try opening serial when GUI starts
        self.connect_serial()

        # Start the periodic CRSF transmit loop and the non-blocking
        # motherboard receive monitor.
        self.send_loop()
        self.receive_loop()

        # Cleanly close serial when window closes
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def setup_layout(self):
        title_lbl = ttk.Label(
            self,
            text="Solar Gamera Controller",
            font=("Calibri", 22, "bold")
        )
        title_lbl.grid(row=0, column=0, pady=20)

        self.status_label = ttk.Label(
            self,
            text="Status: Starting...",
            font=("Calibri", 16)
        )
        self.status_label.grid(row=0, column=1, pady=10)

        self.North_Arm = Arm_Control_Container(self, 1, [3, 0])
        self.East_Arm = Arm_Control_Container(self, 2, [3, 1])
        self.South_Arm = Arm_Control_Container(self, 3, [3, 2])
        self.West_Arm = Arm_Control_Container(self, 4, [3, 3])

        arm_btn = tk.Button(
            self,
            text="ARM",
            bg="green",
            fg="white",
            font=("Arial", 20, "bold"),
            command=self.arm_system
        )
        arm_btn.grid(row=0, column=2, pady=20)

        reset_btn = tk.Button(
            self,
            text="RESET E-STOP",
            bg="orange",
            fg="black",
            font=("Arial", 20, "bold"),
            command=self.reset_estop
        )
        reset_btn.grid(row=0, column=3, pady=20)

        e_stop_btn = tk.Button(
            self,
            text="EMERGENCY STOP",
            bg="red",
            fg="white",
            font=("Arial", 30, "bold"),
            command=self.estop
        )
        e_stop_btn.grid(row=0, column=4, pady=20)

        max_throttle_btn = tk.Button(
            self,
            text="SET ON ARMS: MAX",
            bg="purple",
            fg="white",
            font=("Arial", 16, "bold"),
            command=lambda: self.set_enabled_arms_throttle(100.0)
        )
        max_throttle_btn.grid(row=1, column=2, pady=10)

        zero_throttle_btn = tk.Button(
            self,
            text="SET ON ARMS: 0%",
            bg="purple",
            fg="white",
            font=("Arial", 16, "bold"),
            command=lambda: self.set_enabled_arms_throttle(0.0)
        )
        zero_throttle_btn.grid(row=1, column=3, pady=10)

        restart_btn = tk.Button(
            self,
            text="RESTART CONTROLLER",
            bg="steel blue",
            fg="white",
            font=("Arial", 14, "bold"),
            command=self.restart_controller
        )
        restart_btn.grid(row=1, column=4, pady=10)

        # One shared input for commanding the same throttle on all four motors.
        all_motor_frame = ttk.LabelFrame(
            self,
            text="All Motor Control",
            padding=10
        )
        all_motor_frame.grid(
            row=2,
            column=0,
            columnspan=5,
            pady=15
        )

        all_motor_label = ttk.Label(
            all_motor_frame,
            text="Throttle percent (0-100):",
            font=("Calibri", 14, "bold")
        )
        all_motor_label.grid(row=0, column=0, padx=8, pady=5)

        self.all_motor_entry = ttk.Entry(all_motor_frame, width=12)
        self.all_motor_entry.grid(row=0, column=1, padx=8, pady=5)
        self.all_motor_entry.bind(
            "<Return>",
            lambda event: self.set_all_motors_from_entry()
        )

        all_motor_btn = tk.Button(
            all_motor_frame,
            text="SET ALL MOTORS",
            bg="purple",
            fg="white",
            font=("Arial", 14, "bold"),
            command=self.set_all_motors_from_entry
        )
        all_motor_btn.grid(row=0, column=2, padx=8, pady=5)

        velocity_label = ttk.Label(
            all_motor_frame,
            text="Throttle velocity (%/s, 0 = instant):",
            font=("Calibri", 14, "bold")
        )
        velocity_label.grid(row=1, column=0, padx=8, pady=5)

        self.throttle_velocity_entry = ttk.Entry(all_motor_frame, width=12)
        self.throttle_velocity_entry.insert(0, "0")
        self.throttle_velocity_entry.grid(row=1, column=1, padx=8, pady=5)
        self.throttle_velocity_entry.bind(
            "<Return>",
            lambda event: self.set_throttle_velocity_from_entry()
        )

        velocity_btn = tk.Button(
            all_motor_frame,
            text="SET VELOCITY",
            bg="purple",
            fg="white",
            font=("Arial", 14, "bold"),
            command=self.set_throttle_velocity_from_entry
        )
        velocity_btn.grid(row=1, column=2, padx=8, pady=5)

    def arm_controls(self):
        return (self.North_Arm, self.East_Arm, self.South_Arm, self.West_Arm)

    def set_throttle_velocity_from_entry(self):
        """Set the global upward throttle ramp speed in percent per second."""
        try:
            velocity = float(self.throttle_velocity_entry.get().strip())
        except ValueError:
            velocity = -1

        if velocity < 0 or not math.isfinite(velocity):
            self.status_label.config(
                text="Status: Throttle velocity must be a non-negative number"
            )
            print("Throttle velocity must be a non-negative number.")
            return

        self.throttle_velocity_percent_per_second = velocity
        self.last_throttle_ramp_time = time.monotonic()

        # Switching back to zero restores the original immediate behavior,
        # including any throttle targets that were in the middle of a ramp.
        if velocity == 0:
            for arm in self.arm_controls():
                arm.apply_target_throttle_immediately()

        self.status_label.config(
            text=f"Status: Throttle velocity set to {velocity:g}%/s"
        )
        print(f"Throttle velocity set to {velocity:g}%/s.")

    def update_throttle_ramps(self):
        """Advance all motor throttles once per transmit-loop tick."""
        now = time.monotonic()
        elapsed = now - self.last_throttle_ramp_time
        self.last_throttle_ramp_time = now

        if self.throttle_velocity_percent_per_second <= 0:
            return

        for arm in self.arm_controls():
            arm.advance_throttle(elapsed, self.throttle_velocity_percent_per_second)

    def set_all_motors_from_entry(self):
        """Apply one 0-100% throttle command to all four arm controls."""
        try:
            entry_text = self.all_motor_entry.get().strip()
            throttle_percent = float(entry_text)
        except ValueError:
            self.status_label.config(
                text="Status: Enter a valid all-motor throttle from 0 to 100"
            )
            print("All motor control: enter a valid number from 0 to 100.")
            return

        if not 0 <= throttle_percent <= 100:
            self.status_label.config(
                text="Status: All-motor throttle must be from 0 to 100"
            )
            print("All motor control: value must be from 0 to 100.")
            return

        arms = self.arm_controls()

        # Turn every arm control on so the shared command applies to all four.
        for arm in arms:
            arm.switch_state = True
            arm.switch_btn.config(text="ON", bg="green", fg="white")
            arm.throttle_value = throttle_percent

        # Send an immediate armed frame in addition to the normal 50 Hz loop.
        if self.is_armed and not self.is_estopped:
            n, e, s, w = self.data_packet_list[1:5]
            self.write_frame(build_frame_A(n, e, s, w))

        self.status_label.config(
            text=f"Status: Set all motors to {throttle_percent:.1f}%"
        )
        print(f"Set all four motors to {throttle_percent:.1f}%.")

    def connect_serial(self):
        try:
            # timeout=0 keeps reads non-blocking so serial monitoring cannot
            # pause the Tkinter GUI or disturb the 50 Hz transmit loop.
            self.ser = serial.Serial(PORT, BAUD, timeout=0)
            self.serial_connected = True
            self.status_label.config(text=f"Status: Connected on {PORT}")
            print(f"Connected on {PORT} at {BAUD} baud.")

        except serial.SerialException as error:
            self.serial_connected = False
            self.status_label.config(text="Status: Serial not connected")
            print(f"Serial connection failed: {error}")

    def arm_system(self):
        if self.is_estopped:
            print("Cannot arm: system is currently E-stopped. Reset E-stop first.")
            self.status_label.config(text="Status: Cannot arm. Reset E-stop first.")
            return

        self.is_armed = True
        self.data_packet_list[0] = FLAG_A
        self.status_label.config(text="Status: ARMED")
        print("System armed.")

    def set_enabled_arms_throttle(self, throttle_percent):
        arms = self.arm_controls()
        enabled_arms = [arm for arm in arms if arm.switch_state]

        if not enabled_arms:
            self.status_label.config(text="Status: No arms are ON")
            print("Throttle preset ignored: no arms are ON.")
            return

        for arm in enabled_arms:
            # The MAX and 0% presets are intentionally instant overrides.
            arm.set_throttle_immediately(throttle_percent)

        if self.is_armed and not self.is_estopped:
            n, e, s, w = self.data_packet_list[1:5]
            self.write_frame(build_frame_A(n, e, s, w))

        self.status_label.config(
            text=f"Status: Set {len(enabled_arms)} ON arm(s) to {throttle_percent:.0f}%"
        )
        print(f"Set ON arms to {throttle_percent:.0f}%.")

    def restart_controller(self):
        print("Restarting controller and re-establishing E-stop state.")

        # Put the currently powered motherboard in a safe state before the
        # process restarts, then let the new instance begin E-stopped.
        self.is_armed = False
        self.is_estopped = True
        self.send_estop_burst()

        if self.ser is not None:
            self.ser.close()

        os.execv(sys.executable, [sys.executable, *sys.argv])

    def estop(self):
        self.is_armed = False
        self.is_estopped = True

        self.data_packet_list = [FLAG_B, 0, 0, 0, 0]

        # Also force all GUI arms off
        self.North_Arm.force_off()
        self.East_Arm.force_off()
        self.South_Arm.force_off()
        self.West_Arm.force_off()

        self.status_label.config(text="Status: E-STOPPED")
        print("Emergency stop triggered.")

        # Send E-stop immediately several times
        self.send_estop_burst()

    def reset_estop(self):
        self.is_armed = False
        self.is_estopped = False

        self.data_packet_list = [FLAG_C, 0, 0, 0, 0]

        self.status_label.config(text="Status: Resetting E-stop for 10 seconds...")
        print("Resetting E-stop. Sending reset packets for 10 seconds.")

        self.reset_start_time = time.time()
        self.send_reset_loop()

    def send_reset_loop(self):
        elapsed = time.time() - self.reset_start_time

        if elapsed < 10:
            frame = build_frame_C()
            self.write_frame(frame)
            self.after(SEND_RATE_MS, self.send_reset_loop)

        else:
            self.data_packet_list[0] = FLAG_B
            self.is_estopped = False
            self.status_label.config(text="Status: E-stop reset complete. System disarmed.")
            print("E-stop reset complete. System is disarmed.")

    def send_estop_burst(self):
        for _ in range(10):
            frame = build_frame_B()
            self.write_frame(frame)
            time.sleep(0.02)

    def send_loop(self):
        """
        Main periodic sending loop.
        Runs every 20 ms without freezing the GUI.
        """

        self.update_throttle_ramps()

        if self.is_estopped:
            frame = build_frame_B()
            channels = build_channels(FLAG_B)


        elif self.is_armed:
            n = self.data_packet_list[1]
            e = self.data_packet_list[2]
            s = self.data_packet_list[3]
            w = self.data_packet_list[4]

            channels = build_channels(FLAG_A, n, e, s,w)
            frame = build_frame_A(n, e, s, w)

        else:
            # Disarmed and not E-stopped.
            # Send reset/disarmed flag or zero packet depending on your receiver logic.
            frame = build_frame_C()
            channels = build_channels(FLAG_C)


        self.write_frame(frame)
        self.debug_print_once_per_second(channels, frame)

        # Schedule next loop
        self.after(SEND_RATE_MS, self.send_loop)

    def debug_print_once_per_second(self, channels, frame):
        now = time.time()

        if not hasattr(self, "last_debug_print"):
            self.last_debug_print = 0

        if now - self.last_debug_print >= 1.0:
            print("TX channels:", channels)
            print("TX frame hex:", frame.hex())
            print(f"Motherboard RX: {self.latest_rx_line}")
            print(
                f"Motherboard RX rate: {self.latest_rx_rate:.1f} "
                "telemetry frames/s"
            )
            print(
                f"Serial RX raw: {self.latest_raw_rx_byte_rate:.1f} bytes/s "
                f"last={self.latest_raw_rx_hex}"
            )
            parser_stats = self.crsf_parser.get_stats()
            print(
                f"CRSF RX rate: {self.latest_crsf_rx_rate:.1f} frames/s "
                f"(TX echoes: {self.latest_tx_echo_rate:.1f}/s, "
                f"wire CRC errors: {self.wire_crc_errors}, "
                f"discarded bytes: {self.wire_discarded_bytes}, "
                f"payload parse CRC errors: {parser_stats.crc_errors})"
            )
            print(f"ELRS link: {self.latest_link_line}")
            self.last_debug_print = now

    def write_frame(self, frame):
        if not hasattr(self, "tx_count"):
                self.tx_count = 0
                self.tx_start_time = time.time()

        self.tx_count += 1

        elapsed = time.time() - self.tx_start_time
        if elapsed >= 1.0:
            print(f"TX rate: {self.tx_count / elapsed:.1f} Hz")
            self.tx_count = 0
            self.tx_start_time = time.time()

        if self.serial_connected and self.ser is not None:
            try:
                self.ser.write(frame)
                self.latest_tx_frame = bytes(frame)
            except serial.SerialException as error:
                self.serial_connected = False
                self.status_label.config(text="Status: Serial write failed")
                print(f"Serial write failed: {error}")
        
    def on_close(self):
        print("Closing controller.")

        # Try to send E-stop before closing
        try:
            for _ in range(10):
                frame = build_frame_B()
                self.write_frame(frame)
                time.sleep(0.02)
        except:
            pass

        if self.ser is not None:
            self.ser.close()

        self.destroy()

    def receive_loop(self):
        """Parse binary CRSF return traffic without blocking the 50 Hz uplink."""

        if self.serial_connected and self.ser is not None:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting)
                    self.raw_rx_byte_count += len(chunk)
                    self.latest_raw_rx_hex = chunk[-32:].hex()
                    self.rx_buffer.extend(chunk)

                    frames, discarded, crc_errors = extract_crsf_frames(
                        self.rx_buffer
                    )
                    self.wire_frame_count += len(frames)
                    self.wire_discarded_bytes += discarded
                    self.wire_crc_errors += crc_errors

                    for raw_frame in frames:
                        if raw_frame == self.latest_tx_frame:
                            self.tx_echo_count += 1
                            continue

                        telemetry = decode_primary_telemetry(raw_frame)
                        if telemetry is not None:
                            self.latest_rx_line = format_primary_telemetry(
                                telemetry
                            )
                            self.rx_count += 1
                            continue

                        if raw_frame[2] not in KNOWN_RETURN_TYPES:
                            continue

                        # The payload parser requires 0xC8 even though framing
                        # above has already accepted any valid CRSF address.
                        normalized = bytearray(raw_frame)
                        normalized[0] = 0xC8
                        self.crsf_parser.parse_stream(normalized)

                now = time.time()
                elapsed = now - self.rx_start_time
                if elapsed >= 1.0:
                    self.latest_rx_rate = self.rx_count / elapsed
                    self.latest_raw_rx_byte_rate = (
                        self.raw_rx_byte_count / elapsed
                    )
                    self.latest_tx_echo_rate = self.tx_echo_count / elapsed
                    self.latest_crsf_rx_rate = (
                        self.wire_frame_count - self.tx_echo_count
                    ) / elapsed
                    self.rx_count = 0
                    self.raw_rx_byte_count = 0
                    self.wire_frame_count = 0
                    self.tx_echo_count = 0
                    self.rx_start_time = now

            except (serial.SerialException, OSError) as error:
                self.serial_connected = False
                self.status_label.config(text="Status: Serial read failed")
                print(f"Serial read failed: {error}")

        # Poll at the same 20 ms cadence as the 50 Hz transmit loop.
        self.after(SEND_RATE_MS, self.receive_loop)

    def on_crsf_frame(self, frame, status):
        """Consume known CRSF return frames handled by the payload parser."""
        if status != PacketValidationStatus.VALID:
            return

        if frame.header.type == PacketsTypes.LINK_STATISTICS:
            self.latest_link_line = (
                f"uplink_lq={frame.payload.uplink_link_quality}% "
                f"downlink_lq={frame.payload.downlink_link_quality}% "
                f"uplink_rssi=-{frame.payload.uplink_rssi_ant_1}dBm"
            )

# -------- Arm Controller Widget -------------

class Arm_Control_Container(ttk.Frame):
    def __init__(self, parent, id, location, throttle_value_percent=0):
        super().__init__(parent)

        self.parent = parent
        self.id = id

        self.throttle_value_percent = float(throttle_value_percent)
        self.target_throttle_percent = float(throttle_value_percent)
        self.throttle_value_raw = self.percent_to_raw(throttle_value_percent)

        self.xloc = location[0]
        self.yloc = location[1]

        self.switch_state = False

        self.grid(
            row=self.xloc,
            column=self.yloc,
            padx=20,
            pady=5,
            sticky="nsew"
        )

        self.setup_widgets()

    def percent_to_raw(self, throttle_value_percent):
        return int(CRSF_MIN + (throttle_value_percent / 100) * (CRSF_MAX - CRSF_MIN))

    def raw_to_percent(self, throttle_value_raw):
        return ((throttle_value_raw - CRSF_MIN) / (CRSF_MAX - CRSF_MIN) * 100)

    @property
    def throttle_value(self):
        return self.throttle_value_percent

    @throttle_value.setter
    def throttle_value(self, new_value):
        if not 0 <= new_value <= 100:
            print("Incorrect throttle value entered. Use 0-100.")
            self.throttle_display_label.config(text="Invalid Range")
            return

        self.target_throttle_percent = float(new_value)

        # A zero rate deliberately keeps the legacy immediate behavior.
        if self.parent.throttle_velocity_percent_per_second <= 0:
            self.apply_target_throttle_immediately()

    def set_throttle_immediately(self, new_value):
        """Bypass the configured velocity for an explicit instant command."""
        if not 0 <= new_value <= 100:
            print("Incorrect throttle value entered. Use 0-100.")
            self.throttle_display_label.config(text="Invalid Range")
            return

        self.target_throttle_percent = float(new_value)
        self.apply_target_throttle_immediately()

    def apply_target_throttle_immediately(self):
        """Apply the pending target to the outgoing CRSF channel now."""
        self._apply_throttle(self.target_throttle_percent)

    def advance_throttle(self, elapsed_seconds, velocity_percent_per_second):
        """Apply one smooth, rate-limited change at the transmit cadence."""
        next_throttle = ramp_throttle(
            self.throttle_value_percent,
            self.target_throttle_percent,
            velocity_percent_per_second,
            elapsed_seconds,
        )
        if next_throttle != self.throttle_value_percent:
            self._apply_throttle(next_throttle)

    def _apply_throttle(self, throttle_percent):
        self.throttle_value_percent = float(throttle_percent)
        self.throttle_value_raw = self.percent_to_raw(throttle_percent)

        # This updates the parent packet list.
        # id 1 = North, 2 = East, 3 = South, 4 = West
        self.parent.data_packet_list[self.id] = self.throttle_value_raw

        self.throttle_display_text = f"{self.throttle_value_percent:.1f}%"
        self.throttle_display_label.config(text=self.throttle_display_text)

        print(f"Updated packet values: {self.parent.data_packet_list}")

    def setup_widgets(self):
        arm_names = {
            1: "North Arm",
            2: "East Arm",
            3: "South Arm",
            4: "West Arm"
        }

        self.label_name = arm_names.get(self.id, "Unknown Arm")

        arm_label = ttk.Label(
            self,
            text=self.label_name,
            font=("Calibri", 24, "bold")
        )
        arm_label.pack(pady=10)

        self.switch_btn = tk.Button(
            self,
            text="OFF",
            bg="red",
            fg="white",
            font=("Arial", 14, "bold"),
            width=10,
            command=self.toggle_switch
        )
        self.switch_btn.pack(pady=10)

        self.throttle_display_label = ttk.Label(
            self,
            text=f"{self.throttle_value_percent:.1f}%",
            font=("Calibri", 24, "bold")
        )
        self.throttle_display_label.pack(pady=10)

        self.entry_box = ttk.Entry(self, width=30)
        self.entry_box.pack(pady=10)

        self.entry_box_btn = tk.Button(
            self,
            text="Enter",
            width=5,
            font=("Arial", 10),
            command=self.set_entry_box
        )
        self.entry_box_btn.pack(pady=5)

    def toggle_switch(self):
        self.switch_state = not self.switch_state

        if self.switch_state:
            self.switch_btn.config(text="ON", bg="green", fg="white")

            if self.entry_box.get():
                self.set_entry_box()
            else:
                self.throttle_value = 0.0

            print(f"{self.label_name} switched ON. Throttle: {self.throttle_value}%")

        else:
            self.force_off()
            print(f"{self.label_name} switched OFF. Throttle forced to 0%")

    def force_off(self):
        self.switch_state = False
        self.switch_btn.config(text="OFF", bg="red", fg="white")
        self.set_throttle_immediately(0.0)

    def set_entry_box(self):
        if not self.switch_state:
            print(f"Cannot set throttle. {self.label_name} is turned off.")
            self.throttle_display_label.config(text="Turn ON First")
            return

        try:
            user_input = float(self.entry_box.get()) if self.entry_box.get() else 0.0
            self.throttle_value = user_input

        except ValueError:
            print("Please enter a valid number from 0 to 100.")
            self.throttle_display_label.config(text="Invalid Input")


if __name__ == "__main__":
    app = App()
    app.mainloop()
