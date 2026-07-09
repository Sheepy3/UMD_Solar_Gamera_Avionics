import tkinter as tk
from tkinter import ttk

from crsf_parser.payloads import PacketsTypes
from crsf_parser.handling import crsf_build_frame

import serial
import time


# -------- Packet Constants -------------
FLAG_A = 172    # arm
FLAG_B = 992   # e-stop
FLAG_C = 1811   # reset e-stop, disarmed
FLAG_D = 410   # reset e-stop and arm this not used

NUM_CHANNELS = 16

CRSF_MIN = 172
CRSF_MAX = 2047

PORT = "COM5"
BAUD = 921600

SEND_RATE_MS = 20  # 20 ms = 50 Hz


# -------- Packet Functions -------------

def clamp(value, low=CRSF_MIN, high=CRSF_MAX):
    return max(low, min(high, int(value)))


def build_channels(command_flag, n=0, e=0, s=0, w=0):
    channels = [0] * NUM_CHANNELS

    channels[0] = clamp(command_flag)
    channels[1] = clamp(n)
    channels[2] = clamp(e)
    channels[3] = clamp(s)
    channels[4] = clamp(w)

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


# -------- GUI Application -------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Solar Gamera Controller")
        self.geometry("1000x1000")

        # Packet values:
        # index 0 = flag
        # index 1 = North
        # index 2 = East
        # index 3 = South
        # index 4 = West
        self.data_packet_list = [FLAG_B, 0, 0, 0, 0]

        self.serial_connected = False
        self.ser = None

        self.is_armed = False
        self.is_estopped = True

        self.setup_layout()

        # Try opening serial when GUI starts
        self.connect_serial()

        # Start periodic CRSF send loop
        self.send_loop()

        #self.receive_loop()   <--- Test Reciever

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

    def connect_serial(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
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
            print("Channels:", channels)
            print("Frame hex:", frame.hex())
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
        """
        Non-blocking serial monitor loop.
        Reads incoming serial data and prints it to the VS Code terminal.
        """

        if self.serial_connected and self.ser is not None:
            try:
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()

                    if line:
                        print("RX:", line)

            except serial.SerialException as error:
                self.serial_connected = False
                self.status_label.config(text="Status: Serial read failed")
                print(f"Serial read failed: {error}")

        # Run again after 20 ms
        self.after(20, self.receive_loop)
# -------- Arm Controller Widget -------------

class Arm_Control_Container(ttk.Frame):
    def __init__(self, parent, id, location, throttle_value_percent=0):
        super().__init__(parent)

        self.parent = parent
        self.id = id

        self.throttle_value_percent = float(throttle_value_percent)
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
        return int((throttle_value_percent / 100) * (CRSF_MAX - CRSF_MIN))

    def raw_to_percent(self, throttle_value_raw):
        return (throttle_value_raw / (CRSF_MAX - CRSF_MIN) * 100)

    @property
    def throttle_value(self):
        return self.throttle_value_percent

    @throttle_value.setter
    def throttle_value(self, new_value):
        if new_value > 100 or new_value < 0:
            print("Incorrect throttle value entered. Use 0-100.")
            self.throttle_display_label.config(text="Invalid Range")
            return

        self.throttle_value_percent = float(new_value)
        self.throttle_value_raw = self.percent_to_raw(new_value)

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
        self.throttle_value = 0.0

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
