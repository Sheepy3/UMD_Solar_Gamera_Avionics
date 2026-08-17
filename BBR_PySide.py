"""
Blade Balancing Rig (BBR) Python Script 
For recording, ploting, and storing live current and rpm readings from Arduino's serial monitor
Use in conjuction with "BBR_ArdSide.ino"

Use UV for imports
"""
import serial   #PySerial
import time 
import csv
import re
import matplotlib.pyplot as plt
from datetime import datetime
import sys

# ------------------------------------------------
# User settings 
COM_PORT = 'COM3'             # change to match to Arduino COM port
BAUD_RATE = 9600           # must match Arduino BaudRate

now = datetime.now()
date_format = now.strftime("%Y-%m-%d--%H-%M-%S")
CSV_FILE = "motor_data" + date_format +".csv"


# Data Arrays
time_data = []
rpm_data = []
current_data = []

#--------------------------------------------------
# Serial Setup 
try: 
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout =1)
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"Connected to {COM_PORT}")
    print("Collecting motor data...")
except serial.SerialException: 
    print(f"ERROR: Could not connect to Arduino on {COM_PORT}.")
    print(e)
    sys.exit()

#CSV Setup
csv_file = open(CSV_FILE, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "Time (s)", "RPM", "Current (A)"
])

# Live Plot Setup
plt.ion()
fig, (ax_rpm, ax_current) = plt.subplots(2,1, figsize=(10,7))

# RPM Plot
rpm_line, = ax_rpm.plot([], [])
ax_rpm.set_title("Motor RPM")
ax_rpm.set_xlabel("Time (s)")
ax_rpm.set_ylabel("RPM")
ax_rpm.grid(True)

# Current Plot
current_line, = ax_current.plot([], [])
ax_current.set_title("Motor Current")
ax_current.set_xlabel("Time (s)")
ax_current.set_ylabel("Current (A)")
ax_current.grid(True)

plt.tight_layout()

start_time = time.time() 

#-----------------------------------------------------------
# Main Loop
try:
    while True:
        # -----------------------------------------------------
        # Read & Decode Serial Terminal from Arduino
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        print(line) 
        if not line.startswith("RPM:"):
            continue

        # Extract RPM & Current from arduino message
        rpm_match = re.search(r"RPM:\s*([-+]?\d*\.?\d+)", line)
        current_match = re.search(r"Current \(A\):\s*([-+]?\d*\.?\d+)",line)
        # make sure both values were found
        if rpm_match is None or current_match is None:
            print("Could not parse data line")
            continue

        rpm = float(rpm_match.group(1))
        current = float(current_match.group(1))

        elapsed_time = time.time() - start_time

        # Store in arrays
        time_data.append(elapsed_time)
        rpm_data.append(rpm)
        current_data.append(current)

        #---------------------------------------------
        # Save to CSV
        csv_writer.writerow([ elapsed_time, rpm, current])
        csv_file.flush()

        #----------------------------------------------------
        # Update Plots
        # RPM Plot
        rpm_line.set_data(time_data, rpm_data)
        ax_rpm.relim()
        ax_rpm.autoscale_view()

        # Current Plot
        current_line.set_data(time_data, current_data)
        ax_current.relim()
        ax_current.autoscale_view()

        # Redraw
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

#-----------------------------------------------------
except KeyboardInterrupt: 
     print("\nData collection stopped.")

finally:
    csv_file.close()
    ser.close()

    plt.ioff()

    print(f"Data saved to {CSV_FILE}")

    # Keep final graph open
    plt.show()

