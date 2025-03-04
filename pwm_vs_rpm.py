import serial
import csv
import time

SERIAL_PORT = "/dev/ttyACM0"  # Update this for your system
BAUD_RATE = 115200
CSV_FILE = "motor_data.csv"

# Wait for Teensy to reset after upload
print("Waiting for Teensy...")
time.sleep(2)  # Give time for the serial port to reconnect

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Open CSV file to write data
with open(CSV_FILE, "w", newline="") as file:
    writer = csv.writer(file)
    
    while True:
        line = ser.readline().decode("utf-8").strip()
        if line:
            print(line)  # Print to console
            
            # Write CSV header
            if "PWM,RPM" in line:
                writer.writerow(["PWM", "RPM"])
            elif "Done" in line:
                break  # Stop when Teensy finishes logging
            else:
                writer.writerow(line.split(","))

print(f"Data saved to {CSV_FILE}")
ser.close()
