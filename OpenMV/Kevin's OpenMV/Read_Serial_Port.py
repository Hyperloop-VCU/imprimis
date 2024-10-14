import serial
import time

# Configure the serial connection
port = 'COM5'
baudrate = 115200
timeout = 1 # in seconds, adjust if needed

try:
    # Open the serial port
    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        print(f"Connected to {port} with baudrate {baudrate}")

        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf').rstrip()
            if line:
                print(line)
            else:
                # Optionally, you can add a small delay to reduce CPU usage in case of no data
                time.sleep(0.01)

except serial.SerialException as e:
    print(f"Error connection to {port}: {e}")

except KeyboardInterrupt:
    print("\nExiting. . .")