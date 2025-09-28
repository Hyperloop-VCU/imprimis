import serial
import threading
import time

# === CONFIGURATION ===
PORT = "/dev/ttyUSB1"   # Change to match your ESP32 port
BAUD = 115200
SEND_RATE = 1.0 / 20.0  


def user_input_loop():
    """Thread to update x and y from user input."""
    global x, y, running
    while running:
        try:
            line = input("Enter new values for x and y (format: x y): ")
            parts = line.strip().split()
            if len(parts) == 2:
                x = float(parts[0])
                y = float(parts[1])
                print(f"Updated values: x={x}, y={y}")
            else:
                print("Invalid format. Please type: x y")
        except Exception as e:
            print(f"Input error: {e}")


def main():
    global x, y, running
    # Initial values
    x, y = 0.0, 0.0
    running = True

    # Open serial connection
    ser = serial.Serial(PORT, BAUD, timeout=1)

    # Start user input thread
    t = threading.Thread(target=user_input_loop, daemon=True)
    t.start()

    print("Starting message loop. Press Ctrl+C to quit.")

    try:
        while running:
            msg = f"s {x:.3f} {y:.3f}\n"
            ser.write(msg.encode())
            time.sleep(SEND_RATE)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        running = False
        ser.close()


if __name__ == "__main__":
    main()
