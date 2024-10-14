import serial
import time
import cv2

# Configure the serial connection
port = 'COM5'
baudrate = 115200
timeout = 1 # in seconds, adjust if needed

try:
    # Open the serial port
    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        print(f"Connected to {port} with baudrate {baudrate}")

        # Initialize OpenCV window
        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)

        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf').rstrip()
            if line:
                print(line)  # Assuming line contains some useful information from the camera
                # Read the image from camera (replace this with actual code to get image data from the camera)
                # For demo purpose, let's just create a placeholder image
                image = cv2.imread('placeholder_image.jpg')

                # Display the image
                cv2.imshow('Camera Feed', image)
                cv2.waitKey(1)  # Update window, wait 1ms

            else:
                # Optionally, you can add a small delay to reduce CPU usage in case of no data
                time.sleep(0.01)

except serial.SerialException as e:
    print(f"Error connection to {port}: {e}")

except KeyboardInterrupt:
    print("\nExiting. . .")

# Close OpenCV windows
cv2.destroyAllWindows()
