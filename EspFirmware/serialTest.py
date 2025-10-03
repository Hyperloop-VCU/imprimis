import serial


prt = input("Enter port: ")
arduino = serial.Serial(port=prt, baudrate=115200, timeout=.1)

def send_message(msg):
    arduino.write(bytes(msg + '\n', 'utf-8'))  # Send the message



while True:
    message = input("Enter a message to send to Arduino (or 'exit' to quit): ")
    if message.lower() == "exit":
        break
    send_message(message)

arduino.close()

