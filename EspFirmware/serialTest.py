import serial
from time import sleep


prt = '/dev/ttyUSB0'
arduino = serial.Serial(port=prt, baudrate=115200, timeout=.1)
rate = 30

def send_message(msg):
    arduino.write(msg + '\n')  # Send the message



while True:
    message = input("Enter a message to send to Arduino (or 'exit' to quit): ")
    if message.lower() == "exit":
        break
    if len(message) != 0 and message[0] == '!':
        while True:
            send_message(message[1:])
            print("Sent: " + message[1:])
            sleep(1/rate)


    send_message(message)

arduino.close()

