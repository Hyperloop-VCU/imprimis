from sys import path
path.insert(0, r'C:\Users\Raymond\AppData\Local\Packages\PythonSoftwareFoundation.Python.3.10_qbz5n2kfra8p0\LocalCache\local-packages\Python310\site-packages')
import serial
import struct
from time import sleep

ser = serial.Serial('COM4', 115200, timeout=0.5, bytesize=8)

def getData():
    ser.write(b'd')
    # Read bytes from serial
    data_bytes = ser.read(12)  # read three floats

    if len(data_bytes) == 12:
        # Decode bytes to float
        f1, f2, f3 = struct.unpack('fff', data_bytes)
        return f1, f2, f3

def newSetpoint(linX, angZ):
    ser.write(b't')
    data = struct.pack('ff', linX, angZ)
    ser.write(data)

while True:
    command = input('>')
    if command[0] == 't':
        arg1, arg2 = command.split(" ")[1:]
        newSetpoint(float(arg1), float(arg2))
    elif command == 'd':
        print(getData())
    elif command == 'WAIT':
        print(ser.in_waiting)
    else:
        ser.write(command.encode('ascii'))