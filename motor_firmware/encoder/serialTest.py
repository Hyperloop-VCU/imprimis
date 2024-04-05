from sys import path
path.insert(0, r'C:\Users\Raymond\AppData\Local\Packages\PythonSoftwareFoundation.Python.3.10_qbz5n2kfra8p0\LocalCache\local-packages\Python310\site-packages')
import serial
import struct

ser = serial.Serial('COM3', 115200, timeout=0.5, bytesize=8)
ser.set_buffer_size(rx_size=12800, tx_size=12800)

def getData():
    ser.write(b'd')
    data_bytes = ser.read(16)  # two floats
    if len(data_bytes) == 16:
        return struct.unpack('ffff', data_bytes)

def newSetpoint(linX, angZ):
    ser.write(b't')
    data = struct.pack('ff', linX, angZ)
    ser.write(data)

def getInts():
    ser.write(b'p')
    data_bytes = ser.read(4)
    if len(data_bytes) == 4:
        return struct.unpack('hh', data_bytes)

while True:
    command = input('>')
    if command[0] == 't':
        arg1, arg2 = command.split(" ")[1:]
        newSetpoint(float(arg1), float(arg2))
    elif command == 'd':
        print(getData())
    elif command == 'dd':
        for i in range(1000):
            print(getData())
    elif command == 'p':
        print(getInts())
    elif command == 'pp':
        for i in range(1000):
            print(getInts())
    else:
        ser.write(command.encode('ascii'))