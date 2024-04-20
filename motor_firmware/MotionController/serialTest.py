from sys import path
path.insert(0, r'C:\Users\Raymond\AppData\Local\Packages\PythonSoftwareFoundation.Python.3.10_qbz5n2kfra8p0\LocalCache\local-packages\Python310\site-packages')
import serial
import struct
import time

ser = serial.Serial('COM4', 115200, timeout=0.5, bytesize=8)
ser.set_buffer_size(rx_size=12800, tx_size=12800)

def getData():
    ser.write(b'd')
    data_bytes = ser.read(16)  # 4 longs
    if len(data_bytes) == 16:
        return struct.unpack('llll', data_bytes)

def newSetpoint(linX, angZ):
    ser.write(b's')
    data = struct.pack('ff', linX, angZ)
    ser.write(data)

def newSpeed(left, right):
    ser.write(b'o')
    data = struct.pack('ii', left, right)
    ser.write(data)

def getFloats():
    ser.write(b'p')
    data_bytes = ser.read(8)
    if len(data_bytes) == 8:
        return struct.unpack('ff', data_bytes)

def getInts():
    ser.write(b'p')
    data_bytes = ser.read(4)
    if len(data_bytes) == 4:
        return struct.unpack('hh', data_bytes)

while True:
    command = input('>')
    
    if command[0] == 's':
        arg1, arg2 = command.split(" ")[1:]
        newSetpoint(float(arg1), float(arg2))
    if command[0] == 'o':
        arg1, arg2 = command.split(" ")[1:]
        newSpeed(int(arg1), int(arg2))
        
    elif command == 'd':
        print(getData())
    elif command == 'dd':
        for i in range(100):
            time.sleep(0.1)
            print(getData())
            
    elif command == 'f':
        print(getFloats())
    elif command == 'ff':
        for i in range(100):
            time.sleep(0.1)
            print(getFloats())

    elif command == 'i':
        print(getInts())
    elif command == 'ii':
        for i in range(100):
            time.sleep(0.1)
            print(getInts())

    else:
        ser.write(command.encode('ascii'))
