import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time
import struct
from threading import Thread

class Esp32:
    def __init__(self, port: str, baudrate: int, timeout: float):
        
        self.ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout,
        write_timeout=timeout
        )
    
    def send_setpoint(self, left: float, right: float):

        msg = f"s {left:.2f} {right:.2f}"
        self.ser.write(bytes(msg + '\n', 'utf-8'))

    def read_angvels(self) -> tuple[float, float]:
        data = self.ser.read(8)
        if len(data) == 8:
            left, right = struct.unpack('<ff', data)
            return (left, right)
        print(f"Could not read data from ESP32")
        return (0.0, 0.0)
    
    def send_PI(self, p: float, i: float, d: float, motor: int = 1):
        msg = f"e {p:.2f} {i:.2f} {d:.2f} {motor}"
        self.ser.write(bytes(msg + '\n', 'utf-8'))
    
    def close(self):
        if self.ser.open(): self.ser.close()

def init():
    line.set_data([], [])
    return line,

def update(frame):
    t = time.time() - start_time
    x_data.append(t)
    left_data.append(esp32.read_angvels()[1])

    # Keep only the last 10 seconds of data
    if t > 10:
        ax.set_xlim(t - 10, t)

    line.set_data(x_data, left_data)
    esp32.send_setpoint(0.5, 0.5)
    return line,

# Storage for data
x_data, left_data = [], []

# Create the figure and axis
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=1)
ax.set_xlim(0, 10)   # 10 seconds window
ax.set_ylim(-2.0, 2.0)    # adjust depending on expected value range

esp32 = Esp32("/dev/ttyUSB0", 115200, 1.0)

esp32.send_PI(0.6, 2.0, 0.00, 2)
start_time = time.time()

ani = animation.FuncAnimation(fig, update, init_func=init, interval=33, blit=True)
plt.show()
esp32.close()
