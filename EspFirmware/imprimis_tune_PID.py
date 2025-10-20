import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time
import struct
from threading import Thread




# This class handles all communication with the ESP32.

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
    
    def send_PI(self, p: float, i: float, d: float, motor: int):
        msg = f"e {p:.2f} {i:.2f} {d:.2f} {motor}"
        self.ser.write(bytes(msg + '\n', 'utf-8'))

    def reset(self):
        msg = 'r'
        self.ser.write(bytes(msg + '\n', 'utf-8'))
    
    def close(self):
        if self.ser.open(): self.ser.close()




# Initializes the lines to be plotted
def init():
    left_line.set_data([], [])
    right_line.set_data([], [])
    setpoint_line.set_data([], [])
    return left_line, right_line, setpoint_line

# Runs every frame
def update(frame):
    t = time.time() - start_time
    x_data.append(t)
    lAngvel, rAngvel = esp32.read_angvels()
    left_data.append(lAngvel)
    right_data.append(rAngvel)
    setpoint_data.append(cursor_y)

    # Keep only the last 10 seconds of data
    if t > 10:
        ax.set_xlim(t - 10, t)

    left_line.set_data(x_data, left_data)
    right_line.set_data(x_data, right_data)
    setpoint_line.set_data(x_data, setpoint_data)
    esp32.send_setpoint(cursor_y, cursor_y)
    return left_line, right_line, setpoint_line

# Gets the current Y-position of the mouse
def on_mouse_move(event):
    global cursor_y
    if event.inaxes is None:
        cursor_y = 0.0
    else:
        cursor_y = event.ydata


# Storage for data
x_data, left_data, right_data, setpoint_data = [], [], [], []
cursor_y = 0.0

# Create the figure and axis
fig, ax = plt.subplots()
left_line, = ax.plot([], [], lw=1, label="Left")
right_line, = ax.plot([], [], lw=1, label="Right")
setpoint_line, = ax.plot([], [], "k--", lw=1, label="Setpoint")
ax.set_xlim(0, 10)
ax.set_ylim(-3.0, 3.0)
ax.legend(loc="upper right")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angular velocity")

# attach mouse move event callback
fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
esp32 = Esp32("/dev/ttyUSB0", 115200, 1.0)
time.sleep(1)
esp32.send_PI(2.0, 6.0, 0.0, 1)
esp32.send_PI(2.0, 6.0, 0.0, 2)
esp32.reset()
start_time = time.time()

ani = animation.FuncAnimation(fig, update, init_func=init, interval=50, blit=True)
plt.show()

esp32.close()
