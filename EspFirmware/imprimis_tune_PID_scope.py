import bisect
import math
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
        # write
        msg = f"g"
        self.ser.write(bytes(msg + '\n', 'utf-8'))

        # then read
        line = self.ser.readline()
        if not line:
            print("Failed to read from board A: line is false")
            return 0, 0
        text = line.decode("ascii", errors="ignore").strip()[1:]
        try:
            left, right, mode, conn, latency_ignore = [float(x) for x in text.split(" ")]
        except ValueError as e:
            print(f"Failed to read from board A: {e}")
            return 0, 0

        return left, right
    
    def send_PI(self, p: float, i: float, d: float, motor: int):
        msg = f"e {p:.2f} {i:.2f} {d:.2f} {motor}"
        self.ser.write(bytes(msg + '\n', 'utf-8'))

    def reset(self):
        msg = 'r'
        self.ser.write(bytes(msg + '\n', 'utf-8'))
    
    def close(self):
        if self.ser.open(): self.ser.close()


# Drop the matplotlib default bindings we want for the scope controls, so that
# 'c' does not trigger the "back" navigation action while we are measuring.
for _action, _key in (("back", "c"), ("all_axes", "a"), ("save", "s")):
    _param = f"keymap.{_action}"
    if _param in plt.rcParams and _key in plt.rcParams[_param]:
        plt.rcParams[_param] = [k for k in plt.rcParams[_param] if k != _key]


# Index of the sample closest to x_target. x_data is monotonically increasing.
def nearest_index(x_target):
    if not x_data:
        return None
    i = bisect.bisect_left(x_data, x_target)
    if i <= 0:
        return 0
    if i >= len(x_data):
        return len(x_data) - 1
    return i if (x_data[i] - x_target) < (x_target - x_data[i - 1]) else i - 1


# Snap a cursor ('A' or 'B') onto the nearest captured sample
def place_cursor(which, x_target):
    idx = nearest_index(x_target)
    if idx is None:
        return
    cursors[which] = idx


def fmt(value):
    return "--" if value is None or math.isnan(value) else f"{value:.3f}"


# Builds the oscilloscope-style readout shown while the plot is frozen
def cursor_readout():
    ia, ib = cursors["A"], cursors["B"]
    lines = []
    for name, idx in (("A", ia), ("B", ib)):
        if idx is None:
            lines.append(f"{name}:  --")
        else:
            lines.append(
                f"{name}:  t={x_data[idx] * 1000:9.1f} ms   "
                f"L={fmt(left_data[idx])}  R={fmt(right_data[idx])}  "
                f"SP={fmt(setpoint_data[idx])}"
            )

    if ia is not None and ib is not None:
        dt_ms = (x_data[ib] - x_data[ia]) * 1000.0
        dl = left_data[ib] - left_data[ia]
        dr = right_data[ib] - right_data[ia]
        freq = f"{1000.0 / abs(dt_ms):.2f} Hz" if abs(dt_ms) > 1e-9 else "-- Hz"
        lines.append(f"dt = {dt_ms:+.1f} ms   (1/dt = {freq})")
        lines.append(f"dL = {fmt(dl)}   dR = {fmt(dr)}")
    else:
        lines.append("dt = --  ms")

    return "\n".join(lines)


# Pushes the cursor artists and the readout text into their current state
def refresh_cursors():
    for name, line, pts in (
        ("A", cursor_a_line, cursor_a_pts),
        ("B", cursor_b_line, cursor_b_pts),
    ):
        idx = cursors[name]
        if idx is None or not frozen:
            line.set_visible(False)
            pts.set_visible(False)
            continue
        t = x_data[idx]
        line.set_xdata([t, t])
        line.set_visible(True)
        pts.set_data([t, t, t], [left_data[idx], right_data[idx], setpoint_data[idx]])
        pts.set_visible(True)

    if frozen:
        readout_text.set_text(cursor_readout())
        readout_text.set_visible(True)
        status_text.set_text("FROZEN  --  space: run   left/right click or a/b: cursors   c: clear")
    else:
        readout_text.set_visible(False)
        status_text.set_text("RUNNING  --  space: freeze & measure")


# Oscilloscope look for the frozen snapshot: sample dots and a measuring grid
def set_frozen_style(on: bool):
    for line in (left_line, right_line):
        line.set_marker("." if on else "")
        line.set_markersize(3)
    ax.grid(on, which="both", alpha=0.3)


# Initializes the lines to be plotted
def init():
    left_line.set_data([], [])
    right_line.set_data([], [])
    setpoint_line.set_data([], [])
    return artists()


def artists():
    return (
        left_line, right_line, setpoint_line,
        cursor_a_line, cursor_b_line, cursor_a_pts, cursor_b_pts,
        readout_text, status_text,
    )


# Runs every frame
def update(frame):
    global cursor_y

    # While frozen the traces hold still so they can be measured. We keep the
    # setpoint alive and throw away incoming samples instead of letting them
    # pile up in the OS buffer, which would make the plot lag on un-freeze.
    if frozen:
        esp32.send_setpoint(cursor_y, cursor_y)
        esp32.ser.reset_input_buffer()
        refresh_cursors()
        return artists()

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
    refresh_cursors()
    return artists()


# Gets the current Y-position of the mouse
def on_mouse_move(event):
    global cursor_y, mouse_x
    mouse_x = event.xdata if event.inaxes is not None else None

    # Frozen means hands off the setpoint: the mouse is a measuring tool now.
    if frozen:
        return

    if event.inaxes is None:
        cursor_y = 0.0
    else:
        cursor_y = event.ydata


# Space freezes/unfreezes, a/b drop a cursor at the mouse, c clears them
def on_key(event):
    global frozen

    if event.key == " ":
        frozen = not frozen
        set_frozen_style(frozen)
        if not frozen:
            # Break the trace where the frozen gap is, so the resumed data is
            # not joined to the old samples by a meaningless straight line.
            cursors["A"] = cursors["B"] = None
            x_data.append(time.time() - start_time)
            left_data.append(float("nan"))
            right_data.append(float("nan"))
            setpoint_data.append(float("nan"))
        refresh_cursors()
        fig.canvas.draw_idle()
        return

    if not frozen:
        return

    if event.key in ("a", "b") and mouse_x is not None:
        place_cursor(event.key.upper(), mouse_x)
    elif event.key == "c":
        cursors["A"] = cursors["B"] = None

    refresh_cursors()
    fig.canvas.draw_idle()


# Left click places cursor A, right click places cursor B
def on_click(event):
    if not frozen or event.inaxes is None or event.xdata is None:
        return
    if event.button == 1:
        place_cursor("A", event.xdata)
    elif event.button == 3:
        place_cursor("B", event.xdata)
    refresh_cursors()
    fig.canvas.draw_idle()


# Storage for data
x_data, left_data, right_data, setpoint_data = [], [], [], []
cursor_y = 0.0

# Freeze / measurement state
frozen = False
mouse_x = None
cursors = {"A": None, "B": None}

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

# Oscilloscope cursors and their readout
cursor_a_line = ax.axvline(0.0, color="tab:green", lw=1, visible=False)
cursor_b_line = ax.axvline(0.0, color="tab:red", lw=1, visible=False)
cursor_a_pts, = ax.plot([], [], "o", color="tab:green", ms=5, visible=False)
cursor_b_pts, = ax.plot([], [], "o", color="tab:red", ms=5, visible=False)
readout_text = ax.text(
    0.01, 0.02, "", transform=ax.transAxes, va="bottom", ha="left",
    family="monospace", fontsize=8, visible=False,
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
)
status_text = ax.text(
    0.01, 0.985, "RUNNING  --  space: freeze & measure", transform=ax.transAxes,
    va="top", ha="left", family="monospace", fontsize=8,
)

# attach mouse move event callback
fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
fig.canvas.mpl_connect('key_press_event', on_key)
fig.canvas.mpl_connect('button_press_event', on_click)
esp32 = Esp32("/dev/ttyUSB0", 921600, 1.0)
time.sleep(1)
esp32.send_PI(2.3, 6.2, 0.0, 1)
time.sleep(1)
esp32.send_PI(2.3, 6.2, 0.0, 2)
#esp32.reset()
start_time = time.time()
time.sleep(1)

ani = animation.FuncAnimation(fig, update, init_func=init, interval=10, blit=True)
plt.show()

esp32.close()
