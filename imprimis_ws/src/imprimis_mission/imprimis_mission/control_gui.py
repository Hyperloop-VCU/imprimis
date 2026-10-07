"""IMPRIMIS control window: pick Manual or Automatic, drive with W A S D and the mouse, watch the lap.

The window shows the robot's front camera with a heads-up display over it, in the manner of a game:
a start menu with two mode cards, a pause menu on Esc, and a banner when the finish line is crossed.

Manual driving
  W / S        forward / reverse          Shift   full speed
  A / D        turn left / right          Space   brake
  mouse        look around: sideways turns the view camera without limit (azimuth), up and down tilts it
               between straight down and straight up (elevation). The mouse does not steer.
  C            center the view
  M            switch Manual / Automatic  N       new lap
  Esc          menu                       F11     full screen

In Manual mode this window publishes the velocity command. In Automatic mode it publishes nothing and
the lap manager sends goals to Nav2.

Topics out:  diffbot_base_controller/cmd_vel (TwistStamped, Manual mode only)
             drive_mode_request (String), lap/command (String)
Topics in:   cameras/front/color/image_raw, drive_mode, lap/status, ramp/info, odometry/filtered/local
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QCursor, QFont, QImage, QLinearGradient, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QApplication, QWidget

# palette
RED = QColor(218, 41, 42)        # Automatic
TEAL = QColor(46, 211, 198)      # Manual
AMBER = QColor(255, 180, 0)
WHITE = QColor(255, 255, 255)
GREY = QColor(170, 176, 186)
DARK = QColor(11, 15, 20)
PANEL = QColor(11, 15, 20, 205)


class GuiNode(Node):
    def __init__(self):
        super().__init__('control_gui')
        p = self.declare_parameter
        self.course_file = p('course_file', '').value
        self.v_normal = p('speed_normal', 1.0).value
        self.v_boost = p('speed_boost', 2.1).value      # 4.7 mph. At 2.2 the true speed of hand-driven laps touched 5.05 mph; the limit is 5
        self.v_reverse = p('speed_reverse', 0.6).value
        self.w_max = p('turn_rate_max', 1.4).value
        self.w_key = p('turn_rate_keys', 1.0).value
        self.mouse_sensitivity = p('mouse_radians_per_pixel', 0.004).value   # how far the view turns per pixel of mouse travel
        self.view_rest_elevation = p('view_rest_elevation', -0.2).value      # radians; the centered view looks a little down
        self.screenshot_dir = p('screenshot_dir', '').value
        self.demo_image = p('demo_image', '').value
        self.live_shot_dir = p('live_shot_dir', '').value        # save the window to this folder now and then
        self.live_shot_period = p('live_shot_period', 20.0).value
        self.start_view = p('start_view', 'menu').value

        self.frame = None
        self.frame_count = 0
        self.view_frames = 0        # frames received from the view camera
        self.mode = 'manual'
        self.mode_confirmed = False
        self.status = {}
        self.status_time = 0.0
        self.ramp = {}
        self.governor = {}
        self.notice = None          # (text, time) from the course watcher
        self.odom = None
        self.stamp = None           # header stamp of the newest odometry message: the robot's clock

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(TwistStamped, p('cmd_topic', 'diffbot_base_controller/cmd_vel').value, 5)
        self.mode_pub = self.create_publisher(String, 'drive_mode_request', 5)
        self.command_pub = self.create_publisher(String, 'lap/command', 5)
        # The window shows the view camera, which stands on a pan and tilt head and follows the mouse. The front
        # camera, which navigation uses, is the fallback when the robot model has no view camera.
        self.front_topic = p('image_topic', 'cameras/front/color/image_raw').value
        self.front_sub = None
        self.create_subscription(Image, p('view_image_topic', 'cameras/view/image_raw').value, self.view_image_cb,
                                 qos_profile_sensor_data)
        self.pan_pub = self.create_publisher(Float64, 'view/pan_cmd', 5)
        self.tilt_pub = self.create_publisher(Float64, 'view/tilt_cmd', 5)
        self.create_subscription(String, 'lap/notice', self.course_cb, 5)
        self.create_subscription(String, 'drive_mode', self.mode_cb, latched)
        self.create_subscription(String, 'lap/status', self.status_cb, 5)
        self.create_subscription(String, 'ramp/info', self.ramp_cb, 5)
        self.create_subscription(String, 'speed_governor/info', self.governor_cb, 5)
        self.create_subscription(String, 'course/changed', self.course_cb, 5)
        self.create_subscription(Odometry, p('odom_topic', 'odometry/filtered/local').value, self.odom_cb, 5)

    def view_image_cb(self, msg):
        if msg.encoding in ('rgb8', 'bgr8'):
            self.frame = msg
            self.frame_count += 1
            self.view_frames += 1

    def image_cb(self, msg):
        if self.view_frames == 0 and msg.encoding in ('rgb8', 'bgr8'):
            self.frame = msg
            self.frame_count += 1

    def use_front_camera(self):
        if self.front_sub is None:
            self.front_sub = self.create_subscription(Image, self.front_topic, self.image_cb, qos_profile_sensor_data)

    def look(self, azimuth, elevation):
        """Point the view camera. Azimuth is counted to the left and has no limit; elevation is positive upward."""
        self.pan_pub.publish(Float64(data=float(azimuth)))
        self.tilt_pub.publish(Float64(data=float(-elevation)))     # the tilt joint counts downward

    def mode_cb(self, msg):
        self.mode = msg.data
        self.mode_confirmed = True

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_time = time.time()
        except ValueError:
            pass

    def ramp_cb(self, msg):
        try:
            self.ramp = json.loads(msg.data)
        except ValueError:
            pass

    def governor_cb(self, msg):
        try:
            self.governor = json.loads(msg.data)
        except ValueError:
            pass

    def course_cb(self, msg):
        try:
            self.notice = (json.loads(msg.data).get('text', ''), time.time())
        except ValueError:
            pass

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x,
                     msg.twist.twist.angular.z)
        self.stamp = msg.header.stamp

    def request_mode(self, mode):
        self.mode_pub.publish(String(data=mode))
        if not self.mode_confirmed:
            self.mode = mode          # no lap manager running: the window is the only authority

    def command(self, text):
        self.command_pub.publish(String(data=text))

    def drive(self, v, w):
        msg = TwistStamped()
        if self.stamp is not None:       # a zero stamp is read by the drive controller as "now"
            msg.header.stamp = self.stamp
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clock_text(seconds):
    seconds = max(0.0, float(seconds or 0.0))
    return '%02d:%04.1f' % (int(seconds // 60), seconds % 60)


class ControlWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('IMPRIMIS Control')
        self.resize(1280, 760)
        self.setMinimumSize(900, 560)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.view = node.start_view if node.start_view in ('menu', 'drive') else 'menu'   # menu | drive | controls | finish
        self.keys = set()
        self.buttons = []             # (QRectF, callable) rebuilt on every paint
        self.hover = QPointF(-1, -1)
        self.v = 0.0
        self.w = 0.0
        self.azimuth = 0.0                       # view direction, radians to the left of straight ahead; no limit
        self.elevation = node.view_rest_elevation   # radians above the horizon, held between -pi/2 and +pi/2
        self.look_sent = (None, None, 0.0)
        self.started = time.time()
        self.last_command_time = 0.0
        self.last_mouse = None
        self.warp = True
        self.captured = False
        self.finish_shown_for = None
        self.image = None
        self.image_seen = -1
        self.toast = ('', 0.0)
        self.course = None
        self.map_bounds = None
        self.course_mtime = None
        self.ticks = 0
        self.load_course()
        if node.demo_image and os.path.isfile(node.demo_image):
            self.image = QImage(node.demo_image)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.control_tick)
        self.control_timer.start(50)
        self.paint_timer = QTimer(self)
        self.paint_timer.timeout.connect(self.update)
        self.paint_timer.start(50)
        self.shot_count = 0
        if node.live_shot_dir:
            os.makedirs(node.live_shot_dir, exist_ok=True)
            self.shot_timer = QTimer(self)
            self.shot_timer.timeout.connect(self.live_shot)
            self.shot_timer.start(int(node.live_shot_period * 1000))

    def live_shot(self):
        self.shot_count += 1
        self.grab().save(os.path.join(self.node.live_shot_dir, 'live_%03d_%s.png' % (self.shot_count, self.view)))

    def load_course(self):
        """Read the course file for the map. Called again whenever the file changes (a barrel moved in Blender)."""
        path = self.node.course_file
        if not (path and os.path.isfile(path)):
            return
        try:
            mtime = os.stat(path).st_mtime
            if mtime == self.course_mtime:
                return
            with open(path, encoding='utf-8') as f:
                course = json.load(f)
        except (OSError, ValueError):
            return
        self.course_mtime = mtime
        self.course = course
        xs = [p[0] for line in course.get('lane_lines', []) for p in line] + [b['xy'][0] for b in course.get('barrels', [])]
        ys = [p[1] for line in course.get('lane_lines', []) for p in line] + [b['xy'][1] for b in course.get('barrels', [])]
        if xs:
            self.map_bounds = (min(xs) - 2, max(xs) + 2, min(ys) - 2, max(ys) + 2)

    # ------------------------------------------------------------------ state helpers
    @property
    def mode(self):
        return self.node.mode

    @property
    def lap_state(self):
        return self.node.status.get('state', 'none')

    def driving(self):
        return self.view == 'drive' and self.mode == 'manual' and self.lap_state != 'finished'

    def looking(self):
        """The mouse turns the view in both modes, whenever the driving view is up."""
        return self.view == 'drive' and self.lap_state != 'finished'

    def center_view(self):
        self.azimuth = 2 * math.pi * round(self.azimuth / (2 * math.pi))   # the nearest "straight ahead": no spinning back
        self.elevation = self.node.view_rest_elevation

    def set_view(self, screen):
        self.view = screen
        self.keys.clear()
        self.capture(self.looking())

    def capture(self, on):
        """Hide the pointer and use its movement for looking around, or give it back."""
        if on and not self.captured:
            self.setCursor(Qt.BlankCursor)
            self.last_mouse = None
            self.captured = True
        elif not on and self.captured:
            self.unsetCursor()
            self.captured = False

    def choose_mode(self, mode):
        if self.lap_state in ('finished', 'failed'):
            self.node.command('new_lap')
            self.finish_shown_for = self.node.status.get('runs')
        self.node.request_mode(mode)
        self.node.mode = mode if not self.node.mode_confirmed else self.node.mode
        self.view = 'drive'
        self.keys.clear()
        self.capture(True)
        self.toast = ('MANUAL: W A S D to drive, mouse to look' if mode == 'manual'
                      else 'AUTOMATIC: the robot drives the remembered route. Mouse to look', time.time())

    def new_lap(self):
        self.node.command('new_lap')
        self.finish_shown_for = self.node.status.get('runs')
        self.toast = ('NEW LAP ARMED', time.time())
        if self.view == 'finish':
            self.set_view('drive')

    # ------------------------------------------------------------------ input
    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        k = e.key()
        if k == Qt.Key_Escape:
            self.set_view('drive' if self.view in ('menu', 'controls') else 'menu')
        elif k == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
        elif k == Qt.Key_M and self.view == 'drive':
            self.choose_mode('automatic' if self.mode == 'manual' else 'manual')
        elif k == Qt.Key_N and self.view in ('drive', 'finish'):
            self.new_lap()
        elif k == Qt.Key_C and self.view == 'drive':
            self.center_view()
        elif k in (Qt.Key_Return, Qt.Key_Enter) and self.view == 'finish':
            self.new_lap()
        else:
            self.keys.add(k)
            if self.view == 'drive' and self.mode == 'automatic' and k in (Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D):
                self.toast = ('AUTOMATIC MODE. Press M to take over.', time.time())

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.keys.discard(e.key())

    def focusOutEvent(self, e):
        self.keys.clear()

    def mouseMoveEvent(self, e):
        self.hover = QPointF(e.pos())
        if not self.captured:
            return
        pos = e.globalPos()
        if self.last_mouse is not None:
            k = self.node.mouse_sensitivity
            # Azimuth: mouse to the right looks to the right, and it may go round and round.
            self.azimuth -= (pos.x() - self.last_mouse.x()) * k
            # Elevation: mouse up looks up, and stops at straight up and straight down so the view never flips over.
            self.elevation = clamp(self.elevation - (pos.y() - self.last_mouse.y()) * k, -math.pi / 2, math.pi / 2)
        self.last_mouse = pos
        if self.warp:
            center = self.mapToGlobal(self.rect().center())
            if abs(pos.x() - center.x()) > self.width() * 0.25 or abs(pos.y() - center.y()) > self.height() * 0.25:
                QCursor.setPos(center)
                now = QCursor.pos()
                if abs(now.x() - center.x()) <= 2 and abs(now.y() - center.y()) <= 2:
                    self.last_mouse = center
                else:
                    self.warp = False       # this display server will not move the pointer for us

    def mousePressEvent(self, e):
        if e.button() != Qt.LeftButton:
            return
        for rect, action in self.buttons:
            if rect.contains(QPointF(e.pos())):
                action()
                return

    def closeEvent(self, e):
        if self.mode == 'manual':
            self.node.drive(0.0, 0.0)
        e.accept()

    # ------------------------------------------------------------------ manual driving
    def control_tick(self):
        dt = 0.05
        n = self.node
        target_v, w = 0.0, 0.0
        self.ticks += 1
        if self.ticks % 20 == 0:
            self.load_course()
        if n.notice is not None:
            self.toast = n.notice
            n.notice = None
        if self.lap_state == 'finished' and self.finish_shown_for != n.status.get('runs'):
            self.finish_shown_for = n.status.get('runs')
            self.set_view('finish')
        if n.view_frames == 0 and time.time() - self.started > 6.0:
            n.use_front_camera()             # this robot model has no view camera: show the front camera
        self.capture(self.looking())
        if self.looking():
            # the arrow keys look around too, for a laptop without a mouse
            rate = 1.6 * dt
            self.azimuth += ((Qt.Key_Left in self.keys) - (Qt.Key_Right in self.keys)) * rate
            self.elevation = clamp(self.elevation + ((Qt.Key_Up in self.keys) - (Qt.Key_Down in self.keys)) * rate,
                                   -math.pi / 2, math.pi / 2)
            if not self.warp and self.hover.x() >= 0:      # pointer pinned at an edge of the window: keep turning the view
                ex = self.hover.x() / max(1.0, self.width())
                ey = self.hover.y() / max(1.0, self.height())
                self.azimuth += (1.2 * dt if ex < 0.03 else (-1.2 * dt if ex > 0.97 else 0.0))
                self.elevation = clamp(self.elevation + (0.8 * dt if ey < 0.03 else (-0.8 * dt if ey > 0.97 else 0.0)),
                                       -math.pi / 2, math.pi / 2)
        sent_az, sent_el, sent_time = self.look_sent
        if sent_az is None or abs(self.azimuth - sent_az) > 0.002 or abs(self.elevation - sent_el) > 0.002 \
                or time.time() - sent_time > 0.5:
            n.look(self.azimuth, self.elevation)
            self.look_sent = (self.azimuth, self.elevation, time.time())
        if self.driving():
            # W A S D act on the rover alone, in its own frame: W and S along its length, A and D turn it.
            # Where the camera looks changes nothing here. A rover on two drive wheels cannot move sideways.
            forward, back = Qt.Key_W in self.keys, Qt.Key_S in self.keys
            if forward and not back:
                target_v = n.v_boost if Qt.Key_Shift in self.keys else n.v_normal
            elif back and not forward:
                target_v = -n.v_reverse
            w = clamp(((Qt.Key_A in self.keys) - (Qt.Key_D in self.keys)) * n.w_key, -n.w_max, n.w_max)
            if Qt.Key_Space in self.keys:
                target_v, w = 0.0, 0.0
                self.v = 0.0
        self.v += clamp(target_v - self.v, -3.0 * dt, 3.0 * dt)
        self.w = w
        # Publish only while there is something to say, and for half a second after, so that the
        # robot is told to stop. Staying quiet otherwise leaves the way free for a goal set in RViz.
        if abs(self.v) > 1e-3 or abs(self.w) > 1e-3:
            self.last_command_time = time.time()
        if self.mode == 'manual' and time.time() - self.last_command_time < 0.5:
            n.drive(self.v, self.w)

    # ------------------------------------------------------------------ painting
    def make_font(self, size, bold=True, mono=False):
        f = QFont('DejaVu Sans Mono' if mono else 'DejaVu Sans')
        f.setPixelSize(max(8, int(size * self.scale)))
        f.setBold(bold)
        if not mono:
            f.setLetterSpacing(QFont.PercentageSpacing, 104)
        return f

    def text(self, p, rect, s, size, color=WHITE, align=Qt.AlignLeft | Qt.AlignVCenter, bold=True, mono=False, shadow=True):
        p.setFont(self.make_font(size, bold, mono))
        if shadow:
            p.setPen(QColor(0, 0, 0, 190))
            off = max(1.0, 2.0 * self.scale)
            p.drawText(rect.translated(off, off), align, s)
        p.setPen(color)
        p.drawText(rect, align, s)

    def slanted(self, rect, lean):
        return QPolygonF([QPointF(rect.left() + lean, rect.top()), QPointF(rect.right(), rect.top()),
                          QPointF(rect.right() - lean, rect.bottom()), QPointF(rect.left(), rect.bottom())])

    def block_button(self, p, rect, label, action, accent=None, enabled=True):
        """A square-cornered beveled button, in the manner of Minecraft's menus."""
        hot = enabled and rect.contains(self.hover)
        b = max(2.0, 3.0 * self.scale)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0))
        p.drawRect(rect)
        inner = rect.adjusted(b, b, -b, -b)
        face = QColor(108, 112, 158) if hot else (QColor(96, 96, 100) if enabled else QColor(48, 48, 52))
        p.setBrush(face)
        p.drawRect(inner)
        p.setBrush(QColor(255, 255, 255, 95 if enabled else 30))
        p.drawRect(QRectF(inner.left(), inner.top(), inner.width(), b))
        p.drawRect(QRectF(inner.left(), inner.top(), b, inner.height()))
        p.setBrush(QColor(0, 0, 0, 110))
        p.drawRect(QRectF(inner.left(), inner.bottom() - b, inner.width(), b))
        p.drawRect(QRectF(inner.right() - b, inner.top(), b, inner.height()))
        if accent is not None:
            p.setBrush(accent)
            p.drawRect(QRectF(inner.left(), inner.top(), 3 * b, inner.height()))
        self.text(p, rect, label, 20, QColor(255, 255, 170) if hot else (WHITE if enabled else QColor(120, 120, 120)), Qt.AlignCenter)
        if enabled:
            self.buttons.append((rect, action))

    def mode_card(self, p, rect, title, lines, color, mode, glyph):
        """One of the two big mode cards on the start menu."""
        hot = rect.contains(self.hover)
        active = self.mode == mode
        lean = 26 * self.scale
        shape = self.slanted(rect, lean)
        p.setPen(QPen(color if (hot or active) else QColor(90, 96, 106), (4 if hot else 2) * self.scale))
        grad = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        grad.setColorAt(0, QColor(color.red(), color.green(), color.blue(), 70 if hot else 36))
        grad.setColorAt(1, QColor(8, 10, 14, 235))
        p.setBrush(QBrush(grad))
        p.drawPolygon(shape)
        strip = QRectF(rect.left() + lean, rect.top(), rect.width() - lean, 10 * self.scale)
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawPolygon(self.slanted(strip, 4 * self.scale))
        pad = 34 * self.scale
        glyph_rect = QRectF(rect.left() + pad, rect.top() + 34 * self.scale, rect.width() - 2 * pad, 92 * self.scale)
        glyph(p, glyph_rect, color)
        self.text(p, QRectF(rect.left() + pad, glyph_rect.bottom() + 12 * self.scale, rect.width() - 2 * pad, 46 * self.scale),
                  title, 34, WHITE)
        y = glyph_rect.bottom() + 64 * self.scale
        for line in lines:
            self.text(p, QRectF(rect.left() + pad - 8 * self.scale, y, rect.width() - 2 * pad, 26 * self.scale), line, 15, GREY, bold=False)
            y += 25 * self.scale
        tag = 'ACTIVE' if active else 'SELECT'
        tag_rect = QRectF(rect.left() + pad - 18 * self.scale, rect.bottom() - 52 * self.scale, 150 * self.scale, 32 * self.scale)
        p.setPen(Qt.NoPen)
        p.setBrush(color if (active or hot) else QColor(60, 64, 72))
        p.drawPolygon(self.slanted(tag_rect, 10 * self.scale))
        self.text(p, tag_rect, tag, 15, DARK if (active or hot) else WHITE, Qt.AlignCenter, shadow=False)
        self.buttons.append((rect, lambda m=mode: self.choose_mode(m)))

    def glyph_keys(self, p, rect, color):
        """W A S D key caps and a mouse."""
        s = min(rect.height() / 2.15, rect.width() / 7.0)
        x0, y0 = rect.left(), rect.top()
        for label, col, row in (('W', 1, 0), ('A', 0, 1), ('S', 1, 1), ('D', 2, 1)):
            r = QRectF(x0 + col * s * 1.08, y0 + row * s * 1.08, s, s)
            p.setPen(QPen(color, 2 * self.scale))
            p.setBrush(QColor(0, 0, 0, 120))
            p.drawRect(r)
            self.text(p, r, label, 20, WHITE, Qt.AlignCenter)
        m = QRectF(x0 + 4.1 * s, y0 + 0.1 * s, 1.15 * s, 1.95 * s)
        p.setPen(QPen(color, 2 * self.scale))
        p.setBrush(QColor(0, 0, 0, 120))
        p.drawRoundedRect(m, 0.5 * s, 0.5 * s)
        p.drawLine(QPointF(m.center().x(), m.top()), QPointF(m.center().x(), m.top() + 0.7 * s))
        p.drawLine(QPointF(m.left(), m.top() + 0.7 * s), QPointF(m.right(), m.top() + 0.7 * s))
        self.text(p, QRectF(m.right() + 0.25 * s, m.top(), 2.4 * s, m.height()), '↔', 34, color)

    def glyph_route(self, p, rect, color):
        """A looping route with a finish flag."""
        path = QPainterPath()
        w, h = rect.width() * 0.62, rect.height()
        x0, y0 = rect.left(), rect.top()
        path.moveTo(x0 + 0.05 * w, y0 + 0.85 * h)
        path.cubicTo(x0 + 0.05 * w, y0 + 0.05 * h, x0 + 0.55 * w, y0 + 0.05 * h, x0 + 0.55 * w, y0 + 0.45 * h)
        path.cubicTo(x0 + 0.55 * w, y0 + 0.95 * h, x0 + 0.98 * w, y0 + 0.95 * h, x0 + 0.98 * w, y0 + 0.2 * h)
        p.setPen(QPen(color, 5 * self.scale, Qt.DashLine, Qt.RoundCap))
        p.setBrush(Qt.NoBrush)
        p.drawPath(path)
        p.setPen(Qt.NoPen)
        p.setBrush(WHITE)
        p.drawEllipse(QPointF(x0 + 0.05 * w, y0 + 0.85 * h), 7 * self.scale, 7 * self.scale)
        fx, fy, c = x0 + 0.98 * w, y0 + 0.2 * h, 9 * self.scale
        for i in range(3):
            for j in range(2):
                p.setBrush(WHITE if (i + j) % 2 == 0 else DARK)
                p.drawRect(QRectF(fx + i * c, fy - (2 - j) * c, c, c))
        p.setPen(QPen(WHITE, 2 * self.scale))
        p.drawLine(QPointF(fx, fy - 2 * c), QPointF(fx, fy + 1.5 * c))

    def paintEvent(self, _):
        n = self.node
        self.scale = min(self.width() / 1280.0, self.height() / 760.0)
        self.buttons = []
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        W, H, s = self.width(), self.height(), self.scale

        # camera
        if n.frame is not None and n.frame_count != self.image_seen:
            m = n.frame
            fmt = QImage.Format_RGB888 if m.encoding == 'rgb8' else QImage.Format_BGR888
            self.image = QImage(bytes(m.data), m.width, m.height, m.step, fmt).copy()
            self.image_seen = n.frame_count
        p.fillRect(self.rect(), DARK)
        if self.image is not None and not self.image.isNull():
            iw, ih = self.image.width(), self.image.height()
            k = max(W / iw, H / ih)
            p.setRenderHint(QPainter.SmoothPixmapTransform, False)    # the picture is large; smooth scaling costs a lot
            p.drawImage(QRectF((W - iw * k) / 2, (H - ih * k) / 2, iw * k, ih * k), self.image)
        else:
            self.text(p, QRectF(0, H * 0.42, W, 60 * s), 'NO CAMERA SIGNAL', 30, QColor(120, 128, 140), Qt.AlignCenter)
            self.text(p, QRectF(0, H * 0.42 + 54 * s, W, 30 * s), 'waiting for cameras/view/image_raw', 15,
                      QColor(100, 106, 116), Qt.AlignCenter, bold=False)
        for top, y0, y1 in ((True, 0, 120 * s), (False, H - 190 * s, H)):
            g = QLinearGradient(0, y0, 0, y1)
            g.setColorAt(0, QColor(0, 0, 0, 190 if top else 0))
            g.setColorAt(1, QColor(0, 0, 0, 0 if top else 200))
            p.fillRect(QRectF(0, y0, W, y1 - y0), QBrush(g))

        if self.view == 'drive':
            self.paint_hud(p, W, H, s)
        if self.view == 'menu':
            self.paint_menu(p, W, H, s)
        elif self.view == 'controls':
            self.paint_controls(p, W, H, s)
        elif self.view == 'finish':
            self.paint_finish(p, W, H, s)
        p.end()

    def paint_hud(self, p, W, H, s):
        n = self.node
        st = n.status
        mode = self.mode
        color = TEAL if mode == 'manual' else RED
        # mode badge
        badge = QRectF(22 * s, 20 * s, 250 * s, 46 * s)
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawPolygon(self.slanted(badge, 16 * s))
        self.text(p, badge, 'MANUAL' if mode == 'manual' else 'AUTOMATIC', 24, DARK if mode == 'manual' else WHITE, Qt.AlignCenter, shadow=False)
        self.text(p, QRectF(badge.right() + 10 * s, badge.top(), 300 * s, badge.height()), '[M] SWITCH   [ESC] MENU', 13, GREY, bold=False)

        # lap clock
        state = st.get('state', 'none')
        label = {'ready': 'READY', 'running': 'LAP RUNNING', 'finished': 'LAP COMPLETE', 'failed': 'RUN STOPPED'}.get(state, 'NO COURSE FILE')
        self.text(p, QRectF(0, 12 * s, W, 44 * s), clock_text(st.get('lap_time', 0.0)), 38, WHITE, Qt.AlignCenter, mono=True)
        detail = label
        if 'checkpoints' in st:
            detail += '   ·   CHECKPOINT %d/%d   ·   %.0f m' % (st.get('checkpoint', 0), st['checkpoints'], st.get('distance', 0.0))
            if st.get('average_mph'):
                detail += '   ·   AVERAGE %.1f mph' % st['average_mph']
        self.text(p, QRectF(0, 56 * s, W, 24 * s), detail, 14, AMBER if state == 'running' else GREY, Qt.AlignCenter)
        # where the view points, when it is not straight ahead
        az = math.degrees(math.atan2(math.sin(self.azimuth), math.cos(self.azimuth)))
        el = math.degrees(self.elevation - n.view_rest_elevation)
        if abs(az) > 3.0 or abs(el) > 3.0:
            side = 'BEHIND' if abs(az) > 170 else ('%.0f° %s' % (abs(az), 'LEFT' if az > 0 else 'RIGHT') if abs(az) > 3.0 else 'AHEAD')
            tilt = '   ·   %.0f° %s' % (abs(el), 'UP' if el > 0 else 'DOWN') if abs(el) > 3.0 else ''
            self.text(p, QRectF(0, 82 * s, W, 22 * s), 'VIEW  %s%s   ·   [C] CENTER' % (side, tilt), 13, WHITE, Qt.AlignCenter, bold=False)

        # best and run counter
        best = st.get('best_time')
        self.text(p, QRectF(W - 330 * s, 16 * s, 308 * s, 28 * s), 'BEST  ' + (clock_text(best) if best else '--:--.-'), 20, WHITE,
                  Qt.AlignRight | Qt.AlignVCenter, mono=True)
        self.text(p, QRectF(W - 330 * s, 44 * s, 308 * s, 22 * s), 'RUNS ON RECORD  %s' % st.get('runs', 0), 13, GREY,
                  Qt.AlignRight | Qt.AlignVCenter, bold=False)

        # status chips
        chips = []
        ramp = n.ramp.get('state', st.get('ramp', 'flat'))
        if ramp == 'ramp_ahead':
            chips.append(('RAMP AHEAD  %.1f m  %.0f%%' % (n.ramp.get('distance') or 0.0, 100 * (n.ramp.get('grade') or 0.0)), AMBER))
        elif ramp in ('climbing', 'descending'):
            chips.append(('RAMP: %s  %.0f%%' % (ramp.upper(), 100 * (n.ramp.get('grade') or 0.0)), AMBER))
        lane = st.get('lane', {})
        if lane.get('cells'):
            left = lane.get('left_offset')
            right = lane.get('right_offset')
            chips.append(('LANE  L %s  R %s' % ('%.1f m' % left if left is not None else '--', '%.1f m' % right if right is not None else '--'), WHITE))
        if st.get('barrel_contacts') or st.get('lane_touches') or st.get('pothole_touches'):
            chips.append(('CONTACTS  barrel %d  line %d  pothole %d' % (st.get('barrel_contacts', 0), st.get('lane_touches', 0),
                                                                    st.get('pothole_touches', 0)), RED))
        ahead = st.get('pothole_ahead')
        if ahead:
            chips.append(('POTHOLE  %.1f m ahead' % ahead, AMBER))
        if mode == 'automatic' and st.get('goals'):
            chips.append(('ROUTE  %d%%' % round(100.0 * st.get('goal', 0) / max(1, st['goals'])), RED))
        if mode == 'automatic' and n.governor.get('limit_mph'):
            why = str(n.governor.get('reason', '')).split(',')[0]
            why = 'clear road' if why == 'clear' else why.split(' ')[0]
            chips.append(('LIMIT  %.1f mph  %s' % (n.governor['limit_mph'], why), AMBER if n.governor['limit_mph'] < 4.5 else WHITE))
        if st and not st.get('nav_ready', True):
            chips.append(('NAVIGATION STARTING', AMBER))
        edge, edge_limit = st.get('boundary_m'), st.get('boundary_limit')
        if mode == 'automatic' and edge is not None and edge_limit and edge > 0.6 * edge_limit:
            chips.append(('COURSE EDGE  %.1f of %.0f m' % (edge, edge_limit), RED if edge > edge_limit else AMBER))
        y = 84 * s
        for label, c in chips:
            r = QRectF(W - 300 * s, y, 278 * s, 28 * s)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 150))
            p.drawPolygon(self.slanted(r, 8 * s))
            p.setBrush(c)
            p.drawRect(QRectF(r.left() + 8 * s, r.top(), 5 * s, r.height()))
            self.text(p, r.adjusted(22 * s, 0, -8 * s, 0), label, 13, WHITE, shadow=False)
            y += 34 * s

        # keys
        size = 46 * s
        kx, ky = 28 * s, H - 150 * s
        active = self.driving()
        for label, key, col, row in (('W', Qt.Key_W, 1, 0), ('A', Qt.Key_A, 0, 1), ('S', Qt.Key_S, 1, 1), ('D', Qt.Key_D, 2, 1)):
            r = QRectF(kx + col * (size + 6 * s), ky + row * (size + 6 * s), size, size)
            down = active and key in self.keys
            p.setPen(QPen(TEAL if active else QColor(90, 96, 106), 2 * s))
            p.setBrush(TEAL if down else QColor(0, 0, 0, 140))
            p.drawRect(r)
            self.text(p, r, label, 20, DARK if down else (WHITE if active else GREY), Qt.AlignCenter, shadow=not down)
        hint = 'SHIFT full speed    SPACE brake    MOUSE look    C center' if mode == 'manual' \
            else 'AUTOPILOT ENGAGED    MOUSE look    press M to take over'
        self.text(p, QRectF(kx, H - 38 * s, 520 * s, 22 * s), hint, 13, GREY, bold=False)

        # speed and steering
        speed = n.odom[3] if n.odom else st.get('speed', 0.0)
        turn = n.odom[4] if n.odom else 0.0
        cx = W / 2
        self.text(p, QRectF(cx - 200 * s, H - 132 * s, 400 * s, 56 * s), '%.1f' % abs(speed), 54, WHITE, Qt.AlignCenter, mono=True)
        self.text(p, QRectF(cx - 200 * s, H - 80 * s, 400 * s, 22 * s),
                  'm/s   ·   %.1f mph%s' % (abs(speed) * 2.237, '   ·   REVERSE' if speed < -0.05 else ''), 14,
                  AMBER if speed < -0.05 else GREY, Qt.AlignCenter)
        bar = QRectF(cx - 170 * s, H - 52 * s, 340 * s, 8 * s)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(255, 255, 255, 50))
        p.drawRect(bar)
        p.setBrush(color)
        p.drawRect(QRectF(bar.left(), bar.top(), bar.width() * clamp(abs(speed) / 2.2, 0, 1), bar.height()))
        steer = QRectF(cx - 170 * s, H - 36 * s, 340 * s, 6 * s)
        p.setBrush(QColor(255, 255, 255, 50))
        p.drawRect(steer)
        frac = clamp(-turn / 1.5, -1, 1)
        p.setBrush(WHITE)
        p.drawRect(QRectF(cx, steer.top(), steer.width() / 2 * frac, steer.height()) if frac >= 0 else
                   QRectF(cx + steer.width() / 2 * frac, steer.top(), -steer.width() / 2 * frac, steer.height()))
        p.drawRect(QRectF(cx - 1 * s, steer.top() - 4 * s, 2 * s, steer.height() + 8 * s))

        self.paint_map(p, QRectF(W - 250 * s, H - 250 * s, 228 * s, 228 * s), s)
        self.paint_arming(p, W, H, s, st)

        # toast
        text, when = self.toast
        message = st.get('message') if state == 'failed' else ''
        if message:
            text, when = message + '   ·   press N for a new lap', time.time()
        if text and time.time() - when < 4.0:
            r = QRectF(W / 2 - 330 * s, H * 0.72, 660 * s, 40 * s)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 170))
            p.drawPolygon(self.slanted(r, 12 * s))
            self.text(p, r, text, 16, WHITE, Qt.AlignCenter, shadow=False)

    def paint_arming(self, p, W, H, s, st):
        """The arming check: what is being checked before an automatic run, and what stopped it if it was refused."""
        arming = st.get('arming')
        checks = st.get('arming_checks') or []
        if arming not in ('checking', 'refused') or not checks or (arming == 'refused' and st.get('arming_age', 0.0) > 15.0):
            return
        row = 30 * s
        panel = QRectF(W / 2 - 330 * s, H * 0.20, 660 * s, 74 * s + row * len(checks))
        p.setPen(QPen(RED if arming == 'refused' else AMBER, 2 * s))
        p.setBrush(QColor(6, 8, 12, 215))
        p.drawRect(panel)
        title = 'NOT ARMED   ·   THE ROBOT STAYS IN MANUAL' if arming == 'refused' else 'ARMING CHECK'
        self.text(p, QRectF(panel.left(), panel.top() + 10 * s, panel.width(), 34 * s), title, 22,
                  RED if arming == 'refused' else AMBER, Qt.AlignCenter)
        y = panel.top() + 56 * s
        for name, passed, found in checks:
            mark = QRectF(panel.left() + 26 * s, y + 6 * s, 16 * s, 16 * s)
            p.setPen(Qt.NoPen)
            p.setBrush(TEAL if passed else RED)
            p.drawRect(mark)
            self.text(p, QRectF(mark.right() + 14 * s, y, 180 * s, row), str(name).upper(), 13, WHITE, shadow=False)
            self.text(p, QRectF(mark.right() + 200 * s, y, panel.width() - 260 * s, row), str(found), 13,
                      WHITE if passed else AMBER, bold=False, shadow=False)
            y += row

    def paint_map(self, p, rect, s):
        p.setPen(QPen(QColor(255, 255, 255, 70), 1.5 * s))
        p.setBrush(QColor(0, 0, 0, 150))
        p.drawRect(rect)
        n = self.node
        if self.course is None or self.map_bounds is None:
            self.text(p, rect, 'NO MAP', 13, GREY, Qt.AlignCenter)
            return
        x0, x1, y0, y1 = self.map_bounds
        k = min(rect.width() / (y1 - y0), rect.height() / (x1 - x0)) * 0.94
        cx, cy = rect.center().x(), rect.center().y()
        xm, ym = (x0 + x1) / 2, (y0 + y1) / 2

        def to_map(x, y):       # course +X (south) points down the screen, +Y (east) to the right
            return QPointF(cx + (y - ym) * k, cy + (x - xm) * k)
        p.setPen(QPen(QColor(255, 255, 255, 210), 1.4 * s))
        for line in self.course.get('lane_lines', []):
            p.drawPolyline(QPolygonF([to_map(*pt) for pt in line]))
        p.setPen(Qt.NoPen)
        colors = {'orange': QColor(255, 140, 30), 'red': QColor(230, 60, 60), 'green': QColor(60, 190, 90),
                  'yellow': QColor(250, 220, 60), 'black': QColor(150, 150, 150)}
        for b in self.course.get('barrels', []):
            p.setBrush(colors.get(b.get('color'), QColor(255, 140, 30)))
            p.drawEllipse(to_map(*b['xy']), 1.8 * s, 1.8 * s)
        ramp = self.course.get('ramp')
        if ramp:
            hx, hy = ramp['length'] / 2, ramp['width'] / 2
            rx, ry = ramp['center']
            p.setBrush(QColor(190, 150, 100, 200))
            p.drawPolygon(QPolygonF([to_map(rx - hx, ry - hy), to_map(rx + hx, ry - hy), to_map(rx + hx, ry + hy), to_map(rx - hx, ry + hy)]))
        a, b = self.course['finish_line']
        p.setPen(QPen(AMBER, 3 * s))
        p.drawLine(to_map(*a), to_map(*b))
        for hole in self.course.get('potholes', []):          # where the map has potholes: thin rings
            p.setPen(QPen(QColor(255, 255, 255, 170), 1 * s))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(to_map(*hole['xy']), 3.0 * s, 3.0 * s)
        for hole in n.status.get('potholes_seen', []):       # where the robot has detected one on this run: filled
            p.setPen(Qt.NoPen)
            p.setBrush(WHITE)
            p.drawEllipse(to_map(hole[0], hole[1]), 2.4 * s, 2.4 * s)
        for cp in self.course.get('checkpoints', [])[n.status.get('checkpoint', 0):][:1]:
            p.setPen(QPen(AMBER, 1.5 * s, Qt.DotLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(to_map(cp['x'], cp['y']), cp.get('radius', 2.0) * k, cp.get('radius', 2.0) * k)
        # the lap manager's pose is in the course frame, which is lined up with the map; raw odometry drifts
        pose = (n.status['x'], n.status['y'], n.status.get('yaw', 0.0)) if 'x' in n.status and time.time() - n.status_time < 1.5 else None
        if pose is None and n.odom:
            pose = n.odom[:3]
        if pose is not None:
            c = to_map(pose[0], pose[1])
            dx, dy = math.sin(pose[2]), math.cos(pose[2])     # heading in screen coordinates
            r = 8 * s
            tri = QPolygonF([QPointF(c.x() + dx * r, c.y() + dy * r),
                             QPointF(c.x() - dx * r * 0.7 - dy * r * 0.6, c.y() - dy * r * 0.7 + dx * r * 0.6),
                             QPointF(c.x() - dx * r * 0.7 + dy * r * 0.6, c.y() - dy * r * 0.7 - dx * r * 0.6)])
            # the direction of the view, as a thin line from the robot
            look = pose[2] + self.azimuth
            p.setPen(QPen(QColor(255, 255, 255, 200), 1.2 * s))
            p.drawLine(c, QPointF(c.x() + math.sin(look) * 20 * s, c.y() + math.cos(look) * 20 * s))
            p.setPen(QPen(DARK, 1 * s))
            p.setBrush(TEAL if self.mode == 'manual' else RED)
            p.drawPolygon(tri)
        self.text(p, QRectF(rect.left() + 6 * s, rect.top() + 2 * s, rect.width(), 18 * s), 'COURSE MAP', 11, GREY, bold=False, shadow=False)

    def dim(self, p, W, H, alpha=170):
        p.fillRect(QRectF(0, 0, W, H), QColor(4, 6, 10, alpha))

    def paint_menu(self, p, W, H, s):
        self.dim(p, W, H)
        self.text(p, QRectF(0, H * 0.07, W, 84 * s), 'IMPRIMIS', 76, WHITE, Qt.AlignCenter)
        bar = QRectF(W / 2 - 215 * s, H * 0.07 + 88 * s, 430 * s, 6 * s)
        p.setPen(Qt.NoPen)
        p.setBrush(RED)
        p.drawPolygon(self.slanted(bar, 4 * s))
        self.text(p, QRectF(0, H * 0.07 + 100 * s, W, 30 * s), 'IGVC 2027 AUTONAV SIMULATOR   ·   SELECT DRIVE MODE', 16, GREY, Qt.AlignCenter)
        cw, ch, gap = 400 * s, 318 * s, 36 * s
        top = H * 0.07 + 150 * s
        self.mode_card(p, QRectF(W / 2 - cw - gap / 2, top, cw, ch), 'MANUAL',
                       ['You drive. W A S D to move,', 'mouse to look, Shift for speed.', 'A finished lap is saved as a lesson.'],
                       TEAL, 'manual', self.glyph_keys)
        self.mode_card(p, QRectF(W / 2 + gap / 2, top, cw, ch), 'AUTOMATIC',
                       ['The robot drives the best lap', 'on record, stops at the finish', 'line, and saves the result.'],
                       RED, 'automatic', self.glyph_route)
        bw, bh = 260 * s, 50 * s
        by = top + ch + 30 * s
        x = W / 2 - (3 * bw + 2 * 16 * s) / 2
        self.block_button(p, QRectF(x, by, bw, bh), 'New Lap', self.new_lap, enabled=bool(self.node.status))
        self.block_button(p, QRectF(x + bw + 16 * s, by, bw, bh), 'Controls', lambda: self.set_view('controls'))
        self.block_button(p, QRectF(x + 2 * (bw + 16 * s), by, bw, bh), 'Back to Driving', lambda: self.set_view('drive'))
        st = self.node.status
        route = st.get('route')
        if route:
            self.text(p, QRectF(0, by + bh + 16 * s, W, 24 * s), 'Route for the next automatic lap: ' + route, 13, GREY, Qt.AlignCenter, bold=False)

    def paint_controls(self, p, W, H, s):
        self.dim(p, W, H, 200)
        panel = QRectF(W / 2 - 360 * s, H / 2 - 310 * s, 720 * s, 620 * s)
        p.setPen(QPen(QColor(90, 96, 106), 2 * s))
        p.setBrush(PANEL)
        p.drawRect(panel)
        self.text(p, QRectF(panel.left(), panel.top() + 18 * s, panel.width(), 44 * s), 'CONTROLS', 34, WHITE, Qt.AlignCenter)
        rows = [('W', 'Drive forward'), ('S', 'Reverse'), ('A  /  D', 'Turn the rover left / right'),
                ('MOUSE', 'Look around: all the way round, and up or down'), ('ARROWS', 'Look around without a mouse'),
                ('C', 'Center the view'), ('SHIFT', 'Full speed (4.7 mph)'),
                ('SPACE', 'Brake'), ('M', 'Switch Manual / Automatic'), ('N', 'New lap'), ('ESC', 'Menu'),
                ('F11', 'Full screen')]
        y = panel.top() + 84 * s
        for key, what in rows:
            r = QRectF(panel.left() + 60 * s, y, 150 * s, 32 * s)
            p.setPen(QPen(TEAL, 2 * s))
            p.setBrush(QColor(0, 0, 0, 140))
            p.drawRect(r)
            self.text(p, r, key, 15, WHITE, Qt.AlignCenter, shadow=False)
            self.text(p, QRectF(r.right() + 28 * s, y, 420 * s, 32 * s), what, 16, WHITE, bold=False, shadow=False)
            y += 38 * s
        self.block_button(p, QRectF(panel.center().x() - 130 * s, panel.bottom() - 66 * s, 260 * s, 48 * s), 'Back',
                          lambda: self.set_view('menu'))

    def paint_finish(self, p, W, H, s):
        st = self.node.status
        self.dim(p, W, H, 150)
        band = QRectF(0, H * 0.24, W, 190 * s)
        g = QLinearGradient(band.topLeft(), band.topRight())
        g.setColorAt(0.0, QColor(218, 41, 42, 0))
        g.setColorAt(0.5, QColor(218, 41, 42, 235))
        g.setColorAt(1.0, QColor(218, 41, 42, 0))
        p.fillRect(band, QBrush(g))
        self.text(p, QRectF(0, band.top() + 14 * s, W, 84 * s), 'LAP COMPLETE', 70, WHITE, Qt.AlignCenter)
        last = st.get('last_result') or {}
        self.text(p, QRectF(0, band.top() + 100 * s, W, 60 * s), clock_text(last.get('lap_time_s', st.get('lap_time', 0.0))), 50,
                  WHITE, Qt.AlignCenter, mono=True)
        info = '%s   ·   %.0f m' % (str(last.get('mode', self.mode)).upper(), last.get('distance_m', st.get('distance', 0.0)))
        if last.get('average_speed_mph'):
            info += '   ·   average %.1f mph' % last['average_speed_mph']
        contacts, touches = last.get('barrel_contacts'), last.get('lane_line_touches')
        holes = last.get('pothole_touches') or 0
        if contacts is not None:
            if contacts == 0 and touches == 0 and holes == 0:
                info += '   ·   CLEAN: no barrel contact, no lane line or pothole touched'
            else:
                info += '   ·   barrel contacts %d   ·   lane line touches %d   ·   pothole touches %d' % (contacts, touches, holes)
            if 'odometry' in str(last.get('scored_with', '')):
                info += ' (by odometry)'
        self.text(p, QRectF(0, band.bottom() + 12 * s, W, 26 * s), info, 16, WHITE, Qt.AlignCenter)
        if st.get('new_best'):
            verdict, c = 'NEW BEST. This lap is now the route for the next automatic run.', AMBER
        else:
            best = st.get('best_time')
            verdict, c = 'Saved to the lap record. Best lap stays at %s.' % (clock_text(best) if best else '--'), GREY
        self.text(p, QRectF(0, band.bottom() + 42 * s, W, 26 * s), verdict, 16, c, Qt.AlignCenter)
        bw, bh = 300 * s, 52 * s
        y = band.bottom() + 92 * s
        x = W / 2 - (3 * bw + 2 * 16 * s) / 2
        self.block_button(p, QRectF(x, y, bw, bh), 'Run Again: Automatic', lambda: self.choose_mode('automatic'), accent=RED)
        self.block_button(p, QRectF(x + bw + 16 * s, y, bw, bh), 'Drive Again: Manual', lambda: self.choose_mode('manual'), accent=TEAL)
        self.block_button(p, QRectF(x + 2 * (bw + 16 * s), y, bw, bh), 'Menu', lambda: self.set_view('menu'))
        self.text(p, QRectF(0, y + bh + 14 * s, W, 22 * s), 'The robot is held at the finish line until you choose.', 13, GREY,
                  Qt.AlignCenter, bold=False)


def take_screenshots(window, node, folder):
    """Draw each screen with stand-in data and save it. Used to check the layout without a display."""
    os.makedirs(folder, exist_ok=True)
    base = {'state': 'running', 'lap_time': 133.4, 'distance': 81.0, 'checkpoint': 2, 'checkpoints': 6, 'best_time': 445.3,
            'runs': 3, 'goal': 11, 'goals': 38, 'nav_ready': True, 'lane': {'cells': 180, 'left_offset': 1.9, 'right_offset': 2.0},
            'route': '', 'x': 12.0, 'y': -17.6, 'yaw': -1.5, 'speed': 1.2}
    node.odom = (12.0, -17.6, -1.5, 1.2, -0.3)
    shots = [('1_menu', 'menu', 'manual', dict(base, state='ready', lap_time=0.0, distance=0.0, checkpoint=0), {}),
             ('2_manual', 'drive', 'manual', base, {'state': 'ramp_ahead', 'distance': 3.2, 'grade': 0.15}),
             ('3_automatic', 'drive', 'automatic', base, {'state': 'flat'}),
             ('4_finish', 'finish', 'automatic', dict(base, state='finished', lap_time=412.6, distance=158.2, checkpoint=6, new_best=True,
                                                     last_result={'result': 'success', 'lap_time_s': 412.6, 'distance_m': 158.2,
                                                                  'min_barrel_clearance_m': 0.31, 'mode': 'automatic', 'barrel_contacts': 0,
                                                                  'lane_line_touches': 0, 'clean': True,
                                                                  'scored_with': 'simulator true pose'}), {}),
             ('5_controls', 'controls', 'manual', base, {})]
    for name, screen, mode, status, ramp in shots:
        node.status, node.ramp, node.mode = status, ramp, mode
        window.view = screen
        window.keys = {Qt.Key_W} if name == '2_manual' else set()
        window.hover = QPointF(window.width() * 0.33, window.height() * 0.45) if name == '1_menu' else QPointF(-1, -1)
        window.finish_shown_for = status.get('runs')
        window.repaint()
        window.grab().save(os.path.join(folder, 'gui_%s.png' % name))


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    app = QApplication(sys.argv[:1])
    window = ControlWindow(node)
    if node.screenshot_dir:
        window.control_timer.stop()
        window.show()
        app.processEvents()
        take_screenshots(window, node, node.screenshot_dir)
        node.destroy_node()
        rclpy.shutdown()
        return
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_quietly():
        try:
            executor.spin()
        except Exception:      # ROS was shut down while this thread was waiting: nothing left to do
            pass

    spinner = threading.Thread(target=spin_quietly, daemon=True)
    spinner.start()
    window.show()
    try:
        app.exec_()
    finally:
        try:
            if node.mode == 'manual':
                node.drive(0.0, 0.0)
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
