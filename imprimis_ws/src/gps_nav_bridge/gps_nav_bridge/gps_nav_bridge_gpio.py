import json
import math
import threading
import time
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import serial
from geographiclib.geodesic import Geodesic
from control_msgs.msg import DynamicInterfaceGroupValues


def yaw_to_quat(yaw_rad: float) -> Quaternion:
    """Create a Z-only quaternion from yaw (radians)."""
    q = Quaternion()
    q.w = math.cos(yaw_rad * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    return q


class GpsNavBridge(Node):
    """
    Input modes:
      - use_gpio_states:=True  -> read GPS fields from /gpio_controller/gpio_states
                                  and publish /gps/fix
      - use_serial:=True       -> read Arduino JSON lines over USB serial and publish /gps/fix
      - neither                -> do NOT publish /gps/fix; only handle /gps/goal -> Nav2 goals
                                 (expects /gps/fix from another node)

    Auto datum:
      - auto_set_datum:=True -> subscribe to /gps/fix and set datum_lat/datum_lon from first valid fix
      - auto_set_datum_if_unset:=True -> only set datum if datum was unset (0,0)
      - auto_set_datum_once:=True -> set only once 

    GPS goal handling:
      - Subscribe /gps/goal 
      - Convert lat/lon -> local ENU meters from datum -> rotate into map frame
      - Publish /gps/goal_pose for RViz
      - Send Nav2 NavigateToPose goal to /navigate_to_pose
    """

    def __init__(self):
        super().__init__("gps_nav_bridge_gpio")

        self.use_serial = bool(self.declare_parameter("use_serial", True).value)
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = int(self.declare_parameter("baud", 115200).value)

        self.use_gpio_states = bool(self.declare_parameter("use_gpio_states", False).value)
        self.gpio_states_topic = self.declare_parameter(
            "gpio_states_topic", "/gpio_controller/gpio_states"
        ).value

        self.datum_lat = float(self.declare_parameter("datum_lat", 0.0).value)
        self.datum_lon = float(self.declare_parameter("datum_lon", 0.0).value)
        self.datum_alt = float(self.declare_parameter("datum_alt", 0.0).value)

        self.auto_set_datum = bool(self.declare_parameter("auto_set_datum", True).value)
        self.auto_set_datum_if_unset = bool(self.declare_parameter("auto_set_datum_if_unset", True).value)
        self.auto_set_datum_once = bool(self.declare_parameter("auto_set_datum_once", True).value)

        self.map_yaw_offset_deg = float(self.declare_parameter("map_yaw_offset_deg", 0.0).value)
        self.map_yaw_offset = math.radians(self.map_yaw_offset_deg)

        self.gps_frame_id = self.declare_parameter("gps_frame_id", "gps_link").value
        self.map_frame = self.declare_parameter("map_frame", "map").value

        self.fix_topic = self.declare_parameter("fix_topic", "/gps/fix").value
        self.goal_topic = self.declare_parameter("goal_topic", "/gps/goal").value
        self.goal_pose_topic = self.declare_parameter("goal_pose_topic", "/gps/goal_pose").value

        self.nav2_action_name = self.declare_parameter("nav2_action_name", "/navigate_to_pose").value
        self.max_read_hz = float(self.declare_parameter("max_read_hz", 10.0).value)

        self.default_position_covariance_m2 = float(
            self.declare_parameter("default_position_covariance_m2", -1.0).value
        )

        self._datum_lock = threading.Lock()
        self._datum_initialized = not (self.datum_lat == 0.0 and self.datum_lon == 0.0)

        if not self._datum_initialized:
            self.get_logger().warn(
                "datum_lat/datum_lon are not set yet. "
                "Will auto-set from /gps/fix if auto_set_datum:=True."
            )

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)
        self.goal_sub = self.create_subscription(NavSatFix, self.goal_topic, self.on_goal_gps, 10)

        self.fix_sub = self.create_subscription(
            NavSatFix, self.fix_topic, self.on_fix_for_datum, qos_profile_sensor_data
        )

        self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        self._serial: Optional[serial.Serial] = None
        self._stop = False
        self._thread = threading.Thread(target=self.serial_worker, daemon=True)

        if self.use_gpio_states:
            self.get_logger().info(
                f"use_gpio_states:=True -> subscribing to {self.gpio_states_topic} "
                f"and publishing {self.fix_topic}"
            )
            self.gpio_sub = self.create_subscription(
                DynamicInterfaceGroupValues,
                self.gpio_states_topic,
                self.on_gpio_states,  # TODO minor issue where if we switch to modes or shut off motors, this callback will run
                qos_profile_sensor_data,
            )
        elif self.use_serial:
            self.open_serial()
            self._thread.start()
        else:
            self.get_logger().info(
                "No GPS input enabled (use_gpio_states:=False and use_serial:=False). "
                "Expecting /gps/fix from another node."
            )

        self.get_logger().info(
            f"gps_nav_bridge running. use_gpio_states={self.use_gpio_states}, use_serial={self.use_serial}. "
            f"fix_topic={self.fix_topic}, goal_topic={self.goal_topic}, goal_pose_topic={self.goal_pose_topic}. "
            f"Nav2 action={self.nav2_action_name}, map_frame={self.map_frame}. "
            f"auto_set_datum={self.auto_set_datum}, datum_initialized={self._datum_initialized}."
        )

        self._gpio_last_pub = 0.0

    def publish_fix_from_fields(self, fix: int, lat, lon, alt: float, hdop) -> None:
        """
        Shared helper: take data from serial or gpio and publish NavSatFix to fix_topic.
        """
        try:
            fix_i = int(fix)
        except Exception:
            fix_i = 0

        if fix_i == 0 or lat is None or lon is None:
            self.get_logger().info(f"Lost GPS signal. Will not publish to /gps/fix.", throttle_duration_sec=4.0)
            return

        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception:
            self.get_logger().warn(f"Invalid value in GPS latitude or longitude. Will not publish to /gps/fix.", throttle_duration_sec=0.5)
            return

        if abs(lat_f) < 1e-9 or abs(lon_f) < 1e-9:
            self.get_logger().warn(f"Zero value in latitude or longitude. Will not publish to /gps/fix.", throttle_duration_sec=0.5)
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gps_frame_id

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = lat_f
        msg.longitude = lon_f
        msg.altitude = float(alt)

        if self.default_position_covariance_m2 > 0.0:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            msg.position_covariance[0] = self.default_position_covariance_m2
            msg.position_covariance[4] = self.default_position_covariance_m2
            msg.position_covariance[8] = self.default_position_covariance_m2 * 2.0
        elif hdop is not None:
            try:
                hd = float(hdop)
                if hd > 0.01:
                    # rough estimate: sigma_h ~ hdop * 5m
                    sigma_h = hd * 5.0
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    msg.position_covariance[0] = sigma_h * sigma_h
                    msg.position_covariance[4] = sigma_h * sigma_h
                    msg.position_covariance[8] = (2.0 * sigma_h) * (2.0 * sigma_h)
                else:
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            except Exception:
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)

    def on_fix_for_datum(self, msg: NavSatFix) -> None:
        if not self.auto_set_datum:
            return

        if msg.status.status < NavSatStatus.STATUS_FIX:
            return

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        # reject (0,0) otherwise it desyncs and cries
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return

        with self._datum_lock:
            if self.auto_set_datum_if_unset and self._datum_initialized:
                return

            if self.auto_set_datum_once and self._datum_initialized:
                return

            self.datum_lat = lat
            self.datum_lon = lon
            self._datum_initialized = True

            try:
                self.set_parameters(
                    [
                        Parameter("datum_lat", Parameter.Type.DOUBLE, lat),
                        Parameter("datum_lon", Parameter.Type.DOUBLE, lon),
                    ]
                )
            except Exception:
                pass

        self.get_logger().info(f"Auto-set datum from {self.fix_topic}: datum_lat={lat:.7f}, datum_lon={lon:.7f}")

    def on_gpio_states(self, msg: DynamicInterfaceGroupValues) -> None:
        """
        Read GPS fields from /gpio_controller/gpio_states and publish NavSatFix 
        using the same logic as the serial path. (I'm not writing new logic)

        Expected names:
          gpsFix, gpsLat, gpsLong, gpsAlt, gpsHdop

        When fake hardware is used, these are 0, so we ignore it :)
        """
        now_wall = time.time()
        min_dt = 1.0 / max(1.0, self.max_read_hz)
        if (now_wall - self._gpio_last_pub) < min_dt:
            return

        if not msg.interface_values:
            return

        fix = 0
        lat = None
        lon = None
        alt = 0.0
        hdop = None

        for iv in msg.interface_values:
            try:
                names = list(iv.interface_names)
                vals = list(iv.values)
            except Exception:
                continue

            if len(names) != len(vals) or len(names) == 0:
                continue

            m = dict(zip(names, vals))

            if "gpsFix" in m:
                try:
                    fix = int(m["gpsFix"])
                except Exception: # if nan
                    fix = 0
            if "gpsLat" in m:
                lat = m["gpsLat"]
            if "gpsLong" in m:
                lon = m["gpsLong"]
            if "gpsAlt" in m:
                try:
                    alt = float(m["gpsAlt"])
                except Exception:
                    alt = 0.0
            if "gpsHdop" in m:
                hdop = m["gpsHdop"]


        # Will not publish if no fix
        self.publish_fix_from_fields(fix, lat, lon, alt, hdop)
        self._gpio_last_pub = now_wall

    def open_serial(self) -> None:
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
            time.sleep(1.5)
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial {self.port}: {e}")
            raise

    def serial_worker(self) -> None:
        """Read Arduino JSON lines and publish NavSatFix."""
        min_dt = 1.0 / max(1.0, self.max_read_hz)
        last_pub = 0.0

        while rclpy.ok() and not self._stop:
            if self._serial is None:
                time.sleep(0.1)
                continue

            try:
                line = self._serial.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.5)
                continue

            if not line:
                continue

            now_wall = time.time()
            if (now_wall - last_pub) < min_dt:
                continue

            try:
                d = json.loads(line)
            except Exception:
                continue

            if isinstance(d, dict) and d.get("status") == "gps_bridge_ready":
                self.get_logger().info("Arduino reports gps_bridge_ready")
                continue

            if not isinstance(d, dict):
                continue

            fix = d.get("fix", 0)
            lat = d.get("lat", None)
            lon = d.get("lon", None)
            alt = d.get("alt", 0.0)
            hdop = d.get("hdop", None)

            self.publish_fix_from_fields(fix, lat, lon, alt, hdop)
            last_pub = now_wall

    def latlon_to_local_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert (lat, lon) to local ENU meters relative to datum using WGS84 geodesic.
        Returns: (east_m, north_m)
        """
        with self._datum_lock:
            lat0 = self.datum_lat
            lon0 = self.datum_lon

        geod = Geodesic.WGS84.Inverse(lat0, lon0, lat, lon)
        s12 = geod["s12"]  # meters
        azi1 = geod["azi1"]  # degrees from north, clockwise

        az = math.radians(azi1)
        east = s12 * math.sin(az)
        north = s12 * math.cos(az)
        return east, north

    def enu_to_map(self, east: float, north: float) -> Tuple[float, float]:
        """
        Rotate ENU into map frame by map_yaw_offset.
        Note: As of version 2.2.1, all heading data
        is assumed to start with it's zero point
        facing east. Why? I have no clue (L Steve)
        """
        c = math.cos(self.map_yaw_offset)
        s = math.sin(self.map_yaw_offset)
        x_map = c * east - s * north
        y_map = s * east + c * north
        return x_map, y_map

    def on_goal_gps(self, msg: NavSatFix) -> None:
        """
        Receives target lat/lon (NavSatFix) on /gps/goal, converts to map (x,y),
        publishes /gps/goal_pose, and sends NavigateToPose goal to Nav2.
        """
        if not self._datum_initialized:
            self.get_logger().error(
                "Refusing goal: datum not initialized yet. Wait for /gps/fix or set datum_lat/datum_lon."
            )
            return

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        east, north = self.latlon_to_local_enu(lat, lon)
        x_map, y_map = self.enu_to_map(east, north)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.map_frame

        goal_pose.pose.position.x = float(x_map)
        goal_pose.pose.position.y = float(y_map)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = yaw_to_quat(0.0)  # position-only goal

        self.goal_pose_pub.publish(goal_pose)

        self.get_logger().info(
            f"GPS goal lat/lon=({lat:.7f},{lon:.7f}) -> map x/y=({x_map:.2f},{y_map:.2f})"
        )

        self.send_nav2_goal(goal_pose)

    def send_nav2_goal(self, pose: PoseStamped) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server not available: {self.nav2_action_name}")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected.")
            return

        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        try:
            result = future.result().result
            self.get_logger().info(f"Nav2 result: {result}")
        except Exception as e:
            self.get_logger().error(f"Failed to get Nav2 result: {e}")

    def destroy_node(self):
        self._stop = True
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GpsNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
