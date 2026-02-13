import json
import math
import threading
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial

class ArduinoGPSDriver(Node):
    """
    Reads GPS JSON Data written from an Arduino and publishes it as a NavSatFix message.
    """

    def __init__(self):
        super().__init__("arduino_gps_driver")

        self.use_serial = bool(self.declare_parameter("use_serial", True).value)

        # ROS params
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = int(self.declare_parameter("baud", 115200).value)
        self.gps_frame_id = self.declare_parameter("gps_frame_id", "gps_link").value
        self.fix_topic = self.declare_parameter("fix_topic", "/gps/fix").value
        self.max_read_hz = float(self.declare_parameter("max_read_hz", 10.0).value)
        self.default_position_covariance_m2 = float(
            self.declare_parameter("default_position_covariance_m2", -1.0).value
        )

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)

        self._serial: Optional[serial.Serial] = None
        self._stop = False
        self._thread = threading.Thread(target=self.serial_worker, daemon=True)

        try:
            self.open_serial()
        except Exception as e:
            self.destroy_node()
            return
        
        self._thread.start()


    def open_serial(self) -> None:
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
            time.sleep(1.5)
            self.get_logger().info(f"Arduino GPS driver successfuly opened serial on port {self.port} @ {self.baud} baud")
        except Exception as e:
            self.get_logger().fatal(f"Arduino GPS driver failed to open serial on port {self.port}: {e}")
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
                self.get_logger().error(f"Arduino serial read error: {e}")
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

            fix = int(d.get("fix", 0))
            lat = d.get("lat", None)
            lon = d.get("lon", None)
            alt = float(d.get("alt", 0.0))
            hdop = d.get("hdop", None)

            if fix == 0 or lat is None or lon is None:
                continue

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.gps_frame_id

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            msg.latitude = float(lat)
            msg.longitude = float(lon)
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
            last_pub = now_wall

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
    node = ArduinoGPSDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
