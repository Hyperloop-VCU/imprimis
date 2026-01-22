import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

EARTH_R = 6378137.0  # meters (WGS84)

class SimGpsFromTF(Node):
    def __init__(self):
        super().__init__('sim_gps_from_tf')

        self.declare_parameter('datum_lat', 37.5407000)
        self.declare_parameter('datum_lon', -77.4360000)
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('hz', 10.0)

        self.datum_lat = float(self.get_parameter('datum_lat').value)
        self.datum_lon = float(self.get_parameter('datum_lon').value)
        self.fix_topic = self.get_parameter('fix_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.hz = float(self.get_parameter('hz').value)

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        period = 1.0 / max(1.0, self.hz)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"Sim GPS from TF: {self.map_frame}->{self.base_frame}, publishing {self.fix_topic} @ {self.hz}Hz "
            f"datum=({self.datum_lat},{self.datum_lon})"
        )

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        x_map = t.transform.translation.x
        y_map = t.transform.translation.y

        # Treat map +x as East, +y as North (fits many maps; if yours is rotated, use yaw offset in bridge)
        east_m = x_map
        north_m = y_map

        lat0 = math.radians(self.datum_lat)
        dlat = north_m / EARTH_R
        dlon = east_m / (EARTH_R * math.cos(lat0))

        lat = self.datum_lat + math.degrees(dlat)
        lon = self.datum_lon + math.degrees(dlon)

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "gps_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = 0.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(fix)

def main():
    rclpy.init()
    node = SimGpsFromTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
