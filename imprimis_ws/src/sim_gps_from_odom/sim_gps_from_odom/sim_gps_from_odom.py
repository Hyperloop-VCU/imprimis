import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.qos import qos_profile_sensor_data


# Simple local tangent plane approximation:
# lat = lat0 + (north_m / R)
# lon = lon0 + (east_m / (R*cos(lat0)))
# Good for small areas (your sim world is tiny).
EARTH_R = 6378137.0  # meters (WGS84)

class SimGpsFromOdom(Node):
    def __init__(self):
        super().__init__('sim_gps_from_odom')

        self.declare_parameter('datum_lat', 37.5407000)
        self.declare_parameter('datum_lon', -77.4360000)
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('odom_topic', '/odometry/filtered/local')
        self.declare_parameter('hz', 10.0)
        

        self.datum_lat = float(self.get_parameter('datum_lat').value)
        self.datum_lon = float(self.get_parameter('datum_lon').value)
        self.fix_topic = self.get_parameter('fix_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.hz = float(self.get_parameter('hz').value)

        self.latest_xy = None

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)


        period = 1.0 / max(1.0, self.hz)
        self.timer = self.create_timer(period, self.publish_fix)

        self.get_logger().info(
            f"Sim GPS publishing {self.fix_topic} from {self.odom_topic} at {self.hz} Hz, "
            f"datum=({self.datum_lat},{self.datum_lon})"
        )

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # ROS base convention in TB3 sim: x forward, y left in odom frame,
        # but for a flat world we just treat:
        # East = x, North = y (it’s arbitrary as long as consistent).
        self.latest_xy = (x, y)

    def publish_fix(self):
        if self.latest_xy is None:
            return

        east_m, north_m = self.latest_xy

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
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 0.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(fix)

def main():
    rclpy.init()
    node = SimGpsFromOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

