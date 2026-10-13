#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


# ==============================
# DEBUG CONFIGURATION
# ==============================
DEBUG = False
# ==============================


class SimulatedGPSNode(Node):
    def __init__(
        self,
        navsat_topic="/gps/fix",
        gga_topic="/gps/gga",
        latitude=37.7749,
        longitude=-122.4194,
        altitude=0.0,
        rate_hz=1.0,
    ):
        super().__init__('simulated_gps')

        self.navsat_topic = navsat_topic
        self.gga_topic = gga_topic
        self.latitude = float(latitude)
        self.longitude = float(longitude)
        self.altitude = float(altitude)
        self.rate_hz = float(rate_hz)

        self.navsat_pub = self.create_publisher(
            NavSatFix,
            self.navsat_topic,
            10
        )

        self.gga_pub = self.create_publisher(
            String,
            self.gga_topic,
            10
        )

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 1.0
        self.timer = self.create_timer(period, self.publish)

    @staticmethod
    def _nmea_checksum(sentence):
        checksum = 0

        for ch in sentence:
            checksum ^= ord(ch)

        return f"{checksum:02X}"

    def _build_gga(self):
        utc_time = time.strftime("%H%M%S", time.gmtime())

        lat_abs = abs(self.latitude)
        lon_abs = abs(self.longitude)

        lat_deg = int(lat_abs)
        lat_min = (lat_abs - lat_deg) * 60.0

        lon_deg = int(lon_abs)
        lon_min = (lon_abs - lon_deg) * 60.0

        lat_dir = "N" if self.latitude >= 0 else "S"
        lon_dir = "E" if self.longitude >= 0 else "W"

        sentence = (
            f"$GPGGA,{utc_time},"
            f"{lat_deg:02d}{lat_min:07.4f},{lat_dir},"
            f"{lon_deg:03d}{lon_min:07.4f},{lon_dir},"
            "1,09,0.9,0.0,M,0.0,M,,"
        )

        checksum = self._nmea_checksum(sentence[1:])

        return f"{sentence}*{checksum}\r\n"

    def publish(self):
        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )

        self.navsat_pub.publish(msg)

        gga_msg = String()
        gga_msg.data = self._build_gga()

        self.gga_pub.publish(gga_msg)

        if DEBUG:
            self.get_logger().info(
                f"Published simulated GPS fix to "
                f"{self.navsat_topic} and GGA to {self.gga_topic}"
            )


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Simulated GPS publisher'
    )

    parser.add_argument(
        '--navsat-topic',
        default='/gps/fix'
    )

    parser.add_argument(
        '--gga-topic',
        default='/gps/gga'
    )

    parser.add_argument(
        '--latitude',
        type=float,
        default=37.7749
    )

    parser.add_argument(
        '--longitude',
        type=float,
        default=-122.4194
    )

    parser.add_argument(
        '--altitude',
        type=float,
        default=0.0
    )

    parser.add_argument(
        '--rate-hz',
        type=float,
        default=1.0
    )

    parsed, _ = parser.parse_known_args(args)

    rclpy.init(args=None)

    node = SimulatedGPSNode(
        navsat_topic=parsed.navsat_topic,
        gga_topic=parsed.gga_topic,
        latitude=parsed.latitude,
        longitude=parsed.longitude,
        altitude=parsed.altitude,
        rate_hz=parsed.rate_hz,
    )

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
