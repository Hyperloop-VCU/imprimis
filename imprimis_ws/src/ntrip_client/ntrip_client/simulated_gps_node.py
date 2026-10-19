'''
This script will simulate a GPS by creating a GGA message
and publishing that to a ROS2 topic at a specificed rate.

The GPS will move in a circlular path in San Francisco.

'''


#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


# ==============================
# DEBUG CONFIGURATION
# ==============================
DEBUG = False

EARTH_RADIUS_M = 6371008.8
METERS_PER_MILE = 1609.344
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
        radius_miles=1.0,
        circle_period=300.0,
    ):
        super().__init__("simulated_gps")

        self.navsat_topic = navsat_topic
        self.gga_topic = gga_topic

        self.center_latitude = float(latitude)
        self.center_longitude = float(longitude)
        self.altitude = float(altitude)

        self.rate_hz = float(rate_hz)
        self.radius_m = float(radius_miles) * METERS_PER_MILE
        self.circle_period = float(circle_period)

        if self.rate_hz <= 0:
            raise ValueError("rate_hz must be greater than zero")

        if self.radius_m < 0:
            raise ValueError("radius_miles cannot be negative")

        if self.circle_period <= 0:
            raise ValueError("circle_period must be greater than zero")

        if not -90 <= self.center_latitude <= 90:
            raise ValueError("Latitude must be between -90 and 90")

        if not -180 <= self.center_longitude <= 180:
            raise ValueError("Longitude must be between -180 and 180")

        if abs(self.center_latitude) >= 89.9 and self.radius_m > 0:
            raise ValueError(
                "This circular approximation is unsuitable near the poles"
            )

        # Time reference for the simulated motion.
        self.start_time = time.monotonic()

        self.navsat_pub = self.create_publisher(
            NavSatFix,
            self.navsat_topic,
            10,
        )

        self.gga_pub = self.create_publisher(
            String,
            self.gga_topic,
            10,
        )

        period = 1.0 / self.rate_hz

        self.timer = self.create_timer(period, self.publish)

        self.get_logger().info(
            f"Simulated GPS moving in a circle: "
            f"radius={radius_miles} mile(s), "
            f"period={circle_period} seconds, "
            f"rate={rate_hz} Hz"
        )

    def get_position(self):
        """
        Return the current simulated latitude and longitude.

        The center coordinates define the center of the circle.
        The circle begins north of the center and moves clockwise.
        """

        elapsed = time.monotonic() - self.start_time

        # Angular position around the circle.
        theta = (
            2.0 * math.pi * elapsed / self.circle_period
        )

        # Angular radius of the circle, in radians.
        angular_radius = self.radius_m / EARTH_RADIUS_M

        center_lat_rad = math.radians(self.center_latitude)
        center_lon_rad = math.radians(self.center_longitude)

        # Local north/east offsets converted to angular coordinates.
        latitude_rad = (
            center_lat_rad
            + angular_radius * math.cos(theta)
        )

        longitude_rad = (
            center_lon_rad
            + angular_radius * math.sin(theta)
            / math.cos(center_lat_rad)
        )

        latitude = math.degrees(latitude_rad)
        longitude = math.degrees(longitude_rad)

        # Keep longitude within the conventional range.
        longitude = (longitude + 180.0) % 360.0 - 180.0

        return latitude, longitude

    @staticmethod
    def _nmea_checksum(sentence):
        checksum = 0

        for ch in sentence:
            checksum ^= ord(ch)

        return f"{checksum:02X}"

    @staticmethod
    def _decimal_to_nmea(value, is_latitude):
        """Convert decimal degrees to NMEA degrees and minutes."""

        direction_positive = "N" if is_latitude else "E"
        direction_negative = "S" if is_latitude else "W"

        direction = (
            direction_positive if value >= 0
            else direction_negative
        )

        absolute_value = abs(value)

        degrees = int(absolute_value)
        minutes = (absolute_value - degrees) * 60.0

        # Handle rounding at the 60-minute boundary.
        if minutes >= 59.9999995:
            degrees += 1
            minutes = 0.0

        degree_width = 2 if is_latitude else 3

        coordinate = (
            f"{degrees:0{degree_width}d}"
            f"{minutes:07.4f}"
        )

        return coordinate, direction

    def _build_gga(self, latitude, longitude):
        utc_time = time.strftime("%H%M%S", time.gmtime())

        lat_str, lat_dir = self._decimal_to_nmea(
            latitude, True
        )

        lon_str, lon_dir = self._decimal_to_nmea(
            longitude, False
        )

        sentence = (
            f"$GPGGA,{utc_time},"
            f"{lat_str},{lat_dir},"
            f"{lon_str},{lon_dir},"
            "4,12,0,79,49.5,M,-35.0,M,1.0,"
        )

#         sentence = (
        #     f"$GPGGA,{utc_time},"
        #     f"{lat_str},{lat_dir},"
        #     f"{lon_str},{lon_dir},"
        #     "1,09,0.9,0.0,M,0.0,M,,"
        # )

        checksum = self._nmea_checksum(sentence[1:])

        return f"{sentence}*{checksum}\r\n"

    def publish(self):
        # Calculate one position and use it for both messages.
        latitude, longitude = self.get_position()

        # Publish NavSatFix.
        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = self.altitude

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )

        self.navsat_pub.publish(msg)

        # Publish matching NMEA GGA.
        gga_msg = String()
        gga_msg.data = self._build_gga(latitude, longitude)

        self.gga_pub.publish(gga_msg)

        if DEBUG:
            self.get_logger().info(
                f"GPS: lat={latitude:.7f}, "
                f"lon={longitude:.7f}"
            )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Circular simulated GPS publisher"
    )

    parser.add_argument(
        "--navsat-topic",
        default="/gps/fix",
    )

    parser.add_argument(
        "--gga-topic",
        default="/gps/gga",
    )

    parser.add_argument(
        "--latitude",
        type=float,
        default=37.7749,
        help="Latitude at the center of the circle",
    )

    parser.add_argument(
        "--longitude",
        type=float,
        default=-122.4194,
        help="Longitude at the center of the circle",
    )

    parser.add_argument(
        "--altitude",
        type=float,
        default=0.0,
    )

    parser.add_argument(
        "--rate-hz",
        type=float,
        default=1.0,
        help="GPS publishing frequency in Hz",
    )

    parser.add_argument(
        "--radius-miles",
        type=float,
        default=1.0,
        help="Radius of the circular path in miles",
    )

    parser.add_argument(
        "--circle-period",
        type=float,
        default=300.0,
        help="Time in seconds to complete one circle",
    )

    parsed, _ = parser.parse_known_args(args)

    rclpy.init(args=None)

    node = None

    try:
        node = SimulatedGPSNode(
            navsat_topic=parsed.navsat_topic,
            gga_topic=parsed.gga_topic,
            latitude=parsed.latitude,
            longitude=parsed.longitude,
            altitude=parsed.altitude,
            rate_hz=parsed.rate_hz,
            radius_miles=parsed.radius_miles,
            circle_period=parsed.circle_period,
        )

        rclpy.spin(node)

    except ValueError as exc:
        print(f"Configuration error: {exc}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
