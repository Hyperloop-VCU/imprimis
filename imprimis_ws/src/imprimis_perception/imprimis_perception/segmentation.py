"""White lane-mark segmentation for one camera.

The node deliberately does no geometry. It publishes masks with the source image's
timestamp and optical frame so a downstream projector can combine several cameras.
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LaneSegmentation(Node):
    def __init__(self):
        super().__init__('lane_segmentation')
        self.declare_parameter('image_topic', 'cameras/front/color/image_raw')
        self.declare_parameter('mask_topic', 'perception/front/lane_mask')
        self.declare_parameter('raw_mask_topic', 'perception/front/lane_mask_raw')
        self.declare_parameter('process_rate_hz', 10.0)
        self.declare_parameter('lower_white', [0, 0, 200])
        self.declare_parameter('upper_white', [179, 60, 255])
        self.declare_parameter('min_area', 50.0)
        self.declare_parameter('max_area', 60000.0)
        self.declare_parameter('publish_raw_mask', False)
        self.declare_parameter('processing_width', 640)

        self.image_topic = self.get_parameter('image_topic').value
        self.process_period = self._period(self.get_parameter('process_rate_hz').value)
        self.lower_white = self._threshold(self.get_parameter('lower_white').value, 'lower_white')
        self.upper_white = self._threshold(self.get_parameter('upper_white').value, 'upper_white')
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area = float(self.get_parameter('max_area').value)
        if (not math.isfinite(self.min_area) or not math.isfinite(self.max_area)
                or self.min_area < 0 or self.max_area < self.min_area):
            raise ValueError('area thresholds must satisfy 0 <= min_area <= max_area')
        self.publish_raw = bool(self.get_parameter('publish_raw_mask').value)
        self.processing_width = int(self.get_parameter('processing_width').value)
        if self.processing_width < 0:
            raise ValueError('processing_width must be zero (native size) or positive')
        self.kernel = np.ones((3, 3), np.uint8)
        self.bridge = CvBridge()
        self.last_process_time = None
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(
            Image, self.get_parameter('mask_topic').value, qos_profile_sensor_data)
        self.raw_mask_pub = self.create_publisher(
            Image, self.get_parameter('raw_mask_topic').value, qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(
            f'Segmentation listening on {self.image_topic}; publishing mask to '
            f'{self.get_parameter("mask_topic").value}')

    @staticmethod
    def _period(rate):
        if not math.isfinite(float(rate)) or float(rate) <= 0:
            raise ValueError('process_rate_hz must be greater than zero')
        return 1.0 / float(rate)

    @staticmethod
    def _threshold(value, name):
        values = [int(v) for v in value]
        limits = (179, 255, 255)
        if len(values) != 3 or any(v < 0 or v > limit for v, limit in zip(values, limits)):
            raise ValueError(f'{name} must contain HSV values in ranges H=0..179, S/V=0..255')
        return np.asarray(values, dtype=np.uint8)

    def parameter_callback(self, params):
        try:
            proposed = {
                'process_period': self.process_period,
                'lower_white': self.lower_white,
                'upper_white': self.upper_white,
                'min_area': self.min_area,
                'max_area': self.max_area,
                'publish_raw': self.publish_raw,
                'processing_width': self.processing_width,
            }
            for param in params:
                if param.name == 'process_rate_hz':
                    proposed['process_period'] = self._period(param.value)
                elif param.name in ('lower_white', 'upper_white'):
                    proposed[param.name] = self._threshold(param.value, param.name)
                elif param.name in ('min_area', 'max_area'):
                    proposed[param.name] = float(param.value)
                elif param.name == 'publish_raw_mask':
                    proposed['publish_raw'] = bool(param.value)
                elif param.name == 'processing_width':
                    proposed['processing_width'] = int(param.value)
            if (not math.isfinite(proposed['min_area']) or not math.isfinite(proposed['max_area'])
                    or proposed['min_area'] < 0 or proposed['max_area'] < proposed['min_area']):
                raise ValueError('area thresholds must satisfy 0 <= min_area <= max_area')
            if np.any(proposed['lower_white'] > proposed['upper_white']):
                raise ValueError('lower_white values must not exceed upper_white values')
            if proposed['processing_width'] < 0:
                raise ValueError('processing_width must be zero or positive')
            self.process_period = proposed['process_period']
            self.lower_white = proposed['lower_white']
            self.upper_white = proposed['upper_white']
            self.min_area = proposed['min_area']
            self.max_area = proposed['max_area']
            self.publish_raw = proposed['publish_raw']
            self.processing_width = proposed['processing_width']
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def image_cb(self, msg):
        now = self.get_clock().now()
        if self.last_process_time is not None:
            elapsed = (now - self.last_process_time).nanoseconds / 1e9
            if elapsed < self.process_period:
                return
        self.last_process_time = now
        if msg.encoding not in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
            self.get_logger().warning(
                f'Unsupported image encoding {msg.encoding}; expected BGR/RGB color.',
                throttle_duration_sec=5.0)
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, ValueError, TypeError) as exc:
            self.get_logger().error(f'Could not decode {self.image_topic}: {exc}')
            return

        if self.processing_width and bgr.shape[1] > self.processing_width:
            height = max(1, round(bgr.shape[0] * self.processing_width / bgr.shape[1]))
            bgr = cv2.resize(
                bgr, (self.processing_width, height), interpolation=cv2.INTER_AREA)

        raw, mask = self.segment_lane_pixels(bgr)
        if self.publish_raw and self.raw_mask_pub.get_subscription_count() > 0:
            self.raw_mask_pub.publish(self._mask_message(raw, msg))
        self.mask_pub.publish(self._mask_message(mask, msg))

    @staticmethod
    def _mask_message(mask, source):
        out = Image()
        out.header = source.header
        out.height, out.width = mask.shape
        out.encoding = 'mono8'
        out.is_bigendian = False
        out.step = mask.shape[1]
        out.data = mask.tobytes()
        return out

    def segment_lane_pixels(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv)
        enhanced_value = cv2.createCLAHE(
            clipLimit=2.0, tileGridSize=(8, 8)).apply(value)
        blurred_value = cv2.GaussianBlur(enhanced_value, (5, 5), 0)
        enhanced_hsv = cv2.merge((hue, saturation, blurred_value))
        raw = cv2.inRange(enhanced_hsv, self.lower_white, self.upper_white)
        raw = cv2.morphologyEx(raw, cv2.MORPH_OPEN, self.kernel)
        filtered = np.zeros_like(raw)
        contours, _ = cv2.findContours(
            raw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area <= area <= self.max_area:
                cv2.drawContours(filtered, [contour], -1, 255, cv2.FILLED)
        return raw, filtered


def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
