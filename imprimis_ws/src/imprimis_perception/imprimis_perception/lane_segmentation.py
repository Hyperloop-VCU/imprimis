
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult


def segment_lane_pixels(cv_image: np.ndarray, lower_white, upper_white, kernel: np.ndarray,
                         min_area: float, max_area: float):
    """Threshold white lane paint out of a BGR frame.

    Returns (mask, raw_mask): raw_mask is the result right after the morphological open
    (kept around for debugging), mask is the final result after filtering contours by area.
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    val_enhanced = clahe.apply(val)
    blurred = cv2.GaussianBlur(val_enhanced, (5, 5), 0) #0 controls the spread of the gaussian distribution, 0 is set to auto instead of choosing our own
    hsv_enhanced = cv2.merge([hue, sat, blurred])

    raw_mask = cv2.inRange(hsv_enhanced, lower_white, upper_white)
    raw_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)

    mask = np.zeros_like(raw_mask)
    contours, _ = cv2.findContours(
        raw_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    for contour in contours:
        if min_area <= cv2.contourArea(contour) <= max_area:
            cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)

    return mask, raw_mask


class LaneSegmentation(Node):
    def __init__(self):
        super().__init__('lane_segmentation')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('process_rate_hz', 10.0)
        self.declare_parameter('lower_white', (0, 0, 200))
        self.declare_parameter('upper_white', (179, 60, 255))
        self.declare_parameter('min_area', 400.0) #below this is mostly ceiling-light speckle, measured from real test footage
        self.declare_parameter('max_area', 60000.0)
        self.declare_parameter('publish_raw_mask', True) #should we publish the raw mask view, keep true for testing

        image_topic = self.get_parameter('image_topic').value
        self.process_period = 1.0 / self.get_parameter('process_rate_hz').value
        self.lower_white = self.get_parameter('lower_white').value
        self.upper_white = self.get_parameter('upper_white').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.publish_raw_mask = self.get_parameter('publish_raw_mask').value
        self.kernel = np.ones((3, 3), np.uint8)

        self.bridge = CvBridge() #converts Ros message into OpenCv numpy matrix

        self.last_process_time = self.get_clock().now()
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_cb,
            1  # 1 is queue size, should stay at 1 to stay real time
        )

        self.mask_pub = self.create_publisher(
            CompressedImage,
            'perception/mask/compressed', #image_transport naming convention, so rviz/rqt_image_view can find it
            1
        )

        self.raw_mask_pub = self.create_publisher(
            CompressedImage,
            'perception/mask_raw/compressed',
            1
        )

        self.get_logger().info(
            f'Lane segmentation started. Waiting on {image_topic}'
        )

    def parameter_callback(self, params):
        for param in params:
            if (param.name == 'process_rate_hz'):
                if param.value <= 0:
                    return SetParametersResult(successful=False, reason='process_rate_hz was 0 or lower')
                self.process_period = 1.0 / param.value

            if (param.name == 'lower_white'):
                self.lower_white = param.value

            if (param.name == 'upper_white'):
                self.upper_white = param.value

            if param.name == 'min_area':
                self.min_area = param.value

            if param.name == 'max_area':
                self.max_area = param.value

            if param.name == 'publish_raw_mask':
                self.publish_raw_mask = param.value

        return SetParametersResult(successful=True)

    def image_cb(self, msg: Image):
        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds / 1e9
        if elapsed < self.process_period:
            return
        self.last_process_time = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        mask, raw_mask = segment_lane_pixels(
            cv_image, self.lower_white, self.upper_white, self.kernel, self.min_area, self.max_area)

        mask_msg = self.bridge.cv2_to_compressed_imgmsg(mask, dst_format='png')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        if self.publish_raw_mask:
            raw_mask_msg = self.bridge.cv2_to_compressed_imgmsg(raw_mask, dst_format='png')
            raw_mask_msg.header = msg.header
            self.raw_mask_pub.publish(raw_mask_msg)

        lane_pixel_count = cv2.countNonZero(mask)
        self.get_logger().info(f'Lane pixels detected: {lane_pixel_count}', throttle_duration_sec=1.0)# for testing


def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmentation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
