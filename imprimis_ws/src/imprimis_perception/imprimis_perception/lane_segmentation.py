
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult


def segment_lane_pixels(cv_image: np.ndarray, lower_white, upper_white, kernel: np.ndarray,
                        min_area: float, max_area: float):
    
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


def shrink_to_width(cv_image: np.ndarray, max_width: int):
    """The image resized to at most max_width pixels wide (0 = leave it), and the scale that was applied."""
    width = cv_image.shape[1]
    if max_width <= 0 or width <= max_width:
        return cv_image, 1.0
    scale = max_width / float(width)
    height = int(round(cv_image.shape[0] * scale))
    return cv2.resize(cv_image, (max_width, height), interpolation=cv2.INTER_AREA), scale


class LaneSegmentation(Node):
    def __init__(self):
        super().__init__('lane_segmentation')

        self.declare_parameter('image_topic', '/cameras/d435i/color/image_raw')
        # base name; the PNG mask goes out on <mask_topic>/compressed (image_transport naming, so rviz/rqt_image_view find it)
        self.declare_parameter('mask_topic', '/perception/d435i/lane_mask')
        self.declare_parameter('raw_mask_topic', '/perception/d435i/lane_mask_raw')
        self.declare_parameter('process_rate_hz', 10.0)
        self.declare_parameter('processing_width', 640) #shrink wider images to this before segmenting, saves Pi CPU and network; 0 keeps native size
        self.declare_parameter('lower_white', (0, 0, 200))
        self.declare_parameter('upper_white', (179, 60, 255))
        self.declare_parameter('min_area', 400.0) #below this is mostly ceiling-light speckle, measured from real test footage
        self.declare_parameter('max_area', 60000.0) #min_area and max_area are in pixels of the camera's native image, and are scaled when it is shrunk
        self.declare_parameter('publish_raw_mask', False) #publish the mask before the area filter, turn on for testing

        image_topic = self.get_parameter('image_topic').value
        mask_topic = self.get_parameter('mask_topic').value
        raw_mask_topic = self.get_parameter('raw_mask_topic').value
        self.process_period = 1.0 / self.get_parameter('process_rate_hz').value
        self.processing_width = self.get_parameter('processing_width').value
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
            mask_topic + '/compressed',
            1
        )

        self.raw_mask_pub = self.create_publisher(
            CompressedImage,
            raw_mask_topic + '/compressed',
            1
        )

        self.get_logger().info(
            f'Lane segmentation started. Waiting on {image_topic}, publishing {mask_topic}/compressed'
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

            if param.name == 'processing_width':
                if param.value < 0:
                    return SetParametersResult(successful=False, reason='processing_width was below 0')
                self.processing_width = param.value

        return SetParametersResult(successful=True)

    def image_cb(self, msg: Image):
        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds / 1e9
        if elapsed < self.process_period:
            return
        self.last_process_time = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image, scale = shrink_to_width(cv_image, self.processing_width)

        # the projector rescales the camera intrinsics to whatever size the mask arrives at
        area_scale = scale * scale
        mask, raw_mask = segment_lane_pixels(
            cv_image, self.lower_white, self.upper_white, self.kernel,
            self.min_area * area_scale, self.max_area * area_scale)

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