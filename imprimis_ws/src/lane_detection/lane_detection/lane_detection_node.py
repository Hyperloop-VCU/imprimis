import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("fx", 615.0)
        self.declare_parameter("fy", 615.0)
        self.declare_parameter("min_area", 20)      
        self.declare_parameter("max_area", 8000)
        self.declare_parameter("max_depth_m", 3.0) 
        self.declare_parameter("no_roi", True)     
        
        
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        
        self.pc_pub = self.create_publisher(PointCloud2, '/perception/lane_pointcloud', 1)
        self.debug_pub = self.create_publisher(Image, '/perception/debug', 1)
        
    def sync_callback(self, rgb_msg, depth_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1") 
        except Exception as e:
            self.get_logger().error(f"CV Error: {e}")
            return

        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        min_area = self.get_parameter("min_area").value
        max_area = self.get_parameter("max_area").value
        max_depth = self.get_parameter("max_depth_m").value
        no_roi = self.get_parameter("no_roi").value

        h, w = frame.shape[:2]
        
        if no_roi:
            roi = frame
            offset_y = 0
        else:
            roi = frame[int(h/2):h, :] 
            offset_y = int(h/2)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 60, 60])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        kernel = np.ones((3, 3), np.uint8)
        bin_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[offset_y:offset_y + bin_mask.shape[0], :] = bin_mask

        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filter_bin_mask = np.zeros_like(full_mask)

        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area <= area <= max_area:
                cv2.drawContours(filter_bin_mask, [contour], 0, 255, cv2.FILLED)

        if self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(filter_bin_mask, "mono8")
            debug_msg.header = rgb_msg.header
            self.debug_pub.publish(debug_msg)

        sampled_mask = np.zeros_like(filter_bin_mask)
        sampled_mask[::4, ::4] = filter_bin_mask[::4, ::4]
        
        v_coords, u_coords = np.nonzero(sampled_mask)
        if len(v_coords) == 0: return

        Z_mm = depth_img[v_coords, u_coords]
        valid_depth = (Z_mm > 0) & (Z_mm < (max_depth * 1000.0))
        
        u_valid = u_coords[valid_depth]
        v_valid = v_coords[valid_depth]
        Z_clean_mm = Z_mm[valid_depth]

        if len(Z_clean_mm) == 0: return

        Z_clean = Z_clean_mm.astype(np.float32) / 1000.0
        X_clean = (u_valid - cx) * Z_clean / fx
        Y_clean = (v_valid - cy) * Z_clean / fy

        points = np.vstack((X_clean, Y_clean, Z_clean)).T
        self.publish_pointcloud(points, depth_msg.header)

    def publish_pointcloud(self, points, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        buffer = [struct.pack('fff', float(x), float(y), float(z)) for x, y, z in points]
        msg.data = b''.join(buffer)
        self.pc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()