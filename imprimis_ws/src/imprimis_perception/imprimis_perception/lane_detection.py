import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointField, PointCloud2
import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')



        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('process_rate_hz',10.0)
        self.declare_parameter('camera_height', 0.675) #meters
        self.declare_parameter('camera_angle', 84.5) #0 = pointing straight down, 90 = looking out to the horizon
        self.declare_parameter('frame_id', 'front_link')

        self.theta = self.get_parameter('camera_angle').value # remember to use to calculate the distance in z and x ranges
        self.height = self.get_parameter('camera_height').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.process_period = 1.0 / self.get_parameter('process_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        


        self.bridge = CvBridge() #converts Ros message into OpenCv numpy matrix
        self.maskBridge = CvBridge()

        self.camera_matrix = None
        self.have_intrinsics = False
        self.maskPublish = True #should we publish the mask view, keep true for testing



        #this is for inverse mapping, find a spot on the ground and then find that point in the image and check whether or not it is a white pixel
        self.z_values = np.arange(0.5, 7.0, 0.05)   # 0.5m to 7m ahead, 5cm steps
        self.x_values = np.arange(-3.0, 3.0, 0.05)  # 3m left to 3m right, 5cm steps
        self.X, self.Z = np.meshgrid(self.x_values,self.z_values)




        self.last_process_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            camera_info_topic, 
            self.camera_info_cb, 
            1)

        self.image_sub = self.create_subscription( 
            Image, 
            image_topic,
            self.image_cb,
            1  # 1 is queue size, should stay at 1 to stay real time
        )


        self.camera_mask = self.create_publisher(
            Image,
            'camera/mask',
            1
        )


        self.lane_pointcloud = self.create_publisher(
            PointCloud2,
            'camera/lane_points',
            1
        )

        self.get_logger().info(
            f'Lane detector started. Waiting on {image_topic} and {camera_info_topic}'
        )




    def camera_info_cb(self, msg: CameraInfo):
        if self.have_intrinsics:
            return

        self.camera_matrix= np.array(msg.k).reshape(3,3)
        self.have_intrinsics = True
        self.get_logger().info(f'Cached Camera intrinsics:\n{self.camera_matrix}')

    def parameter_callback(self, params):
        for param in params:
            if (param.name == 'process_rate_hz'):
                if param.value <= 0:
                    return SetParametersResult(sucessful = False, reason='process_rate_hz was 0 or lower')
                self.process_period = 1.0/param.value
            
        return SetParametersResult(successful=True)

    def image_cb(self, msg: Image):
        if not self.have_intrinsics:
            return #need to have intrinsics before we can really do much

        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds / 1e9
        if elapsed < self.process_period:
            return
        self.last_process_time = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        mask = self.segment_lane_pixels(cv_image)


        lane_pixel_count = cv2.countNonZero(mask)
        if self.maskPublish == True:
            self.publishMask(mask)
        lane_points = self.raycast(mask,self.camera_matrix)
        self.publishPointCloud(lane_points)
        #self.get_logger().info(f'LanePoints: {lane_points}')
        self.get_logger().info(f'Lane pixels detected: {lane_pixel_count}', throttle_duration_sec=1.0)# for testing



    def publishMask(self, cv_image: np.ndarray):
        cv_image = self.maskBridge.cv2_to_imgmsg(cv_image,encoding='mono8')
        self.camera_mask.publish(cv_image)

    def segment_lane_pixels(self, cv_image:np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0) #0 controls the spread of the gaussian distribution, 0 is set to auto instead of choosing our own
        



        threshold_value = 200
        _, mask = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)
        
        return mask


    def raycast(self, cv_image: np.ndarray,camera_matrix):
        lane_points = []
        fx = camera_matrix[0][0] #focal length horizontal
        fy = camera_matrix[1][1] #focal length vertical
        cx = camera_matrix[0][2] #principle point horizontal
        cy = camera_matrix[1][2] #principle point vertical
        u = np.round(fx*(self.X/self.Z) + cx).astype(int)
        v = np.round(fy * (self.height/self.Z) + cy).astype(int)

        valid = (u >= 0) & (u < cv_image.shape[1]) & (v >= 0) & (v <cv_image.shape[0])
        u_valid = u[valid]
        v_valid = v[valid]
        x = self.X[valid]
        z = self.Z[valid]

        
        valid_lane_points = cv_image[v_valid,u_valid] == 255
        x_lane = x[valid_lane_points]
        z_lane = z[valid_lane_points]
        for i in range(len(x[valid_lane_points])):
            x_point = x_lane[i]
            z_point = z_lane[i]
            point = (x_point, self.height, z_point)
            lane_points.append(point)

        return lane_points

    def publishPointCloud(self, lane_points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()

        cloud_msg = point_cloud2.create_cloud(header, fields, lane_points)
        self.lane_pointcloud.publish(cloud_msg)        
        
def main(args=None):
    rclpy.init(args=args)
    node = LaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 

            

