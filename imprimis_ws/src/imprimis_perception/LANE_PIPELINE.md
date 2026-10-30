# Four-camera lane pipeline across Pis

Each camera computer runs one segmentation node next to its camera. The laptop runs one projector that subscribes to all four masks, the four `CameraInfo` topics, and the robot TF tree. Segmentation publishes small `mono8` masks instead of streaming full color video. The mask keeps the source image timestamp and optical `frame_id`; projection uses the matching camera intrinsics and timestamped camera-to-robot transform.

The default camera names match `utils/launch/multicam_test.launch.py`:

| Camera | Color image | CameraInfo | Mask sent to laptop |
| --- | --- | --- | --- |
| `d455` | `/cameras/d455/color/image_raw` | `/cameras/d455/color/camera_info` | `/perception/d455/lane_mask` |
| `d435i` | `/cameras/d435i/color/image_raw` | `/cameras/d435i/color/camera_info` | `/perception/d435i/lane_mask` |
| `d435_1` | `/cameras/d435_1/color/image_raw` | `/cameras/d435_1/color/camera_info` | `/perception/d435_1/lane_mask` |
| `d435_2` | `/cameras/d435_2/color/image_raw` | `/cameras/d435_2/color/camera_info` | `/perception/d435_2/lane_mask` |

## Run on each Pi

Build and source the workspace on every Pi. Use compatible ROS 2 distributions and RMW/DDS settings across the Pis and laptop. Start that Pi's camera driver so its color image and `CameraInfo` use the matching camera name from the table. Synchronize the Pi and laptop clocks (for example with chrony/NTP): masks keep their camera timestamps, TF is queried at those timestamps, and the projector drops samples older than `max_camera_age`. Set the same `ROS_DOMAIN_ID` on all Pis and the laptop, keep `ROS_LOCALHOST_ONLY=0`, and allow ROS 2 DDS discovery and traffic through the Ethernet interface/firewall.

Start one segmenter per attached camera. Example for the `d455` Pi (change the camera name and image topic on each machine):

```bash
ros2 launch imprimis_perception lane_segmentation.launch.py \
  camera_name:=d455 \
  image_topic:=/cameras/d455/color/image_raw \
  mask_topic:=/perception/d455/lane_mask \
  raw_mask_topic:=/perception/d455/lane_mask_raw \
  processing_width:=640
```

`processing_width:=640` caps the mask width before it leaves the Pi, reducing both Pi CPU load and Ethernet traffic; the projector rescales each camera's intrinsics to match the mask resolution. Use `processing_width:=0` to keep the camera's full resolution if distant markings need more detail and the network has enough capacity. At 640 pixels wide and 10 Hz, an uncompressed 640x480 mask is about 3.1 MB/s per camera before transport overhead.

If the laptop owns one of the cameras, run the same one-camera segmentation launch there too. The `camera_name` sets the node name; the image and mask topics must be explicitly unique for each camera. Do not start all four segmentation nodes on the laptop when the cameras are physically attached to separate Pis.

## Run on the laptop

Make sure its ROS graph can see all four masks and CameraInfo topics. The laptop must also publish one robot TF tree with calibrated transforms from `base_link` to every camera's optical frame; extend the URDF and run one `robot_state_publisher` for the full robot rather than creating identity transforms. Then run:

```bash
ros2 launch imprimis_perception lane_projection.launch.py
```

The projector combines fresh points from all four cameras and publishes `/perception/lane_points` in `base_link`. `output_z` defaults to `0.5` m for the current Nav2 lane observation height. Ground height is read from the `base_link` to `base_footprint` transform when available; otherwise the `ground_z` fallback is `-0.1277` m. To exercise the whole pipeline on one computer with all four camera topics already present, use `lane_detection.launch.py`; it starts four segmenters and one projector locally, but does not start the camera drivers.

## Network and TF checks

```bash
ros2 topic list | grep perception
ros2 topic hz /perception/d455/lane_mask
ros2 topic echo --once /perception/lane_points
ros2 run tf2_ros tf2_echo base_link <camera_optical_frame>
```

All computers need the same ROS domain and a DDS configuration that discovers peers across the Ethernet link. Each camera's `CameraInfo` and TF frame IDs must match its mask header. The projector reports and skips a camera if its calibration, frame match, or timestamped transform is missing; it never invents camera positions. The current robot URDF only describes the `front` camera, so measured URDF/TF transforms for the other three cameras are still needed before all four can contribute. The Pi hardware also needs ROS 2, OpenCV, NumPy, and `cv_bridge` installed for the selected ROS distribution.
