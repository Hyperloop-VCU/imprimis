#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#include <memory>
#include <chrono>

#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


// helper function taken from realsense2 examples.
// It opens the connection to the device on out_serial.
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
    rs2::context ctx;
    auto devs = ctx.query_devices();
    std::vector <rs2_stream> unavailable_streams = stream_requests;
    for (auto dev : devs)
    {
        std::map<rs2_stream, bool> found_streams;
        for (auto& type : stream_requests)
        {
            found_streams[type] = false;
            for (auto& sensor : dev.query_sensors())
            {
                for (auto& profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == type)
                    {
                        found_streams[type] = true;
                        unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
                        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                            out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    }
                }
            }
        }
        // Check if all streams are found in current device
        bool found_all_streams = true;
        for (auto& stream : found_streams)
        {
            if (!stream.second)
            {
                found_all_streams = false;
                break;
            }
        }
        if (found_all_streams)
            return true;
    }
    // After scanning all devices, not all requested streams were found
    for (auto& type : unavailable_streams)
    {
        switch (type)
        {
        case RS2_STREAM_POSE:
        case RS2_STREAM_FISHEYE:
            std::cerr << "Connect T26X and rerun the demo" << std::endl;
            break;
        case RS2_STREAM_DEPTH:
            std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;
            break;
        case RS2_STREAM_COLOR:
            std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;
            break;
        default:
            throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
        }
    }
    return false;
}

class t265Node : public rclcpp::Node
{

  // Private data members: publisher and camera connection stuff
  private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _publisher;
  std::string serial;
  rs2::pipeline pipe;
  rs2::config cfg;

  float covariance_diagonal;

  public:

  // Constructor for the node
  t265Node() : Node("t265_node"), covariance_diagonal(0.001)
  {
    RCLCPP_INFO(get_logger(), "Starting Intel Realsense t265 driver node.");

    // initialize camera connection
    if (!device_with_streams({ RS2_STREAM_POSE}, serial))
    {
      RCLCPP_FATAL(get_logger(), "Failed to connect to t265 camera.");
      rclcpp::shutdown();
    }
    if (!serial.empty()) cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    pipe.start(cfg);


    _publisher = create_publisher<nav_msgs::msg::Odometry>("/t265/odom", 10);
  }


  // Main function that runs constantly. Should be called once after creating the node.
  // This function will block in the thread that called it.
  void run()
  {
    while (rclcpp::ok())
    {
      // Get camera data (blocking)
      auto frames = pipe.wait_for_frames();
      auto f = frames.first_or_default(RS2_STREAM_POSE);
      auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

      // Create and stamp message
      auto msg = nav_msgs::msg::Odometry();
      msg.header.stamp = this->now();
      msg.header.frame_id = "odom";
      msg.child_frame_id = "lidar_pole";

      // set message data 
      // I have to do it like this, since rs2 defines custom vector types that don't implicitly cast
      msg.pose.pose.position.x = -pose_data.translation.z; // position (m)
      msg.pose.pose.position.y = -pose_data.translation.x;
      msg.pose.pose.position.z = pose_data.translation.y;
      msg.pose.pose.orientation.x = pose_data.rotation.z; // rotation (quat, r)
      msg.pose.pose.orientation.y = -pose_data.rotation.x;
      msg.pose.pose.orientation.z = pose_data.rotation.y;
      msg.pose.pose.orientation.w = pose_data.rotation.w;
      msg.twist.twist.linear.x = -pose_data.velocity.z; // velocity (m/s)
      msg.twist.twist.linear.y = -pose_data.velocity.x;
      msg.twist.twist.linear.z = pose_data.velocity.y;
      msg.twist.twist.angular.x = pose_data.angular_velocity.z; // angular velocity (r/s)
      msg.twist.twist.angular.y = pose_data.angular_velocity.x;
      msg.twist.twist.angular.z = pose_data.angular_velocity.y;

      for (int i = 0; i <= 35; i += 7)
      {
        msg.pose.covariance[i] = covariance_diagonal;
        msg.twist.covariance[i] = covariance_diagonal;
      }
    
      _publisher->publish(msg);
    }
    rclcpp::shutdown();
  }


};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<t265Node>();
  node->run(); // this node doesn't have timers, callbacks, services, or anything else that requires a call to rclcpp::spin.
  rclcpp::shutdown();
  return 0;
}
