#ifndef LANE_LAYER_HPP_
#define LANE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <string>

using namespace sensor_msgs::msg;

namespace lanes_costmap_plugin
{

class LaneLayer : public nav2_costmap_2d::Layer
{
    public:
    LaneLayer();
    void onInitialize();
    void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double* min_y, double* max_x, double* max_y);
    void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
    void reset() {return;}
    void onFootprintChanged();
    bool isClearable() {return false;}
    void pointcloudCb(const PointCloud2::UniquePtr msg);
    void imageCb(const Image::UniquePtr msg);

    private:
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    // Indicates that the entire gradient should be recalculated next time.
    bool need_recalculation_;

    // Topic names
    std::string pointcloud_topic;
    std::string image_topic;

    // Subscriptions
    rclcpp::Subscription<PointCloud2>::SharedPtr pointcloudSub;
    rclcpp::Subscription<Image>::SharedPtr imageSub;

    // Size of gradient in cells
    int GRADIENT_SIZE = 20;
    // Step of increasing cost per one cell in gradient
    int GRADIENT_FACTOR = 10;


};

}  // namespace lanes_costmap_plugin

#endif  // LANE_LAYER_HPP_
