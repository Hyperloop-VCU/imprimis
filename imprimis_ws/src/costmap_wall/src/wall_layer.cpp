#include "costmap_wall/wall_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap_wall
{

void WallLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("wall_relative_to", rclcpp::ParameterValue(std::string("base_link")));
  declareParameter("y_offset", rclcpp::ParameterValue(-1.0));
  declareParameter("wall_length", rclcpp::ParameterValue(4.0));
  declareParameter("cost_value", rclcpp::ParameterValue(254));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"costmap_wall::WallLayer: failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "wall_relative_to", wall_relative_to_);
  node->get_parameter(name_ + "." + "y_offset", y_offset_);
  node->get_parameter(name_ + "." + "wall_length", wall_length_);

  int cost_value = 254;
  node->get_parameter(name_ + "." + "cost_value", cost_value);
  if (cost_value < 0 || cost_value > 255) {
    RCLCPP_WARN(
      logger_,
      "costmap_wall::WallLayer (%s): cost_value %d is outside [0, 255], clamping.",
      name_.c_str(), cost_value);
    cost_value = std::clamp(cost_value, 0, 255);
  }
  cost_value_ = static_cast<unsigned char>(cost_value);

  if (wall_length_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "costmap_wall::WallLayer (%s): wall_length %.3f is not positive, no wall will be drawn.",
      name_.c_str(), wall_length_);
  }

  RCLCPP_INFO(
    logger_,
    "costmap_wall::WallLayer (%s): %.2f m wall at %.2f m along the +x axis of '%s', cost %d.",
    name_.c_str(), wall_length_, y_offset_, wall_relative_to_.c_str(), cost_value);

  current_ = true;
  checkCostmapBounds();
}

void WallLayer::checkCostmapBounds()
{
  if (bounds_checked_) {
    return;
  }

  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  const double size_x = costmap->getSizeInMetersX();
  const double size_y = costmap->getSizeInMetersY();

  // The costmap is not sized yet; try again on the first update.
  if (size_x <= 0.0 || size_y <= 0.0) {
    return;
  }
  bounds_checked_ = true;

  // Worst case is a wall endpoint, at this distance from the frame origin.
  const double reach = std::hypot(y_offset_, wall_length_ / 2.0);
  const double half_span = std::min(size_x, size_y) / 2.0;

  if (reach > half_span) {
    RCLCPP_WARN(
      logger_,
      "costmap_wall::WallLayer (%s): the wall reaches %.2f m from '%s', but the costmap is only "
      "%.2f x %.2f m, so part of the wall will fall outside it and be dropped. Enlarge the "
      "costmap (width/height) or reduce y_offset/wall_length.",
      name_.c_str(), reach, wall_relative_to_.c_str(), size_x, size_y);
  }
}

void WallLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  checkCostmapBounds();

  // Always re-expose last cycle's footprint so those cells get cleared, even if the
  // wall is disabled or the transform is missing this time around.
  if (has_prev_) {
    *min_x = std::min(*min_x, prev_min_x_);
    *min_y = std::min(*min_y, prev_min_y_);
    *max_x = std::max(*max_x, prev_max_x_);
    *max_y = std::max(*max_y, prev_max_y_);
    has_prev_ = false;
  }

  wall_valid_ = false;
  if (!enabled_ || wall_length_ <= 0.0) {
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(global_frame, wall_relative_to_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "costmap_wall::WallLayer (%s): no transform from '%s' to '%s': %s",
      name_.c_str(), wall_relative_to_.c_str(), global_frame.c_str(), ex.what());
    current_ = false;
    return;
  }

  const double ox = tf.transform.translation.x;
  const double oy = tf.transform.translation.y;
  const double yaw = tf2::getYaw(tf.transform.rotation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double half = wall_length_ / 2.0;

  // Endpoints in the wall frame are (y_offset, -half) and (y_offset, +half): the wall runs
  // along the frame's y axis and is pushed y_offset along its x (forward) axis.
  wall_x0_ = ox + c * y_offset_ - s * (-half);
  wall_y0_ = oy + s * y_offset_ + c * (-half);
  wall_x1_ = ox + c * y_offset_ - s * (half);
  wall_y1_ = oy + s * y_offset_ + c * (half);
  wall_valid_ = true;

  prev_min_x_ = std::min(wall_x0_, wall_x1_);
  prev_min_y_ = std::min(wall_y0_, wall_y1_);
  prev_max_x_ = std::max(wall_x0_, wall_x1_);
  prev_max_y_ = std::max(wall_y0_, wall_y1_);
  has_prev_ = true;

  *min_x = std::min(*min_x, prev_min_x_);
  *min_y = std::min(*min_y, prev_min_y_);
  *max_x = std::max(*max_x, prev_max_x_);
  *max_y = std::max(*max_y, prev_max_y_);

  current_ = true;
}

void WallLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !wall_valid_) {
    return;
  }

  // Step along the segment at half a cell so the stamped line has no gaps.
  const double resolution = master_grid.getResolution();
  const double length = std::hypot(wall_x1_ - wall_x0_, wall_y1_ - wall_y0_);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (resolution / 2.0))));

  for (int k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / steps;
    const double wx = wall_x0_ + t * (wall_x1_ - wall_x0_);
    const double wy = wall_y0_ + t * (wall_y1_ - wall_y0_);

    unsigned int mx, my;
    if (!master_grid.worldToMap(wx, wy, mx, my)) {
      continue;  // Outside the costmap; warned about at startup.
    }
    if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }

    const unsigned char old_cost = master_grid.getCost(mx, my);
    if (old_cost == nav2_costmap_2d::NO_INFORMATION || old_cost < cost_value_) {
      master_grid.setCost(mx, my, cost_value_);
    }
  }
}

void WallLayer::reset()
{
  wall_valid_ = false;
  current_ = false;
}

}  // namespace costmap_wall

PLUGINLIB_EXPORT_CLASS(costmap_wall::WallLayer, nav2_costmap_2d::Layer)
