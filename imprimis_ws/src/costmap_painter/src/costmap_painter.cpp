#include "pluginlib/class_list_macros.hpp"
#include "costmap_painter/costmap_painter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace costmap_painter
{

void PaintedLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) throw std::runtime_error("Failed to lock node");

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  matchSize();          // allocate this layer's grid to match the master
  current_ = true;
  // create subscriptions here (node->create_subscription<...>)
}

void PaintedLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 1) Mark/clear cells in this layer's own grid from your latest data:
  //    unsigned int mx, my;
  //    if (worldToMap(wx, wy, mx, my)) setCost(mx, my, LETHAL_OBSTACLE);

  //for (int i = -5; i < 5; i++)
  //  setCost(0, i, LETHAL_OBSTACLE);

  // 2) Expand the bounds to cover every area you touched
  *min_x = std::min(*min_x, robot_x - 5.0);
  *min_y = std::min(*min_y, robot_y - 5.0);
  *max_x = std::max(*max_x, robot_x + 5.0);
  *max_y = std::max(*max_y, robot_y + 5.0);
}

void PaintedLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace my_costmap_plugin

PLUGINLIB_EXPORT_CLASS(costmap_painter::PaintedLayer, nav2_costmap_2d::Layer)