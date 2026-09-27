#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace costmap_painter
{
class PaintedLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PaintedLayer() = default;
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override { resetMaps(); current_ = false; }
  bool isClearable() override { return true; }
  void matchSize() override { CostmapLayer::matchSize(); }

private:
  std::mutex data_mutex_;
  // your subscriber, cached data, params...
};
}  // namespace costmap_painter