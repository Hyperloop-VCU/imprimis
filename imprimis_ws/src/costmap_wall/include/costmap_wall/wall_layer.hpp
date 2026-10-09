#ifndef COSTMAP_WALL__WALL_LAYER_HPP_
#define COSTMAP_WALL__WALL_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace costmap_wall
{

/**
 * @class WallLayer
 * @brief Stamps a straight line of fixed cost at a fixed pose relative to a robot frame.
 *
 * The line is perpendicular to the frame's forward (+x) axis and sits y_offset metres
 * along it, so a negative y_offset puts the wall behind the robot. It is re-projected
 * into the costmap's global frame on every update, so it translates and rotates with
 * the frame.
 */
class WallLayer : public nav2_costmap_2d::Layer
{
public:
  WallLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  // The wall is generated, not sensed, so clear_costmap services must not wipe it.
  bool isClearable() override {return false;}

private:
  /** @brief Warn once if the wall cannot fit inside the costmap. */
  void checkCostmapBounds();

  std::string wall_relative_to_;
  double y_offset_{-1.0};
  double wall_length_{4.0};
  unsigned char cost_value_{nav2_costmap_2d::LETHAL_OBSTACLE};

  // Wall endpoints in the costmap's global frame, refreshed by updateBounds().
  double wall_x0_{0.0}, wall_y0_{0.0}, wall_x1_{0.0}, wall_y1_{0.0};
  bool wall_valid_{false};

  // Bounding box reported last cycle. LayeredCostmap only clears the region the layers
  // ask for, so the previous wall has to stay inside the bounds or its cells linger
  // in the master grid after the robot moves.
  double prev_min_x_{0.0}, prev_min_y_{0.0}, prev_max_x_{0.0}, prev_max_y_{0.0};
  bool has_prev_{false};

  bool bounds_checked_{false};
};

}  // namespace costmap_wall

#endif  // COSTMAP_WALL__WALL_LAYER_HPP_
