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
 * @brief Drops a straight line of fixed cost behind the robot once, at startup, and leaves
 * it there forever.
 *
 * On the first cycle a transform from wall_relative_to to map_frame is available, the layer
 * computes the wall's endpoints in map_frame and locks them in: wall_relative_to is never
 * consulted again, and the robot driving away does not move the wall. Because the costmap's
 * own global frame (e.g. a rolling local costmap's "odom") can drift relative to map_frame,
 * the locked endpoints are re-projected into the costmap's global frame on every cycle so the
 * wall still lines up with the map after a localization correction.
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

  /** @brief Try to lock the wall's pose into map_frame_. No-op once locked_ is true. */
  void tryLockWall();

  std::string wall_relative_to_;
  std::string map_frame_;
  double y_offset_{-1.0};
  double wall_length_{4.0};
  unsigned char cost_value_{nav2_costmap_2d::LETHAL_OBSTACLE};

  // The wall's endpoints in map_frame_, set once by tryLockWall() and never touched again.
  bool locked_{false};
  double map_wall_x0_{0.0}, map_wall_y0_{0.0}, map_wall_x1_{0.0}, map_wall_y1_{0.0};

  // Wall endpoints re-projected into the costmap's current global frame, refreshed every
  // updateBounds() call from the locked map_frame_ pose above.
  double wall_x0_{0.0}, wall_y0_{0.0}, wall_x1_{0.0}, wall_y1_{0.0};
  bool wall_valid_{false};

  // Bounding box reported last cycle. LayeredCostmap only clears the region the layers
  // ask for, so if the re-projected pose shifts slightly (global frame drifting against
  // map_frame_), the previous footprint has to stay inside the bounds or its cells linger.
  double prev_min_x_{0.0}, prev_min_y_{0.0}, prev_max_x_{0.0}, prev_max_y_{0.0};
  bool has_prev_{false};

  bool bounds_checked_{false};
};

}  // namespace costmap_wall

#endif  // COSTMAP_WALL__WALL_LAYER_HPP_
