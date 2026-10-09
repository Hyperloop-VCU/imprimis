# costmap_wall

A Nav2 costmap layer that stamps a straight line of fixed cost at a fixed pose relative to
a robot frame. The line translates and rotates with that frame, so it is always in the same
place as far as the robot is concerned.

The intended use is a virtual wall behind the robot that keeps the planner from backing up
or turning around.

```
                              (robot)

    ------------------------------------------------   <- wall_length, at y_offset
```

## Usage

Add it to the `plugins` list of either costmap:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "wall_layer", "inflation_layer"]

      wall_layer:
        plugin: "costmap_wall::WallLayer"
        wall_relative_to: "base_link"
        y_offset: -1.0
        wall_length: 4.0
        cost_value: 254 # lethal obstacle
```

Put `wall_layer` *before* `inflation_layer` if you want the wall inflated like any other
obstacle, or after it if you want the raw line only.

## Parameters

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `enabled` | bool | `true` | Turn the layer off without removing it. |
| `wall_relative_to` | string | `base_link` | Frame the wall is pinned to. |
| `y_offset` | double | `-1.0` | Distance along the frame's **forward (+x)** axis. `0.0` puts the wall through the frame origin, negative puts it behind the robot (the intended use), positive puts it in front. |
| `wall_length` | double | `4.0` | Total length of the wall. It is centred on the robot's projection onto it, so it extends `wall_length / 2` to either side. |
| `cost_value` | int | `254` | Cost written into each cell the wall passes through. `254` is `LETHAL_OBSTACLE`. |

`y_offset` is named for the vertical axis of the picture above, not for the frame's y axis —
it moves the wall backward and forward, perpendicular to the wall itself.

## Notes

- The wall is re-projected into the costmap's global frame every update cycle from the live
  `global_frame -> wall_relative_to` transform, so it follows the robot's pose and heading.
- Cells are only ever raised: an existing cost higher than `cost_value` is left alone. Cells
  marked `NO_INFORMATION` are overwritten, so the wall shows up in unknown space too.
- On startup the layer checks whether the wall fits in the costmap. The worst case is a wall
  endpoint, `hypot(y_offset, wall_length / 2)` from the frame origin; if that exceeds half the
  costmap's smaller dimension, a warning is printed, because any part of the wall that falls
  outside the grid is silently dropped. With a rolling window the robot sits at the centre, so
  this is exactly the condition for the wall fitting.
- `isClearable()` is `false`: the wall is generated rather than sensed, so `clear_costmap`
  services leave it alone.
