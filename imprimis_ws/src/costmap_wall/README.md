# costmap_wall

A Nav2 costmap layer that drops a straight line of fixed cost behind the robot **once**, at
startup, and leaves it there forever, anchored in the map.

The intended use is a one-shot virtual wall at the robot's starting position that keeps the
planner from ever doubling back past that point, for the rest of the run.

```
                              (robot, at startup)

    ------------------------------------------------   <- wall_length, at y_offset
```

## How it works

As soon as a transform from `wall_relative_to` to `map_frame` becomes available (typically on
the very first costmap update after the robot's pose is known), the layer computes the wall's
two endpoints in `map_frame` and locks them in. From that point on `wall_relative_to` is never
looked at again — the robot driving away does not move the wall, and nothing in this layer
re-triggers the drop.

Because the costmap's own frame can be something other than `map_frame` (a rolling local
costmap typically tracks `odom`, which drifts relative to `map` as localization corrects
itself), the locked endpoints are re-projected into the costmap's global frame on every
update cycle. The wall's world position never changes; only the coordinates it's drawn at,
inside whatever frame the costmap happens to use, are recomputed each cycle.

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
        map_frame: "map"
        y_offset: -1.0
        wall_length: 4.0
        cost_value: 254 # lethal obstacle
```

Put `wall_layer` *before* `inflation_layer` if you want the wall inflated like any other
obstacle, or after it if you want the raw line only.

## Parameters

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `enabled` | bool | `true` | Turn the layer off. While disabled, the wall is neither dropped nor drawn; once dropped it stays locked even if disabled and later re-enabled. |
| `wall_relative_to` | string | `base_link` | Frame the wall is dropped relative to, read exactly once, at drop time. |
| `map_frame` | string | `map` | Frame the wall is permanently anchored in once dropped. |
| `y_offset` | double | `-1.0` | At drop time, distance along `wall_relative_to`'s **forward (+x)** axis. `0.0` drops the wall through the frame's origin, negative drops it behind the robot (the intended use), positive drops it in front. |
| `wall_length` | double | `4.0` | Total length of the wall. It is centred on the robot's projection onto it, so it extends `wall_length / 2` to either side. |
| `cost_value` | int | `254` | Cost written into each cell the wall passes through. `254` is `LETHAL_OBSTACLE`. |

`y_offset` is named for the vertical axis of the picture above, not for the frame's y axis —
at drop time it moves the wall backward and forward relative to the robot, perpendicular to
the wall itself.

## Notes

- The wall only drops once per node lifetime. If `wall_relative_to -> map_frame` isn't
  available yet (e.g. localization hasn't published a pose), the layer logs a throttled
  warning and keeps retrying every cycle until it succeeds, then locks and never looks again.
- Cells are only ever *raised*: an existing cost higher than `cost_value` is left alone. Cells
  marked `NO_INFORMATION` are overwritten, so the wall shows up in unknown space too.
- On startup the layer checks whether the wall fits in the costmap. The worst case is a wall
  endpoint, `hypot(y_offset, wall_length / 2)` from the frame origin; if that exceeds half the
  costmap's smaller dimension, a warning is printed, because any part of the wall that falls
  outside the grid is silently dropped. With a rolling window the robot sits at the centre at
  drop time, so this is exactly the condition for the wall fitting at the moment it's placed.
- `isClearable()` is `false`: the wall is generated rather than sensed, so `clear_costmap`
  services leave it alone. A costmap `reset()` (deactivate/activate, or a clear triggered by
  another layer) does not unlock or move the wall either — once dropped, it is permanent for
  the life of the node.
