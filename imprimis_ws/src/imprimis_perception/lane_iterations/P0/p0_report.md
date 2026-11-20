# P0 — Lane memory in the cost map (2026-11-14)

**Question:** once the camera stops seeing a lane line (it passes beside the robot, or curves out of view), does the
cost map still hold it? Which of the two lane layers in the repo does that better?

**Answer:** Lane2027's lane layer (nav2 ObstacleLayer) keeps the lines. Main's Course2027 setup (STVL `lane_mark`
in the same voxel layer as the Velodyne frustum clearing) forgets most of a line beside the robot at 0.5 to 1 m/s,
and keeps false marks. Do not move lanes into main's STVL layer; keep lanes in their own layer.

Tool: `harness/clearing_study.py` with `harness/configs/p0_*.yaml`. Raw output in `harness/out/p0/` (not committed);
the summary and figures used here are in `Replays/`. Checked independently by a second agent (section 5).

## 1. Setup

- A real `nav2_costmap_2d` node (Kilted) with one lane-layer configuration, no inflation, no LiDAR marking: a lethal
  cell is a lane mark and nothing else.
  - `p0_obstacle_layer.yaml`: the lane layer copied from **Lane2027.yaml** (ObstacleLayer, 2D ray-trace clearing from
    `d435i_color_optical_frame`, 0.1 m cells).
  - `p0_stvl.yaml`: copied from **main's Course2027.yaml** (commit 6bd5dd2): STVL, 0.025 m voxels, `lane_mark`
    (marking only, 30 s decay) and `velodyne_clear` (Velodyne frustum, `decay_acceleration` 5) in one layer.
  - `p0_stvl_no_lidar_clear.yaml`: the same without `velodyne_clear` (ablation).
- A virtual robot drives the centerline of a 10 ft lane at 1.0 m/s. At 10 Hz the script publishes the robot's pose
  on TF and the cloud `lane_projection` would publish at that pose, made with the real projection code
  (`build_ground_lookup`, `mask_to_ground_xy`, `thin_structures` with the same arguments as `mask_cb`). Front D435i
  as in `test/conftest.py` (0.686 m up, 12 deg down), intrinsics from the real CameraInfo. Segmentation is perfect
  except in the noisy case.
- Paths: straight 12 m; left and right arcs of 120 deg with inner-line radius 1.524 m (the tightest reading of the
  5 ft rule); an S-curve; the left arc with 20 % empty frames and 3 cm lateral jitter; one false mark (a 0.2 m
  patch at 0.8 m off the path, one frame).
- Metrics, per published cost map:
  - **beside memory:** fraction of the true line beside the robot (base_link x from -0.9 to 1.3 m: the tail to where
    the camera starts seeing the ground) that the camera saw earlier and that is still marked.
  - **ahead:** fraction of the line in view (x 1.3 to 4.2 m) that is marked.
  - **band width:** lethal area within 0.3 m of a line, divided by the marked line length, in the same window.
  - **false-mark lifetime:** from the first cost map that shows the false patch to the first that does not.

## 2. Results (1.0 m/s)

| Layer | Path | Beside memory | Ahead | Band width | False mark lasts |
|---|---|---|---|---|---|
| Lane2027 ObstacleLayer | straight | **1.00** | 0.93 | 0.13 m | |
| | left arc | **1.00** | 0.94 | 0.15 m | |
| | right arc | **1.00** | 0.93 | 0.15 m | |
| | S-curve | **1.00** | 0.92 | 0.13 m | |
| | left arc, noisy | **1.00** | 0.92 | 0.16 m | |
| | false mark | 1.00 | 0.96 | 0.13 m | **0.7 s** |
| main STVL | straight | **0.17** | 0.88 | (dotted line) | |
| | left arc | **0.64** | 0.87 | | |
| | right arc | **0.63** | 0.88 | | |
| | S-curve | **0.49** | 0.86 | | |
| | left arc, noisy | **0.61** | 0.87 | | |
| | false mark | 0.49 | 0.91 | | **> 5 s** (5 of 9 cells, until the run ended) |
| STVL without Velodyne clearing | straight | **1.00** | 0.91 | | |
| | left arc | **1.00** | 0.92 | | |
| | false mark | 1.00 | 0.91 | | **> 5 s** (30 s decay) |
| main STVL, `output_height` 0.0 | straight | **0.00** | **0.00** | nothing marked | |

`Replays/stvl__left_arc.png`: at the end of the curve, the whole inner arc and most of the outer arc are gone from
the cost map. `Replays/obstacle_layer__left_arc.png`: both lines are still there.

## 3. What it means

1. **Lane2027's ObstacleLayer remembers lines** on straights and on the tightest legal curves. Its 2D ray-trace
   clearing never erased a true line cell in any case (0 erased). It also clears a false mark inside the camera's
   view in 0.7 s, because rays to the real lines behind the mark pass over it. A false mark outside that fan, or one
   left when the camera sees no tape, has no decay in this layer and stays.
2. **Main's STVL setup forgets lanes, because of the Velodyne.** The camera sees a line at x 2.1 to 4 m. The
   Velodyne's frustum (sensor 1.10 m above base_link, lowest ring 15 deg down) covers lane voxels there (at
   z = 0.5 m in odom, anything farther than 2.26 m from the LiDAR). An unseen voxel inside the frustum lives about
   0.6 s at `decay_acceleration` 5. Once the camera has passed it, a lane voxel must leave the frustum (reach
   x < 1.32 m) within those 0.6 s or it is erased. Removing `velodyne_clear` restores full memory, so this is the
   only cause.
   - **It depends on speed** (verifier's sweep, straight): beside memory 0.00 at 0.5 m/s, 0.21 at 1.0, 1.00 at 1.5 m/s
     and above. The IGVC minimum is 1 mph (0.45 m/s) and the robot runs at up to 1.0 m/s and slows in curves and
     near obstacles, so the bad range is the one we drive in.
   - It also keeps false marks for the full 30 s outside the frustum: the opposite of what we want.
   - Lowering the Velodyne (planned by the mechanical team) moves the frustum edge and will change these numbers.
3. **Lane heights are a trap.** Main's `lane_mark` keeps points from z = 0.0 m in odom. On the robot the ground is
   at about -0.12 m in odom, so `lane_projection` with `output_height` 0.0 (the default on lanemerge, and what
   Lane2027 expects) has every lane point dropped. On the tripod, `base_link` sits on the ground, so the same
   setting lands exactly on 0.0. A height window tuned on the tripod can silently drop all lanes on the robot.
4. **The cost map is not what thickens the lanes.** With clean input the band is 0.13 to 0.16 m for 0.10 m tape
   (1.3 to 1.6 cells at 0.1 m). With 3 cm jitter it is 0.16 m. If lanes look much wider than that on the robot, the
   cause is upstream (segmentation and projection noise, calibration) or the inflation layer. That is what P1 to P3
   measure on real bags.
5. At 0.025 m voxels the 5 cm skeleton becomes a dotted line (cells every other voxel). Whatever resolution Lane2027
   ends up at must be checked against this in P4.

## 4. Limits

Synthetic and perfect segmentation (one noisy case); one front camera; no odometry drift, pitch or roll; flat
ground; no obstacles, LiDAR veto, inflation or other layers; Velodyne tilt (1.9 deg) not modelled; one run per case
(a rerun of `stvl__straight` gave 0.26 instead of 0.17); runs end about 5 s after the last frame, so the 30 s decay
itself was never observed; false mark tested at one place. "main" here means main's STVL configuration fed by
lanemerge's `lane_projection`; main's own (older) projector publishes at z = 0.5 m in base_link.

## 5. Verification (second agent, sonnet)

Recomputed claims 1 to 4 from the saved runs with its own code and its own probes: claims 1, 3 and 4 confirmed; claim
2 confirmed at 1 m/s and found speed-dependent (section 3.2); the frustum mechanism located precisely. It found two
harness bugs, both fixed before this report: the band-width metric counted remembered cells behind the robot (it
read 0.20 to 0.28 m, now 0.13 to 0.16 m), and earlier false-mark runs were confounded by footprint clearing (the
patch was on the robot's path; now 0.8 m off it). It notes the scoring tolerance (8 cm) is generous for the 2.5 cm
STVL cells, which flatters STVL.

## 6. Recommendation

- Keep lanes in their own ObstacleLayer (as Lane2027 does). Do not put them in main's shared STVL layer.
- When Lane2027 adopts main's STVL Velodyne layer, keep `lane_layer` separate, and set the
  lane height window to work for both the tripod (ground at 0 in odom) and the robot (ground at -0.12).
- Add a decay or a clearing rule for false marks the camera never re-sees; test it here before using it.
- Next: real bags (P1 to P3) to find where real lanes thicken.
