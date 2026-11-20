# Lane Detection Test Harness

How the team and an AI assistant tune Imprimis's lane detection for the IGVC 2027 Auto-Nav Challenge, one measured
iteration at a time. The structure follows the Nav2 tuning cycle in the repository's `iterations/` folder: the team tests
and reports, the AI analyses the bags and proposes changes, and every change has to show up in the numbers.

This folder belongs to `imprimis_perception` so that lane work stays separate from navigation work and the branches merge
cleanly.

---

## 1. Project

The robot competes in the **Auto-Nav Challenge** only, not Self-Drive. The course rules that matter for lanes (IGVC 2027
rules, 7 July 2026 version, section II.2 and II.4):

| Rule | Value |
|---|---|
| Surface | Asphalt |
| Track width | 10 to 20 ft, "approximately ten feet" (3.05 m) |
| Turning radius | "not less than five feet". The rules do not say where it is measured. This harness assumes the **inner line** has a radius of at least 1.52 m (centerline 3.05 m in a 10 ft lane), the tightest reading that can exist. |
| Lines | White tape, 3 to 4 in wide, continuous **or dashed** |
| Shape | "Primarily sinusoidal curves" with barrels |
| Potholes | Solid white circles, 2 ft across. Touching one ends the run. |
| Obstacles | At least 5 ft of driving space between a line and an obstacle |
| Careless driving | Any part of the vehicle over a line: -5 ft. All of it over: leave the course, -10 ft and end of run. |

## 2. The lane pipeline as it stands (lanemerge)

| Stage | Where | What it does |
|---|---|---|
| Segmentation | `lane_segmentation.py`, one per camera, on a Pi | Shrinks the color image to 640 px wide, keeps pixels that are brighter than their surroundings (top-hat) and colorless, drops blobs outside an area range. Publishes a PNG mask on `/perception/<cam>/lane_mask/compressed`. |
| Projection | `lane_projection.py`, on the laptop | Builds a lookup from mask pixel to flat ground using CameraInfo `K` and the static TF of the camera. Removes white areas wider than 0.45 m, thins the tape to a 5 cm center line (`surface_tools.thin_structures`), vetoes points near or behind LiDAR obstacles. Publishes `/perception/lane_points` in `base_link`. |
| Cost map | `config/nav2/Lane2027.yaml` | `lane_layer` is an ObstacleLayer with 2D ray-trace clearing from `d435i_color_optical_frame`, 0.1 m cells, inflation 1.3 m global / 1.0 m local, `cost_scaling_factor` 1.7. |
| Tripod | `launch/tripod.launch.py` | Static `odom -> base_footprint -> base_link -> front_link` with only `height` and `tilt` as arguments, then `single_cam.launch.py`. |

Facts measured so far (the tool or source is named for each):

- **Camera intrinsics.** CameraInfo in `lane_test_103026`: 1280x720, `plumb_bob` with all-zero distortion, fx 909.3,
  fy 907.0, cx 636.1, cy 354.3. At 640 px wide this is the (455, 455, 320, 180) used by `test/conftest.py`.
- **Blind zone, front D435i** (0.686 m high, 12 deg down; `test_projection_geometry.py::test_nearest_visible_ground`):
  ground visible from x = 1.24 m in `base_link` straight ahead; a line 1.52 m to the side first visible at x = 2.2 m.
  The robot's nose is at x = +0.22 m.
- **Curves, one front camera** (scratch simulation with `build_ground_lookup`): robot on the centerline of a 10 ft lane
  turning left, centerline radius 2 to 6 m. The inner line is never in view. The outer line is (2.5 to 4 m of it).
- **Tilt sensitivity** (geometry, 0.686 m camera height): 1 deg of tilt error moves tape about 11 cm at 2 m, 22 cm at
  3 m, 38 to 47 cm at 4 m. **Height sensitivity:** 1 cm of height error moves tape about 4.4 cm at 3 m.
- **One pixel at 640 px wide** covers about 2.2 cm of ground along the range at 3 m, 3.9 cm at 4 m.
- **Lane2027.yaml was copied from Course2027.yaml before Nav2 iterations 1 and 2.** Velodyne is still on an
  ObstacleLayer with clearing off and the global map is 100 m. Course2027.yaml now uses STVL (30 m, 0.025 m), a
  different planner, controller critics and footprint, and has its own STVL `lane_mark` source (clearing off, 30 s
  decay). Which lane memory model to use is open; see P0.
- **How ObstacleLayer clearing treats lanes.** Each update first clears along rays from the sensor to every new point,
  then marks the new points. So a remembered cell is erased only when a ray to a new detection passes through it.
  Consequences: cells right around the camera are wiped every frame; in a turn, rays sweep across remembered lines;
  with several cameras, one camera's rays cross another's lines; a frame with no detections clears nothing, so a wrong
  mark never goes away. None of this has been measured yet.
- **Projection caches.** `lane_projection` keeps the first CameraInfo, every static TF and the ground height forever.
  After any change to the camera mount, restart it.
- **Not used yet:** depth (the lane pipeline never reads it), IMU pitch and roll (the projection assumes the robot
  is level on flat ground), the LiDAR veto (the tripod has no LiDAR).
- **Existing data:** `~/Desktop/lane_test_103026-20261113T201554Z-1-002/lane_test_103026` (7.7 GB mcap: color, depth,
  CameraInfo, tf_static, velodyne, odom, masks) and `~/Desktop/lane_detection_102726_2` (2.7 GB, older
  `/camera/camera/*` topic names).

Still undecided: **2 to 4 cameras.** P0 is there to help decide.

## 3. Folder structure

```
imprimis_perception/
  lane_iterations/
    README.md                     this file
    1/
      real_report_1.pdf           team: what was tested, conditions, photos, observations, questions
      layout_1.yaml               team: measured tape layout, tripod poses, slope, sun, camera settings (section 7)
      RealBags/                   team: the stationary bags (gitignored; never deleted)
        real_bag1
        ...
      Replays/                    AI: real bags run through the old and the new code; commit figures and CSVs only
      OldFiles/                   AI: every file as it was before this iteration changed it (gitignored)
      change_report_1.pdf         AI: section 5
      verification_report_1.pdf   a second, independent agent: section 6
    2/
      (same as 1)
  harness/                        the tools, committed (section 9)
```

The Nav2 iterations lost their analysis tools because `claude_tools/` was never committed. Here every tool that produces
a number in a report lives in `harness/` and is committed with the report.

## 4. The cycle

1. **Team:** lay out tape on asphalt as the phase's test card says (section 8), measure it into `layout_N.yaml`, record
   the bags with the bag command (section 7), and write `real_report_N.pdf`.
2. **AI:** copy every file it intends to change into `OldFiles/`, then run the harness on `RealBags/` to get baseline
   numbers.
3. **AI:** explain each proposed change and the reasoning, and get the user's permission **for each file** before
   touching it.
4. **AI:** replay the **same** bags through the changed code into `Replays/`. A change is kept only if the phase's
   target metric improves on real bags and no other metric gets worse.
5. **AI:** write `change_report_N.pdf`. A second agent (model: sonnet) checks it and writes
   `verification_report_N.pdf`.
6. **Team:** re-test on asphalt with the changes. Next iteration.

Do not move to the next phase until the current phase's pass criteria hold on real bags.

## 5. Change report contents

The formatting is up to the author as long as it covers:

- a. Issues seen in the real bags
- b. Root causes, with the tool and bag behind every number
- c. Issues seen only in replay or synthetic tests
- d. Root causes of those
- e. Metrics table: each pass-criterion metric, baseline -> after, per bag
- f. Parameter changes
- g. Code changes
- h. Cost map configuration changes
- i. How f to h address a to d, with the replay evidence
- j. Changes recommended outside the allowed files (described, not made)
- k. The test card for the next iteration

## 6. Verification report contents

Written by a different agent that did not make the changes:

- Re-derive every headline number in the change report from the bags and tools, and say which ones it reproduced.
- Check that the changed files match what the change report says, and that nothing else changed.
- Look for the change helping one metric while hurting another.
- List anything it could not verify.

## 7. Recording a stationary test

### Bag command (tripod, one camera)

```bash
ros2 bag record -s mcap -o real_bagK \
  /cameras/d435i/color/image_raw /cameras/d435i/color/camera_info /cameras/d435i/color/metadata \
  /cameras/d435i/imu \
  /perception/d435i/lane_mask/compressed /perception/lane_points /perception/pothole_points \
  /local_costmap/costmap /global_costmap/costmap \
  /tf /tf_static /rosout
```

- `/cameras/d435i/imu` gives an independent tilt reference (gravity). It needs the RealSense driver started with
  `enable_accel:=true enable_gyro:=true unite_imu_method:=2`; `lane_segmentation.launch.py` does not pass these yet.
- Add `/cameras/d435i/depth/image_rect_raw /cameras/d435i/depth/camera_info /cameras/d435i/extrinsics/depth_to_color`
  only for P1 bags, where depth gives a second, independent fit of the ground plane.
- On the robot (P6) also add `/velodyne_points /velodyne_points_nav /imu/data /diffbot_base_controller/odom
  /odometry/filtered/local`.
- **When replaying,** `/tf_static` needs `--qos-profile-overrides-path` with transient-local durability, or it is
  silently lost. Do not run `tripod.launch` or the camera driver at the same time as a replay.

### Each bag

- At least 30 s with the tripod still. Note in `layout_N.yaml` when it was moved; those frames are cut.
- Lock exposure and white balance in the driver, or record `color/metadata` (above). Segmentation uses fixed thresholds.

### `layout_N.yaml`

```yaml
date: 2026-11-20
site: "lot behind engineering, east end"
surface: {material: asphalt, condition: "dry, some sealant cracks"}
slope: {along_x_pct: 1.2, along_y_pct: 0.4, how: "digital level, 1 m bar"}
sun: {time: "13:10", sky: "clear", heading_to_sun_deg: 140}   # camera heading relative to the sun
tape: {width_in: 4, color: white, condition: "new"}
camera: {model: d435i, exposure: "locked 156", white_balance: "locked 4600"}
lines:                                # ground frame of the layout, metres, x forward from the origin mark
  - id: left
    kind: solid                       # or dashed: dash_m, gap_m
    points: [[0.0, 1.524], [6.0, 1.524]]
  - id: right
    kind: solid
    points: [[0.0, -1.524], [6.0, -1.524]]
  - id: arc_inner                     # an arc: center, radius, start and end angle in degrees
    kind: solid
    arc: {center: [6.0, 3.05], radius: 1.524, from_deg: -90, to_deg: 0}
potholes:
  - {center: [4.0, 0.3], diameter: 0.61}
poses:                                # where the tripod stood for each bag
  - bag: real_bag1
    base_link: {x: 0.0, y: 0.0, yaw_deg: 0.0}
    height_m: 0.686                   # camera lens center above the ground, tape measure
    tilt_deg: 12.0                    # from the inclinometer
    held_out: false                   # true: never used for calibration, only for scoring
    moved_at_s: []                    # bag times when the tripod was being moved
measurement_error: {position_m: 0.01, yaw_deg: 0.5}
```

## 8. Phases

Each phase has a test card for the team, the files the AI may ask to change, and pass criteria. The numbers are set
against the hardware: 2.2 cm per pixel at 3 m, 5 cm skeleton cells, 10 cm cost map cells.

### P0. Geometry study (no bags)

- **Camera count.** For 2, 3 and 4 candidate mounts, compute how much of each line is visible at every pose along
  arcs with inner radius 1.52 m, 3 m and 5 m, and along a sinusoid. Gives the team the 2 vs 4 camera decision.
- **Cost map memory.** Feed synthetic lane clouds to a real `nav2_costmap_2d` node while sweeping the sensor's position
  and yaw. Measure remembered cells erased per update, by bearing and range, for the ObstacleLayer in Lane2027 and the
  STVL `lane_mark` in Course2027. Gives the lane memory decision.
- **Files:** none. **Done when** both decisions are written up and agreed.

### P1. Calibration (tripod)

- **Test card:** two lines 3.05 m apart and 6 m long, cross lines at 1, 2, 3 and 4 m. Three tripod poses for
  calibration and at least two **held-out** poses. Re-mount the camera three times.
- **Method:** fit tilt, height, roll and yaw from tape corners picked by hand (not from the segmentation mask). Check
  tilt and roll against the D435i accelerometer and a depth-image ground plane fit. Score only on held-out poses.
- **Files:** `tripod.launch.py` (add x, yaw and roll arguments).
- **Pass:** median ground position error on held-out poses at most 5 to 6 cm at 3 m and 10 cm at 4 m; the three
  re-mounts agree to within the same.

### P2. Segmentation

- **Test card:** the P1 layout plus a dashed section, a pothole, a barrel with white stripes and a worn piece of tape,
  recorded in sun, shade, the tripod's own shadow, overcast, wet, glare, and at least four headings to the sun.
- **Method:** project the measured layout through the P1 calibration to get the true tape in each frame. Score
  precision and recall on the ground in metres, ignoring 2 px at each tape edge. Run segmentation offline by calling
  `segment_lane_pixels` directly on bag frames, not through the node, so replays are repeatable.
- **Files:** `lane_segmentation.py` (parameters, then code).
- **Pass:** recall at least 0.9 to 3 m and false-positive ground area off the tape under a limit set from the P2
  baseline, in every condition.

### P3. Projected lane points

- **Method:** compare `/perception/lane_points` (replayed) with the true lines on held-out poses: offset from the line
  by range, points off any line, frame-to-frame jitter with the tripod still.
- **Files:** `lane_projection.py`, `surface_tools.py`.
- **Pass:** jitter at most one cost map cell (10 cm); offset within the P1 limits.

### P4. Cost map

- **First:** rebase the lane setup onto the current Course2027.yaml with the lane memory model chosen in P0 (needs
  permission for `Lane2027.yaml`).
- **Method:** replay the bags into a real `nav2_costmap_2d` node. Measure lane band width per metre of line, offset of
  the band from the true line, growth over 5 minutes, cells flipping per second, the inflation cost across the lane,
  and whether a straight path down the lane is plannable with the real footprint.
- **Files:** `Lane2027.yaml` (lane layer and inflation).
- **Pass:** band at most 2 cells wide and centered within 1 cell of the line; no growth over 5 minutes.

### P5. Curves and memory (stop-motion)

- **Test card:** arcs with inner radius 1.52 m, 3 m and 5 m, one dashed. Surveyed tripod spots every 0.5 m along the
  centerline, through the curve entry, apex and exit, with the heading of each.
- **Method:** `harness/step_pose_publisher.py` publishes the surveyed `odom -> base_link` for each spot as a step (not
  interpolated) while the bag replays with sim time. Frames taken while moving the tripod are cut. This checks memory
  and clearing with known poses; it does not check latency, motion blur or odometry drift.
- **Files:** the lane memory settings from P4.
- **Pass:** every line that the chosen cameras can see along the next 3 m of path is present in the cost map at every
  spot, and no remembered line cell is wrongly cleared. With one front camera the inner line of a tight curve is
  expected to be missing; P0 says which camera count fixes it.

### P6. On the robot

- Repeat P1 and P4 with the cameras on the robot, using the URDF mounts and the Pi-to-laptop link.
- New items: LiDAR veto, robot pitch and roll (braking, slopes, ramps up to 15%), camera height as loaded, clock sync
  between the Pis and the laptop (chrony), mask latency.
- Then dynamic testing.

## 9. Harness tools (`imprimis_perception/harness/`)

Kept out of the installed package (`harness/` has no `__init__.py`, so `find_packages` in `setup.py` skips it) and out
of `package.xml`'s runtime dependencies.
Tests for the tools go in `test/` and use `conftest.py`.

| Tool | Phase | Does |
|---|---|---|
| `bag_tools.py` | all | Read mcap bags without starting ROS nodes |
| `layout.py` | all | Load and check `layout_N.yaml`; turn lines and arcs into ground points |
| `coverage.py` | P0 | Visible line length per camera mount set, along arcs and sinusoids |
| `clearing_study.py` | P0 | Synthetic clouds into a real cost map node; erased-cell statistics |
| `calibrate_mount.py` | P1 | Fit tilt, height, roll and yaw from picked corners; IMU and depth cross-checks; held-out scoring |
| `score_segmentation.py` | P2 | Offline segmentation of bag frames, scored against the projected layout |
| `sweep_segmentation.py` | P2 | Parameter sweeps over the same frames |
| `score_projection.py` | P3 | Offset, off-line points and jitter of the lane cloud |
| `costmap_replay.py` | P4, P5 | Real `nav2_costmap_2d` fed by bag replay; band width, growth, flicker, plannability |
| `step_pose_publisher.py` | P5 | Surveyed poses published as steps in sim time |
| `report_figures.py` | all | The figures and CSVs that go in `Replays/` and the reports |

## 10. Rules for the AI

- Touch no file without the user's permission, given for that file. Changes that would help elsewhere go in section j
  of the change report.
- Explain the reasoning and every planned change before making it.
- Have an independent agent (model: sonnet) check big changes; that is the verification report.
- Every number in a report names the tool and the bag it came from.
- Never delete real bags.
