# Lane iteration 1 — Change report (2026-11-14)

**One change:** `Lane2027.yaml` brought up to main's real-robot Nav2 tuning (Course2027.yaml, nav iterations 1-5),
keeping lanes out of main's STVL layer. No real bags yet; the evidence is the P0 study and two checks against real
Nav2 servers (sections e, i).

Files changed (old versions in `OldFiles/`):

| File | Change |
|---|---|
| `imprimis_navigation/config/nav2/Lane2027.yaml` | Rebuilt: main's `Course2027.yaml` (origin/main 6bd5dd2) + edits tagged `LANE` |
| `imprimis_navigation/config/nav2/diff_4cm.json` | New, copied unchanged from main (the Smac Lattice planner's primitives) |
| `imprimis_navigation/nav2_bt/recovery_nav_bt.xml` | Replaced with main's iteration-5 tree, unchanged (added after the verification below; see j.1) |
| `imprimis_perception/harness/nav_config_check.py` | New tool: loads a params file into the real Nav2 servers and plans around a lane curve |
| `imprimis_perception/harness/configs/p1_lane2027_local.yaml` | New, generated from the new Lane2027 local cost map for `clearing_study.py` |
| `imprimis_perception/harness/clearing_study.py` | Cases `lane2027__*` added |

## a-b. Issues and root causes (from the configs and P0; no real lane bags yet)

1. **Old Lane2027 predated every nav fix made on the real robot.** It was copied from Course2027 before nav
   iteration 1. It still had:
   - The Velodyne in an ObstacleLayer with clearing off, which is what closed the corridor in nav iteration 1.
   - `vx_max` 2.2 with `wz_max` 1.5. A speed limit scales the turn rate by the same ratio, so at 1.0 m/s the turn
     rate drops to 0.68 rad/s.
   - The old footprint plus 0.08 m padding.
   - 50 Hz cost map updates.
   - The Hybrid planner at 0.1 m.
2. **Main's own lane setup erases lanes.** P0 showed that main's STVL `lane_mark` forgets 36 to 100 % of the line
   beside the robot at 0.5 to 1.0 m/s. The cause is the Velodyne frustum clearing in the same layer.
3. **Old Lane2027 dropped every pothole on the robot.** Its pothole source kept points from z = 0.0 m in odom, while
   `lane_projection` puts them at about -0.12 m. The verifier showed this directly: with the old file the planned
   path ran straight through a pothole-marked curve (closest approach 0.00 m).
4. **Old Lane2027 logged a real error.** Nav2 reported the local inflation radius (1.0 m) as smaller than the padded
   footprint's circumscribed radius (1.03 m). Non-circular collision checking falls back when that happens.

## c-d. Replay-only / synthetic issues

- The P0 harness scores a dotted line for lanes at 0.025 m cells (the 5 cm skeleton fills every other cell). Inflation
  makes the 253 band continuous, so the planner cannot pass through, but the lethal line itself has gaps.

## e. Metrics

| Check | Old Lane2027 | New Lane2027 | Tool |
|---|---|---|---|
| All 6 Nav2 servers active | yes (1 inflation ERROR) | **yes, no errors** | `nav_config_check.py` |
| Planned path around a 5 ft inner-radius curve, closest approach to inner line | 1.06 m | **1.33 m** | same |
| Same, potholes instead of lanes | 0.00 m (cuts through) | **1.33 m** | verifier's variant |
| Same, no lane cloud (proves the planner sees lanes) | | 0.01 m (cuts the corner) | verifier's variant |
| Line beside the robot remembered, local cost map, 0.5 m/s, 5 paths | | **1.00** all | `clearing_study.py lane2027__v0.5__*` |
| Same at 1.0 m/s | | **0.994 to 1.00** | `lane2027__v1.0__*` |
| Off-path false lane mark lifetime | | 1.3 s (0.5 m/s), 0.6 s (1.0 m/s) | `*__false_positive` |

## f-h. What changed in Lane2027

All of main's Course2027, including its comments and reasoning:
- **Planner:** Smac Lattice with `diff_4cm.json`.
- **MPPI:** `vx_max` 1.0, `wz_max` 0.9, `vx_min` -0.3, `time_steps` 30 and the re-scaled critics.
- **Footprint:** the re-measured 8-point footprint with no padding.
- **Cost maps:**
  - Resolution: global 0.04 m, local 0.025 m.
  - Rates: local 10 Hz update / 20 Hz publish, global 2 Hz update.
  - Velodyne in an STVL layer with frustum clearing, plus the wall layer.
- **Checkers:** goal tolerance 1.0 m, progress checker, `failure_tolerance`.
- **Velocity smoother:** 20 Hz, timeout 0.4 s.

The `LANE` edits on top of that (also listed in the file's header):
1. `lane_mark` and `pothole_mark` taken out of `stvl_layer` in both maps.
2. `lane_layer` added to both maps. It is identical to the old Lane2027's: ObstacleLayer, ray clearing from
   `d435i_color_optical_frame`, height window -0.3 to 0.1.
3. `pothole_layer` added to both maps, marking only. It now uses the -0.3 to 0.1 window so that ground points are kept.
4. `lattice_filepath` written as `$(find-pkg-share imprimis_navigation)/../../../../src/...`, like the behaviour tree
   path. The verifier confirmed it resolves through `basic_nav` and nav2_bringup. Main's version is hard-coded to
   `/home/imprimis-pc`.
5. Inflation kept at the old Lane2027 values (global 1.3, local 1.0) instead of main's 2.0 and 1.3. See section j.

## i. How this addresses a-d

The navigation behaviour is now the one the team tuned on the real robot. Lanes stay in a layer that remembers them,
and potholes are no longer dropped. Section e shows each of these in real Nav2 servers.

## j. Recommended, not done (outside what was approved)

1. **Behaviour tree (most important).** Lane2027 points at this branch's `nav2_bt/recovery_nav_bt.xml`, which is
   older than main's iteration-5 tree. Differences:
   - Main retries 4 times; this branch retries 8. Main measured that 8 retries outlive the 30 s voxel decay and
     led to a collision.
   - This branch's tree does not recover from START_OCCUPIED.
   - Its BackUp distances get vetoed.
   - It clears the whole cost map as a last resort, which also wipes the remembered lanes.

   Fix: copy main's `recovery_nav_bt.xml` too.

   **DONE (approved by the user after the first verification).** Main's iteration-5 tree is now in this branch,
   byte for byte. The old one is in `OldFiles/recovery_nav_bt.xml`. Checks:
   - `nav_config_check.py` on Lane2027: all 6 servers active, no errors, same path as before.
   - A scratch test (robot held still, real `NavigateToPose` goal, `/behavior_tree_log` recorded for 25 s): the tree
     loaded and ticked `ComputePathToPose`, `FollowPath`, then the recovery branch (`Wait`, `BackUp`,
     `AreErrorCodesPresent`, `EscalateToClearing`). FollowPath failed with 105 (no progress) because the fake robot
     cannot move, which is the expected result.

   Two things that come with it:
   - **Other params files now use this tree too.** This branch's `Course2027.yaml` (the iteration-2 version) and
     `FullFootprint.yaml` point at the same tree file. Main pairs this tree with main's Course2027, not this branch's.
   - **The last resort still wipes the remembered lanes.** Its final step clears the whole local cost map, lanes
     included; the global map is kept. A lane-aware alternative (clear only `stvl_layer`, or clear except around
     the robot) is for a later iteration.
2. **Inflation.** It is a choice between main's proven barrel clearance (2.0 / 1.3) and the old lane values
   (1.3 / 1.0). With `cost_scaling_factor` 1.7, the lane center costs about 28 from each line at 2.0, so main's values
   would probably be fine for lanes too. This gets measured in P4. Until then the barrel margin is below what main
   proved.
3. **Speed.** The effective cap is now 1.0 m/s and 0.9 rad/s. No launch file used with Lane2027 starts the speed
   governor. If one does, `top_speed` must equal `vx_max` (main's note), or the turn rate drops with it.
4. **Wall layer on the tripod.** `costmap_wall` waits for a `map` frame. The tripod has none, so it logs a WARN every
   2 s and places no wall. This is harmless.
5. **Branch drift.** This branch lacks main's newer URDF, wheel and ros2_control commits (f85035d, 7c80739). Main's
   footprint was derived against those.
6. **CPU.** The global map is 100 x 100 m at 0.04 m (6.25 M cells), now with two more layers. Update rates on the
   laptop have not been measured.

## k. Next test card

1. On the tripod, run `tripod.launch` with Nav2 on Lane2027 (`single_cam.launch.py use_nav:=true` starts
   `basic_nav` with Lane2027). Check that every server comes up and that `/local_costmap/costmap` shows the taped
   lines. Record a bag with the README's bag command for 30 s.
2. That bag is the start of P1 (calibration) and P3 (projected lane points on real data).

## Verification (second agent, sonnet)

The verifier checked items 1 to 6 independently:
- **Diff against main:** only the stated LANE edits; the saved copy of main is byte-identical.
- **Lane layer:** identical in behaviour to the old one.
- **Pothole fix:** proven with a planner run.
- **Lattice path:** the substitution resolves through the real launch construct.
- **Integration check:** reproduced, with no errors. Its no-lane variant showed the planner really uses the lane marks.
- **Harness config:** matches Lane2027.
- **Lane memory:** reproduced from the saved runs. Nothing that was marked was ever erased.

It raised the issues in section j. Two of them, the behaviour tree and inflation, should be settled before the team
drives with Lane2027.
