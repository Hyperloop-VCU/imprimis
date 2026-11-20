# Lane iteration 1: first tripod test, calibration (P1) (2026-11-15)

**Short version:** the camera's real tilt was 10.1–11.0°, against 11.2° given to `tripod.launch`. It also leaned 1–2°, and the
launch assumes 0°. That small error moved the far end of each line 0.04–0.50 m toward the camera in the cost map. The
projection code itself is right: with the angles it was given, it put the lane points on the tape to within about 5 cm.
On the ground the tape lines were not parallel: they were 1.2 m apart at the near end and 0.8 m at the far end. No code
or config was changed in this step.

Tool: `harness/calibrate_mount.py`. Numbers: `Replays/calibration_1.csv`. Figures: `Replays/calibration_*.png`.

## What was tested

Tested by the team on 2026-11-15, outdoors on a residential street (asphalt), overcast late afternoon.

**Setup**
- One D435i on a tripod, run with `tripod.launch.py tilt:=11.2 height:=0.686`.
- Lens height: 27 in.
- Tilt: 11.2°, measured with a phone level resting on the camera.
- Two tape lines. Each is two 48 in × 4 in paper strips taped end to end, so 2.438 m long. They were laid by eye and
  picked up afterwards, so their positions were not measured.
- Two cross strips, used in the `*_cross` bags.
- A standalone Lane2027 local cost map (`harness/configs/p1_lane2027_local.yaml`).

**Tripod spots**
- Spot 1.
- Spot 2: about 2 ft to the left of spot 1.
- Spot 3: about 2 ft behind spot 1.
- The tilt was not re-checked after moving.

**Bags in `RealBags/`, 31 s each**

| Spot | With cross strips | Lines only |
|---|---|---|
| 1 | `real_bag1` | `spot1_clean` |
| 2 | `spot2_cross` | `spot2_clean` |
| 3 | `spot3_cross` | `spot3_clean` |

**Problems during the test**
- The camera's accelerometer (`/cameras/d435i/accel/sample`) recorded 0 messages in every bag. Turning it on while the
  camera runs did not work, so `harness/imu_tilt.py` could not be used.
- The curb, gutter and black rubber mats beside the right line are bright or high-contrast, and segmentation picks them
  up.

## Method

On flat ground, the projected length of a line of known length depends on the camera's tilt and height: both ends of
the line move. The difference between the two lines' lengths depends on roll. `calibrate_mount.py` holds the height at
the TF value of 0.686 m and fits the tilt and roll that make both lines 2.438 m long.

- **Input:** a median of every 30th frame, so people walking past drop out.
- **Finding the lines:** white, unsaturated blobs within 8 m. It takes the two that span the most image rows.
  `Replays/calibration_lines_picked.png` shows the picks; they are correct in all three clean bags.
- **Yaw:** not fitted, because it changes no length.
- **Lane points check:** the tool compares `/perception/lane_points` with the tape lines projected with the launch
  angles, which are the ones `lane_projection` used.

The `*_cross` bags are not used for the result. A cross strip touches the lines, merges with their blobs and spoils
the roll. Their tilt is used only as a repeatability check.

## Results (`calibrate_mount.py`, clean bags)

| | spot1_clean | spot2_clean | spot3_clean |
|---|---|---|---|
| Fitted tilt (launch 11.2°) | **10.50°** | **10.99°** | **10.14°** |
| Fitted tilt if the lens were 2 in lower / higher | 9.96 / 11.03° | 10.45 / 11.53° | 9.66 / 10.61° |
| Fitted roll (launch 0°), right side low, ± about 0.5–1° (see Verification) | **+1.10°** | **+1.96°** | **+1.27°** |
| Line length at the launch angles (true 2.438 m), L / R | 2.21 / 2.24 m | 2.36 / 2.42 m | 2.08 / 2.10 m |
| Far end moved toward the camera at the launch angles, L / R | 0.31 / 0.23 m | 0.11 / 0.04 m | 0.50 / 0.41 m |
| Lines, fitted: ahead of the camera | 1.54–3.98 m | 1.47–3.93 m | 1.86–4.32 m |
| Gap between the lines, near / far end | 1.25 / 0.77 m | 1.22 / 0.75 m | 1.22 / 0.78 m |
| Angle between the lines | 11.4° | 11.0° | 10.4° |
| Lane points within 0.15 m of the tape | 66 % | 84 % | 75 % |
| Median offset of those points from the tape centreline | 4.5 cm | 4.7 cm | 4.1 cm |

**Repeatability** (cross bag against the clean bag at the same spot, tilt only)

| Spot | Cross bag | Clean bag | Difference |
|---|---|---|---|
| 1 | 10.44° | 10.50° | 0.06° |
| 2 | 11.05° | 10.99° | 0.06° |
| 3 | 10.22° | 10.14° | 0.08° |

**Consistency with the tripod moves** (fitted distance from the spot under the camera to each line; yaw does not
change it)
- **Spot 2:** the left line got 0.59 m closer and the right line 0.54 m farther, against the 0.61 m (2 ft) reported.
- **Spot 3:** the near ends moved 0.33 m farther away, against the 0.61 m reported. Either the tripod moved about 1 ft,
  or the tilt changed when it was moved. The bags cannot separate the two.

## What it means

1. **The phone reading was close, but not close enough at range.** It was 0.2–1.1° off in tilt and 1–2° off in roll.
   At 4 m that is up to half a metre along the line: the far end of a line lands short. P1's pass limit is 10 cm at
   4 m. A tilt error matters most where the costmap gets its early warning of a curve.
2. **Every move of the tripod changed the angle.** Spot 3's tilt is 0.4° below spot 1's. On the robot the mount is
   fixed, but this shows that a hand reading per setup is not enough. The angle has to be measured from the image, or
   by the IMU, each time.
3. **`lane_projection` is correct for the angles it is given.** The lane points that fall on the tape sit about 4–5 cm
   from the centreline. That is about the 5 cm skeleton cell. The error in the cost map comes from the mount angle,
   not from the projection code.
4. **About 16–34 % of the lane points are not on the tape.** They are mostly the curb, the gutter edge and the mats on
   the right. This is P2 (segmentation) work. A course without a concrete curb may not show it.
5. **The tape was converging (by about 11°), not parallel.** This is the same in every spot, so it is the layout, not
   the camera. With the lines only 0.8–1.25 m apart, this layout cannot show what a real 10 ft lane looks like in the
   cost map.
6. **Part of the line is never published.** At spot 3 the far ends are 4.3 m out, beyond `lane_projection`'s 4.0 m
   `max_range`, so they never reach the cost map.

## Limits of this check

- **Height not measured independently.** It was held at 0.686 m. Every 2 in of height error moves the fitted tilt by
  about 0.5°.
- **Flat ground assumed.** The street has a crown and a gutter slope.
- **No held-out pose, and no surveyed tape positions.** So the absolute ground error (P1's pass metric) cannot be
  scored yet. What this test gives is the angle error and how far it moves the lines.
- **The 1.5 cm offset of the colour sensor inside the D435i is ignored.**

## Recommended, not done (needs permission per file)

1. **`lane_segmentation.launch.py`:** pass `enable_accel` and `enable_gyro` to the RealSense driver, so the IMU runs
   from the start and is recorded. This gives a second tilt and roll measurement for every bag.

   **DONE (approved).** There is a new `enable_imu` argument, default `false`. The old file is in `OldFiles/`.

   Tested on the real camera: with the IMU on, the driver fails with "Permission denied" on the camera's IMU device
   (`/sys/.../iio:device1/scan_elements`), and then the whole camera fails to start, colour included. The same cause
   explains the 0 IMU messages in this test's bags. The fix is the RealSense udev rules (`99-realsense-libusb.rules`),
   which are not installed on the laptop. They need sudo, so the team installs them. The default stays `false` until
   they are in.
2. **`tripod.launch.py`:** add a `roll` argument, plus `x` and `yaw` as the README already plans. Without roll, the
   1–2° lean measured here cannot be corrected.
3. **README section 9:** `calibrate_mount.py` fits from two lines of known length, not from hand-picked corners. Update
   the table to match.

## Next test card

1. **Lay the two lines parallel and 10 ft (3.05 m) apart**, starting 6 ft ahead of the camera. Measure the gap at both
   ends before recording.
2. **Mark the spot under the camera** and measure where each line's ends are from it before picking anything up. That
   gives a true position for P1's error.
3. **Record lines-only bags** at three spots, plus one more spot that is not used for fitting (held out).
4. **If the IMU launch change is approved,** record `/cameras/d435i/accel/sample` too.
5. **Run `calibrate_mount.py`** on the bags, then restart `tripod.launch` with the fitted tilt (and roll, once there is
   an argument for it). Record once more to confirm the lines land on the tape.

## Verification (second agent, sonnet)

The verifier wrote its own scripts and did not change any repo file.

| Check | Verdict |
|---|---|
| Geometry code (axes, ground projection, roll sign, fit) | No math bugs found. |
| Re-running the tool on all three clean bags | Every number in the results table reproduced. |
| Synthetic images at known angles, with the verifier's own renderer | **Tilt** recovered within 0.03°. **Roll** is biased by 0.1–0.6°, up to 0.9° when the lines are off-centre (as at spot 2). |
| Spot 2 shift | 0.56 m left (and 0.07 m forward). Agrees with the reported 2 ft. |
| Spot 3 shift | About 0.33 m (13 in), not the reported 2 ft. |
| `lane_points` frame | It is `base_link`, which is the spot under the camera on the tripod, so the comparison is valid. |
| Thresholds | `WHITE_V` 170–210 changes the tilt by 0.01°. At 150, a line merges with the background and the roll fit fails (+37°). |

**Caveats it raised, which this report accepts**
1. **Roll is the weak output.** Read the fitted roll as 1–2° ± about 0.5–1°. The lean is probably real, but the
   difference between spots is within that error. Tilt is solid.
2. **Tilt trades off with height**, by about 0.5° per 2 in. The next test should measure the lens height carefully.
3. **Two equations, two unknowns.** The fit always matches exactly, so a zero residual is not a check.
4. **The fitted blobs are about 8 cm wide, against the 10.2 cm tape.** The threshold eats the edges. The effect on
   tilt is about 0.06°.
5. **The lane points check only covers out to 4.0 m.** Points beyond `max_range` are never published, so the on-tape
   fraction says nothing about the far ends.
6. **No safety margin on the brightness threshold.** In dimmer light, check `*_lines.jpg` every time. The tool now
   warns when the fitted roll is over 5°, which is the sign of a merged blob.
