# Context for New Chat Session

## Current Focus (continue here)

You are working on the **stereo camera depth estimation stack + OpenVINS VIO**. The core algorithm package is at `src/stereo_test/`; the new ROS 2 bridge package is at `src/stereo_depth_ros2/`. The immediate goal is stable visual-inertial odometry from the stereo + IMU pair so the navigation stack can run without a physical LiDAR or RealSense.

**Current status (2026-07-04):** VIO is now **stable at slow / handheld speed with minimal drift**, and the **navigation stack is wired end-to-end**. The stereo pair was recalibrated with a 7×5 / 29.0 mm checkerboard, bringing RMS from ~7.6 px down to **~1.04 px** and the spurious ~11° cam0→cam1 rotation down to **<0.3°**. The depth node now captures from the native 4:3 sensor mode (1640×1232) and scales to 640×480 for processing, which avoids the previous 16:9 distortion. ZUPT remains enabled (`try_zupt: true`) so the filter initializes while held still and clamps velocity / accel bias at rest. The `vio_watchdog.py` node monitors odometry, logs concise divergence/heartbeat lines to `/tmp/vio_diagnostics.log`, and serves `/vio/reset` + `/vio/clear_log`. A `map -> global` static TF was added so RViz/map see cam0/cam1/imu, and the occupancy-map depth intrinsics / IMU-to-depth transform were corrected.

The **navigation stack integration** is now complete: `map_manager`, `onboard_detector` (dynamic + YOLO), `safe_action_node`, and `navigation_node` configs are all synchronized to the same stereo calibration and transforms. A single launch file, `stereo_vio_navigation.launch.py`, brings up cameras + IMU + VIO + mapping + dynamic obstacle detection + safe-action checking + navigation + watchdog + RViz. YOLO is disabled by default because `torchvision` is not compatible with the Jetson-optimized PyTorch build; navigation is enabled by default because `torchrl` has been installed successfully.

**Remaining limitation:** drift is now very small but still visible when the device is lifted or moved. When kept perfectly still, ZUPT holds the origin; under motion the residual drift is likely a mix of camera-IMU time-sync, IMU noise parameters, and the remaining calibration/rolling-shutter limits. **Next step: squeeze out the last motion drift via time-sync / IMU tuning / better stationary calibration data**, not another full recalibration.

---

## Stereo Camera Hardware

| Parameter | Value |
|---|---|
| Left camera | `nvarguscamerasrc sensor-id=1` |
| Right camera | `nvarguscamerasrc sensor-id=0` |
| Native sensor mode | 1640×1232 (4:3, mode 3) |
| Capture resolution | 640×480 (scaled from native 4:3) |
| Stereo processing | 320×240 |
| Focal length | 2.6 mm |
| HFOV | 73° → fx ≈ 415–420 px at 640×480, fx ≈ 208–210 px at 320×240 |
| Distortion | calibrated radtan (now included) |
| Baseline | ~61 mm (calibrated) |

---

## Current Working Pipeline

```
Calibration:     ./calibrate_offline 640 480 7 5 29.0  →  stereo_calib.yml
Depth viewer:    ./stereo_sgbm_fast                 →  Left/Right/Depth windows
ROS 2 publisher: ros2 launch stereo_depth_ros2 stereo_depth.launch.py
                 →  /stereo/depth, /stereo/left/color, /camera/left/image_raw
```

The latest `stereo_calib.yml` was generated with:
- Checkerboard: **7×5 internal corners**, **29.0 mm** squares
- Native 4:3 sensor capture (1640×1232) scaled to 640×480 for calibration
- Full intrinsic + distortion + extrinsic solve (not `CALIB_FIX_INTRINSIC`)
- Baseline calibrated to **~60.97 mm**
- Reprojection RMS ≈ **1.04 px** (down from ~7.6 px)
- cam0→cam1 rotation **<0.3°** (down from ~11°)

These values were copied into `src/stereo_depth_ros2/cfg/stereo_calib.yml` and `src/stereo_depth_ros2/config/openvins/cam_chain.yaml`.

---

## What Already Works

1. ✅ Both cameras open reliably via GStreamer with `appsink max-buffers=1 drop=true`
2. ✅ 500 ms stagger prevents `nvargus-daemon` race on dual open
3. ✅ `fix_camera.sh` recovers hung daemon (`kill -9` + systemd restart)
4. ✅ `sigaction()` signal handlers prevent segfault on Ctrl+C cleanup
5. ✅ Left/Right camera assignment is consistent across all programs
6. ✅ Calibration detects 7×5 chessboard and saves 15 pairs to `calib_images/`
7. ✅ Rectification maps are valid (full 640×480 ROI, scanlines align)
8. ✅ `stereo_sgbm_fast` loads calibration and rectifies frames
9. ✅ SGBM finds matches on textured targets; depth is metrically scaled
10. ✅ New `stereo_depth_ros2` package wraps the pipeline in an `rclcpp` node
11. ✅ Publishes `/stereo/depth` (`TYPE_32FC1` meters), `/stereo/depth/color` (JET color), `/stereo/left/color` (`bgr8`), and `/camera/left/image_raw`
12. ✅ Downstream configs in `navigation_runner/cfg/` and `onboard_detector/cfg/` remapped to stereo topics and 320×240 intrinsics
13. ✅ `fake_odom_node` publishes `nav_msgs/Odometry` on `/unitree_go2/odom`
14. ✅ `map_manager` consumes `/stereo/depth` + `/unitree_go2/odom` and publishes occupancy map topics (range now 10 m)
15. ✅ OpenVINS (`ov_msckf`) built with local Ceres solver
16. ✅ OpenVINS config + launch for stereo + IMU; publishes `/ov/odomimu`, remapped to `/unitree_go2/odom`
17. ✅ Combined `stereo_vio_mapping.launch.py` launches stereo + IMU + OpenVINS + `map_manager` + RViz + VIO watchdog
18. ✅ `stereo_vio_navigation.launch.py` launches the full stack: stereo + IMU + OpenVINS + `map_manager` + `dynamic_detector` + YOLO + `safe_action_node` + `navigation_node` + RViz + VIO watchdog
19. ✅ RViz config updated with camera/depth/VIO image displays and odometry trail
20. ✅ `vio_watchdog.py` node monitors `/unitree_go2/odom`, flags divergence, publishes `/vio/status`, republishes origin-relative `/vio/odom_local` + `/vio/pose`, writes concise diagnostics to `/tmp/vio_diagnostics.log` (auto-rotated to 2000 lines), and serves `/vio/reset` + `/vio/clear_log`. Detection is tuned to avoid false positives: **absolute position > 50 m** trips instantly; the velocity check requires **sustained** high velocity (5 consecutive frames) so a single-frame filter/init correction no longer alarms; a warmup skips the init transient; a throttled **HEARTBEAT** line logs current/peak distance so healthy tracking is visible from the log alone; and it auto-logs **RECOVERED**. Verified end-to-end (lone spike ignored, sustained runaway flagged, heartbeat + reset work).

### Key finding from the 2026-07-04 run
After recalibration (RMS ~1.04 px, cam0→cam1 rotation <0.3°) the filter now tracks with **sub-meter drift** over slow/handheld motion and no longer produces the catastrophic km-scale runaway. Holding the device still keeps `dist ≈ 0.01 m`; lifting or translating it introduces only small residual drift. The watchdog velocity false-positive was already fixed (sustained-velocity gate), so the remaining drift is real tracking error rather than an alarm artifact. The quality ceiling has shifted from calibration to camera-IMU time-sync and IMU noise/tuning.

### Initialization behavior (important — verified from OpenVINS source)
`InertialInitializer.cpp` + `VioManagerHelper.cpp`: `wait_for_jerk = (updaterZUPT == nullptr)`.
- With `try_zupt: false`, `wait_for_jerk=true`, so **static init only fires on a "jerk" = stationary window THEN a moving window**. Holding perfectly still forever prints `[init]: failed static init: no accel jerk detected` and **never initializes**.
- **Fix applied:** set `try_zupt: true`. This flips `wait_for_jerk=false` so the filter **initializes while held still**, and adds zero-velocity updates that curb drift during pauses.
- Alternative not taken: `init_dyn_use: true` (init while moving) — riskier with the current bad calibration since dynamic init triangulates features to bootstrap.
- **How to know init happened:** the OpenVINS log prints `[init]: successful initialization`, then `q_GtoI=... p_IinG=... dist=...` lines begin, and the watchdog starts logging `HEARTBEAT status=OK`.

### TF frames (why RViz showed "no mapping ... to map")
OpenVINS publishes its tree under **`global`**: `global -> imu` (odom + tf) and `imu -> cam0`, `imu -> cam1` (calibration tf). RViz/`map_manager` fixed frame is **`map`**. Nothing linked them. **Fix applied:** `stereo_vio_mapping.launch.py` now launches a `static_transform_publisher` for identity **`map -> global`**, so cam0/cam1/imu resolve into `map`. (Note: while VIO is diverged the tf is still garbage because `imu` sits thousands of metres from `global`.)

### ROOT CAUSE of divergence (resolved by 2026-07-04 recalibration)
`config/openvins/cam_chain.yaml` originally encoded a spurious **~11.2°** cam0→cam1 rotation and idealized lens-spec intrinsics with zero distortion. That bad geometry wrecked epipolar constraints, caused stereo matches to be rejected, and let the filter coast on IMU until it ran away quadratically. **Fix applied:** full stereo recalibration with a 7×5 / 29.0 mm checkerboard, producing RMS ~1.04 px and a cam0→cam1 rotation <0.3°. The new intrinsics, distortion coefficients, and extrinsics were propagated to both `stereo_calib.yml` and `cam_chain.yaml`. The km-scale divergence is gone; only small motion drift remains.

### ZUPT result (validated 2026-07-04)
With `try_zupt: true` and the new calibration, the filter initializes while held still, holds `dist ≈ 0.01–0.05 m` with `|v_IinG| ≈ 0.002` while stationary (ZUPT accepting on `disparity < 0.5`), keeps `ba` bounded, and tracks slowly to ~1 m with only small residual drift. ZUPT remains the decisive fix for init + stationary drift + accel-bias observability. **Residual drift now appears mainly during deliberate motion / lifting**, which ZUPT cannot suppress; the next levers are camera-IMU time-sync and IMU noise/tuning. The `map->global` static TF is visualization-only and does not affect tracking.

### What ZUPT is and why it matters here
ZUPT = Zero-velocity UPdaTe: when the system detects it is stationary (OpenVINS uses image `disparity < zupt_max_disparity` = 0.5 px), it injects a pseudo-measurement "velocity = 0". Effects: (1) kills velocity drift at rest; (2) makes the **accelerometer bias observable** (at true zero velocity the accel reads only gravity, so residual = bias) — a wrong unbounded `ba` was what double-integrated into the runaway; (3) in OpenVINS it flips `wait_for_jerk=false`, enabling init-while-still. **ZUPT only acts when nearly still** — it is a rest-time safety net, not a motion fix.

### Path to higher stability / high-speed VIO (roadmap, in priority order)
ZUPT is irrelevant at speed; further accuracy depends on the visual-inertial core.
1. ✅ **Stereo recalibration — DONE.** RMS reduced from ~7.6 px to ~1.04 px; cam0→cam1 rotation from ~11° to <0.3°. This removed the catastrophic divergence and reduced motion drift dramatically.
2. **Camera-IMU time sync (next priority for residual motion drift).** `timeshift_cam_imu = 0` today; at speed a few-ms offset = large error. Calibrate the offset (e.g. Kalibr) and/or enable `calib_cam_timeoffset`.
3. **IMU noise / bias tuning.** The current `imu_chain.yaml` values are hand-tuned. A stationary recording analyzed with `imu_utils` or Kalibr would give proper Allan-variance noise and random-walk parameters, which should reduce the slow drift seen when lifting the device.
4. **Motion blur / rolling shutter — likely hardware ceiling.** Need short exposure (more light/gain). If these CSI sensors are rolling-shutter (probable), fast motion warps geometry; OpenVINS assumes global shutter → genuinely fast motion wants global-shutter cameras.
5. **Higher frame rate + tracking rate.** 30 fps loses KLT tracks when features jump far between frames; 60–120 fps keeps per-frame motion small, but needs compute headroom (the `[TIME]` lines already run several ms behind → consider CUDA tracking / lower tracking resolution).
6. **IMU headroom + filter tuning.** Confirm gyro/accel full-scale ranges don't saturate; then raise process noise and check chi2 gates aren't rejecting good features during fast motion.
Sequence: time-sync → IMU noise tuning → then push speed and find where hardware (shutter/fps/exposure) taps out.

### Stereo calibration prep (completed 2026-07-04; kept for reference)
The working calibration used a **7×5 internal-corner** checkerboard with **29.0 mm** squares. If you recalibrate in the future, use the same pattern and square size for consistency.
- A checkerboard generator script is included: `src/stereo_test/scripts/generate_checkerboard.py`.
- **Pattern used:** 7 rows × 5 cols internal corners, 29.0 mm squares.
- Generate / print command:
  ```bash
  cd /workspaces/ros2_ws
  python3 src/stereo_test/scripts/generate_checkerboard.py 7 5 \
      --square_mm 29.0 --output src/stereo_test/checkerboard_a4.svg
  ```
- Open `checkerboard_a4.svg` and **print at 100% / actual size** (disable fit-to-page).
- Mount the printout on a **flat, rigid backing** (cardboard/acrylic/foam board); matte finish preferred.
- **Measure one square with a ruler** and update the square size in the command below if it differs from 29.0 mm.

### Calibration capture procedure
```bash
cd /workspaces/ros2_ws/src/stereo_test/build
./calibrate_stereo 640 480 30 5 7 29.0
```
In the capture window:
- Show the board to **both cameras at the same time**.
- Press **SPACE** only when **both left and right views show green corners**.
- Capture **15–25 pairs** with the board filling roughly **1/3 to 1/2** of the image.
- Vary: distance, tilt angle, roll/pitch, and board position (center, corners, edges).
- Hold steady while saving to avoid motion blur.
- Press **C** to run calibration when done.

### Offline calibration (if images already saved)
```bash
cd /workspaces/ros2_ws/src/stereo_test/build
./calibrate_offline 640 480 5 7 29.0
```

### After a good calibration
Target RMS **< 0.5 px**, with a cam0→cam1 rotation well under 1° and baseline close to 61 mm. The 2026-07-04 calibration achieved ~1.04 px RMS and <0.3° rotation. Propagate new values:
```bash
cp src/stereo_test/build/stereo_calib.yml src/stereo_depth_ros2/cfg/stereo_calib.yml
# Manually update src/stereo_depth_ros2/config/openvins/cam_chain.yaml
# with the new intrinsics, distortion, and cam0->cam1 T_cn_cnm1 transform.
```

---

## Known Problems (in priority order)

1. **Small residual drift while lifting / moving the device** (remaining quality ceiling): with the device perfectly still, ZUPT holds the origin. When lifted or translated, a small drift is still visible. This is no longer caused by calibration (RMS ~1.04 px, rotation <0.3°). Likely contributors are:
   - **Camera-IMU time offset** still set to `timeshift_cam_imu = 0`; a few-ms misalignment becomes significant during motion.
   - **IMU noise/bias parameters** are hand-tuned; a stationary Allan-variance recording would improve them.
   - Possible **rolling-shutter / motion-blur** limits on these CSI sensors.
   **Next levers:** time-sync calibration → IMU noise tuning → only then push speed / exposure.
2. **No diagnostic logging / reset button**: ✅ **RESOLVED** by `vio_watchdog.py`. It detects divergence, writes concise lines to `/tmp/vio_diagnostics.log` instead of flooding the terminal, and offers `/vio/reset` (set a new local origin without restarting the stack) and `/vio/clear_log`. Note this **detects and mitigates** divergence — it does **not** fix the root cause (now addressed by recalibration, with residual motion drift handled by the items above).

---

## Most Likely Next Steps

1. ✅ **DONE — VIO watchdog** (`vio_watchdog.py`) and ✅ **DONE — ZUPT enabled** (init-while-still + stationary anti-drift, validated sub-meter tracking).
2. ✅ **DONE — Stereo recalibration** with 7×5 / 29.0 mm checkerboard. RMS ~1.04 px, cam0→cam1 rotation <0.3°, baseline ~61 mm. New intrinsics/distortion/extrinsics are in `stereo_calib.yml` and `cam_chain.yaml`.
3. ✅ **DONE — Navigation stack integration** (`stereo_vio_navigation.launch.py`). `map_manager`, `onboard_detector`, `safe_action_node`, and `navigation_node` configs are synchronized to the same stereo calibration and transforms.
4. **Camera-IMU time-offset calibration** (top priority for residual motion drift): set `timeshift_cam_imu` / enable `calib_cam_timeoffset`. Even a few ms matters once the device moves.
5. **IMU noise / bias tuning** from a stationary recording: collect ~30 min of stationary IMU data, run `imu_utils` or Kalibr Allan-variance, and update `src/stereo_depth_ros2/config/openvins/imu_chain.yaml`. This should reduce the slow drift when lifting the device.
6. **Closed-loop navigation test:** launch `stereo_vio_navigation.launch.py`, publish a `/goal_pose`, and verify the robot plans and moves toward it without crashes. Start with the robot on the ground and a nearby goal.
7. **Verify 1-meter accuracy:** move exactly 1 m out and back, check `/unitree_go2/odom` position; expect a few-cm error once time-sync + IMU tuning are in place.
8. **For higher speed:** see the "Path to higher stability / high-speed VIO" roadmap above (fps, exposure/shutter, compute).
9. **Depth-fill / quality (optional):** WLS filter after SGBM, or `cv::cuda::StereoSGM` on the Jetson.

---

## Quick Command Reference

```bash
# --- Build all stereo tools ---
cd /workspaces/ros2_ws/src/stereo_test/build
cmake .. && make -j$(nproc)

# --- Build stereo ROS 2 package ---
cd /workspaces/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stereo_depth_ros2

# --- Recover hung cameras ---
./fix_camera.sh

# --- Live calibration capture ---
./calibrate_stereo 640 480 30 5 7 29.0
# SPACE = save pair (both cameras must show green corners)
# C     = run calibration
# ESC   = quit

# --- Offline calibration from saved images ---
./calibrate_offline 640 480 5 7 29.0

# --- Run rectified depth viewer ---
./stereo_sgbm_fast
# L = log/linear depth scaling
# I = toggle HUD
# D = toggle depth / raw disparity view
# SPACE = save screenshot + depth XML

# --- Launch ROS 2 stereo depth publisher ---
cd /workspaces/ros2_ws
source install/setup.bash
ros2 launch stereo_depth_ros2 stereo_depth.launch.py

# --- Launch stereo depth + fake odometry together ---
ros2 launch stereo_depth_ros2 stereo_depth_with_fake_odom.launch.py

# --- Launch fake odometry only (if stereo is already running) ---
ros2 launch stereo_depth_ros2 fake_odom.launch.py

# --- Launch map_manager with stereo depth ---
ros2 launch map_manager occupancy_map.launch.py

# --- Launch RViz with the saved map_manager config ---
ros2 launch map_manager rviz.launch.py
# In RViz set Fixed Frame to 'map' to see the voxel/2D maps.

# --- One-launch stereo perception stack (stereo + fake odom + map_manager + RViz) ---
ros2 launch stereo_depth_ros2 stereo_perception.launch.py

# --- Launch stereo + IMU + OpenVINS VIO ---
# (move the camera gently after launch to initialize)
ros2 launch stereo_depth_ros2 openvins.launch.py

# --- Launch full real-odometry mapping stack (now includes VIO watchdog) ---
ros2 launch stereo_depth_ros2 stereo_vio_mapping.launch.py

# --- Launch full navigation stack (stereo + IMU + VIO + mapping + obstacle detection + navigation) ---
# Default: navigation ON, YOLO OFF (torchvision is not available on this Jetson torch build).
# Add `use_yolo:=true` only if you have built/installed a Jetson-compatible torchvision.
ros2 launch stereo_depth_ros2 stereo_vio_navigation.launch.py

# --- Run only VIO + mapping (disable navigation for debugging) ---
ros2 launch stereo_depth_ros2 stereo_vio_navigation.launch.py use_navigation:=false

# --- Run with YOLO (requires torchvision) ---
ros2 launch stereo_depth_ros2 stereo_vio_navigation.launch.py use_yolo:=true

# --- Publish a navigation goal (after the stack is running and VIO is initialized) ---
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' --once

# --- Emergency stop the navigation node ---
ros2 topic pub /navigation_emergency_stop std_msgs/Bool '{data: true}' --once

# --- VIO watchdog: monitor status, read diagnostics, reset origin ---
ros2 topic echo /vio/status                 # OK / DIVERGED
tail -f /tmp/vio_diagnostics.log            # concise divergence + reset log
ros2 service call /vio/reset std_srvs/srv/Trigger        # set new local origin
ros2 service call /vio/clear_log std_srvs/srv/Trigger    # truncate the log

# --- Check that all expected nodes are up ---
ros2 node list | grep -E "stereo_depth|icm20948|ov_msckf|map_manager|dynamic_detector|safe_action|navigation_node|vio_watchdog"

# --- Check navigation stack topic/service wiring ---
ros2 topic list | grep -E "stereo|occupancy|odom|goal_pose|cmd_vel|yolo|dynamic"
ros2 service list | grep -E "raycast|get_dynamic_obstacles|get_safe_action"

# --- Verify published topics ---
ros2 topic list | grep -E "stereo|occupancy|odom|ov|imu"
ros2 topic hz /stereo/depth
ros2 topic hz /stereo/left/color
ros2 topic hz /occupancy_map/occupancy_map_2D
ros2 topic hz /occupancy_map/voxel_map
ros2 topic hz /ov/odomimu
ros2 topic hz /imu/data_raw
```

---

## File Map

| File | What it does |
|---|---|
| `src/stereo_test/CMakeLists.txt` | Builds all standalone executables |
| `src/stereo_test/calibrate_stereo.cpp` | Live calibration capture tool |
| `src/stereo_test/calibrate_offline.cpp` | Offline calibration from `calib_images/` |
| `src/stereo_test/stereo_sgbm_fast.cpp` | **Main depth viewer** — loads calibration, rectifies, SGBM |
| `src/stereo_test/diag_board_size.cpp` | Tests a saved image to find correct chessboard size |
| `src/stereo_test/fix_camera.sh` | Hard-recovers hung `nvargus-daemon` |
| `src/stereo_test/build/stereo_calib.yml` | Current calibration file |
| `src/stereo_test/build/calib_images/` | Saved left/right calibration pairs |
| `src/stereo_depth_ros2/src/stereo_depth_node.cpp` | **ROS 2 stereo depth publisher node** |
| `src/stereo_depth_ros2/include/stereo_depth_ros2/stereo_depth_node.hpp` | Node class header |
| `src/stereo_depth_ros2/cfg/stereo_depth_node_param.yaml` | Node parameters |
| `src/stereo_depth_ros2/cfg/stereo_calib.yml` | Calibration copy used by the node |
| `src/stereo_depth_ros2/launch/stereo_depth.launch.py` | Launch file for the node |
| `src/stereo_depth_ros2/src/fake_odom_node.cpp` | Placeholder odometry publisher (replace with VIO/IMU later) |
| `src/stereo_depth_ros2/cfg/fake_odom_node_param.yaml` | Fake odometry parameters |
| `src/stereo_depth_ros2/launch/fake_odom.launch.py` | Launch fake odometry only |
| `src/stereo_depth_ros2/launch/stereo_depth_with_fake_odom.launch.py` | Launch stereo depth + fake odometry together |
| `src/stereo_depth_ros2/launch/stereo_perception.launch.py` | Launch stereo depth + fake odom + map_manager + RViz |
| `src/stereo_depth_ros2/launch/openvins.launch.py` | Launch stereo depth + OpenVINS VIO |
| `src/stereo_depth_ros2/launch/stereo_vio_mapping.launch.py` | Launch stereo + IMU + OpenVINS + map_manager + RViz |
| `src/stereo_depth_ros2/launch/stereo_vio_navigation.launch.py` | **Full navigation stack** — stereo + IMU + OpenVINS + map_manager + dynamic detector + YOLO + safe action + navigation + RViz |
| `src/stereo_depth_ros2/config/openvins/estimator_config.yaml` | OpenVINS estimator parameters |
| `src/stereo_depth_ros2/config/openvins/cam_chain.yaml` | OpenVINS camera intrinsics / extrinsics |
| `src/stereo_depth_ros2/config/openvins/imu_chain.yaml` | OpenVINS IMU noise parameters |
| `src/open_vins/` | Cloned OpenVINS source |
| `src/ceres-solver/` | Cloned Ceres source (local build) |
| `src/ros2/map_manager/cfg/map_param.yaml` | `map_manager` config wired to `/stereo/depth` (updated stereo intrinsics + IMU-to-depth transform) |
| `src/ros2/navigation_runner/cfg/map_param.yaml` | Same config, used by `navigation_runner` launch (now synchronized) |
| `src/ros2/navigation_runner/cfg/dynamic_detector_param.yaml` | `onboard_detector` config wired to stereo topics (now synchronized) |
| `src/ros2/onboard_detector/cfg/dynamic_detector_param.yaml` | Same config, used by standalone launch (now synchronized) |
| `src/ros2/navigation_runner/cfg/navigation_param.yaml` | Navigation node parameters (`odom_topic`, `cmd_topic`, velocity limit) |
| `src/ros2/navigation_runner/cfg/safe_action_param.yaml` | Safe-action node parameters (collision horizon, safety distance) |
| `src/icm20948_ros2/src/icm20948_node.cpp` | IMU publisher (separate git repo; fixed timestamp bug and gyro scale) |
| `src/stereo_depth_ros2/scripts/record_imu_stats.py` | Stationary IMU statistics recorder for OpenVINS tuning |
| `src/stereo_depth_ros2/scripts/vio_watchdog.py` | **VIO watchdog** — divergence detection, `/tmp/vio_diagnostics.log`, `/vio/status`, `/vio/reset`, `/vio/clear_log` |
| `src/stereo_test/scripts/generate_checkerboard.py` | **Generates A4 checkerboard SVG** for stereo calibration (configurable rows/cols/margin) |
| `src/stereo_test/checkerboard_a4.svg` | Generated checkerboard SVG; the 2026-07-04 calibration used a 7×5 / 29.0 mm board (regenerate with the command above if needed) |
| `src/stereo_test/checkerboard_a4.txt` | Sidecar with exact calibration commands for the generated pattern |

---

## Git State

- Top-level workspace is a git repo (`/workspaces/ros2_ws`). Committed files: `stereo_depth_ros2`, downstream YAML remaps, `CONTEXT.md`, `.gitignore`.
- `src/icm20948_ros2/` is a **separate git repo**. The IMU timestamp fix is committed there separately.
- `src/ros2/` is a **separate git repo / submodule** containing `map_manager`, `navigation_runner`, and `onboard_detector`. Changes to their configs must be committed inside `src/ros2/`, not from the top-level workspace.
- External dependencies (`src/open_vins/`, `src/ceres-solver/`) are **not** committed in the top-level repo.

---

## MiDaS / V4L2 Background (previous work, not current focus)

See the original notes below if you ever switch back to the `midas_trt_nvargus` package.

### What Was Done

1. **Abandoned GStreamer path** — `nvarguscamerasrc + appsink` fails silently on this JetPack. `pull_sample()` blocks forever. `nvvidconv` outputs DMA-BUF that `appsink` cannot map. `nvgstcapture-1.0` works because it uses display sinks, not `appsink`.
2. **Switched to raw V4L2 capture** — opens `/dev/video0`, requests `V4L2_PIX_FMT_SRGGB10`, mmap's one buffer, runs `DQBUF/QBUF` loop.
3. **Added OpenCV debayering** — `cv::cvtColor(bayer8, bgr, COLOR_BayerRG2BGR)` with `1.0/64.0` scale.
4. **Fixed resolution/framerate/encoding** — native 3280×2464, frame rate raised to ~21 Hz, encoding changed to `bgr8`.
5. **Added software white balance and CCM** — gray-world WB + aggressive linear CCM to fight green cast.

### What Still Does NOT Work

- Colors are still wrong (sensor-specific CCM needed).
- GStreamer ISP path remains blocked for appsink-based capture.
- Image is grainy (no ISP denoising).

### MiDaS Commands

```bash
# Build
cd /workspaces/ros2_ws && colcon build --symlink-install --packages-select midas_trt_nvargus

# Launch visualization
cd /workspaces/ros2_ws && source install/setup.bash && ros2 launch perception_viz sensor_viz.launch.py

# Check camera controls
v4l2-ctl -d /dev/video0 --list-ctrls

# Test GStreamer (will not work with appsink)
nvgstcapture-1.0 --sensor-id=0
```

### MiDaS Key Files

- `src/midas_trt_nvargus/src/midas_v4l2_node.cpp`
- `src/midas_trt_nvargus/src/cuda_preprocess.cu`
- `src/perception_viz/launch/sensor_viz.launch.py`
- `docs/CAMERA_CAPTURE.md`
