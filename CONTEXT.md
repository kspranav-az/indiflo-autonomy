# Context for New Chat Session

## Current Focus (continue here)

You are working on the **stereo camera depth estimation stack + OpenVINS VIO**. The core algorithm package is at `src/stereo_test/`; the new ROS 2 bridge package is at `src/stereo_depth_ros2/`. The immediate goal is stable visual-inertial odometry from the stereo + IMU pair so the navigation stack can run without a physical LiDAR or RealSense.

The user last asked you to **fix OpenVINS divergence and commit the changes**. The latest state is below.

---

## Stereo Camera Hardware

| Parameter | Value |
|---|---|
| Left camera | `nvarguscamerasrc sensor-id=1` |
| Right camera | `nvarguscamerasrc sensor-id=0` |
| Resolution run | 640×480 |
| Stereo processing | 320×240 |
| Focal length | 2.6 mm |
| HFOV | 73° → fx ≈ 433 px at 640×480, fx ≈ 216 px at 320×240 |
| Distortion | < 1% (treated as zero) |
| Baseline | 60 mm |

---

## Current Working Pipeline

```
Calibration:     ./calibrate_offline 640 480 7 9 20  →  stereo_calib.yml
Depth viewer:    ./stereo_sgbm_fast                 →  Left/Right/Depth windows
ROS 2 publisher: ros2 launch stereo_depth_ros2 stereo_depth.launch.py
                 →  /stereo/depth, /stereo/left/color, /camera/left/image_raw
```

The latest `stereo_calib.yml` was generated with:
- Fixed intrinsics from lens specs (fx=433 px, distortion=0)
- `stereoCalibrate(..., CALIB_FIX_INTRINSIC)` to solve only R and T
- Baseline forced to exactly **60 mm**
- Reprojection RMS ≈ **7.6 px** (usable but improvable)

---

## What Already Works

1. ✅ Both cameras open reliably via GStreamer with `appsink max-buffers=1 drop=true`
2. ✅ 500 ms stagger prevents `nvargus-daemon` race on dual open
3. ✅ `fix_camera.sh` recovers hung daemon (`kill -9` + systemd restart)
4. ✅ `sigaction()` signal handlers prevent segfault on Ctrl+C cleanup
5. ✅ Left/Right camera assignment is consistent across all programs
6. ✅ Calibration detects 7×9 chessboard and saves 15 pairs to `calib_images/`
7. ✅ Rectification maps are valid (full 640×480 ROI, scanlines align)
8. ✅ `stereo_sgbm_fast` loads calibration and rectifies frames
9. ✅ SGBM finds matches on textured targets; depth is metrically scaled
10. ✅ New `stereo_depth_ros2` package wraps the pipeline in an `rclcpp` node
11. ✅ Publishes `/stereo/depth` (`TYPE_32FC1` meters), `/stereo/left/color` (`bgr8`), and `/camera/left/image_raw`
12. ✅ Downstream configs in `navigation_runner/cfg/` remapped to stereo topics and 320×240 intrinsics
13. ✅ `fake_odom_node` publishes `nav_msgs/Odometry` on `/unitree_go2/odom`
14. ✅ `map_manager` consumes `/stereo/depth` + `/unitree_go2/odom` and publishes occupancy map topics
15. ✅ OpenVINS (`ov_msckf`) built with local Ceres solver
16. ✅ OpenVINS config + launch for stereo + IMU; publishes `/ov/odomimu`, remapped to `/unitree_go2/odom`
17. ✅ Combined `stereo_vio_mapping.launch.py` launches stereo + IMU + OpenVINS + `map_manager` + RViz

---

## Known Problems (in priority order)

1. **VIO diverges to kilometers**: The IMU-to-camera rotation was wrong. After fixing the `T_cam_imu` inversion, the next run still diverged because the actual chip orientation is **y-up, x-left, z-forward**, not the originally assumed x-forward/y-left/z-up. The `cam_chain.yaml` has been updated to the new rotation.
2. **IMU/camera module tilt**: Stationary IMU reads `y ≈ +9.4` and `z ≈ +2.15`, so the module is pitched ~13° from vertical. The current transform assumes the camera optical axis is aligned with IMU z (i.e., camera is also pitched). If the camera is actually horizontal while the IMU is tilted on the board, the transform still needs a pitch correction.
3. **OpenVINS build is heavy**: required building Ceres from source and disabling OpenVINS test executables to fit in Jetson memory.
4. **Stereo calibration RMS = 7.6 px**: will degrade VIO until recalibrated.
5. **Depth is mostly black / sparse on low-texture scenes** (person against plain wall)
   - Block matching needs texture; plain walls and smooth skin have almost none.
   - Left/Right auto-exposure differs, which hurts SGBM correlation.
6. **Depth visualization is inverted** in rqt/RViz: close objects are dark, far objects are bright (raw `32FC1` meters mapped directly). This is correct for downstream nodes but confusing for humans. A `/stereo/depth/display` topic with close=white / far=black or a color map (like `stereo_sgbm_fast`) should be added later.
7. **No post-processing filter** — holes are not filled (WLS filter not implemented).

---

## Most Likely Next Steps

1. **Rerun VIO with the new y-up transform** (immediate):
   - Launch `stereo_vio_mapping.launch.py`.
   - Gently translate/rotate the robot/camera until OpenVINS initializes.
   - Watch `/ov/odomimu` (remapped to `/unitree_go2/odom`) for smooth tracking vs divergence.
   - If it still diverges to large coordinates, verify the IMU mounting orientation first.
2. **Verify IMU axes if divergence persists**:
   - Run `python3 /workspaces/ros2_ws/check_imu_axes.py`.
   - Hold still for 2 s, then push forward / left and note which bars spike positive.
   - The current config assumes: up = +y, forward = +z, left = +x.
3. **Recalibrate stereo** (biggest quality gain):
   - Retake 15–20 pairs with board filling ~1/3 of frame
   - More angles/distances; hold board perfectly flat
   - Verify square size with ruler (currently assumes 20 mm)
   - Then rerun `./calibrate_offline 640 480 7 9 <actual_mm>` and copy the new file to `src/stereo_depth_ros2/cfg/stereo_calib.yml` and `src/stereo_depth_ros2/config/openvins/`
4. **Tune OpenVINS IMU noise values** using Allan variance plots from a stationary IMU recording.
5. **Improve depth fill on low texture**:
   - Add a WLS (Weighted Least Squares) filter after SGBM
   - Or test larger block size / different SGBM mode (`MODE_SGBM_3WAY`)
   - Or normalize brightness more aggressively between L/R
6. **Evaluate moving to CUDA StereoSGM** (`cv::cuda::StereoSGM`) for faster + better results on Jetson
7. **Add a human-friendly depth display topic** (`/stereo/depth/display` or `/stereo/depth/color`) with inverted grayscale or JET color map so close=white/far=black, matching `stereo_sgbm_fast`.

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
./calibrate_stereo 640 480 30 7 9 20
# SPACE = save pair (both cameras must show green corners)
# C     = run calibration
# ESC   = quit

# --- Offline calibration from saved images ---
./calibrate_offline 640 480 7 9 20

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

# --- Launch full real-odometry mapping stack ---
ros2 launch stereo_depth_ros2 stereo_vio_mapping.launch.py

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
| `src/stereo_depth_ros2/config/openvins/estimator_config.yaml` | OpenVINS estimator parameters |
| `src/stereo_depth_ros2/config/openvins/cam_chain.yaml` | OpenVINS camera intrinsics / extrinsics |
| `src/stereo_depth_ros2/config/openvins/imu_chain.yaml` | OpenVINS IMU noise parameters |
| `src/open_vins/` | Cloned OpenVINS source |
| `src/ceres-solver/` | Cloned Ceres source (local build) |
| `src/ros2/map_manager/cfg/map_param.yaml` | `map_manager` config wired to `/stereo/depth` |
| `src/ros2/navigation_runner/cfg/map_param.yaml` | Same config, used by `navigation_runner` launch |
| `src/ros2/navigation_runner/cfg/dynamic_detector_param.yaml` | `onboard_detector` config wired to stereo topics |
| `src/ros2/onboard_detector/cfg/dynamic_detector_param.yaml` | Same config, used by standalone launch |
| `src/icm20948_ros2/src/icm20948_node.cpp` | IMU publisher (separate git repo; fixed timestamp bug) |

---

## Git State

- Top-level workspace is a git repo (`/workspaces/ros2_ws`). Committed files: `stereo_depth_ros2`, downstream YAML remaps, `CONTEXT.md`, `.gitignore`.
- `src/icm20948_ros2/` is a **separate git repo**. The IMU timestamp fix is committed there separately.
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
