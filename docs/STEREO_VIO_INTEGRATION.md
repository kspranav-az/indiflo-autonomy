# Stereo Visual-Inertial Odometry (VIO) Integration

This document describes the current stereo + IMU integration on the robot, the data flow, the coordinate frames, the bugs that were found and fixed, and the remaining limitations.

---

## 1. What we are building

A visual-inertial odometry pipeline that lets the robot estimate its own motion using:

- a **stereo camera pair** for visual feature tracking and depth estimation, and
- an **IMU** for high-rate inertial measurements.

The estimated odometry is published as `/unitree_go2/odom` and is consumed by `map_manager` to build an occupancy map. The long-term goal is to replace (or supplement) a physical LiDAR / RealSense with this stereo + IMU pair.

---

## 2. Sensors in use

| Sensor | Interface | ROS node | Output topics |
|--------|-----------|----------|---------------|
| Stereo cameras (2× CSI, IMX219) | `nvarguscamerasrc` GStreamer → OpenCV | `stereo_depth_node` | `/camera/left/image_raw`, `/stereo/right/image_raw`, `/stereo/depth`, `/stereo/left/color` |
| ICM-20948 IMU | I2C | `icm20948_node` | `/imu/data_raw` (`sensor_msgs/Imu`) |

### Stereo cameras
- The two cameras are physically separated by a baseline of about **60 mm**.
- The sensor natively runs at 1280×720, but the pipeline requests **640×480 @ 30 fps** and `nvvidconv` scales it down.
- `stereo_depth_node` processes depth at **320×240** (`process_scale: 0.5`) for speed.

### IMU
- The chip provides 3-axis accelerometer, 3-axis gyroscope, and a magnetometer.
- Only accelerometer and gyroscope are used by OpenVINS.
- The node runs on a 5 ms timer (200 Hz target), but I2C reads limit the real output rate to about **130 Hz**.
- The chip was left at its default gyro full-scale range of **±250 °/s** while the ROS scale factor assumed **±1000 °/s**. This was fixed by configuring the register (see Section 5).

---

## 3. Data flow and conversions

```text
Cameras --GStreamer--> OpenCV 640x480 frames
                          |
                          v
                    Rectification (stereo_calib.yml)
                          |
            +-------------+-------------+
            |                           |
            v                           v
    Raw left/right images          SGBM disparity
    (to OpenVINS)                       |
                                        v
                                  Depth image
                                  (32FC1 meters)
                                        |
                                        v
                                  map_manager
                                  occupancy map

IMU --I2C--> raw LSB values
              |
              v
        scaled to m/s² and rad/s
              |
              v
         /imu/data_raw
              |
              v
          OpenVINS
              |
              v
         /ov/odomimu  (remapped to /unitree_go2/odom)
```

### 3.1 Stereo depth

1. **Capture**: `nvarguscamerasrc` reads left (sensor id 1) and right (sensor id 0) cameras.
2. **Rectify**: OpenCV `initUndistortRectifyMap` + `remap` uses the calibration file `stereo_calib.yml`.
3. **Disparity**: SGBM computes disparity for each pixel.
4. **Depth**: For each valid disparity value `d`:

   ```text
   Z = (f_x * baseline) / d
   ```

   At processing resolution, `f_x ≈ 216` px and `baseline ≈ 0.06 m`, so:

   ```text
   d = 1   → Z ≈ 13 m
   d = 95  → Z ≈ 0.14 m
   ```

   The node clamps the output to the configured `depth_min_m = 0.2` and `depth_max_m = 10.0`.

### 3.2 IMU scaling

- **Accelerometer**: default ±2 g range, `1 g = 16384 LSB`. Scale factor is `9.80665 / 16384 ≈ 0.0005986 m/s²/LSB`.
- **Gyroscope**: the driver now configures **±1000 °/s** (`GYRO_CONFIG_1`, `FS_SEL = 2`). Scale factor is `(π/180) / 32.8 ≈ 0.000532 rad/s/LSB`.

### 3.3 OpenVINS VIO

OpenVINS subscribes to:
- `/camera/left/image_raw`
- `/stereo/right/image_raw`
- `/imu/data_raw`

It runs a **Multi-State Constraint Kalman Filter (MSCKF)** that:
1. Uses static initialization: while the platform is stationary it averages gravity to estimate the initial IMU orientation and IMU biases.
2. Integrates IMU measurements between camera frames.
3. Tracks visual features in the stereo pair and uses them to correct the IMU-predicted state.
4. Publishes odometry at the camera rate.

The odometry topic is remapped from `/ov/odomimu` to `/unitree_go2/odom` in the launch file.

### 3.4 Coordinate frames

| Frame | Convention | Notes |
|-------|------------|-------|
| IMU / body | `x = left`, `y = up`, `z = forward` (along camera optical axis, tilted together with the camera) | Determined from the physical arrow labels on the chip |
| Camera optical | `x = right`, `y = down`, `z = forward` (standard OpenCV / OpenVINS) | Defined by the rectified image |
| World / map | `z-up` (ENU-like) | Set by OpenVINS during initialization |

The transform `T_imu_cam` in `cam_chain.yaml` maps vectors from the IMU frame to the camera frame. For the current mounting it is a simple 180° rotation around the optical z-axis:

- camera `x` = `-IMU x` (right = -left)
- camera `y` = `-IMU y` (down = -up)
- camera `z` = `IMU z` (forward)

Because the camera module and the IMU are tilted together, no separate pitch correction is needed.

---

## 4. Issues found and fixed

### 4.1 IMU timestamps were wrong
**Symptom**: OpenVINS initialized once but then stopped responding to motion.  
**Cause**: The IMU node was using the last camera timestamp for every IMU sample, so all IMU messages after the first had the same timestamp (`dt ≈ 0`).  
**Fix**: Changed `icm20948_node.cpp` to use `this->now()` for each IMU and magnetometer sample.

### 4.2 Gyroscope scale was 4× too large
**Symptom**: Position exploded to hundreds of kilometers within seconds.  
**Cause**: The ICM-20948 resets to **±250 °/s** by default, but the ROS node scaled the raw readings as if the chip was in **±1000 °/s** mode.  
**Fix**: Added a register write in `icm20948_driver.cpp` to configure `GYRO_CONFIG_1` with `FS_SEL = 2` (±1000 °/s), matching the scale factor used in the node.

### 4.3 IMU-to-camera extrinsics were wrong
**Symptom**: Repeated kilometer-scale divergence even after the gyro fix.  
**Root causes**:
- The YAML originally gave `T_cam_imu`, but OpenVINS prints and uses `T_CtoI` (the inverse), so the rotation was inverted.
- The camera x-axis sign was wrong: the physical IMU x arrow points **left**, but earlier transforms assumed it pointed right.
- Earlier gravity-based transforms assumed the camera was horizontal while the IMU was tilted; the actual module has the camera and IMU tilted together.

**Fix**: Switched to `T_imu_cam` in `cam_chain.yaml` and set it to a 180° rotation around the optical z-axis (`diag(-1, -1, 1)`), matching the physical mounting where `camera x = -IMU x`, `camera y = -IMU y`, and `camera z = IMU z`. Online calibration of camera extrinsics/intrinsics/timeoffset was disabled until the initial guess is close enough.

### 4.4 Stereo calibration quality is poor
**Status**: still open.  
**Issue**: The current calibration has an RMS reprojection error of **~7.6 px**. For good VIO this should be **< 0.5 px**.  
**Effect**: OpenVINS can initialize and track short motions, but long-term accuracy will drift.

---

## 5. Current status

- OpenVINS now **initializes and tracks motion**.
- Position updates in the meter range when the robot moves.
- The occupancy map builds a wedge from the first depth frame but stops updating once odometry drifts too far.
- Depth output is valid for **~50% of pixels** at 320×240.

### Next steps

1. **Verify 1-meter tracking accuracy**:
   - Start at the origin.
   - Move the robot exactly 1 m forward.
   - Read `/unitree_go2/odom/pose/pose/position`.
   - Move back and read again.
   - If drift is < ~20 cm over 1 m, the pipeline is usable.

2. **Increase occupancy-map range** from 5 m to 10 m if desired by changing `pcdMaxZ` and `raycastMaxLength` in `map_param.yaml`.

3. **Recalibrate stereo** with a larger, well-lit chessboard, many poses, and the board filling ~1/3 of the frame to get RMS < 0.5 px. This is the biggest remaining accuracy improvement.

4. **Optional**: run a full IMU-camera calibration tool such as Kalibr to refine `T_imu_cam` further.

---

## 6. Useful commands

```bash
# Launch the full stack
cd /workspaces/ros2_ws
source install/setup.bash
ros2 launch stereo_depth_ros2 stereo_vio_mapping.launch.py

# Check odometry
ros2 topic echo /unitree_go2/odom/pose/pose/position --once
ros2 topic hz /unitree_go2/odom

# Check IMU axes
python3 /workspaces/ros2_ws/check_imu_axes.py

# Plot IMU accelerations
ros2 run rqt_plot rqt_plot /imu/data_raw/linear_acceleration/x /imu/data_raw/linear_acceleration/y /imu/data_raw/linear_acceleration/z
```
