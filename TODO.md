# VIO Stability Tuning — TODO

This file tracks the next phase of stereo VIO integration: camera-IMU time sync and IMU noise/bias tuning using Kalibr.

---

## Goal

Remove the small residual drift that appears when the device is lifted or moved.
At rest ZUPT already holds the origin; the remaining drift is mostly caused by:

1. Camera-IMU time offset (`timeshift_cam_imu = 0` today).
2. Hand-tuned IMU noise / random-walk parameters.

Kalibr will give calibrated values for both.

---

## Hardware / environment constraints

- Jetson (`aarch64`) runs the sensors and records bags.
- Kalibr is `x86_64`-only in its official Docker image, so computation must run on an x86 PC/VM.
- Kalibr only reads ROS 1 bags; ROS 2 bags must be converted with `rosbags-convert`.

---

## Preparation checklist

- [ ] Print an AprilGrid target (preferred over checkerboard for camera-IMU calibration).
  - Download/generate from: https://github.com/ethz-asl/kalibr/wiki/calibration-targets
  - Note the `tagSize` (meters) and `tagSpacing` (fraction of tagSize).
- [ ] Mount the AprilGrid on a flat, rigid backing (cardboard / acrylic / foam board).
- [ ] Confirm x86 PC/VM with Docker is available for Kalibr.

---

## Step 1 — Record stationary IMU bag (for Allan variance)

**Who:** operator (on Jetson)

Place the rig on a solid table. Do not touch it for **at least 30 minutes** (1 hour is better).

```bash
cd /workspaces/ros2_ws
source install/setup.bash
mkdir -p kalibr_data
ros2 bag record /imu/data_raw -o kalibr_data/imu_static
# wait 30+ min, then Ctrl+C
```

Output: `kalibr_data/imu_static/` (ROS 2 SQLite3 bag).

- [ ] Done

---

## Step 2 — Record camera-IMU motion bag

**Who:** operator (on Jetson)

Use the AprilGrid target. Move the rig slowly for **90–120 seconds**:

- Translate: left/right, up/down, forward/back.
- Rotate: around all three axes.
- Keep the AprilGrid in view as much as possible.
- Avoid motion blur and fast rotations.
- Do not shake.

```bash
ros2 bag record /camera/left/image_raw /stereo/right/image_raw /imu/data_raw -o kalibr_data/cam_imu
# move for ~90-120 s, then Ctrl+C
```

Notes:

- Verify `/imu/data_raw` is publishing at **200 Hz**.
- Camera frame rate can stay at 30 Hz; 20 Hz is also acceptable for Kalibr.

Output: `kalibr_data/cam_imu/` (ROS 2 SQLite3 bag).

- [ ] Done

---

## Step 3 — Convert ROS 2 bags to ROS 1 format

**Who:** operator (on Jetson)

```bash
pip3 install rosbags
cd /workspaces/ros2_ws/kalibr_data
rosbags-convert imu_static --dst imu_static.bag
rosbags-convert cam_imu --dst cam_imu.bag
```

Output: `imu_static.bag`, `cam_imu.bag`.

Transfer these two `.bag` files to the x86 PC/VM (`scp`, USB stick, etc.).

- [ ] Done

---

## Step 4 — Prepare Kalibr input files on x86 PC/VM

**Who:** operator, with config files supplied by Kimi

Create a working directory, e.g. `~/kalibr_run/`, and place inside:

1. `imu_static.bag`
2. `cam_imu.bag`
3. `target.yaml` — AprilGrid definition.
4. `imu.yaml` — initial IMU parameters.
5. `camchain.yaml` — stereo intrinsics/extrinsics.

Kimi will generate `target.yaml`, `imu.yaml`, and `camchain.yaml` from the current OpenVINS configs (`cam_chain.yaml`, `imu_chain.yaml`).

- [ ] Done

---

## Step 5 — Run Kalibr Docker on x86 PC/VM

**Who:** operator (on x86 PC/VM)

Pull or build Kalibr:

```bash
# Try prebuilt image first
docker pull stereolabs/kalibr:latest
# or
docker pull ghcr.io/ethz-asl/kalibr:latest

# If neither works, build from source:
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t kalibr -f Dockerfile .
```

### 5a — Allan variance (IMU noise parameters)

```bash
cd ~/kalibr_run
docker run --rm -v $(pwd):/data kalibr \
  kalibr_bagextractor --bag /data/imu_static.bag --output /data/allan_out

docker run --rm -v $(pwd):/data kalibr \
  kalibr_allan --data /data/allan_out/imu.csv --output /data/allan_out
```

Read the four values from the output:

- `accelerometer_noise_density`
- `accelerometer_random_walk`
- `gyroscope_noise_density`
- `gyroscope_random_walk`

- [ ] Done

### 5b — Camera-IMU calibration (time offset + extrinsics)

```bash
cd ~/kalibr_run
docker run --rm -v $(pwd):/data kalibr \
  kalibr_calibrate_imu_camera \
    --target /data/target.yaml \
    --cam /data/camchain.yaml \
    --imu /data/imu.yaml \
    --bag /data/cam_imu.bag \
    --bag-from-to 5 115
```

Output: `camchain-imucam.yaml`. Read:

- `timeshift_cam_imu` for each camera.

- [ ] Done

---

## Step 6 — Apply Kalibr outputs to OpenVINS configs

**Who:** Kimi

Files to update:

- `src/stereo_depth_ros2/config/openvins/cam_chain.yaml`
  - Set `timeshift_cam_imu` to the Kalibr value.
- `src/stereo_depth_ros2/config/openvins/imu_chain.yaml`
  - Update the four Allan-variance noise parameters.
- `src/stereo_depth_ros2/config/openvins/estimator_config.yaml`
  - Set `calib_cam_timeoffset: true` so OpenVINS refines the offset online.

- [ ] Done

---

## Step 7 — Rebuild and re-test VIO

**Who:** together

```bash
cd /workspaces/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stereo_depth_ros2
source install/setup.bash
ros2 launch stereo_depth_ros2 stereo_vio_mapping.launch.py
```

Test:

1. Keep the rig still → expect `dist ≈ 0.01 m`, no drift.
2. Move exactly 1 m out and back → compare `/unitree_go2/odom` / `/vio/odom_local`.
3. Check `/tmp/vio_diagnostics.log` for `HEARTBEAT status=OK` and small `peak` distance.

- [ ] Done

---

## What else is left for the full integration

After Kalibr tuning is verified, the remaining integration items are:

1. **Long-duration robustness test**
   - Run the full stack for 10–20 minutes of normal use.
   - Confirm no divergence, no memory growth, and stable map output.

2. **Map-to-odometry validation**
   - Walk a known closed loop (e.g. 2 m out, turn, return to start).
   - Check whether the voxel map and odometry agree at the closing point.

3. **Depth quality / fill improvements** (optional)
   - WLS filter on SGBM disparity, or `cv::cuda::StereoSGM` on Jetson.
   - Better depth = better voxel map, but does not affect VIO.

4. **Higher speed / hardware limits** (optional)
   - Test at faster motion to find where rolling shutter / motion blur breaks tracking.
   - Consider 60 fps capture if CPU allows, or global-shutter cameras if needed.

5. **Navigation stack integration**
   - Verify `navigation_runner` / `onboard_detector` consume `/unitree_go2/odom` and `/stereo/depth` correctly.
   - Test obstacle avoidance / path planning with the stereo-generated map.

6. **Documentation / cleanup**
   - Update `CONTEXT.md` with Kalibr results and final tuning values.
   - Commit the updated configs.

---

## Current blockers

- Need an AprilGrid target printed.
- Need access to an x86 PC/VM with Docker.

---

Last updated: 2026-07-09
