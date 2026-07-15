# Hardware Setup

## Compute

- **NVIDIA Jetson** running Ubuntu 20.04/22.04 (JetPack) with ROS 2 Humble.

## Sensors

### Stereo cameras

- Two CSI IMX219 cameras (Raspberry Pi V2 style modules).
- Left: `nvarguscamerasrc sensor-id=1`, Right: `nvarguscamerasrc sensor-id=0`.
- Native sensor mode: 1640×1232 (4:3, mode 3).
- Capture resolution: 640×480 scaled from native 4:3.
- Stereo processing resolution: 320×240.
- Baseline: ~60.97 mm.
- HFOV ≈ 73°, fx ≈ 415–420 px at 640×480, fx ≈ 208–210 px at 320×240.

### IMU

- **ICM-20948** over I2C.
- Output: `/imu/data_raw` at ~200 Hz (target; real rate may be ~130 Hz due to I2C).
- Gyro configured to ±1000 °/s (`FS_SEL = 2`).
- Accelerometer ±2 g default.

## Sensor coordinate frames

| Frame | Convention |
|---|---|
| IMU/body | `x = left`, `y = up`, `z = forward` |
| Camera optical | `x = right`, `y = down`, `z = forward` (OpenCV) |
| World/map | z-up ENU-like, set by OpenVINS at init |

The real camera mounting requires a 180° z-rotation between IMU and camera (`camera x = -IMU x`, `camera y = -IMU y`, `camera z = IMU z`). See `config/openvins/cam_chain.yaml`.

## Calibration

Target pattern: **7×5 internal corners**, **29.0 mm** squares.

Generate a printable board:

```bash
cd /workspaces/ros2_ws
python3 src/stereo_test/scripts/generate_checkerboard.py 7 5 \
    --square_mm 29.0 --output src/stereo_test/checkerboard_a4.svg
```

Live capture:

```bash
cd /workspaces/ros2_ws/src/stereo_test/build
./calibrate_stereo 640 480 30 5 7 29.0
```

- Press **SPACE** only when both views show green corners.
- Capture 15–25 pairs, varying distance/angle/position.
- Press **C** to calibrate.

Offline calibration:

```bash
./calibrate_offline 640 480 5 7 29.0
```

Target quality:

- RMS < 0.5 px (current best ~1.04 px).
- cam0→cam1 rotation well under 1°.
- Baseline close to 61 mm.

After calibration:

```bash
cp src/stereo_test/build/stereo_calib.yml src/stereo_depth_ros2/cfg/stereo_calib.yml
# Manually update src/stereo_depth_ros2/config/openvins/cam_chain.yaml
```

## Camera recovery

If `nvargus-daemon` hangs:

```bash
./fix_camera.sh
```

## Useful tools

- `stereo_sgbm_fast`: live rectified depth viewer.
- `check_imu_axes.py`: verify IMU axis conventions.

See [VIO_INTEGRATION.md](VIO_INTEGRATION.md) for how the sensors feed OpenVINS.
