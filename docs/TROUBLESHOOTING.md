# Troubleshooting

## DDS / network

### `ros2 topic list` is empty or missing Mac topics

- Confirm both sides source their env scripts.
- Check `ROS_DOMAIN_ID=42` on both.
- Check `CYCLONEDDS_URI` points to the current XML file.
- Restart both sides after any XML change.
- Verify peer IPs match the current subnet (wired `192.168.55.x`).

### `ddsi_udp_conn_write ... retcode -5`

Cause: reliable QoS retransmission overflow under image load.
Fix: ensure image/IMU publishers and subscribers use `rclcpp::SensorDataQoS()` (BEST_EFFORT).

## Cameras

### `nvargus-daemon` hung / no frames

```bash
./fix_camera.sh
```

### Left/right swapped

The convention is:
- Left camera = `nvarguscamerasrc sensor-id=1`
- Right camera = `nvarguscamerasrc sensor-id=0`

## VIO

### OpenVINS never prints `successful initialization`

- Verify `/imu/data_raw` is publishing at ~200 Hz.
- Verify `/camera/left/image_raw` and `/stereo/right/image_raw` are arriving.
- Check that image callbacks are matched:
  ```bash
  ros2 topic info -v /camera/left/image_raw
  ```
- If held still, confirm `try_zupt: true` in `estimator_config.yaml`.

### VIO diverges / km-scale runaway

- Check calibration RMS and cam0→cam1 rotation in `cam_chain.yaml`.
- Check IMU gyro scale: chip must be in ±1000 °/s mode (`FS_SEL = 2`).
- Check IMU timestamps are monotonic and unique.
- Use watchdog reset and inspect `/tmp/vio_diagnostics.log`.

## Navigation

### Robot spins in place forever after goal

- This is the alignment phase. Verify yaw is changing on `/unitree_go2/odom`.
- If yaw converges but `linear.x` stays zero, check:
  - `/occupancy_map/raycast` is available.
  - `safe_action_node` is not blocking forward motion.
  - RL policy checkpoint loaded without errors.

### `navigation_node` fails to import

Install missing Python deps:

```bash
pip3 install torchrl einops hydra-core
```

### YOLO fails to start

YOLO needs torchvision, which is not available on the Jetson torch build. Keep `use_yolo:=false`.

## General

### Stale ROS 2 daemon

```bash
pkill -f ros2-daemon
```
