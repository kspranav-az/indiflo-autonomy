# Development Guide

## Workspace layout

`/workspaces/ros2_ws` is a ROS 2 workspace with source packages under `src/`.

## Building

### Stereo test tools

```bash
cd /workspaces/ros2_ws/src/stereo_test/build
cmake .. && make -j$(nproc)
```

### Select ROS 2 packages

```bash
cd /workspaces/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stereo_depth_ros2
```

### Rebuild after config changes

Configs are loaded at runtime; most changes do not require a rebuild. Rebuild only when C++/Python code changes.

## Calibration workflow

1. Print checkerboard from `src/stereo_test/scripts/generate_checkerboard.py`.
2. Capture pairs with `calibrate_stereo` or use existing `calib_images/`.
3. Run `calibrate_offline`.
4. Copy `stereo_calib.yml` to `src/stereo_depth_ros2/cfg/`.
5. Update `src/stereo_depth_ros2/config/openvins/cam_chain.yaml` manually.

## Git state

- Top-level `/workspaces/ros2_ws` is the main repo.
- `src/ros2/` is a **separate repo / submodule** containing `map_manager`, `navigation_runner`, `onboard_detector`.
- `src/icm20948_ros2/` is a **separate git repo**.
- `src/open_vins/` and `src/ceres-solver/` are external dependencies, not committed.

Commit changes in the correct repo. Do not commit submodule/external code from the top level.

## Testing checklist

- [ ] `ros2 topic hz /camera/left/image_raw` shows ~30 Hz.
- [ ] `ros2 topic hz /imu/data_raw` shows ~200 Hz.
- [ ] OpenVINS prints `successful initialization`.
- [ ] `/unitree_go2/odom` publishes.
- [ ] `/occupancy_map/raycast` service is available.
- [ ] Publishing `/goal_pose` produces non-zero `/unitree_go2/cmd_vel`.
