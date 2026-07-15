# Launch File Reference

## Real hardware

| Launch file | Package | Purpose | Key arguments |
|---|---|---|---|
| `stereo_depth.launch.py` | `stereo_depth_ros2` | Stereo capture + depth publisher | — |
| `fake_odom.launch.py` | `stereo_depth_ros2` | Placeholder odometry publisher | — |
| `stereo_depth_with_fake_odom.launch.py` | `stereo_depth_ros2` | Stereo depth + fake odometry | — |
| `stereo_perception.launch.py` | `stereo_depth_ros2` | Stereo depth + fake odom + map_manager + RViz | — |
| `openvins.launch.py` | `stereo_depth_ros2` | Stereo depth + OpenVINS VIO | — |
| `stereo_vio_mapping.launch.py` | `stereo_depth_ros2` | Cameras + IMU + OpenVINS + map_manager + RViz + watchdog | — |
| `stereo_vio_navigation.launch.py` | `stereo_depth_ros2` | Full stack: VIO + mapping + dynamic detector + safe action + navigation + watchdog | `use_navigation`, `use_yolo`, `use_rviz` |

## Hybrid simulation

| Launch file | Package | Purpose | Key arguments |
|---|---|---|---|
| `stereo_vio_navigation_sim.launch.py` | `stereo_depth_ros2` | Same as full navigation stack, but receives camera/IMU/depth from the Mac simulator. Does not launch `stereo_depth_node` or `icm20948_node`. | `use_navigation`, `use_yolo`, `use_rviz` |

## Standalone / subsystem launches

| Launch file | Package | Purpose |
|---|---|---|
| `occupancy_map.launch.py` | `map_manager` | Standalone occupancy mapper |
| `dynamic_detector.launch.py` | `onboard_detector` | Standalone dynamic obstacle detector |
| `navigation.launch.py` | `navigation_runner` | Standalone navigation + safe action |
| `rviz.launch.py` | various | RViz with package-specific config |
| `sensor_viz.launch.py` | `perception_viz` | Visualize legacy MiDaS output |
| `imu_camera_sync.launch.py` | `icm20948_ros2` | IMU + camera sync helper |

## Common launch arguments

| Argument | Default | Effect |
|---|---|---|
| `use_navigation` | `true` | Launch `navigation_node` and `safe_action_node` |
| `use_yolo` | `false` | Launch YOLO detector (needs torchvision) |
| `use_rviz` | `false` (sim), various | Launch RViz2 |

## Typical startup sequences

Real robot:

```bash
source /workspaces/ros2_ws/install/setup.bash
ros2 launch stereo_depth_ros2 stereo_vio_navigation.launch.py
```

Hybrid simulation:

```bash
# Mac
bash
source /Users/pranav/personal/indiflo/sim/setup_env.sh
ros2 launch go2_sim sim_world.launch.py gui:=false

# Jetson
source /workspaces/ros2_ws/scripts/setup_jetson_sim_env.sh
ros2 launch stereo_depth_ros2 stereo_vio_navigation_sim.launch.py
```
