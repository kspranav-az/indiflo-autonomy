# Configuration Reference

## DDS / network

| File | Purpose |
|---|---|
| `cyclonedds.xml` | Jetson CycloneDDS config: bind to `192.168.55.7`, peers Mac + loopback |
| `~/cyclonedds.xml` (Mac) | Mac CycloneDDS config: no interface binding, explicit peers |
| `scripts/setup_jetson_sim_env.sh` | Jetson env vars: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI` |
| `~/personal/indiflo/sim/setup_env.sh` (Mac) | Mac env vars and workspace sourcing |

## OpenVINS

Real robot:

- `src/stereo_depth_ros2/config/openvins/cam_chain.yaml`
- `src/stereo_depth_ros2/config/openvins/imu_chain.yaml`
- `src/stereo_depth_ros2/config/openvins/estimator_config.yaml`

Simulator:

- `src/stereo_depth_ros2/config/openvins_sim/cam_chain.yaml`
- `src/stereo_depth_ros2/config/openvins_sim/imu_chain.yaml`
- `src/stereo_depth_ros2/config/openvins_sim/estimator_config.yaml`

Key OpenVINS parameters:

| Parameter | Typical value | Meaning |
|---|---|---|
| `try_zupt` | `true` | Enable zero-velocity updates and init-while-still |
| `zupt_max_disparity` | `0.5` | Disparity threshold for stationary detection |
| `timeshift_cam_imu` | `0.0` | Camera-IMU time offset (todo: calibrate) |
| `calib_cam_timeoffset` | `false` | Online time-offset refinement (enable after good initial guess) |

## Stereo depth

- `src/stereo_depth_ros2/cfg/stereo_depth_node_param.yaml`
- `src/stereo_depth_ros2/cfg/stereo_calib.yml`
- `src/stereo_depth_ros2/cfg/fake_odom_node_param.yaml`

## Mapping

- `src/ros2/map_manager/cfg/map_param.yaml`
- `src/stereo_depth_ros2/cfg/map_param_sim.yaml`

## Navigation

- `src/ros2/navigation_runner/cfg/navigation_param.yaml`
- `src/ros2/navigation_runner/cfg/safe_action_param.yaml`
- `src/ros2/navigation_runner/cfg/dynamic_detector_param.yaml`
- `src/ros2/navigation_runner/cfg/map_param.yaml`
- `src/ros2/onboard_detector/cfg/dynamic_detector_param.yaml`
- `src/ros2/onboard_detector/cfg/yolo_detector_param.yaml`

## Watchdog

Parameters in `stereo_vio_navigation.launch.py` / `stereo_vio_navigation_sim.launch.py` under the `vio_watchdog` node:

| Parameter | Default | Meaning |
|---|---|---|
| `odom_topic` | `/unitree_go2/odom` | Odometry to monitor |
| `status_topic` | `/vio/status` | Published status |
| `diagnostics_path` | `/tmp/vio_diagnostics.log` | Log file |
| `position_threshold_m` | `50.0` | Instant divergence threshold |
| `velocity_threshold_m_s` | `10.0` | Sustained velocity alarm |
| `delta_threshold_m` | `5.0` | Position jump alarm |
| `window_size` | `10` | Jump detection window |
