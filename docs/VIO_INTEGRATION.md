# VIO Integration (OpenVINS)

OpenVINS provides stereo-inertial odometry. It subscribes to the left/right images and IMU, and publishes `/unitree_go2/odom` (remapped from `/ov/odomimu`).

## Packages

| Package | Role |
|---|---|
| `ov_core` | Core math, feature tracking, state types |
| `ov_init` | Static / dynamic initialization |
| `ov_msckf` | MSCKF estimator + ROS 2 node |
| `ov_eval` | Evaluation tools |
| `ceres-solver` | Non-linear solver (local build) |

## ROS 2 interface

### Subscriptions

- `/camera/left/image_raw`
- `/stereo/right/image_raw`
- `/imu/data_raw`

### Publications

- `/unitree_go2/odom` (`nav_msgs/Odometry`, remapped from `/ov/odomimu`)
- `/ov/pose` etc. (optional, see launch)

## Pipeline

```mermaid
graph TD
    A[/camera/left/image_raw] -->|KLT feature tracking| B[ov_msckf]
    C[/stereo/right/image_raw] -->|stereo matching| B
    D[/imu/data_raw] -->|state propagation| B
    B -->|initialization| E[static init / ZUPT]
    B -->|/unitree_go2/odom| F[navigation + mapping]
```

## Configuration files

Real robot configs live in `src/stereo_depth_ros2/config/openvins/`:

| File | Purpose |
|---|---|
| `cam_chain.yaml` | Intrinsics, distortion, and `T_imu_cam` extrinsics |
| `imu_chain.yaml` | IMU noise density / random walk |
| `estimator_config.yaml` | Feature tracking, MSCKF, ZUPT, time-offset options |

Simulator counterparts are in `config/openvins_sim/`.

## Initialization

OpenVINS uses static initialization by default.

- With `try_zupt: false`, the filter waits for an accel jerk (stationary → motion) and may never initialize if held still.
- With `try_zupt: true` (current setting), the filter initializes while held still and applies zero-velocity updates (ZUPT) when image disparity < 0.5 px.

Signs of successful init:

```text
[init]: successful initialization
q_GtoI = ... | p_IinG = ... | dist = ...
```

## TF frames

OpenVINS publishes:

- `global -> imu` (odometry)
- `imu -> cam0`
- `imu -> cam1`

RViz and `map_manager` use `map` as the fixed frame. An identity `map -> global` static TF bridges the two.

## Tuning roadmap

1. **Camera-IMU time offset** (`timeshift_cam_imu`). Currently 0. Even a few ms matters at speed.
2. **IMU noise parameters** via Allan variance / Kalibr.
3. **Frame rate / exposure** for faster motion.
4. **Online calibration** of intrinsics/extrinsics/timeoffset once the initial guess is close.

See `docs/STEREO_VIO_INTEGRATION.md` for historical debugging notes.
