# Repository Structure

This workspace follows a standard ROS 2 underlay layout with some vendored dependencies.

```mermaid
graph TD
    A[/workspaces/ros2_ws] --> B[docs/]
    A --> C[src/]
    A --> D[scripts/]
    A --> E[cyclonedds.xml]
    A --> F[CONTEXT.md]
    A --> G[PROJECT_DOCUMENTATION.md]
    C --> H[stereo_depth_ros2]
    C --> I[ros2_common
map_manager, navigation_runner, onboard_detector]
    C --> J[icm20948_ros2]
    C --> K[perception_viz]
    C --> L[midas_trt_nvargus]
    C --> M[stereo_test]
    C --> N[third_party
open_vins, ceres-solver]
    A --> O[sim_go2
Mac Gazebo simulator]
```

## Top-level files

| File | Purpose |
|---|---|
| `CONTEXT.md` | Running session notes and quick reference |
| `PROJECT_DOCUMENTATION.md` | Master documentation with diagrams |
| `TODO.md` | Kalibr tuning and integration TODO |
| `cyclonedds.xml` | Jetson CycloneDDS config |
| `scripts/setup_jetson_sim_env.sh` | Jetson sim environment |
| `docs/` | Structured documentation |

## Source packages

| Path | Package | Notes |
|---|---|---|
| `src/stereo_depth_ros2` | Main orchestration + stereo depth + watchdog | Part of top-level repo |
| `src/ros2/map_manager` | Occupancy mapping | Separate repo/submodule |
| `src/ros2/navigation_runner` | RL navigation + safe action | Separate repo/submodule |
| `src/ros2/onboard_detector` | Dynamic obstacle detection | Separate repo/submodule |
| `src/icm20948_ros2` | IMU driver | Separate repo |
| `src/perception_viz` | Visualization helpers | Part of top-level repo |
| `src/midas_trt_nvargus` | Legacy monocular depth | Part of top-level repo |
| `src/stereo_test` | Standalone calibration/depth tools | Part of top-level repo |

## External dependencies

| Path | Source | Update |
|---|---|---|
| `src/open_vins` | Clone of OpenVINS | `git pull` upstream |
| `src/ceres-solver` | Clone of Ceres solver | `git pull` upstream |
