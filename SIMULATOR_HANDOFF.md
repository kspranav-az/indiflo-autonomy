# Gazebo Harmonic + ROS 2 Humble Simulator — Jetson Handoff

This document summarizes what has been built on the MacBook Air in `/Users/pranav/personal/indiflo/sim`, what is verified working, and how a Jetson (or any ROS 2 Humble node) can connect to this simulator.

---

## 1. Goal

Provide a native macOS Gazebo Harmonic simulation of a Unitree Go2-like robot that publishes the same ROS 2 sensor topics and accepts the same velocity commands as the real robot. This lets the Jetson autonomy stack run against the simulator for development and testing before deploying to hardware.

---

## 2. What Is Built on the Mac

### 2.1 Environment

| Component | Location / Version | Notes |
|-----------|-------------------|-------|
| Python venv (uv-managed) | `/Users/pranav/personal/indiflo/sim/.venv` | Python 3.10, colcon, vcstool, rosdep, catkin tools |
| Gazebo Sim | Homebrew, `gz sim 8.14.0` | Installed via `brew install gz-sim` |
| ROS 2 Humble | `/Users/pranav/personal/indiflo/sim/ros2_humble` | Built from source with **CycloneDDS only** |
| `go2_sim` package | `/Users/pranav/personal/indiflo/sim/gz_sim_ws` | Custom world, robot model, and Gazebo ↔ ROS 2 bridge |
| Env setup script | `/Users/pranav/personal/indiflo/sim/setup_env.sh` | Sources ROS 2 + `go2_sim`; must be run from **bash** |

### 2.2 ROS 2 Build Details

- **RMW implementation:** `rmw_cyclonedds_cpp` (Fast-DDS, Connext, iceoryx, and test vendors were skipped).
- **Skipped packages:** `mimick_vendor`, `uncrustify_vendor` (harmless; produce “not found” warnings during env setup).
- **Patches applied during build:**
  - `python_cmake_module` FindPythonExtra fallback for uv Python lib path.
  - `rmw_implementation/package.xml` group_depend removed.
  - `libyaml_vendor` CMake policy and CycloneDDS iceoryx dependency handling.

---

## 3. Simulation Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MacBook Air                               │
│  ┌─────────────────────┐      ┌──────────────────────────────┐  │
│  │  Gazebo Harmonic    │      │  go2_sim gz_ros_bridge       │  │
│  │  - nav_world.sdf    │◄────►│  C++ rclcpp + gz-transport13 │  │
│  │  - go2 robot model  │      │                              │  │
│  └─────────────────────┘      └──────────────────────────────┘  │
│           ▲                              │                       │
│           │ Gazebo Transport             │ ROS 2                 │
│           │                              ▼                       │
│  /camera/left/image       /camera/left/image_raw                │
│  /stereo/right/image  ──► /stereo/right/image_raw               │
│  /stereo/depth            /stereo/depth                         │
│  /imu/data                /imu/data_raw                         │
│  /unitree_go2/cmd_vel ◄── /unitree_go2/cmd_vel                  │
│  /odom (DiffDrive)        /odom                                 │
└─────────────────────────────────────────────────────────────────┘
                              ▲
                              │ Ethernet / Wi-Fi (same ROS_DOMAIN_ID)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                          Jetson                                  │
│   Autonomy stack (SLAM, Nav2, perception, planning, etc.)       │
│   - Subscribes to image/depth/IMU topics                         │
│   - Publishes /unitree_go2/cmd_vel                               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. ROS 2 Topic Contract

The simulator exposes these ROS 2 topics. **The Jetson stack should be configured to use the exact topic names below.**

### 4.1 Published by simulator (Mac → Jetson)

| Topic | Type | Frame ID | Rate | Notes |
|-------|------|----------|------|-------|
| `/camera/left/image_raw` | `sensor_msgs/msg/Image` | `left_camera_link` | 30 Hz | RGB, `rgb8`, 640×480 |
| `/stereo/right/image_raw` | `sensor_msgs/msg/Image` | `right_camera_link` | 30 Hz | RGB, `rgb8`, 640×480 |
| `/stereo/depth` | `sensor_msgs/msg/Image` | `depth_camera_link` | 30 Hz | Depth, 320×240 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | `imu_link` | 200 Hz | Covariance fields are zero-filled |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` | DiffDrive output | — |

### 4.2 Subscribed by simulator (Jetson → Mac)

| Topic | Type | Notes |
|-------|------|-------|
| `/unitree_go2/cmd_vel` | `geometry_msgs/msg/Twist` | Forwarded to Gazebo DiffDrive plugin |

---

## 5. Coordinate Frames & Sensor Placement

All sensor links are fixed to `base_link`:

- `base_link`: robot center, z = 0.1 m above ground.
- `left_camera_link`: `(0.15, 0, 0.05)` relative to `base_link`.
- `right_camera_link`: `(0.15, -0.06097, 0.05)` — 61 mm baseline to the right.
- `imu_link`: `(0.15, 0, 0.05)` — coincident with left camera.
- `depth_camera_link`: `(0.15, 0, 0.05)`.

Wheel radius: 0.05 m, wheel separation: 0.24 m.

---

## 6. How to Start the Simulator on the Mac

Open a terminal and run **from bash** (the setup script is bash-only):

```bash
bash
source /Users/pranav/personal/indiflo/sim/setup_env.sh
```

You should see:

```
Environment ready.
  SIM_DIR=/Users/pranav/personal/indiflo/sim
  PYTHON=/Users/pranav/personal/indiflo/sim/.venv/bin/python3
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 6.1 Headless mode (for remote / terminal use)

```bash
ros2 launch go2_sim sim_world.launch.py gui:=false
```

### 6.2 GUI mode (for visualizing on the Mac desktop)

```bash
ros2 launch go2_sim sim_world.launch.py gui:=true
```

On macOS, Gazebo requires separate server and GUI processes; the launch file handles this automatically.

### 6.3 Verify topics are live

In another terminal (after sourcing `setup_env.sh`):

```bash
ros2 topic list --no-daemon
ros2 topic hz /imu/data_raw
ros2 topic echo /camera/left/image_raw
```

---

## 7. How to Connect the Jetson

### 7.1 Required environment on the Jetson

The Jetson must have **ROS 2 Humble** installed and use the same middleware:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42   # must match the Mac
```

Source the Jetson’s own ROS 2 setup:

```bash
source /opt/ros/humble/setup.bash   # or wherever Humble is installed
```

### 7.2 Network checklist

1. Mac and Jetson on the same subnet (or routed with UDP multicast enabled).
2. Same `ROS_DOMAIN_ID` on both machines.
3. Same `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both machines.
4. No firewall blocking DDS discovery / traffic.

### 7.3 Verify connection

On the Jetson:

```bash
ros2 topic list
```

You should see the Mac’s topics:

```
/camera/left/image_raw
/imu/data_raw
/parameter_events
/rosout
/stereo/depth
/stereo/right/image_raw
/unitree_go2/cmd_vel
```

Send a test velocity command from the Jetson:

```bash
ros2 topic pub /unitree_go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

The simulated robot should move in Gazebo.

### 7.4 If DDS discovery is flaky over Wi-Fi

Create a CycloneDDS config file on both machines, e.g. `cyclonedds.xml`:

```xml
<?xml version="1.0"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="42">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.10"/>   <!-- Mac IP -->
        <Peer address="192.168.1.20"/>   <!-- Jetson IP -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Then export:

```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

---

## 8. What Is Verified Working

- [x] ROS 2 Humble CLI works (`ros2 topic list`, `ros2 topic echo`, `ros2 topic hz`).
- [x] Gazebo server loads `nav_world.sdf` without plugin errors.
- [x] `go2_sim` robot model loads with cameras, depth, IMU, and DiffDrive.
- [x] `gz_ros_bridge` connects to Gazebo transport and creates ROS 2 publishers/subscribers.
- [x] `/camera/left/image_raw` publishes live 640×480 `rgb8` images.
- [x] `/imu/data_raw` publishes live IMU messages.
- [x] `/unitree_go2/cmd_vel` is wired to receive Twist messages and forward them to Gazebo.
- [x] `ros2 launch go2_sim sim_world.launch.py gui:=false` starts successfully.
- [x] `setup_env.sh` correctly sets up the environment when sourced from bash.

---

## 12. Jetson-side configuration adjustments

Based on the Mac model poses (all sensor links coincident with zero rotation), the Jetson simulator configs were updated:

- `src/stereo_depth_ros2/config/openvins_sim/cam_chain.yaml` — `T_imu_cam` set to **identity** (was 180° z-rotation for the real robot).
- `src/stereo_depth_ros2/cfg/map_param_sim.yaml` — `body_to_depth_sensor` set to **identity**.
- `src/stereo_depth_ros2/launch/stereo_vio_navigation_sim.launch.py` — uses the sim OpenVINS config and sim map config.
- `scripts/setup_jetson_sim_env.sh` — sets `ROS_DOMAIN_ID=42` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- `cyclonedds.xml` — template for unicast peer fallback if multicast fails.

---

## 13. Known Limitations & Notes

1. **Shell requirement:** `setup_env.sh` must be sourced from `bash`, not `zsh`. The colcon-generated `setup.bash` files rely on `${BASH_SOURCE[0]}` and break under zsh.
2. **macOS Gazebo GUI:** Gazebo on macOS cannot run server and GUI in the same process; the launch file runs them separately. Use `gui:=false` for headless operation.
3. **No real-time guarantees:** Gazebo `real_time_factor` is set to 1, but actual timing depends on Mac load.
4. **Image bandwidth:** Three image streams over Wi-Fi can saturate the link. Consider throttling or reducing resolution for remote Jetson development.
5. **Covariance:** IMU covariance fields are zero because Gazebo’s IMU message does not provide them.
6. **Skipped vendor packages:** `mimick_vendor` and `uncrustify_vendor` warnings during env setup are harmless.

---

## 10. What Is Not Built Yet

- [ ] Real robot bringup / Unitree SDK integration on the Jetson.
- [ ] SLAM or navigation stack (e.g., Nav2, `slam_toolbox`, RTAB-Map) on the Jetson.
- [ ] Camera calibration files or rectification nodes.
- [ ] TF/static transform publishers for the full robot URDF.
- [ ] Sensor noise profiles beyond the basic IMU noise already in the SDF.
- [ ] Jetson-specific DDS tuning / CycloneDDS config file.

---

## 11. File Reference

| File | Purpose |
|------|---------|
| `/Users/pranav/personal/indiflo/sim/setup_env.sh` | One-command environment setup for the Mac |
| `/Users/pranav/personal/indiflo/sim/gz_sim_ws/src/go2_sim/worlds/nav_world.sdf` | Gazebo world with inlined go2 model |
| `/Users/pranav/personal/indiflo/sim/gz_sim_ws/src/go2_sim/models/go2_sim/model.sdf` | Standalone go2 model (used as reference; world inlines it) |
| `/Users/pranav/personal/indiflo/sim/gz_sim_ws/src/go2_sim/src/gz_ros_bridge.cpp` | C++ bridge between Gazebo transport and ROS 2 |
| `/Users/pranav/personal/indiflo/sim/gz_sim_ws/src/go2_sim/launch/sim_world.launch.py` | Launch file for Gazebo server/GUI + bridge |
| `/Users/pranav/personal/indiflo/sim/ros2_humble/install/setup.bash` | ROS 2 Humble environment |
| `/Users/pranav/personal/indiflo/sim/gz_sim_ws/install/setup.bash` | `go2_sim` package environment |
