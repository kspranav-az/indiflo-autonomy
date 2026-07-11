# Mac Agent: Gazebo Simulator Setup Instructions

## Your goal
Set up and run a Gazebo Harmonic simulator on macOS that publishes the sensor topics needed by the Jetson navigation stack.

## Architecture
- macOS runs Gazebo Harmonic + robot/sensor plugins.
- The Jetson runs OpenVINS, map_manager, navigation_node, etc.
- ROS 2 Humble must be installed on macOS so Gazebo plugins can publish ROS 2 topics.
- Both machines use the same `ROS_DOMAIN_ID` on the same LAN.

## Step 1 — Install prerequisites

```bash
# Install Homebrew if not present
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Xcode Command Line Tools
xcode-select --install

# Install Python and basic tools
brew install python@3.10 git cmake
```

## Step 2 — Install ROS 2 Humble on macOS

Try the Homebrew tap first:

```bash
brew tap osrf/simulation
brew install ros-humble-desktop
```

If that fails, build from source following:
https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html

After installation, verify:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Step 3 — Install Gazebo Harmonic

```bash
brew tap osrf/simulation
brew install gz-harmonic
```

Verify:

```bash
gz sim --version
```

## Step 4 — Create simulator workspace

```bash
mkdir -p ~/gz_sim_ws/src
cd ~/gz_sim_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
```

Try to build `ros_gz`:

```bash
cd ~/gz_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros_gz
```

If `ros_gz` fails to build on macOS, skip it and use the custom Python bridge approach described in Step 8.

## Step 5 — Create robot model

Create the following files:

```
~/gz_sim_ws/src/go2_sim/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── sim_world.launch.py
├── models/
│   └── go2_sim/
│       ├── model.config
│       └── model.sdf
└── worlds/
    └── nav_world.sdf
```

The robot model must include:
- A base link with mass and inertia.
- Left camera link at baseline origin, right camera link 61 mm to the right.
- IMU link coincident with the camera/IMU origin.
- A depth camera or third camera for `/stereo/depth`.
- A diff-drive or skid-steer plugin subscribed to `/unitree_go2/cmd_vel`.

Camera parameters (match the real calibration):
- Resolution: 640×480
- Left intrinsics: fx=415.10, fy=420.31, cx=355.06, cy=221.25
- Right intrinsics: fx=418.75, fy=419.11, cx=316.57, cy=264.48
- Baseline: ~60.97 mm
- Frame rate: 30 Hz

IMU parameters:
- Rate: 200 Hz
- Noise: accel 0.002 m/s²/√Hz, gyro 0.0003 rad/s/√Hz (or adjustable)

Depth camera parameters:
- Resolution: 320×240
- Depth range: 0.2–10 m
- Frame rate: 30 Hz

## Step 6 — Create textured world

Create `~/gz_sim_ws/src/go2_sim/worlds/nav_world.sdf`.

Requirements:
- Textured floor and walls (checkerboard, brick, or office textures).
- A few obstacles (boxes/cylinders).
- Even lighting.
- Large enough for 2–5 m navigation tests.

## Step 7 — Launch file

Create `~/gz_sim_ws/src/go2_sim/launch/sim_world.launch.py` that:
1. Starts Gazebo with `nav_world.sdf`.
2. Spawns the robot.
3. Bridges Gazebo sensor topics to ROS 2:
   - `/camera/left/image_raw`
   - `/stereo/right/image_raw`
   - `/imu/data_raw`
   - `/stereo/depth`
   - `/unitree_go2/cmd_vel`

If `ros_gz` is unavailable, use a custom Gazebo system plugin or a Python script that uses `rclpy` to publish/subscribe.

## Step 8 — Custom Python bridge (fallback if ros_gz fails)

Create `~/gz_sim_ws/src/go2_sim/scripts/gz_ros_bridge.py`:

```python
#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

# This script would connect to Gazebo's transport system or read rendered images
# and publish ROS 2 messages. A full implementation requires Gazebo's Python bindings.
```

For a more robust fallback, write a small C++ Gazebo system plugin using `rclcpp`.

## Step 9 — Network setup

Connect macOS and Jetson to the same network. On both machines:

```bash
export ROS_DOMAIN_ID=42
```

Test connectivity:

```bash
# On macOS
ros2 topic pub /test std_msgs/String 'data: hello'

# On Jetson
ros2 topic echo /test
```

If multicast fails, configure CycloneDDS with explicit peers.

## Step 10 — Run the simulator

```bash
cd ~/gz_sim_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch go2_sim sim_world.launch.py
```

Verify these topics are visible on the Jetson:

```bash
ros2 topic list | grep -E 'camera/left|stereo/right|imu/data_raw|stereo/depth|cmd_vel'
ros2 topic hz /camera/left/image_raw
ros2 topic hz /imu/data_raw
ros2 topic hz /stereo/depth
```

## Deliverables back to Jetson-side Kimi

Once the simulator is running and topics are visible on the Jetson, report:
1. ROS 2 Humble installation method used (Homebrew or source).
2. Gazebo version installed.
3. Whether `ros_gz` built successfully or a custom bridge was needed.
4. Exact topic names and rates.
5. SDF/launch file paths so Kimi can inspect if needed.
