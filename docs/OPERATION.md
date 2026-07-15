# Operation Guide

## Startup — hybrid simulation

1. **Mac:** start Gazebo.
   ```bash
   bash
   source /Users/pranav/personal/indiflo/sim/setup_env.sh
   ros2 launch go2_sim sim_world.launch.py gui:=false
   ```

2. **Jetson:** start autonomy stack.
   ```bash
   source /workspaces/ros2_ws/scripts/setup_jetson_sim_env.sh
   ros2 launch stereo_depth_ros2 stereo_vio_navigation_sim.launch.py
   ```

3. Verify DDS discovery on Jetson:
   ```bash
   ros2 topic list
   ros2 topic hz /camera/left/image_raw
   ros2 topic hz /imu/data_raw
   ```

4. Verify OpenVINS initialized:
   ```bash
   ros2 topic hz /unitree_go2/odom
   ros2 topic echo /vio/status
   ```

## Startup — real robot

```bash
source /workspaces/ros2_ws/install/setup.bash
ros2 launch stereo_depth_ros2 stereo_vio_navigation.launch.py
```

## Sending a navigation goal

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' --once
```

The robot will first rotate toward the goal, then translate.

## Monitoring

| What | Command |
|---|---|
| Odometry | `ros2 topic echo /unitree_go2/odom` |
| VIO status | `ros2 topic echo /vio/status` |
| Diagnostics log | `tail -f /tmp/vio_diagnostics.log` |
| Commanded velocity | `ros2 topic echo /unitree_go2/cmd_vel` |

## Watchdog services

Reset the local origin:

```bash
ros2 service call /vio/reset std_srvs/srv/Trigger
```

Truncate diagnostics log:

```bash
ros2 service call /vio/clear_log std_srvs/srv/Trigger
```

## Emergency stop

```bash
ros2 topic pub /navigation_emergency_stop std_msgs/Bool '{data: true}' --once
```

## Graceful shutdown

Stop Jetson stack first, then Mac simulator:

```bash
# Jetson
Ctrl+C

# Mac
pkill -f "gz sim"
pkill -f gz_ros_bridge
```
