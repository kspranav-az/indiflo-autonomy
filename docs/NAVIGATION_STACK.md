# Navigation Stack

The navigation stack consumes VIO odometry, builds an occupancy map, detects dynamic obstacles, and computes `/unitree_go2/cmd_vel` to reach a `/goal_pose`.

## Architecture

```mermaid
graph TD
    A[/unitree_go2/odom] --> B[navigation_node]
    C[/goal_pose] --> B
    D[map_manager] -->|/occupancy_map/raycast| B
    E[onboard_detector] -->|/onboard_detector/get_dynamic_obstacles| B
    B -->|/safe_action/get_safe_action| F[safe_action_node]
    F --> B
    B -->|/unitree_go2/cmd_vel| G[robot / simulator]
```

## Nodes

| Node | Package | Responsibility |
|---|---|---|
| `map_manager_node` | `map_manager` | Builds voxel + 2D occupancy maps; serves `/occupancy_map/raycast` |
| `dynamic_detector_node` | `onboard_detector` | Detects dynamic obstacles; serves `/onboard_detector/get_dynamic_obstacles` |
| `safe_action_node` | `navigation_runner` | Collision-aware velocity correction; serves `/safe_action/get_safe_action` |
| `navigation_node` | `navigation_runner` | RL policy + PID yaw control; consumes goal and publishes `cmd_vel` |
| `vio_watchdog` | `stereo_depth_ros2` | Monitors odometry divergence and provides reset/clear-log services |

## Goal interface

Publish a `geometry_msgs/PoseStamped` to `/goal_pose` with `frame_id: "map"`:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' --once
```

Behavior:

1. `navigation_node` stores the goal and overwrites `goal.pose.position.z` with the current odometry height.
2. It computes the 2-D goal direction and rotates in place until `|angle_diff| < 0.3 rad`.
3. Once aligned, the RL policy computes a velocity command.
4. `safe_action_node` checks for collisions and may override the velocity.
5. When the robot is within 1.0 m of the goal, velocity is zeroed.

## Services used by navigation_node

| Service | Provider | Purpose |
|---|---|---|
| `/occupancy_map/raycast` | `map_manager_node` | Lidar-like obstacle scan |
| `/onboard_detector/get_dynamic_obstacles` | `dynamic_detector_node` | Moving obstacles |
| `/safe_action/get_safe_action` | `safe_action_node` | Final velocity correction |

## Dependencies

- `torchrl` (with `Composite` / `UnboundedContinuous` classes)
- `einops`
- `hydra-core`
- Jetson-optimized PyTorch

**YOLO** is disabled by default (`use_yolo:=false`) because `torchvision` is not compatible with the Jetson PyTorch build.

## Emergency stop

```bash
ros2 topic pub /navigation_emergency_stop std_msgs/Bool '{data: true}' --once
```

See [OPERATION.md](OPERATION.md) for full operating procedures.
