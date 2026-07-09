#!/usr/bin/env python3
"""VIO watchdog node.

Monitors /unitree_go2/odom, detects divergence, logs concise diagnostics,
and provides a /vio/reset service to clear the local origin.

Usage:
    ros2 run stereo_depth_ros2 vio_watchdog.py

Or add to launch file:
    Node(package='stereo_depth_ros2', executable='vio_watchdog.py', ...)

Diagnostics are written to /tmp/vio_diagnostics.log.
A reset publishes a static transform map -> odom at the current robot pose,
so downstream nodes see the robot back near the origin.
"""

import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class VIOWatchdog(Node):
    def __init__(self):
        super().__init__('vio_watchdog')

        # Parameters
        self.declare_parameter('odom_topic', '/unitree_go2/odom')
        self.declare_parameter('status_topic', '/vio/status')
        self.declare_parameter('diagnostics_path', '/tmp/vio_diagnostics.log')
        self.declare_parameter('position_threshold_m', 50.0)
        self.declare_parameter('velocity_threshold_m_s', 10.0)
        self.declare_parameter('delta_threshold_m', 5.0)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('log_max_lines', 2000)
        # Require sustained high velocity (this many consecutive frames) before
        # flagging, so a single-frame filter correction / init transient does
        # not false-alarm.
        self.declare_parameter('velocity_consecutive', 5)
        # Ignore the first few samples after start/reset (OpenVINS init jump).
        self.declare_parameter('warmup_samples', 5)
        # Consecutive healthy samples needed to auto-clear a DIVERGED state.
        self.declare_parameter('recover_samples', 60)
        # Throttled heartbeat log period (seconds) so the log shows healthy
        # tracking, not just failures.
        self.declare_parameter('heartbeat_period_s', 5.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.diagnostics_path = self.get_parameter('diagnostics_path').value
        self.position_threshold = self.get_parameter('position_threshold_m').value
        self.velocity_threshold = self.get_parameter('velocity_threshold_m_s').value
        self.delta_threshold = self.get_parameter('delta_threshold_m').value
        self.window_size = self.get_parameter('window_size').value
        self.log_max_lines = self.get_parameter('log_max_lines').value
        self.velocity_consecutive = self.get_parameter('velocity_consecutive').value
        self.warmup_samples = self.get_parameter('warmup_samples').value
        self.recover_samples = self.get_parameter('recover_samples').value
        self.heartbeat_period_s = self.get_parameter('heartbeat_period_s').value

        # State
        self.status = 'OK'
        self.last_position = None
        self.last_time = None
        self.position_window = []
        self.reset_origin = None
        self.initialized = False
        self.sample_count = 0
        self.high_vel_count = 0
        self.healthy_count = 0
        self.peak_dist = 0.0
        self.last_velocity = 0.0
        self.last_heartbeat_s = None

        # Publishers
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.local_odom_pub = self.create_publisher(Odometry, '/vio/odom_local', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vio/pose', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, 50)

        # Services
        self.reset_srv = self.create_service(Trigger, '/vio/reset', self.reset_cb)
        self.clear_log_srv = self.create_service(Trigger, '/vio/clear_log', self.clear_log_cb)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Timer for status publishing
        self.create_timer(1.0, self.publish_status)

        self.log_info('VIO watchdog started')
        self.log_info(f'  position_threshold={self.position_threshold} m')
        self.log_info(f'  velocity_threshold={self.velocity_threshold} m/s '
                      f'(sustained {self.velocity_consecutive} frames)')
        self.log_info(f'  delta_threshold={self.delta_threshold} m '
                      f'over {self.window_size} samples')
        self.log_info(f'  warmup={self.warmup_samples} samples, '
                      f'recover={self.recover_samples} samples')

    def log_info(self, msg):
        line = f'{datetime.now().isoformat()}  INFO  {msg}'
        self.get_logger().info(msg)
        self.append_log(line)

    def log_warn(self, msg):
        line = f'{datetime.now().isoformat()}  WARN  {msg}'
        self.get_logger().warn(msg)
        self.append_log(line)

    def append_log(self, line):
        try:
            with open(self.diagnostics_path, 'a') as f:
                f.write(line + '\n')
            self.rotate_log()
        except Exception as e:
            self.get_logger().error(f'Failed to write diagnostics log: {e}')

    def rotate_log(self):
        if not os.path.exists(self.diagnostics_path):
            return
        try:
            with open(self.diagnostics_path, 'r') as f:
                lines = f.readlines()
            if len(lines) > self.log_max_lines:
                with open(self.diagnostics_path, 'w') as f:
                    f.writelines(lines[-self.log_max_lines:])
        except Exception:
            pass

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        t = self.get_clock().now()
        pos = (p.x, p.y, p.z)
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        # Compute velocity
        velocity = 0.0
        if self.last_position is not None and self.last_time is not None:
            dt = (t - self.last_time).nanoseconds / 1e9
            if dt > 0:
                dx = pos[0] - self.last_position[0]
                dy = pos[1] - self.last_position[1]
                dz = pos[2] - self.last_position[2]
                velocity = math.sqrt(dx**2 + dy**2 + dz**2) / dt

        self.last_position = pos
        self.last_time = t
        self.last_velocity = velocity
        self.sample_count += 1
        self.peak_dist = max(self.peak_dist, dist)

        # Maintain a short window of recent positions to detect jumps
        self.position_window.append(pos)
        if len(self.position_window) > self.window_size:
            self.position_window.pop(0)

        # Track sustained (not single-frame) high velocity so that a lone
        # filter correction or the post-init jump does not false-alarm.
        if velocity > self.velocity_threshold:
            self.high_vel_count += 1
        else:
            self.high_vel_count = 0

        # Detect divergence. Skip during warmup (OpenVINS init transient).
        diverged = False
        reason = ''
        warming = self.sample_count <= self.warmup_samples

        if not warming:
            if dist > self.position_threshold:
                diverged = True
                reason = (f'position magnitude {dist:.2f} m > threshold '
                          f'{self.position_threshold} m')
            elif self.high_vel_count >= self.velocity_consecutive:
                diverged = True
                reason = (f'sustained velocity {velocity:.2f} m/s for '
                          f'{self.high_vel_count} frames > threshold '
                          f'{self.velocity_threshold} m/s')
            elif len(self.position_window) >= 2:
                p0 = self.position_window[0]
                p1 = self.position_window[-1]
                jump = math.sqrt(
                    (p1[0] - p0[0])**2 +
                    (p1[1] - p0[1])**2 +
                    (p1[2] - p0[2])**2)
                if jump > self.delta_threshold:
                    diverged = True
                    reason = (f'position jump {jump:.2f} m in '
                              f'{len(self.position_window)} samples > threshold '
                              f'{self.delta_threshold} m')

        # State machine: latch on divergence, auto-clear after sustained health.
        if diverged:
            self.healthy_count = 0
            if self.status == 'OK':
                self.status = 'DIVERGED'
                self.log_warn(f'DIVERGENCE DETECTED: {reason}')
                self.log_warn(f'  pose: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} '
                              f'dist={dist:.3f} peak={self.peak_dist:.1f}')
        else:
            if self.status == 'DIVERGED':
                self.healthy_count += 1
                if self.healthy_count >= self.recover_samples:
                    self.status = 'OK'
                    self.log_info(f'RECOVERED: pose healthy again '
                                  f'(dist={dist:.3f} m) after '
                                  f'{self.healthy_count} samples')
                    self.healthy_count = 0

        # Throttled heartbeat so the log shows the trajectory is sane, not just
        # failures. Lets you confirm healthy tracking from the log alone.
        now_s = t.nanoseconds / 1e9
        if (self.last_heartbeat_s is None or
                (now_s - self.last_heartbeat_s) >= self.heartbeat_period_s):
            self.last_heartbeat_s = now_s
            self.log_info(f'HEARTBEAT status={self.status} dist={dist:.3f} m '
                          f'vel={velocity:.2f} m/s peak={self.peak_dist:.1f} m')

        # Publish local odometry (relative to reset origin)
        local_msg = Odometry()
        local_msg.header = msg.header
        local_msg.child_frame_id = msg.child_frame_id
        local_msg.pose = msg.pose
        local_msg.twist = msg.twist

        if self.reset_origin is not None:
            ox, oy, oz = self.reset_origin
            local_msg.pose.pose.position.x -= ox
            local_msg.pose.pose.position.y -= oy
            local_msg.pose.pose.position.z -= oz

        self.local_odom_pub.publish(local_msg)

        # Publish pose for RViz
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = local_msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # Publish tf map -> vio_odom_local so RViz can show a stable local frame
        tfs = TransformStamped()
        tfs.header = msg.header
        tfs.header.frame_id = 'map'
        tfs.child_frame_id = 'vio_odom_local'
        tfs.transform.translation.x = 0.0
        tfs.transform.translation.y = 0.0
        tfs.transform.translation.z = 0.0
        tfs.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tfs)

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)

    def reset_cb(self, request, response):
        if self.last_position is None:
            response.success = False
            response.message = 'No odometry received yet; cannot reset.'
            return response

        self.reset_origin = self.last_position
        self.status = 'OK'
        self.position_window.clear()
        # Re-arm warmup and clear counters so the transient right after a
        # reset does not immediately re-flag divergence.
        self.sample_count = 0
        self.high_vel_count = 0
        self.healthy_count = 0
        self.peak_dist = 0.0

        ox, oy, oz = self.reset_origin
        self.log_info(f'RESET: new local origin set to x={ox:.3f} y={oy:.3f} z={oz:.3f}')

        # Publish a static transform so downstream nodes see the reset
        static_tfs = TransformStamped()
        static_tfs.header.stamp = self.get_clock().now().to_msg()
        static_tfs.header.frame_id = 'map'
        static_tfs.child_frame_id = 'odom'
        static_tfs.transform.translation.x = ox
        static_tfs.transform.translation.y = oy
        static_tfs.transform.translation.z = oz
        static_tfs.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tfs)

        response.success = True
        response.message = f'Local origin reset to x={ox:.3f} y={oy:.3f} z={oz:.3f}'
        return response

    def clear_log_cb(self, request, response):
        try:
            with open(self.diagnostics_path, 'w') as f:
                f.write('')
            response.success = True
            response.message = 'Diagnostics log cleared.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main():
    rclpy.init()
    node = VIOWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
