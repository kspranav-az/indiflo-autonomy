#!/usr/bin/env python3
"""Record stationary IMU data and compute noise statistics.

Run with the IMU node active and the robot/camera perfectly still:

    python3 record_imu_stats.py --duration 60

Outputs:
    - mean (bias estimate)
    - standard deviation
    - suggested OpenVINS noise_density values
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUStats(Node):
    def __init__(self, duration):
        super().__init__('imu_stats')
        self.duration = duration
        self.samples = []
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.cb, 200)
        self.create_timer(1.0, self.print_progress)
        self.start_time = self.get_clock().now()
        self.done = False
        print(f'Collecting {duration} seconds of stationary IMU data...')
        print('Keep the robot/camera completely still.')

    def cb(self, msg):
        if self.done:
            return
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            self.done = True
            self.compute_and_print()
            return
        self.samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

    def print_progress(self):
        if self.done:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        print(f'  elapsed: {elapsed:.1f}s / {self.duration}s  samples: {len(self.samples)}', end='\r', flush=True)

    def compute_and_print(self):
        n = len(self.samples)
        if n == 0:
            print('No samples received.')
            return

        # Compute means and stds
        sums = [0.0] * 6
        for s in self.samples:
            for i in range(6):
                sums[i] += s[i]
        means = [s / n for s in sums]

        var_sums = [0.0] * 6
        for s in self.samples:
            for i in range(6):
                var_sums[i] += (s[i] - means[i]) ** 2
        stds = [math.sqrt(v / n) for v in var_sums]

        # Estimate continuous-time noise density assuming effective bandwidth ~100 Hz
        bw = 100.0
        nd = [s / math.sqrt(bw) for s in stds]

        print('\n')
        print('=' * 60)
        print(f'Samples collected: {n}')
        print(f'Sample rate: {n / self.duration:.1f} Hz')
        print('\nAccelerometer (m/s^2):')
        print(f'  mean:  x={means[0]:+.4f}  y={means[1]:+.4f}  z={means[2]:+.4f}')
        print(f'  std:   x={stds[0]:.4f}    y={stds[1]:.4f}    z={stds[2]:.4f}')
        print(f'  suggested noise_density: {max(nd[0], nd[1], nd[2]):.5f}')
        print('\nGyroscope (rad/s):')
        print(f'  mean:  x={means[3]:+.4f}  y={means[4]:+.4f}  z={means[5]:+.4f}')
        print(f'  std:   x={stds[3]:.5f}   y={stds[4]:.5f}   z={stds[5]:.5f}')
        print(f'  suggested noise_density: {max(nd[3], nd[4], nd[5]):.5f}')
        print('\nSuggested imu_chain.yaml values:')
        print(f'  accelerometer_noise_density: {max(nd[0], nd[1], nd[2]):.5f}')
        print(f'  gyroscope_noise_density:     {max(nd[3], nd[4], nd[5]):.5f}')
        print('=' * 60)


def main():
    parser = argparse.ArgumentParser(description='Record stationary IMU stats')
    parser.add_argument('--duration', type=float, default=60.0, help='Recording duration in seconds')
    args = parser.parse_args()

    rclpy.init()
    node = IMUStats(args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
