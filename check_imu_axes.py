#!/usr/bin/env python3
"""Live IMU acceleration visualizer to find chip axes.

Run with the IMU node active:
  python3 check_imu_axes.py

Keep the robot still for 2 s while it collects a baseline, then:
  - push it forward and note which bar grows on the POSITIVE side
  - push it to the left and note which bar grows on the POSITIVE side
  - the axis that stays near +9.8 while still is UP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import sys


class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.cb, 50)
        self.samples = []
        self.baseline = None
        self.latest = np.zeros(3)
        self.create_timer(0.1, self.print_bars)
        print('Collecting 2-second baseline... hold the robot still.')
        sys.stdout.flush()

    def cb(self, msg):
        a = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        self.latest = a
        if self.baseline is None:
            self.samples.append(a)
            if len(self.samples) >= 200:  # ~2 s at ~100-130 Hz
                self.baseline = np.median(self.samples, axis=0)
                print(f'\nBaseline  x={self.baseline[0]:+.2f}  y={self.baseline[1]:+.2f}  z={self.baseline[2]:+.2f}')
                print('Now move the robot.  +++++ = positive spike, ----- = negative spike\n')
                sys.stdout.flush()

    def bar(self, val):
        width = 25
        scale = 2.0  # one # per 0.5 m/s^2
        n = int(round(abs(val) * scale))
        n = min(n, width)
        if val >= 0:
            return '+' * n + ' ' * (width - n)
        else:
            return ' ' * (width - n) + '-' * n

    def print_bars(self):
        if self.baseline is None:
            print(f'  collecting... {len(self.samples)} samples', end='\r', flush=True)
            return
        d = self.latest - self.baseline
        line = (
            f"x {self.bar(d[0])} {d[0]:+6.2f}  "
            f"y {self.bar(d[1])} {d[1]:+6.2f}  "
            f"z {self.bar(d[2])} {d[2]:+6.2f}"
        )
        print(line, end='\r', flush=True)


def main():
    rclpy.init()
    node = IMUVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
