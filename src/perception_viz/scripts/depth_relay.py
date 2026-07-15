#!/usr/bin/env python3
"""
Depth relay node.

Subscribes to /camera/left/depth_raw (from midas_v4l2_node)
and republishes to /midas/depth so midas_pointcloud_node can
consume the v4l2 pipeline without code changes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class DepthRelay(Node):
    def __init__(self):
        super().__init__('depth_relay')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            Image,
            '/camera/left/depth_raw',
            self.cb,
            sensor_qos,
        )

        self.pub = self.create_publisher(
            Image,
            '/midas/depth',
            sensor_qos,
        )

        self.get_logger().info(
            'DepthRelay: /camera/left/depth_raw -> /midas/depth'
        )

    def cb(self, msg: Image):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
