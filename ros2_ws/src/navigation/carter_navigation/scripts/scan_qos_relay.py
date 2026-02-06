#!/usr/bin/env python3
"""
Relay scan topic from BEST_EFFORT to RELIABLE QoS.
Fixes QoS mismatch between pointcloud_to_laserscan (BEST_EFFORT) and AMCL (RELIABLE).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanQoSRelay(Node):
    def __init__(self):
        super().__init__('scan_qos_relay')

        # Subscribe with BEST_EFFORT (matching pointcloud_to_laserscan output)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publish with RELIABLE (matching AMCL expectation)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan, 'scan', self.callback, best_effort_qos)
        self.pub = self.create_publisher(
            LaserScan, 'scan_reliable', reliable_qos)

        self.get_logger().info('Scan QoS relay started: scan (BEST_EFFORT) -> scan_reliable (RELIABLE)')

    def callback(self, msg):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanQoSRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
