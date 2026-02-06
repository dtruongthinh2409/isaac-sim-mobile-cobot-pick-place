#!/usr/bin/env python3
"""
Relay /tf_static to /carter/tf_static with transient_local durability.
Nav2 in carter namespace subscribes to /carter/tf_static but static_transform_publisher
publishes to /tf_static (global). This relay bridges the gap.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRelay(Node):
    def __init__(self):
        super().__init__('tf_static_relay')

        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
        )

        self.sub = self.create_subscription(
            TFMessage, '/tf_static', self.callback, static_qos)
        self.pub = self.create_publisher(
            TFMessage, '/carter/tf_static', static_qos)

        self.get_logger().info('TF static relay: /tf_static -> /carter/tf_static')

    def callback(self, msg):
        self.pub.publish(msg)
        for t in msg.transforms:
            self.get_logger().info(
                f'Relayed: {t.header.frame_id} -> {t.child_frame_id}',
                once=True)


def main():
    rclpy.init()
    node = TfStaticRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
