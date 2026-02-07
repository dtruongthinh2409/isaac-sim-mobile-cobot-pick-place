#!/usr/bin/env python3
"""
Manipulation Client — Test script to call manipulation services.

Usage (after manipulation_server is running):
  ros2 run mobile_manipulation manipulation_client.py --ros-args -p test:=named -p target:=ready
  ros2 run mobile_manipulation manipulation_client.py --ros-args -p test:=open_gripper
  ros2 run mobile_manipulation manipulation_client.py --ros-args -p test:=close_gripper
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mobile_manipulation.srv import GoToNamedTarget, GoToPose
from geometry_msgs.msg import PoseStamped


class ManipulationClient(Node):
    def __init__(self):
        super().__init__('manipulation_client')

        # Service clients
        self.named_target_client = self.create_client(
            GoToNamedTarget, 'manipulation/go_to_named_target')
        self.pose_client = self.create_client(
            GoToPose, 'manipulation/go_to_pose')
        self.open_gripper_client = self.create_client(
            Trigger, 'manipulation/open_gripper')
        self.close_gripper_client = self.create_client(
            Trigger, 'manipulation/close_gripper')

        # Parameters for testing
        self.declare_parameter('test', 'named')
        self.declare_parameter('target', 'ready')

    def wait_for_services(self):
        """Wait for all manipulation services."""
        self.get_logger().info('Waiting for manipulation services...')
        self.named_target_client.wait_for_service()
        self.pose_client.wait_for_service()
        self.open_gripper_client.wait_for_service()
        self.close_gripper_client.wait_for_service()
        self.get_logger().info('All services available!')

    def call_sync(self, client, request):
        """Synchronous service call."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def go_to_named_target(self, name):
        """Move arm to named target (ready, stowed_back, transport, extended)."""
        req = GoToNamedTarget.Request()
        req.target_name = name
        self.get_logger().info(f'Calling go_to_named_target("{name}")...')
        resp = self.call_sync(self.named_target_client, req)
        self.get_logger().info(f'Result: success={resp.success}, message="{resp.message}"')
        return resp.success

    def go_to_pose(self, pose_stamped, cartesian=False):
        """Move arm to pose target."""
        req = GoToPose.Request()
        req.target_pose = pose_stamped
        req.cartesian_path = cartesian
        self.get_logger().info(
            f'Calling go_to_pose([{pose_stamped.pose.position.x:.3f}, '
            f'{pose_stamped.pose.position.y:.3f}, '
            f'{pose_stamped.pose.position.z:.3f}], '
            f'cartesian={cartesian})...')
        resp = self.call_sync(self.pose_client, req)
        self.get_logger().info(f'Result: success={resp.success}, message="{resp.message}"')
        return resp.success

    def open_gripper(self):
        """Open gripper."""
        self.get_logger().info('Calling open_gripper...')
        resp = self.call_sync(self.open_gripper_client, Trigger.Request())
        self.get_logger().info(f'Result: success={resp.success}, message="{resp.message}"')
        return resp.success

    def close_gripper(self):
        """Close gripper."""
        self.get_logger().info('Calling close_gripper...')
        resp = self.call_sync(self.close_gripper_client, Trigger.Request())
        self.get_logger().info(f'Result: success={resp.success}, message="{resp.message}"')
        return resp.success


def main(args=None):
    rclpy.init(args=args)
    client = ManipulationClient()
    client.wait_for_services()

    test = client.get_parameter('test').get_parameter_value().string_value
    target = client.get_parameter('target').get_parameter_value().string_value

    if test == 'named':
        client.go_to_named_target(target)
    elif test == 'open_gripper':
        client.open_gripper()
    elif test == 'close_gripper':
        client.close_gripper()
    elif test == 'pick_test':
        # Quick pick test: ready → open → close → stowed_back
        client.go_to_named_target('ready')
        client.open_gripper()
        client.close_gripper()
        client.go_to_named_target('stowed_back')
    else:
        client.get_logger().error(f'Unknown test: {test}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
