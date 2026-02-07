#!/usr/bin/env python3
"""
Navigation Client — Test script to navigate to tables using Nav2 Simple Commander.

Usage (after Nav2 is running):
  ros2 run mobile_manipulation navigation_client.py --ros-args -p table:=1
  ros2 run mobile_manipulation navigation_client.py --ros-args -p table:=2
"""

import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        self.declare_parameter('table', 1)

        # Load config
        self._load_config()

        # Create navigator (separate node internally)
        self.navigator = BasicNavigator(namespace='carter')

    def _load_config(self):
        """Load table positions from .env file."""
        # Find project root (navigate up from installed script location)
        # Fallback: use environment variable or hardcoded path
        project_dir = os.environ.get(
            'PROJECT_DIR',
            os.path.expanduser('~/Documents/Thinh_Dang/isaac-sim-mobile-cobot-pick-place'))

        env_path = os.path.join(project_dir, 'configs', '.env')
        env = {}
        if os.path.exists(env_path):
            with open(env_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, value = line.split('=', 1)
                        try:
                            env[key.strip()] = float(value.strip())
                        except ValueError:
                            env[key.strip()] = value.strip()

        table1_x = env.get('TABLE_1_X', -4.0)
        table1_y = env.get('TABLE_1_Y', -3.0)
        table2_x = env.get('TABLE_2_X', -1.0)
        table2_y = env.get('TABLE_2_Y', -3.0)
        robot_dist = env.get('ROBOT_DISTANCE_FROM_TABLE', 0.5)

        # Positions in front of tables (robot facing -Y → quaternion z=-0.707, w=0.707)
        self.table_poses = {
            1: {'x': table1_x, 'y': table1_y + robot_dist,
                'qx': 0.0, 'qy': 0.0, 'qz': -0.707, 'qw': 0.707},
            2: {'x': table2_x, 'y': table2_y + robot_dist,
                'qx': 0.0, 'qy': 0.0, 'qz': -0.707, 'qw': 0.707},
        }

        self.get_logger().info(f'Table 1 goal: ({table1_x:.1f}, {table1_y + robot_dist:.1f})')
        self.get_logger().info(f'Table 2 goal: ({table2_x:.1f}, {table2_y + robot_dist:.1f})')

    def navigate_to_table(self, table_number):
        """Navigate to specified table. Returns True on success."""
        if table_number not in self.table_poses:
            self.get_logger().error(f'Invalid table number: {table_number}')
            return False

        pose_data = self.table_poses[table_number]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_data['x']
        goal_pose.pose.position.y = pose_data['y']
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = pose_data['qx']
        goal_pose.pose.orientation.y = pose_data['qy']
        goal_pose.pose.orientation.z = pose_data['qz']
        goal_pose.pose.orientation.w = pose_data['qw']

        self.get_logger().info(
            f'Navigating to Table {table_number} '
            f'({pose_data["x"]:.2f}, {pose_data["y"]:.2f})...')

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Log estimated time remaining every few seconds
                pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Reached Table {table_number}!')
            return True
        else:
            self.get_logger().error(f'Failed to reach Table {table_number}: {result}')
            return False


def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()

    table = client.get_parameter('table').get_parameter_value().integer_value

    client.get_logger().info('Waiting for Nav2 to be active...')
    client.navigator.waitUntilNav2Active()
    client.get_logger().info('Nav2 is active!')

    success = client.navigate_to_table(table)
    client.get_logger().info(f'Navigation result: {"SUCCESS" if success else "FAILED"}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
