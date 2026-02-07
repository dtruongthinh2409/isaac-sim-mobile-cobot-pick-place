#!/usr/bin/env python3
"""
Test Arm Poses — Script để debug và tìm orientation đúng cho pick & place.

Test từng bước:
1. Đưa arm về ready → đọc current pose (lấy orientation thật)
2. Thử di chuyển arm đến vị trí object với orientation từ ready
3. Thử các orientation khác nhau
4. Test full pick sequence: approach → grasp → retreat

Usage:
  ros2 run mobile_manipulation test_arm_poses.py --ros-args -p test:=get_pose
  ros2 run mobile_manipulation test_arm_poses.py --ros-args -p test:=test_orientations
  ros2 run mobile_manipulation test_arm_poses.py --ros-args -p test:=test_pick

Requires: manipulation_server đang chạy
"""

import os
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from mobile_manipulation.srv import GoToNamedTarget, GoToPose


# Project directory
PROJECT_DIR = os.environ.get(
    'PROJECT_DIR',
    os.path.expanduser('~/Documents/Thinh_Dang/isaac-sim-mobile-cobot-pick-place'))


class ArmTester(Node):
    def __init__(self):
        super().__init__('test_arm_poses')

        self.declare_parameter('test', 'get_pose')

        # Service clients
        self.named_target_client = self.create_client(
            GoToNamedTarget, 'manipulation/go_to_named_target')
        self.pose_client = self.create_client(
            GoToPose, 'manipulation/go_to_pose')
        self.open_gripper_client = self.create_client(
            Trigger, 'manipulation/open_gripper')
        self.close_gripper_client = self.create_client(
            Trigger, 'manipulation/close_gripper')
        self.get_pose_client = self.create_client(
            Trigger, 'manipulation/get_current_pose')

    def wait_for_services(self):
        self.get_logger().info('Waiting for manipulation services...')
        self.named_target_client.wait_for_service()
        self.pose_client.wait_for_service()
        self.open_gripper_client.wait_for_service()
        self.close_gripper_client.wait_for_service()
        # get_current_pose is optional (needs C++ server update)
        if not self.get_pose_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('get_current_pose service not available (optional)')
        self.get_logger().info('All services available!')

    def call_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        return future.result()

    def go_to_named(self, name):
        self.get_logger().info(f'Arm → {name}')
        req = GoToNamedTarget.Request()
        req.target_name = name
        resp = self.call_sync(self.named_target_client, req)
        self.get_logger().info(f'  Result: {resp.success} — {resp.message}')
        return resp.success

    def go_to_pose(self, x, y, z, ox, oy, oz, ow, frame='panda_link0', cartesian=False):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(ox)
        pose.pose.orientation.y = float(oy)
        pose.pose.orientation.z = float(oz)
        pose.pose.orientation.w = float(ow)

        mode = 'cartesian' if cartesian else 'free-space'
        self.get_logger().info(
            f'Arm → [{x:.3f}, {y:.3f}, {z:.3f}] '
            f'orient=[{ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f}] '
            f'frame={frame} ({mode})')

        req = GoToPose.Request()
        req.target_pose = pose
        req.cartesian_path = cartesian
        resp = self.call_sync(self.pose_client, req)
        self.get_logger().info(f'  Result: {resp.success} — {resp.message}')
        return resp.success

    def get_current_pose(self):
        resp = self.call_sync(self.get_pose_client, Trigger.Request())
        self.get_logger().info(f'Current pose: {resp.message}')
        return resp.message

    def open_gripper(self):
        self.get_logger().info('Gripper → open')
        resp = self.call_sync(self.open_gripper_client, Trigger.Request())
        return resp.success

    def close_gripper(self):
        self.get_logger().info('Gripper → close')
        resp = self.call_sync(self.close_gripper_client, Trigger.Request())
        return resp.success

    # ==================================================================
    # TEST 1: Đọc orientation thật từ ready pose
    # ==================================================================
    def test_get_pose(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('TEST: Get current pose at different named targets')
        self.get_logger().info('=' * 60)

        for name in ['ready', 'stowed_back', 'transport', 'extended']:
            self.get_logger().info(f'\n--- {name} ---')
            if self.go_to_named(name):
                time.sleep(1.0)
                self.get_current_pose()

        self.get_logger().info('\n=== Done! Copy orientation from "ready" for grasp. ===')

    # ==================================================================
    # TEST 2: Thử các orientation khác nhau tại 1 vị trí cố định
    # ==================================================================
    def test_orientations(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('TEST: Try different orientations at a fixed position')
        self.get_logger().info('Vị trí: 0.4m trước, 0.0m bên, 0.2m trên panda_link0')
        self.get_logger().info('=' * 60)

        # Vị trí test: 0.4m trước arm, 0.2m trên arm base
        test_x, test_y, test_z = 0.4, 0.0, 0.2

        orientations = {
            '180° around X (current)': (1.0, 0.0, 0.0, 0.0),
            '180° around Y (Panda common)': (0.0, 1.0, 0.0, 0.0),
            '180° around X+Y (45°)': (0.707, 0.707, 0.0, 0.0),
            '135° around Y': (0.383, 0.924, 0.0, 0.0),
        }

        self.go_to_named('ready')
        time.sleep(1.0)

        for name, (ox, oy, oz, ow) in orientations.items():
            self.get_logger().info(f'\n--- Trying: {name} ---')
            self.get_logger().info(f'    Quaternion: ({ox}, {oy}, {oz}, {ow})')

            success = self.go_to_pose(test_x, test_y, test_z, ox, oy, oz, ow)

            if success:
                self.get_logger().info(f'  ✓ {name} — SUCCESS! Check gripper direction in Isaac Sim.')
                time.sleep(3.0)  # Pause to visually check
                self.go_to_named('ready')  # Return to ready
                time.sleep(1.0)
            else:
                self.get_logger().info(f'  ✗ {name} — FAILED (IK no solution)')

        self.get_logger().info('\n=== Done! Check which orientation made gripper point DOWN. ===')

    # ==================================================================
    # TEST 3: Test pick tại vị trí object thật
    # ==================================================================
    def test_pick(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('TEST: Pick sequence at first object position')
        self.get_logger().info('=' * 60)

        # Load object positions
        obj_pos_path = os.path.join(PROJECT_DIR, 'configs', 'object_positions.json')
        if not os.path.exists(obj_pos_path):
            self.get_logger().error(f'object_positions.json not found!')
            return

        with open(obj_pos_path, 'r') as f:
            positions = json.load(f)

        # Load .env for robot position
        env_path = os.path.join(PROJECT_DIR, 'configs', '.env')
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
                            pass

        # Robot at Table 2
        robot_x = env.get('TABLE_2_X', -1.0)
        robot_y = env.get('TABLE_2_Y', -3.0) + env.get('ROBOT_DISTANCE_FROM_TABLE', 0.5)
        robot_yaw = env.get('ROBOT_HOME_YAW', -1.57079)
        panda_z = 0.4125

        self.get_logger().info(f'Robot position: ({robot_x:.2f}, {robot_y:.2f}), yaw={robot_yaw:.3f}')

        # Pick first object
        first_obj = list(positions.keys())[0]
        obj = positions[first_obj]
        self.get_logger().info(f'Target object: {first_obj} at [{obj["x"]:.3f}, {obj["y"]:.3f}, {obj["z"]:.3f}]')

        # Convert odom → panda_link0
        dx = obj['x'] - robot_x
        dy = obj['y'] - robot_y
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        lx = cos_yaw * dx - sin_yaw * dy
        ly = sin_yaw * dx + cos_yaw * dy
        lz = obj['z'] - panda_z

        self.get_logger().info(f'In panda_link0 frame: [{lx:.3f}, {ly:.3f}, {lz:.3f}]')
        self.get_logger().info(f'Distance: {math.sqrt(lx**2 + ly**2 + lz**2):.3f}m (Panda max: 0.855m)')

        # Step 1: Ready
        self.go_to_named('ready')
        time.sleep(1.0)

        # Step 2: Get ready orientation (use this for grasp)
        self.get_logger().info('\nReading ready pose orientation...')
        pose_str = self.get_current_pose()

        # Step 3: Try approach with 180° around Y (Panda common)
        self.get_logger().info(f'\n--- Approaching {first_obj} with orient=(0, 1, 0, 0) ---')
        approach_z = lz + 0.12  # 12cm above
        success = self.go_to_pose(lx, ly, approach_z, 0.0, 1.0, 0.0, 0.0)

        if success:
            self.get_logger().info('Approach SUCCESS!')
            self.get_logger().info('Pausing 3s — check arm position in Isaac Sim...')
            time.sleep(3.0)

            # Step 4: Move down (Cartesian)
            self.get_logger().info(f'--- Moving down to grasp height ---')
            grasp_z = lz + 0.02
            self.go_to_pose(lx, ly, grasp_z, 0.0, 1.0, 0.0, 0.0, cartesian=True)

            time.sleep(2.0)

            # Step 5: Close gripper
            self.close_gripper()
            time.sleep(1.0)

            # Step 6: Retreat
            self.go_to_pose(lx, ly, approach_z, 0.0, 1.0, 0.0, 0.0, cartesian=True)
        else:
            self.get_logger().info('Approach FAILED! Try orient=(1, 0, 0, 0)')
            success = self.go_to_pose(lx, ly, approach_z, 1.0, 0.0, 0.0, 0.0)
            if success:
                self.get_logger().info('orient=(1, 0, 0, 0) worked!')

        # Return to ready
        self.go_to_named('ready')
        self.get_logger().info('\n=== Test complete! ===')


def main(args=None):
    rclpy.init(args=args)
    tester = ArmTester()
    tester.wait_for_services()

    test = tester.get_parameter('test').get_parameter_value().string_value

    if test == 'get_pose':
        tester.test_get_pose()
    elif test == 'test_orientations':
        tester.test_orientations()
    elif test == 'test_pick':
        tester.test_pick()
    else:
        tester.get_logger().error(f'Unknown test: {test}. Use: get_pose, test_orientations, test_pick')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
