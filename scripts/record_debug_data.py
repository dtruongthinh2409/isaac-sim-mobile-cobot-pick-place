#!/usr/bin/env python3
"""
Record Debug Data — Ghi lại dữ liệu debug trong quá trình pick & place.

Chạy ĐỒNG THỜI với task_executor.py. Script này subscribe các ROS2 topics
và ghi lại dữ liệu liên tục vào file JSON trong thư mục records/.

Dữ liệu ghi lại:
- Vị trí Nova Carter (odometry): x, y, z, yaw
- Vị trí cánh tay Franka (joint states): 7 joint values
- Vị trí gripper (finger joints)
- TF frames: odom→base_link, chassis_link→panda_link0
- Object positions (từ config)
- Timestamps

Output: records/debug_YYYYMMDD_HHMMSS.json

Usage:
  Terminal riêng, chạy đồng thời với task_executor:
  python3 scripts/record_debug_data.py

  Hoặc:
  cd ros2_ws && source install/setup.bash
  python3 ../scripts/record_debug_data.py
"""

import os
import sys
import json
import time
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


# ==============================================================================
# CONFIG
# ==============================================================================

PROJECT_DIR = os.environ.get(
    'PROJECT_DIR',
    os.path.expanduser('~/Documents/Thinh_Dang/isaac-sim-mobile-cobot-pick-place'))

RECORDS_DIR = os.path.join(PROJECT_DIR, 'records')
RECORD_INTERVAL = 0.5  # Ghi mỗi 0.5 giây


class DebugRecorder(Node):
    def __init__(self):
        super().__init__('debug_recorder')

        # ----- State storage -----
        self.latest_odom = None
        self.latest_carter_joints = None
        self.latest_franka_joints = None
        self.latest_tf_odom_base = None
        self.latest_tf_chassis_panda = None
        self.records = []
        self.record_count = 0

        # ----- Load object positions -----
        obj_pos_path = os.path.join(PROJECT_DIR, 'configs', 'object_positions.json')
        if os.path.exists(obj_pos_path):
            with open(obj_pos_path, 'r') as f:
                self.object_positions = json.load(f)
        else:
            self.object_positions = {}

        # ----- Load .env config -----
        self.env_config = self._load_env()

        # ----- QoS profiles -----
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ----- Subscribers -----

        # Nova Carter odometry
        self.create_subscription(
            Odometry, '/carter/odom', self.odom_callback, best_effort_qos)

        # Carter joint states (wheels + all robot joints)
        self.create_subscription(
            JointState, '/carter/joint_states', self.carter_joints_callback, best_effort_qos)

        # Franka joint states (arm + gripper)
        self.create_subscription(
            JointState, '/franka/joint_states', self.franka_joints_callback, best_effort_qos)

        # TF (dynamic: odom → base_link)
        self.create_subscription(
            TFMessage, '/tf', self.tf_callback, reliable_qos)

        # TF static (chassis_link → panda_link0)
        self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, reliable_qos)

        # ----- Timer for periodic recording -----
        self.timer = self.create_timer(RECORD_INTERVAL, self.record_tick)

        # ----- Output file -----
        os.makedirs(RECORDS_DIR, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_file = os.path.join(RECORDS_DIR, f'debug_{timestamp}.json')

        self.get_logger().info('=' * 60)
        self.get_logger().info('DEBUG RECORDER STARTED')
        self.get_logger().info(f'Output: {self.output_file}')
        self.get_logger().info(f'Recording every {RECORD_INTERVAL}s')
        self.get_logger().info(f'Object positions: {list(self.object_positions.keys())}')
        self.get_logger().info('Press Ctrl+C to stop and save.')
        self.get_logger().info('=' * 60)

    def _load_env(self):
        env_path = os.path.join(PROJECT_DIR, 'configs', '.env')
        config = {}
        if os.path.exists(env_path):
            with open(env_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, value = line.split('=', 1)
                        try:
                            config[key.strip()] = float(value.strip())
                        except ValueError:
                            config[key.strip()] = value.strip()
        return config

    # ==================================================================
    # CALLBACKS
    # ==================================================================

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # Extract yaw from quaternion
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.latest_odom = {
            'position': {'x': round(p.x, 4), 'y': round(p.y, 4), 'z': round(p.z, 4)},
            'orientation': {'x': round(o.x, 4), 'y': round(o.y, 4),
                            'z': round(o.z, 4), 'w': round(o.w, 4)},
            'yaw_deg': round(math.degrees(yaw), 2),
            'frame': 'odom → base_link (odometry, relative to robot start position)'
        }

    def carter_joints_callback(self, msg):
        self.latest_carter_joints = {
            name: round(pos, 4)
            for name, pos in zip(msg.name, msg.position)
        }

    def franka_joints_callback(self, msg):
        joints = {}
        arm_joints = {}
        gripper_joints = {}
        for name, pos in zip(msg.name, msg.position):
            joints[name] = round(pos, 4)
            if name.startswith('panda_joint'):
                arm_joints[name] = round(pos, 4)
            elif name.startswith('panda_finger'):
                gripper_joints[name] = round(pos, 4)

        self.latest_franka_joints = {
            'all': joints,
            'arm': arm_joints,
            'gripper': gripper_joints,
            'gripper_open': all(v > 0.03 for v in gripper_joints.values()) if gripper_joints else None,
        }

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            t = transform.transform.translation
            r = transform.transform.rotation

            if parent == 'odom' and child == 'base_link':
                self.latest_tf_odom_base = {
                    'parent': parent, 'child': child,
                    'translation': {'x': round(t.x, 4), 'y': round(t.y, 4), 'z': round(t.z, 4)},
                    'rotation': {'x': round(r.x, 4), 'y': round(r.y, 4),
                                 'z': round(r.z, 4), 'w': round(r.w, 4)},
                }

    def tf_static_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            t = transform.transform.translation
            r = transform.transform.rotation

            if 'chassis' in parent and 'panda_link0' in child:
                self.latest_tf_chassis_panda = {
                    'parent': parent, 'child': child,
                    'translation': {'x': round(t.x, 4), 'y': round(t.y, 4), 'z': round(t.z, 4)},
                    'rotation': {'x': round(r.x, 4), 'y': round(r.y, 4),
                                 'z': round(r.z, 4), 'w': round(r.w, 4)},
                }

    # ==================================================================
    # RECORDING
    # ==================================================================

    def record_tick(self):
        """Called every RECORD_INTERVAL seconds."""
        self.record_count += 1

        # Compute derived values
        robot_world_pos = self._compute_robot_world_position()
        panda_world_pos = self._compute_panda_world_position(robot_world_pos)

        record = {
            'timestamp': time.time(),
            'record_number': self.record_count,

            # Raw data
            'carter_odom': self.latest_odom,
            'franka_joints': self.latest_franka_joints,
            'tf_odom_to_base_link': self.latest_tf_odom_base,
            'tf_chassis_to_panda_link0': self.latest_tf_chassis_panda,

            # Computed positions
            'robot_world_position': robot_world_pos,
            'panda_link0_world_position': panda_world_pos,

            # Static reference data
            'object_positions_world': self.object_positions,

            # Distance from panda_link0 to each object
            'distances_to_objects': self._compute_distances(panda_world_pos),
        }

        self.records.append(record)

        # Log summary every 10 records
        if self.record_count % 10 == 0:
            self._log_summary(record)

    def _compute_robot_world_position(self):
        """Compute robot position in WORLD frame from odometry.

        Odom frame starts at robot HOME position with HOME orientation.
        World position = HOME + R(HOME_yaw) * odom_position
        """
        if self.latest_odom is None:
            return None

        home_x = self.env_config.get('ROBOT_HOME_X', -2.5)
        home_y = self.env_config.get('ROBOT_HOME_Y', -1.5)
        home_yaw = self.env_config.get('ROBOT_HOME_YAW', -math.pi / 2)

        odom_x = self.latest_odom['position']['x']
        odom_y = self.latest_odom['position']['y']
        odom_z = self.latest_odom['position']['z']

        # Rotate odom position by HOME yaw to get world delta
        cos_yaw = math.cos(home_yaw)
        sin_yaw = math.sin(home_yaw)
        world_x = home_x + cos_yaw * odom_x - sin_yaw * odom_y
        world_y = home_y + sin_yaw * odom_x + cos_yaw * odom_y

        # Robot yaw in world = HOME_yaw + odom_yaw
        odom_yaw = math.radians(self.latest_odom['yaw_deg'])
        world_yaw = home_yaw + odom_yaw

        return {
            'x': round(world_x, 4),
            'y': round(world_y, 4),
            'z': round(odom_z, 4),
            'yaw_deg': round(math.degrees(world_yaw), 2),
            'note': 'Robot base_link position in Isaac Sim world frame'
        }

    def _compute_panda_world_position(self, robot_world_pos):
        """Compute panda_link0 position in world frame.

        panda_link0 is mounted on chassis_link at Z=0.4125m.
        chassis_link ≈ base_link (approximately same position).
        """
        if robot_world_pos is None:
            return None

        panda_z_offset = 0.4125
        return {
            'x': robot_world_pos['x'],
            'y': robot_world_pos['y'],
            'z': round(robot_world_pos['z'] + panda_z_offset, 4),
            'note': 'panda_link0 position in world frame (robot_pos + Z offset)'
        }

    def _compute_distances(self, panda_world_pos):
        """Compute 3D distance from panda_link0 to each object."""
        if panda_world_pos is None:
            return None

        distances = {}
        for name, pos in self.object_positions.items():
            dx = pos['x'] - panda_world_pos['x']
            dy = pos['y'] - panda_world_pos['y']
            dz = pos['z'] - panda_world_pos['z']
            dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            distances[name] = {
                'distance_3d': round(dist, 4),
                'delta_x': round(dx, 4),
                'delta_y': round(dy, 4),
                'delta_z': round(dz, 4),
                'within_reach': dist < 0.855,  # Panda max reach
            }
        return distances

    def _log_summary(self, record):
        """Log a brief summary to terminal."""
        robot = record.get('robot_world_position')
        if robot:
            self.get_logger().info(
                f'[#{self.record_count}] Robot world: '
                f'({robot["x"]:.3f}, {robot["y"]:.3f}) yaw={robot["yaw_deg"]:.1f}°')

        franka = record.get('franka_joints')
        if franka and franka.get('gripper'):
            gripper_state = 'OPEN' if franka.get('gripper_open') else 'CLOSED'
            self.get_logger().info(f'  Gripper: {gripper_state} {franka["gripper"]}')

        distances = record.get('distances_to_objects')
        if distances:
            for name, d in distances.items():
                reach = '✓' if d['within_reach'] else '✗'
                self.get_logger().info(
                    f'  → {name}: {d["distance_3d"]:.3f}m [{reach}] '
                    f'(dx={d["delta_x"]:.3f}, dy={d["delta_y"]:.3f}, dz={d["delta_z"]:.3f})')

    def save_records(self):
        """Save all records to JSON file."""
        output = {
            'metadata': {
                'total_records': len(self.records),
                'record_interval_sec': RECORD_INTERVAL,
                'start_time': self.records[0]['timestamp'] if self.records else None,
                'end_time': self.records[-1]['timestamp'] if self.records else None,
                'duration_sec': round(
                    self.records[-1]['timestamp'] - self.records[0]['timestamp'], 2
                ) if len(self.records) > 1 else 0,
            },
            'config': {
                'env': {k: v for k, v in self.env_config.items()
                        if isinstance(v, (int, float))},
                'object_positions': self.object_positions,
                'panda_z_offset': 0.4125,
                'panda_max_reach': 0.855,
            },
            'records': self.records,
        }

        with open(self.output_file, 'w') as f:
            json.dump(output, f, indent=2)

        self.get_logger().info(f'Saved {len(self.records)} records to {self.output_file}')


def main(args=None):
    rclpy.init(args=args)
    recorder = DebugRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Stopping recorder...')
    finally:
        recorder.save_records()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
