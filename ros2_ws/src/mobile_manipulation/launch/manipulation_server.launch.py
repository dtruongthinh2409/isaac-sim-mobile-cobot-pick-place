"""
Launch file for the manipulation server node.
Loads MoveIt2 robot description so MoveGroupInterface can initialize.

Usage:
  ros2 launch mobile_manipulation manipulation_server.launch.py use_sim_time:=true

Requires: move_group node running (from isaac_moveit.launch.py)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    # Load MoveIt config (same as isaac_moveit.launch.py)
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "isaac"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    manipulation_server_node = Node(
        package='mobile_manipulation',
        executable='manipulation_server',
        name='manipulation_server',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time,
        manipulation_server_node,
    ])
