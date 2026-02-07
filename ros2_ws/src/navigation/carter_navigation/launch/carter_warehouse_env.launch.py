## Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.


import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def load_robot_config():
    """Load robot spawn config from input.json"""
    # Config path: from ros2_ws/install/carter_navigation -> ../../../../configs/input.json
    carter_nav_dir = get_package_share_directory("carter_navigation")
    # Go from install/carter_navigation/share/carter_navigation up to project root
    project_root = os.path.abspath(os.path.join(carter_nav_dir, "..", "..", "..", "..", ".."))
    config_path = os.path.join(project_root, "configs", "input.json")
    
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = json.load(f)
            print(f"[INFO] Loaded config from: {config_path}")
            return config
    else:
        print(f"[WARN] Config not found at {config_path}, using defaults")
        return {"start_table": 1, "target_table": 2}


def load_env_config(carter_nav_dir):
    """Load environment configuration from .env file"""
    # Go from install/carter_navigation up to project root
    project_root = os.path.abspath(os.path.join(carter_nav_dir, "..", "..", "..", "..", ".."))
    env_path = os.path.join(project_root, "configs", ".env")
    
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


def get_initial_pose_from_config(spawn_config, env_config):
    """Calculate AMCL initial pose based on robot HOME position from .env"""
    x = env_config.get('ROBOT_HOME_X', -2.5)
    y = env_config.get('ROBOT_HOME_Y', -1.5)
    yaw = env_config.get('ROBOT_HOME_YAW', -1.57079)
    
    return {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}


def generate_launch_description():
    # Get the launch and rviz directories
    carter_nav2_bringup_dir = get_package_share_directory("carter_navigation")
    
    # Load environment config from .env
    env_config = load_env_config(carter_nav2_bringup_dir)
    
    # Load robot spawn configuration from input.json
    robot_config = load_robot_config()
    
    # Calculate initial pose from both configs
    initial_pose = get_initial_pose_from_config(robot_config, env_config)
    
    print(f"=" * 60)
    print(f"Robot spawns at HOME position")
    print(f"AMCL initial pose: x={initial_pose['x']:.2f}, y={initial_pose['y']:.2f}, yaw={initial_pose['yaw']:.3f}")
    print(f"Start Table: {robot_config.get('start_table', 1)}, Target Table: {robot_config.get('target_table', 2)}")
    print(f"=" * 60)

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    rviz_config_dir = os.path.join(carter_nav2_bringup_dir, "rviz2", "carter_navigation_namespaced.rviz")

    # Names and poses of the robots
    robots = [{"name": "carter"}]

    # Common settings
    ENV_MAP_FILE = "warehouse_env.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_yaml_file = LaunchConfiguration("map")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(carter_nav2_bringup_dir, "maps", ENV_MAP_FILE),
        description="Full path to map file to load",
    )

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        "carter_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "carter_warehouse_env_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot1 launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the stacks"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_dir, description="Full path to the RVIZ config file to use."
    )

    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot["name"] + "_params_file")
        
        # Create params file with overridden initial pose
        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'amcl.ros__parameters.initial_pose.x': str(initial_pose['x']),
                'amcl.ros__parameters.initial_pose.y': str(initial_pose['y']),
                'amcl.ros__parameters.initial_pose.z': str(initial_pose['z']),
                'amcl.ros__parameters.initial_pose.yaw': str(initial_pose['yaw']),
            },
            convert_types=True
        )

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot["name"]),
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(carter_nav2_bringup_dir, "launch", "carter_navigation_individual.launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot["name"],
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": use_sim_time,
                        "params_file": configured_params,
                        "default_bt_xml_filename": default_bt_xml_filename,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                    }.items(),
                ),

                Node(
                    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                    remappings=[('cloud_in', ['point_cloud']),
                                ('scan', ['scan'])],
                    parameters=[{
                        'target_frame': 'front_3d_lidar',
                        'transform_tolerance': 0.01,
                        'min_height': -0.4,
                        'max_height': 1.5,
                        'angle_min': -1.5708,  # -M_PI/2
                        'angle_max': 1.5708,  # M_PI/2
                        'angle_increment': 0.0087,  # M_PI/360.0
                        'scan_time': 0.3333,
                        'range_min': 0.05,
                        'range_max': 100.0,
                        'use_inf': True,
                        'inf_epsilon': 1.0,
                    }],
                    name='pointcloud_to_laserscan',
                    namespace=robot["name"]
                ),

                # Scan QoS relay: BEST_EFFORT -> RELIABLE (fixes AMCL QoS mismatch)
                Node(
                    package='carter_navigation',
                    executable='scan_qos_relay.py',
                    name='scan_qos_relay',
                    namespace=robot["name"],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),

                # Relay /carter/clock to /clock (for use_sim_time)
                Node(
                    package='topic_tools',
                    executable='relay',
                    name='clock_relay',
                    arguments=[f'/{robot["name"]}/clock', '/clock'],
                    parameters=[{'use_sim_time': False}]  # Clock relay should NOT use sim time
                ),

                # Relay /carter/tf to /tf (for navigation)
                Node(
                    package='topic_tools',
                    executable='relay',
                    name='tf_relay',
                    arguments=[f'/{robot["name"]}/tf', '/tf'],
                    parameters=[{'use_sim_time': True}]
                ),

                # Static TF: base_link -> front_3d_lidar (publishes on /tf_static global)
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_lidar',
                    arguments=['-0.2317', '0', '0.526', '0', '0', '0', 'base_link', 'front_3d_lidar'],
                    parameters=[{'use_sim_time': True}],
                ),

                # Relay /tf_static -> /carter/tf_static (Nav2 in carter namespace needs this)
                Node(
                    package='carter_navigation',
                    executable='tf_static_relay.py',
                    name='tf_static_relay',
                    parameters=[{'use_sim_time': False}],
                    output='screen'
                ),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot["name"]]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " map yaml: ", map_yaml_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " params yaml: ", params_file]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " behavior tree xml: ", default_bt_xml_filename],
                ),
                LogInfo(
                    condition=IfCondition(log_settings), msg=[robot["name"], " rviz config file: ", rviz_config_file]
                ),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " autostart: ", autostart]),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld