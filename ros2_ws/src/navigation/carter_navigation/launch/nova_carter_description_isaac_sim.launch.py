## Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import *
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('calibrated_urdf_file', default='/etc/nova/calibration/isaac_calibration.urdf')

    return LaunchDescription([
        # Push all nodes into the 'carter' namespace
        PushRosNamespace('carter'),

        # Add robot description
        lu.add_robot_description(
            nominals_package='nova_carter_description',
            nominals_file='urdf/nova_carter.urdf.xacro',
            robot_calibration_path=args.calibrated_urdf_file,
        ),

    ])
